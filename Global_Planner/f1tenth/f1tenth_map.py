import argparse
import os
import cv2
import csv
import math
import time
from collections import deque
import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.interpolate import splprep, splev
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning.utils import Grid, SearchFactory
from pathlib import Path


def load_map(yaml_path, downsample_factor=1, robot_radius=0.0):
    """Carga un mapa ROS y devuelve una rejilla conservadora de ocupacion.

    Los pixeles desconocidos (gris 205 en los mapas guardados por SLAM) se
    consideran ocupados. El radio del robot se aplica antes de reducir el
    mapa y la reduccion conserva un obstaculo si aparece en cualquier pixel
    del bloque. De esta forma no desaparecen paredes delgadas.
    """
    yaml_path = Path(yaml_path)
    if downsample_factor < 1:
        raise ValueError("downsample_factor debe ser un entero mayor o igual a 1.")
    if robot_radius < 0:
        raise ValueError("robot_radius no puede ser negativo.")

    with yaml_path.open('r') as f:
        map_config = yaml.safe_load(f)

    img_path = Path(map_config['image'])
    if not img_path.is_absolute():
        img_path = (yaml_path.parent / img_path).resolve()
    map_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if map_img is None:
        raise FileNotFoundError(f"No se pudo leer la imagen del mapa: {img_path}")

    resolution = float(map_config['resolution'])
    origin = tuple(float(value) for value in map_config['origin'])
    negate = bool(map_config.get('negate', 0))

    # Los mapas de SLAM usan 254 para libre, 0 para ocupado y 205 para
    # desconocido. Solo se acepta como libre el extremo correspondiente.
    free = map_img <= 5 if negate else map_img >= 250
    map_bin = np.logical_not(free).astype(np.uint8)

    inflation_pixels = int(math.ceil(robot_radius / resolution))
    if inflation_pixels:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * inflation_pixels + 1, 2 * inflation_pixels + 1),
        )
        map_bin = cv2.dilate(map_bin, kernel, iterations=1)

    # Max-pooling: si un pixel del bloque esta ocupado, la celda queda
    # ocupada. Se rellena el borde para no perder las ultimas filas/columnas.
    h, w = map_bin.shape
    new_h = math.ceil(h / downsample_factor)
    new_w = math.ceil(w / downsample_factor)
    pad_h = new_h * downsample_factor - h
    pad_w = new_w * downsample_factor - w
    padded = np.pad(
        map_bin,
        ((0, pad_h), (0, pad_w)),
        mode="constant",
        constant_values=1,
    )
    map_bin = padded.reshape(
        new_h, downsample_factor, new_w, downsample_factor
    ).max(axis=(1, 3)).astype(np.uint8)

    resolution *= downsample_factor

    return map_bin, resolution, origin


def grid_from_map(map_bin):
    h, w = map_bin.shape
    env = Grid(w, h)

    # Grid crea los obstáculos del borde. Hay que conservarlos para impedir
    # que los planificadores intenten explorar fuera de [0, w) x [0, h).
    obstacles = set(env.obstacles)
    obstacles.update(
        (x, h - 1 - y)
        for y in range(h)
        for x in range(w)
        if map_bin[y, x] == 1
    )

    env.update(obstacles)
    return env


def clearance_map_from_env(env):
    """Distancia en celdas desde cada punto libre hasta la pared mas cercana."""
    free = np.ones((env.y_range, env.x_range), dtype=np.uint8)
    for x, y in env.obstacles:
        if 0 <= x < env.x_range and 0 <= y < env.y_range:
            free[y, x] = 0
    return cv2.distanceTransform(free, cv2.DIST_L2, 5)


def restrict_to_track(env, start):
    """
    Conserva como espacio transitable únicamente la región libre conectada
    con el inicio. Así, el exterior y el interior encerrado por las líneas
    negras de la pista no se consideran zonas válidas para planificar.
    """
    validate_free_cell(start, env, "inicio")

    reachable = {start}
    pending = deque([start])
    motions = ((1, 0), (-1, 0), (0, 1), (0, -1))

    while pending:
        x, y = pending.popleft()
        for dx, dy in motions:
            neighbor = (x + dx, y + dy)
            if (
                0 <= neighbor[0] < env.x_range
                and 0 <= neighbor[1] < env.y_range
                and neighbor not in env.obstacles
                and neighbor not in reachable
            ):
                reachable.add(neighbor)
                pending.append(neighbor)

    all_cells = {
        (x, y)
        for x in range(env.x_range)
        for y in range(env.y_range)
    }
    env.update(env.obstacles | (all_cells - reachable))
    return env


def world_to_map(x_world, y_world, resolution, origin):
    x_map = math.floor((x_world - origin[0]) / resolution)
    y_map = math.floor((y_world - origin[1]) / resolution)
    return (x_map, y_map)


def map_to_world(x_map, y_map, resolution, origin):
    """Devuelve el centro de una celda en el sistema de coordenadas ROS."""
    x_world = (x_map + 0.5) * resolution + origin[0]
    y_world = (y_map + 0.5) * resolution + origin[1]
    return (x_world, y_world)


def validate_free_cell(cell, env, label):
    x, y = cell
    if not (0 <= x < env.x_range and 0 <= y < env.y_range):
        raise ValueError(
            f"El punto de {label} {cell} esta fuera del mapa "
            f"({env.x_range} x {env.y_range} celdas)."
        )
    if cell in env.obstacles:
        raise ValueError(f"El punto de {label} {cell} esta ocupado.")


def save_path_as_csv(path, filename):
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["x", "y"])
        for x, y in path:
            writer.writerow([f"{x:.6f}", f"{y:.6f}"])


def path_length(path):
    return sum(
        math.hypot(x2 - x1, y2 - y1)
        for (x1, y1), (x2, y2) in zip(path, path[1:])
    )


def path_map_to_world(path, resolution, origin):
    return [map_to_world(x, y, resolution, origin) for x, y in path]


def resample_path(path, spacing):
    """Genera waypoints separados por distancia acumulada constante."""
    points = np.asarray(path, dtype=float)
    if len(points) < 2:
        return [tuple(point) for point in points]

    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total = cumulative[-1]
    if total == 0:
        return [tuple(points[0])]

    samples = np.arange(0.0, total, spacing)
    if len(samples) == 0 or not np.isclose(samples[-1], total):
        samples = np.append(samples, total)

    x = np.interp(samples, cumulative, points[:, 0])
    y = np.interp(samples, cumulative, points[:, 1])
    return list(zip(x, y))


def resample_path_safe(path, spacing_m, resolution, env):
    """Remuestrea con separacion maxima y evita atajos entre obstaculos.

    En curvas cerradas, unir dos muestras separadas exactamente ``spacing_m``
    puede cortar por dentro de la curva. Se elige de forma voraz el punto mas
    lejano visible dentro de esa distancia; por eso algunos waypoints quedan
    mas juntos, pero nunca mas separados ni en colision.
    """
    spacing_cells = spacing_m / resolution
    fine_spacing = min(0.25, spacing_cells / 10.0)
    candidates = resample_path(path, fine_spacing)
    if len(candidates) < 2:
        return candidates

    waypoints = [candidates[0]]
    index = 0
    while index < len(candidates) - 1:
        limit = index + 1
        while (
            limit + 1 < len(candidates)
            and path_length(candidates[index:limit + 2]) <= spacing_cells + 1e-9
        ):
            limit += 1

        next_index = None
        for candidate_index in range(limit, index, -1):
            if segment_is_free(candidates[index], candidates[candidate_index], env):
                next_index = candidate_index
                break
        if next_index is None:
            raise RuntimeError("No se pudo generar un tramo de waypoints sin colision.")

        waypoints.append(candidates[next_index])
        index = next_index

    return waypoints


def segment_is_free(point_a, point_b, env):
    """Comprueba que todo un segmento permanezca dentro del espacio libre."""
    x1, y1 = point_a
    x2, y2 = point_b
    distance = math.hypot(x2 - x1, y2 - y1)
    sample_count = max(2, int(math.ceil(distance * 4)) + 1)

    for t in np.linspace(0.0, 1.0, sample_count):
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        cell = (int(round(x)), int(round(y)))
        if not (0 <= cell[0] < env.x_range and 0 <= cell[1] < env.y_range):
            return False
        if cell in env.obstacles:
            return False
    return True


def shortcut_smooth(path, env, iterations=2500, seed=7):
    """Suavizado por atajos que conserva únicamente segmentos sin colisión."""
    smoothed = [tuple(point) for point in path]
    rng = np.random.default_rng(seed)

    for _ in range(iterations):
        if len(smoothed) <= 2:
            break
        i, j = sorted(rng.integers(0, len(smoothed), size=2))
        if j <= i + 1:
            continue
        if segment_is_free(smoothed[i], smoothed[j], env):
            smoothed[i + 1:j] = []
    return smoothed


def smooth_path_bspline(path, env, degree=3, samples_per_cell=4,
                        smoothing_per_point=0.075):
    """Aproxima la ruta con una B-Spline suave y libre de colisiones."""
    if len(path) < 3:
        return [tuple(point) for point in path]

    points = np.asarray(path, dtype=float)
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    if cumulative[-1] == 0:
        return [tuple(points[0])]
    parameters = cumulative / cumulative[-1]
    sample_count = max(
        100, int(math.ceil(cumulative[-1] * samples_per_cell))
    )
    max_degree = min(degree, len(points) - 1)
    target_smoothing = smoothing_per_point * len(points)

    # Si una aproximacion invade una pared, se reduce progresivamente el
    # suavizado hasta encontrar una B-Spline segura.
    smoothing_values = (
        target_smoothing,
        target_smoothing / 2.0,
        target_smoothing / 4.0,
        0.0,
    )
    for candidate_degree in range(max_degree, 1, -1):
        for smoothing in smoothing_values:
            try:
                spline, _ = splprep(
                    [points[:, 0], points[:, 1]],
                    u=parameters,
                    s=smoothing,
                    k=candidate_degree,
                )
                curve_array = np.asarray(
                    splev(np.linspace(0.0, 1.0, sample_count), spline)
                ).T
            except (TypeError, ValueError):
                continue

            curve_array[0], curve_array[-1] = points[0], points[-1]
            curve = [tuple(point) for point in curve_array]
            if all(
                segment_is_free(point_a, point_b, env)
                for point_a, point_b in zip(curve, curve[1:])
            ):
                return curve

    raise RuntimeError(
        "La B-Spline invade una celda ocupada. Aumenta --robot-radius, "
        "reduce --downsample o cambia inicio/meta."
    )


def plan_rrt_grid(start, goal, env, max_dist=6.0, sample_num=30000,
                  goal_sample_rate=0.12, seed=7):
    """Implementación de RRT compatible con el Grid del mapa F1TENTH."""
    if start in env.obstacles or goal in env.obstacles:
        raise ValueError("El inicio o la meta de RRT está en un obstáculo.")

    rng = np.random.default_rng(seed)
    nodes = [tuple(map(float, start))]
    parents = [-1]
    free_cells = np.asarray([
        (x, y)
        for x in range(1, env.x_range - 1)
        for y in range(1, env.y_range - 1)
        if (x, y) not in env.obstacles
    ], dtype=float)
    if len(free_cells) == 0:
        raise RuntimeError("No existen celdas transitables para ejecutar RRT.")

    for _ in range(sample_num):
        if rng.random() < goal_sample_rate:
            sample = np.asarray(goal, dtype=float)
        else:
            sample = free_cells[rng.integers(0, len(free_cells))]

        nodes_array = np.asarray(nodes)
        nearest_index = int(np.argmin(np.linalg.norm(nodes_array - sample, axis=1)))
        nearest = nodes_array[nearest_index]
        direction = sample - nearest
        distance = np.linalg.norm(direction)
        if distance == 0:
            continue

        new_node = nearest + direction / distance * min(max_dist, distance)
        if not segment_is_free(nearest, new_node, env):
            continue

        nodes.append(tuple(new_node))
        parents.append(nearest_index)
        new_index = len(nodes) - 1

        if (
            np.linalg.norm(new_node - np.asarray(goal)) <= max_dist
            and segment_is_free(new_node, goal, env)
        ):
            nodes.append(tuple(map(float, goal)))
            parents.append(new_index)

            path = []
            index = len(nodes) - 1
            while index != -1:
                path.append(nodes[index])
                index = parents[index]
            path.reverse()
            return path

    raise RuntimeError(
        "RRT no encontró una ruta. Aumenta sample_num o verifica que el "
        "obstáculo no cierre completamente el circuito."
    )


def plot_result(map_bin, raw_path, smooth_path, waypoints, algorithm,
                spacing, planning_time, resolution, output_path):
    """Crea y guarda uno de los cuatro gráficos solicitados."""
    fig, ax = plt.subplots(figsize=(7, 10))
    ax.imshow(
        map_bin,
        cmap="gray_r",
        origin="upper",
        extent=(0, map_bin.shape[1], 0, map_bin.shape[0]),
        interpolation="nearest",
    )

    raw = np.asarray(raw_path)
    smooth = np.asarray(smooth_path)
    waypoint_array = np.asarray(waypoints)
    ax.plot(raw[:, 0], raw[:, 1], "--", color="#ff9800", linewidth=1,
            label="Ruta original")
    ax.plot(smooth[:, 0], smooth[:, 1], color="#16a34a", linewidth=2,
            label="Ruta suavizada")
    ax.scatter(
        waypoint_array[:, 0], waypoint_array[:, 1],
        s=22, color="#2563eb", edgecolors="white", linewidths=0.5,
        label=f"Waypoints (max. {spacing:g} m)", zorder=4
    )
    ax.scatter(*raw_path[0], marker="s", s=65, color="red",
               label="Inicio", zorder=5)
    ax.scatter(*raw_path[-1], marker="s", s=65, color="#1155cc",
               label="Meta", zorder=5)

    ax.set_title(
        f"{algorithm} — separacion maxima {spacing:g} m\n"
        f"longitud suavizada: {path_length(smooth_path) * resolution:.2f} m"
        f" | planificación: {planning_time:.3f} s"
    )
    ax.set_xlabel("x [celda]")
    ax.set_ylabel("y [celda]")
    ax.set_xlim(0, map_bin.shape[1])
    ax.set_ylim(0, map_bin.shape[0])
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), borderaxespad=0)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    return fig


def animate_planning_process(map_bin, expanded, raw_path, smooth_path,
                             waypoints, spacing=0.5, interval_ms=30):
    """Muestra la exploracion D* y dibuja las trayectorias progresivamente."""
    expanded = np.asarray(expanded, dtype=float)
    raw = np.asarray(raw_path, dtype=float)
    smooth = np.asarray(smooth_path, dtype=float)
    waypoint_array = np.asarray(waypoints, dtype=float)
    empty = np.empty((0, 2))

    fig, ax = plt.subplots(figsize=(8, 10))
    ax.imshow(
        map_bin,
        cmap="gray_r",
        origin="upper",
        extent=(0, map_bin.shape[1], 0, map_bin.shape[0]),
        interpolation="nearest",
    )
    explored_plot = ax.scatter(
        [], [], s=8, color="#38bdf8", alpha=0.45,
        label="Celdas exploradas", zorder=2,
    )
    current_plot = ax.scatter(
        [], [], s=34, color="#a855f7", label="Expansion actual", zorder=4,
    )
    raw_line, = ax.plot(
        [], [], "--", color="#ff9800", linewidth=1.4,
        label="Ruta original", zorder=5,
    )
    smooth_line, = ax.plot(
        [], [], color="#16a34a", linewidth=2.5,
        label="Ruta suavizada", zorder=6,
    )
    waypoint_plot = ax.scatter(
        [], [], s=24, color="#2563eb", edgecolors="white", linewidths=0.5,
        label=f"Waypoints (max. {spacing:g} m)", zorder=7,
    )
    ax.scatter(*raw[0], marker="s", s=70, color="red", label="Inicio", zorder=8)
    ax.scatter(
        *raw[-1], marker="s", s=70, color="#1155cc", label="Meta", zorder=8
    )
    ax.set_xlabel("x [celda]")
    ax.set_ylabel("y [celda]")
    ax.set_xlim(0, map_bin.shape[1])
    ax.set_ylim(0, map_bin.shape[0])
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), borderaxespad=0)

    expansion_frames = min(120, max(1, len(expanded)))
    expansion_batch = max(1, math.ceil(len(expanded) / expansion_frames))
    expansion_frames = math.ceil(len(expanded) / expansion_batch)
    raw_batch = 4
    raw_frames = math.ceil(len(raw) / raw_batch)
    smooth_batch = max(1, math.ceil(len(smooth) / 60))
    smooth_frames = math.ceil(len(smooth) / smooth_batch)
    hold_frames = 25
    total_frames = expansion_frames + raw_frames + smooth_frames + hold_frames

    def update(frame):
        if frame < expansion_frames:
            end = min(len(expanded), (frame + 1) * expansion_batch)
            explored_plot.set_alpha(0.45)
            explored_plot.set_offsets(expanded[:end] if end else empty)
            current_plot.set_offsets(expanded[end - 1:end] if end else empty)
            ax.set_title(
                f"D* explorando el mapa — {end}/{len(expanded)} celdas"
            )
        elif frame < expansion_frames + raw_frames:
            explored_plot.set_alpha(0.18)
            explored_plot.set_offsets(expanded if len(expanded) else empty)
            current_plot.set_offsets(empty)
            route_frame = frame - expansion_frames
            end = min(len(raw), (route_frame + 1) * raw_batch)
            raw_line.set_data(raw[:end, 0], raw[:end, 1])
            ax.set_title("D* reconstruyendo la trayectoria original")
        elif frame < expansion_frames + raw_frames + smooth_frames:
            explored_plot.set_alpha(0.08)
            raw_line.set_data(raw[:, 0], raw[:, 1])
            smooth_frame = frame - expansion_frames - raw_frames
            end = min(len(smooth), (smooth_frame + 1) * smooth_batch)
            smooth_line.set_data(smooth[:end, 0], smooth[:end, 1])
            ax.set_title("Suavizando la trayectoria con B-Spline")
        else:
            explored_plot.set_offsets(empty)
            smooth_line.set_data(smooth[:, 0], smooth[:, 1])
            waypoint_plot.set_offsets(waypoint_array)
            ax.set_title(
                f"Planificacion terminada — {len(waypoints)} waypoints"
            )

        return (
            explored_plot, current_plot, raw_line, smooth_line, waypoint_plot
        )

    animation = FuncAnimation(
        fig,
        update,
        frames=total_frames,
        interval=interval_ms,
        repeat=False,
        blit=False,
        cache_frame_data=False,
    )
    fig.tight_layout()
    return fig, animation


def plan_routes(map_bin, resolution, origin, start_world, goal_world,
                include_rrt=False, clearance_weight=20.0):
    """Ejecuta D* (y opcionalmente RRT) y devuelve rutas seguras en rejilla."""
    env = grid_from_map(map_bin)
    start = world_to_map(*start_world, resolution, origin)
    goal = world_to_map(*goal_world, resolution, origin)

    validate_free_cell(start, env, "inicio")
    validate_free_cell(goal, env, "meta")
    restrict_to_track(env, start)
    validate_free_cell(goal, env, "meta (no conectada con el inicio)")
    clearance_map = clearance_map_from_env(env)

    print(f"Inicio mundo/mapa: {tuple(start_world)} -> {start}")
    print(f"Meta mundo/mapa:   {tuple(goal_world)} -> {goal}")
    print("Planificando con D*...")
    begin = time.perf_counter()
    dstar = SearchFactory()(
        "d_star",
        start=start,
        goal=goal,
        env=env,
        clearance_map=clearance_map,
        clearance_weight=clearance_weight,
    )
    _, dstar_path, _ = dstar.plan()
    expanded = [node.current for node in dstar.EXPAND]
    routes = {
        "D*": ("dstar", dstar_path, time.perf_counter() - begin, expanded)
    }

    if include_rrt:
        print("Planificando tambien con RRT...")
        begin = time.perf_counter()
        rrt_path = plan_rrt_grid(start, goal, env)
        routes["RRT"] = ("rrt", rrt_path, time.perf_counter() - begin, [])

    return env, routes


def parse_args():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Planificador global D* con suavizado B-Spline para F1TENTH."
    )
    parser.add_argument(
        "--map", type=Path,
        default=here.parent / "Mapas-F1Tenth" / "F1tenth_Map.yaml",
        help="Archivo YAML del mapa ROS.",
    )
    parser.add_argument(
        "--start", nargs=2, type=float, metavar=("X", "Y"),
        default=(0.69, -1.45), help="Inicio bajo la linea de salida [m].",
    )
    parser.add_argument(
        "--goal", nargs=2, type=float, metavar=("X", "Y"),
        default=(0.69, -0.25), help="Meta sobre la linea de llegada [m].",
    )
    parser.add_argument(
        "--downsample", type=int, default=2,
        help="Factor de reduccion de la rejilla (por defecto: 2).",
    )
    parser.add_argument(
        "--robot-radius", type=float, default=0.15,
        help="Radio de seguridad usado para inflar obstaculos [m].",
    )
    parser.add_argument(
        "--clearance-weight", type=float, default=20.0,
        help="Penalizacion D* por cercania a paredes (por defecto: 20).",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=here / "resultados_planificacion",
        help="Directorio para CSV, PNG y resumen.",
    )
    parser.add_argument(
        "--include-rrt", action="store_true",
        help="Ejecuta RRT como comparacion adicional; D* siempre se ejecuta.",
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Muestra las figuras; sin esta opcion solo se guardan.",
    )
    parser.add_argument(
        "--process", action="store_true",
        help="Anima la exploracion D*, la ruta cruda y el suavizado.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.clearance_weight < 0:
        raise ValueError("--clearance-weight no puede ser negativo.")

    map_bin, resolution, origin = load_map(
        args.map, args.downsample, args.robot_radius
    )
    x_max = origin[0] + map_bin.shape[1] * resolution
    y_max = origin[1] + map_bin.shape[0] * resolution
    print(
        f"Mapa: {args.map.name} | rejilla: {map_bin.shape[1]} x "
        f"{map_bin.shape[0]} | resolucion: {resolution:.3f} m/celda"
    )
    print(
        f"Rango aproximado: x=[{origin[0]:.2f}, {x_max:.2f}], "
        f"y=[{origin[1]:.2f}, {y_max:.2f}] m"
    )

    env, routes = plan_routes(
        map_bin, resolution, origin, args.start, args.goal,
        include_rrt=args.include_rrt,
        clearance_weight=args.clearance_weight,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    figures = []
    animations = []
    summary_rows = []
    spacing = 0.5

    for algorithm, (slug, raw_path, planning_time, expanded) in routes.items():
        smooth_path = smooth_path_bspline(raw_path, env)
        waypoints_map = resample_path_safe(
            smooth_path, spacing, resolution, env
        )
        waypoints_world = path_map_to_world(
            waypoints_map, resolution, origin
        )

        suffix = str(float(spacing)).replace(".", "_")
        base_name = f"{slug}_{suffix}m"
        csv_path = args.output_dir / f"{base_name}.csv"
        png_path = args.output_dir / f"{base_name}.png"
        save_path_as_csv(waypoints_world, csv_path)
        result_figure = plot_result(
            map_bin, raw_path, smooth_path, waypoints_map,
            f"{algorithm} centrado", spacing, planning_time,
            resolution, png_path
        )
        if args.process:
            plt.close(result_figure)
        else:
            figures.append(result_figure)
        if args.process and algorithm == "D*":
            process_figure, process_animation = animate_planning_process(
                map_bin, expanded, raw_path, smooth_path, waypoints_map, spacing
            )
            figures.append(process_figure)
            animations.append(process_animation)
        summary_rows.append([
            algorithm,
            spacing,
            planning_time,
            path_length(raw_path) * resolution,
            path_length(smooth_path) * resolution,
            len(waypoints_world),
        ])
        print(
            f"{algorithm} {spacing:g} m: {len(waypoints_world)} waypoints "
            f"-> {csv_path.name}, {png_path.name}"
        )

    with (args.output_dir / "resumen.csv").open("w", newline="") as summary_file:
        writer = csv.writer(summary_file, lineterminator="\n")
        writer.writerow([
            "algoritmo", "separacion_m", "tiempo_s",
            "longitud_original_m", "longitud_suavizada_m", "waypoints",
        ])
        writer.writerows(summary_rows)

    print(f"Resultados guardados en: {args.output_dir}")
    if args.show or args.process:
        if args.process:
            print("Mostrando el proceso D*. Cierra la ventana para terminar.")
        plt.show()
    else:
        for figure in figures:
            plt.close(figure)


if __name__ == "__main__":
    main()
