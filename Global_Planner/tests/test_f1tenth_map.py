import math
import unittest
from pathlib import Path

import matplotlib.pyplot as plt

from f1tenth.f1tenth_map import (
    animate_planning_process,
    clearance_map_from_env,
    load_map,
    path_map_to_world,
    plan_routes,
    resample_path_safe,
    segment_is_free,
    smooth_path_bspline,
)
from python_motion_planning.utils import Grid, SearchFactory


ROOT = Path(__file__).resolve().parents[1]
MAP_YAML = ROOT / "Mapas-F1Tenth" / "F1tenth_Map.yaml"
START = (0.69, -1.45)
GOAL = (0.69, -0.25)


class F1TenthDStarTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.map_bin, cls.resolution, cls.origin = load_map(
            MAP_YAML, downsample_factor=2, robot_radius=0.15
        )
        cls.env, cls.routes = plan_routes(
            cls.map_bin, cls.resolution, cls.origin, START, GOAL
        )
        cls.raw_path = cls.routes["D*"][1]
        cls.smooth_path = smooth_path_bspline(cls.raw_path, cls.env)

    def test_selected_map_metadata(self):
        self.assertEqual(self.map_bin.shape, (155, 56))
        self.assertAlmostEqual(self.resolution, 0.10)
        self.assertEqual(tuple(self.origin), (-3.76, -8.5, 0.0))

    def test_dstar_path_reaches_goal_without_collisions(self):
        self.assertEqual(self.raw_path[0], (44, 70))
        self.assertEqual(self.raw_path[-1], (44, 82))
        # Para llegar al otro lado de la linea negra debe completar la vuelta:
        # bajar hasta la curva inferior y subir hasta la curva superior.
        self.assertLessEqual(min(y for _, y in self.raw_path), 18)
        self.assertGreaterEqual(max(y for _, y in self.raw_path), 134)
        clearance = clearance_map_from_env(self.env)
        self.assertGreaterEqual(min(
            clearance[y, x] for x, y in self.raw_path
        ), 3.0)
        self.assertTrue(all(
            segment_is_free(a, b, self.env)
            for a, b in zip(self.raw_path, self.raw_path[1:])
        ))

    def test_bspline_and_exported_waypoints_are_collision_free(self):
        self.assertGreater(len(self.smooth_path), len(self.raw_path))
        self.assertTrue(all(
            segment_is_free(a, b, self.env)
            for a, b in zip(self.smooth_path, self.smooth_path[1:])
        ))

        for spacing in (0.5,):
            waypoints = resample_path_safe(
                self.smooth_path, spacing, self.resolution, self.env
            )
            self.assertTrue(all(
                segment_is_free(a, b, self.env)
                for a, b in zip(waypoints, waypoints[1:])
            ))
            world = path_map_to_world(waypoints, self.resolution, self.origin)
            self.assertTrue(all(
                math.hypot(x2 - x1, y2 - y1) <= spacing + 1e-6
                for (x1, y1), (x2, y2) in zip(world, world[1:])
            ))
            self.assertAlmostEqual(world[0][0], START[0], places=6)
            self.assertAlmostEqual(world[0][1], START[1], places=6)
            self.assertAlmostEqual(world[-1][0], GOAL[0], places=6)
            self.assertAlmostEqual(world[-1][1], GOAL[1], places=6)

    def test_out_of_bounds_start_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "fuera del mapa"):
            plan_routes(
                self.map_bin, self.resolution, self.origin,
                (10.0, 5.0), GOAL,
            )

    def test_process_animation_can_draw_initial_and_final_states(self):
        waypoints = resample_path_safe(
            self.smooth_path, 0.5, self.resolution, self.env
        )
        figure, animation = animate_planning_process(
            self.map_bin,
            self.routes["D*"][3],
            self.raw_path,
            self.smooth_path,
            waypoints,
        )
        self.assertEqual(len(animation._func(0)), 5)
        self.assertEqual(len(animation._func(10**6)), 5)
        animation._draw_was_started = True
        plt.close(figure)

    def test_dstar_reports_when_no_route_exists(self):
        env = Grid(5, 5)
        env.update(env.obstacles | {(2, 1), (2, 2), (2, 3)})
        planner = SearchFactory()(
            "d_star", start=(1, 2), goal=(3, 2), env=env
        )
        with self.assertRaisesRegex(RuntimeError, "no encontro una ruta"):
            planner.plan()


if __name__ == "__main__":
    unittest.main()
