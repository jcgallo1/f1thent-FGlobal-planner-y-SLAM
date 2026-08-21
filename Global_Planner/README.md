# Planificador global D* para F1TENTH

Este proyecto resuelve la Parte B del taller usando el mapa creado en la
Parte A (`F1tenth_Map.pgm` + `F1tenth_Map.yaml`). Genera una ruta discreta con
**D* (Dynamic A\*)**, la suaviza mediante una **B-Spline cubica** y exporta
waypoints navegables en coordenadas metricas del mapa ROS.

## Flujo de planificacion

1. Lee la imagen y los metadatos del YAML (resolucion y origen).
2. Considera ocupados tanto los muros como las zonas desconocidas.
3. Infla los obstaculos con el radio de seguridad del vehiculo.
4. Reduce la rejilla de forma conservadora, sin eliminar paredes delgadas.
5. Calcula la distancia de cada celda libre a la pared mas cercana.
6. Ejecuta D* penalizando las celdas proximas a paredes para mantener la ruta
   cerca del centro del carril.
7. Aproxima la ruta completa con una B-Spline cubica y verifica toda la curva.
8. Exporta unicamente waypoints con separacion maxima de 0.5 m. En curvas
   cerradas agrega puntos intermedios si hacen falta para evitar colisiones.

La implementacion principal esta en `f1tenth/f1tenth_map.py`. La implementacion
del algoritmo asignado esta en
`python_motion_planning/global_planner/graph_search/d_star.py`.

## Configuracion validada para `F1tenth_Map`

| Parametro | Valor predeterminado |
|---|---:|
| Resolucion original | 0.05 m/pixel |
| Factor de reduccion | 2 |
| Resolucion de planificacion | 0.10 m/celda |
| Radio de seguridad | 0.15 m |
| Peso de centrado D* | 20 |
| Separacion maxima | 0.5 m |
| Inicio `(x, y)` | `(0.69, -1.45)` m, debajo de la linea |
| Meta `(x, y)` | `(0.69, -0.25)` m, encima de la linea |

El rango aproximado del mapa es `x = [-3.76, 1.84] m` y
`y = [-8.50, 7.00] m`. Las coordenadas anteriores de `x=10` y `x=35` no
pertenecian al mapa y hacian que D* agotara su lista de busqueda.

## Instalacion

Desde la raiz del repositorio:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Ejecucion

La ejecucion predeterminada ya usa `F1tenth_Map.yaml` y D*:

```bash
source venv/bin/activate
python3 f1tenth/f1tenth_map.py
```

No se abre una ventana grafica por defecto, por lo que tambien funciona en un
equipo sin interfaz grafica. Para mostrar los resultados al terminar:

```bash
python3 f1tenth/f1tenth_map.py --show
```

Para observar el proceso paso a paso en una sola ventana interactiva:

```bash
python3 f1tenth/f1tenth_map.py --show --process
```

La animacion presenta primero las celdas exploradas por D*, luego reconstruye
la ruta cruda, dibuja progresivamente la B-Spline y finalmente muestra los
waypoints de 0.5 m. `--process` tambien abre la ventana aunque se omita
`--show`.

Inicio, meta, margen de seguridad y fuerza de centrado se pueden cambiar sin
editar el codigo:

```bash
python3 f1tenth/f1tenth_map.py \
  --start 0.69 -1.45 \
  --goal 0.69 -0.25 \
  --robot-radius 0.15 \
  --downsample 2 \
  --clearance-weight 20
```

Para ejecutar ademas RRT como comparacion opcional se puede agregar
`--include-rrt`. Este algoritmo no reemplaza al D* asignado.

## Resultados generados

Los archivos se guardan en `f1tenth/resultados_planificacion/`:

- `dstar_0_5m.csv` y `dstar_0_5m.png`
- `resumen.csv`

Los CSV contienen `x,y` en metros y en el mismo sistema de referencia definido
por el `origin` del YAML. Los PNG muestran la ruta discreta, la trayectoria
suavizada, inicio, meta y waypoints.

## Pruebas

```bash
source venv/bin/activate
MPLBACKEND=Agg python3 -m unittest discover -s tests -v
```

Las pruebas verifican el mapa seleccionado, la vuelta completa, el margen
minimo respecto de las paredes, la ausencia de colisiones en la B-Spline y en
los waypoints exportados, y el rechazo de coordenadas fuera del mapa.
