# Simulación en MATLAB

`cinematica_inversa_base_flotante.mlx` es un live script que modela el
cuadrúpedo, resuelve la cinemática inversa de base flotante y visualiza el
resultado.

## Secciones del script

| Sección | Qué hace |
|---|---|
| Parámetros + **Test** | Define eslabones, cuerpo y orígenes de las patas; resuelve la IK con el cuerpo a 7.5 cm de altura y los pies en el suelo |
| **Plot** | Cadena cinemática de cada pata con transformaciones homogéneas (`RotX`, `RotY`, `RotZ`, `Transform`) |
| **Plot 2** | Robot completo en 3D: cuerpo, ejes de orientación y las 4 patas; calcula los ángulos reales que van a los servos |
| **Animation** | Mueve el cuerpo con los pies fijos en el suelo, en 3 fases: subir y bajar (±1.5 cm), adelante y atrás (±2 cm) y cabeceo (±10°) |
| Funciones | `legIK` (IK planar de 2 eslabones), `ik_leg_floating` (IK con base flotante), rotaciones y transformación homogénea |

## Cómo usarlo

1. Abrir `cinematica_inversa_base_flotante.mlx` en MATLAB.
2. Ejecutar todo (**Run**) o sección por sección (**Run Section**).

Requiere `eul2rotm`, incluida en el **Robotics System Toolbox** (o en el
Navigation Toolbox).

Las mismas funciones (`legIK`, `ik_leg_floating`, mapeo a ángulos de servo) se
portaron a C++ en [`../implementacion_esp32`](../implementacion_esp32).
