# Quadruped Cat Robot

Robot cuadrúpedo de bajo costo inspirado en la locomoción felina. Cada pata
tiene 2 grados de libertad (cadera y rodilla), y el movimiento se calcula con
**cinemática inversa de base flotante**: dada la posición y orientación deseada
del cuerpo y la posición de cada pie en el suelo, se obtienen los ángulos de los
8 servos.

El proyecto tiene dos etapas:

| Etapa | Carpeta | Herramienta |
|---|---|---|
| **1. Simulación** | [`simulacion_matlab/`](simulacion_matlab) | MATLAB (live script) |
| **2. Implementación** | [`implementacion_esp32/`](implementacion_esp32) | ESP32 + Arduino IDE |

```text
 Simulación (MATLAB)                         Implementación (ESP32)
 ───────────────────                         ──────────────────────
 Modelo cinemático de base flotante   ──►    Misma IK portada a C++
 IK por pata en el plano X-Z                 8 servos (2 por pata)
 Visualización 3D y animación                Posturas y caminata por trote
 de movimientos del cuerpo                   Control remoto por Bluetooth
```

## Parámetros del robot

| Parámetro | Valor |
|---|---|
| Muslo (`l1`) / pierna (`l2`) | 4.0 cm / 6.3 cm |
| Cuerpo (largo × ancho) | 12 cm × 10 cm |
| Altura nominal del cuerpo | 7.5 cm |
| Actuadores | 8 servos (cadera + rodilla por pata) |

Patas: `ULL` delantera izquierda, `URL` delantera derecha, `LLL` trasera
izquierda, `LRL` trasera derecha.

## Cinemática inversa

Cada pata se resuelve como un brazo planar de 2 eslabones en el plano X-Z:

1. Con la pose del cuerpo `(x, y, z, roll, pitch, yaw)` se calcula dónde está
   la cadera de cada pata en el mundo: `p_cadera = R · p_cadera_cuerpo + p_base`.
2. El vector cadera → pie da el objetivo `(x, z)` de la pata.
3. Ley de cosenos para la rodilla y `atan2` para la cadera (rodilla hacia abajo):

```text
cos θ2 = (r² − l1² − l2²) / (2·l1·l2),   r² = x² + z²
θ1     = atan2(−z, −x) − atan2(l2·sin θ2, l1 + l2·cos θ2)
```

Un offset de 20° (π/9) en cada articulación ajusta el modelo a la posición de
montaje de los servos, y las patas derechas se espejan respecto a las izquierdas.

## Autor

MiTo Olórtegui Huamán — UTEC
