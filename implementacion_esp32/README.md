# Implementación en ESP32

`control_cuadrupedo_esp32/control_cuadrupedo_esp32.ino` es el firmware del
robot. Corre la cinemática inversa en el ESP32, mueve los 8 servos y recibe
comandos por Bluetooth.

## Hardware

- ESP32 con Bluetooth clásico (p. ej. ESP32-DevKitC / WROOM-32; **no** sirven
  ESP32-S3 ni C3, que solo tienen BLE)
- 8 servos de posición (500–2500 µs) con fuente externa

| Pata | Cadera (GPIO) | Rodilla (GPIO) |
|---|---|---|
| Delantera izquierda (ULL) | 18 | 19 |
| Delantera derecha (URL) | 21 | 22 |
| Trasera izquierda (LLL) | 33 | 25 |
| Trasera derecha (LRL) | 26 | 27 |

## Comandos por Bluetooth

El robot aparece como **`ESP32-Bluetooth`**. Desde cualquier app de terminal
Bluetooth serial se envía un byte:

| Byte | Acción |
|---|---|
| `1` | **Caminar**: trote por pares diagonales (delantera izquierda + trasera derecha, luego delantera derecha + trasera izquierda) |
| `3` | Pararse: IK con el cuerpo a 7.5 cm de altura |
| `4` | Echarse |
| `5` | Estirarse (postura predefinida, como un gato que se estira) |
| `6` | Sentarse |

El último comando recibido se repite en cada ciclo: con `1` el robot sigue
caminando hasta que llega otro comando. Las posturas se alcanzan interpolando
los ángulos durante ~1 s para que los movimientos sean suaves.

Al encender, el robot se coloca en la postura inicial calculada con la IK y
espera 5 s. El estado se imprime por el puerto serie a 115200 baudios.

## Compilar y cargar

1. Arduino IDE con el core **esp32** de Espressif (Boards Manager).
2. Instalar la librería **ESP32Servo** (Library Manager). `BluetoothSerial` ya
   viene con el core.
3. Abrir `control_cuadrupedo_esp32/control_cuadrupedo_esp32.ino`, elegir la
   placa (p. ej. *ESP32 Dev Module*) y cargar.

La cinemática (`legIK`, `ik_leg_floating`) es la misma que se validó en la
[simulación de MATLAB](../simulacion_matlab).
