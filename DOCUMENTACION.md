# Sistema Controlador de Vuelo en Tiempo Real para Ala Volante (Alerones)
## Documentación técnica completa

**Materia:** Sistemas en Tiempo Real — Ingeniería Informática  
**Institución:** Centro Regional Universitario Córdoba — IUA  
**Grupo 5:** Rodeyro Maximo, Macedo Rodriguez Bautista, Garibaldi Bruno, Bertorello Massimo  
**Año:** 2026

---

## Tabla de contenidos

1. [Problema que resuelve el sistema](#1-problema-que-resuelve-el-sistema)
2. [Solución implementada](#2-solución-implementada)
3. [Hardware utilizado](#3-hardware-utilizado)
   - 3.1 [Microcontrolador ESP32](#31-microcontrolador-esp32)
   - 3.2 [Sensor inercial MPU6050](#32-sensor-inercial-mpu6050)
   - 3.3 [Servomotores SG90](#33-servomotores-sg90)
   - 3.4 [Sensor táctil TTP223](#34-sensor-táctil-ttp223)
4. [Software y sistema operativo](#4-software-y-sistema-operativo)
5. [Conexiones y mapeo de pines](#5-conexiones-y-mapeo-de-pines)
6. [Estructura del proyecto](#6-estructura-del-proyecto)
7. [Flujo completo de arranque](#7-flujo-completo-de-arranque)
8. [Subsistema I2C y driver del MPU6050](#8-subsistema-i2c-y-driver-del-mpu6050)
9. [Adquisición y procesamiento de datos](#9-adquisición-y-procesamiento-de-datos)
10. [Arquitectura de tareas FreeRTOS](#10-arquitectura-de-tareas-freertos)
11. [ISR del pulsador — punto cero](#11-isr-del-pulsador--punto-cero)
12. [Mecanismos de sincronización](#12-mecanismos-de-sincronización)
13. [Lógica de control y actuación](#13-lógica-de-control-y-actuación)
14. [Subsistema MCPWM — generación de PWM](#14-subsistema-mcpwm--generación-de-pwm)
15. [Tarea de monitoreo](#15-tarea-de-monitoreo)
16. [Sistema de construcción](#16-sistema-de-construcción)
17. [Cómo compilar y flashear](#17-cómo-compilar-y-flashear)
18. [Configuración del SDK (sdkconfig)](#18-configuración-del-sdk-sdkconfig)
19. [Parámetros ajustables](#19-parámetros-ajustables)
20. [Diagrama de flujo de datos](#20-diagrama-de-flujo-de-datos)

---

## 1. Problema que resuelve el sistema

Un avión tipo **ala volante** (flying wing), como el bombardero B-2 Spirit, carece de cola y de estabilizadores verticales. Esta geometría lo hace **inherentemente inestable**: cualquier perturbación externa —una ráfaga de viento, una vibración del motor, un movimiento brusco— provoca una inclinación en los ejes de alabeo (roll) o cabeceo (pitch) que, si no se corrige de inmediato, resulta en pérdida de control de la aeronave.

Un piloto humano no puede reaccionar con la velocidad y frecuencia necesarias para mantener el avión nivelado de forma continua. Además, el ruido mecánico generado por las vibraciones del motor contamina las mediciones del sensor de orientación, haciendo que las lecturas directas sean erráticas e inutilizables sin filtrado previo.

El sistema implementado resuelve ambos problemas:
- Lee la orientación del avión a **50 Hz** mediante un sensor inercial.
- Filtra el ruido mediante un **filtro complementario** que combina giroscopio y acelerómetro.
- Corrige la inclinación detectada moviendo **dos alerones** mediante servomotores, en tiempo real y sin intervención humana.
- Permite al operador **redefinir el punto de equilibrio** en cualquier momento tocando un sensor táctil.

---

## 2. Solución implementada

El sistema es un **controlador de estabilización activa de lazo cerrado** con la siguiente cadena de procesamiento:

```
MPU6050 (sensor) → I2C → Filtro complementario → Cola FreeRTOS → Control proporcional → MCPWM → Servos SG90
                                                       ↑
                                               ISR TTP223 (punto cero)
```

En términos de tiempo real, el sistema garantiza que entre la detección de una inclinación y la corrección mecánica haya un ciclo de **20 ms como máximo** (frecuencia de muestreo de 50 Hz). La ISR del pulsador tiene la máxima prioridad de hardware y se ejecuta en microsegundos.

---

## 3. Hardware utilizado

### 3.1 Microcontrolador ESP32

El ESP32 es el cerebro del sistema. Sus características relevantes para este proyecto son:

- **Arquitectura:** Xtensa LX6 dual-core a 240 MHz. Solo se usa un núcleo (el sistema no utiliza el segundo núcleo explícitamente, FreeRTOS puede distribuir tareas automáticamente).
- **RAM:** 520 KB SRAM. Suficiente para FreeRTOS, stacks de tareas y buffers I2C.
- **Flash:** 2 MB (el módulo específico de este proyecto). El firmware compilado ocupa ~186 KB, un 18% del total disponible.
- **Periféricos usados:**
  - Bus I2C (módulo I2C_NUM_0) para comunicarse con el MPU6050.
  - Módulo MCPWM (Motor Control PWM) para generar señales PWM de precisión hacia los servos.
  - Sistema de interrupciones GPIO para el sensor táctil.
  - UART0 para la salida serie (monitor/consola).
- **Identificación del chip:** ESP32-D0WD-V3, revisión v3.1, MAC b4:bf:e9:33:53:a4.
- **Alimentación:** 3.3V lógico. Se alimenta desde el puerto USB (5V regulados internamente).

### 3.2 Sensor inercial MPU6050

El MPU6050 es una **IMU** (Inertial Measurement Unit) de 6 ejes fabricada por InvenSense/TDK. Combina en un solo chip:

- **Acelerómetro de 3 ejes:** mide la aceleración en los ejes X, Y, Z en unidades de *g* (gravedad terrestre = 9.8 m/s²). En reposo sobre una superficie plana, el eje perpendicular al suelo registra 1g y los otros dos registran 0g. Con esto se puede calcular la inclinación estática usando trigonometría (atan2).
- **Giroscopio de 3 ejes:** mide la velocidad angular en los ejes X, Y, Z en grados por segundo (°/s). Permite calcular cuánto rotó el sensor integrando la velocidad en el tiempo.

**Por qué se necesitan ambos:** El acelerómetro solo detecta la orientación estática pero es muy sensible a vibraciones (cualquier aceleración no gravitacional contamina la lectura). El giroscopio integra la rotación con alta precisión a corto plazo pero acumula error (drift) con el tiempo. El **filtro complementario** combina ambos para obtener lo mejor de cada uno.

**Comunicación:** I2C a 400 kHz (modo fast). Dirección I2C: `0x68` (pin AD0 conectado a GND).

**Registros usados:**
| Registro | Dirección | Función |
|---|---|---|
| PWR_MGMT_1 | 0x6B | Control de energía. Escribir 0x00 despierta el sensor del modo sleep. |
| GYRO_CONFIG | 0x1B | Rango del giroscopio. Valor 0x00 = ±250 °/s. |
| ACCEL_CONFIG | 0x1C | Rango del acelerómetro. Valor 0x00 = ±2g. |
| ACCEL_XOUT_H | 0x3B | Primer byte de los 14 bytes de datos (accel + temp + gyro). |

**Formato de los datos:** El MPU6050 entrega datos en **big-endian**: el byte más significativo llega primero. Cada eje se representa como un entero con signo de 16 bits (int16_t), así que los 14 bytes contienen: ACC_X (2 bytes) + ACC_Y (2 bytes) + ACC_Z (2 bytes) + TEMP (2 bytes, ignorada) + GYRO_X (2 bytes) + GYRO_Y (2 bytes) + GYRO_Z (2 bytes). De estos, el firmware solo **guarda y usa** ACC_X/Y/Z, GYRO_X y GYRO_Y; **TEMP y GYRO_Z (guiñada) se leen pero se descartan** porque no intervienen en el control de un eje.

**Factores de escala (del datasheet):**
- Acelerómetro en rango ±2g: **16384 LSB/g** → dividir el raw por 16384.0 da el valor en *g*.
- Giroscopio en rango ±250 °/s: **131 LSB/°/s** → dividir el raw por 131.0 da la velocidad en °/s.

### 3.3 Servomotores SG90

El SG90 es un micro-servo de bajo costo y bajo torque, estándar en prototipos. Sus características:

- **Control:** Señal **PWM** (Pulse Width Modulation) con período de **20 ms** (50 Hz).
- **Protocolo de posicionamiento:** El ancho del pulso en alto dentro de cada período de 20 ms determina la posición del servo:
  - **1000 µs (1 ms):** posición mínima (aprox. −90°).
  - **1500 µs (1.5 ms):** posición neutra (0°).
  - **2000 µs (2 ms):** posición máxima (aprox. +90°).
- **Rango mecánico:** ~180° en total.
- **Alimentación:** 4.8–6V. Se alimenta directamente desde la línea de 5V del USB/placa.
- **Conexión:** 3 cables: GND (marrón/negro), VCC 5V (rojo), señal PWM (naranja/amarillo).

El sistema tiene **dos servos**:
- **Servo 1 (GPIO 16):** controla el alerón izquierdo.
- **Servo 2 (GPIO 17):** controla el alerón derecho.

En un ala volante, los alerones reciben el nombre de **elevones** porque combinan la función de alerones (control de alabeo) y elevadores (control de cabeceo).

### 3.4 Pulsador de "punto cero" (entrada de evento)

El sistema usa un **pulsador de un solo toque** para redefinir el punto de equilibrio. En el anteproyecto
este componente es un sensor táctil capacitivo **TTP223**, pero **el firmware lo trata eléctricamente como
un contacto momentáneo a GND** (un pulsador clásico), y así debe estar cableado o configurado el módulo.

- **Comportamiento eléctrico esperado por el código:** entrada digital de 1 bit, **activa en bajo**.
  En **reposo la línea está en ALTO** (la mantiene el pull-up interno del ESP32); al **accionar**, la
  entrada se lleva a **GND (BAJO)**.
- **Conexión a la ESP32:** la señal se conecta al **GPIO 19**, configurado con **resistencia pull-up
  interna** (`GPIO_PULLUP_ENABLE`). La interrupción se arma en **flanco de bajada** (`GPIO_INTR_NEGEDGE`):
  el evento que dispara la ISR es la transición **ALTO→BAJO** al accionar el pulsador.
- **Nota sobre el TTP223:** un TTP223 en configuración de fábrica es *activo en alto* (reposo BAJO, toque
  ALTO). Para funcionar con este firmware (activo en bajo, flanco de bajada) el módulo debe estar
  jumpereado/cableado en consecuencia, o bien usarse directamente un pulsador mecánico a GND. Conviene
  verificarlo contra el hardware real de la maqueta.
- **Función en el sistema:** al accionarlo con el avión en la posición deseada de equilibrio, el sistema
  **fija esa inclinación como el nuevo "cero"**, ajustando el punto de referencia de la estabilización
  sin necesidad de reiniciar.

---

## 4. Software y sistema operativo

### ESP-IDF (Espressif IoT Development Framework)

El proyecto se programa en **C** sobre **ESP-IDF versión 6.1** (instalado en `~/esp/esp-idf`). ESP-IDF es el framework oficial de Espressif para el ESP32 y provee:

- El sistema operativo en tiempo real **FreeRTOS** adaptado para ESP32 (dual-core).
- Drivers de hardware: I2C, GPIO, MCPWM, UART, timers, etc.
- Herramientas de construcción (CMake + Ninja) y flasheo (esptool).
- Sistema de configuración por menú (`idf.py menuconfig` / `sdkconfig`).

### FreeRTOS

FreeRTOS es un **sistema operativo de tiempo real** de código abierto diseñado para microcontroladores. En este proyecto se usa para:

- Crear **tareas** (threads) independientes con prioridades asignadas.
- **Comunicar** tareas mediante colas (queues), con semántica de bloqueo.
- **Proteger** datos compartidos mediante mutex (semáforos de exclusión mutua).
- Garantizar **periodicidad exacta** mediante `vTaskDelayUntil`.

FreeRTOS corre en este proyecto con un tick de **1000 Hz** (1 ms por tick), lo que permite temporización con resolución de 1 ms.

---

## 5. Conexiones y mapeo de pines

| Señal | GPIO ESP32 | Componente | Pin del componente | Descripción |
|---|---|---|---|---|
| SDA (I2C) | GPIO 21 | MPU6050 | SDA | Datos del bus I2C |
| SCL (I2C) | GPIO 22 | MPU6050 | SCL | Reloj del bus I2C |
| PWM Servo 1 | GPIO 16 | SG90 #1 | Señal (naranja) | Alerón izquierdo |
| PWM Servo 2 | GPIO 17 | SG90 #2 | Señal (naranja) | Alerón derecho |
| Pulsador | GPIO 19 | TTP223 | OUT | ISR flanco de bajada |
| GND | GND | Todos | GND | Tierra común |
| 3.3V | 3V3 | MPU6050 | VCC | Alimentación del sensor |
| 5V | VIN/5V | SG90 x2, TTP223 | VCC | Alimentación servos y táctil |

**Nota sobre el pull-up del pulsador:** GPIO 19 tiene activada la resistencia pull-up interna del ESP32 (`GPIO_PULLUP_ENABLE`). Esto significa que la línea está en ALTO en reposo (sin accionar). La interrupción se configura en flanco de bajada (falling edge), que ocurre en la transición de ALTO a BAJO al **accionar** el pulsador (contacto a GND). Ver la sección 3.4 sobre la naturaleza del pulsador.

**Nota sobre el bus I2C:** Las líneas SDA y SCL también tienen pull-up activado desde el software (`sda_pullup_en = GPIO_PULLUP_ENABLE`, `scl_pullup_en = GPIO_PULLUP_ENABLE`). En un diseño final se usarían resistencias pull-up externas de 4.7 kΩ, pero las internas del ESP32 son suficientes a 400 kHz para la longitud de cable de esta maqueta.

---

## 6. Estructura del proyecto

```
STR_ENTREGA/
├── CMakeLists.txt          # Archivo raíz de construcción ESP-IDF
├── sdkconfig               # Configuración activa del SDK (generada por idf.py)
├── sdkconfig.defaults      # Valores por defecto del proyecto
└── main/
    ├── CMakeLists.txt      # Registro del componente principal
    ├── config.h            # Todos los parámetros configurables del sistema
    ├── main.c              # Punto de entrada: app_main(), inicialización y lanzamiento
    ├── mpu6050.h           # Interfaz pública del driver I2C del MPU6050
    ├── mpu6050.c           # Implementación del driver (I2C + lectura de registros)
    ├── servos.h            # Interfaz pública del control de servos
    ├── servos.c            # Implementación MCPWM (timer→operator→comparator→generator)
    ├── tasks.h             # Interfaz pública de tareas e ISR, structs compartidas
    └── tasks.c             # Las 3 tareas FreeRTOS + ISR del pulsador
```

### `CMakeLists.txt` (raíz)

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ala_volante_rtos)
```

Este archivo es el punto de entrada del sistema de construcción CMake de ESP-IDF. La línea `include(...)` carga toda la maquinaria de ESP-IDF: búsqueda de componentes, toolchain para Xtensa, scripts de flasheo, etc. La variable de entorno `IDF_PATH` debe apuntar a la instalación de ESP-IDF (se activa con `source ~/esp/esp-idf/export.sh`).

### `main/CMakeLists.txt`

```cmake
idf_component_register(
    SRCS "main.c" "mpu6050.c" "servos.c" "tasks.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_gpio esp_driver_mcpwm esp_timer
)
```

Registra el **componente principal** de ESP-IDF. `idf_component_register` es la macro de ESP-IDF que declara qué archivos `.c` pertenecen a este componente y de qué otros componentes del SDK depende:

- `driver`: provee la API I2C legacy (`driver/i2c.h`).
- `esp_driver_gpio`: provee `driver/gpio.h` (GPIO y sistema de interrupciones). En ESP-IDF v5+, el driver de GPIO fue separado del componente `driver` en su propio componente; declararlo explícitamente evita el error de compilación.
- `esp_driver_mcpwm`: provee `driver/mcpwm_prelude.h` (control PWM de precisión).
- `esp_timer`: provee `esp_timer_get_time()` para el antirrebote de la ISR.

### `config.h`

Centraliza **todos** los parámetros del sistema en un único lugar. Si se necesita ajustar cualquier constante (pines, ganancias, frecuencias, prioridades), este es el único archivo a modificar.

**Sección: eje de control**

El sistema controla **un solo eje: el ROLL (alabeo)**, que es el eje natural de los alerones en un ala
volante. El **PITCH (cabeceo) se mide y se reporta** por consola, pero **no genera corrección mecánica**.
Esta elección es fija en el firmware (el actuador aplica la corrección de roll directamente).

**Sección: pines de hardware**
```c
#define SERVO1_GPIO   16
#define SERVO2_GPIO   17
#define BUTTON_GPIO   19
#define I2C_MASTER_SDA_GPIO  21
#define I2C_MASTER_SCL_GPIO  22
```

**Sección: bus I2C / MPU6050**
```c
#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_FREQ_HZ    400000
#define I2C_MASTER_TIMEOUT_MS 100
#define MPU6050_ADDR          0x68
```
`I2C_NUM_0` es el primer controlador I2C del ESP32 (hay dos). Velocidad de 400 kHz es el modo "fast" del estándar I2C. El timeout de 100 ms evita que el sistema se bloquee indefinidamente si el sensor no responde.

**Sección: servos**
```c
#define SERVO1_ZERO_US  1500
#define SERVO2_ZERO_US  1500
#define SERVO_MIN_US    1000
#define SERVO_MAX_US    2000
#define SERVO_PWM_FREQ_HZ  50
#define SERVO1_DIR  (+1.0f)
#define SERVO2_DIR  (+1.0f)
```
`SERVO1_ZERO_US` y `SERVO2_ZERO_US` son los pulsos de posición neutra. Si el servo no queda centrado físicamente al montar la maqueta, estos valores se ajustan (por ejemplo, 1480 o 1520 µs) para compensar sin desmontar el brazo mecánico. `SERVO1_DIR` y `SERVO2_DIR` permiten invertir el sentido de corrección de cada servo individualmente (`+1.0` = directo, `-1.0` = invertido) para compensar montajes físicos espejo sin tocar la lógica de control.

**Sección: filtro complementario**
```c
#define SENSOR_PERIOD_MS  20       // 50 Hz
#define SENSOR_DT_S  (SENSOR_PERIOD_MS / 1000.0f)  // 0.02 s
#define COMP_ALPHA  0.98f
```
El `dt` (delta time) es el intervalo entre muestras en segundos, usado para integrar el giroscopio. `COMP_ALPHA = 0.98` significa que el 98% del ángulo viene del giroscopio y el 2% viene del acelerómetro. Ver sección 9 para la explicación matemática completa.

**Sección: ISR / antirrebote**
```c
#define BUTTON_DEBOUNCE_US  300000   // 300 ms
```
El antirrebote evita que una sola pulsación genere múltiples eventos. Se implementa en la ISR: si el tiempo transcurrido desde el último flanco aceptado es menor a 300 ms, el nuevo evento se descarta.

**Sección: FreeRTOS**
```c
#define IMU_QUEUE_LENGTH         5
#define MONITOR_PERIOD_MS        200
#define ACTUATOR_QUEUE_TIMEOUT_MS 100
#define PRIO_SENSOR    5
#define PRIO_ACTUATOR  7
#define PRIO_MONITOR   3
#define STACK_SENSOR   4096
#define STACK_ACTUATOR 4096
#define STACK_MONITOR  3072
```
La cola tiene capacidad para 5 muestras. En condiciones normales, el actuador consume una muestra cada vez que el sensor produce una, así que la cola está casi siempre en 0 o 1 elementos. Los 5 slots son un buffer de seguridad. Las prioridades siguen el principio de que la tarea más crítica en tiempo (actuador) tiene mayor prioridad que la productora (sensor), y el monitor tiene la menor porque puede retrasarse sin consecuencias para el control. El stack en palabras (4096 palabras = 16 KB en arquitecturas de 32 bits) es suficiente para las variables locales y el contexto de las llamadas a funciones matemáticas (`atan2f`, `sqrtf`).

### `main.c`

Contiene la función `app_main()`, que es el equivalente al `main()` de C estándar en ESP-IDF. FreeRTOS ya está corriendo cuando `app_main` se ejecuta (es en sí misma una tarea de FreeRTOS de alta prioridad). Su cuerpo son los 5 pasos de arranque (ver sección 7) y nada más: es un archivo corto de ~40 líneas.

---

## 7. Flujo completo de arranque

La función `app_main()` ejecuta los pasos de inicialización en este orden, y cada uno debe completarse sin error antes de continuar:

```
1. mpu6050_i2c_init()   → Configura el hardware I2C del ESP32 y lo activa.
2. mpu6050_init()       → Despierta el MPU6050 y fija sus rangos de medida.
3. servos_init()        → Configura MCPWM, lleva ambos servos a posición neutra.
4. tasks_sync_init()    → Crea la cola, el mutex, configura GPIO 19 e instala la ISR.
5. tasks_start()        → Crea las 3 tareas FreeRTOS y cede el control al scheduler.
```

El uso de `ESP_ERROR_CHECK()` en cada paso garantiza que si cualquier inicialización falla, el sistema entra en pánico controlado con un mensaje de error claro en la consola (en lugar de continuar con hardware mal configurado).

Después de `tasks_start()`, `app_main()` retorna. En ESP-IDF, cuando `app_main` retorna, su tarea se elimina automáticamente y el scheduler de FreeRTOS sigue corriendo las tareas creadas.

---

## 8. Subsistema I2C y driver del MPU6050

### Protocolo I2C

I2C (Inter-Integrated Circuit) es un protocolo de comunicación serie **síncrono** que usa dos líneas:
- **SDA** (Serial Data): transporta los datos.
- **SCL** (Serial Clock): marca el ritmo de la transferencia (generado siempre por el maestro).

El ESP32 actúa como **maestro** (master) y el MPU6050 como **esclavo** (slave). Cada transacción I2C sigue este patrón:

**Escritura de un registro:**
```
START → [addr(7 bits) + W(1 bit)] + ACK → [reg(8 bits)] + ACK → [data(8 bits)] + ACK → STOP
```

**Lectura de registros (repeated start):**
```
START → [addr + W] + ACK → [reg] + ACK → RESTART → [addr + R] + ACK → [byte1] ACK → ... → [byteN] NACK → STOP
```
El NACK en el último byte le indica al esclavo que el maestro ya no quiere más datos.

### Inicialización del bus (`mpu6050_i2c_init`)

```c
i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_GPIO,   // GPIO 21
    .scl_io_num = I2C_MASTER_SCL_GPIO,   // GPIO 22
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,  // 400000 Hz
};
i2c_param_config(I2C_MASTER_NUM, &conf);
i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
```

`i2c_param_config` configura los parámetros lógicos (pines, velocidad). `i2c_driver_install` instala el driver en el sistema operativo y activa el periférico hardware. Los últimos tres ceros son: tamaño del buffer de recepción en modo esclavo (0 porque somos maestro), tamaño del buffer de transmisión en modo esclavo (0), y flags de instalación (0 = sin opciones especiales).

### Inicialización del MPU6050 (`mpu6050_init`)

Se realizan tres escrituras de registro:

1. `PWR_MGMT_1 ← 0x00`: Por defecto, el MPU6050 arranca en **modo sleep** para ahorrar energía. Escribir 0x00 en este registro lo despierta y selecciona el oscilador interno de 8 MHz como fuente de reloj.

2. `ACCEL_CONFIG ← 0x00`: Los bits 4:3 de este registro seleccionan el rango del acelerómetro. `0x00` = bits 4:3 = `00` = rango ±2g. Este es el rango más sensible (mayor resolución), adecuado porque el avión no experimentará más de 2g en maniobras normales.

3. `GYRO_CONFIG ← 0x00`: Los bits 4:3 de este registro seleccionan el rango del giroscopio. `0x00` = ±250 °/s. Un ala volante no debería rotar más de ~200 °/s en situaciones normales de vuelo, así que este rango es apropiado.

### Lectura de una muestra (`mpu6050_read`)

```c
uint8_t raw[14];
mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT, raw, 14);
```

Se leen 14 bytes consecutivos a partir del registro `0x3B` (ACCEL_XOUT_H). El MPU6050 incrementa automáticamente el puntero de registro, así que una sola transacción I2C de lectura múltiple obtiene todos los datos del acelerómetro, temperatura y giroscopio.

**Reconstrucción de los valores de 16 bits:**
```c
int16_t acc_x = (int16_t)((raw[0] << 8) | raw[1]);
```
El byte de mayor peso (`raw[0]`) se desplaza 8 bits a la izquierda y se combina con el byte de menor peso (`raw[1]`) mediante OR. El cast a `int16_t` es esencial: sin él, el valor se interpretaría como `uint16_t` y los valores negativos serían incorrectos. Los raw values son complemento a dos.

**Escala a unidades físicas:**
```c
out->acc_x  = acc_x  / ACCEL_SCALE_2G;      // raw / 16384.0 → g
out->gyro_x = gyr_x  / GYRO_SCALE_250DPS;   // raw / 131.0   → °/s
```

---

## 9. Adquisición y procesamiento de datos

### El problema de la inclinación

Para conocer la inclinación del avión, se necesita calcular el ángulo que forma el eje del sensor con respecto a la gravedad. Hay dos formas de hacerlo, cada una con sus limitaciones:

**Ángulo por acelerómetro:**
En reposo, el único vector de aceleración es la gravedad (1g apuntando hacia abajo). Usando trigonometría inversa se puede deducir la inclinación:

```
accel_roll  = atan2(acc_y, acc_z)                          * (180/π)
accel_pitch = atan2(-acc_x, sqrt(acc_y² + acc_z²))        * (180/π)
```

`atan2` devuelve el arco tangente de dos argumentos en el rango [-π, π], y se multiplica por `RAD_TO_DEG = 57.2957795` para convertir a grados. Esta medida es estable a largo plazo (no tiene drift) pero **muy ruidosa** bajo vibraciones: cualquier aceleración no gravitacional (por ejemplo, la vibración del motor) suma al vector y corrompe el cálculo.

**Ángulo por giroscopio:**
Integrando la velocidad angular en el tiempo se obtiene el ángulo acumulado:

```
angle_gyro += gyro_rate * dt
```

Esta integración es precisa a corto plazo y no se ve afectada por vibraciones. Sin embargo, acumula **error de integración (drift)**: pequeños errores en la medición de velocidad se suman con cada muestra, desviando el ángulo estimado del ángulo real con el tiempo.

### Filtro complementario

El filtro complementario combina ambas fuentes de manera que se compensan mutuamente:

```
angle = α × (angle + gyro_rate × dt) + (1 − α) × accel_angle
```

Con `α = 0.98` (`COMP_ALPHA`):
- El **98%** del ángulo viene de la integración del giroscopio, que es suave y precisa en el corto plazo.
- El **2%** viene del acelerómetro, que "ancla" el ángulo a la referencia real de gravedad, previniendo el drift acumulado.

El resultado es un ángulo que:
- Responde rápido a rotaciones (gracias al giroscopio).
- No deriva con el tiempo (corregido por el acelerómetro).
- Es mucho menos sensible a vibraciones que el acelerómetro puro.

Este filtro es la alternativa simple y eficiente al más preciso (pero computacionalmente costoso) filtro de Kalman.

**Implementación en código:**
```c
roll  = COMP_ALPHA * (roll  + r.gyro_x * SENSOR_DT_S) + (1.0f - COMP_ALPHA) * accel_roll;
pitch = COMP_ALPHA * (pitch + r.gyro_y * SENSOR_DT_S) + (1.0f - COMP_ALPHA) * accel_pitch;
```

`r.gyro_x` es la velocidad angular en torno al eje X (que corresponde a roll) en °/s. Multiplicado por `SENSOR_DT_S = 0.02 s` da el incremento angular de esta muestra en grados. Sumado al ángulo anterior y ponderado con `α`, se obtiene el nuevo ángulo filtrado.

---

## 10. Arquitectura de tareas FreeRTOS

El sistema tiene tres tareas concurrentes y una rutina de interrupción (ISR). Cada tarea corre en un loop infinito (`for(;;)`) con su propio stack y prioridad.

```
Prioridad 7 (ALTA)   → task_actuator  ←─── imu_queue ←──── task_sensor (prio 5)
Prioridad 5 (MEDIA)  → task_sensor    ──────────────────────────────────────────
Prioridad 3 (BAJA)   → task_monitor   ←─── last_roll/pitch (spinlock)
ISR (hardware)       → gpio_isr_handler → zero_reset_requested (spinlock)
```

### task_sensor — Tarea productora (prioridad 5, 50 Hz)

**Responsabilidad:** Leer el MPU6050 cada 20 ms, aplicar el filtro complementario, verificar si la ISR pidió recalibración, y publicar el resultado en la cola.

**Periodicidad exacta con `vTaskDelayUntil`:**
```c
TickType_t last_wake = xTaskGetTickCount();
for (;;) {
    // ... trabajo ...
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
}
```
A diferencia de `vTaskDelay` (que espera un tiempo relativo desde que se llama), `vTaskDelayUntil` espera hasta un tiempo absoluto calculado desde la última activación. Esto garantiza que la frecuencia de muestreo sea exactamente 50 Hz incluso si la lectura I2C tarda un tiempo variable. Si el cuerpo de la tarea tarda más de 20 ms (lo que no debería ocurrir), `vTaskDelayUntil` retorna inmediatamente y la tarea ejecuta la siguiente muestra sin espera, recuperando el ritmo.

**Publicación en la cola:**
```c
imu_data_t data = { .roll = roll, .pitch = pitch };
xQueueSend(imu_queue, &data, 0);
```
El último argumento `0` es el timeout de espera: si la cola está llena, se descarta la muestra y se emite un warning. No se espera porque la tarea tiene que seguir su ritmo de 50 Hz.

### task_actuator — Tarea consumidora (prioridad 7, guiada por la cola)

**Responsabilidad:** Recibir ángulos de la cola, calcular el error respecto al punto cero, aplicar la ganancia proporcional, limitar el resultado al rango físico de los servos, y mover los servos.

**Espera en la cola:**
```c
xQueueReceive(imu_queue, &data, pdMS_TO_TICKS(ACTUATOR_QUEUE_TIMEOUT_MS))
```
Esta llamada **bloquea** la tarea hasta que haya un elemento en la cola o hasta que pasen 100 ms (timeout). Si hay datos disponibles retorna `pdTRUE`; si se agotó el timeout retorna `pdFALSE` y se emite un warning. Este comportamiento es central a la arquitectura de tiempo real: la tarea de actuación no "hace polling" ni usa `vTaskDelay`; se despierta exactamente cuando hay datos nuevos.

**Alta prioridad del actuador:** `PRIO_ACTUATOR = 7` > `PRIO_SENSOR = 5`. Esto garantiza que cuando el sensor deposita datos en la cola, el actuador es desbloqueado y, si el scheduler decide conmutar (preemption), el actuador tiene prioridad sobre el sensor para ejecutar. La corrección mecánica ocurre lo antes posible tras cada medición.

### task_monitor — Tarea de monitoreo (prioridad 3, 200 ms)

**Responsabilidad:** Imprimir el estado del sistema por el puerto serie cada 200 ms para depuración y monitoreo en tiempo real.

Imprime: ángulo de roll actual, ángulo de pitch actual, punto cero configurado (roll y pitch), y pulso en µs de cada servo. Al ser la tarea de menor prioridad, se ejecuta solo cuando el sensor y el actuador no están activos, sin interferir con el control.

---

## 11. ISR del pulsador — punto cero

### Qué es una ISR

Una ISR (Interrupt Service Routine, Rutina de Servicio de Interrupción) es una función que el hardware invoca automáticamente cuando ocurre un evento específico, interrumpiendo cualquier código que estuviera ejecutando el procesador. Las ISRs deben ser **extremadamente cortas** porque:
- Bloquean la ejecución de otras ISRs de igual o menor prioridad mientras corren.
- No pueden llamar a la mayoría de las funciones del sistema operativo (las que puedan bloquear).
- No pueden tomar semáforos ni mutex normales (solo variantes `FromISR`).

### `IRAM_ATTR`

```c
static void IRAM_ATTR gpio_isr_handler(void *arg)
```

Este atributo instruye al compilador a colocar la función en la **IRAM** (Internal RAM) del ESP32, en lugar de en la flash. Normalmente, el código se ejecuta desde flash mediante una caché de instrucciones. Si la caché tiene un fallo (cache miss), el procesador debe esperar para cargar el código desde flash, lo que puede tardar cientos de nanosegundos. Al estar en IRAM, la ISR siempre está disponible con latencia mínima y determinística, sin depender de la caché.

### Antirrebote por software

Los sensores mecánicos (y algunos electrónicos) pueden generar múltiples flancos eléctricos en una sola pulsación (efecto de rebote). Para ignorar flancos múltiples generados por una sola pulsación:

```c
int64_t now = esp_timer_get_time();    // tiempo actual en microsegundos
if ((now - last_isr_time_us) >= BUTTON_DEBOUNCE_US) {   // 300 ms
    last_isr_time_us = now;
    zero_reset_requested = true;
}
```

`esp_timer_get_time()` devuelve el tiempo en microsegundos desde el arranque del sistema. Es usable desde una ISR porque no involucra el scheduler de FreeRTOS.

### Spinlock para acceso a variables compartidas entre ISR y tarea

La ISR y `task_sensor` comparten las variables `zero_reset_requested`, `last_roll` y `last_pitch`. En un sistema de un solo núcleo, deshabilitar interrupciones sería suficiente para proteger el acceso. En el ESP32 dual-core, se usa un **spinlock** (portMUX):

```c
static portMUX_TYPE isr_mux = portMUX_INITIALIZER_UNLOCKED;

// En la ISR:
portENTER_CRITICAL_ISR(&isr_mux);
zero_reset_requested = true;
portEXIT_CRITICAL_ISR(&isr_mux);

// En la tarea:
portENTER_CRITICAL(&isr_mux);
bool do_reset = zero_reset_requested;
zero_reset_requested = false;
portEXIT_CRITICAL(&isr_mux);
```

`portENTER_CRITICAL_ISR` deshabilita interrupciones en el núcleo actual y adquiere el spinlock atómicamente. `portENTER_CRITICAL` hace lo mismo desde una tarea. La sección entre ENTER y EXIT es **atómica**: ningún otro hilo ni ISR puede acceder a las variables compartidas durante ese intervalo.

### Patrón bandera + tarea

La ISR **no hace el trabajo** de fijar el punto cero directamente: solo levanta una bandera booleana (`zero_reset_requested = true`). El trabajo real (tomar el mutex, actualizar `zero_roll` y `zero_pitch`) lo hace `task_sensor` en su próximo ciclo. Este patrón es el correcto en FreeRTOS porque:
- La ISR termina en microsegundos (solo escribe un bool).
- La tarea puede usar el mutex sin problemas.
- El punto cero se fija con el ángulo del próximo ciclo completo del filtro (dato fresco).

---

## 12. Mecanismos de sincronización

### Cola `imu_queue`

**Tipo:** `QueueHandle_t` de FreeRTOS.  
**Capacidad:** 5 elementos de tipo `imu_data_t` (dos floats: roll y pitch).  
**Productor:** `task_sensor` (usa `xQueueSend` con timeout 0).  
**Consumidor:** `task_actuator` (usa `xQueueReceive` con timeout 100 ms).

La cola implementa el patrón **productor-consumidor** con desacoplamiento temporal: el sensor produce datos a 50 Hz y el actuador los consume a la misma velocidad, pero si el actuador se retrasa brevemente, hasta 5 muestras quedan en buffer sin perder datos.

### Mutex `zero_mutex`

**Tipo:** `SemaphoreHandle_t` creado con `xSemaphoreCreateMutex()`.  
**Protege:** las variables globales `zero_roll` y `zero_pitch`.  
**Escritores:** `task_sensor` (al recibir la bandera de la ISR).  
**Lectores:** `task_actuator` y `task_monitor`.

Un mutex garantiza **exclusión mutua**: solo una tarea a la vez puede acceder a las variables protegidas. Se usa `xSemaphoreTake` para adquirirlo y `xSemaphoreGive` para liberarlo. Si una tarea no puede adquirir el mutex en 10 ms, usa un valor seguro (0.0) para ese ciclo y emite un warning.

---

## 13. Lógica de control y actuación

### Cálculo del error

```c
float error_roll  = data.roll  - z_roll;
float error_pitch = data.pitch - z_pitch;
```

El error es la diferencia entre el ángulo actual (filtrado) y el punto cero configurado. Si el avión está perfectamente nivelado según el punto cero, el error es 0. Si el avión está inclinado 5° a la derecha respecto al cero, `error_roll = 5.0`.

### Control proporcional (P)

```c
s1 += SERVO1_DIR * error_roll * CONTROL_GAIN;
s2 += SERVO2_DIR * error_roll * CONTROL_GAIN;
```

La corrección aplicada a cada servo es **proporcional al error**. `CONTROL_GAIN = 10.0` significa que cada grado de error produce 10 µs de corrección en el pulso del servo. Con un error de 5°, la corrección es 50 µs. El neutro del servo es 1500 µs, así que el servo se desplazaría a 1550 µs (+3.6° de ángulo mecánico aproximadamente).

Este es un controlador **P puro** (proporcional sin integral ni derivativo). Es suficiente para estabilización rápida donde el objetivo principal es llevar el error a cero. No elimina el error estático perfectamente, pero para esta aplicación (estabilización en tiempo real de una maqueta) es adecuado.

### Direcciones de los servos en un ala volante

**Para corregir alabeo (roll):**  
Ambos servos se mueven en sentido opuesto entre sí para generar un momento de alabeo. Como en este montaje físico particular ambos servos están invertidos especularmente, `SERVO1_DIR` y `SERVO2_DIR` son ambos `+1.0`, y la ecuación hace que ambos se muevan en el mismo sentido eléctrico pero en sentido mecánico opuesto (porque uno está montado al revés).

**Para corregir cabeceo (pitch) — no implementado, solo conceptual:**  
Ambos servos se moverían en el mismo sentido para inclinar toda la superficie alar, generando un momento de cabeceo. El firmware **no** corrige pitch (solo lo mide y lo reporta); esto se documenta a título ilustrativo de cómo sería el control del otro eje en un ala volante.

### Saturación (clamping)

```c
if (s1 < SERVO_MIN_US) s1 = SERVO_MIN_US;   // 1000 µs
if (s1 > SERVO_MAX_US) s1 = SERVO_MAX_US;   // 2000 µs
```

Antes de enviar el pulso al servo se limita al rango físico seguro [1000, 2000] µs. Esto evita que errores grandes (por ejemplo, al inicio cuando el avión está muy inclinado) generen pulsos fuera de especificación que podrían dañar el servo o hacer que pierda el paso.

---

## 14. Subsistema MCPWM — generación de PWM

### Por qué MCPWM y no `ledc`

ESP-IDF ofrece dos drivers para generar PWM: `ledc` (orientado a LEDs con control de brillo) y `mcpwm` (Motor Control PWM, orientado a servos y motores). Para servos se usa MCPWM porque:
- Permite ajustar el ancho de pulso en **microsegundos** con resolución de 1 µs.
- La frecuencia de 50 Hz y los pulsos de 1000–2000 µs son su caso de uso natural.
- Actualizar el pulso durante la operación (sin reinicializar) es trivial: se cambia el valor del comparador.

### API nueva (handles) de ESP-IDF v5/v6

El driver legacy (`driver/mcpwm.h`) fue eliminado en ESP-IDF v6.0. Este proyecto usa la API nueva basada en **handles** (`driver/mcpwm_prelude.h`), que sigue una cadena jerárquica de objetos:

```
MCPWM Timer
    └─ MCPWM Operator
           └─ MCPWM Comparator
           └─ MCPWM Generator (→ GPIO)
```

**Timer:** Define la frecuencia de la señal PWM. Configurado a 1 MHz de resolución (1 tick = 1 µs) y 20000 ticks de período (20 ms = 50 Hz). Cada servo tiene su propio timer para independencia total.

**Operator:** Vincula el timer con los objetos de comparación y generación. Es el "contenedor" lógico de un canal PWM.

**Comparator:** Almacena el valor de comparación que determina cuándo cambia la salida. En este sistema, el valor del comparador **es igual al ancho de pulso en microsegundos**: si el comparador vale 1500, la salida está en alto durante 1500 µs de cada período de 20000 µs.

```c
mcpwm_comparator_config_t cmp_config = { .flags.update_cmp_on_tez = true };
```
`update_cmp_on_tez` significa "actualizar el comparador cuando el contador llega a cero" (Timer Event: Zero). Esto sincroniza los cambios de pulso con el inicio del siguiente período, evitando glitches (pulsos de ancho incorrecto en el período de transición).

**Generator:** Es la salida física (GPIO). Se le definen las acciones:
- Al inicio del período (timer en cero, evento EMPTY): poner GPIO en ALTO.
- Cuando el contador alcanza el valor del comparador: poner GPIO en BAJO.

El resultado es un pulso en alto de exactamente `comparator_value` microsegundos, seguido de la parte baja del resto del período.

```
GPIO: ─────▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀────────────────
         ←───── comparador (ej: 1500 µs) ──────→←────── resto (18500 µs) ─────→
         ←─────────────────────── 20000 µs (50 Hz) ───────────────────────────→
```

**Mover el servo:**
```c
void servos_set_us(uint32_t servo1_us, uint32_t servo2_us) {
    mcpwm_comparator_set_compare_value(s_cmp1, clamp_us(servo1_us));
    mcpwm_comparator_set_compare_value(s_cmp2, clamp_us(servo2_us));
}
```
Cambiar el valor del comparador es suficiente; el timer sigue corriendo sin interrupción.

---

## 15. Tarea de monitoreo

Cada 200 ms imprime por UART0 (visible en la consola via `idf.py monitor`):

```
I (1234) MONITOR: [VUELO] Roll: +3.4° | Pitch: -1.2° | Zero(R/P): 0.0°/0.0°
I (1234) MONITOR:         Servo1: 1534us | Servo2: 1534us
```

El prefijo `I (1234) MONITOR:` es generado automáticamente por `ESP_LOGI`: `I` indica nivel INFO, `1234` es el tiempo en ms desde arranque, y `MONITOR` es el tag de la tarea.

Para leer los ángulos de forma segura (sin tomar el mutex, que podría bloquearse), usa la sección crítica con spinlock ya existente: las variables `last_roll` y `last_pitch` se actualizan por `task_sensor` bajo ese spinlock, y `task_monitor` las lee bajo el mismo spinlock. Para el punto cero sí usa el mutex porque la lectura de dos floats no es atómica.

---

## 16. Sistema de construcción

### CMake + Ninja

ESP-IDF usa **CMake** como sistema de meta-construcción (genera los archivos de build) y **Ninja** como el sistema de construcción real (ejecuta las compilaciones). La cadena es:

```
idf.py build
    → CMake genera build.ninja
        → Ninja invoca xtensa-esp32-elf-gcc para compilar cada .c
        → Ninja invoca el linker para generar ala_volante_rtos.elf
        → esptool convierte el .elf en .bin flasheable
```

El compilador es `xtensa-esp32-elf-gcc`, un GCC cross-compilado para la arquitectura Xtensa LX6 del ESP32. Está incluido en las herramientas de ESP-IDF (`~/.espressif/tools/`).

### `sdkconfig`

El archivo `sdkconfig` (en la raíz del proyecto) es generado y gestionado por ESP-IDF. Contiene **todos** los parámetros de configuración del SDK: desde el tamaño de la flash hasta las opciones de FreeRTOS, pasando por los niveles de log, habilitación de drivers, etc. Se actualiza con `idf.py menuconfig` (interfaz de texto) o editando `sdkconfig.defaults`.

No se edita a mano directamente porque tiene dependencias entre opciones que el sistema de Kconfig gestiona automáticamente.

---

## 17. Cómo compilar y flashear

### Requisito previo: activar el entorno ESP-IDF

En cada nueva terminal, antes de usar `idf.py`:

```bash
source ~/esp/esp-idf/export.sh
```

Este script agrega el compilador Xtensa, `idf.py`, `esptool` y el resto de las herramientas al PATH de la sesión.

### Permisos del puerto USB

La ESP32 aparece en Linux como `/dev/ttyUSB0`. Para acceder sin sudo:

```bash
sudo usermod -a -G dialout $USER
# cerrar sesión y volver a entrar para que tome efecto
```

O de forma temporal (se resetea al desconectar):
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Comandos

```bash
# Compilar
idf.py build

# Flashear (la build ocurre automáticamente si es necesario)
idf.py -p /dev/ttyUSB0 flash

# Abrir monitor serie (115200 baud). Salir con Ctrl+]
idf.py -p /dev/ttyUSB0 monitor

# Compilar + flashear + monitorear en un solo comando
idf.py -p /dev/ttyUSB0 flash monitor

# Borrar toda la flash del chip (útil para resetear estado)
idf.py -p /dev/ttyUSB0 erase-flash
```

---

## 18. Configuración del SDK (sdkconfig)

El archivo `sdkconfig.defaults` establece los parámetros no-default del proyecto:

| Opción | Valor | Razón |
|---|---|---|
| `CONFIG_ESP_CONSOLE_UART_BAUDRATE` | 115200 | Velocidad estándar del monitor serie |
| `CONFIG_FREERTOS_HZ` | 1000 | 1 tick = 1 ms, necesario para `pdMS_TO_TICKS` con períodos de 20 ms y 200 ms |
| `CONFIG_GPIO_CTRL_FUNC_IN_IRAM` | y | Las funciones del driver GPIO en IRAM para baja latencia en la ISR |
| `CONFIG_LOG_DEFAULT_LEVEL_INFO` | y | `ESP_LOGI` visible, `ESP_LOGD/V` suprimidos |
| `CONFIG_ESPTOOLPY_FLASHSIZE_2MB` | y | Tamaño real de la flash de este módulo ESP32 |
| `CONFIG_I2C_SUPPRESS_DEPRECATE_WARN` | y | Silencia el aviso de API I2C legacy (en ESP-IDF 6.x está marcada como EOL pero sigue presente y funcional) |

**Nota sobre el driver I2C:** En ESP-IDF v6.0 se introdujo un nuevo driver I2C (`esp_driver_i2c`). El driver legacy (`driver/i2c.h`) fue marcado como "End Of Life" pero no eliminado. Este proyecto usa el driver legacy porque es el que se enseña en clase y es completamente funcional. La opción `CONFIG_I2C_SUPPRESS_DEPRECATE_WARN` evita que el compilador emita miles de líneas de advertencias que oscurecen los mensajes útiles.

---

## 19. Parámetros ajustables

Todos los parámetros están en `main/config.h`. Los más relevantes para ajuste físico:

### Centrado de servos

Si al montar los servos no quedan centrados con el alerón en posición neutra:

```c
#define SERVO1_ZERO_US  1500   // cambiar a ej. 1480 o 1520
#define SERVO2_ZERO_US  1500   // cambiar a ej. 1480 o 1520
```

Incrementar el valor desplaza el servo en un sentido; decrementar, en el otro. El rango útil es típicamente 1400–1600 µs para encontrar el neutro.

### Inversión de sentido de un servo

Si un servo corrige en el sentido equivocado (exacerba la inclinación en lugar de corregirla):

```c
#define SERVO1_DIR  (-1.0f)   // invertir Servo 1
#define SERVO2_DIR  (+1.0f)   // Servo 2 sin cambios
```

### Ganancia del control

Si la corrección es muy lenta o el avión no se estabiliza a tiempo:
```c
#define CONTROL_GAIN  15.0f   // más agresivo (más µs por grado)
```

Si la corrección es muy brusca o genera oscilaciones:
```c
#define CONTROL_GAIN  5.0f    // más suave
```

### Peso del filtro complementario

Si el sistema es muy sensible a vibraciones (el ángulo salta bajo vibración del motor), reducir el peso del acelerómetro:
```c
#define COMP_ALPHA  0.995f   // 99.5% giroscopio, 0.5% acelerómetro
```

Si el ángulo deriva lentamente con el tiempo (drift del giroscopio visible), aumentar el peso del acelerómetro:
```c
#define COMP_ALPHA  0.95f    // 95% giroscopio, 5% acelerómetro
```

---

## 20. Diagrama de flujo de datos

```
                    ┌─────────────────────────────────────────┐
                    │              HARDWARE                    │
                    │                                          │
  Motor/viento ─→  │ [Inclinación física del avión]          │
                    └──────────────┬──────────────────────────┘
                                   │ aceleración + velocidad angular
                                   ▼
                    ┌──────────────────────────────────────────┐
                    │  MPU6050 (I2C 400kHz, GPIO 21/22)        │
                    │  14 bytes: acc_xyz (int16) + gyro_xyz    │
                    │  Escala: /16384 → g,  /131 → °/s        │
                    └──────────────┬───────────────────────────┘
                                   │ mpu6050_reading_t (5 floats: acc_xyz, gyro_x, gyro_y)
                                   ▼
                    ┌──────────────────────────────────────────┐
                    │  task_sensor (prio 5, 50 Hz exactos)     │
                    │                                          │
                    │  accel_roll  = atan2(acc_y, acc_z)       │
                    │  accel_pitch = atan2(-acc_x, |acc_yz|)   │
                    │                                          │
                    │  roll  = 0.98*(roll + gyro_x*0.02)       │
                    │        + 0.02*accel_roll                 │
                    │  pitch = 0.98*(pitch + gyro_y*0.02)      │
                    │        + 0.02*accel_pitch                │
                    │                                          │
                    │  [si ISR pidió cero] → actualiza         │
                    │  zero_roll/zero_pitch (mutex)            │
                    └──────────────┬───────────────────────────┘
                                   │ imu_data_t {roll, pitch}
                                   │ xQueueSend(imu_queue)
                                   ▼
                    ┌──────────────────────────────────────────┐
                    │  imu_queue (cap. 5 elementos)            │
                    └──────────────┬───────────────────────────┘
                                   │ xQueueReceive(imu_queue)
                                   ▼
                    ┌──────────────────────────────────────────┐
                    │  task_actuator (prio 7)                  │
                    │                                          │
                    │  error = roll - zero_roll                │
                    │  s1 = 1500 + SERVO1_DIR * error * 10    │
                    │  s2 = 1500 + SERVO2_DIR * error * 10    │
                    │  clamp([1000, 2000])                     │
                    └──────────────┬───────────────────────────┘
                                   │ servos_set_us(s1, s2)
                                   ▼
                    ┌──────────────────────────────────────────┐
                    │  MCPWM (comparator_set_compare_value)    │
                    │  GPIO 16 → Servo 1 (alerón izquierdo)   │
                    │  GPIO 17 → Servo 2 (alerón derecho)     │
                    │  50 Hz, pulsos 1000–2000 µs             │
                    └──────────────┬───────────────────────────┘
                                   │ posición mecánica
                                   ▼
                    ┌──────────────────────────────────────────┐
                    │  SG90 x2 → Alerones → Corrección física  │
                    └──────────────────────────────────────────┘

        TTP223 (GPIO 19, falling edge)
               │
               ▼ ISR (IRAM, spinlock, debounce 300ms)
        zero_reset_requested = true
               │
               └──→ task_sensor lee la bandera y fija
                    zero_roll = roll_actual
                    zero_pitch = pitch_actual

        task_monitor (prio 3, 200ms)
               └──→ Lee last_roll/pitch (spinlock)
                    Lee zero_roll/pitch (mutex)
                    ESP_LOGI → UART0 → consola serie
```
