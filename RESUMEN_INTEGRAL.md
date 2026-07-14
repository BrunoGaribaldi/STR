# Resumen Integral para la Defensa — Controlador de Vuelo en Tiempo Real (Ala Volante)

> **Propósito de este documento:** guía de estudio para presentar el proyecto ante el profesor.
> Explica *qué* hace cada parte, *por qué* se decidió así, y *contrasta* la implementación real
> con (a) lo visto en clases/laboratorios y (b) lo prometido en el Anteproyecto y en `DOCUMENTACION.md`.
>
> **Fuentes de verdad usadas:** el código en `main/`, el `Anteproyecto - STR.pdf` y el material de
> `Clases/`. Donde el código y los documentos difieren, se marca explícitamente.
>
> **Materia:** Sistemas en Tiempo Real — Ing. Informática · IUA · Grupo 5
> (Rodeyro, Macedo Rodriguez, Garibaldi, Bertorello) · 2026

---

## 0. TL;DR — Lo que tenés que poder decir en 60 segundos

Es un **controlador de estabilización activa de lazo cerrado** para un avión *ala volante*, corriendo
sobre un **ESP32** con el RTOS **FreeRTOS**. Un sensor inercial **MPU6050** se lee por **I2C a 50 Hz**;
sus datos se **fusionan con un filtro complementario** (giroscopio + acelerómetro) para obtener el
ángulo de inclinación limpio; una **tarea productora** publica ese ángulo en una **cola de FreeRTOS**;
una **tarea actuadora de mayor prioridad** lo consume, calcula un **control proporcional** y mueve
**dos servos SG90** vía **MCPWM**. Un **sensor táctil (GPIO con ISR)** permite fijar el "punto cero"
en caliente. Una **tarea de monitor** imprime el estado por puerto serie cada 200 ms. El requisito de
tiempo real es que **entre medir y corregir pase ≤ 20 ms**.

---

## A. Guía de lectura del código (por dónde empezar)

El proyecto son **5 módulos**. Leelos **en este orden** — está pensado para ir de lo simple a lo
complejo, de modo que cuando llegues al archivo importante (`tasks.c`) ya entiendas todo lo que usa.

| Orden | Archivo | Qué mirar | Tiempo | Prioridad |
|:---:|---|---|:---:|:---:|
| 1️⃣ | **`config.h`** | Tu "tablero de control". Leelo entero: pines, escalas del sensor, rango de servos, `CONTROL_GAIN=10`, `COMP_ALPHA=0.98`, prioridades (5/7/3), tamaños de stack. No hay lógica, solo números. | 5 min | ⭐⭐⭐ |
| 2️⃣ | **`main.c`** | El **arranque**. Solo tiene `app_main()` con los 5 pasos de inicialización en orden. Es el "esqueleto" del sistema. Corto (~40 líneas). | 3 min | ⭐⭐ |
| 3️⃣ | **`mpu6050.c`** (+`.h`) | Cómo se **lee el sensor** por I2C: `mpu6050_read()` trae 14 bytes, los reconstruye (big-endian) y los escala a *g* y °/s. | 10 min | ⭐⭐ |
| 4️⃣ | **`tasks.c`** (+`.h`) | ⭐ **EL ARCHIVO CLAVE.** Las 3 tareas + la ISR + toda la sincronización (cola, mutex, spinlock). Aquí está el filtro complementario, el control P y el patrón ISR→bandera→tarea. **Es lo que más te van a preguntar.** | 30 min | ⭐⭐⭐ |
| 5️⃣ | **`servos.c`** (+`.h`) | Cómo se **genera el PWM** con MCPWM: la cadena timer→operator→comparator→generator, y que mover el servo = cambiar el comparador. | 10 min | ⭐⭐ |

**Si tenés poquísimo tiempo:** leé `config.h` (1) y `tasks.c` (4). Con esos dos entendés el 80 % del
proyecto. `mpu6050.c` y `servos.c` son "cómo hablar con el hardware"; `tasks.c` es "la inteligencia".

**Cómo leer `tasks.c` por dentro** (en este orden):
1. Las variables globales de sincronización (cola, mutex, spinlock, banderas) — arriba del archivo.
2. `gpio_isr_handler` — la ISR (corta: solo levanta una bandera).
3. `task_sensor` — lee, filtra, atiende recalibración, publica.
4. `task_actuator` — consume de la cola, control P, mueve servos.
5. `task_monitor` — imprime.
6. `tasks_sync_init` y `tasks_start` — crean todo (al final, es "plomería").

---

## B. Recorrido paso a paso de los datos (de la vibración al servo)

Este es el **flujo completo** que pediste: seguí un dato desde que el avión se inclina hasta que el
servo lo corrige. Entre cada paso se indica **qué viaja** y **en qué formato**.

### 🔵 Flujo principal (lazo de control, se repite 50 veces por segundo)

**Paso 1 — El mundo físico.**
El avión se inclina (viento, vibración del motor, una mano que lo mueve). El **MPU6050** siente dos
cosas: la **aceleración** (incluida la gravedad) y la **velocidad angular** (giroscopio).

**Paso 2 — Lectura por I2C.** `task_sensor` (cada 20 ms) llama a `mpu6050_read()`.
- Se arma una transacción I2C: `START → dirección 0x68+W → registro 0x3B → RESTART → 0x68+R → 14 bytes → STOP`.
- El ESP32 (maestro) marca el reloj por **SCL** y los datos van y vienen por **SDA**, a 400 kHz.
- ↓ *Viaja:* **14 bytes crudos** (`uint8_t raw[14]`), big-endian.

**Paso 3 — Reconstrucción y escalado** (dentro de `mpu6050_read`).
- Cada par de bytes se une en un entero con signo de 16 bits: `(int16_t)((raw[0]<<8) | raw[1])`.
- Se divide por el factor del datasheet: `/16384` → *g* (acelerómetro), `/131` → °/s (giroscopio).
- ↓ *Viaja:* un `mpu6050_reading_t` = **5 floats** (`acc_x/y/z`, `gyro_x/y`) en unidades físicas.

**Paso 4 — Cálculo de ángulos** (en `task_sensor`).
- *Ángulo por acelerómetro* (dónde está la gravedad): `accel_roll = atan2(acc_y, acc_z)` → grados.
- *Ángulo por giroscopio* (cuánto rotó): se integra `gyro_x · dt`.
- ↓ *Viaja:* dos estimaciones del mismo ángulo, una ruidosa (acel) y una con drift (giro).

**Paso 5 — Filtro complementario** (el "cerebro" del procesamiento, en `task_sensor`).
```c
roll = 0.98*(roll + gyro_x*0.02) + 0.02*accel_roll;   // (y lo mismo para pitch)
```
- Fusiona ambas: 98 % giroscopio (rápido, sin vibración) + 2 % acelerómetro (ancla, sin drift).
- ↓ *Viaja:* **el ángulo limpio** `roll` (y `pitch`), en grados.

**Paso 6 — Publicación en la cola** (en `task_sensor`).
```c
imu_data_t data = { .roll = roll, .pitch = pitch };
xQueueSend(imu_queue, &data, 0);   // si está llena, descarta (no frena los 50 Hz)
```
- ↓ *Viaja:* una copia del struct `imu_data_t` **por dentro de la cola** `imu_queue` (IPC por copia).

**Paso 7 — El actuador despierta** (`task_actuator`, prioridad 7 = ALTA).
- Estaba **bloqueado** en `xQueueReceive(imu_queue, &data, 100ms)`. Al llegar el dato, se **desbloquea**
  y —por tener más prioridad que el sensor— **desaloja** (preempt) al sensor para correr ya mismo.
- ↓ *Viaja:* el `imu_data_t` recibido de la cola.

**Paso 8 — Cálculo del error y control proporcional** (en `task_actuator`).
```c
float z_roll = zero_roll;                 // leído bajo mutex (punto cero)
float error_roll = data.roll - z_roll;    // cuánto está desviado del "nivelado"
float s1 = 1500 + SERVO1_DIR * error_roll * 10;   // 10 µs por grado
float s2 = 1500 + SERVO2_DIR * error_roll * 10;
```
- ↓ *Viaja:* dos anchos de pulso `s1`, `s2` en µs.

**Paso 9 — Saturación** (en `task_actuator`).
- Se recorta cada pulso a `[1000, 2000]` µs (protege el servo).

**Paso 10 — Orden a los servos.** `servos_set_us(s1, s2)`.
- Cambia el **valor del comparador** de cada canal MCPWM: `mcpwm_comparator_set_compare_value(...)`.
- ↓ *Viaja:* el nuevo ancho de pulso al hardware MCPWM.

**Paso 11 — Generación del PWM** (hardware MCPWM, sin CPU).
- El timer cuenta de 0 a 20000 (20 ms). Al inicio del periodo pone el GPIO en **ALTO**; al llegar al
  comparador (p. ej. 1550) lo pone en **BAJO**. Resultado: un pulso de 1550 µs cada 20 ms, en **GPIO16/17**.
- ↓ *Viaja:* una **señal eléctrica PWM** por el cable de señal del servo.

**Paso 12 — Movimiento mecánico.**
- El SG90 traduce el ancho de pulso a un ángulo del brazo → mueve el **alerón** → **corrige la inclinación**.
- El avión se nivela → en el próximo ciclo el error es menor → el servo se mueve menos. **Lazo cerrado.**

> **Todo esto (pasos 2 a 12) ocurre en menos de 20 ms, 50 veces por segundo.** Ese es el requisito de tiempo real.

### 🟡 Flujo secundario 1 — Fijar el punto cero (evento del pulsador)

1. Tocás el sensor → **flanco de bajada en GPIO19** → el hardware dispara la **ISR** `gpio_isr_handler`.
2. La ISR (mínima) chequea el antirrebote (300 ms) y **solo** hace `zero_reset_requested = true`
   (protegido por el spinlock `isr_mux`). Termina en microsegundos.
3. En su próximo ciclo, `task_sensor` ve la bandera, toma el `zero_mutex` y hace
   `zero_roll = roll; zero_pitch = pitch;` → **la inclinación actual pasa a ser el nuevo "nivelado"**.
4. Desde ahí, el `error` del Paso 8 se mide contra este nuevo cero.

### 🟢 Flujo secundario 2 — Monitoreo (cada 200 ms)

1. `task_monitor` (prioridad 3 = BAJA) lee `last_roll`/`last_pitch` (bajo spinlock) y `zero_*` (bajo mutex).
2. Los imprime por **UART0** con `ESP_LOGI` → los ves en la consola (`idf.py monitor`, 115200 baud).
3. Por ser la de menor prioridad, **nunca estorba** al sensor ni al actuador.

### Resumen de "quién le habla a quién"

```
                    ┌──────────── zero_reset (spinlock) ────────────┐
   TTP223/botón ──ISR──> [bandera] ──> task_sensor                  │
                                          │  (lee I2C, filtra)       │
   MPU6050 ──I2C──> task_sensor ──imu_queue(copia)──> task_actuator ─┴─> servos_set_us ──PWM──> SG90 x2
                        │                                 │
                        └── zero_roll/pitch (mutex) ──────┤
                        └── last_roll/pitch (spinlock) ─> task_monitor ──UART──> Consola
```

---

## 1. Veredicto de las tres verificaciones pedidas

### 1.A ¿Todo lo implementado lo vimos en la cursada?

**Mayormente sí, pero hay 2–3 cosas que NO aparecen en el material de clase** y que conviene tener
preparadas para justificar (no está mal usarlas, pero el profesor pidió saber de dónde salió cada cosa).

| Elemento implementado | ¿Visto en clase/lab? | Dónde |
|---|---|---|
| RTOS, tareas, prioridades, planificador preemptivo | ✅ Sí | Diapo `12-rtos`, `01-sistemas_tiempo_real`, `FreeRTOS.pdf` |
| `xTaskCreate`, stacks en *words*, prioridades | ✅ Sí | `FreeRTOS.pdf` |
| `vTaskDelayUntil` (periodicidad exacta) | ✅ Sí | `FreeRTOS.pdf` |
| Colas (queue) productor-consumidor, `xQueueSend/Receive`, `pdTRUE/pdFALSE` | ✅ Sí | Lab `U3_lab4` (ESP32+FreeRTOS), Diapo `12-rtos` |
| Mutex de exclusión mutua | ✅ Sí | Diapo `08-sinc_hilos`, `FreeRTOS.pdf`, Lab `U3_lab1` |
| Semáforo binario + ISR (patrón evento→tarea) | ✅ Sí (concepto) | Lab `U3_lab3` |
| ISR por flanco de bajada en un pulsador, sin funciones bloqueantes | ✅ Sí | Lab `U3_lab3`, `U1_lab52` |
| Sección crítica / spinlock | ✅ Sí (concepto) | Diapo `08-sinc_hilos` ("Spinlocks POSIX"), `12-rtos` |
| Protocolo I2C (maestro/esclavo, SDA/SCL, pull-ups, direccionamiento 7 bits) | ✅ Sí | Lab `I2C_intro`, Diapo (sensores) |
| Sensor MPU6050 (IMU: acelerómetro + giroscopio) | ✅ Sí | Lab `U3_lab1` (usa MPU6050), Diapo `02-señales` |
| Servo SG90 por PWM (periodo 20 ms, pulso 1–2 ms) | ✅ Sí | Lab `U1_lab52`, `U2_lab1`, `U3_lab1` |
| Actuadores / PWM / muestreo / cuantización / aliasing | ✅ Sí | Diapo `02-señales-sistemas_digitales` |
| Filtro digital, FIR/IIR, **media móvil** | ✅ Sí | Diapo `13-filtros_digitales`, Lab `U3_lab1` |
| **Filtro complementario (fusión giro+acelerómetro) + `atan2`** | ❌ **NO** | *No aparece en ningún material.* Ver §1.A.1 |
| **API MCPWM de ESP-IDF (`mcpwm_prelude.h`, timer→operator→comparator→generator)** | ⚠️ Parcial | El *concepto* PWM/servo sí; **esta API concreta no** (los labs usaban `pigpio/gpioServo` en Raspberry). Ver §1.A.2 |
| **API I2C legacy de ESP-IDF (`i2c_cmd_link`, `i2c_master_cmd_begin`)** | ⚠️ Parcial | El *protocolo* I2C sí; **esta API concreta no** (el lab usaba `pigpio/i2cOpen` en Raspberry) |

#### 1.A.1 — El punto más importante: **el filtro cambió respecto a lo enseñado**

- **En clase y en el Anteproyecto** el filtro era una **media móvil** (promediar las últimas N muestras).
  Eso es un filtro **FIR pasabajos**, tema explícito de la diapo `13-filtros_digitales` y del `U3_lab1`.
- **En el código real** se usa un **filtro complementario**:
  `angle = α·(angle + gyro·dt) + (1−α)·accel_angle` con `α = 0.98`.
  Esto **no es una media móvil**: es una **fusión de sensores** (sensor fusion) que combina la
  integración del giroscopio con el ángulo del acelerómetro calculado con `atan2`.
- **Ninguno de estos dos elementos (filtro complementario ni `atan2` para inclinación) aparece en el
  material de clase.** No está "mal", de hecho es la técnica estándar y superior para estabilización,
  pero **tenés que poder explicar por qué se cambió** (ver §5.5). Es el punto donde más probablemente
  te pregunten "¿esto lo vieron?".

#### 1.A.2 — Migración de plataforma: Raspberry Pi (POSIX) → ESP32 (FreeRTOS)

Buena parte de la cursada fue en **Raspberry Pi con POSIX** (pthreads, colas de mensajes POSIX `mq_send`,
pipes, timers POSIX, `pigpio`). El proyecto final está en **ESP32 con FreeRTOS**, que usa **otras APIs**
para los **mismos conceptos**. La correspondencia es directa (ver §11), pero conviene tenerla clara
porque el profesor puede preguntar "esto en POSIX ¿cómo era?".

---

### 1.B ¿Coincide la implementación con `DOCUMENTACION.md`?

**Sí, en un ~95%.** `DOCUMENTACION.md` describe fielmente el código (verifiqué pines, escalas, filtro,
control P, cadena MCPWM, prioridades, colas, mutex, spinlock, ISR). Encontré 4 discrepancias; **3 ya las
corregí** (en el código y/o en la doc) y **1 queda para verificar en el hardware**:

| # | Tema | Discrepancia detectada | Estado |
|---|---|---|---|
| 1 | **Naturaleza del pulsador** | La doc (§3.4) lo describía como TTP223 capacitivo con una explicación enrevesada y contradictoria del flanco; el código lo trata como entrada **pull-up + flanco de bajada** (botón a GND). | ✅ **`DOCUMENTACION.md` reescrita** (§3.4 y nota de pines) para describirlo tal como lo hace el código. ⚠️ Queda **verificar el módulo físico** (ver §17). |
| 2 | **Comentario de la ISR** | `tasks.c` rotulaba la ISR como *"flanco positivo"*, pero la config real es `GPIO_INTR_NEGEDGE` (flanco **negativo**). | ✅ **Corregido** en `tasks.c`. |
| 3 | **Uso de núcleos** | Doc: "solo se usa un núcleo". En realidad usa `xTaskCreate` (no `PinnedToCore`), así que FreeRTOS **puede repartir** en los 2 núcleos. | ℹ️ La doc ya lo aclaraba entre paréntesis. Tenelo presente: **no fijamos núcleo** (los labs sí lo pedían). |
| 4 | **Comentario "pull-down"** | `main.c` (modo de prueba del botón) comentaba *"pull-down"* usando en realidad `GPIO_PULLUP_ENABLE`. | ✅ **Resuelto**: el modo de prueba se eliminó en la limpieza "modo estudio". |

Ninguna afectaba el funcionamiento; eran inconsistencias de **comentarios/descripción**, salvo el punto
1 que conviene verificar contra el hardware real.

---

### 1.C ¿Coincide con el Anteproyecto?

El objetivo, hardware y arquitectura general **coinciden**. Hay **tres desviaciones de alcance**
deliberadas que debés saber justificar:

| # | Anteproyecto prometía | Implementación real | Por qué |
|---|---|---|---|
| 1 | Filtro de **media móvil** | **Filtro complementario** | Mejor rechazo de vibración y sin drift; ver §5.5 |
| 2 | Corregir **ambos ejes** (roll: servos opuestos; pitch: servos iguales) | **Un solo eje: roll**; el pitch solo se mide y reporta | `config.h` lo declara "por requerimiento, un solo eje". (En la versión "modo estudio" la rama de control de pitch se quitó por no usarse.) |
| 3 | 3 hilos: Productor(raw) → Consumidor(filtra) → Actuación(servo) | 3 tareas: **Sensor**(lee **y filtra**) → **Actuador**(consume y mueve) → **Monitor**(imprime) | El filtrado se movió al productor; se agregó una tarea de monitor. Concepto productor-consumidor preservado |

Además, el Anteproyecto dice "servos en sentidos contrarios" para roll; el código usa
`SERVO1_DIR = SERVO2_DIR = +1.0` (mismo signo eléctrico) porque **el servo 2 está montado espejado
físicamente**, con lo que mecánicamente giran opuestos. Es un ajuste de montaje, no un cambio de lógica.

---

## 2. Tecnologías y herramientas

| Capa | Tecnología | Rol en el proyecto |
|---|---|---|
| Lenguaje | **C** (C11) | Todo el firmware |
| Framework | **ESP-IDF 6.1** (Espressif IoT Development Framework) | SDK oficial del ESP32: drivers, build, flasheo |
| Sistema operativo | **FreeRTOS** (incluido en ESP-IDF) | RTOS: tareas, colas, mutex, planificación preemptiva |
| Build | **CMake + Ninja**; compilador `xtensa-esp32-elf-gcc` | Compilación cruzada a Xtensa LX6 |
| Flasheo/monitor | `idf.py flash monitor`, `esptool` | Cargar y ver la consola serie (115200 baud) |
| Config del SDK | `sdkconfig` / `sdkconfig.defaults` (Kconfig) | Tick 1000 Hz, IRAM para GPIO, flash 2 MB, etc. |

---

## 3. Sistema operativo (FreeRTOS) — lo esencial

- **Definición (de la diapo 12):** un RTOS "gestiona tareas y recursos de un sistema cuya operación
  correcta depende tanto del **resultado** como del **tiempo** en que se produce".
- **Preemptivo por prioridades:** la tarea *lista* de mayor prioridad siempre se ejecuta; si llega una
  de mayor prioridad, **desaloja** (preempt) a la que corría. En este proyecto esto es clave: cuando el
  sensor deposita un dato, el **actuador (prio 7)** desaloja al **sensor (prio 5)** para corregir cuanto antes.
- **Tick del sistema: 1000 Hz** (`CONFIG_FREERTOS_HZ=1000`), es decir **1 tick = 1 ms**. Necesario para
  que `pdMS_TO_TICKS(20)` y `pdMS_TO_TICKS(200)` sean exactos.
- **Componentes del RTOS que usamos** (de la teoría): planificador, controlador de interrupciones
  (ISR del pulsador), gestor de recursos (stacks de tareas), reloj de tiempo real (`esp_timer`).

---

## 4. Hardware / dispositivos

### 4.1 ESP32 (microcontrolador — "el cerebro")
- Xtensa **LX6 dual-core @ 240 MHz**, 520 KB SRAM, 2 MB flash (firmware ~186 KB ≈ 18%).
- Periféricos usados: **I2C** (bus del sensor), **MCPWM** (servos), **GPIO+interrupciones** (pulsador),
  **UART0** (consola serie), **esp_timer** (antirrebote).
- Lógica a 3.3 V; alimentado por USB.

### 4.2 MPU6050 (sensor inercial / IMU, entrada)
- IMU de **6 ejes** = acelerómetro 3 ejes + giroscopio 3 ejes.
- **Acelerómetro:** mide aceleración en *g*; en reposo detecta la gravedad → da la inclinación **estática**
  vía trigonometría (`atan2`). Estable a largo plazo pero **muy ruidoso** ante vibración.
- **Giroscopio:** mide velocidad angular (°/s); integrando da el ángulo. Preciso a **corto plazo** pero
  **acumula drift**.
- **Se usan los dos** y se fusionan (§5.5). I2C **400 kHz**, dirección **0x68**, rangos **±2g** y **±250 °/s**.

### 4.3 Servos SG90 (actuadores, salida) ×2
- Micro-servos controlados por **PWM**: periodo **20 ms (50 Hz)**; ancho de pulso define la posición:
  **1000 µs → −90°**, **1500 µs → 0° (neutro)**, **2000 µs → +90°**.
- **Servo 1 = GPIO16 (alerón izq)**, **Servo 2 = GPIO17 (alerón der)**. En un ala volante se llaman
  **elevones** (combinan alerón + elevador).

### 4.4 Sensor táctil / pulsador (evento, entrada)
- Conectado a **GPIO19**, con **pull-up interno** e **interrupción por flanco de bajada**.
- Función: fijar el **punto cero** (la inclinación actual pasa a ser el nuevo "nivelado").
- ⚠️ El código lo trata como **botón clásico a GND**; el Anteproyecto/doc lo llaman **TTP223 capacitivo**
  (ver §1.B punto 1 y §6).

### 4.5 Mapa de pines
| Señal | GPIO | Componente |
|---|---|---|
| I2C SDA / SCL | 21 / 22 | MPU6050 |
| PWM Servo 1 / 2 | 16 / 17 | SG90 izq / der |
| Pulsador (ISR) | 19 | Táctil / botón |

---

## 5. Tratamiento de la señal y procesamiento de datos

### 5.1 Señales, sensores y actuadores (marco teórico, diapo 02)
El proyecto es un **sistema digital de control** clásico:
`Sensor (MPU6050) → Controlador digital (ESP32) → Actuador (servos) → Planta (avión)`.
El MPU6050 **muestrea** una magnitud física (inclinación) → señal **discreta en tiempo**;
internamente la **cuantiza** a enteros de 16 bits (int16). Nosotros muestreamos a **50 Hz**.

### 5.2 Lectura cruda por I2C
Se leen **14 bytes de golpe** desde `ACCEL_XOUT_H (0x3B)`: acc(XYZ)+temp+gyro(XYZ). El MPU6050 entrega
**big-endian** (byte alto primero):
```c
int16_t acc_x = (int16_t)((raw[0] << 8) | raw[1]);   // cast a int16_t = complemento a 2
```
El cast a `int16_t` es esencial para interpretar bien los valores **negativos**.

### 5.3 Escala a unidades físicas (factores del datasheet)
```c
out->acc_x  = acc_x / 16384.0f;   // ±2g   → g
out->gyro_x = gyr_x / 131.0f;     // ±250  → °/s
```

### 5.4 Ángulo por acelerómetro (`atan2`)
```c
accel_roll  = atan2f(acc_y, acc_z) * 57.2957795f;                 // 180/π
accel_pitch = atan2f(-acc_x, sqrtf(acc_y*acc_y + acc_z*acc_z)) * 57.2957795f;
```
`atan2` devuelve el ángulo en [−π, π] usando el vector gravedad. Es exacto en reposo pero **se ensucia
con la vibración del motor**.

### 5.5 ⭐ Filtro complementario (el corazón del procesamiento)
```c
roll  = COMP_ALPHA * (roll  + r.gyro_x * SENSOR_DT_S) + (1.0f - COMP_ALPHA) * accel_roll;
pitch = COMP_ALPHA * (pitch + r.gyro_y * SENSOR_DT_S) + (1.0f - COMP_ALPHA) * accel_pitch;
```
Con `α = 0.98` y `dt = 0.02 s`:
- **98 %** viene de **integrar el giroscopio** → respuesta rápida y sin ruido de vibración.
- **2 %** viene del **acelerómetro** → "ancla" el ángulo a la gravedad real y **elimina el drift**.

**Por qué se eligió sobre la media móvil del Anteproyecto:** la media móvil solo suaviza el ruido del
acelerómetro pero **no resuelve el drift** ni aprovecha el giroscopio; introduce **retardo** (malo en
tiempo real). El complementario **fusiona ambos sensores**, da lo mejor de cada uno y es la alternativa
**barata al filtro de Kalman**. **Este es el tema a defender porque no se vio en clase.**

---

## 6. Señales, interrupciones (ISR) y punto cero

```c
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    int64_t now = esp_timer_get_time();               // µs desde el arranque
    portENTER_CRITICAL_ISR(&isr_mux);
    if ((now - last_isr_time_us) >= BUTTON_DEBOUNCE_US) {  // antirrebote 300 ms
        last_isr_time_us = now;
        zero_reset_requested = true;                  // ← SOLO levanta una bandera
    }
    portEXIT_CRITICAL_ISR(&isr_mux);
}
```
Conceptos (todos vistos en `U3_lab3`):
- **`IRAM_ATTR`**: coloca la ISR en RAM interna → latencia mínima y determinística (no depende de la
  caché de flash). Alineado con `CONFIG_GPIO_CTRL_FUNC_IN_IRAM=y`.
- **ISR corta y no bloqueante**: no imprime, no toma mutex, no hace el trabajo pesado. Solo escribe un
  `bool`. El trabajo real (fijar el cero) lo hace `task_sensor` en su próximo ciclo con el ángulo fresco.
  Este es el **patrón bandera/semáforo → tarea** ("deferred processing").
- **Antirrebote por software**: 300 ms para que un toque no genere varios eventos.
- **Flanco de bajada** (`GPIO_INTR_NEGEDGE`) con **pull-up**: en reposo la línea está en ALTO; el evento
  se dispara al caer a BAJO.

**El Anteproyecto exige "ISR de máxima prioridad"**: se cumple porque una interrupción de hardware
**siempre desaloja** a cualquier tarea de FreeRTOS.

---

## 7. Arquitectura de tareas, prioridades y gestión

Tres tareas + una ISR (`xTaskCreate` — **no** se fija núcleo):

| Tarea | Prioridad | Disparo | Stack | Rol |
|---|---|---|---|---|
| `task_sensor` | **5** (media) | periódica **50 Hz** (`vTaskDelayUntil`) | 4096 w | Lee IMU, filtra, atiende recalibración, publica en la cola |
| `task_actuator` | **7** (alta) | **por la cola** (`xQueueReceive`, timeout 100 ms) | 4096 w | Consume ángulo, control P, mueve servos |
| `task_monitor` | **3** (baja) | periódica **200 ms** (`vTaskDelayUntil`) | 3072 w | Imprime estado por serie |
| ISR pulsador | HW (máx) | flanco GPIO19 | — | Pide fijar punto cero |

**Criterio de prioridades (justificación):** el **actuador** (crítico en tiempo, cierra el lazo) tiene
**más prioridad que el productor**, de modo que apenas hay un dato, corrige. El **monitor** tiene la
**menor** porque puede retrasarse sin afectar el control (solo informa). Esto es el modelo preemptivo
por prioridades de la diapo 12.

**Periodicidad exacta con `vTaskDelayUntil`** (visto en `FreeRTOS.pdf`):
```c
TickType_t last_wake = xTaskGetTickCount();
for (;;) { /* trabajo */ vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS)); }
```
A diferencia de `vTaskDelay` (relativo, acumula deriva), `vTaskDelayUntil` calcula el próximo despertar
sobre un **instante absoluto** → 50 Hz reales aunque la lectura I2C tarde distinto cada vez.

---

## 8. Comunicación y sincronización entre tareas / ISR

El proyecto usa **hilos (tareas de FreeRTOS), no procesos** → todo comparte el mismo espacio de memoria.
Tres mecanismos, cada uno para su caso:

### 8.1 Cola `imu_queue` — comunicación productor↔consumidor (push por copia)
- `xQueueCreate(5, sizeof(imu_data_t))`; `imu_data_t = {float roll, pitch}`.
- Productor `xQueueSend(..., 0)`: si está llena **descarta** y avisa (no puede frenar los 50 Hz).
- Consumidor `xQueueReceive(..., 100 ms)`: **se bloquea** hasta que hay dato o vence el timeout. No hace
  *polling* ni `vTaskDelay`; **la cola es su reloj**. Esto es exactamente el `U3_lab4` (cola productor-
  consumidor entre tareas de distinta prioridad) y es una **IPC de tipo push** (diapo `10-ipc_push`):
  el mensaje viaja por copia, no por memoria compartida.

### 8.2 Mutex `zero_mutex` — exclusión mutua del punto cero
- Protege `zero_roll` / `zero_pitch` (dos `float`, cuya lectura conjunta **no es atómica**).
- Escritor: `task_sensor` (al recalibrar). Lectores: `task_actuator` y `task_monitor`.
- `xSemaphoreTake(zero_mutex, 10 ms)` / `xSemaphoreGive(...)`. Si no lo toma en 10 ms usa un valor seguro
  (0.0) ese ciclo. **Mutex = semáforo binario para exclusión mutua** (diapo 08, `FreeRTOS.pdf`).

### 8.3 Spinlock `isr_mux` (`portMUX`) — sincronización tarea↔ISR
- La ISR **no puede tomar un mutex** (bloquearía). En un ESP32 **dual-core** deshabilitar interrupciones
  no basta, así que se usa un **spinlock** (espera activa) que protege `zero_reset_requested`,
  `last_roll/pitch`, `last_servo*`.
- `portENTER_CRITICAL_ISR(&isr_mux)` en la ISR / `portENTER_CRITICAL(&isr_mux)` en la tarea → sección
  **atómica**. Concepto de **sección crítica / spinlock** de la diapo `08-sinc_hilos`.

**Por qué mutex en un lado y spinlock en el otro:** el spinlock es para tramos **ultra cortos** que la
ISR también toca (no puede dormir); el mutex es para el recurso `zero_*` al que acceden **solo tareas**
(pueden dormir esperando).

---

## 9. Lógica de control y actuación

### 9.1 Control Proporcional (P)
```c
float error_roll = data.roll - z_roll;          // desvío respecto al punto cero
float s1 = SERVO1_ZERO_US, s2 = SERVO2_ZERO_US; // parten del neutro (1500 µs)
s1 += SERVO1_DIR * error_roll * CONTROL_GAIN;   // GAIN = 10 µs por grado
s2 += SERVO2_DIR * error_roll * CONTROL_GAIN;
```
- Corrección **proporcional al error** (5° → 50 µs). Es un **P puro** (sin I ni D): simple y suficiente
  para llevar el error a ~0 en esta maqueta.
- **Se controla un solo eje: el ROLL.** El pitch se calcula y se reporta por consola, pero no genera
  corrección (así lo pide el alcance del proyecto para un ala volante, donde el roll es el eje natural).
- `SERVO1_DIR/SERVO2_DIR` permiten invertir el sentido de un servo sin tocar la lógica (montaje espejo).

### 9.2 Saturación (clamping)
```c
if (s1 < SERVO_MIN_US) s1 = SERVO_MIN_US;   // 1000
if (s1 > SERVO_MAX_US) s1 = SERVO_MAX_US;   // 2000
```
Nunca se manda un pulso fuera de [1000, 2000] µs → protege el servo de errores grandes (p. ej. al arrancar).

---

## 10. Generación de PWM — subsistema MCPWM

Se usa **MCPWM** (Motor Control PWM), no `ledc`, porque permite fijar el ancho de pulso **en µs con
resolución de 1 µs** — ideal para servos. La API nueva de ESP-IDF v6 es por **handles encadenados**:

```
Timer (1 MHz, 20000 ticks = 20 ms) → Operator → Comparator → Generator → GPIO
```
- **Timer**: define la frecuencia (50 Hz). Resolución 1 µs/tick, periodo 20000 ticks. **Un timer por servo**.
- **Comparator**: su valor **es el ancho de pulso en µs**. `update_cmp_on_tez = true` → el cambio se aplica
  al inicio del periodo (evita *glitches*).
- **Generator**: en `TIMER_EMPTY` (inicio) pone la salida **ALTA**; al llegar al comparador la pone **BAJA**.
- Mover el servo = **cambiar el valor del comparador**:
```c
void servos_set_us(uint32_t s1, uint32_t s2) {
    mcpwm_comparator_set_compare_value(s_cmp1, clamp_us(s1));
    mcpwm_comparator_set_compare_value(s_cmp2, clamp_us(s2));
}
```
> ⚠️ **A tener presente:** esta API específica **no se vio en clase** (los labs controlaban el SG90 con
> `gpioServo` de `pigpio` en Raspberry). El **concepto** de PWM/servo sí (`U1_lab52`, `U2_lab1`).

---

## 11. ¿Usamos POSIX? Correspondencia POSIX ↔ FreeRTOS

**El proyecto NO usa POSIX directamente** (no hay `pthread`, `mq_send`, `pipe`, `SIGALRM`, timers POSIX).
Corre sobre **FreeRTOS**. Pero **los conceptos** que enseñó la cátedra en POSIX se aplican con la API de
FreeRTOS. Tenelo listo por si preguntan:

| Concepto (visto en POSIX) | En POSIX/Linux | En este proyecto (FreeRTOS) |
|---|---|---|
| Hilo / unidad de ejecución | `pthread_create` | `xTaskCreate` |
| Espera temporizada / periódica | `sleep`, `usleep`, timers POSIX | `vTaskDelay`, **`vTaskDelayUntil`** |
| Exclusión mutua | `pthread_mutex_*` | `xSemaphoreCreateMutex` + `Take/Give` |
| Sección crítica / spinlock | `pthread_spin_*` | `portENTER_CRITICAL` (`portMUX`) |
| Cola de mensajes (IPC push) | `mq_send` / `mq_receive` | `xQueueSend` / `xQueueReceive` |
| Señal / evento asíncrono | señales (`SIGALRM`), handlers | **ISR de GPIO** + bandera |
| Reloj de alta resolución | `clock_gettime` | `esp_timer_get_time()` |

> Nota: **procesos** (fork), **señales POSIX** e **IPC pull** (memoria compartida / archivos) se vieron
> en clase pero **no se usan** acá: en un microcontrolador con FreeRTOS todo son **hilos** en un único
> espacio de memoria, no hay procesos separados. Buen punto para aclarar "planificación/sincronización
> de **procesos** no aplica; sí de **hilos/tareas**".

---

## 12. Manejo del tiempo

- **Tick de FreeRTOS**: 1 ms (1000 Hz).
- **Muestreo periódico exacto**: `vTaskDelayUntil` (sensor 20 ms, monitor 200 ms).
- **Tiempo absoluto en µs**: `esp_timer_get_time()` (usable dentro de la ISR, no toca el scheduler) para
  el **antirrebote** de 300 ms.
- **Timeouts** como mecanismo de robustez: cola (100 ms), mutex (10 ms), transacción I2C (100 ms).

---

## 13. Flujo de arranque (`app_main`)
```
1. mpu6050_i2c_init()  → configura bus I2C maestro (GPIO21/22, 400 kHz)
2. mpu6050_init()      → despierta el sensor (PWR_MGMT_1=0), rangos ±2g / ±250°/s
3. servos_init()       → cadena MCPWM ×2, servos al neutro (1500 µs)
4. tasks_sync_init()   → crea cola + mutex, configura GPIO19 e instala la ISR
5. tasks_start()       → crea las 3 tareas; el scheduler toma el control
```
Cada paso va envuelto en `ESP_ERROR_CHECK(...)`: si algo falla, **pánico controlado** con mensaje claro
en vez de seguir con hardware mal inicializado. Al retornar `app_main`, su tarea se elimina y el resto sigue.

---

## 14. Estructura del proyecto
```
STR_ENTREGA/
├── CMakeLists.txt          # raíz ESP-IDF → project(ala_volante_rtos)
├── sdkconfig(.defaults)    # tick 1000 Hz, IRAM GPIO, flash 2 MB, I2C legacy
└── main/
    ├── CMakeLists.txt      # SRCS + REQUIRES (driver, esp_driver_gpio/mcpwm, esp_timer)
    ├── config.h            # TODOS los parámetros (pines, ganancias, prioridades…)
    ├── main.c              # app_main: los 5 pasos de arranque (~40 líneas)
    ├── mpu6050.[ch]        # driver I2C del sensor
    ├── servos.[ch]         # control MCPWM de los 2 servos
    └── tasks.[ch]          # las 3 tareas + ISR + objetos de sincronización
```
Todo parámetro ajustable está **centralizado en `config.h`** (buena práctica: un solo lugar para tocar
pines, ganancia, α del filtro, prioridades, etc.).

> **Nota "modo estudio":** la versión que estás estudiando fue limpiada de código que no se ejecutaba
> (se quitaron `gyro_z` sin usar, la rama de control de pitch, y los dos modos de prueba de hardware
> `BUTTON_TEST_MODE`/`SERVO_TEST_MODE`). Eso deja `main.c` en ~40 líneas y el actuador sin ramas
> condicionales, sin afectar nada de la lógica de tiempo real evaluable.

---

## 14.1 Sistema de compilación: CMake, CMakeLists y sdkconfig

Estas cuatro piezas **no son código del proyecto**, son la "maquinaria" que ESP-IDF necesita para
compilar. Conviene saber qué es cada una porque el profesor puede preguntar por qué están ahí.

### ¿Qué es CMake?
Es un **generador de sistema de compilación**. No compila directamente: vos describís el proyecto (qué
archivos hay y de qué dependen) en archivos `CMakeLists.txt`, y **CMake genera** los archivos que el
compilador real va a usar (en ESP-IDF genera archivos de **Ninja**, que es quien finalmente llama a `gcc`).

Cadena completa al hacer `idf.py build`:
```
idf.py build
   → CMake lee los CMakeLists.txt y genera build.ninja
       → Ninja llama a xtensa-esp32-elf-gcc para compilar cada .c
           → el linker arma el .elf → esptool lo convierte en .bin flasheable
```
> Analogía: `CMakeLists.txt` es la **receta**; **CMake** es el cocinero que la lee y prepara las
> instrucciones; **Ninja/gcc** son quienes cocinan.

### Los dos `CMakeLists.txt` (obligatorios)

**`CMakeLists.txt` (raíz)** — punto de entrada, 3 líneas:
```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)   # trae TODA la maquinaria de ESP-IDF
project(ala_volante_rtos)                            # nombre del proyecto
```

**`main/CMakeLists.txt`** — registra tu código:
```cmake
idf_component_register(
    SRCS "main.c" "mpu6050.c" "servos.c" "tasks.c"              # qué archivos compilar
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_gpio esp_driver_mcpwm esp_timer  # de qué componentes depende
)
```
Acá le decís a ESP-IDF **cuáles son tus `.c`** y **qué módulos del SDK usás** (I2C, GPIO, MCPWM, timer).

**¿Se pueden sacar?** ❌ **No, son obligatorios.** Sin ellos ESP-IDF ni siquiera reconoce la carpeta como
un proyecto: **no compila**. Son tan esenciales como el propio código.

### `sdkconfig.defaults` vs `sdkconfig`

| Archivo | Qué es | ¿Va en git? | ¿Se puede sacar? |
|---|---|---|---|
| **`sdkconfig.defaults`** | **Fuente** (34 líneas, escrito a mano). Las opciones del SDK que cambiamos respecto al default de fábrica. ESP-IDF lo lee **por convención** para generar el `sdkconfig`. | ✅ Sí | ❌ No conviene (ver abajo) |
| **`sdkconfig`** | **Generado** (~3700 líneas, dice "DO NOT EDIT"). Se recrea solo a partir de `sdkconfig.defaults` + `menuconfig` en cada build. | ❌ No (lo pusimos en `.gitignore`) | ✅ Sí, se regenera solo |

**Opciones clave que fija `sdkconfig.defaults`** (por eso NO se saca):

| Opción | Qué hace | Si lo sacás… |
|---|---|---|
| `CONFIG_FREERTOS_HZ=1000` | **Tick de FreeRTOS = 1 ms** | Vuelve al default (**100 Hz = 10 ms**): peor resolución temporal y más *jitter*. En un sistema de **tiempo real** es justo lo que no querés. |
| `CONFIG_GPIO_CTRL_FUNC_IN_IRAM=y` | Driver GPIO en RAM → **ISR de baja latencia** | La ISR del pulsador podría tardar más. |
| `CONFIG_ESPTOOLPY_FLASHSIZE_2MB=y` | Declara **2 MB de flash** (tu módulo) | Podría asumir otro tamaño y fallar al flashear. |
| `CONFIG_I2C_SUPPRESS_DEPRECATE_WARN=y` | Silencia el aviso de API I2C legacy | Cosmético: vuelven los warnings. |

> **En una frase:** `sdkconfig.defaults` es la **memoria escrita** de las decisiones de configuración del
> proyecto (sobre todo el tick de 1 kHz que sostiene el tiempo real). Compila igual sin él, pero el
> comportamiento temporal cambiaría → por eso se versiona y se mantiene.

*(Todo esto también está en `DOCUMENTACION.md` §6, §16 y §18.)*

---

## 15. Requisito de tiempo real (cómo se cumple)

- **Restricción dura del Anteproyecto:** un retraso implica perder el control del avión.
- **Garantía del diseño:** el lazo *medir → filtrar → publicar → corregir* cierra en **≤ 20 ms** (50 Hz).
  El actuador tiene **prioridad mayor** que el sensor, así que la corrección ocurre apenas hay dato; la
  ISR (máxima prioridad de HW) redefine el cero al instante.
- **Métrica de validación (Anteproyecto §4):** (1) con vibración, la consola muestra ángulos **estables,
  sin picos** (mérito del filtro); (2) al tocar el sensor, el sistema **fija el nuevo 0** al instante;
  (3) inclinación en roll → servos se mueven (opuestos mecánicamente); pitch → mismo sentido (si se
  activara el control de pitch).

---

## 16. Preguntas probables del profesor (y respuesta corta)

1. **¿Este filtro lo vieron en clase?** — En clase vimos media móvil (FIR). Acá usamos **filtro
   complementario** (fusión giro+acelerómetro), que resuelve el drift y la vibración sin el retardo de la
   media móvil; es la alternativa liviana al Kalman. *(Es la desviación principal — tenela sólida.)*
2. **¿Por qué el actuador tiene más prioridad que el sensor?** — Porque cierra el lazo de control y es lo
   crítico en tiempo; apenas hay dato, desaloja al sensor y corrige.
3. **¿Por qué `vTaskDelayUntil` y no `vTaskDelay`?** — Para muestreo periódico **exacto** (instante
   absoluto), sin acumular deriva por el tiempo variable de la lectura I2C.
4. **¿Por qué la ISR solo levanta una bandera?** — Debe ser mínima y no bloqueante; el trabajo (con mutex)
   lo hace la tarea con el ángulo fresco del próximo ciclo. Patrón evento→tarea.
5. **¿Mutex vs spinlock, por qué los dos?** — Spinlock (`portMUX`) para el tramo cortísimo que también
   toca la ISR (no puede dormir) y por ser dual-core; mutex para `zero_*` entre tareas que sí pueden esperar.
6. **¿Usan procesos o hilos? ¿POSIX?** — Hilos (tareas FreeRTOS), un solo espacio de memoria. No POSIX
   directo; los conceptos POSIX se mapean a la API FreeRTOS (ver §11).
7. **¿Por qué MCPWM y no LEDC?** — Resolución en µs y actualización sin glitches, natural para servos.
8. **¿Por qué controlan un solo eje si el Anteproyecto decía dos?** — Decisión de alcance: para un ala
   volante el **roll** es el eje natural de los alerones. El **pitch se mide y se reporta**, pero no se
   corrige.
9. **¿El pulsador es capacitivo o botón?** — *(Verificá el hardware real.)* Eléctricamente el firmware lo
   trata como botón a GND (pull-up + flanco de bajada). Si es un TTP223 estándar (activo en alto), habría
   que revisar el flanco/pull.
10. **¿Para qué sirven los `CMakeLists.txt` y `sdkconfig.defaults`?** — Los `CMakeLists.txt` le dicen a
    ESP-IDF qué archivos compilar y de qué depende el proyecto (son obligatorios: sin ellos no compila).
    `sdkconfig.defaults` fija las opciones del SDK que necesitamos, sobre todo el **tick de FreeRTOS a
    1 kHz** que sostiene el tiempo real. El `sdkconfig` grande es generado y no se versiona. (Ver §14.1.)

---

## 17. Cosas a arreglar/verificar antes de presentar

**Ya resueltas en esta versión** (parte del "modo estudio"):
- ✅ Comentario erróneo de la ISR en `tasks.c` ("flanco positivo" → corregido a **negativo/bajada**).
- ✅ Comentario "pull-down" en `main.c`: desapareció al quitar el modo de prueba del botón.
- ✅ `DOCUMENTACION.md` §3.4 y nota de pines: reescritas para describir el pulsador tal como lo trata
  el código (pull-up interno + flanco de bajada).

**Pendientes / a tener en cuenta:**
1. **Coherencia física del pulsador.** El firmware trata GPIO19 como **entrada activa en bajo**
   (reposo ALTO, se acciona a GND, flanco de bajada). Un TTP223 estándar es *activo en alto*: si el
   módulo real es capacitivo, verificá que esté jumpereado como activo-bajo o que efectivamente sea un
   botón a GND. En la maqueta se comprueba tocando y viendo si el "punto cero" se fija.
2. **Cambio de filtro** (media móvil → complementario) respecto al Anteproyecto: preséntalo como una
   **mejora justificada** (ver §5.5), no como un olvido.

*(Nada de esto afecta el funcionamiento; son de coherencia/documentación.)*
