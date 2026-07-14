# Guía de Estudio Integral — Controlador de Vuelo en Tiempo Real (Ala Volante)

> **Qué es este documento.** Un material de estudio **completo y desde cero** para rendir el examen.
> No supone que ya sabés de sistemas embebidos: cada concepto se explica desde su raíz (qué es, por qué
> existe, cómo se usa en *nuestro* proyecto y dónde lo vimos en la cursada). Si un término aparece por
> primera vez, se define. Leelo de arriba hacia abajo la primera vez; después usá el índice para repasar.
>
> **Materia:** Sistemas en Tiempo Real — Ing. Informática · IUA · Grupo 5
> (Rodeyro, Macedo Rodriguez, Garibaldi, Bertorello) · 2026

---

## Cómo está organizado

- **PARTE I — El proyecto en criollo:** qué problema resuelve y la idea de la solución, sin tecnicismos.
- **PARTE II — Conceptos desde cero:** toda la teoría que necesitás (RTOS, tareas, interrupciones,
  sincronización, señales, filtros, I2C, PWM, puerto serie). Acá están respondidas **tus 8 preguntas**.
- **PARTE III — El hardware pieza por pieza.**
- **PARTE IV — El software:** archivos, tareas, parámetros, guía de lectura.
- **PARTE V — Recorrido paso a paso** de un dato, reexplicado para que se entienda.
- **PARTE VI — Contexto académico:** qué vimos en clase, qué cambió respecto al anteproyecto, build system.
- **PARTE VII — Profundización:** detalles finos que dan puntos extra (I2C byte a byte, stacks, IRAM,
  glitches de PWM, robustez) y cómo compilar/flashear/monitorear en la práctica.
- **PARTE VIII — Repaso:** preguntas probables del profesor y glosario.

> A lo largo del texto vas a ver cajas **❓ Tu pregunta** que responden exactamente lo que preguntaste.

> **🔗 Correlación código ↔ guía (en los dos sentidos).** El código fuente tiene comentarios con marcas
> **`[Guia §X]`** que apuntan a la sección de esta guía que explica ese concepto. Por ejemplo, en `tasks.c`
> la línea del filtro dice `[Guia §II.8]`, y acá §II.8 lo explica desde cero. Así podés estudiar leyendo el
> código y saltando a la teoría, o al revés. La tabla de correspondencia rápida:
>
> | Archivo | Concepto principal | Sección |
> |---|---|---|
> | `config.h` | Parámetros ajustables | §IV.4 |
> | `main.c` | Arranque del sistema | §II.2, §IV |
> | `mpu6050.c/.h` | Sensor + protocolo I2C | §II.9.a, §III.2 |
> | `servos.c/.h` | PWM / MCPWM | §II.9.c |
> | `tasks.c` (clave) | Tareas, ISR, sincronización, filtro, control | §II.4–II.8, §IV.3, §V |
> | `tasks.h` | Struct de la cola | §II.6.c |

---
---

# PARTE I — EL PROYECTO EN CRIOLLO

## I.1 El problema

Un avión **ala volante** (como el bombardero B-2) no tiene cola ni timón vertical: es solo un ala con
forma de boomerang. Esa forma lo hace **inestable**: cualquier soplo de viento o vibración lo inclina, y
si nadie corrige esa inclinación al instante, se desestabiliza y cae. Un piloto humano **no reacciona lo
suficientemente rápido** (habría que corregir decenas de veces por segundo). Además, el motor vibra y esa
vibración **ensucia** las mediciones del sensor de inclinación.

Entonces hay **dos problemas**: (1) corregir la inclinación automáticamente y muy rápido, y (2) medir la
inclinación de forma **limpia** a pesar de la vibración.

## I.2 La idea de la solución (en una imagen)

Es un **sistema de control de lazo cerrado**. La mejor analogía es el **termostato** de una estufa:
mide la temperatura, la compara con la deseada, y si hay diferencia, enciende/apaga la estufa. Repite
esto para siempre. Nuestro sistema hace lo mismo pero con **ángulo de inclinación** en vez de temperatura:

1. **Mide** la inclinación del avión (con un sensor).
2. La **compara** con la inclinación deseada (el "nivelado", que llamamos *punto cero*).
3. Si hay diferencia (*error*), **mueve dos aletas** (alerones) para contrarrestarla.
4. Vuelve al paso 1. Esto pasa **50 veces por segundo**.

"Lazo cerrado" significa justamente eso: la salida (mover los alerones) afecta a lo que se mide después
(el avión se endereza), y esa nueva medición vuelve a entrar al sistema. Se retroalimenta.

## I.3 Vista de pájaro de las piezas

```
   [Sensor de inclinación]  →  [Cerebro: ESP32]  →  [Aletas motorizadas]
        MPU6050                (mide, filtra,         2 servos SG90
      (acelerómetro            decide, ordena)        que mueven los alerones
       + giroscopio)                 │
                                     │
                              [Botón] para decir
                            "esta posición es el 0"
                                     │
                              [Puerto serie] para
                             ver los números en la PC
```

- **ESP32:** la computadorcita (microcontrolador) que ejecuta todo el programa.
- **MPU6050:** el sensor que dice cómo está inclinado el avión.
- **Servos SG90:** dos motorcitos que mueven los alerones.
- **Botón:** para fijar cuál es la posición "nivelada" de referencia.
- **Puerto serie:** cable por el que el ESP32 le "cuenta" a la PC qué está pasando.

Todo el resto del documento explica **cada una de estas piezas y cómo se hablan entre sí**.

---
---

# PARTE II — CONCEPTOS DESDE CERO

Esta es la parte teórica. Cada sección: **qué es → por qué existe → cómo lo usamos → dónde lo vimos.**

## II.1 ¿Qué es un "Sistema de Tiempo Real"? (el nombre de la materia)

Un **sistema de tiempo real (STR)** es un sistema donde **no alcanza con dar el resultado correcto: hay
que darlo a tiempo**. Si la respuesta llega tarde, aunque sea correcta, **no sirve** (o es un desastre).

- Ejemplo cotidiano: el airbag de un auto. De nada sirve que se infle "correctamente" medio segundo
  después del choque. Tiene que ser *ya*.
- En nuestro proyecto: de nada sirve calcular perfectamente la corrección del alerón si el avión ya se
  cayó porque tardamos demasiado.

Se distinguen dos tipos:
- **Tiempo real duro (hard):** si te pasás del plazo, el sistema **falla** (airbag, marcapasos). Nuestro
  control de vuelo es de este tipo conceptualmente.
- **Tiempo real blando (soft):** pasarse del plazo **degrada** la calidad pero no es catástrofe (un video
  que se traba un instante).

> **La palabra clave del STR es "determinismo":** poder **garantizar** que algo va a ocurrir dentro de un
> plazo conocido, siempre. No "rápido en promedio", sino "nunca más tarde de X". Todo lo que sigue (RTOS,
> prioridades, interrupciones) existe para lograr ese determinismo.

*(Diapositiva `01-sistemas_tiempo_real`.)*

## II.2 Microcontrolador y el ESP32 — ❓ Tu pregunta 6

> **❓ Tu pregunta 6:** *Usamos una ESP32. ¿Cómo funciona? ¿Qué necesitamos para compilar y guardar el
> programa en ella? ¿FreeRTOS ya viene dentro de la ESP32?*

### ¿Qué es un microcontrolador?
Una **computadora entera dentro de un solo chip**, pensada para controlar cosas físicas (sensores,
motores), no para navegar por internet. Tiene lo mismo que una PC pero en miniatura y modesto:
- **CPU** (procesador que ejecuta las instrucciones),
- **RAM** (memoria de trabajo, se borra al apagar),
- **Flash** (memoria permanente donde vive tu programa, como el "disco"),
- **Periféricos** (bloques de hardware para hablar con el mundo: pines, I2C, PWM, puerto serie…).

La diferencia con un microprocesador (el de tu PC) es que el micro **trae todo integrado** y está hecho
para ejecutar **un solo programa** a la vez, de forma confiable y por años.

### El ESP32 en concreto
- **CPU:** Xtensa LX6 de **dos núcleos** a 240 MHz (dos "cerebros" que pueden trabajar en paralelo).
- **RAM:** 520 KB. **Flash:** 2 MB (donde guardamos nuestro firmware, que ocupa ~186 KB).
- **Periféricos que usamos:** I2C (para el sensor), MCPWM (para los servos), UART (puerto serie), GPIO con
  interrupciones (para el botón), timers.
- **Alimentación:** 3.3 V, se alimenta por el cable USB.

> **"Firmware"** = el programa que grabás dentro de un dispositivo embebido. Es "software", pero como vive
> pegado al hardware y no se cambia todos los días, se lo llama firmware.

### ¿Cómo se ejecuta un programa en el ESP32? (arranque)
Cuando le das energía: un pequeño programa grabado de fábrica (**bootloader de ROM**) arranca, carga
nuestro **bootloader** (que está en la flash), y este a su vez carga **nuestra aplicación**. En ese punto
arranca el sistema operativo (FreeRTOS) y finalmente se llama a nuestra función `app_main()`.

### ¿Qué necesitamos para compilar y grabar? 
Necesitás el **ESP-IDF** (Espressif IoT Development Framework), que es el kit oficial de desarrollo del
ESP32. Trae **todo**:
1. El **compilador** cruzado `xtensa-esp32-elf-gcc` (traduce nuestro C al lenguaje máquina del ESP32).
2. El **sistema de compilación** (CMake + Ninja) que orquesta todo.
3. El **grabador** `esptool`, que manda el programa por el cable USB a la flash del chip.
4. **FreeRTOS y los drivers** ya incluidos como librerías.

El flujo práctico es: `idf.py build` (compila) y `idf.py flash` (graba por USB). El cable USB tiene un
chip que convierte USB ↔ puerto serie, y el ESP32 tiene un modo especial de arranque que permite que
`esptool` le escriba la flash por ese puerto serie.

### ¿FreeRTOS "viene dentro de la ESP32"?
**No como hardware.** El chip ESP32 **no trae FreeRTOS grabado de fábrica**. FreeRTOS es **parte del
ESP-IDF** (viene como código fuente/librería), y **se compila junto con tu programa** dentro del firmware.
Cuando grabás el firmware, adentro va tu código **más** FreeRTOS. Al arrancar, ese FreeRTOS incluido toma
el control y ejecuta tus tareas. O sea: no es del chip, es del SDK, y termina adentro de tu `.bin`.

## II.3 ¿Qué es un RTOS? Y ❓ Tu pregunta 1 (µC/OS vs FreeRTOS)

### El problema que resuelve un sistema operativo
Nuestro programa tiene que hacer **varias cosas a la vez**: leer el sensor cada 20 ms, mover servos,
imprimir por consola cada 200 ms, atender el botón. Si lo escribiéramos como un único `while(true)`
gigante haciendo todo en orden, sería un lío frágil y difícil de temporizar.

Un **sistema operativo (SO)** resuelve esto dejándote escribir cada actividad por separado (una **tarea**)
y encargándose él de **repartir el tiempo del procesador** entre ellas.

### ¿Qué lo hace "de tiempo real" (RTOS)?
Un SO común (Windows, Linux de escritorio) reparte el CPU "de forma justa" entre todos los programas: nadie
tiene garantías estrictas de *cuándo* le toca. Un **RTOS** (Real-Time Operating System) hace lo contrario:
te deja fijar **prioridades** y **garantiza** que la tarea más importante que esté lista corre **ya**,
desalojando a las menos importantes. Esa previsibilidad es lo que necesita el tiempo real.

> Definición de la diapo `12-rtos`: *un RTOS gestiona las tareas y recursos de un sistema cuya operación
> correcta depende tanto del resultado como del tiempo en que se produce.*

### > ❓ Tu pregunta 1: ¿Vimos µC/OS? ¿Diferencias con FreeRTOS?

**Sí.** En la teoría (diapo `12-rtos`) el profe explicó el RTOS usando **µC/OS-III** (se lee "micro C-OS
tres"), de la empresa Micrium. Se ven sus funciones `OSTaskCreate`, `OSTaskQPend` (esperar en una cola),
`OSMemCreate` (memoria), etc. **Pero en los laboratorios y en el proyecto usamos FreeRTOS**, porque es el
RTOS que **viene incluido en el ESP-IDF** del ESP32. No es que uno sea "mejor": son dos implementaciones de
**las mismas ideas** (tareas, prioridades, colas, semáforos, mutex). Cambia sobre todo el **nombre de las
funciones**.

| Aspecto | µC/OS-III (teoría) | FreeRTOS (nuestro proyecto) |
|---|---|---|
| Origen / dueño | Micrium (hoy Silicon Labs) | Mantenido por Amazon (AWS) |
| Licencia | Comercial históricamente; hoy abierto (Apache 2.0) | Abierta (MIT), gratis siempre |
| Nombre de funciones | `OSTaskCreate`, `OSQPost`, `OSSemPost`… | `xTaskCreate`, `xQueueSend`, `xSemaphoreGive`… |
| Crear tarea | `OSTaskCreate(...)` | `xTaskCreate(...)` |
| Cola de mensajes | `OSTaskQPost/Pend` | `xQueueSend/xQueueReceive` |
| Memoria | Bloques de tamaño fijo (`OSMemCreate`) para que sea determinista | Varios esquemas de *heap* (heap_1…heap_5) |
| Dónde corre | Ejemplos sobre Linux/µC/OS | **Incluido en ESP-IDF**, corre en el ESP32 |
| Certificaciones | Fuerte en seguridad crítica (médica, avónica) | Existe variante certificada (SafeRTOS) |

**Cómo contestarlo en el examen (una frase):** *"En teoría vimos el concepto de RTOS con µC/OS-III; en el
proyecto usamos FreeRTOS porque es el que trae el ESP-IDF del ESP32. Son la misma familia de conceptos
—tareas, prioridades, colas, mutex— con distinta API. Nosotros aplicamos exactamente esas ideas."*

## II.4 Tareas, prioridades y el planificador

### ¿Qué es una tarea (task)?
Una **tarea** es una función que corre "en paralelo" con las demás, cada una en su propio bucle infinito.
En FreeRTOS es lo que en la teoría POSIX se llamaba un **hilo (thread)**: una **unidad de ejecución**. Todas
las tareas comparten la misma memoria del programa (importante, porque de ahí surge la necesidad de
sincronizar, más abajo).

En nuestro proyecto hay **tres tareas**:
- `task_sensor`: lee el sensor y filtra, 50 veces por segundo.
- `task_actuator`: recibe el ángulo y mueve los servos.
- `task_monitor`: imprime por consola cada 200 ms.

### ¿Qué es el planificador (scheduler) y la "prioridad"?
Como el ESP32 tiene pocos núcleos y varias tareas, alguien tiene que decidir **cuál corre en cada momento**.
Ese árbitro es el **planificador** de FreeRTOS. Su regla en un RTOS **preemptivo** es simple y estricta:

> **Siempre corre la tarea de mayor prioridad que esté lista para ejecutarse.** Si mientras corre una tarea
> de baja prioridad se despierta una de mayor prioridad, el planificador **interrumpe** (desaloja, en inglés
> *preempt*) a la de baja y le da el CPU a la de alta **de inmediato**.

Analogía: un gerente con empleados. Si entra un pedido urgente (tarea de alta prioridad), el gerente le
saca la máquina al empleado que hacía algo secundario y se la da al urgente.

En nuestro proyecto las prioridades son (número más alto = más importante):

| Tarea | Prioridad | Por qué esa prioridad |
|---|:---:|---|
| `task_actuator` | **7 (alta)** | Es la que **cierra el lazo** (mueve los servos). Es lo más crítico en el tiempo: apenas hay un dato nuevo, tiene que corregir ya. |
| `task_sensor` | **5 (media)** | Produce los datos. Importante, pero si el actuador está corrigiendo, que corrija primero. |
| `task_monitor` | **3 (baja)** | Solo imprime para que miremos. Puede esperar; **nunca** debe estorbar al control. |

> **Decisión de diseño clave:** el consumidor (actuador) tiene **más** prioridad que el productor (sensor).
> Esto asegura que la corrección ocurre lo antes posible después de cada medición.

### ¿Cómo se garantiza que el sensor corra "exactamente" cada 20 ms?
Con una función especial: **`vTaskDelayUntil`**. La diferencia con la típica `vTaskDelay` (que espera "20 ms
desde ahora") es que `vTaskDelayUntil` despierta la tarea en **instantes absolutos** (20, 40, 60, 80 ms…),
sin importar cuánto tardó el trabajo. Así el muestreo es **exactamente 50 Hz** sin acumular retrasos. Es
una herramienta de tiempo real: garantiza **periodicidad**.

*(Tareas/prioridades: diapos `05-hilos`, `12-rtos` y `FreeRTOS.pdf`. `vTaskDelayUntil`: `FreeRTOS.pdf`.)*

## II.5 Interrupciones e ISR — ❓ Tu pregunta 7

> **❓ Tu pregunta 7:** *El botón genera una ISR. ¿Cómo funciona esta interrupción? ¿Es una interrupción de
> FreeRTOS? Cuando presionamos el botón, ¿hay que tener en cuenta el efecto rebote? ¿Cómo lo manejamos?*

### ¿Qué es una interrupción?
Normalmente el CPU ejecuta tu programa línea por línea. Una **interrupción** es un mecanismo del **hardware**
que le dice al CPU: *"pará todo lo que estás haciendo AHORA y atendé esto urgente"*. El CPU guarda dónde
estaba, salta a ejecutar una función especial (la **ISR**, *Interrupt Service Routine* = rutina de servicio
de interrupción), y cuando esta termina, vuelve exactamente a donde estaba.

Analogía: estás cocinando (tu programa) y suena el **timbre** (la interrupción). Dejás todo, atendés la
puerta (la ISR), y volvés a cocinar donde ibas.

**¿Por qué usar una interrupción para el botón y no "preguntar" todo el tiempo si está apretado?** Porque
preguntar constantemente (*polling*) gasta CPU y puede perder el evento si justo estabas ocupado. La
interrupción es **inmediata y no gasta nada** hasta que ocurre. Por eso el anteproyecto pedía que el botón
tuviera "máxima prioridad": una interrupción de hardware desaloja a **cualquier** tarea.

### ¿Es una interrupción "de FreeRTOS"?
**No.** La interrupción es del **hardware del ESP32** (el periférico de GPIO detecta el cambio en el pin y
fuerza el salto). FreeRTOS **no** genera la interrupción. Lo único que FreeRTOS aporta es un conjunto de
funciones especiales (terminadas en `...FromISR`) por si la ISR necesita comunicarse con una tarea. En
nuestro caso, la ISR **ni siquiera usa FreeRTOS**: solo levanta una bandera y lee el reloj. Es una ISR de
hardware pura. La registramos con dos funciones del ESP-IDF: `gpio_install_isr_service()` y
`gpio_isr_handler_add()`.

### La regla de oro: la ISR debe ser CORTÍSIMA
Mientras corre una ISR, **el resto del sistema está frenado**. Por eso una ISR **no debe** hacer trabajo
pesado, ni imprimir, ni esperar. Nuestra ISR hace lo mínimo: anota "me apretaron el botón" en una bandera
(`zero_reset_requested = true`) y termina en microsegundos. **El trabajo real** (fijar el nuevo punto cero)
lo hace después, tranquila, la `task_sensor`. Este patrón se llama **"trabajo diferido"** (deferred
processing): la ISR **avisa**, la tarea **hace**.

También la marcamos con `IRAM_ATTR`, que la coloca en la RAM interna (en vez de en la flash). Así siempre
está disponible al instante, con latencia mínima y predecible (no depende de la caché de la flash).

### > El efecto rebote (debounce)
Los botones físicos, al apretarse, **no** hacen un contacto limpio: los metales "rebotan" durante unos
milisegundos y generan **muchos pulsos eléctricos** por una sola pulsación. Si no lo tenemos en cuenta, un
solo toque dispararía la ISR varias veces.

**Cómo lo manejamos (antirrebote por software):** guardamos el instante del último toque aceptado y, si el
nuevo llega antes de **300 ms** después, lo **ignoramos**. Usamos `esp_timer_get_time()`, que da el tiempo
en microsegundos desde el arranque (se puede llamar desde una ISR porque no involucra al planificador):

```c
int64_t now = esp_timer_get_time();
if ((now - last_isr_time_us) >= BUTTON_DEBOUNCE_US) {   // 300 ms
    last_isr_time_us = now;
    zero_reset_requested = true;   // recién acá aceptamos el toque
}
```

*(Concepto de ISR + debounce + patrón "ISR entrega evento a tarea": laboratorio `U3_lab3`.)*

## II.6 Sincronización y comunicación entre tareas — ❓ Tu pregunta 4

Como **todas las tareas comparten la misma memoria**, si dos tocan la misma variable al mismo tiempo se
pueden pisar y corromper los datos. Esto se llama **condición de carrera** (*race condition*). Para evitarlo
hay tres herramientas en el proyecto. Las explico por lo que resuelven.

### II.6.a — Sección crítica y spinlock (proteger algo por un instante)
Una **sección crítica** es un tramo de código donde queremos que **nadie más** toque ciertas variables. En
el ESP32 (que tiene dos núcleos) se protege con un **spinlock** (`portMUX`). "Spin" = el otro que quiera
entrar **gira esperando activamente** (busy-wait) hasta que se libera. Sirve solo para tramos **ultra
cortos** (leer/escribir un par de variables), y es lo único que se puede usar **también desde una ISR**
(porque una ISR no puede "dormir").

Lo usamos para las variables que comparten la **ISR** y la `task_sensor` (la bandera del botón y el último
ángulo):
```c
portENTER_CRITICAL_ISR(&isr_mux);   // desde la ISR
zero_reset_requested = true;
portEXIT_CRITICAL_ISR(&isr_mux);
```

### II.6.b — Mutex (proteger un recurso entre tareas)
Un **mutex** (de *MUTual EXclusion*, exclusión mutua) es como la **llave de un baño**: solo una tarea puede
tener la llave a la vez; las demás **esperan** (durmiendo, sin gastar CPU) hasta que se libere. A diferencia
del spinlock, el que espera **duerme** (no gira). Se usa para recursos que se tocan por un rato y solo entre
tareas (no ISR).

Lo usamos para el **punto cero** (`zero_roll`, `zero_pitch`), que son dos números que la `task_sensor`
**escribe** y el `task_actuator`/`task_monitor` **leen**. Como son dos floats, leerlos "a medias" (uno
actualizado y el otro no) daría un valor inconsistente; el mutex garantiza que se lean/escriban juntos.
```c
if (xSemaphoreTake(zero_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {  // pedir la llave (esperar máx 10 ms)
    z_roll = zero_roll;                                          // sección protegida
    xSemaphoreGive(zero_mutex);                                  // devolver la llave
}
```

> **¿Mutex o spinlock?** Spinlock = tramo cortísimo que **también toca la ISR** (no puede dormir). Mutex =
> recurso entre **tareas** que sí pueden esperar durmiendo. Por eso usamos cada uno donde corresponde.

### II.6.c — Cola (queue): pasar datos de una tarea a otra

> **❓ Tu pregunta 4:** *Una tarea productora publica el ángulo en una cola y otra lo consume. ¿Dónde se ve
> esto? ¿Ventajas? ¿Desventajas? ¿Otras opciones? ¿Puede haber datos no consumidos? ¿En las colas siempre
> se manejan datos estructurados?*

Una **cola** es una **cinta transportadora** (o un buzón) entre dos tareas: una **deja** datos (productor) y
la otra los **retira** (consumidor), en orden (el primero que entra es el primero que sale, FIFO). Es el
mecanismo de comunicación **seguro** por excelencia entre tareas, porque **copia** el dato: el productor no
comparte una variable con el consumidor, le **entrega una copia**, así no se pisan.

**Dónde se ve en el código** (`tasks.c`):
- Se crea con capacidad 5: `imu_queue = xQueueCreate(5, sizeof(imu_data_t));`
- El productor `task_sensor` deja el ángulo: `xQueueSend(imu_queue, &data, 0);`
- El consumidor `task_actuator` lo retira: `xQueueReceive(imu_queue, &data, ...);`

Esto es el **patrón productor-consumidor** clásico, idéntico al laboratorio `U3_lab4`.

**Ventajas:**
- **Desacople:** productor y consumidor corren a su ritmo; no dependen uno del otro instante a instante.
- **Seguridad automática:** la cola ya está protegida internamente; no tenés que poner mutex a mano.
- **Sin espera activa:** el consumidor **se bloquea** (duerme) en `xQueueReceive` hasta que hay dato; no
  malgasta CPU preguntando. Se despierta justo cuando llega algo. (En nuestro caso, la cola es el "reloj"
  del actuador.)
- **Amortigua baches:** si el consumidor se demora un instante, se acumulan hasta 5 muestras sin perder nada.

**Desventajas:**
- **Copia datos:** cada envío copia el struct (para structs chicos como el nuestro, insignificante; para
  datos grandes, se preferiría enviar un puntero).
- **Capacidad fija:** si se llena (consumidor más lento que productor), hay que decidir qué hacer (esperar o
  descartar).
- **Un pelín de latencia y memoria** frente a compartir una variable directamente.

**Otras opciones que existían (y por qué la cola es la adecuada acá):**
- **Variable global + mutex:** funciona, pero el consumidor tendría que "preguntar" cada tanto (polling) o
  necesitarías otro mecanismo para avisarle que hay dato nuevo. Más frágil.
- **Notificaciones directas a tarea (task notifications):** más livianas y rápidas, pero llevan solo un
  número/bits, no un struct cómodo. Buenas para "avisar", no tanto para "pasar datos".
- **Stream/Message buffers:** para flujos de bytes; acá no aplica.

**¿Puede haber datos no consumidos?** **Sí, en dos sentidos:**
1. Si la cola se **llena** (el actuador se atrasó), el `xQueueSend(..., 0)` con espera **cero** devuelve
   error y **descartamos esa muestra** (con un warning). Es una decisión de diseño: el sensor **no puede
   frenarse** esperando, porque debe mantener los 50 Hz. Perder una muestra vieja es preferible a romper el
   ritmo.
2. En el struct viaja también `pitch`, pero el actuador **solo usa roll** (controlamos un eje). Ese `pitch`
   viaja "de acompañante" y no se consume para actuar (sí se reporta por otro camino en el monitor).

**¿Las colas siempre manejan datos estructurados?** **No.** Una cola transporta elementos de un **tamaño
fijo** cualquiera: puede ser un `int`, un `float`, un puntero, o —como acá— un **struct**. Usamos un struct
(`imu_data_t { float roll; float pitch; }`) simplemente porque queremos mandar **dos valores juntos** como
una sola unidad.

*(Colas: `U3_lab4` y diapo `12-rtos`. Mutex/spinlock: diapo `08-sinc_hilos`.)*

## II.7 Señales, sensores y muestreo (base para entender el I2C y los filtros)

Antes de meternos con el sensor conviene una base mínima (diapo `02-señales`):

- **Señal:** una magnitud que varía y lleva información (la inclinación del avión es una señal).
- **Sensor:** convierte una magnitud física en una señal eléctrica (el MPU6050 convierte movimiento en
  números).
- **Señal analógica vs digital:** el mundo real es **analógico** (varía de forma continua). Las computadoras
  trabajan en **digital** (números discretos). Por eso todo sensor internamente **muestrea** (toma fotos de
  la señal cada cierto tiempo) y **cuantiza** (redondea cada foto a un número de bits).
- **Frecuencia de muestreo:** cuántas "fotos" por segundo tomamos. Se mide en **Hz** (muestras por segundo).
  Nosotros muestreamos a **50 Hz** (una foto cada 20 ms).
- **Teorema del muestreo (Nyquist):** para captar bien una señal hay que muestrear a **más del doble** de la
  frecuencia más alta que querés medir; si no, aparecen errores (*aliasing*). Como el avión no oscila más
  rápido que unas pocas veces por segundo, 50 Hz sobra.

## II.8 Filtros: de la media móvil al filtro complementario — ❓ Tu pregunta 3

> **❓ Tu pregunta 3:** *¿Cómo funciona el filtro complementario exactamente? ¿Por qué fusionar giroscopio y
> acelerómetro mejora nuestros datos?*

### ¿Qué es un filtro y por qué lo necesitamos?
Un **filtro** deja pasar lo que querés y elimina lo que no. Acá el problema es que las mediciones del sensor
vienen **sucias** (la vibración del motor las contamina) y necesitamos un ángulo **limpio y estable** para
controlar bien. El filtro es lo que las limpia.

### Los dos sensores y sus defectos (la clave de todo)
El MPU6050 tiene dos sensores que miden la inclinación de **maneras distintas**, y **cada uno tiene un
defecto opuesto**:

| Sensor | Cómo da el ángulo | Bueno en | Malo en |
|---|---|---|---|
| **Acelerómetro** | Mide dónde "tira" la gravedad y con trigonometría deduce la inclinación | **Largo plazo:** siempre apunta a la vertical real, **no se desvía nunca** | **Corto plazo:** cualquier vibración o sacudón lo **ensucia** (mucho ruido) |
| **Giroscopio** | Mide la **velocidad** de giro; sumándola en el tiempo (integrando) deduce cuánto rotó | **Corto plazo:** suave, rápido, **ignora la vibración** | **Largo plazo:** acumula pequeños errores → el ángulo **se va desviando** (*drift*) |

Fijate que son **complementarios**: donde uno falla, el otro anda bien. El acelerómetro es confiable en el
largo plazo pero ruidoso; el giroscopio es confiable en el corto plazo pero deriva. **La idea genial es
combinarlos** para quedarnos con lo bueno de cada uno. Eso es la **fusión de sensores**.

### El filtro complementario, línea por línea
La fórmula que usamos (en `task_sensor`) es:

```c
roll = COMP_ALPHA * (roll + gyro_x * dt) + (1.0f - COMP_ALPHA) * accel_roll;
//     └──────── parte del GIROSCOPIO ────┘   └──── parte del ACELERÓMETRO ────┘
//          con COMP_ALPHA = 0.98            dt = 0.02 s (20 ms)
```

Traducida a palabras, cada 20 ms el nuevo ángulo se calcula así:
1. `roll + gyro_x * dt` → tomá el **ángulo anterior** y sumale **cuánto rotó** en estos 20 ms según el
   giroscopio (velocidad × tiempo = ángulo). Esto es rápido y suave, pero solito derivaría.
2. Multiplicá eso por **0.98** (le damos 98% de confianza al giroscopio para el corto plazo).
3. `accel_roll` → calculá el ángulo **absoluto** que dice la gravedad (acelerómetro). Esto es ruidoso, pero
   no deriva.
4. Multiplicá eso por **0.02** (solo 2%) y sumalo.

El 2% del acelerómetro actúa como un **ancla suave**: en cada ciclo "corrige un poquito" el ángulo hacia la
vertical real, **cancelando el drift** del giroscopio, pero como es solo 2%, la vibración (que aparece en el
acelerómetro) casi no se cuela. Resultado: un ángulo **rápido, suave y sin desvío**.

### ¿Por qué mejora los datos? (la intuición en frecuencias)
Otra forma de verlo: el filtro **pasa-altos** al giroscopio (le quita el drift, que es un error "lento") y
**pasa-bajos** al acelerómetro (le quita la vibración, que es un ruido "rápido"), y **suma** ambos. Como las
dos mitades se complementan y suman 1 (0.98 + 0.02), no perdés ni agregás ganancia. De ahí el nombre
**"complementario"**.

Hay incluso un número que marca la frontera: la **constante de tiempo** ≈ `α·dt/(1-α)` = `0.98·0.02/0.02` ≈
**1 segundo**. Significa: *para cambios más rápidos que ~1 s manda el giroscopio (suaviza); para cambios más
lentos que ~1 s manda el acelerómetro (corrige el rumbo)*. Justo lo que queremos.

### ¿Por qué NO usamos la media móvil (que era lo del anteproyecto y la clase)?
La **media móvil** promedia las últimas N muestras de **un solo** sensor para suavizar el ruido. Problemas:
usa un solo sensor (desperdicia el otro), **no arregla el drift**, e introduce **retardo** (cuanto más
suaviza, más tarde reacciona) — malísimo para tiempo real. El complementario **fusiona los dos sensores**,
no tiene ese retardo, y elimina el drift. Es, de hecho, la versión **simple y barata del filtro de Kalman**
(el "de verdad", pero mucho más costoso de calcular).

> ⚠️ **Ojo, punto importante para el examen:** el filtro complementario **no aparece en el material de
> clase** (ahí se vio media móvil / FIR). Es el único concepto central "de más". Prepará esta respuesta:
> *"Cambiamos media móvil por filtro complementario porque necesitábamos fusionar giroscopio y acelerómetro
> para tener un ángulo sin drift y sin retardo, algo imposible con una media móvil sola."*

*(Filtros digitales, FIR/IIR, media móvil: diapo `13-filtros_digitales`, lab `U3_lab1`.)*

## II.9 Comunicación por buses: I2C, puerto serie (UART) y PWM

El ESP32 se comunica con las otras piezas por distintos "idiomas" (protocolos). Explico los tres.

### II.9.a I2C — cómo le hablamos al sensor — ❓ Tu pregunta 2

> **❓ Tu pregunta 2:** *¿Qué es I2C? ¿Cómo funciona? ¿Cómo sabés que es a 50 Hz? ¿Dónde se configura? ¿Y si
> subo ese valor? ¿Hay que corregir la gravedad o lo hace el chip? Y no entiendo nada del Paso 2 y 3.*

**¿Qué es I2C?** (se lee "i-dos-ce" o "i-squared-c"). Es un **protocolo de comunicación** que conecta chips
usando **solo dos cables**:
- **SDA** (*Serial Data*): por donde viajan los **datos** (bits, de a uno).
- **SCL** (*Serial Clock*): el **reloj**, un pulso que marca el **ritmo** (cada pulso = "ahora leé un bit").

Analogía: dos personas pasándose un mensaje letra por letra, donde una **golpea la mesa** (SCL) para marcar
"ahora la próxima letra", y la otra **dice la letra** (SDA) en cada golpe.

**Maestro y esclavo:** uno manda (el **maestro** = ESP32) y otro obedece (el **esclavo** = MPU6050). El
maestro **siempre genera el reloj** y decide cuándo se habla. En el mismo par de cables pueden convivir
varios esclavos, y cada uno tiene una **dirección** única de 7 bits (como un número de casa). La del
MPU6050 es **0x68**.

**Velocidad del bus:** 400 kHz. Ese número es **cuán rápido corre el relojito SCL** (400.000 pulsos por
segundo). **¡Ojo, no confundir con los 50 Hz!** Son dos cosas distintas:
- **400 kHz** = velocidad del *cable* I2C (qué tan rápido se transmiten los bits de **una** lectura).
- **50 Hz** = cada cuánto **decidimos nosotros** pedir una lectura nueva (una vez cada 20 ms).

O sea: 50 veces por segundo hacemos una lectura, y **cada** lectura viaja por el cable a 400 kHz (tarda
microsegundos). Se configura en `config.h`: la velocidad con `I2C_MASTER_FREQ_HZ 400000`.

> **¿De dónde salen los 50 Hz y dónde se configuran?** **No** es algo del sensor: **lo decidimos nosotros**.
> En `config.h` está `SENSOR_PERIOD_MS 20` (20 ms). En `task_sensor`, `vTaskDelayUntil(..., 20 ms)` hace que
> el bucle se repita cada 20 ms → 1000/20 = **50 Hz**. El sensor por dentro mide continuamente; nosotros
> le "sacamos una foto" cada 20 ms.

> **¿Y si subo ese valor (muestreo más rápido, p.ej. 100 Hz)?** El sistema reaccionaría más rápido (más
> correcciones por segundo). Se puede, porque cada lectura I2C tarda microsegundos y sobra tiempo dentro de
> los 20/10 ms. **Bonus:** el `dt` del filtro se calcula solo a partir del período (`SENSOR_DT_S =
> SENSOR_PERIOD_MS/1000`), así que la integración del giroscopio se ajusta automáticamente. Límites: más
> frecuencia = más carga de CPU y de bus; y a partir de cierto punto no ganás nada porque el avión y los
> servos no reaccionan tan rápido. Para retocar fino habría que reajustar `COMP_ALPHA`.

> **¿Hay que corregir el efecto de la gravedad, o lo hace el chip?** **Ni una cosa ni la otra: la gravedad
> NO se corrige, se APROVECHA.** El chip **no** la quita. El acelerómetro mide **toda** la aceleración,
> incluida la gravedad, y justamente **usamos la gravedad** para saber hacia dónde es "abajo" y así calcular
> la inclinación (con `atan2`). El "problema" no es la gravedad, sino las **otras** aceleraciones (vibración,
> sacudones) que se suman y ensucian la lectura — y eso lo maneja el **filtro complementario** (apoyándose
> en el giroscopio para el corto plazo). Resumen: gravedad = amiga (nos da la referencia); vibración =
> enemiga (la filtra el complementario).

**Ahora sí, el Paso 2 (la transacción I2C) explicado símbolo por símbolo:**
```
START → dirección 0x68+W → registro 0x3B → RESTART → 0x68+R → 14 bytes → STOP
```
- **START:** el maestro "levanta la mano" y dice "voy a hablar" (una condición eléctrica especial en los
  cables). Abre la conversación.
- **dirección 0x68+W:** el maestro grita "¡quiero hablar con el chip 0x68, para ESCRIBIRLE!" (W = write).
  Solo el MPU6050 responde "presente" (manda un **ACK**, un "ok").
- **registro 0x3B:** el maestro le dice "posicioná tu puntero en tu casillero 0x3B" (ese casillero es
  `ACCEL_XOUT_H`, donde empiezan los datos de medición). Un sensor por dentro es como una **planilla de
  casilleros (registros)**; le indicamos desde cuál queremos leer.
- **RESTART:** en vez de cerrar y volver a abrir, el maestro hace un "reinicio en caliente" para cambiar de
  modo escritura a modo lectura sin soltar el bus.
- **0x68+R:** ahora "¡quiero hablar con 0x68 para LEER!" (R = read).
- **14 bytes:** el maestro va marcando el reloj y el sensor le va **entregando 14 bytes** seguidos (el chip
  avanza solo su puntero de casillero: 0x3B, 0x3C, 0x3D…). Esos 14 bytes son todas las mediciones.
- **STOP:** el maestro "baja la mano" y cierra la conversación.

**¿Por qué 14 bytes?** Porque el sensor guarda cada medición en **2 bytes** (16 bits) y son 7 mediciones
seguidas: aceleración X, Y, Z (6 bytes) + temperatura (2 bytes, que ignoramos) + giro X, Y, Z (6 bytes) =
**14 bytes**. Los leemos todos de una sola transacción porque están en casilleros consecutivos.

**El Paso 3 (reconstrucción y escalado) explicado:**
Los 14 bytes son "crudos": bytes sueltos. Hay que convertirlos en números útiles. Dos sub-pasos:

1. **Unir 2 bytes en un número de 16 bits.** Cada medición llegó partida en dos bytes: uno "alto" (los 8
   bits de más peso) y uno "bajo". El sensor los manda en orden **big-endian** = el byte **alto primero**.
   Los reunimos así:
   ```c
   int16_t acc_x = (int16_t)((raw[0] << 8) | raw[1]);
   ```
   - `raw[0] << 8` corre el byte alto 8 posiciones a la izquierda (lo pone en la mitad de arriba del número).
   - `| raw[1]` le "encastra" el byte bajo en la mitad de abajo.
   - El `(int16_t)` es **clave**: le dice a C que interprete esos 16 bits como un número **con signo**
     (puede ser negativo). Sin eso, una inclinación negativa se leería como un número gigante positivo.
     (Los sensores usan "complemento a dos", la forma estándar de representar negativos en binario.)

2. **Escalar a unidades físicas.** Ese entero todavía no está en "g" ni en "grados/seg": es un número
   interno del chip. El datasheet dice por cuánto dividir según el rango elegido:
   ```c
   out->acc_x  = acc_x / 16384.0f;   // rango ±2g   → resultado en g (gravedades)
   out->gyro_x = gyr_x / 131.0f;     // rango ±250  → resultado en °/s (grados por segundo)
   ```
   ¿De dónde salen 16384 y 131? Un entero de 16 bits con signo va de −32768 a +32767. Si configuramos el
   acelerómetro en rango ±2g, entonces +2g corresponde a ~32767, así que **1g = 32767/2 ≈ 16384**. Igual el
   giroscopio: rango ±250°/s → 32767/250 ≈ **131** por cada °/s. Por eso dividimos por esos números:
   convierten "cuentas internas" en unidades reales.

Después de esto, tenemos la lectura en **5 números con sentido físico**: `acc_x, acc_y, acc_z` (en g) y
`gyro_x, gyro_y` (en °/s). Listos para calcular ángulos (Parte V, Paso 4).

*(I2C: laboratorio `I2C_intro`. Datasheet: `MPU-6000.PDF`.)*

### II.9.b Puerto serie / UART — cómo vemos los datos — ❓ Tu pregunta 8

> **❓ Tu pregunta 8:** *La tarea monitor imprime por puerto serie cada 200 ms. ¿Por qué 200? ¿Qué es un
> puerto serie? ¿Cómo lo manejamos? ¿Tiene que ver con UART? ¿Qué es?*

**¿Qué es un "puerto serie"?** Es una forma de mandar datos entre dos aparatos **bit por bit, de a uno, en
fila** (por eso "serie", en serie uno tras otro), por un solo cable de datos. Es el modo más simple y viejo
de comunicación entre una computadora y un dispositivo.

**¿Qué es UART?** *Universal Asynchronous Receiver/Transmitter*. Es el **bloque de hardware** dentro del
ESP32 que implementa el puerto serie: agarra un byte y lo va "sacando" bit por bit por el pin de transmisión
(TX), y recibe bits por el pin de recepción (RX) rearmándolos en bytes. La "A" de **asíncrono** significa que
**no hay cable de reloj** (a diferencia del I2C): en su lugar, **ambos lados acuerdan de antemano la
velocidad**, llamada **baud rate**. Nosotros usamos **115200 baudios** (≈115200 bits por segundo). Si los
dos lados no coinciden en esa velocidad, se ve basura.

**¿Cómo llega a tu PC?** El cable USB del ESP32 tiene un chip que convierte **USB ↔ serie**. Tu PC entonces
"ve" un puerto serie (en Linux aparece como `/dev/ttyUSB0`). Con el comando `idf.py monitor` abrís ese
puerto y ves en la terminal lo que el ESP32 escribe.

**¿Cómo lo manejamos en el código?** No manejamos el UART "a mano": usamos la función de log del ESP-IDF
**`ESP_LOGI(...)`** (I = Info). Esa función formatea el texto y lo manda al UART0 (que es el puerto serie
de consola). Ejemplo de lo que imprime la tarea monitor:
```
I (1234) MONITOR: [VUELO] Roll: +3.4° | Pitch: -1.2° | Zero(R/P): 0.0°/0.0°
```
(La `I` = Info, `1234` = milisegundos desde el arranque, `MONITOR` = la etiqueta de la tarea.)

> **¿Por qué imprime cada 200 ms y no más seguido?** Por tres razones:
> 1. **Para vos (humano):** 5 líneas por segundo ya es cómodo de leer. A 50 Hz (cada 20 ms) sería una
>    catarata ilegible de texto.
> 2. **Para no saturar:** imprimir mucho gasta CPU y llena el cable serie con datos que nadie necesita tan
>    seguido.
> 3. **Para no molestar al control:** la tarea monitor tiene la **prioridad más baja (3)**. Corre en los
>    ratos libres. Aunque imprimir tarde, **nunca** le quita tiempo al sensor ni al actuador. Imprimir es lo
>    menos urgente del sistema.

*(Actuadores/señales de control y noción de comunicación: diapo `02-señales`. Puerto serie: se usa en todos
los labs de ESP32 para ver la salida.)*

### II.9.c PWM y MCPWM — cómo movemos los servos — ❓ Tu pregunta 5

> **❓ Tu pregunta 5:** *¿Qué es MCPWM? ¿Cómo funciona? ¿Los servos giran para el mismo lado o están
> invertidos? ¿Qué hay que hacer para moverlos? ¿Qué dificultades y consideraciones hay?*

**¿Qué es PWM?** *Pulse Width Modulation* (modulación por ancho de pulso). Es una técnica para "comunicar un
valor" prendiendo y apagando una señal muy rápido. Lo que importa es **cuánto tiempo está prendida** dentro
de cada ciclo. Un servo SG90 se controla así:
- La señal se repite cada **20 ms** (una frecuencia de **50 Hz**).
- Dentro de esos 20 ms, el **ancho del pulso en alto** define la posición del brazo del servo:
  - **1000 µs (1 ms) en alto → ≈ −90°**
  - **1500 µs (1.5 ms) → 0° (centro)**
  - **2000 µs (2 ms) → ≈ +90°**

O sea: le "dibujamos" un pulso de cierto ancho y el servo mueve su brazo al ángulo correspondiente. Es como
un idioma: "pulso de 1500 µs" significa "andá al centro".

**¿Qué es MCPWM?** El ESP32 tiene un **periférico de hardware** dedicado a generar PWM para motores, llamado
**MCPWM** (*Motor Control PWM*). Lo elegimos en vez del otro generador de PWM del ESP32 (`ledc`, pensado para
LEDs) porque MCPWM permite fijar el ancho del pulso **en microsegundos con precisión de 1 µs**, justo lo que
los servos necesitan. **Lo importante:** una vez configurado, MCPWM genera la señal **solo, por hardware**,
sin gastar CPU. El procesador solo interviene para **cambiar el ancho** cuando quiere mover el servo.

**¿Cómo funciona por dentro?** MCPWM se arma como una cadena de 4 bloques (por cada servo):
```
Timer → Operator → Comparator → Generator → pin GPIO
```
- **Timer:** un contador que va de 0 a 20000 y vuelve a empezar. Como cuenta a 1 MHz (1 tick = 1 µs), llegar
  a 20000 toma 20000 µs = **20 ms**. Ese ciclo es la frecuencia de 50 Hz.
- **Comparator (comparador):** guarda un número, p. ej. **1500**. **Ese número ES el ancho del pulso en µs.**
- **Generator (generador):** controla el pin. Su regla: cuando el timer está en **0**, pone el pin en
  **ALTO**; cuando el timer **llega al número del comparador (1500)**, pone el pin en **BAJO**. Resultado:
  el pin está en alto durante **1500 µs** y en bajo el resto → ¡un pulso de 1500 µs cada 20 ms!

**¿Qué hay que hacer para mover el servo?** Simplemente **cambiar el número del comparador**. Nada más:
```c
void servos_set_us(uint32_t s1, uint32_t s2) {
    mcpwm_comparator_set_compare_value(s_cmp1, s1);   // p.ej. 1550 → servo 1 se corre un poco
    mcpwm_comparator_set_compare_value(s_cmp2, s2);
}
```
El timer sigue corriendo; en el próximo ciclo el pulso ya sale con el nuevo ancho. (Usamos una opción,
`update_cmp_on_tez`, para que el cambio se aplique justo al inicio del ciclo y no "a mitad de un pulso",
evitando pulsos deformes.)

> **¿Los dos servos giran para el mismo lado o están invertidos?** **Físicamente giran en sentidos
> OPUESTOS**, que es lo que hace falta para corregir el **alabeo** (roll) de un ala volante: si el avión se
> va para un lado, un alerón sube y el otro baja. **Pero en el código los dos reciben la misma orden con el
> mismo signo** (`SERVO1_DIR = SERVO2_DIR = +1.0`). ¿Por qué funciona? Porque el **servo 2 está montado
> físicamente al revés (espejado)** respecto del servo 1. Entonces, aunque eléctricamente les mandamos lo
> mismo, **mecánicamente se mueven al revés uno del otro**. Los parámetros `SERVO1_DIR`/`SERVO2_DIR` existen
> justamente para poder **invertir el sentido de un servo por software** si el montaje lo requiere, sin tocar
> la lógica.

**Dificultades y consideraciones (buenas para mencionar en el examen):**
1. **Calibración del centro:** un servo real quizás no queda perfecto a 0° con 1500 µs. Por eso hay
   `SERVO1_ZERO_US`/`SERVO2_ZERO_US` ajustables (p. ej. 1480 o 1520) para centrarlo sin desarmar nada.
2. **Sentido de montaje (espejo):** ya explicado; se resuelve con `SERVOx_DIR`.
3. **Saturación (clamping):** nunca hay que mandar un pulso fuera de 1000–2000 µs, o el servo fuerza contra
   su tope mecánico y se puede dañar o "perder el paso". Por eso el código **recorta** siempre al rango
   seguro antes de mandar.
4. **Alimentación:** los servos consumen picos de corriente al moverse. Hay que alimentarlos con 5 V y
   **masa (GND) común** con el ESP32; si no, pueden reiniciar el micro (brownout) o moverse errático.
5. **Velocidades distintas:** dos SG90 no son idénticos; uno puede ser un pelín más lento. (De hecho el
   proyecto tenía un modo de prueba para compararlos, que quitamos en la limpieza.)

*(PWM/servos: se vio como concepto en `02-señales`, y en labs `U1_lab52`, `U2_lab1`, `U3_lab1` —aunque ahí
con Raspberry Pi y la librería `pigpio`; la API MCPWM del ESP32 es específica de este proyecto.)*

---
---

# PARTE III — EL HARDWARE PIEZA POR PIEZA

## III.1 ESP32 (el cerebro)
Ver **II.2**. Es el microcontrolador que ejecuta todo. Dual-core 240 MHz, 520 KB RAM, 2 MB flash. Usa sus
periféricos I2C, MCPWM, UART y GPIO.

## III.2 MPU6050 (el sensor de inclinación)
Es una **IMU** (*Inertial Measurement Unit*, unidad de medición inercial) de 6 ejes: un **acelerómetro** de
3 ejes + un **giroscopio** de 3 ejes, en un solo chip. Se comunica por **I2C** (dirección 0x68). Lo
configuramos con 3 escrituras al arrancar (`mpu6050_init`):
- `PWR_MGMT_1 = 0`: lo **despierta** (arranca dormido para ahorrar energía).
- `ACCEL_CONFIG = 0`: rango del acelerómetro **±2g** (el más sensible; suficiente para un avión).
- `GYRO_CONFIG = 0`: rango del giroscopio **±250 °/s**.

De sus 6 ejes usamos: aceleración X/Y/Z y giro X (para roll) e Y (para pitch). El giro Z (guiñada) y la
temperatura se leen pero **se descartan** (no los necesitamos).

## III.3 Servos SG90 ×2 (los músculos)
Ver **II.9.c**. Dos micro-servos que mueven los alerones (en un ala volante se llaman **elevones** porque
hacen de alerón + elevador). Controlados por PWM a 50 Hz, pulsos 1000–2000 µs. Servo 1 = **GPIO16** (alerón
izquierdo), Servo 2 = **GPIO17** (alerón derecho, montado espejado).

## III.4 Botón / sensor táctil (la entrada del usuario)
Un pulsador en **GPIO19** que sirve para **fijar el punto cero** (decir "esta inclinación de ahora es el
nivelado"). Genera una **interrupción** (ver II.5). El anteproyecto lo pensaba como sensor táctil capacitivo
**TTP223**, pero el firmware lo trata eléctricamente como un **botón a masa (GND)**: en reposo el pin está
en ALTO (por el pull-up interno) y al accionarlo cae a BAJO (flanco de bajada). ⚠️ *Conviene verificar en la
maqueta qué es exactamente el módulo físico (ver Parte VI).*

## III.5 Mapa de pines (conexiones)
| Señal | GPIO | Va a |
|---|---|---|
| I2C SDA / SCL | 21 / 22 | MPU6050 (datos / reloj) |
| PWM Servo 1 / 2 | 16 / 17 | SG90 izquierdo / derecho |
| Botón (interrupción) | 19 | Pulsador / táctil |
| GND y 3.3V / 5V | — | Masa común y alimentación |

---
---

# PARTE IV — EL SOFTWARE

## IV.1 Estructura de archivos y guía de lectura

El proyecto son **5 módulos**. En C, cada módulo suele tener dos archivos: un **`.c`** (la implementación, el
código de verdad) y un **`.h`** (el "índice" o **contrato**: solo declara qué funciones existen, para que
otros archivos las usen sin ver el código interno). Separar en módulos hace el código **más fácil de
estudiar**: cada archivo es **un solo tema**.

**Leelos en este orden:**

| Orden | Archivo | Qué mirar | Prioridad |
|:---:|---|---|:---:|
| 1️⃣ | **`config.h`** | El "tablero de control": todos los números ajustables (pines, escalas, ganancia, α, prioridades). Sin lógica. | ⭐⭐⭐ |
| 2️⃣ | **`main.c`** | El arranque: `app_main()` con los 5 pasos de inicialización. ~40 líneas. | ⭐⭐ |
| 3️⃣ | **`mpu6050.c`** (+`.h`) | Cómo se lee el sensor por I2C y se escala. | ⭐⭐ |
| 4️⃣ | **`tasks.c`** (+`.h`) | ⭐ **EL ARCHIVO CLAVE:** las 3 tareas + la ISR + toda la sincronización + el filtro + el control. **Es lo que más te van a preguntar.** | ⭐⭐⭐ |
| 5️⃣ | **`servos.c`** (+`.h`) | Cómo se genera el PWM con MCPWM. | ⭐⭐ |

**Si tenés poquísimo tiempo:** `config.h` + `tasks.c`. Con esos dos entendés el 80%.

**Dentro de `tasks.c`, leé en este orden:** (1) las variables globales de sincronización, (2) `gpio_isr_handler`
(la ISR), (3) `task_sensor`, (4) `task_actuator`, (5) `task_monitor`, (6) `tasks_sync_init`/`tasks_start`.

> **Nota "modo estudio":** esta versión fue limpiada de código que no se ejecutaba (se quitaron `gyro_z` sin
> usar, la rama de control de pitch, y dos modos de prueba de hardware). Así `main.c` quedó en ~40 líneas y el
> actuador sin ramas condicionales, sin afectar nada de la lógica evaluable.

## IV.2 Las 3 tareas + la ISR (resumen)

| Elemento | Prioridad | Cuándo corre | Qué hace |
|---|:---:|---|---|
| `task_sensor` | 5 | cada 20 ms (`vTaskDelayUntil`) | Lee MPU → calcula ángulos → **filtro complementario** → atiende el botón → publica en la cola |
| `task_actuator` | 7 | cuando llega dato a la cola | Recibe ángulo → calcula error vs punto cero → **control P** → mueve servos |
| `task_monitor` | 3 | cada 200 ms | Imprime estado por puerto serie |
| ISR del botón | HW (máx) | al accionar GPIO19 | Levanta la bandera "recalibrar punto cero" (nada más) |

## IV.3 El control proporcional (P)

Cuando el actuador recibe el ángulo, calcula el **error** (cuánto se desvió del nivelado) y aplica una
corrección **proporcional** a ese error:
```c
float error_roll = data.roll - z_roll;                        // desvío respecto al punto cero
float s1 = 1500 + SERVO1_DIR * error_roll * CONTROL_GAIN;      // CONTROL_GAIN = 10 µs por grado
float s2 = 1500 + SERVO2_DIR * error_roll * CONTROL_GAIN;
```
- Si el error es 0 (avión nivelado), los servos quedan en 1500 µs (centro).
- Si el error es 5°, la corrección es 5 × 10 = 50 µs → servo a 1550 µs.
- Es un controlador **"P puro"** (solo Proporcional, sin las partes Integral ni Derivativa de un PID
  completo). Para estabilizar una maqueta es suficiente: cuanto más inclinado, más corrige; a medida que se
  endereza, corrige menos. **Lazo cerrado.**

## IV.4 config.h — los parámetros que podés tocar

Todo se ajusta desde un solo archivo. Los más importantes:

| Parámetro | Valor | Qué controla |
|---|---|---|
| `SENSOR_PERIOD_MS` | 20 | Período de muestreo (→ 50 Hz) |
| `COMP_ALPHA` | 0.98 | Peso del giroscopio en el filtro (0.98 = 98%) |
| `CONTROL_GAIN` | 10.0 | Cuántos µs corrige por grado de error |
| `SERVOx_ZERO_US` | 1500 | Centro de cada servo (calibración física) |
| `SERVOx_DIR` | +1.0 | Sentido de cada servo (invertir con −1.0) |
| `PRIO_*` | 5/7/3 | Prioridades de las tareas |
| `BUTTON_DEBOUNCE_US` | 300000 | Antirrebote del botón (300 ms) |
| `I2C_MASTER_FREQ_HZ` | 400000 | Velocidad del bus I2C |

---
---

# PARTE V — RECORRIDO PASO A PASO (de la vibración al servo)

Ahora seguimos **un dato** en su viaje completo, reexplicado con todo el contexto de las partes anteriores.
Es el mismo flujo que se repite **50 veces por segundo**. En cada paso digo **qué pasa** y **qué viaja** al
siguiente.

### 🔵 El lazo de control (el corazón, 50 veces por segundo)

**Paso 1 — El avión se inclina.** Viento, vibración del motor o una mano lo mueven. El **MPU6050**, pegado
al avión, siente dos cosas a la vez: la **aceleración** (que incluye la gravedad) y la **velocidad de giro**.
*Contexto:* ver II.7 (señales) y III.2 (sensor).

**Paso 2 — `task_sensor` lee el sensor por I2C** (cada 20 ms, porque tiene `vTaskDelayUntil` a 20 ms).
Llama a `mpu6050_read()`, que hace la transacción I2C `START → 0x68+W → 0x3B → RESTART → 0x68+R → 14 bytes →
STOP`. *(Si no entendés esta línea, está explicada símbolo por símbolo en **II.9.a**.)* El ESP32 es el
maestro, marca el reloj por SCL y recibe los datos por SDA.
→ **Viaja:** 14 bytes crudos (`uint8_t raw[14]`).

**Paso 3 — Reconstrucción y escalado** (dentro de `mpu6050_read`). Los 14 bytes se juntan de a dos para
formar 7 números de 16 bits con signo, y se dividen por los factores del datasheet (16384 para g, 131 para
°/s). *(Detalle completo en **II.9.a**, "el Paso 3 explicado".)*
→ **Viaja:** un `mpu6050_reading_t` = **5 números físicos**: `acc_x/y/z` (en g) y `gyro_x/y` (en °/s).

**Paso 4 — Calcular dos versiones del ángulo** (en `task_sensor`):
- **Por el acelerómetro** (usando la gravedad): `accel_roll = atan2(acc_y, acc_z)` → da grados. `atan2` es
  una función trigonométrica que, dados los dos componentes del vector gravedad, devuelve el ángulo de
  inclinación (funciona en todos los cuadrantes, con signo). Es **exacta pero ruidosa**.
- **Por el giroscopio** (integrando): tomar el ángulo anterior y sumarle `gyro_x · dt` (velocidad × tiempo).
  Es **suave pero deriva**.
→ **Viaja:** dos estimaciones del mismo ángulo, con defectos opuestos. *(Ver II.8.)*

**Paso 5 — Filtro complementario** (el "cerebro"). Fusiona las dos estimaciones:
```c
roll = 0.98*(roll + gyro_x*0.02) + 0.02*accel_roll;
```
98% giroscopio (rápido, ignora vibración) + 2% acelerómetro (ancla que borra el drift). *(Explicado a fondo
en II.8.)*
→ **Viaja:** el **ángulo limpio** `roll` (y `pitch`), en grados.

**Paso 6 — Publicar en la cola** (en `task_sensor`):
```c
imu_data_t data = { .roll = roll, .pitch = pitch };
xQueueSend(imu_queue, &data, 0);   // el "0" = no esperar; si está llena, descarta la muestra
```
La cola desacopla al sensor del actuador. *(Ver II.6.c.)*
→ **Viaja:** una **copia** del struct por dentro de la cola `imu_queue`.

**Paso 7 — El actuador se despierta.** `task_actuator` (prioridad 7, alta) estaba **dormido** esperando en
`xQueueReceive`. Al llegar el dato, se despierta y —por tener más prioridad que el sensor— el planificador
**desaloja al sensor** y lo pone a correr **ya**. *(Ver II.4, preemption.)*
→ **Viaja:** el struct con el ángulo.

**Paso 8 — Calcular el error y la corrección** (control proporcional):
```c
float z_roll = zero_roll;                        // el "nivelado" de referencia (leído bajo mutex)
float error_roll = data.roll - z_roll;           // cuánto está desviado
float s1 = 1500 + SERVO1_DIR * error_roll * 10;  // 10 µs de corrección por grado
float s2 = 1500 + SERVO2_DIR * error_roll * 10;
```
*(Ver IV.3.)* → **Viaja:** dos anchos de pulso `s1`, `s2` en µs.

**Paso 9 — Saturar (clamping).** Se recorta cada pulso al rango seguro `[1000, 2000]` µs para no dañar el
servo. *(Ver II.9.c, dificultad 3.)*

**Paso 10 — Ordenar a los servos.** `servos_set_us(s1, s2)` cambia el **número del comparador** de cada canal
MCPWM. *(Ver II.9.c.)* → **Viaja:** el nuevo ancho de pulso al hardware MCPWM.

**Paso 11 — El hardware genera el PWM** (sin CPU). El timer cuenta 0→20000 (20 ms); pone el pin en ALTO al
inicio y en BAJO al llegar al comparador (p. ej. 1550). Sale un pulso de 1550 µs cada 20 ms por GPIO16/17.
→ **Viaja:** una señal eléctrica PWM por el cable de señal del servo.

**Paso 12 — El servo se mueve.** El SG90 traduce el ancho del pulso en un ángulo del brazo → mueve el alerón
→ **corrige la inclinación**. El avión se endereza → en el próximo ciclo el error es menor → el servo se
mueve menos. Y vuelta a empezar. **Eso es el lazo cerrado.**

> **Todo esto (pasos 2 a 12) ocurre en muchísimo menos de 20 ms, 50 veces por segundo.** Ese margen es lo que
> hace que el sistema cumpla el requisito de tiempo real.

### 🟡 Flujo secundario 1 — Fijar el punto cero (cuando apretás el botón)
1. Accionás el botón → **flanco de bajada en GPIO19** → el hardware dispara la **ISR** `gpio_isr_handler`.
2. La ISR (mínima) revisa el antirrebote (300 ms) y **solo** hace `zero_reset_requested = true` (protegido
   por el spinlock). Termina en microsegundos. *(Ver II.5.)*
3. En su próximo ciclo, `task_sensor` ve la bandera, toma el mutex y hace `zero_roll = roll; zero_pitch =
   pitch;` → **la inclinación actual se vuelve el nuevo "nivelado"**.
4. Desde ahí, el error del Paso 8 se mide contra ese nuevo cero.

### 🟢 Flujo secundario 2 — Monitoreo (cada 200 ms)
1. `task_monitor` (prioridad 3, baja) lee el último ángulo (bajo spinlock) y el punto cero (bajo mutex).
2. Los imprime por **UART0** con `ESP_LOGI` → los ves en la consola (`idf.py monitor`, 115200 baud). *(Ver
   II.9.b.)*
3. Por ser la de menor prioridad, **nunca** estorba al control.

### Mapa de "quién le habla a quién"
```
                    ┌──────────── zero_reset (spinlock) ────────────┐
   Botón ──────ISR──> [bandera] ──> task_sensor                     │
   (GPIO19)                            │  (lee I2C, calcula ángulo,  │
                                       │   filtra)                   │
   MPU6050 ──I2C──> task_sensor ──imu_queue(copia)──> task_actuator ─┴─> servos_set_us ──PWM──> SG90 x2
                        │                                 │
                        ├── zero_roll/pitch (mutex) ──────┤   (punto cero compartido)
                        └── last_roll/pitch (spinlock) ─> task_monitor ──UART──> Consola PC
```
Leído en palabras: el **botón** avisa por una bandera; el **sensor** lee el MPU por I2C, filtra y manda el
ángulo por la **cola** al **actuador**, que mueve los **servos** por PWM; en paralelo, el **monitor** lee las
variables compartidas y las imprime por el **puerto serie**. Las líneas de mutex/spinlock son las
protecciones para que nadie pise a nadie.

---
---

# PARTE VI — CONTEXTO ACADÉMICO (para defender decisiones)

## VI.1 ¿Todo lo implementado lo vimos en la cursada?

**Casi todo sí.** Lo verifiqué contra las diapositivas y laboratorios. Excepciones a tener presentes:

| Elemento | ¿Visto en clase? | Dónde / nota |
|---|---|---|
| RTOS, tareas, prioridades, preemption | ✅ | `12-rtos`, `01-...`, `FreeRTOS.pdf` |
| `xTaskCreate`, `vTaskDelayUntil` | ✅ | `FreeRTOS.pdf` |
| Colas productor-consumidor | ✅ | `U3_lab4` |
| Mutex, semáforos, ISR con semáforo | ✅ | `08-sinc_hilos`, `U3_lab3` |
| Sección crítica / spinlock | ✅ | `08-sinc_hilos`, `12-rtos` |
| I2C (maestro/esclavo, SDA/SCL, direcciones) | ✅ | `I2C_intro` |
| MPU6050 (IMU) | ✅ | `U3_lab1` (usa MPU6050) |
| PWM / servo SG90 | ✅ | `U1_lab52`, `U2_lab1`, `U3_lab1` |
| Filtro digital, FIR/IIR, **media móvil** | ✅ | `13-filtros_digitales`, `U3_lab1` |
| **Filtro complementario + `atan2`** | ❌ **NO** | *No está en ningún material.* Es la técnica que reemplazó a la media móvil. **Preparar esta respuesta (ver II.8).** |
| **API MCPWM del ESP32** | ⚠️ Parcial | El *concepto* PWM/servo sí; **esta API concreta no** (los labs usaban `pigpio` en Raspberry). |
| **API I2C del ESP-IDF** | ⚠️ Parcial | El *protocolo* I2C sí; **esta API concreta no** (el lab era con `pigpio`). |

**Detalle clave — µC/OS vs FreeRTOS y POSIX vs FreeRTOS:** buena parte de la teoría fue con **µC/OS-III** y
con **POSIX en Raspberry Pi** (pthreads, colas POSIX, timers POSIX). El proyecto usa **FreeRTOS en ESP32**,
que aplica **los mismos conceptos con otra API**. La correspondencia:

| Concepto (teoría POSIX/µC/OS) | En el proyecto (FreeRTOS) |
|---|---|
| Hilo / tarea | `xTaskCreate` |
| Espera periódica | `vTaskDelayUntil` |
| Mutex | `xSemaphoreCreateMutex` |
| Cola de mensajes | `xQueueSend`/`xQueueReceive` |
| Sección crítica / spinlock | `portENTER_CRITICAL` (`portMUX`) |
| Señal/evento | ISR de GPIO + bandera |
| Reloj de alta resolución | `esp_timer_get_time()` |

> Nota: **no usamos procesos ni señales POSIX**: en un microcontrolador con FreeRTOS todo son **tareas
> (hilos)** en un único espacio de memoria. Si preguntan por "procesos", la respuesta es que acá aplica el
> modelo de **hilos/tareas**, no de procesos separados.

## VI.2 ¿Coincide con el Anteproyecto?

Objetivo y hardware: **sí**. Tres desviaciones deliberadas a saber justificar:
1. **Filtro:** el anteproyecto decía media móvil; implementamos **complementario** (mejor, ver II.8).
2. **Ejes:** el anteproyecto quería corregir dos ejes; corregimos **solo roll** (el pitch se mide y reporta).
   Es el eje natural de los alerones en un ala volante.
3. **Tareas:** el anteproyecto separaba "productor (lee) / consumidor (filtra) / actuación"; en la práctica el
   sensor **lee y filtra**, el actuador **consume y mueve**, y agregamos un **monitor**. Mismo espíritu
   productor-consumidor.

## VI.3 Sobre la documentación técnica previa

El proyecto tenía una `DOCUMENTACION.md` generada por IA. Se **consolidó todo su contenido útil en esta
guía** (que además lo explica desde cero y más a fondo), así que se eliminó para no mantener dos documentos
en paralelo que se desincronizan. Durante esa consolidación se detectaron y **corrigieron** algunas
inconsistencias entre aquella doc y el código: el comentario erróneo de la ISR ("flanco positivo" → es
flanco de bajada), un comentario "pull-down" que era pull-up, y la descripción enrevesada del pulsador.
**Queda un solo punto a verificar contra el hardware:** si el pulsador físico es un botón a GND o un TTP223
configurado como activo-bajo (ver §III.4 y §VIII).

## VI.4 Sistema de compilación: CMake y sdkconfig

Estas piezas no son "código del proyecto" sino la **maquinaria** para compilar.

- **CMake** es un **generador de sistema de compilación**: vos describís el proyecto en archivos
  `CMakeLists.txt` y CMake genera las instrucciones que usa el compilador. Cadena: `idf.py build` → CMake →
  Ninja → `gcc` compila → se arma el `.bin` → `esptool` lo graba.
- Los **`CMakeLists.txt`** (uno en la raíz, otro en `main/`) le dicen a ESP-IDF **qué archivos compilar** y
  **de qué componentes depende**. Son **obligatorios**: sin ellos no compila.
- **`sdkconfig.defaults`** (34 líneas, fuente, va en git): fija las opciones del SDK que necesitamos, sobre
  todo **`CONFIG_FREERTOS_HZ=1000`** (tick de 1 ms, base del tiempo real), el tamaño de flash y la IRAM para
  la ISR. **No conviene sacarlo:** sin él, el tick vuelve a 100 Hz y cambia el comportamiento temporal.
- **`sdkconfig`** (~3700 líneas, dice "DO NOT EDIT"): es **generado** a partir de `sdkconfig.defaults`; se
  recrea solo en cada build, por eso **no se versiona** (lo pusimos en `.gitignore`).

---
---

# PARTE VII — PROFUNDIZACIÓN (detalles finos para puntos extra)

Esta parte junta detalles que **no hacen falta para entender el sistema**, pero que **suman mucho** si el
profesor "escarba" en un tema. Cada uno profundiza un concepto ya visto.

## VII.1 I2C a fondo: el byte de dirección y el ACK/NACK  [amplía §II.9.a]

En §II.9.a vimos la transacción a grandes rasgos. Dos detalles finos que están en el código:

**(a) El byte de dirección lleva la dirección Y el bit de lectura/escritura juntos.** En el código verás:
```c
i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
```
La dirección I2C es de **7 bits** (0x68), pero se transmite un **byte de 8 bits**: los 7 bits de dirección
van arriba y el **bit 0 indica la operación** (0 = escribir, 1 = leer). Por eso se hace `MPU6050_ADDR << 1`
(corre la dirección un lugar a la izquierda para dejar libre el bit 0) y luego se le pega el bit de R/W:
- Escritura: `0x68 << 1 | 0` = **0xD0**
- Lectura:  `0x68 << 1 | 1` = **0xD1**

**(b) Cada byte se confirma con un ACK; el último de una lectura, con NACK.** En I2C, después de cada byte
el receptor manda un bit de **ACK** ("recibido, seguí") o **NACK** ("no"). Fijate en el código de lectura:
```c
i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);      // todos menos el último: ACK
i2c_master_read_byte(cmd, &buf[len - 1], I2C_MASTER_NACK); // el último: NACK
```
El maestro pone **NACK en el último byte** para decirle al sensor **"ya no quiero más datos"**; sin eso, el
sensor seguiría entregando. Es la forma de cerrar la lectura ordenadamente.

**(c) El "RESTART" (repeated start).** Entre escribir el registro y leer los datos, en vez de hacer STOP y
un nuevo START (que soltaría el bus y otro maestro podría meterse), se hace un **RESTART**: un START nuevo
sin soltar el bus. Garantiza que la lectura es **atómica** respecto a la escritura del registro.

**(d) `i2c_cmd_link`.** El driver legacy de ESP-IDF arma la transacción como una **lista de comandos**
(`i2c_cmd_link_create` → agregar START, bytes, STOP → `i2c_master_cmd_begin` la ejecuta de una). Es como
escribir la "receta" de la transacción y después mandarla a ejecutar toda junta.

## VII.2 Los stacks de las tareas: "palabras" vs bytes  [amplía §II.4]

Al crear una tarea le damos un **stack** (pila): la memoria privada donde guarda sus variables locales y el
"rastro" de las funciones que va llamando. En `config.h`:
```c
#define STACK_SENSOR   4096   // ¡en PALABRAS (words), no en bytes!
```
Detalle fino: FreeRTOS mide el stack en **palabras**, no en bytes. En el ESP32 (arquitectura de 32 bits) **1
palabra = 4 bytes**, así que `4096 palabras = 16 KB`. ¿Por qué tanto? Porque `task_sensor` llama a funciones
matemáticas (`atan2f`, `sqrtf`) que usan bastante stack, y **quedarse corto de stack (stack overflow)
corrompe la memoria y cuelga el sistema**. Por eso se da un margen holgado. El monitor usa menos (3072)
porque solo imprime.

## VII.3 `IRAM_ATTR` y por qué importa la caché  [amplía §II.5]

Vimos que la ISR se marca `IRAM_ATTR` para "baja latencia". El detalle de por qué: normalmente el programa
vive en la **flash** y el CPU lo ejecuta a través de una **caché** de instrucciones. Si justo la instrucción
que necesita **no está en la caché** (un *cache miss*), el CPU tiene que **ir a buscarla a la flash**, lo que
tarda **cientos de nanosegundos** — un tiempo **impredecible**. En un sistema de tiempo real, "impredecible"
es una mala palabra. Al poner la ISR en **IRAM** (RAM interna), **siempre está disponible al instante**, con
latencia mínima y **determinística**. Por eso las ISR (y otras rutinas críticas) se ponen en IRAM. Esto se
apoya en la opción `CONFIG_GPIO_CTRL_FUNC_IN_IRAM=y` del `sdkconfig.defaults`.

## VII.4 MCPWM sin glitches y el dibujo del pulso  [amplía §II.9.c]

Un detalle clave de la configuración del comparador:
```c
mcpwm_comparator_config_t cmp_config = { .flags.update_cmp_on_tez = true };
```
`update_cmp_on_tez` = "actualizar el comparador cuando el timer llega a cero" (*Timer Event Zero*). ¿Por qué
importa? Si cambiáramos el ancho del pulso **en mitad de un pulso**, ese pulso saldría deforme (más corto o
más largo de lo debido) — un **glitch** que haría "temblar" al servo. Al aplicar el cambio **solo al inicio
del próximo período**, cada pulso sale siempre completo y limpio.

Visualmente, un pulso de 1500 µs dentro del período de 20000 µs se ve así:
```
GPIO: ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁
      ←── 1500 µs en ALTO ──→←──────── 18500 µs en BAJO ─────────→
      ←──────────────────── 20000 µs (un período, 50 Hz) ─────────→
```
El generador pone el pin en ALTO cuando el timer está en 0, y en BAJO cuando llega a 1500. Mover el servo =
cambiar ese 1500.

## VII.5 Robustez del arranque: `ESP_ERROR_CHECK`  [amplía §IV / arranque]

En `main.c`, cada paso de inicialización va envuelto así:
```c
ESP_ERROR_CHECK(mpu6050_i2c_init());
```
`ESP_ERROR_CHECK` es una macro que **verifica el código de retorno**: si la función devolvió un error (por
ejemplo, el sensor no responde por I2C), **detiene el sistema con un "pánico controlado"** y un mensaje claro
en la consola, **en vez de seguir andando con hardware mal inicializado** (lo que daría fallas raras después).
Es una decisión de diseño defensiva: en un sistema de tiempo real es mejor **fallar rápido y claro** que
arrastrar un estado inconsistente.

## VII.6 Detalles del entorno (contexto para el examen)

- **Driver I2C "legacy" vs nuevo.** En ESP-IDF v6 salió un driver I2C nuevo (`esp_driver_i2c`); el que
  usamos (`driver/i2c.h`, con `i2c_cmd_link`) quedó marcado como *"End Of Life"* pero **sigue funcionando**.
  Lo usamos porque es el que se ve en clase; la opción `CONFIG_I2C_SUPPRESS_DEPRECATE_WARN=y` solo silencia
  el aviso de "obsoleto" para que no llene la consola. (Con MCPWM pasó al revés: el *legacy* **sí** se
  eliminó en v6, por eso servos.c usa la API nueva de handles.)
- **Huella del firmware.** El programa compilado ocupa **~186 KB**, un **18 %** de los 2 MB de flash del
  módulo. Queda muchísimo lugar libre: el sistema es chico y eficiente.

## VII.7 Recetario de calibración y ajuste (tuning)  [amplía §IV.4]

Todos los parámetros están en `config.h`. Esta es la guía práctica de "qué tocar según el síntoma"
(útil si el profesor pregunta "¿cómo lo ajustan si no anda bien?"):

**Centrado de los servos** — si al montarlos no quedan a 0° con el alerón neutro:
```c
#define SERVO1_ZERO_US  1500   // probar 1480 o 1520 (rango útil ~1400–1600)
```
Subir/bajar el valor corre el servo en un sentido u otro. Es un ajuste físico, no toca la lógica.

**Sentido de un servo** — si un servo corrige *al revés* (agranda la inclinación en vez de reducirla):
```c
#define SERVO1_DIR  (-1.0f)   // invertir solo ese servo
```

**Ganancia del control (`CONTROL_GAIN`)** — cuántos µs corrige por grado de error:
- Corrige **muy lento** / no se estabiliza a tiempo → **subir** (ej. 15.0), más agresivo.
- Corrige **muy brusco** / oscila (tiembla) → **bajar** (ej. 5.0), más suave.

**Peso del filtro (`COMP_ALPHA`)** — balance giroscopio/acelerómetro:
- El ángulo **salta con la vibración** del motor → **subir** hacia 0.995 (más giroscopio, menos ruido).
- El ángulo **deriva** lentamente con el tiempo (drift visible) → **bajar** hacia 0.95 (más acelerómetro,
  más "ancla").

> Es un balance: demasiado giroscopio = drift; demasiado acelerómetro = ruido. 0.98 es un buen punto medio.

## VII.8 Detalles de hardware finos

- **Pull-ups del bus I2C.** SDA y SCL necesitan resistencias *pull-up* (el bus es de "colector abierto":
  los chips solo pueden llevar la línea a 0, y la resistencia la devuelve a 1). Acá se activan los pull-ups
  **internos** del ESP32 por software. En un diseño definitivo se pondrían resistencias **externas de
  4.7 kΩ**, pero las internas alcanzan a 400 kHz para el cable corto de la maqueta.
- **Por qué la dirección del MPU6050 es 0x68.** El chip tiene un pin **AD0** que elige entre dos
  direcciones: AD0 a **GND → 0x68**; AD0 a 3.3V → 0x69. Como en la maqueta AD0 está a masa, la dirección es
  **0x68**. (Sirve para poner dos MPU6050 en el mismo bus sin que choquen.)

## VII.9 Práctico: compilar, flashear y ver la consola

Útil para la demo y para responder "¿cómo lo corren?". En cada terminal nueva hay que **activar el entorno**
ESP-IDF primero:
```bash
source ~/esp/esp-idf/export.sh      # agrega idf.py, el compilador y esptool al PATH
```
Comandos principales (la placa aparece en Linux como `/dev/ttyUSB0`):
```bash
idf.py build                                 # compilar
idf.py -p /dev/ttyUSB0 flash                 # grabar en la ESP32 (compila si hace falta)
idf.py -p /dev/ttyUSB0 monitor               # ver la consola serie (salir con Ctrl+])
idf.py -p /dev/ttyUSB0 flash monitor         # las dos cosas de una
idf.py -p /dev/ttyUSB0 erase-flash           # borrar toda la flash (resetear estado)
```
**Permisos del puerto USB** (si da "permission denied" al flashear): agregar tu usuario al grupo `dialout`
(`sudo usermod -aG dialout $USER`, y volver a iniciar sesión) o, temporalmente, `sudo chmod 666 /dev/ttyUSB0`.

---
---

# PARTE VIII — REPASO

## VIII.1 Preguntas probables del profesor (respuestas cortas)

1. **¿Qué es un sistema de tiempo real?** — Uno donde el resultado debe ser correcto **y** llegar dentro de
   un plazo garantizado; si llega tarde, no sirve. Lo importante es el **determinismo**.
2. **¿Por qué un RTOS y no un `while(1)` común?** — Para correr varias tareas con **prioridades** y garantías
   temporales; el RTOS desaloja lo menos importante para atender lo crítico.
3. **¿Vieron µC/OS? ¿Por qué FreeRTOS?** — En teoría vimos el RTOS con µC/OS-III; usamos FreeRTOS porque
   viene en el ESP-IDF del ESP32. Mismos conceptos, distinta API.
4. **¿Por qué el actuador tiene más prioridad que el sensor?** — Porque cierra el lazo de control; apenas hay
   dato, debe corregir ya, desalojando al sensor.
5. **¿`vTaskDelay` o `vTaskDelayUntil`? ¿Por qué?** — `vTaskDelayUntil`, para muestreo periódico **exacto**
   (instantes absolutos), sin acumular deriva.
6. **¿Este filtro lo vieron en clase?** — En clase vimos media móvil; usamos **filtro complementario**
   (fusión giroscopio+acelerómetro) porque da un ángulo sin drift y sin retardo (ver II.8).
7. **¿Por qué la ISR solo levanta una bandera?** — Debe ser cortísima y no bloqueante; el trabajo real lo
   hace la tarea (trabajo diferido).
8. **¿Mutex vs spinlock?** — Spinlock para tramos cortísimos que también toca la ISR (no puede dormir);
   mutex para recursos entre tareas que sí pueden esperar durmiendo.
9. **¿Qué es I2C y a qué velocidad?** — Bus de 2 cables (SDA datos, SCL reloj), maestro-esclavo,
   direccionamiento de 7 bits, a 400 kHz. Distinto de los 50 Hz de muestreo, que decidimos nosotros.
10. **¿Se corrige la gravedad?** — No; se **aprovecha** para calcular la inclinación con el acelerómetro. Lo
    que se filtra es la vibración (con el giroscopio, vía complementario).
11. **¿Qué es MCPWM y por qué no LEDC?** — Periférico de PWM para motores; resolución en µs y sin glitches,
    ideal para servos.
12. **¿Los servos giran igual o invertido?** — Mecánicamente **opuestos** (para el roll); eléctricamente
    reciben lo mismo porque el servo 2 está montado espejado.
13. **¿Qué es el puerto serie / UART?** — Comunicación bit a bit por un cable; UART es el hardware que lo
    implementa; asíncrono a 115200 baudios; lo vemos con `idf.py monitor`.
14. **¿Usan procesos o hilos? ¿POSIX?** — Hilos (tareas FreeRTOS), un solo espacio de memoria. No POSIX
    directo; los conceptos POSIX/µC/OS se mapean a la API de FreeRTOS.
15. **¿Para qué sirven CMakeLists y sdkconfig.defaults?** — CMakeLists le dice a ESP-IDF qué compilar
    (obligatorio); sdkconfig.defaults fija opciones como el tick de 1 kHz. El sdkconfig grande es generado.

## VIII.2 Glosario rápido

- **Firmware:** el programa grabado dentro del dispositivo.
- **Tarea / hilo:** unidad de ejecución que corre "en paralelo" con otras, compartiendo memoria.
- **Planificador (scheduler):** el árbitro que decide qué tarea corre según prioridad.
- **Preemption (desalojo):** cuando una tarea de más prioridad interrumpe a una de menos.
- **ISR:** función que el hardware ejecuta al ocurrir una interrupción; debe ser cortísima.
- **Mutex:** "llave" para que una sola tarea acceda a un recurso a la vez (los demás duermen esperando).
- **Spinlock (portMUX):** protección de secciones cortísimas por espera activa; usable desde ISR.
- **Cola (queue):** buzón FIFO que pasa copias de datos entre tareas de forma segura.
- **I2C:** bus serie de 2 cables (SDA/SCL) maestro-esclavo con direcciones.
- **UART / puerto serie:** envío de datos bit a bit; asíncrono; baud rate acordado (115200).
- **PWM:** señal que "codifica" un valor en el ancho de su pulso; mueve los servos.
- **MCPWM:** periférico del ESP32 que genera PWM por hardware.
- **IMU:** sensor inercial (acelerómetro + giroscopio), acá el MPU6050.
- **Drift:** desvío acumulado del ángulo del giroscopio con el tiempo.
- **Filtro complementario:** fusión de giroscopio (corto plazo) y acelerómetro (largo plazo).
- **Punto cero:** la inclinación de referencia que se considera "nivelado".
- **Baud rate:** bits por segundo del puerto serie.
- **Determinismo:** poder garantizar que algo ocurre dentro de un plazo conocido, siempre.
- **ACK / NACK:** bit de confirmación en I2C ("recibido" / "no más datos"); el NACK cierra una lectura.
- **Repeated start (RESTART):** reiniciar la conversación I2C sin soltar el bus (lectura atómica).
- **Stack (pila):** memoria privada de cada tarea; medido en *palabras* (1 palabra = 4 bytes en 32 bits).
- **IRAM:** RAM interna del ESP32; poner código ahí (con `IRAM_ATTR`) evita el retardo de la caché de flash.
- **Cache miss:** cuando la instrucción no está en la caché y hay que buscarla en flash (retardo impredecible).
- **Glitch:** pulso deforme (ancho incorrecto); en MCPWM se evita actualizando el comparador al inicio del período.
- **ESP_ERROR_CHECK:** macro que detiene el sistema con pánico controlado si una inicialización falla.
- **Baud / baudios:** unidad de velocidad del puerto serie (símbolos por segundo).
