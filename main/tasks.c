/* ============================================================================
 *  tasks.c   <<< ARCHIVO CLAVE DEL PROYECTO >>>
 *  Implementa las 3 tareas FreeRTOS + la ISR del pulsador.
 *
 *  MATERIAL DE ESTUDIO: es el nucleo del sistema. Referencias a la guia:
 *    - Tareas, prioridades, planificador ....... [Guia §II.4]
 *    - Interrupciones / ISR / antirrebote ...... [Guia §II.5]
 *    - Sincronizacion (mutex, spinlock, cola) .. [Guia §II.6]
 *    - Filtro complementario ................... [Guia §II.8]
 *    - Control proporcional .................... [Guia §IV.3]
 *    - Recorrido completo de un dato ........... [Guia §V]
 *
 *  Arquitectura:
 *    task_sensor   (prio MEDIA) -> lee MPU6050 @50Hz, filtra, publica en cola.
 *    task_actuator (prio ALTA)  -> consume cola, calcula correccion, mueve servos.
 *    task_monitor  (prio BAJA)  -> imprime estado por puerto serie cada 200ms.
 *    gpio_isr_handler (ISR)     -> al pulsar, solicita fijar nuevo punto cero.
 *
 *  CONTROL DE UN SOLO EJE: el actuador corrige ROLL (alabeo). El PITCH se mide
 *  y se reporta, pero no genera correccion (ver [Guia §VI.2]).
 * ==========================================================================*/

#include "tasks.h"
#include "config.h"
#include "mpu6050.h"
#include "servos.h"

#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include <math.h>
#include <stdbool.h>

#define RAD_TO_DEG (57.2957795f)   /* 180/PI */

/* ---------------------------------------------------------------------------
 *  Objetos de sincronizacion globales                         [Guia §II.6]
 *  Como las tareas comparten memoria, hay que protegerse de "condiciones de
 *  carrera" (dos tareas tocando lo mismo a la vez). Usamos 3 mecanismos:
 *    - imu_queue : COLA, pasa el angulo del sensor al actuador [Guia §II.6.c]
 *    - zero_mutex: MUTEX, protege el punto cero (entre tareas) [Guia §II.6.b]
 *    - isr_mux   : SPINLOCK, protege lo que comparte con la ISR [Guia §II.6.a]
 * ------------------------------------------------------------------------- */
QueueHandle_t     imu_queue   = NULL;   /* [Guia §II.6.c] cola productor->consumidor */
SemaphoreHandle_t zero_mutex  = NULL;   /* [Guia §II.6.b] protege el punto cero       */

/* Punto cero logico (referencia de nivelado). Protegido por zero_mutex. */
static float zero_roll  = 0.0f;
static float zero_pitch = 0.0f;

/* ---------------------------------------------------------------------------
 *  Datos compartidos entre la ISR y la tarea sensor.          [Guia §II.6.a]
 *  La ISR no puede tomar un mutex (bloquearia); ademas el ESP32 es dual-core.
 *  Por eso se usa un SPINLOCK (seccion critica) para leer/escribir estas
 *  variables de forma atomica, tanto desde la ISR como desde la tarea.
 * ------------------------------------------------------------------------- */
static portMUX_TYPE isr_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool  zero_reset_requested = false; /* flag puesto por la ISR */
static volatile float last_roll  = 0.0f;            /* ultimo angulo conocido */
static volatile float last_pitch = 0.0f;

/* Para el ultimo valor de servos (solo para el monitor). */
static volatile uint32_t last_servo1_us = SERVO1_ZERO_US;
static volatile uint32_t last_servo2_us = SERVO2_ZERO_US;

/* Marca de tiempo del ultimo flanco aceptado (antirrebote). */
static volatile int64_t last_isr_time_us = 0;

static const char *TAG_SENSOR = "SENSOR";
static const char *TAG_ACT    = "ACTUADOR";
static const char *TAG_MON    = "MONITOR";
static const char *TAG_ISR    = "ISR";

/* ===========================================================================
 *  ISR DEL PULSADOR (GPIO19, flanco de BAJADA)                [Guia §II.5]
 *  Una ISR es una funcion que dispara el HARDWARE al ocurrir un evento (aca,
 *  el flanco del boton). Debe ser CORTISIMA: no imprime, no bloquea. Solo hace
 *  "trabajo diferido": levanta una bandera y termina; el trabajo real (fijar el
 *  cero) lo hace despues la tarea sensor, que es quien conoce el angulo.
 *  - IRAM_ATTR: pone la ISR en RAM interna -> latencia minima y predecible.
 *  - Antirrebote (debounce) 300 ms: ignora rebotes del boton. [Guia §II.5]
 * ==========================================================================*/
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    int64_t now = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&isr_mux);
    if ((now - last_isr_time_us) >= BUTTON_DEBOUNCE_US) {
        last_isr_time_us = now;
        zero_reset_requested = true;
    }
    portEXIT_CRITICAL_ISR(&isr_mux);
}

/* ===========================================================================
 *  TAREA PRODUCTORA (prio MEDIA=5): lee el IMU, filtra y publica en la cola.
 *  Corre exactamente a 50 Hz (ver vTaskDelayUntil al final del bucle).
 *  Es el "Paso 2 a 6" del recorrido de datos de [Guia §V].
 * ==========================================================================*/
static void task_sensor(void *arg)
{
    mpu6050_reading_t r;

    /* Estado del filtro complementario */
    float roll  = 0.0f;
    float pitch = 0.0f;

    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG_SENSOR, "Tarea sensor iniciada (50 Hz)");

    for (;;) {
        if (mpu6050_read(&r) == ESP_OK) {
            /* [Guia §V Paso 4] Angulo por ACELEROMETRO usando la gravedad.
             * atan2 deduce la inclinacion a partir de los componentes del
             * vector gravedad. Es exacto pero RUIDOSO (lo ensucia la vibracion). */
            float accel_roll  = atan2f(r.acc_y, r.acc_z) * RAD_TO_DEG;
            float accel_pitch = atan2f(-r.acc_x,
                                       sqrtf(r.acc_y * r.acc_y + r.acc_z * r.acc_z))
                                * RAD_TO_DEG;

            /* [Guia §II.8] FILTRO COMPLEMENTARIO (el "cerebro" del procesamiento).
             * angle = 0.98*(angle + gyro*dt) + 0.02*accel_angle
             *   98% GIROSCOPIO -> rapido y sin vibracion, pero derivaria (drift).
             *    2% ACELEROMETRO -> ancla a la gravedad real y borra el drift.
             * Para roll integra gyro_x; para pitch integra gyro_y. */
            roll  = COMP_ALPHA * (roll  + r.gyro_x * SENSOR_DT_S)
                    + (1.0f - COMP_ALPHA) * accel_roll;
            pitch = COMP_ALPHA * (pitch + r.gyro_y * SENSOR_DT_S)
                    + (1.0f - COMP_ALPHA) * accel_pitch;

            /* Guardar ultimo angulo para que la ISR/recalibracion lo use */
            portENTER_CRITICAL(&isr_mux);
            last_roll  = roll;
            last_pitch = pitch;
            bool do_reset = zero_reset_requested;
            zero_reset_requested = false;
            portEXIT_CRITICAL(&isr_mux);

            /* Si el pulsador pidio recalibrar, fijar el angulo actual como cero */
            if (do_reset) {
                if (xSemaphoreTake(zero_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    zero_roll  = roll;
                    zero_pitch = pitch;
                    xSemaphoreGive(zero_mutex);
                    ESP_LOGI(TAG_ISR,
                             "Nuevo punto cero fijado: Roll=%.1f Pitch=%.1f",
                             roll, pitch);
                } else {
                    /* No se pudo tomar el mutex: reintentar en el proximo ciclo */
                    portENTER_CRITICAL(&isr_mux);
                    zero_reset_requested = true;
                    portEXIT_CRITICAL(&isr_mux);
                }
            }

            /* [Guia §II.6.c] Publicar los angulos en la COLA (por copia).
             * El "0" = no esperar: si la cola esta llena (actuador atrasado),
             * se DESCARTA la muestra para no frenar el ritmo de 50 Hz. */
            imu_data_t data = { .roll = roll, .pitch = pitch };
            if (xQueueSend(imu_queue, &data, 0) != pdTRUE) {
                ESP_LOGW(TAG_SENSOR, "Cola llena, muestra descartada");
            }
        } else {
            ESP_LOGE(TAG_SENSOR, "Fallo de lectura I2C del MPU6050");
        }

        /* [Guia §II.4] Muestreo periodico EXACTO a 50 Hz. vTaskDelayUntil
         * despierta en instantes absolutos (20,40,60 ms...), sin acumular
         * deriva aunque la lectura tarde distinto cada vez. */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

/* ===========================================================================
 *  TAREA CONSUMIDORA / ACTUACION (prio ALTA=7).    [Guia §II.6.c y §IV.3]
 *  Calcula la correccion y mueve los servos. Es el "Paso 7 a 10" de [Guia §V].
 *  NO usa vTaskDelay: se BLOQUEA en la cola (la cola es su "reloj"). Al llegar
 *  un dato se despierta y, por tener MAS prioridad que el sensor, lo desaloja
 *  para corregir de inmediato (preemption, [Guia §II.4]).
 * ==========================================================================*/
static void task_actuator(void *arg)
{
    imu_data_t data;

    ESP_LOGI(TAG_ACT, "Tarea actuador iniciada (prioridad alta)");

    for (;;) {
        if (xQueueReceive(imu_queue, &data,
                          pdMS_TO_TICKS(ACTUATOR_QUEUE_TIMEOUT_MS)) == pdTRUE) {

            /* Leer el punto cero de ROLL de forma segura (mutex) */
            float z_roll;
            if (xSemaphoreTake(zero_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                z_roll = zero_roll;
                xSemaphoreGive(zero_mutex);
            } else {
                /* Si no se pudo, usar 0 como referencia segura este ciclo */
                z_roll = 0.0f;
                ESP_LOGW(TAG_ACT, "Timeout tomando zero_mutex");
            }

            /* [Guia §IV.3] CONTROL PROPORCIONAL (P).
             * error = cuanto se desvio del "nivelado" (punto cero).
             * (El pitch se mide y se reporta, pero no genera correccion.) */
            float error_roll = data.roll - z_roll;

            /* Partimos del neutro fisico de cada servo y sumamos una correccion
             * PROPORCIONAL al error (CONTROL_GAIN = us por grado). El signo
             * SERVOx_DIR ajusta el sentido segun el montaje. [Guia §II.9.c] */
            float s1 = (float)SERVO1_ZERO_US + SERVO1_DIR * error_roll * CONTROL_GAIN;
            float s2 = (float)SERVO2_ZERO_US + SERVO2_DIR * error_roll * CONTROL_GAIN;

            /* Saturacion (clamping): nunca fuera de [1000,2000] us, o se dana el
             * servo. Es el "Paso 9" de [Guia §V]. */
            if (s1 < SERVO_MIN_US) s1 = SERVO_MIN_US;
            if (s1 > SERVO_MAX_US) s1 = SERVO_MAX_US;
            if (s2 < SERVO_MIN_US) s2 = SERVO_MIN_US;
            if (s2 > SERVO_MAX_US) s2 = SERVO_MAX_US;

            uint32_t s1_us = (uint32_t)s1;
            uint32_t s2_us = (uint32_t)s2;

            /* Mover servos */
            servos_set_us(s1_us, s2_us);

            /* Guardar para el monitor */
            last_servo1_us = s1_us;
            last_servo2_us = s2_us;

        } else {
            /* Timeout: no llegaron muestras. Aviso pero no se bloquea. */
            ESP_LOGW(TAG_ACT, "Sin datos del sensor (timeout cola)");
        }
    }
}

/* ===========================================================================
 *  TAREA DE MONITOREO (prio BAJA=3).                          [Guia §II.9.b]
 *  Imprime el estado por el PUERTO SERIE (UART) cada 200 ms con ESP_LOGI.
 *  Cada 200 ms (no mas seguido) porque es comodo de leer y no debe molestar al
 *  control; por ser la de menor prioridad, corre solo en los ratos libres.
 * ==========================================================================*/
static void task_monitor(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG_MON, "Tarea monitor iniciada (cada %d ms)", MONITOR_PERIOD_MS);

    for (;;) {
        /* Leer ultimos angulos de forma atomica */
        float roll, pitch;
        portENTER_CRITICAL(&isr_mux);
        roll  = last_roll;
        pitch = last_pitch;
        portEXIT_CRITICAL(&isr_mux);

        /* Leer punto cero de forma segura */
        float z_roll = 0.0f, z_pitch = 0.0f;
        if (xSemaphoreTake(zero_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            z_roll  = zero_roll;
            z_pitch = zero_pitch;
            xSemaphoreGive(zero_mutex);
        }

        ESP_LOGI(TAG_MON,
                 "[VUELO] Roll: %+.1f° | Pitch: %+.1f° | Zero(R/P): %.1f°/%.1f°",
                 roll, pitch, z_roll, z_pitch);
        ESP_LOGI(TAG_MON,
                 "        Servo1: %uus | Servo2: %uus",
                 (unsigned)last_servo1_us, (unsigned)last_servo2_us);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MONITOR_PERIOD_MS));
    }
}

/* ===========================================================================
 *  INICIALIZACION DE SINCRONIZACION + ISR       [Guia §II.6 y §II.5]
 *  Crea la cola y el mutex, configura el GPIO del boton e instala su ISR.
 *  Se llama desde app_main ANTES de crear las tareas.
 * ==========================================================================*/
void tasks_sync_init(void)
{
    /* Cola de angulos (capacidad 5) */
    imu_queue = xQueueCreate(IMU_QUEUE_LENGTH, sizeof(imu_data_t));
    configASSERT(imu_queue != NULL);

    /* Mutex del punto cero */
    zero_mutex = xSemaphoreCreateMutex();
    configASSERT(zero_mutex != NULL);

    /* --- Configurar el pulsador GPIO19 como entrada con pull-UP e ISR ---
     * [Guia §II.5 y §III.4] Pulsador tratado como contacto a GND (activo-bajo):
     *   reposo -> el pull-up interno mantiene la linea en ALTO (1)
     *   al apretar -> conecta a GND -> linea en BAJO (0) -> flanco de BAJADA */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,   /* flanco de bajada (falling edge) */
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    /* Servicio de ISR por GPIO e instalacion del handler del pulsador */
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, NULL));

    ESP_LOGI(TAG_ISR, "ISR del pulsador instalada en GPIO%d (falling, pulsador a GND, debounce %d ms)",
             BUTTON_GPIO, BUTTON_DEBOUNCE_US / 1000);
}

/* ===========================================================================
 *  CREACION DE LAS TAREAS                                     [Guia §II.4]
 *  xTaskCreate(funcion, nombre, stack_en_words, param, PRIORIDAD, handle).
 *  No usamos xTaskCreatePinnedToCore: dejamos que FreeRTOS reparta en los 2
 *  nucleos. Las prioridades (5/7/3) estan en config.h.
 * ==========================================================================*/
void tasks_start(void)
{
    xTaskCreate(task_sensor,   "task_sensor",   STACK_SENSOR,   NULL,
                PRIO_SENSOR,   NULL);
    xTaskCreate(task_actuator, "task_actuator", STACK_ACTUATOR, NULL,
                PRIO_ACTUATOR, NULL);
    xTaskCreate(task_monitor,  "task_monitor",  STACK_MONITOR,  NULL,
                PRIO_MONITOR,  NULL);
}
