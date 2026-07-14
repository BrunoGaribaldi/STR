/* ============================================================================
 *  tasks.c
 *  Implementa las 3 tareas FreeRTOS + la ISR del pulsador.
 *
 *  Arquitectura:
 *    task_sensor   (prio MEDIA) -> lee MPU6050 @50Hz, filtra, publica en cola.
 *    task_actuator (prio ALTA)  -> consume cola, calcula correccion, mueve servos.
 *    task_monitor  (prio BAJA)  -> imprime estado por puerto serie cada 200ms.
 *    gpio_isr_handler (ISR)     -> al pulsar, solicita fijar nuevo punto cero.
 *
 *  CONTROL DE UN SOLO EJE: el actuador corrige el eje seleccionado en config.h
 *  (ROLL por defecto). El otro eje se monitorea pero no genera correccion.
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
 *  Objetos de sincronizacion globales
 * ------------------------------------------------------------------------- */
QueueHandle_t     imu_queue   = NULL;
SemaphoreHandle_t zero_mutex  = NULL;

/* Punto cero logico (referencia de nivelado). Protegido por zero_mutex. */
static float zero_roll  = 0.0f;
static float zero_pitch = 0.0f;

/* ---------------------------------------------------------------------------
 *  Datos compartidos entre la ISR y la tarea sensor.
 *  La ISR no puede tomar mutex, asi que se usa una seccion critica con spinlock
 *  para leer/escribir estas variables de forma atomica.
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
 *  ISR DEL PULSADOR (GPIO19, flanco de BAJADA - ver GPIO_INTR_NEGEDGE abajo)
 *  Antirrebote por software de 300 ms. Solo levanta una bandera; el trabajo
 *  real (fijar el cero) lo hace la tarea sensor, que es quien conoce el angulo.
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
 *  TAREA PRODUCTORA: lectura del IMU + filtro complementario (1 eje activo)
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
            /* Angulos por acelerometro (gravedad) usando atan2 */
            float accel_roll  = atan2f(r.acc_y, r.acc_z) * RAD_TO_DEG;
            float accel_pitch = atan2f(-r.acc_x,
                                       sqrtf(r.acc_y * r.acc_y + r.acc_z * r.acc_z))
                                * RAD_TO_DEG;

            /* Filtro complementario:
             * angle = 0.98*(angle + gyro*dt) + 0.02*accel_angle
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

            /* Publicar los angulos filtrados en la cola */
            imu_data_t data = { .roll = roll, .pitch = pitch };
            if (xQueueSend(imu_queue, &data, 0) != pdTRUE) {
                ESP_LOGW(TAG_SENSOR, "Cola llena, muestra descartada");
            }
        } else {
            ESP_LOGE(TAG_SENSOR, "Fallo de lectura I2C del MPU6050");
        }

        /* Muestreo periodico exacto a 50 Hz */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

/* ===========================================================================
 *  TAREA CONSUMIDORA / ACTUACION: calcula correccion y mueve servos.
 *  No usa vTaskDelay: bloquea en la cola con timeout como mecanismo de espera.
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

            /* Error de ALABEO respecto al punto cero.
             * (El pitch se mide y se reporta, pero no genera correccion.) */
            float error_roll = data.roll - z_roll;

            /* Partimos del neutro fisico de cada servo y aplicamos la
             * correccion de roll segun el sentido de cada servo
             * (SERVO1_DIR / SERVO2_DIR en config.h). En este montaje ambos
             * van con el mismo signo (Servo2 estaba invertido). */
            float s1 = (float)SERVO1_ZERO_US + SERVO1_DIR * error_roll * CONTROL_GAIN;
            float s2 = (float)SERVO2_ZERO_US + SERVO2_DIR * error_roll * CONTROL_GAIN;

            /* Limitar al rango seguro [1000, 2000] us */
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
 *  TAREA DE MONITOREO: imprime estado por serie cada 200 ms.
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
 *  INICIALIZACION DE SINCRONIZACION + ISR
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
     * Pulsador clasico de 4 patas cableado a GND:
     *   reposo -> pull-up mantiene la linea en ALTO (1)
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
 *  CREACION DE LAS TAREAS
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
