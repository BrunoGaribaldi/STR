/* ============================================================================
 *  main.c
 *  Sistema Controlador de Vuelo en Tiempo Real para Ala Volante (Alerones)
 *
 *  Flujo de arranque:
 *    1. Inicializa el bus I2C y el MPU6050.
 *    2. Inicializa MCPWM y lleva los servos a su pulso neutro (calibracion).
 *    3. Crea cola, mutex e instala la ISR del pulsador.
 *    4. Lanza las 3 tareas FreeRTOS.
 *
 *  El punto cero logico (zero_roll/zero_pitch) arranca en 0.0 grados y puede
 *  redefinirse en cualquier momento pulsando el boton tactil (GPIO19).
 * ==========================================================================*/

#include "config.h"
#include "mpu6050.h"
#include "servos.h"
#include "tasks.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "MAIN";

#ifdef BUTTON_TEST_MODE
/* ---------------------------------------------------------------------------
 *  MODO DIAGNOSTICO DEL PULSADOR (GPIO19).
 *  Configura el pin como entrada con pull-down y reporta:
 *    - el nivel en cada cambio (con flanco SUBE/BAJA)
 *    - un "latido" periodico con el nivel actual
 *  Sirve para saber si el TTP223 esta bien cableado y si su salida sube o baja
 *  al tocarlo (define que flanco debe usar la ISR real).
 * ------------------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGW(TAG, "=== DIAGNOSTICO PULSADOR en GPIO%d (toca el TTP223) ===", BUTTON_GPIO);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,     /* reposo en ALTO (1)        */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    int prev = gpio_get_level(BUTTON_GPIO);
    ESP_LOGI(TAG, "Nivel inicial (reposo) = %d  %s",
             prev, prev ? "(reposo en ALTO, esperado: pulsador a GND)"
                        : "(reposo en BAJO!)");

    int heartbeat = 0;
    for (;;) {
        int lvl = gpio_get_level(BUTTON_GPIO);
        if (lvl != prev) {
            ESP_LOGI(TAG, ">>> CAMBIO: %d -> %d  (flanco %s)",
                     prev, lvl, lvl ? "SUBE (rising)" : "BAJA (falling)");
            prev = lvl;
        }
        if (++heartbeat >= 100) {   /* cada ~1 s */
            ESP_LOGI(TAG, "nivel actual = %d", lvl);
            heartbeat = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#else

#ifdef SERVO_TEST_MODE
/* ---------------------------------------------------------------------------
 *  MODO DE PRUEBA DE SERVOS (sin MPU ni logica de control).
 *  Barre AMBOS servos de forma identica entre 1000 y 2000 us, ida y vuelta.
 *  Sirve para comparar "en seco" la velocidad real de cada servo: como reciben
 *  exactamente el mismo pulso al mismo tiempo, el que se atrase es el lento.
 * ------------------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGW(TAG, "=== MODO PRUEBA DE SERVOS (barrido 1000<->2000us) ===");

    ESP_ERROR_CHECK(servos_init());

    const int STEP_US   = 10;   /* resolucion del barrido            */
    const int STEP_MS   = 20;   /* tiempo por paso -> ~2s por tramo  */

    for (;;) {
        /* Subida: 1000 -> 2000 us */
        for (int us = SERVO_MIN_US; us <= SERVO_MAX_US; us += STEP_US) {
            servos_set_us(us, us);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }
        ESP_LOGI(TAG, "Extremo +: %d us (pausa)", SERVO_MAX_US);
        vTaskDelay(pdMS_TO_TICKS(500));

        /* Bajada: 2000 -> 1000 us */
        for (int us = SERVO_MAX_US; us >= SERVO_MIN_US; us -= STEP_US) {
            servos_set_us(us, us);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }
        ESP_LOGI(TAG, "Extremo -: %d us (pausa)", SERVO_MIN_US);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

#else /* ----------------------- FIRMWARE NORMAL ----------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "=== Controlador de vuelo ala volante (1 eje) ===");

    /* 1) Bus I2C + sensor inercial */
    ESP_ERROR_CHECK(mpu6050_i2c_init());
    ESP_ERROR_CHECK(mpu6050_init());

    /* 2) Servos: MCPWM @50Hz y posicion neutra de calibracion */
    ESP_ERROR_CHECK(servos_init());

    /* 3) Cola, mutex e ISR del pulsador */
    tasks_sync_init();

    /* 4) Crear las tareas FreeRTOS */
    tasks_start();

    ESP_LOGI(TAG, "Sistema en marcha. Punto cero inicial: 0.0/0.0 grados");
}

#endif /* SERVO_TEST_MODE */

#endif /* BUTTON_TEST_MODE */
