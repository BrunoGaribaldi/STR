/* ============================================================================
 *  main.c
 *  Sistema Controlador de Vuelo en Tiempo Real para Ala Volante (Alerones)
 *
 *  MATERIAL DE ESTUDIO: ver RESUMEN_INTEGRAL.md. Las marcas [Guia §X] enlazan
 *  cada parte del codigo con la seccion que la explica desde cero.
 *  Este archivo (arranque del sistema) -> [Guia §II.2] y [Guia §IV].
 *
 *  app_main() es el "main" de ESP-IDF: FreeRTOS ya esta corriendo cuando se
 *  ejecuta. Hace los 4 pasos de arranque y termina; el scheduler sigue con las
 *  tareas creadas. Flujo de arranque:
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

#include "esp_log.h"

static const char *TAG = "MAIN";

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
