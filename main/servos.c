/* ============================================================================
 *  servos.c
 *  Control de servos por MCPWM usando el driver NUEVO de ESP-IDF v5/v6
 *  (driver/mcpwm_prelude.h). El driver legacy "driver/mcpwm.h" fue eliminado
 *  en ESP-IDF v6.0, por eso se usa la API basada en handles:
 *      timer -> operator -> comparator -> generator
 *
 *  Configuracion: resolucion de 1 us/tick, periodo de 20000 ticks = 20 ms
 *  (50 Hz). El ancho de pulso en microsegundos coincide 1:1 con el valor de
 *  comparacion, por lo que mover el servo es fijar el comparador en 'us' ticks.
 *
 *    - Servo 1: grupo MCPWM 0, timer propio -> GPIO16
 *    - Servo 2: grupo MCPWM 0, timer propio -> GPIO17
 * ==========================================================================*/

#include "servos.h"
#include "config.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

static const char *TAG = "SERVO";

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  /* 1 us por tick            */
#define SERVO_TIMEBASE_PERIOD_TICKS  20000    /* 20 ms -> 50 Hz           */

/* Comparadores: mover un servo = cambiar su valor de comparacion. */
static mcpwm_cmpr_handle_t s_cmp1 = NULL;
static mcpwm_cmpr_handle_t s_cmp2 = NULL;

/* Limita un pulso al rango fisico seguro de los servos. */
static inline uint32_t clamp_us(uint32_t us)
{
    if (us < SERVO_MIN_US) return SERVO_MIN_US;
    if (us > SERVO_MAX_US) return SERVO_MAX_US;
    return us;
}

/* ---------------------------------------------------------------------------
 *  Crea toda la cadena MCPWM para un servo en un GPIO dado.
 *  Devuelve por puntero el comparador (para ajustar el ancho de pulso).
 * ------------------------------------------------------------------------- */
static esp_err_t servo_setup_channel(int gpio, uint32_t init_us,
                                      mcpwm_cmpr_handle_t *out_cmp)
{
    esp_err_t ret;

    /* Timer dedicado a 50 Hz */
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ret = mcpwm_new_timer(&timer_config, &timer);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "mcpwm_new_timer: %s", esp_err_to_name(ret)); return ret; }

    /* Operador asociado al timer */
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_config = { .group_id = 0 };
    ret = mcpwm_new_operator(&oper_config, &oper);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "mcpwm_new_operator: %s", esp_err_to_name(ret)); return ret; }
    ret = mcpwm_operator_connect_timer(oper, timer);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "operator_connect_timer: %s", esp_err_to_name(ret)); return ret; }

    /* Comparador: define el flanco de bajada (ancho del pulso) */
    mcpwm_cmpr_handle_t cmp = NULL;
    mcpwm_comparator_config_t cmp_config = { .flags.update_cmp_on_tez = true };
    ret = mcpwm_new_comparator(oper, &cmp_config, &cmp);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "mcpwm_new_comparator: %s", esp_err_to_name(ret)); return ret; }

    /* Generador: salida fisica en el GPIO */
    mcpwm_gen_handle_t gen = NULL;
    mcpwm_generator_config_t gen_config = { .gen_gpio_num = gpio };
    ret = mcpwm_new_generator(oper, &gen_config, &gen);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "mcpwm_new_generator: %s", esp_err_to_name(ret)); return ret; }

    /* Valor inicial del comparador (pulso neutro) */
    ret = mcpwm_comparator_set_compare_value(cmp, clamp_us(init_us));
    if (ret != ESP_OK) { ESP_LOGE(TAG, "set_compare_value: %s", esp_err_to_name(ret)); return ret; }

    /* Forma de onda PWM:
     *   - al inicio del periodo (timer en cero/empty): poner la salida en ALTO
     *   - al llegar al valor del comparador: poner la salida en BAJO     */
    ret = mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK) { ESP_LOGE(TAG, "action_on_timer: %s", esp_err_to_name(ret)); return ret; }

    ret = mcpwm_generator_set_action_on_compare_event(gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                           cmp,
                                           MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK) { ESP_LOGE(TAG, "action_on_compare: %s", esp_err_to_name(ret)); return ret; }

    /* Habilitar y arrancar el timer */
    ret = mcpwm_timer_enable(timer);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "mcpwm_timer_enable: %s", esp_err_to_name(ret)); return ret; }
    ret = mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "mcpwm_timer_start: %s", esp_err_to_name(ret)); return ret; }

    *out_cmp = cmp;
    return ESP_OK;
}

esp_err_t servos_init(void)
{
    esp_err_t ret;

    ret = servo_setup_channel(SERVO1_GPIO, SERVO1_ZERO_US, &s_cmp1);
    if (ret != ESP_OK) return ret;

    ret = servo_setup_channel(SERVO2_GPIO, SERVO2_ZERO_US, &s_cmp2);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Servos inicializados (50 Hz). Neutro S1=%dus S2=%dus",
             SERVO1_ZERO_US, SERVO2_ZERO_US);
    return ESP_OK;
}

void servos_set_us(uint32_t servo1_us, uint32_t servo2_us)
{
    /* Cambiar el valor de comparacion = cambiar el ancho de pulso (us) */
    mcpwm_comparator_set_compare_value(s_cmp1, clamp_us(servo1_us));
    mcpwm_comparator_set_compare_value(s_cmp2, clamp_us(servo2_us));
}
