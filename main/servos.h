/* ============================================================================
 *  servos.h
 *  Control de los dos servos de los alerones mediante el modulo MCPWM.
 *  MATERIAL DE ESTUDIO -> [Guia §II.9.c] (que es PWM/MCPWM y como mueve el servo).
 * ==========================================================================*/

#ifndef SERVOS_H
#define SERVOS_H

#include "esp_err.h"
#include <stdint.h>

/* Inicializa MCPWM para ambos servos a 50 Hz y los lleva a su pulso neutro. */
esp_err_t servos_init(void);

/* Aplica un ancho de pulso (en microsegundos) a cada servo.
 * Los valores se limitan internamente al rango [SERVO_MIN_US, SERVO_MAX_US]. */
void servos_set_us(uint32_t servo1_us, uint32_t servo2_us);

#endif /* SERVOS_H */
