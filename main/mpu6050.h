/* ============================================================================
 *  mpu6050.h
 *  Driver I2C del sensor inercial MPU6050.  ->  [Guia §II.9.a] y [Guia §III.2]
 *  Expone funciones para inicializar el bus, despertar el sensor y leer
 *  acelerometro + giroscopio ya escalados a unidades fisicas.
 * ==========================================================================*/

#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"

/* Estructura con una lectura cruda escalada del MPU6050.
 *   acc_*  en g  (gravedades)
 *   gyro_* en grados/segundo
 * Nota: solo se usan gyro_x (roll) y gyro_y (pitch); el eje Z del giroscopio
 * (guinada) no interviene en este proyecto, por eso no se guarda.
 */
typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
} mpu6050_reading_t;

/* Inicializa el driver I2C maestro (pines y velocidad de config.h). */
esp_err_t mpu6050_i2c_init(void);

/* Despierta el sensor y configura los rangos (+-2g y +-250 dps). */
esp_err_t mpu6050_init(void);

/* Lee los 14 bytes desde ACCEL_XOUT_H y devuelve los valores escalados. */
esp_err_t mpu6050_read(mpu6050_reading_t *out);

#endif /* MPU6050_H */
