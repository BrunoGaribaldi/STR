/* ============================================================================
 *  config.h
 *  Sistema Controlador de Vuelo en Tiempo Real para Ala Volante (Alerones)
 *  Materia: Sistemas en Tiempo Real - Ingenieria Informatica
 *
 *  ---------------------------------------------------------------------------
 *  GUIA RAPIDA DE CALIBRACION FISICA
 *  ---------------------------------------------------------------------------
 *  Si al montar los servos NO quedan perfectamente centrados, ajustar SOLO:
 *      - SERVO1_ZERO_US : pulso (us) que deja el aleron IZQUIERDO neutro.
 *      - SERVO2_ZERO_US : pulso (us) que deja el aleron DERECHO  neutro.
 *
 *  Si el avion corrige al reves (sobre-vira en lugar de estabilizar):
 *      - Invertir el signo de CONTROL_GAIN, o
 *      - Intercambiar fisicamente las conexiones de los servos.
 *
 *  Si reacciona poco o demasiado:
 *      - Subir/bajar CONTROL_GAIN.
 *
 *  EJE DE TRABAJO DEL MPU:
 *      Por requerimiento, el sistema controla UN SOLO EJE: el ROLL (alabeo),
 *      que es el eje natural de los alerones en un ala volante. El PITCH
 *      (cabeceo) se mide y se reporta por consola, pero no genera correccion.
 * ==========================================================================*/

#ifndef CONFIG_H
#define CONFIG_H

/* ===========================================================================
 *  PINES DE HARDWARE
 * ==========================================================================*/
#define SERVO1_GPIO            16  /* Senal PWM Servo 1 (aleron izquierdo)  */
#define SERVO2_GPIO            17  /* Senal PWM Servo 2 (aleron derecho)    */
#define BUTTON_GPIO            19  /* Pulsador tactil TTP223 (interrupcion) */
#define I2C_MASTER_SDA_GPIO    21  /* SDA del bus I2C (MPU6050)             */
#define I2C_MASTER_SCL_GPIO    22  /* SCL del bus I2C (MPU6050)             */

/* ===========================================================================
 *  BUS I2C / MPU6050
 * ==========================================================================*/
#define I2C_MASTER_NUM         I2C_NUM_0   /* Puerto I2C usado            */
#define I2C_MASTER_FREQ_HZ     400000      /* Velocidad I2C (400 kHz)     */
#define I2C_MASTER_TIMEOUT_MS  100         /* Timeout transacciones I2C   */

#define MPU6050_ADDR           0x68        /* Direccion I2C del MPU6050   */

/* Registros del MPU6050 */
#define MPU6050_REG_PWR_MGMT_1 0x6B        /* Power management            */
#define MPU6050_REG_GYRO_CFG   0x1B        /* Config giroscopio           */
#define MPU6050_REG_ACCEL_CFG  0x1C        /* Config acelerometro         */
#define MPU6050_REG_ACCEL_XOUT 0x3B        /* Primer byte de medidas      */

/* Factores de escala (datasheet MPU6050) */
#define ACCEL_SCALE_2G         16384.0f    /* LSB/g    a rango +-2g       */
#define GYRO_SCALE_250DPS      131.0f      /* LSB/dps  a rango +-250 dps  */

/* ===========================================================================
 *  SERVOS (MCPWM)
 * ==========================================================================*/
#define SERVO1_ZERO_US         1500   /* <-- AJUSTE FISICO: neutro Servo 1 */
#define SERVO2_ZERO_US         1500   /* <-- AJUSTE FISICO: neutro Servo 2 */

#define SERVO_MIN_US           1000   /* Pulso minimo (-90 grados)         */
#define SERVO_MAX_US           2000   /* Pulso maximo (+90 grados)         */

#define SERVO_PWM_FREQ_HZ      50     /* Frecuencia PWM servos (periodo 20 ms) */

/* Sentido de giro de cada servo en la correccion.
 * Si un servo se mueve al REVES de lo esperado por su montaje fisico,
 * invertir su signo (+1.0 <-> -1.0). No requiere tocar la logica de control.
 *   Servo1 (D16): correcto -> +1.0
 *   Servo2 (D17): invertido fisicamente -> +1.0 (mismo sentido que Servo1) */
#define SERVO1_DIR             (+1.0f)
#define SERVO2_DIR             (+1.0f)

/* ===========================================================================
 *  LOGICA DE CONTROL
 * ==========================================================================*/
#define CONTROL_GAIN           10.0f  /* Ganancia us por grado de error    */

/* ===========================================================================
 *  FILTRO COMPLEMENTARIO Y MUESTREO
 * ==========================================================================*/
#define SENSOR_PERIOD_MS       20     /* 50 Hz de muestreo del IMU         */
#define SENSOR_DT_S            (SENSOR_PERIOD_MS / 1000.0f)  /* dt en seg   */
#define COMP_ALPHA             0.98f  /* Peso del giroscopio en el filtro  */

/* ===========================================================================
 *  PULSADOR / ISR
 * ==========================================================================*/
#define BUTTON_DEBOUNCE_US     300000 /* Antirrebote 300 ms en microsegundos */

/* ===========================================================================
 *  FreeRTOS
 * ==========================================================================*/
#define IMU_QUEUE_LENGTH       5      /* Capacidad de la cola de angulos   */

#define MONITOR_PERIOD_MS      200    /* Periodo de impresion del monitor  */
#define ACTUATOR_QUEUE_TIMEOUT_MS 100 /* Timeout de espera en la cola      */

/* Prioridades de las tareas (mayor numero = mayor prioridad) */
#define PRIO_SENSOR            5      /* Productora  - prioridad MEDIA     */
#define PRIO_ACTUATOR          7      /* Consumidora - prioridad ALTA      */
#define PRIO_MONITOR           3      /* Monitoreo   - prioridad BAJA      */

/* Tamano de stack de las tareas (en palabras) */
#define STACK_SENSOR           4096
#define STACK_ACTUATOR         4096
#define STACK_MONITOR          3072

#endif /* CONFIG_H */
