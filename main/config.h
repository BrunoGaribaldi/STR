/* ============================================================================
 *  config.h
 *  Sistema Controlador de Vuelo en Tiempo Real para Ala Volante (Alerones)
 *  Materia: Sistemas en Tiempo Real - Ingenieria Informatica
 *
 *  >>> MATERIAL DE ESTUDIO: este proyecto esta explicado desde cero en el
 *      archivo RESUMEN_INTEGRAL.md. A lo largo del codigo, las marcas
 *      [Guia §X] apuntan a la seccion de esa guia que explica ese concepto.
 *      Este archivo (parametros ajustables) se corresponde con [Guia §IV.4].
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
 *  BUS I2C / MPU6050                                          [Guia §II.9.a]
 *  I2C = bus serie de 2 cables (SDA datos, SCL reloj), maestro-esclavo.
 *  OJO: los 400 kHz son la velocidad del CABLE, distinta de los 50 Hz de
 *  muestreo (cada cuanto pedimos una lectura, ver SENSOR_PERIOD_MS).
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

/* Factores de escala (datasheet MPU6050).                    [Guia §II.9.a "Paso 3"]
 * El sensor da enteros "crudos"; dividir por estos factores los convierte a
 * unidades fisicas. 16384 = 32767/2 (rango +-2g);  131 = 32767/250 (+-250 dps). */
#define ACCEL_SCALE_2G         16384.0f    /* LSB/g    a rango +-2g       */
#define GYRO_SCALE_250DPS      131.0f      /* LSB/dps  a rango +-250 dps  */

/* ===========================================================================
 *  SERVOS (MCPWM)                                             [Guia §II.9.c]
 *  PWM: pulso cada 20 ms (50 Hz); el ANCHO del pulso define el angulo:
 *  1000us=-90 grados, 1500us=centro, 2000us=+90 grados.
 * ==========================================================================*/
#define SERVO1_ZERO_US         1500   /* <-- AJUSTE FISICO: neutro Servo 1 */
#define SERVO2_ZERO_US         1500   /* <-- AJUSTE FISICO: neutro Servo 2 */

#define SERVO_MIN_US           1000   /* Pulso minimo (-90 grados)         */
#define SERVO_MAX_US           2000   /* Pulso maximo (+90 grados)         */

#define SERVO_PWM_FREQ_HZ      50     /* Frecuencia PWM servos (periodo 20 ms) */

/* Sentido de giro de cada servo en la correccion.            [Guia §II.9.c]
 * Los dos servos se mueven MECANICAMENTE OPUESTOS (para corregir el alabeo),
 * pero reciben el mismo signo porque el Servo2 esta montado ESPEJADO.
 * Si un servo se mueve al REVES de lo esperado, invertir su signo (+1.0 <-> -1.0).
 *   Servo1 (D16): correcto -> +1.0
 *   Servo2 (D17): invertido fisicamente -> +1.0 (mismo sentido que Servo1) */
#define SERVO1_DIR             (+1.0f)
#define SERVO2_DIR             (+1.0f)

/* ===========================================================================
 *  LOGICA DE CONTROL                                          [Guia §IV.3]
 *  Control Proporcional (P): correccion = error_grados * CONTROL_GAIN.
 * ==========================================================================*/
#define CONTROL_GAIN           10.0f  /* Ganancia us por grado de error    */

/* ===========================================================================
 *  FILTRO COMPLEMENTARIO Y MUESTREO                           [Guia §II.8]
 *  Fusiona giroscopio (98%) y acelerometro (2%). SENSOR_DT_S se deriva del
 *  periodo, asi que si cambias SENSOR_PERIOD_MS el filtro se ajusta solo.
 * ==========================================================================*/
#define SENSOR_PERIOD_MS       20     /* 50 Hz de muestreo del IMU         */
#define SENSOR_DT_S            (SENSOR_PERIOD_MS / 1000.0f)  /* dt en seg   */
#define COMP_ALPHA             0.98f  /* Peso del giroscopio en el filtro  */

/* ===========================================================================
 *  PULSADOR / ISR                                             [Guia §II.5]
 *  Antirrebote: ignora toques que llegan < 300 ms despues del ultimo.
 * ==========================================================================*/
#define BUTTON_DEBOUNCE_US     300000 /* Antirrebote 300 ms en microsegundos */

/* ===========================================================================
 *  FreeRTOS                                          [Guia §II.4 y §II.6.c]
 * ==========================================================================*/
#define IMU_QUEUE_LENGTH       5      /* Capacidad de la cola de angulos   */

/* Telemetria $ATT a 50 Hz (mismo ritmo que el sensor) para que el attitude
 * indicator de la PC se mueva fluido. Los logs humanos NO hacen falta tan
 * seguido: se emiten cada N tramas (decimacion) para no saturar el UART. */
#define TELEMETRY_PERIOD_MS    20     /* Trama $ATT cada 20 ms (50 Hz)     */
#define MONITOR_LOG_DECIMATION 10     /* Logs humanos cada 10 tramas=200ms */
#define ACTUATOR_QUEUE_TIMEOUT_MS 100 /* Timeout de espera en la cola      */

/* Prioridades (mayor numero = mayor prioridad).  [Guia §II.4]
 * El ACTUADOR (consumidor) tiene MAS prioridad que el SENSOR (productor)
 * para que la correccion ocurra apenas hay un dato nuevo. */
#define PRIO_SENSOR            5      /* Productora  - prioridad MEDIA     */
#define PRIO_ACTUATOR          7      /* Consumidora - prioridad ALTA      */
#define PRIO_MONITOR           3      /* Monitoreo   - prioridad BAJA      */

/* Tamano de stack de las tareas (en palabras) */
#define STACK_SENSOR           4096
#define STACK_ACTUATOR         4096
#define STACK_MONITOR          3072

#endif /* CONFIG_H */
