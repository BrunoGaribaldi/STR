/* ============================================================================
 *  mpu6050.c
 *  Driver I2C del MPU6050 usando el driver I2C nativo (legacy) de ESP-IDF.
 *
 *  MATERIAL DE ESTUDIO -> [Guia §II.9.a] (protocolo I2C, transaccion paso a
 *  paso, reconstruccion y escalado) y [Guia §III.2] (el sensor MPU6050).
 * ==========================================================================*/

#include "mpu6050.h"
#include "config.h"

#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "MPU6050";

/* ---------------------------------------------------------------------------
 *  Escribe un byte en un registro del MPU6050.
 *  Secuencia I2C: START - addr+W - reg - data - STOP
 * ------------------------------------------------------------------------- */
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                         pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ---------------------------------------------------------------------------
 *  Lee 'len' bytes consecutivos a partir de 'reg' (lectura con repeated-start).
 *  Secuencia: START - addr+W - reg - REPEATED START - addr+R - data... - STOP
 *  Esta es la transaccion I2C explicada simbolo por simbolo en [Guia §II.9.a].
 *  El ultimo byte se lee con NACK para decirle al sensor "no quiero mas datos".
 * ------------------------------------------------------------------------- */
static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd); /* repeated start */
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &buf[len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                         pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ---------------------------------------------------------------------------
 *  Inicializa el bus I2C en modo maestro.
 * ------------------------------------------------------------------------- */
esp_err_t mpu6050_i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_GPIO,
        .scl_io_num = I2C_MASTER_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en i2c_param_config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en i2c_driver_install: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Bus I2C inicializado (SDA=%d SCL=%d @ %d Hz)",
             I2C_MASTER_SDA_GPIO, I2C_MASTER_SCL_GPIO, I2C_MASTER_FREQ_HZ);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 *  Despierta el sensor y fija los rangos de medida.            [Guia §III.2]
 *  El MPU6050 arranca "dormido"; hay que despertarlo y elegir sus rangos.
 * ------------------------------------------------------------------------- */
esp_err_t mpu6050_init(void)
{
    esp_err_t ret;

    /* PWR_MGMT_1 = 0x00 -> salir de modo sleep, reloj interno */
    ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo despertar el MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ACCEL_CONFIG = 0x00 -> rango +-2g */
    ret = mpu6050_write_reg(MPU6050_REG_ACCEL_CFG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando acelerometro: %s", esp_err_to_name(ret));
        return ret;
    }

    /* GYRO_CONFIG = 0x00 -> rango +-250 dps */
    ret = mpu6050_write_reg(MPU6050_REG_GYRO_CFG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando giroscopio: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 inicializado (acc +-2g, gyro +-250 dps)");
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 *  Lee una muestra completa (14 bytes) y la escala a unidades fisicas.
 *  Es el "Paso 3" del recorrido de datos, detallado en [Guia §II.9.a].
 *  Layout de los 14 bytes a partir de 0x3B (cada medida ocupa 2 bytes):
 *    [0..1] ACC_X  [2..3] ACC_Y  [4..5] ACC_Z
 *    [6..7] TEMP
 *    [8..9] GYRO_X [10..11] GYRO_Y [12..13] GYRO_Z
 * ------------------------------------------------------------------------- */
esp_err_t mpu6050_read(mpu6050_reading_t *out)
{
    uint8_t raw[14];
    esp_err_t ret = mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT, raw, sizeof(raw));
    if (ret != ESP_OK) {
        return ret;
    }

    /* Reconstruir enteros con signo de 16 bits (big-endian: HIGH primero).
     * El cast a (int16_t) es CLAVE para interpretar bien los valores negativos
     * (complemento a dos). Ver explicacion en [Guia §II.9.a "Paso 3"]. */
    int16_t acc_x = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t acc_y = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t acc_z = (int16_t)((raw[4]  << 8) | raw[5]);
    /* raw[6..7] es temperatura y raw[12..13] es GYRO_Z: no se usan, se ignoran */
    int16_t gyr_x = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gyr_y = (int16_t)((raw[10] << 8) | raw[11]);

    out->acc_x  = acc_x / ACCEL_SCALE_2G;
    out->acc_y  = acc_y / ACCEL_SCALE_2G;
    out->acc_z  = acc_z / ACCEL_SCALE_2G;
    out->gyro_x = gyr_x / GYRO_SCALE_250DPS;
    out->gyro_y = gyr_y / GYRO_SCALE_250DPS;

    return ESP_OK;
}
