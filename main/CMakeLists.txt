# Registro del componente principal con todos los archivos fuente.
idf_component_register(
    SRCS
        "main.c"
        "mpu6050.c"
        "servos.c"
        "tasks.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver             # i2c legacy + gpio
        esp_driver_gpio    # driver/gpio.h en IDF >= 5.x
        esp_driver_mcpwm   # driver MCPWM nuevo (mcpwm_prelude.h)
        esp_timer          # esp_timer_get_time() para el antirrebote
)
