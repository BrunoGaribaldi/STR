/* ============================================================================
 *  tasks.h
 *  Declaracion de las tareas FreeRTOS, la ISR y los objetos de sincronizacion
 *  compartidos del sistema de control de vuelo.
 * ==========================================================================*/

#ifndef TASKS_H
#define TASKS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* Struct compartida con los angulos filtrados publicados por el sensor. */
typedef struct {
    float roll;
    float pitch;
} imu_data_t;

/* ---- Objetos de sincronizacion globales (definidos en tasks.c) ---- */
extern QueueHandle_t     imu_queue;    /* Cola de imu_data_t, capacidad 5   */
extern SemaphoreHandle_t zero_mutex;   /* Protege zero_roll / zero_pitch    */

/* Crea la cola, el mutex e instala la ISR del pulsador.
 * Debe llamarse desde app_main ANTES de crear las tareas. */
void tasks_sync_init(void);

/* Crea las tres tareas FreeRTOS del sistema. */
void tasks_start(void);

#endif /* TASKS_H */
