/**
 * cmsis_os2_impl.c  –  Implementación mock de CMSIS-RTOS v2 para SIL
 *
 * Tick simulado (avanzado por osDelay, no en tiempo real).
 * Mutex: no-op (SIL es single-threaded).
 * Message Queue: ring buffer con malloc – comportamiento FIFO idéntico al real.
 * canRxQueueHandle / canTxQueueHandle: definidos aquí e inicializados
 *   en SIL_RTOS_Init() que debe llamarse antes de Test_IntegrationRunAll().
 */

#include "cmsis_os2.h"
#include "can.h"          /* canRxQueueHandle, canTxQueueHandle, can_qitem16_t */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* =========================================================================
   TICK SIMULADO
   ====================================================================== */

static uint32_t s_tick_ms = 0;

uint32_t osKernelGetTickCount(void)   { return s_tick_ms; }
uint32_t osKernelGetTickFreq(void)    { return 1000u; }   /* 1 tick = 1 ms */

void osDelay(uint32_t ticks)          { s_tick_ms += ticks; }

void osDelayUntil(uint32_t ticks)
{
    if (ticks > s_tick_ms) s_tick_ms = ticks;
}

void SIL_ResetTick(void)              { s_tick_ms = 0; }
void SIL_AdvanceTick(uint32_t ms)     { s_tick_ms += ms; }

/* =========================================================================
   THREADS (stubs – SIL no crea hilos reales)
   ====================================================================== */

void           osThreadExit(void)                                      { /* no-op */ }
osStatus_t     osThreadSuspend(osThreadId_t t)     { (void)t; return osOK; }
osThreadId_t   osThreadNew(void (*f)(void*), void *a, const osThreadAttr_t *at)
                                                   { (void)f; (void)a; (void)at; return NULL; }

/* =========================================================================
   MUTEX  (no-op en single-threaded SIL)
   ====================================================================== */

typedef struct { int dummy; } sil_mutex_t;

osMutexId_t osMutexNew(const osMutexAttr_t *attr)
{
    (void)attr;
    sil_mutex_t *m = (sil_mutex_t *)malloc(sizeof(sil_mutex_t));
    if (!m) return NULL;
    m->dummy = 0;
    return (osMutexId_t)m;
}

osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout)
{
    (void)mutex_id; (void)timeout;
    return osOK;
}

osStatus_t osMutexRelease(osMutexId_t mutex_id)
{
    (void)mutex_id;
    return osOK;
}

osStatus_t osMutexDelete(osMutexId_t mutex_id)
{
    if (mutex_id) free(mutex_id);
    return osOK;
}

/* =========================================================================
   MESSAGE QUEUE  (ring buffer)
   ====================================================================== */

typedef struct {
    uint8_t  *data;       /* datos contiguos: capacity * msg_size bytes      */
    uint32_t  msg_size;
    uint32_t  capacity;
    uint32_t  head;       /* índice de lectura (siguiente Get)               */
    uint32_t  tail;       /* índice de escritura (siguiente Put)             */
    uint32_t  count;      /* número de elementos actualmente en la cola      */
} sil_queue_t;

osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size,
                                     const osMessageQueueAttr_t *attr)
{
    (void)attr;
    sil_queue_t *q = (sil_queue_t *)calloc(1, sizeof(sil_queue_t));
    if (!q) return NULL;

    q->data     = (uint8_t *)calloc(msg_count, msg_size);
    if (!q->data) { free(q); return NULL; }

    q->msg_size = msg_size;
    q->capacity = msg_count;
    q->head     = 0;
    q->tail     = 0;
    q->count    = 0;
    return (osMessageQueueId_t)q;
}

osStatus_t osMessageQueuePut(osMessageQueueId_t mq_id, const void *msg_ptr,
                              uint8_t msg_prio, uint32_t timeout)
{
    (void)msg_prio; (void)timeout;
    sil_queue_t *q = (sil_queue_t *)mq_id;
    if (!q || !msg_ptr) return osErrorParameter;
    if (q->count >= q->capacity) return osErrorResource;   /* Cola llena */

    memcpy(q->data + q->tail * q->msg_size, msg_ptr, q->msg_size);
    q->tail = (q->tail + 1) % q->capacity;
    q->count++;
    return osOK;
}

osStatus_t osMessageQueueGet(osMessageQueueId_t mq_id, void *msg_ptr,
                              uint8_t *msg_prio, uint32_t timeout)
{
    (void)msg_prio; (void)timeout;
    sil_queue_t *q = (sil_queue_t *)mq_id;
    if (!q || !msg_ptr) return osErrorParameter;
    if (q->count == 0) return osErrorResource;              /* Cola vacía  */

    memcpy(msg_ptr, q->data + q->head * q->msg_size, q->msg_size);
    q->head = (q->head + 1) % q->capacity;
    q->count--;
    return osOK;
}

uint32_t osMessageQueueGetCount(osMessageQueueId_t mq_id)
{
    sil_queue_t *q = (sil_queue_t *)mq_id;
    return q ? q->count : 0u;
}

uint32_t osMessageQueueGetSpace(osMessageQueueId_t mq_id)
{
    sil_queue_t *q = (sil_queue_t *)mq_id;
    return q ? (q->capacity - q->count) : 0u;
}

osStatus_t osMessageQueueReset(osMessageQueueId_t mq_id)
{
    sil_queue_t *q = (sil_queue_t *)mq_id;
    if (!q) return osErrorParameter;
    q->head = q->tail = q->count = 0;
    return osOK;
}

/* =========================================================================
   HANDLES GLOBALES CAN  (definidos en freertos.c en el build real)
   En SIL se definen aquí y se inicializan en SIL_RTOS_Init().
   ====================================================================== */

osMessageQueueId_t canRxQueueHandle = NULL;
osMessageQueueId_t canTxQueueHandle = NULL;

/**
 * @brief  Crea las colas CAN y el mutex global.
 *         Llamar UNA VEZ antes de Test_IntegrationRunAll().
 */
void SIL_RTOS_Init(void)
{
    /* Recrear si ya existían (entre test runs) */
    canRxQueueHandle = osMessageQueueNew(128u, sizeof(can_qitem16_t), NULL);
    canTxQueueHandle = osMessageQueueNew(64u,  sizeof(can_qitem16_t), NULL);

    /* g_inMutex se define en app_state.c; se inicializa aquí */
    extern osMutexId_t g_inMutex;
    if (!g_inMutex) {
        g_inMutex = osMutexNew(NULL);
    }

    SIL_ResetTick();
}
