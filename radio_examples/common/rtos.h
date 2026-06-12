// rtos.h — small FreeRTOS scaffolding shared by the radio_examples tools.
//
// Provides a static-allocation task-creation helper and (in rtos.cpp) the
// FreeRTOS static-allocation memory callbacks every binary needs once when
// configSUPPORT_STATIC_ALLOCATION is enabled.
#pragma once

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

// xTaskCreateStatic wrapper that panics with a diagnostic on failure and
// returns the handle (so callers can pin it with vTaskCoreAffinitySet).
TaskHandle_t rtos_task_create( TaskFunction_t fn,
                               const char*    name,
                               uint32_t       stack_depth,
                               void*          param,
                               UBaseType_t    priority,
                               StackType_t*   stack,
                               StaticTask_t*  tcb );

#ifdef __cplusplus
}
#endif
