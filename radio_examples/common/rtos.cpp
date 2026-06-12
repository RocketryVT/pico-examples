// rtos.cpp — see rtos.h.

#include "rtos.h"

#include "pico/platform/panic.h"

TaskHandle_t rtos_task_create( TaskFunction_t fn,
                               const char*    name,
                               uint32_t       stack_depth,
                               void*          param,
                               UBaseType_t    priority,
                               StackType_t*   stack,
                               StaticTask_t*  tcb )
{
    TaskHandle_t h = xTaskCreateStatic( fn, name, stack_depth, param,
                                        priority, stack, tcb );
    if ( !h ) {
        panic( "[rtos] xTaskCreateStatic failed: '%s' stack=%u pri=%u",
               name, (unsigned)stack_depth, (unsigned)priority );
    }
    return h;
}

// -- FreeRTOS static-allocation callbacks --------------------------------------
// Required once per binary because configSUPPORT_STATIC_ALLOCATION == 1. Defined
// here so every example links them without duplicating boilerplate in main.cpp.
extern "C" {

void vApplicationGetIdleTaskMemory( StaticTask_t** ppxIdleTaskTCBBuffer,
                                    StackType_t**  ppxIdleTaskStackBuffer,
                                    uint32_t*      pulIdleTaskStackSize )
{
    static StaticTask_t idle_tcb;
    static StackType_t  idle_stack[ configMINIMAL_STACK_SIZE ];
    *ppxIdleTaskTCBBuffer   = &idle_tcb;
    *ppxIdleTaskStackBuffer =  idle_stack;
    *pulIdleTaskStackSize   =  configMINIMAL_STACK_SIZE;
}

void vApplicationGetPassiveIdleTaskMemory( StaticTask_t** ppxIdleTaskTCBBuffer,
                                           StackType_t**  ppxIdleTaskStackBuffer,
                                           uint32_t*      pulIdleTaskStackSize,
                                           BaseType_t     xPassiveIdleTaskIndex )
{
    static StaticTask_t passive_tcb  [ configNUMBER_OF_CORES - 1 ];
    static StackType_t  passive_stack[ configNUMBER_OF_CORES - 1 ]
                                     [ configMINIMAL_STACK_SIZE ];
    *ppxIdleTaskTCBBuffer   = &passive_tcb  [ xPassiveIdleTaskIndex ];
    *ppxIdleTaskStackBuffer =  passive_stack[ xPassiveIdleTaskIndex ];
    *pulIdleTaskStackSize   =  configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory( StaticTask_t** ppxTimerTaskTCBBuffer,
                                     StackType_t**  ppxTimerTaskStackBuffer,
                                     uint32_t*      pulTimerTaskStackSize )
{
    static StaticTask_t timer_tcb;
    static StackType_t  timer_stack[ configTIMER_TASK_STACK_DEPTH ];
    *ppxTimerTaskTCBBuffer   = &timer_tcb;
    *ppxTimerTaskStackBuffer =  timer_stack;
    *pulTimerTaskStackSize   =  configTIMER_TASK_STACK_DEPTH;
}

}  // extern "C"
