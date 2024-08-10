#include "FreeRTOS.h"
#include "task.h"
#include "TaskActuator.h"

void vMotorControlFunction( void * pvParameters ) {
    (void) pvParameters;
    for (;;) {
        vTaskDelay(1000);
    }
}