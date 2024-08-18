#include "FreeRTOS.h"
#include "task.h"
#include "TaskActuator.h"
#include "TaskStateManage.h"
#include "stm32f1xx_hal.h"
#include "TaskCanManage.h"
#include "stdio.h"

uint16_t updateDelay = 100;
extern TIM_HandleTypeDef htim2, htim3, htim4, htim5;


void UpdateEncoderValue() {
    SetWheelSpeed(0, __HAL_TIM_GET_COUNTER(&htim2));
    SetWheelSpeed(1, __HAL_TIM_GET_COUNTER(&htim3));
    SetWheelSpeed(2, __HAL_TIM_GET_COUNTER(&htim4));
    SetWheelSpeed(3, __HAL_TIM_GET_COUNTER(&htim5));
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    printf("Encoder Value: %d %d %d %d\n", GetWheelSpeed(0), GetWheelSpeed(1), GetWheelSpeed(2), GetWheelSpeed(3));
}

void vMotorControlFunction( void * pvParameters ) {
    (void) pvParameters;
    for (;;) {
        UpdateEncoderValue();
        if (GetRunMode() == 0x00)
        {
        }
        else if (GetRunMode() == 0x01) {
                CurrentAction->Update();
            } else {
                if (CurrentAction->Update() == true) {
                    if (!MessageQueue.empty()) {
                        CurrentAction = MessageQueue.front();
                        MessageQueue.pop();
                    }
                }
            }
        vTaskDelay(updateDelay);
    }
}