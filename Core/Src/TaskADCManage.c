#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "TaskADCManage.h"
#include "TaskStateManage.h"

extern ADC_HandleTypeDef hadc1;

void vADCManageFunction(void *pvParameters) {
    (void) pvParameters;
    HAL_ADCEx_Calibration_Start(&hadc1);
    waitReady();
    HAL_ADC_Start(&hadc1);
    uint32_t adcValue = 0;
    while (1) {
        if (HAL_ADC_PollForConversion(&hadc1, 0) != HAL_OK) {
            vTaskDelay(1);
        } else {
            adcValue =  HAL_ADC_GetValue(&hadc1);
            vTaskDelay(100);
            SetCurrentInputVoltage(((float) adcValue / (4095)) * 3.3);
        }
    }
}