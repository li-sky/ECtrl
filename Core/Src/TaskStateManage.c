#include "TaskStateManage.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "semphr.h"

#define FLASH_ERASE_NO_ERR 0xFFFFFFFF

static const uint32_t* storageAddress = (uint32_t*)0x0807F800;
extern CRC_HandleTypeDef hcrc;

static uint8_t ParamReady = 0;


void waitReady() {
    while(!ParamReady)
        vTaskDelay(1);
}


static struct GlobalParam_NV_t {
    uint8_t boardCANID, hostCANID;
    uint16_t PWMPSC, PWMARR;
    float Kp[4], Ki[4], Kd[4];
} GlobalParam_NV;

static struct GlobalParam_V_t {
    uint8_t runMode, currentCommand, queueLength;
    int16_t wheelSpeed[4];
    int32_t targetDistance;
    float currentInputVoltage;
} GlobalParam_V;

static void InitGlobalParamNV() {
    GlobalParam_NV.boardCANID = 0x0A;
    GlobalParam_NV.PWMPSC = 1;
    GlobalParam_NV.PWMARR = 35999;
    GlobalParam_NV.hostCANID = 0x2B;
    for (uint8_t i = 0; i < 4; i++) {
        GlobalParam_NV.Kp[i] = 0.0;
        GlobalParam_NV.Ki[i] = 0.0;
        GlobalParam_NV.Kd[i] = 0.0;
    }
}

static void InitGlobalParamV() {
    GlobalParam_V.runMode = 0x00;
    GlobalParam_V.queueLength = 0x00;
    GlobalParam_V.targetDistance = 0;
    GlobalParam_V.currentCommand = 0xFF;
    GlobalParam_V.currentInputVoltage = 24.0;
    memset(GlobalParam_V.wheelSpeed, 0, sizeof(GlobalParam_V.wheelSpeed));
}

static void UpdateNVParamToFlash(uint32_t crc) {
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = (uint32_t) storageAddress;
    EraseInitStruct.NbPages = 1;
    uint32_t err;
    HAL_FLASHEx_Erase(&EraseInitStruct, &err);
    if (err != FLASH_ERASE_NO_ERR)
    {
        printf("Error Erasing Flash!\r\n");
        return;
    }
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) storageAddress, crc);
    for (uint32_t current = 0; current < sizeof(GlobalParam_NV) / 2; current++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) (storageAddress + current + 1), *(((uint32_t*) &(GlobalParam_NV)) + current));
    }
}

void vStoreManageFunction ( void *pvParameters ) {
    (void) pvParameters;
    InitGlobalParamV();
    printf("Unlocking Flash...\r\n");
    HAL_FLASH_Unlock();
    // Read from storage address and make sure there data is not corrupted in flash area.
    printf("Initiating CRC Check...\r\n");
    uint32_t crc_result = HAL_CRC_Calculate(&hcrc, (uint32_t*) (storageAddress + 1), sizeof(GlobalParam_NV) / 4);
    printf("CRC Calculation Complete, result = %lx\r\n", crc_result);
    // If CRC is valid, then data is not corrupted.
    if (crc_result == *storageAddress)
    {
        // CRC Check Pass
        printf("CRC Check Pass, data is valid.\r\n");
        memcpy(&GlobalParam_NV, storageAddress + 1, sizeof(GlobalParam_NV));
    } else {
        // CRC Check Fail
        printf("CRC Check Fail, data is corrupted. Resetting...\r\n");
        InitGlobalParamNV();
        uint32_t crc_result_2 = HAL_CRC_Calculate(&hcrc, (uint32_t*)&GlobalParam_NV, sizeof(GlobalParam_NV) / 4);
        UpdateNVParamToFlash(crc_result_2);
    }
    ParamReady = 1;
    vTaskDelete(NULL);
}

uint8_t GetboardCANID() {
    waitReady();
    return GlobalParam_NV.boardCANID;
}

void SetBoardCANID(uint8_t newCANID) {
    waitReady();
    GlobalParam_NV.boardCANID = newCANID;
    uint32_t crc_result = HAL_CRC_Calculate(&hcrc, (uint32_t*)&GlobalParam_NV, sizeof(GlobalParam_NV) / 4);
    UpdateNVParamToFlash(crc_result);
}

uint8_t GetHostCANID() {
    waitReady();
    return GlobalParam_NV.hostCANID;
}

void SetHostCANID(uint8_t newCANID) {
    waitReady();
    GlobalParam_NV.hostCANID = newCANID;
    uint32_t crc_result = HAL_CRC_Calculate(&hcrc, (uint32_t*)&GlobalParam_NV, sizeof(GlobalParam_NV) / 4);
    UpdateNVParamToFlash(crc_result);
}

void SetPWMPSC(uint16_t PSC) {
    waitReady();
    GlobalParam_NV.PWMPSC = PSC;
    uint32_t crc_result = HAL_CRC_Calculate(&hcrc, (uint32_t*)&GlobalParam_NV, sizeof(GlobalParam_NV) / 4);
    UpdateNVParamToFlash(crc_result);
}

uint16_t GetPWMPSC() {
    waitReady();
    return GlobalParam_NV.PWMPSC;
}

void SetPWMARR(uint16_t ARR) {
    waitReady();
    GlobalParam_NV.PWMARR = ARR;
    uint32_t crc_result = HAL_CRC_Calculate(&hcrc, (uint32_t*)&GlobalParam_NV, sizeof(GlobalParam_NV) / 4);
    UpdateNVParamToFlash(crc_result);
}

uint16_t GetPWMARR() {
    waitReady();
    return GlobalParam_NV.PWMARR;
}


void SetKp(uint8_t index, float value) {
    waitReady();
    if (index < 4) {
        GlobalParam_NV.Kp[index] = value;
        uint32_t crc_result = HAL_CRC_Calculate(&hcrc, (uint32_t*)&GlobalParam_NV, sizeof(GlobalParam_NV) / 4);
        UpdateNVParamToFlash(crc_result);
    }
}

float GetKp(uint8_t index) {
    waitReady();
    if (index < 4) {
        return GlobalParam_NV.Kp[index];
    }
    return 0.0f; 
}

void SetKi(uint8_t index, float value) {
    waitReady();
    if (index < 4) {
        GlobalParam_NV.Ki[index] = value;
        uint32_t crc_result = HAL_CRC_Calculate(&hcrc, (uint32_t*)&GlobalParam_NV, sizeof(GlobalParam_NV) / 4);
        UpdateNVParamToFlash(crc_result);
    }
}

float GetKi(uint8_t index) {
    waitReady();
    if (index < 4) {
        return GlobalParam_NV.Ki[index];
    }
    return 0.0f; 
}

void SetKd(uint8_t index, float value) {
    waitReady();
    if (index < 4) {
        GlobalParam_NV.Kd[index] = value;
        uint32_t crc_result = HAL_CRC_Calculate(&hcrc, (uint32_t*)&GlobalParam_NV, sizeof(GlobalParam_NV) / 4);
        UpdateNVParamToFlash(crc_result);
    }
}

float GetKd(uint8_t index) {
    waitReady();
    if (index < 4) {
        return GlobalParam_NV.Kd[index];
    }
    return 0.0f; 
}

void SetRunMode(uint8_t mode) {
    waitReady();
    GlobalParam_V.runMode = mode;
}

uint8_t GetRunMode() {
    waitReady();
    return GlobalParam_V.runMode;
}

void SetCurrentCommand(uint8_t command) {
    waitReady();
    GlobalParam_V.currentCommand = command;
}

uint8_t GetCurrentCommand() {
    waitReady();
    return GlobalParam_V.currentCommand;
}

void SetQueueLength(uint8_t length) {
    waitReady();
    GlobalParam_V.queueLength = length;
}

uint8_t GetQueueLength() {
    waitReady();
    return GlobalParam_V.queueLength;
}

void SetWheelSpeed(uint8_t index, int16_t speed) {
    waitReady();
    if (index < 4) {
        GlobalParam_V.wheelSpeed[index] = speed;
    }
}

int16_t GetWheelSpeed(uint8_t index) {
    waitReady();
    if (index < 4) {
        return GlobalParam_V.wheelSpeed[index];
    }
    return 0; 
}

void SetTargetDistance(int32_t distance) {
    waitReady();
    GlobalParam_V.targetDistance = distance;
}

int32_t GetTargetDistance() {
    waitReady();
    return GlobalParam_V.targetDistance;
}

void SetCurrentInputVoltage(float voltage) {
    waitReady();
    GlobalParam_V.currentInputVoltage = voltage;
}

float GetCurrentInputVoltage() {
    waitReady();
    return GlobalParam_V.currentInputVoltage;
}
