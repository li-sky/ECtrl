#pragma once

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif
void vStoreManageFunction ( void *pvParameters );
void waitReady();

uint8_t GetboardCANID();
void SetBoardCANID(uint8_t newCANID);
uint8_t GetHostCANID();
void SetHostCANID(uint8_t newCANID);
void SetPWMPSC(uint16_t PSC);
uint16_t GetPWMPSC();
void SetPWMARR(uint16_t ARR);
uint16_t GetPWMARR();
void SetKp(uint8_t index, float value);
float GetKp(uint8_t index) ;
void SetKi(uint8_t index, float value);
float GetKi(uint8_t index) ;
void SetKd(uint8_t index, float value);
float GetKd(uint8_t index);
void SetRunMode(uint8_t mode);
uint8_t GetRunMode();
void SetCurrentCommand(uint8_t command);
uint8_t GetCurrentCommand();
void SetQueueLength(uint8_t length);
uint8_t GetQueueLength();
void SetWheelSpeed(uint8_t index, int16_t speed);
int16_t GetWheelSpeed(uint8_t index);
void SetTargetDistance(int32_t distance);
int32_t GetTargetDistance();
void SetCurrentInputVoltage(float voltage);
float GetCurrentInputVoltage();

#ifdef __cplusplus
}

class Action {
    public:
        virtual ~Action() = default;
        virtual void Execute() = 0;
    private:
        uint8_t command;
        uint8_t 
};

#include <queue>
std::queue<Action> MessageQueue;

#endif