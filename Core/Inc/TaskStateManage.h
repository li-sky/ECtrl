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

#include <array>
#include <memory>

using RunComplete = bool;

class Action {
    public:
        ~Action() = default;
        virtual void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction) = 0;
        virtual RunComplete Update() = 0;
    private:
        uint8_t command;
};

class ActionStraightWithDesignatedTime : public Action {
    public:
        ActionStraightWithDesignatedTime() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed;
        uint32_t time;
};

class ActionStraightWithDesignatedDistance : public Action {
    public:
        ActionStraightWithDesignatedDistance() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed;
        int32_t distance;
};

class ActionStraight : public Action {
    public:
        ActionStraight() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed;
};

class ActionNeutralRotateWithDesignatedAngle : public Action {
    public:
        ActionNeutralRotateWithDesignatedAngle() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed;
        int16_t angle;
};

class ActionNeutralRotateWithDesignatedTime : public Action {
    public:
        ActionNeutralRotateWithDesignatedTime() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed;
        uint32_t time;
};

class ActionNeutralRotate : public Action {
    public:
        ActionNeutralRotate() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed;
};

class ActionPivotTurnWithDesignatedAngle : public Action {
    public:
        ActionPivotTurnWithDesignatedAngle() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed;
        int16_t angle;
};

class ActionPivotTurnWithDesignatedTime : public Action {
    public:
        ActionPivotTurnWithDesignatedTime() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed;
        uint32_t time;
};

class ActionPivotTurn : public Action {
    public:
        ActionPivotTurn() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed;
};

class ActionPowerTurnWithDesignatedAngle : public Action {
    public:
        ActionPowerTurnWithDesignatedAngle() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed;
        int16_t angle;
};

class ActionPowerTurnWithDesignatedTime : public Action {
    public:
        ActionPowerTurnWithDesignatedTime() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed;
        uint32_t time;
};

class ActionPowerTurn : public Action {
    public:
        ActionPowerTurn() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed;
};

class ActionWritePWMValues: public Action {
    public:
        ActionWritePWMValues() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed[4];
};

class ActionWriteTargetEncoderValues: public Action {
    public:
        ActionWriteTargetEncoderValues() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        int16_t speed[4];
};

class ActionFastDecay : public Action {
    public:
        ActionFastDecay() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
};

class ActionSlowDecay : public Action {
    public:
        ActionSlowDecay() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
};

class ActionStop : public Action {
    public:
        ActionStop() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
};

class ActionHold : public Action {
    public:
        ActionHold() = default ;
        void Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction);
        RunComplete Update() override;
    private:
        uint32_t time;
        std::shared_ptr<Action> prevAction;
};


#include <queue>
#include <map>
#include <memory>
#include <functional>
extern std::queue<std::shared_ptr<Action>> MessageQueue;

using CreateHandlerFunc = std::shared_ptr<Action>(*)();

class ActionFactory {
public:
    void RegisterAction(uint8_t command, std::function<std::shared_ptr<Action>()> createFunc);
    std::shared_ptr<Action> GetAction(uint8_t command);
private:
    std::map<uint8_t, std::function<std::shared_ptr<Action>()>> actions;
};

#endif