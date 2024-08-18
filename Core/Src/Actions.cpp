#include "TaskStateManage.h"
#include "TaskCanManage.h"
#include "TaskActuator.h"

#include "functional"

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim1, htim8;
float difference[4], integral[4];

void writePWMValue(uint8_t index, int16_t speed) {
    uint32_t TIM_CHANNEL;
    TIM_HandleTypeDef *htim;
    switch (index) {
        case 0:
            TIM_CHANNEL = TIM_CHANNEL_1;
            htim = &htim1;
            break;
        case 1:
            TIM_CHANNEL = TIM_CHANNEL_3;
            htim = &htim1;
            break;
        case 2:
            TIM_CHANNEL = TIM_CHANNEL_1;
            htim = &htim8;
            break;
        case 3:
            TIM_CHANNEL = TIM_CHANNEL_3;
            htim = &htim8;
            break;
        default:
            printf("Invalid index\n");
            return;
    }
    if (speed > 0) {
        __HAL_TIM_SetCompare(htim, TIM_CHANNEL, speed);
        HAL_TIM_PWM_Start(htim, TIM_CHANNEL);
        HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL);
    } else {
        __HAL_TIM_SetCompare(htim, TIM_CHANNEL, GetPWMARR() - speed);
        HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL);
        HAL_TIM_PWM_Stop(htim, TIM_CHANNEL);
    }
}


void ActionFactory::RegisterAction(uint8_t command, std::function<std::shared_ptr<Action>()> createFunc) {
    actions[command] = [createFunc]() -> std::shared_ptr<Action> {
        return createFunc();
    };
}

std::shared_ptr<Action> ActionFactory::GetAction(uint8_t command) {
    if (actions.find(command) != actions.end()) {
        return actions[command]();
    }
    return nullptr;
}


void ActionWritePWMValues::Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction) {
    (void) prevAction;
    speed[0] = (data[0] << 8) | data[1];
    speed[1] = (data[2] << 8) | data[3];
    speed[2] = (data[4] << 8) | data[5];
    speed[3] = (data[6] << 8) | data[7];
}

RunComplete ActionWritePWMValues::Update() {
    return true;
}

void ActionWriteTargetEncoderValues::Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction) {
    (void) prevAction;
    speed[0] = (data[0] << 8) | data[1];
    speed[1] = (data[2] << 8) | data[3];
    speed[2] = (data[4] << 8) | data[5];
    speed[3] = (data[6] << 8) | data[7];
    for (int i = 0; i < 4; i++) {
        difference[i] = 0;
        integral[i] = 0;
    }
}

RunComplete ActionWriteTargetEncoderValues::Update() {
    for (int i = 0; i < 4; i++) {
        difference[i] = speed[i] - GetWheelSpeed(i);
        integral[i] += difference[i];
        writePWMValue(i, GetKp(i) * difference[i] + GetKi(i) * integral[i] + GetKd(i) * (difference[i] - difference[i]));
    }
    return true;
} 

void ActionHold::Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction) {
    time = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    this->prevAction = prevAction;
}

RunComplete ActionHold::Update() {
    if (time == 0) {
        return true;
    }
    prevAction->Update();
    time-=updateDelay;
    return false;
}

void ActionStop::Initialize(std::array<uint8_t, 8> data, std::shared_ptr<Action> prevAction) {
    (void) prevAction;
    (void) data;
}

RunComplete ActionStop::Update() {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    if (abs(GetWheelSpeed(0)) < 10 && abs(GetWheelSpeed(1)) < 10 && abs(GetWheelSpeed(2)) < 10 && abs(GetWheelSpeed(3)) < 10) {
        return true;
    }
    return false;
}