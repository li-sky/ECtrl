#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "TaskCanManage.h"
#ifdef __cplusplus
extern "C" {
#endif


QueueHandle_t MessageQueue;
CANMsg ActiveCANMsg;

#ifdef __cplusplus
}


class Controller {
    public:
        virtual ~Controller() = default;
        virtual void Init();
        virtual void Update();
};

class SimplePWMSetController : Controller {
    public:
        void Init() override;
        void Update() override;
};

#endif
