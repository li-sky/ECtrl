#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

typedef struct {
	uint8_t TargetID, SourceID, Param, OpType;
	uint8_t Data[8];
} CANMsg;

void vCanManageTaskFunction( void *pvParameters );

void TransmitCanMessage( uint8_t targetID, uint8_t sourceID, uint8_t param, uint8_t optype, uint8_t* data);

#ifdef __cplusplus
}

#include "string"
#include "map"
#include "memory"
#include "TaskStateManage.h"

extern std::shared_ptr<Action> CurrentAction;
extern ActionFactory actionFactory;
extern std::queue<std::shared_ptr<Action>> MessageQueue;

class BaseMessageHandler {
	public:
		std::string MessageHandlerID = "GenericMessageHandler";
		uint8_t MessageID = 0xFF;
		virtual void Handler(const CANMsg& msg) = 0;
		virtual ~BaseMessageHandler() = default;
};

class ReadStateMessageHandler : public BaseMessageHandler {
	public:
		ReadStateMessageHandler();
		void Handler(const CANMsg& msg) override;
};

class WriteStateMessageHandler : public BaseMessageHandler {
	public:
		WriteStateMessageHandler();
		void Handler(const CANMsg& msg) override;
};

class AddToQueueMessageHandler : public BaseMessageHandler {
	public:
		AddToQueueMessageHandler();
		void Handler(const CANMsg& msg) override;
};

class ClearQueueMessageHandler : public BaseMessageHandler {
	public:
		ClearQueueMessageHandler();
		void Handler(const CANMsg& msg) override;
};

class PopQueueMessageHandler : public BaseMessageHandler {
	public:
		PopQueueMessageHandler();
		void Handler(const CANMsg& msg) override;
};

class DirectOperationMessageHandler : public BaseMessageHandler {
	public:
		DirectOperationMessageHandler();
		void Handler(const CANMsg& msg) override;
};

class MessageHandlerFactory {
public:
    void RegisterHandler(uint8_t messageID, std::shared_ptr<BaseMessageHandler> handler);

    std::shared_ptr<BaseMessageHandler> GetHandler(uint8_t messageID);

private:
    std::map<uint8_t, std::shared_ptr<BaseMessageHandler>> handlers;
};


#endif