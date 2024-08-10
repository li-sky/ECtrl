#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "string.h"
#include "TaskCanManage.h"
#include "TaskStateManage.h"

#include "iostream"
#include "string"
extern CAN_HandleTypeDef hcan;
CAN_FilterTypeDef Filter;
TaskHandle_t vCanSendTaskHandle;
CAN_TxHeaderTypeDef Header;

static QueueHandle_t canRecvMsgQueue;
extern "C" {
extern void Error_Handler(void);
}

void TransmitCanMessage( CANMsg *Message) {
	Header.ExtId = (Message->TargetID + (Message->SourceID << 8) + (Message->Param << 16)
		               + (Message->OpType << 24)) & 0x1FFFFFFF;
	Header.RTR = CAN_RTR_DATA;
	Header.IDE = CAN_ID_EXT;
	Header.DLC = 8;
	Header.TransmitGlobalTime = DISABLE;
	uint32_t mailbox;
	if (HAL_CAN_AddTxMessage(&hcan, &Header, Message->Data, &mailbox) != HAL_OK) {
		printf("Error when sending CAN message!\r\n");
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
	BaseType_t highertaskworken;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        Error_Handler();
    } else {
		CANMsg msg;
		msg.SourceID = rxHeader.ExtId & 0x000000ff;
		msg.TargetID = (rxHeader.ExtId >> 8) & 0x000000ff;
		msg.Param = (rxHeader.ExtId >> 16) & 0x000000ff;
		msg.OpType = (rxHeader.ExtId >> 24) & 0x0000001f;
		memcpy(msg.Data, rxData, 8);
		xQueueSendFromISR(canRecvMsgQueue, &msg, &highertaskworken);
	}
}

void vCanSendTaskFunction( void *pvParameters ) {
	(void) pvParameters;
	CANMsg message;
	memset(message.Data, 0, sizeof(message.Data));
	TickType_t currentTime;
	for (;;) {
		currentTime = xTaskGetTickCount();
		message.TargetID = GetHostCANID();
		message.SourceID = GetboardCANID();
		message.Param = 0;
		message.OpType = 8;
		message.Data[0] = GetQueueLength();
		message.Data[1] = GetCurrentCommand();
		message.Data[4] = currentTime >> 24;
		message.Data[5] = currentTime >> 16;
		message.Data[6] = currentTime >> 8;
		message.Data[7] = currentTime;
		vTaskDelay(1000);
	}
}

void vCanManageTaskFunction( void *pvParameters ) {
	(void) pvParameters;

	MessageHandlerFactory factory;
	factory.RegisterHandler(0x00, std::make_shared<ReadStateMessageHandler>());
	factory.RegisterHandler(0x01, std::make_shared<WriteStateMessageHandler>());
	factory.RegisterHandler(0x02, std::make_shared<AddToQueueMessageHandler>());
	factory.RegisterHandler(0x03, std::make_shared<ClearQueueMessageHandler>());
	factory.RegisterHandler(0x04, std::make_shared<PopQueueMessageHandler>());
	factory.RegisterHandler(0x06, std::make_shared<DirectOperationMessageHandler>());

	canRecvMsgQueue = xQueueCreate(10, sizeof(CANMsg));
	// CAN Filter setup
	printf("Starting Can Manager....\r\n");
	uint8_t canID = GetboardCANID();
	Filter.FilterBank = 0;
	Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	Filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	Filter.FilterIdHigh = 0x0000;
	Filter.FilterIdLow = canID << 3;
	Filter.FilterMaskIdHigh = 0x0000;
	Filter.FilterMaskIdLow = 0x00FF << 3;
    Filter.FilterActivation = ENABLE;
	Filter.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan, &Filter);
	HAL_CAN_Start(&hcan);
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		Error_Handler();
	xTaskCreate(vCanSendTaskFunction, "CanSendTask", 512, NULL, 2, &vCanSendTaskHandle);
	CANMsg message;
	while (1) {
		if (xQueueReceive(canRecvMsgQueue, &message, portMAX_DELAY) == pdTRUE) {
			auto handler = factory.GetHandler(message.Param);
			if (handler) {
				handler->Handler(message);
			}
		}
	}
}

void MessageHandlerFactory::RegisterHandler(uint8_t messageID, std::shared_ptr<BaseMessageHandler> handler) {
	handlers[messageID] = handler;
}

std::shared_ptr<BaseMessageHandler> MessageHandlerFactory::GetHandler(uint8_t messageID) {
	if (handlers.find(messageID) != handlers.end()) {
		return handlers[messageID];
	}
	return nullptr;
}


CANMsg responseMsg;

// ReadStateMessageHandler implementation
ReadStateMessageHandler::ReadStateMessageHandler() {
    MessageID = 0x00;
    MessageHandlerID = "ReadStateMessageHandler";
}

void ReadStateMessageHandler::Handler(const CANMsg& msg) {
	bool paramFound = true;
	switch (msg.Data[0]) {
		case 0x00: 
			responseMsg.Data[0] = GetboardCANID();
			break;
		case 0x01:
			responseMsg.Data[0] = GetRunMode();
			break;
		case 0x02:
			responseMsg.Data[0] = GetWheelSpeed(0) >> 8;
			responseMsg.Data[1] = GetWheelSpeed(0) & 0xFF;
			responseMsg.Data[2] = GetWheelSpeed(1) >> 8;
			responseMsg.Data[3] = GetWheelSpeed(1) & 0xFF;
			responseMsg.Data[4] = GetWheelSpeed(2) >> 8;
			responseMsg.Data[5] = GetWheelSpeed(2) & 0xFF;
			responseMsg.Data[6] = GetWheelSpeed(3) >> 8;
			responseMsg.Data[7] = GetWheelSpeed(3) & 0xFF;
			break;
		case 0x03:
			responseMsg.Data[0] = GetQueueLength();
			break;
		case 0x04:
			responseMsg.Data[0] = GetTargetDistance() >> 24;
			responseMsg.Data[1] = (GetTargetDistance() >> 16) & 0xFF;
			responseMsg.Data[2] = (GetTargetDistance() >> 8) & 0xFF;
			responseMsg.Data[3] = GetTargetDistance() & 0xFF;
			break;
		case 0x05:
			responseMsg.Data[0] = GetPWMPSC() >> 8;
			responseMsg.Data[1] = GetPWMPSC() & 0xFF;
			responseMsg.Data[2] = GetPWMARR() >> 8;
			responseMsg.Data[3] = GetPWMARR() & 0xFF;
			break;
		case 0x06:
			responseMsg.Data[0] = GetHostCANID();
			break;
		case 0x07:
			float voltage = GetCurrentInputVoltage();
			memcpy(responseMsg.Data, &voltage, sizeof(float));
			break;
		case 0x08:
			responseMsg.Data[0] = GetCurrentCommand();
			break;
		case 0x09:
		case 0x0A:
		case 0x0B:
		case 0x0C:
			float kp = GetKp(msg.Data[0] - 0x09);
			memcpy(responseMsg.Data, &kp, sizeof(float));
			break;
		case 0x0D:
		case 0x0E:
		case 0x0F:
		case 0x10:
			float ki = GetKi(msg.Data[0] - 0x0D);
			memcpy(responseMsg.Data, &ki, sizeof(float));
			break;
		case 0x11:
		case 0x12:
		case 0x13:	
		case 0x14:
			float kd = GetKd(msg.Data[0] - 0x11);
			memcpy(responseMsg.Data, &kd, sizeof(float));
			break;
		default:
			responseMsg.Param = 0xFF;
			paramFound = false;
			break;
	}
	if (!paramFound)
		memset(responseMsg.Data, 0xff, sizeof(responseMsg.Data));
	else {
		responseMsg.SourceID = GetboardCANID();
		responseMsg.TargetID = msg.SourceID;
		responseMsg.Param = msg.Param;
		responseMsg.OpType = 0x00;
	}
	TransmitCanMessage(&responseMsg);
}

// WriteStateMessageHandler implementation
WriteStateMessageHandler::WriteStateMessageHandler() {
    MessageID = 0x01;
    MessageHandlerID = "WriteStateMessageHandler";
}

void WriteStateMessageHandler::Handler(const CANMsg& msg) {
	switch (msg.Param) {
		case 0x00:
			SetBoardCANID(msg.Data[0]);
			break;
		case 0x01:
			SetRunMode(msg.Data[0]);
			break;
		case 0x04:
			SetTargetDistance((msg.Data[0] << 24) + (msg.Data[1] << 16) + (msg.Data[2] << 8) + msg.Data[3]);
			break;
		case 0x05:
			SetPWMPSC((msg.Data[0] << 8) + msg.Data[1]);
			SetPWMARR((msg.Data[2] << 8) + msg.Data[3]);
			break;
		case 0x06:
			SetHostCANID(msg.Data[0]);
			break;
		case 0x09:
		case 0x0A:
		case 0x0B:
		case 0x0C:
			SetKp(msg.Param - 0x09, *(float*)msg.Data);
			break;
		case 0x0D:
		case 0x0E:
		case 0x0F:
		case 0x10:
			SetKi(msg.Param - 0x0D, *(float*)msg.Data);
			break;
		case 0x11:
		case 0x12:
		case 0x13:
		case 0x14:
			SetKd(msg.Param - 0x11, *(float*)msg.Data);
			break;
		default:
			break;
	}
}

// AddToQueueMessageHandler implementation
AddToQueueMessageHandler::AddToQueueMessageHandler() {
    MessageID = 0x02;
    MessageHandlerID = "AddToQueueMessageHandler";
}

void AddToQueueMessageHandler::Handler(const CANMsg& msg) {
	
}

// ClearQueueMessageHandler implementation
ClearQueueMessageHandler::ClearQueueMessageHandler() {
    MessageID = 0x03;
    MessageHandlerID = "ClearQueueMessageHandler";
}

void ClearQueueMessageHandler::Handler(const CANMsg& msg) {
}

// PopQueueMessageHandler implementation
PopQueueMessageHandler::PopQueueMessageHandler() {
    MessageID = 0x04;
    MessageHandlerID = "PopQueueMessageHandler";
}

void PopQueueMessageHandler::Handler(const CANMsg& msg) {
}

// DirectOperationMessageHandler implementation
DirectOperationMessageHandler::DirectOperationMessageHandler() {
    MessageID = 0x05;
    MessageHandlerID = "DirectOperationMessageHandler";
}

void DirectOperationMessageHandler::Handler(const CANMsg& msg) {
}