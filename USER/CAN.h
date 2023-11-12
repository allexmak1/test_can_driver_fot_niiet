#ifndef __CAN_H__
#define __CAN_H__ 

#include "K1921VK01T.h"
#include "main.h"

//can 1 (не используется, сделано на регистрах)
#define CAN1_PORT_RX   NT_GPIOE      //(can1 rx)
#define CAN1_PIN_RX    GPIO_Pin_2
#define CAN1_PORT_TX   NT_GPIOG      //(can1 tx)
#define CAN1_PIN_TX    GPIO_Pin_7

//can 2 (не используется, сделано на регистрах)
#define CAN2_PORT_RX   NT_GPIOF      //(can1 rx)
#define CAN2_PIN_RX    GPIO_Pin_15
#define CAN2_PORT_TX   NT_GPIOF      //(can1 tx)
#define CAN2_PIN_TX    GPIO_Pin_14   

typedef enum{
  CAN1 = 0,
  CAN2, 
}CAN_e;

//скорости передачи данных
typedef enum{
  SPEED_250kbs = 0,
  SPEED_500kbs
}CAN_SPEED_e;
  
typedef enum{
  STD = 0,    // стандартный
  EXT         // расширенный
}IDE_e;

typedef struct{
  uint32_t    id;
  uint8_t     data[8];  
  IDE_e       ide;  
  uint8_t     len;
}CanMsgTypeDef;

typedef struct{
  uint32_t    cntOwerWriteBufTX;       // счетчик переполнения аппаратного буфера отправки сообщений
  uint32_t    cntOwerReadBufRx;           // счетчик переполнения аппаратного буфера приема сообщений
}CanErrorTypeDef;

void initCAN(CAN_e CANx, CAN_SPEED_e speed);
void CanCreateObjectList(uint32_t can, uint8_t rxStartListObject, uint8_t rxMaxListObject, uint8_t txStartListObject, uint8_t txMaxListObject);
void CreateTXFifo (uint32_t base_num, uint32_t end_num);
void CreateRXFifo (uint32_t base_num, uint32_t end_num);
void sendCanMessage(CAN_e CANx, CanMsgTypeDef* TxMessage);
int xMakeId(int Prioritet, int Discriptr, int Istochnic, int Priemnic);
void sendCanTest();
void CAN1_Receive();
void CAN2_Receive();
void CAN1_Parse(CanMsgTypeDef *CanMsgRXx, int cnt);
void CAN2_Parse(CanMsgTypeDef *CanMsgRX, int cnt);
#endif