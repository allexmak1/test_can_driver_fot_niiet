/*******************************************************************************
* Lib: ���������� �������� CAN
*           ��������� ��������:
*           - ���������� ����� can1 ������:    10 �����(0-9)
*           - ���������� ����� can1 ��������:  10 �����(10-19)
*           - ���������� ����� can2 ������:    10 �����(20-29)
*           - ���������� ����� can2 ��������:  10 �����(30-39)
*           ����� ����� 256
*           ������ ������ ��������:
*             �������� ��������� �������������� �������� sendCanMessage().
*             ������ ��������� ������������ �� �������� CUR �������� �������� ���������,
*             ��� �������� ��������� ���� CUR �� ����������, �� ���������� � ����� �������� �
*             ��������� ���������� ����� �� ����� �����������.
*           ������ ������ ������:
*             ����� ��������� �������������� ����������� CAN12_IRQHandler() ��� �������� � ����� CAN1_Receive()
*             ��� �������� �������� ��������� ������� ������ � ����� CAN1_Parse() � ��� ����� ������������ 
*             ��������� ��� � ����� ���������� (������ � �������) ���������� 2 ������� ��������� �� ������ can.
*             ������� ����������� ����� �������� ��������� �� ����������, � �������������� ��� ���� ����� � ���� 
*             ��������� ��������� ����� ���������� ���������� � �� ��� ������ ��������� �� ��������� �������, � ��������.
*             ��� �������� ��� ���� ����� ������ ����� ���������.
* Autor: ������� �.�.(46013)
* Date: 11.10.22
* Version: 2.0.0
*******************************************************************************/
#include "CAN.h"
#include "main.h"
#include "CPUconfig.h"

extern int flag;

//can1
#define CAN1_RX_START_LIST_OBJECT   0
#define CAN1_RX_MAX_LIST_OBJECT     9
#define CAN1_TX_START_LIST_OBJECT   10
#define CAN1_TX_MAX_LIST_OBJECT     19
//can2
#define CAN2_RX_START_LIST_OBJECT   20
#define CAN2_RX_MAX_LIST_OBJECT     29
#define CAN2_TX_START_LIST_OBJECT   30
#define CAN2_TX_MAX_LIST_OBJECT     39

#define CAN1_BUF_RX_MSG             10  // = CAN1_RX_MAX_LIST_OBJECT - CAN1_RX_START_LIST_OBJECT + 1
#define CAN2_BUF_RX_MSG             10  // = CAN2_RX_MAX_LIST_OBJECT - CAN2_RX_START_LIST_OBJECT + 1

CanErrorTypeDef Can1ErrorType;
CanErrorTypeDef Can2ErrorType;
CanMsgTypeDef buf1RxMsg[CAN1_BUF_RX_MSG];
CanMsgTypeDef buf2RxMsg[CAN1_BUF_RX_MSG];
CanMsgTypeDef buf3RxMsg[CAN2_BUF_RX_MSG];
CanMsgTypeDef buf4RxMsg[CAN2_BUF_RX_MSG];
uint8_t cnt1RxBuffCan1 = 0;
uint8_t cnt2RxBuffCan1 = 0;
uint8_t cnt1RxBuffCan2 = 0;
uint8_t cnt2RxBuffCan2 = 0;
uint8_t isParseCan1;
uint8_t isParseCan2;
uint8_t oldCurCan1 = 111, mbNumCan1, stepSendCan1, cntCurrentCurCan1;
uint8_t oldCurCan2 = 111, mbNumCan2, stepSendCan2, cntCurrentCurCan2;

/*******************************************************************************
*  @brief   initCAN - ������������� CAN
*  @param   CAN_e - ����� can
*  @param   CAN_SPEED_e - ��������
*  @retval  none
*******************************************************************************/
void initCAN(CAN_e CANx, CAN_SPEED_e speed){
  // ������������� ���
  // G7 - CAN0_TX, E2 - CAN0_RX
  // F14 - CAN1_TX, F15 - CAN1_RX  
  uint32_t tmp;
  if(CANx == CAN1){
    tmp = (1<<7);
    NT_GPIOG->ALTFUNCSET |= tmp;
    NT_COMMON_REG->GPIOPCTLG &= ~(3 << (2 * 7));
    NT_COMMON_REG->GPIODENG |= tmp;
    tmp = (1<<2);
    NT_GPIOE->ALTFUNCSET |= tmp;
    NT_COMMON_REG->GPIOPCTLE &= ~(3 << (2 * 2));
    NT_COMMON_REG->GPIODENE |= tmp; 
  }else if(CANx == CAN2){
    tmp = (1<<15) | (1<<14);
    NT_GPIOF->ALTFUNCSET |= tmp;
    NT_COMMON_REG->GPIOPCTLF &= ~((3 << (2 * 15)) |
                                  (3 << (2 * 14)));
    NT_COMMON_REG->GPIODENF |= tmp;    
  }
  
  // ������������ can
  NT_CAN->CLC_bit.DISR = 0;                                                     // ��������� ����������� CAN
  while (NT_CAN->CLC_bit.DISS == 1) {};                                         // ���� ���������  ����������� CAN
  while (NT_CAN->PANCTR_bit.PANCMD == 1) {};                                    // ������� ������ ������ ������ ���� 0
  NT_CAN->FDR = (0x1 << CAN_FDR_DM_Pos) | (0x3FF << CAN_FDR_STEP_Pos);          // ���������� ����� ������ � ������������� 1 (������� ���� CAN 100���)
  
  if(CANx == CAN1){
    NT_CAN->CAN_Node[0].NCR_bit.CCE = 1;                                        // ��� ���������� ��������� ������������ ����. (��� ��������� ��������� ���������)
    NT_CAN->CAN_Node[0].NCR_bit.INIT = 1;                                       // ���������� ������� ���� � �������.
    NT_CAN->CAN_Node[0].NPCR_bit.LBM = 0;                                       // ��������� ����� �������� �����
    NT_CAN->CAN_Node[0].NIPR = 0;                                               // ������� ��������� ���������� ����
    NT_CAN->CAN_Node[0].NIPR = (0xC << CAN_NIPR_TRINP_Pos);                     // "12" ��������� ����� ����������, ��� ���������� �� ��������� ��������/������ ���������    
    NT_CAN->CAN_Node[0].NFCR = 0;                                               // ������� �������� ��������� ����
    
    if(speed == SPEED_250kbs){
      NT_CAN->CAN_Node[0].NBTR_bit.DIV8 = 0x0;
      NT_CAN->CAN_Node[0].NBTR_bit.TSEG1 = 0x3;      
      NT_CAN->CAN_Node[0].NBTR_bit.TSEG2 = 0x2;
      NT_CAN->CAN_Node[0].NBTR_bit.SJW = 0x0;
      NT_CAN->CAN_Node[0].NBTR_bit.BRP = 0x31;
    }else if(speed == SPEED_500kbs){
      NT_CAN->CAN_Node[0].NBTR_bit.DIV8 = 0x0;
      NT_CAN->CAN_Node[0].NBTR_bit.TSEG1 = 0x3;
      NT_CAN->CAN_Node[0].NBTR_bit.TSEG2 = 0x2;
      NT_CAN->CAN_Node[0].NBTR_bit.SJW = 0x0;
      NT_CAN->CAN_Node[0].NBTR_bit.BRP = 0x18;
    }
    NT_CAN->CAN_Node[0].NCR = 0;                                                // ��������� ��������� ���������
    NT_CAN->CAN_Node[0].NCR = CAN_NCR_TRIE_Msk;                                 // ��������� ���������� �� ������ ������ � ������ ��������� (CAN12_IRQn)
    
    // ����������� can1
    CanCreateObjectList(CAN1, CAN1_RX_START_LIST_OBJECT, CAN1_RX_MAX_LIST_OBJECT, CAN1_TX_START_LIST_OBJECT, CAN1_TX_MAX_LIST_OBJECT);       // ������� ������ ��������
    CreateRXFifo(CAN1_RX_START_LIST_OBJECT, CAN1_RX_MAX_LIST_OBJECT);           // ������� FIFO ��� ���������
    CreateTXFifo(CAN1_TX_START_LIST_OBJECT, CAN1_TX_MAX_LIST_OBJECT);           // ������� FIFO ��� ��������
    NVIC_EnableIRQ(CAN12_IRQn);                                                 // ���������� ����������
    
    
  }else if(CANx == CAN2){
    NT_CAN->CAN_Node[1].NCR_bit.CCE = 1;                                        // ��� ���������� ��������� ������������ ����. (��� ��������� ��������� ���������)
    NT_CAN->CAN_Node[1].NCR_bit.INIT = 1;                                       // ���������� ������� ���� � �������.
    NT_CAN->CAN_Node[1].NPCR_bit.LBM = 0;                                       // ��������� ����� �������� �����
    NT_CAN->CAN_Node[1].NIPR = 0;                                               // ������� ��������� ���������� ����
    NT_CAN->CAN_Node[1].NIPR = (0xF << CAN_NIPR_TRINP_Pos);                     // "12" ��������� ����� ����������, ��� ���������� �� ��������� ��������/������ ���������    
    NT_CAN->CAN_Node[1].NFCR = 0;                                               // ������� �������� ��������� ����
    
    if(speed == SPEED_250kbs){
      NT_CAN->CAN_Node[1].NBTR_bit.DIV8 = 0x0;
      NT_CAN->CAN_Node[1].NBTR_bit.TSEG1 = 0x3;      
      NT_CAN->CAN_Node[1].NBTR_bit.TSEG2 = 0x2;
      NT_CAN->CAN_Node[1].NBTR_bit.SJW = 0x0;
      NT_CAN->CAN_Node[1].NBTR_bit.BRP = 0x31;
    }else if(speed == SPEED_500kbs){
      NT_CAN->CAN_Node[1].NBTR_bit.DIV8 = 0x0;
      NT_CAN->CAN_Node[1].NBTR_bit.TSEG1 = 0x3;
      NT_CAN->CAN_Node[1].NBTR_bit.TSEG2 = 0x2;
      NT_CAN->CAN_Node[1].NBTR_bit.SJW = 0x0;
      NT_CAN->CAN_Node[1].NBTR_bit.BRP = 0x18;
    }
    NT_CAN->CAN_Node[1].NCR = 0;                                                // ��������� ��������� ���������
    NT_CAN->CAN_Node[1].NCR = CAN_NCR_TRIE_Msk;                                 // ��������� ���������� �� ������ ������ � ������ ��������� (CAN15_IRQn)
    
    // ����������� can2
    CanCreateObjectList(CAN2, CAN2_RX_START_LIST_OBJECT, CAN2_RX_MAX_LIST_OBJECT, CAN2_TX_START_LIST_OBJECT, CAN2_TX_MAX_LIST_OBJECT);      // ������� ������ ��������
    CreateRXFifo(CAN2_RX_START_LIST_OBJECT, CAN2_RX_MAX_LIST_OBJECT);           // ������� FIFO ��� ���������
    CreateTXFifo(CAN2_TX_START_LIST_OBJECT, CAN2_TX_MAX_LIST_OBJECT);           // ������� FIFO ��� ��������
    NVIC_EnableIRQ(CAN15_IRQn);                                                 // ���������� ����������
  }
}

/*******************************************************************************
*  @brief   CanCreateObjectList - �������� ������ �������� ��� ������
*******************************************************************************/
void CanCreateObjectList(uint32_t can, uint8_t rxStartListObject, uint8_t rxMaxListObject, uint8_t txStartListObject, uint8_t txMaxListObject){
  // ��������� ������ ������ ��� ������ �� can
  // can0 ������ 1, can1 ������ 2 
  for (int i = rxStartListObject; i <= txMaxListObject; i++){
    while (NT_CAN->PANCTR & (3<<8));                                    // ���� ���������� ������ ������ ������
    NT_CAN->PANCTR = (2<< CAN_PANCTR_PANCMD_Pos) |                      // ������� ���������� (������� ������ � ������)
      ((i)<< CAN_PANCTR_PANAR1_Pos) |                                   // �������� 1 (����� �������)
        ((can+1)<< CAN_PANCTR_PANAR2_Pos);                              // �������� 2 (����� ������)
  }
  
  // ��������� �������� �������
  for (int i = rxStartListObject; i <= rxMaxListObject; i++){
    NT_CAN->CAN_Msg[i].MOCTR = 0xFFF;                                  // ����� ���� ������ (����� ���� RESET)
    // ������ ��� ������ ��������� (DIR=0)
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETRXEN_Pos);            // ��������� ����� ��������� (RXEN = 1)
    NT_CAN->CAN_Msg[i].MOFCR  = (1<<CAN_MOFCR_RXIE_Pos);               // ���������t �� ������, ����������� ������ ������.
    NT_CAN->CAN_Msg[i].MOAR   = (2<<CAN_MOAR_PRI_Pos) |                // ��� ����������
      (1<<CAN_MOAR_IDE_Pos) |                                          // ������ ������ � ������������ �����������
        0x00;                                                          //
    NT_CAN->CAN_Msg[i].MOAMR  = (0<<CAN_MOAMR_MIDE_Pos) |              // ��� IDE ��� ���������� �� ������ ��������� (����� ���������)
      0;                                                               // ����� ��� ���������� ��������� (0 - ����� ����)
    NT_CAN->CAN_Msg[i].MOFGPR = 0;                                     // FIFO �� ������������
    NT_CAN->CAN_Msg[i].MOIPR  = (i<<CAN_MOIPR_MPN_Pos);                // ����������, ����� ������� ���� ���������.
    // ��������� ��� ��� ������� ���������
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETMSGVAL_Pos);          // ��������� ������ ������� (MSGVAL = 1)
  }
  
  // ��������� ������� �� ��������
  for (int i = txStartListObject; i <= txMaxListObject; i++){
    NT_CAN->CAN_Msg[i].MOCTR = 0xFFF;                                  // ����� ���� ������ (����� ���� RESET)
    // ������ ��� �������� ��������� (DIR=1)
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETTXEN0_Pos)|           // ��������� �������� ��������� (TXEN0 = 1)
      (1<<CAN_MOCTR_SETTXEN1_Pos)|                                     // ��������� �������� ��������� (TXEN1 = 1)
        (1<<CAN_MOCTR_SETDIR_Pos);                                     // ��������� �������� ��������� (DIR = 1)
    NT_CAN->CAN_Msg[i].MOAR   = (1<<CAN_MOAR_PRI_Pos) |                // ��� ����������
      (1<<CAN_MOAR_IDE_Pos) |                                          // ������ ������ � ������������ �����������
        0x0;
    NT_CAN->CAN_Msg[i].MOAMR  = (0<<CAN_MOAMR_MIDE_Pos) |              // ��� IDE ��� ���������� ������ ���������
      0x0;                                                             // ����� ��� ���������� ���������
    NT_CAN->CAN_Msg[i].MOFGPR = 0;                                     // FIFO �� ������������
    NT_CAN->CAN_Msg[i].MOIPR  = (i<<CAN_MOIPR_MPN_Pos);                // ����������, ����� ������� ���� ���������.
    // ��������� ��� ��� ������� ���������
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETMSGVAL_Pos);          // ��������� ������ ������� (MSGVAL = 1)
  }
}

/*******************************************************************************
*  @brief   CreateTXFifo - ��������� FIFO ��������� ��� ��������
*******************************************************************************/
void CreateTXFifo (uint32_t base_num, uint32_t end_num){
  // ������� ������
  NT_CAN->CAN_Msg[base_num].MOCTR  = 0xFFF;                                     // ����� ���� ������ (����� ���� RESET)
  NT_CAN->CAN_Msg[base_num].MOCTR  = (1<<CAN_MOCTR_SETTXEN0_Pos)|               // ��������� �������� ��������� (TXEN0 = 1)
    (1<<CAN_MOCTR_SETTXEN1_Pos)|                                                // ��������� �������� ��������� (TXEN1 = 1)
      (1<<CAN_MOCTR_SETDIR_Pos);                                                // ��������� �������� ��������� (DIR = 1)
  NT_CAN->CAN_Msg[base_num].MOFCR  = (2<<CAN_MOFCR_MMC_Pos)|                    // ������� ������ �������� (��� ���� ����������)
    (0<<CAN_MOFCR_OVIE_Pos)|                                                    // ��� ���������� ���������� �� ���������� ������
      (1<<CAN_MOFCR_STT_Pos);                                                   // � ������ ��������� ��������, ��������� �������� ��������� �� �����
  NT_CAN->CAN_Msg[base_num].MOFGPR = ((base_num/*+1*/)<<CAN_MOFGPR_BOT_Pos) |       // ����� ������� �������� (�������  ������������)
    (end_num<<CAN_MOFGPR_TOP_Pos) |                                             // ����� ���������� �������� 
      (base_num<<CAN_MOFGPR_CUR_Pos) |                                          // ����� ������� �������� ��� ����������
        (0<<CAN_MOFGPR_SEL_Pos);                                                // ����� �������� ��� �������� �� ��������� FIFO (�� �����������)----�����������
  NT_CAN->CAN_Msg[base_num].MOIPR  = (base_num<<CAN_MOIPR_MPN_Pos);             // ���������� �� ������������
  NT_CAN->CAN_Msg[base_num].MOCTR  = (1<<CAN_MOCTR_SETMSGVAL_Pos);              // ��������� ������ ������� (MSGVAL = 1)
  
  // ��������������� �������
  for (int i=base_num+1;i<=end_num;i++){
    NT_CAN->CAN_Msg[i].MOCTR  = 0xFFF;                                          // ����� ���� ������ (����� ���� RESET)
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETTXEN0_Pos)|                    // ��������� �������� ��������� (TXEN0 = 1)
      (1<<CAN_MOCTR_SETDIR_Pos);                                                // ��������� �������� ��������� (DIR = 1)
    NT_CAN->CAN_Msg[i].MOFCR  = (3<<CAN_MOFCR_MMC_Pos)|                         // ��������������� ������ FIFO �������� (�����������)
      (0<<CAN_MOFCR_OVIE_Pos)|                                                  // ��� ���������� ���������� �� ���������� ������
        (1<<CAN_MOFCR_STT_Pos);                                                 // � ������ ��������� ��������, ��������� �������� ��������� �� �����
    NT_CAN->CAN_Msg[i].MOFGPR = (base_num<<CAN_MOFGPR_CUR_Pos);                 // ������� ������
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETMSGVAL_Pos);                   // ��������� ������ ������� (MSGVAL = 1)
    NT_CAN->CAN_Msg[i].MOFCR_bit.DLC = 8;
  }
  
}

/*******************************************************************************
*  @brief   CreateRXFifo - ��������� FIFO ��������� ��� ������
*******************************************************************************/
void CreateRXFifo (uint32_t base_num, uint32_t end_num){
  // ������� ������
  NT_CAN->CAN_Msg[base_num].MOCTR  = 0xFFF;                                     // ����� ���� ������ (����� ���� RESET)
  NT_CAN->CAN_Msg[base_num].MOCTR  = (1<<CAN_MOCTR_SETRXEN_Pos);                // ��������� ����� ���������
  NT_CAN->CAN_Msg[base_num].MOFCR  = (1<<CAN_MOFCR_RXIE_Pos)|                   // ���������� �� ������
    (1<<CAN_MOFCR_OVIE_Pos)|                                                    // ��� ���������� ���������� �� ���������� ������
      (1<<CAN_MOFCR_MMC_Pos);                                                     // ������� ������ ������
  NT_CAN->CAN_Msg[base_num].MOFGPR = ((base_num/*+1*/)<<CAN_MOFGPR_BOT_Pos) |       // ����� ������� �������� (������� ������������)
    (end_num<<CAN_MOFGPR_TOP_Pos) |                                             // ����� ���������� �������� 
      ((base_num+1)<<CAN_MOFGPR_CUR_Pos) |                                      // ����� ������� �������� ��� ������
        (0<<CAN_MOFGPR_SEL_Pos);                                                // ����� �������� ��� �������� �� ��������� FIFO (�� �����������)----�����������
  NT_CAN->CAN_Msg[base_num].MOCTR  = (1<<CAN_MOCTR_SETMSGVAL_Pos);              // ��������� ������ ������� (MSGVAL = 1)
  
  // ��������������� �������
  for (int i=base_num+1;i<=end_num;i++){
    NT_CAN->CAN_Msg[i].MOCTR  = 0xFFF;                                          // ����� ���� ������ (����� ���� RESET)
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_RESTXEN0_Pos);
    NT_CAN->CAN_Msg[i].MOFCR  = (0<<CAN_MOFCR_MMC_Pos);                         // ������������
    NT_CAN->CAN_Msg[i].MOFGPR = (base_num<<CAN_MOFGPR_CUR_Pos);                 // ������� ������
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETMSGVAL_Pos);                   // ��������� ������ ������� (MSGVAL = 1)
    
    NT_CAN->CAN_Msg[i].MOFCR_bit.DLC = 8;
    
  }
}
/*******************************************************************************
*  @brief   CAN12_IRQHandler - ���������� �������� ��������� CAN1
*******************************************************************************/
void CAN12_IRQHandler(void){
  CanMsgTypeDef *bufRxMsg;
  uint8_t cntRxBuffCan1;
  if(!isParseCan1){
    bufRxMsg =  &buf1RxMsg[0];
    cntRxBuffCan1 = cnt1RxBuffCan1;
    cnt1RxBuffCan1++;
    // �������� ����� �������
    if(cnt1RxBuffCan1 > CAN1_BUF_RX_MSG){
      Can1ErrorType.cntOwerReadBufRx++;
      cnt1RxBuffCan1--;
      return;
    }
  }else{
    bufRxMsg =  &buf2RxMsg[0];
    cntRxBuffCan1 = cnt2RxBuffCan1;
    cnt2RxBuffCan1++;
    if(cnt2RxBuffCan1 > CAN1_BUF_RX_MSG){
      Can1ErrorType.cntOwerReadBufRx++;
      cnt2RxBuffCan1--;
      return;
    }
  }
  
  // ���������� ������� ��������� ��������� �����
  int obj = NT_CAN->CAN_Msg[CAN1_RX_START_LIST_OBJECT].MOFGPR_bit.CUR - 1;
  if(obj == CAN1_RX_START_LIST_OBJECT-1)obj = CAN1_RX_MAX_LIST_OBJECT;
  // ���������� � �����
  if(NT_CAN->CAN_Msg[obj].MOAR_bit.IDE){
    bufRxMsg[cntRxBuffCan1].ide = EXT;
    bufRxMsg[cntRxBuffCan1].id = (uint32_t)NT_CAN->CAN_Msg[obj].MOAR_bit.ID;
  }else{ 
    bufRxMsg[cntRxBuffCan1].ide = STD;
    bufRxMsg[cntRxBuffCan1].id = (uint32_t)NT_CAN->CAN_Msg[obj].MOAR_bit.ID>>18;
  }
  bufRxMsg[cntRxBuffCan1].len = (uint8_t)NT_CAN->CAN_Msg[obj].MOFCR_bit.DLC;
  bufRxMsg[cntRxBuffCan1].data[3] = (uint8_t)(NT_CAN->CAN_Msg[obj].MODATAL >> 24);
  bufRxMsg[cntRxBuffCan1].data[2] = (uint8_t)(NT_CAN->CAN_Msg[obj].MODATAL >> 16);
  bufRxMsg[cntRxBuffCan1].data[1] = (uint8_t)(NT_CAN->CAN_Msg[obj].MODATAL >> 8);
  bufRxMsg[cntRxBuffCan1].data[0] = (uint8_t)NT_CAN->CAN_Msg[obj].MODATAL;
  bufRxMsg[cntRxBuffCan1].data[7] = (uint8_t)(NT_CAN->CAN_Msg[obj].MODATAH >> 24);
  bufRxMsg[cntRxBuffCan1].data[6] = (uint8_t)(NT_CAN->CAN_Msg[obj].MODATAH >> 16);
  bufRxMsg[cntRxBuffCan1].data[5] = (uint8_t)(NT_CAN->CAN_Msg[obj].MODATAH >> 8);
  bufRxMsg[cntRxBuffCan1].data[4] = (uint8_t)NT_CAN->CAN_Msg[obj].MODATAH;
  
  // �������� �������
  NT_CAN->CAN_Msg[obj].MOAR_bit.ID = 0;
  NT_CAN->CAN_Msg[obj].MODATAL = 0;
  NT_CAN->CAN_Msg[obj].MODATAH = 0;
  
  // ����� ��������� �������� ��������� ������� ����� � ����������.
  /* ������:
  if(bufRxMsg[cntRxBuffCan1].id == 0xAAA){
  int data = bufRxMsg[cntRxBuffCan1].data[0];
  flag = 1;
}
  */
}

/*******************************************************************************
*  @brief   CAN15_IRQHandler - ���������� �������� ��������� CAN2
*******************************************************************************/
void CAN15_IRQHandler(void){
  CanMsgTypeDef *bufRxMsg;
  uint8_t cntRxBuffCan2;
  if(!isParseCan2){
    bufRxMsg =  &buf3RxMsg[0];
    cntRxBuffCan2 = cnt1RxBuffCan2;
    cnt1RxBuffCan2++;
    if(cnt1RxBuffCan2 > CAN2_BUF_RX_MSG){
      Can2ErrorType.cntOwerReadBufRx++;
      cnt1RxBuffCan2--;
      return;
    }
  }else{
    bufRxMsg =  &buf4RxMsg[0];
    cntRxBuffCan2 = cnt2RxBuffCan2;
    cnt2RxBuffCan2++;
    if(cnt2RxBuffCan2 > CAN2_BUF_RX_MSG){
      Can2ErrorType.cntOwerReadBufRx++;
      cnt2RxBuffCan2--;
      return;
    }
  }
  
  // ���������� ������� ��������� ��������� �����
  int obj = NT_CAN->CAN_Msg[CAN2_RX_START_LIST_OBJECT].MOFGPR_bit.CUR - 1;
  if(obj == CAN2_RX_START_LIST_OBJECT-1)obj = CAN2_RX_MAX_LIST_OBJECT;
  // ���������� � �����
  if(NT_CAN->CAN_Msg[obj].MOAR_bit.IDE){
    bufRxMsg[cntRxBuffCan2].ide = EXT;
    bufRxMsg[cntRxBuffCan2].id = (uint32_t)NT_CAN->CAN_Msg[obj].MOAR_bit.ID;
  }else{ 
    bufRxMsg[cntRxBuffCan2].ide = STD;
    bufRxMsg[cntRxBuffCan2].id = (uint32_t)NT_CAN->CAN_Msg[obj].MOAR_bit.ID>>18;
  }
  bufRxMsg[cntRxBuffCan2].len = (uint8_t)NT_CAN->CAN_Msg[obj].MOFCR_bit.DLC;
  bufRxMsg[cntRxBuffCan2].data[3] = (uint8_t)(NT_CAN->CAN_Msg[obj].MODATAL >> 24);
  bufRxMsg[cntRxBuffCan2].data[2] = (uint8_t)(NT_CAN->CAN_Msg[obj].MODATAL >> 16);
  bufRxMsg[cntRxBuffCan2].data[1] = (uint8_t)(NT_CAN->CAN_Msg[obj].MODATAL >> 8);
  bufRxMsg[cntRxBuffCan2].data[0] = (uint8_t)NT_CAN->CAN_Msg[obj].MODATAL;
  bufRxMsg[cntRxBuffCan2].data[7] = (uint8_t)(NT_CAN->CAN_Msg[obj].MODATAH >> 24);
  bufRxMsg[cntRxBuffCan2].data[6] = (uint8_t)(NT_CAN->CAN_Msg[obj].MODATAH >> 16);
  bufRxMsg[cntRxBuffCan2].data[5] = (uint8_t)(NT_CAN->CAN_Msg[obj].MODATAH >> 8);
  bufRxMsg[cntRxBuffCan2].data[4] = (uint8_t)NT_CAN->CAN_Msg[obj].MODATAH;
  // �������� �������
  NT_CAN->CAN_Msg[obj].MOAR_bit.ID = 0;
  NT_CAN->CAN_Msg[obj].MODATAL = 0;
  NT_CAN->CAN_Msg[obj].MODATAH = 0;
  
  // ����� ��������� �������� ��������� ������� ����� � ����������.
  /* ������:
  if(bufRxMsg[cntRxBuffCan2].id == 0xAAA){
  int data = bufRxMsg[cntRxBuffCan2].data[0];
  flag = 1;
}
  */
}

/*******************************************************************************
*  @brief   sendCanMessage - 
*  @param   CANx - CAN1
*                  CAN2
*  @param   TxMessage - ������� �������������� ��������� ������������� ���������
*  @retval  none
*******************************************************************************/
void sendCanMessage(CAN_e CANx, CanMsgTypeDef* TxMessage){
  uint16_t cur, mbNum;
  if(CANx == CAN1){
    cur = NT_CAN->CAN_Msg[CAN1_TX_START_LIST_OBJECT].MOFGPR_bit.CUR;
    //�������� ���, ��-�� ����, ��� ��� ������������� �������� �������� NT_CAN->CAN_Msg[mbNum].MOSTAT_bit.TXRQ == 1 
    //                                ���������� �� ���������� 1 ��� �������� ������� ���������
    if(stepSendCan1 == 0){
      if(cur != oldCurCan1){
        mbNumCan1 = cur;
        oldCurCan1 = cur;
        stepSendCan1 = 0;
        cntCurrentCurCan1 = 0;
      }else{
        if(++mbNumCan1 > CAN1_TX_MAX_LIST_OBJECT) mbNumCan1 = CAN1_TX_START_LIST_OBJECT;
        //��������� � �������
        if(++cntCurrentCurCan1 >= CAN1_BUF_RX_MSG-1) stepSendCan1 = 1;
      }
    }
    if(stepSendCan1 == 1){
      if(cur != oldCurCan1){
        mbNumCan1 = cur;
        oldCurCan1 = cur;
        stepSendCan1 = 0;
        cntCurrentCurCan1 = 0;
      }else{
        Can1ErrorType.cntOwerWriteBufTX++;
        return;
      }
    }
    mbNum = mbNumCan1;
    
  }else if(CANx == CAN2){
    cur = NT_CAN->CAN_Msg[CAN2_TX_START_LIST_OBJECT].MOFGPR_bit.CUR;
    //�������� ���, ��-�� ����, ��� ��� ������������� �������� �������� NT_CAN->CAN_Msg[mbNum].MOSTAT_bit.TXRQ == 1 
    //                                ���������� �� ���������� 1 ��� �������� ������� ���������
    if(stepSendCan2 == 0){
      if(cur != oldCurCan2){
        mbNumCan2 = cur;
        oldCurCan2 = cur;
        stepSendCan2 = 0;
      }else{
        if(++mbNumCan2 > CAN2_TX_MAX_LIST_OBJECT) mbNumCan2 = CAN2_TX_START_LIST_OBJECT;
        //��������� � �������
        if(++cntCurrentCurCan2 >= CAN2_BUF_RX_MSG-1) stepSendCan2 = 1;
      }
    }
    if(stepSendCan2 == 1){
      if(cur != oldCurCan2){
        mbNumCan2 = cur;
        oldCurCan2 = cur;
        stepSendCan2 = 0;
        cntCurrentCurCan2 = 0;
      }else{
        Can2ErrorType.cntOwerWriteBufTX++;
        return;
      }
    }
    mbNum = mbNumCan2;
  }
  
  
  // ����������
  NT_CAN->CAN_Msg[mbNum].MOAR = 0;
  // ��������� �������� �� �������� ID
  NT_CAN->CAN_Msg[mbNum].MOAR_bit.PRI = 2;                                      
  // �����������/����������� �������������
  if(TxMessage->ide <= 1)
    NT_CAN->CAN_Msg[mbNum].MOAR_bit.IDE = TxMessage->ide;
  // id
  if(NT_CAN->CAN_Msg[mbNum].MOAR_bit.IDE)
    NT_CAN->CAN_Msg[mbNum].MOAR_bit.ID = TxMessage->id;
  else
    NT_CAN->CAN_Msg[mbNum].MOAR_bit.ID = TxMessage->id<<18;
  // ������ 
  if(TxMessage->len <= 8) 
    NT_CAN->CAN_Msg[mbNum].MOFCR_bit.DLC = TxMessage->len;
  // ������
  NT_CAN->CAN_Msg[mbNum].MODATAL = (uint32_t)TxMessage->data[0];
  NT_CAN->CAN_Msg[mbNum].MODATAL |= (uint32_t)TxMessage->data[1]<<8;
  NT_CAN->CAN_Msg[mbNum].MODATAL |= (uint32_t)TxMessage->data[2]<<16;
  NT_CAN->CAN_Msg[mbNum].MODATAL |= (uint32_t)TxMessage->data[3]<<24;
  NT_CAN->CAN_Msg[mbNum].MODATAH = (uint32_t)TxMessage->data[4];
  NT_CAN->CAN_Msg[mbNum].MODATAH |= (uint32_t)TxMessage->data[5]<<8;
  NT_CAN->CAN_Msg[mbNum].MODATAH |= (uint32_t)TxMessage->data[6]<<16;
  NT_CAN->CAN_Msg[mbNum].MODATAH |= (uint32_t)TxMessage->data[7]<<24;
  
  NT_CAN->CAN_Msg[mbNum].MOCTR = CAN_MOCTR_SETMSGVAL_Msk | CAN_MOCTR_SETTXRQ_Msk ; //CAN_MOCTR_SETMSGVAL_Msk - ����������� �������, CAN_MOCTR_SETTXRQ_Msk - ���������� �������� ������
}

/*******************************************************************************
*  @brief   sendCanTest - �������� �������� ���������
*******************************************************************************/
void sendCanTest(){
  CanMsgTypeDef CanMsg;
  CanMsg.ide = EXT;
  CanMsg.len = 8;
  CanMsg.id = 0x10000000;
  CanMsg.data[0] = 0x01;
  CanMsg.data[1] = 0x02;
  CanMsg.data[2] = 0x03;
  CanMsg.data[3] = 0x04;
  CanMsg.data[4] = 0x05;
  CanMsg.data[5] = 0x06;
  CanMsg.data[6] = 0x07;
  CanMsg.data[7] = 0x08;
  sendCanMessage(CAN1, &CanMsg);
  CanMsg.id = 0x20000000;
  sendCanMessage(CAN2, &CanMsg);
}

/*******************************************************************************
*  @brief   xMakeId - (�����������) ������ ����������� (3-14-6-6)
*******************************************************************************/
int xMakeId(int Prioritet, int Discriptr, int Istochnic, int Priemnic){
  int result;
  result = Priemnic;
  result += (Istochnic << 6);
  result += (Discriptr << 12); 
  result += Prioritet << 26;
  return result;
}

/*******************************************************************************
*  @brief   CAN1_Receive - ���������� ������ ������ � ����� �����
*  @retval  none
*******************************************************************************/
void CAN1_Receive(){
  if(cnt1RxBuffCan1 > 0 || cnt2RxBuffCan1 > 0){
    CanMsgTypeDef *CanMsgRX;
    uint8_t cntRxBuffCan1;
    // ���������� 2� ������ ���� �� ��� ������ 1�
    if(cnt2RxBuffCan1  > 0){
      CanMsgRX = &buf2RxMsg[0];
      cntRxBuffCan1 = cnt2RxBuffCan1;
      cnt2RxBuffCan1 = 0;
      // ���������� 1� ������ (�� ���������)
    }else{
      isParseCan1 = 1;
      CanMsgRX = &buf1RxMsg[0];
      cntRxBuffCan1 = cnt1RxBuffCan1;
      cnt1RxBuffCan1 = 0;
    }
    for(int cnt=0; cnt<cntRxBuffCan1; cnt++){
      
      // ����� ��������� ��������� ���������
      CAN1_Parse(CanMsgRX, cnt);
      
    }
    isParseCan1 = 0;
  }
}

/*******************************************************************************
*  @brief   CAN2_Receive - ���������� ������ ������ � ����� �����
*  @retval  none
*******************************************************************************/
void CAN2_Receive(){
  if(cnt1RxBuffCan2 > 0 || cnt2RxBuffCan2 > 0){
    CanMsgTypeDef *CanMsgRX;
    uint8_t cntRxBuffCan2;
    // ���������� 2� ������ ���� �� ��� ������ 1�
    if(cnt2RxBuffCan2  > 0){
      CanMsgRX = &buf4RxMsg[0];
      cntRxBuffCan2 = cnt2RxBuffCan2;
      cnt2RxBuffCan2 = 0;
      // ���������� 1� ������ (�� ���������)
    }else{
      isParseCan2 = 1;
      CanMsgRX = &buf3RxMsg[0];
      cntRxBuffCan2 = cnt1RxBuffCan2;
      cnt1RxBuffCan2 = 0;
    }
    for(int cnt=0; cnt<cntRxBuffCan2; cnt++){
      
      // ����� ��������� ��������� ���������
      CAN2_Parse(CanMsgRX, cnt);
      
    }
    isParseCan2 = 0;
  }
}

//==============================================================================
//======================== ���������������� ������� ============================
//==============================================================================

/*******************************************************************************
*  @brief   CAN1_Receive - ������� ������ ��������� � ����� �����
*  @param   CanMsgRX - ��������� �������� ���������
*  @param   cnt - ����� ������ �������������� �� �������
*  @retval  none
*******************************************************************************/
void CAN1_Parse(CanMsgTypeDef *CanMsgRXx, int cnt){
  CanMsgTypeDef CanMsgTX;
  switch(CanMsgRXx[cnt].id){
  case 0xAAA:
    flag = 1;
    break;
  case 0x777:
    CanMsgTX.ide = EXT;
    CanMsgTX.len = 2;
    CanMsgTX.id = 0x1777;
    CanMsgTX.data[0] = CanMsgRXx[cnt].data[0];
    CanMsgTX.data[1] = 0xBB;
    sendCanMessage(CAN1, &CanMsgTX);
    break;
  case 0x10101010:
    CanMsgTX.ide = EXT;
    CanMsgTX.len = 2;
    CanMsgTX.id = 0x1;
    CanMsgTX.data[0] = Can1ErrorType.cntOwerReadBufRx;
    CanMsgTX.data[1] = Can1ErrorType.cntOwerWriteBufTX;
    sendCanMessage(CAN1, &CanMsgTX);
    break;
  }
}
/*******************************************************************************
*  @brief   CAN2_Receive - ������� ������ ��������� � ����� �����
*  @param   CanMsgRX - ��������� �������� ���������
*  @param   cnt - ����� ������ �������������� �� �������
*  @retval  none
*******************************************************************************/
void CAN2_Parse(CanMsgTypeDef *CanMsgRX, int cnt){
  CanMsgTypeDef CanMsgTX;
  switch(CanMsgRX[cnt].id){
  case 0xAAA:
    flag = 1;
    break;
  case 0x777:
    CanMsgTX.ide = EXT;
    CanMsgTX.len = 2;
    CanMsgTX.id = 0x2777;
    CanMsgTX.data[0] = CanMsgRX[cnt].data[0];
    CanMsgTX.data[1] = 0xBB;
    sendCanMessage(CAN2, &CanMsgTX);
    break;
  case 0x10101010:
    CanMsgTX.ide = EXT;
    CanMsgTX.len = 2;
    CanMsgTX.id = 0x2;
    CanMsgTX.data[0] = Can2ErrorType.cntOwerReadBufRx;
    CanMsgTX.data[1] = Can2ErrorType.cntOwerWriteBufTX;
    sendCanMessage(CAN2, &CanMsgTX);
    break;
  }
}