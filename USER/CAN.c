/*******************************************************************************
* Lib: Библиотека драйвера CAN
*           Настройки драйвера:
*           - аппаратный буфер can1 приема:    10 ячеек(0-9)
*           - аппаратный буфер can1 отправки:  10 ячеек(10-19)
*           - аппаратный буфер can2 приема:    10 ячеек(20-29)
*           - аппаратный буфер can2 отправки:  10 ячеек(30-39)
*           всего ячеек 256
*           Логика работы отправки:
*             отправка сообщений осуществляется функцией sendCanMessage().
*             первое сообщение отправляется по текущему CUR регистру базового сообщения,
*             при отправки следущего если CUR не обновилось, то записываем в буфер отправки и
*             сообщение отправится сразу же после предидущего.
*           Логика работы приема:
*             прием сообщений осуществляется прерыванием CAN12_IRQHandler() или функцией в цикле CAN1_Receive()
*             для удобство вынесена отдельная функция приема в цикле CAN1_Parse() в ней можно обрабатывать 
*             сообщения или в самом прерывании (строки с приером) сушествует 2 массива сообщений на каждый can.
*             Осовной заполняется когда приходят сообщения из прирывания, а дополнительный для того чтобы в него 
*             приходили сообщения когда происходит прерывание и мы уже парсим сообщения из основного массива, и наоборот.
*             Это сделанно для того чтобы небыло битых сообщений.
* Autor: Макаров А.В.(46013)
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
*  @brief   initCAN - инициализация CAN
*  @param   CAN_e - номер can
*  @param   CAN_SPEED_e - скорость
*  @retval  none
*******************************************************************************/
void initCAN(CAN_e CANx, CAN_SPEED_e speed){
  // инициализация ног
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
  
  // конфигурация can
  NT_CAN->CLC_bit.DISR = 0;                                                     // включение контроллера CAN
  while (NT_CAN->CLC_bit.DISS == 1) {};                                         // ждем включения  контроллера CAN
  while (NT_CAN->PANCTR_bit.PANCMD == 1) {};                                    // регистр панели команд должен быть 0
  NT_CAN->FDR = (0x1 << CAN_FDR_DM_Pos) | (0x3FF << CAN_FDR_STEP_Pos);          // нормальный режим работы и предделитьель 1 (частота шины CAN 100мГц)
  
  if(CANx == CAN1){
    NT_CAN->CAN_Node[0].NCR_bit.CCE = 1;                                        // бит разрешения изменения конфигурации узла. (для изменения регистров настройки)
    NT_CAN->CAN_Node[0].NCR_bit.INIT = 1;                                       // прекращает участие узла в трафике.
    NT_CAN->CAN_Node[0].NPCR_bit.LBM = 0;                                       // выключаем режим обратной петли
    NT_CAN->CAN_Node[0].NIPR = 0;                                               // Регистр указателя прерываний узла
    NT_CAN->CAN_Node[0].NIPR = (0xC << CAN_NIPR_TRINP_Pos);                     // "12" Указатель линии прерывания, для прерывания по окончании передачи/приема сообщения    
    NT_CAN->CAN_Node[0].NFCR = 0;                                               // Регистр счетчика сообщений узла
    
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
    NT_CAN->CAN_Node[0].NCR = 0;                                                // закрываем изменение регистров
    NT_CAN->CAN_Node[0].NCR = CAN_NCR_TRIE_Msk;                                 // разрешено прерывание по приему чужого и своего сообщения (CAN12_IRQn)
    
    // настраиваем can1
    CanCreateObjectList(CAN1, CAN1_RX_START_LIST_OBJECT, CAN1_RX_MAX_LIST_OBJECT, CAN1_TX_START_LIST_OBJECT, CAN1_TX_MAX_LIST_OBJECT);       // создаем список объектов
    CreateRXFifo(CAN1_RX_START_LIST_OBJECT, CAN1_RX_MAX_LIST_OBJECT);           // создаем FIFO для получения
    CreateTXFifo(CAN1_TX_START_LIST_OBJECT, CAN1_TX_MAX_LIST_OBJECT);           // создаем FIFO для отправки
    NVIC_EnableIRQ(CAN12_IRQn);                                                 // активируем прерывания
    
    
  }else if(CANx == CAN2){
    NT_CAN->CAN_Node[1].NCR_bit.CCE = 1;                                        // бит разрешения изменения конфигурации узла. (для изменения регистров настройки)
    NT_CAN->CAN_Node[1].NCR_bit.INIT = 1;                                       // прекращает участие узла в трафике.
    NT_CAN->CAN_Node[1].NPCR_bit.LBM = 0;                                       // выключаем режим обратной петли
    NT_CAN->CAN_Node[1].NIPR = 0;                                               // Регистр указателя прерываний узла
    NT_CAN->CAN_Node[1].NIPR = (0xF << CAN_NIPR_TRINP_Pos);                     // "12" Указатель линии прерывания, для прерывания по окончании передачи/приема сообщения    
    NT_CAN->CAN_Node[1].NFCR = 0;                                               // Регистр счетчика сообщений узла
    
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
    NT_CAN->CAN_Node[1].NCR = 0;                                                // закрываем изменение регистров
    NT_CAN->CAN_Node[1].NCR = CAN_NCR_TRIE_Msk;                                 // разрешено прерывание по приему чужого и своего сообщения (CAN15_IRQn)
    
    // настраиваем can2
    CanCreateObjectList(CAN2, CAN2_RX_START_LIST_OBJECT, CAN2_RX_MAX_LIST_OBJECT, CAN2_TX_START_LIST_OBJECT, CAN2_TX_MAX_LIST_OBJECT);      // создаем список объектов
    CreateRXFifo(CAN2_RX_START_LIST_OBJECT, CAN2_RX_MAX_LIST_OBJECT);           // создаем FIFO для получения
    CreateTXFifo(CAN2_TX_START_LIST_OBJECT, CAN2_TX_MAX_LIST_OBJECT);           // создаем FIFO для отправки
    NVIC_EnableIRQ(CAN15_IRQn);                                                 // активируем прерывания
  }
}

/*******************************************************************************
*  @brief   CanCreateObjectList - создание списка объектов для работы
*******************************************************************************/
void CanCreateObjectList(uint32_t can, uint8_t rxStartListObject, uint8_t rxMaxListObject, uint8_t txStartListObject, uint8_t txMaxListObject){
  // формируем длинну списка для одного из can
  // can0 список 1, can1 список 2 
  for (int i = rxStartListObject; i <= txMaxListObject; i++){
    while (NT_CAN->PANCTR & (3<<8));                                    // ждем завершения работы панели команд
    NT_CAN->PANCTR = (2<< CAN_PANCTR_PANCMD_Pos) |                      // команда управления (занести объект в список)
      ((i)<< CAN_PANCTR_PANAR1_Pos) |                                   // аргумент 1 (номер объекта)
        ((can+1)<< CAN_PANCTR_PANAR2_Pos);                              // аргумент 2 (номер списка)
  }
  
  // настройка приемных буферов
  for (int i = rxStartListObject; i <= rxMaxListObject; i++){
    NT_CAN->CAN_Msg[i].MOCTR = 0xFFF;                                  // сброс всех флагов (через биты RESET)
    // объект для приема сообщений (DIR=0)
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETRXEN_Pos);            // разрешаем прием сообщений (RXEN = 1)
    NT_CAN->CAN_Msg[i].MOFCR  = (1<<CAN_MOFCR_RXIE_Pos);               // прерываниt по приему, стандартный объект приема.
    NT_CAN->CAN_Msg[i].MOAR   = (2<<CAN_MOAR_PRI_Pos) |                // тип фильтрации
      (1<<CAN_MOAR_IDE_Pos) |                                          // работа только с расширенными сообщениями
        0x00;                                                          //
    NT_CAN->CAN_Msg[i].MOAMR  = (0<<CAN_MOAMR_MIDE_Pos) |              // бит IDE при фильтрации не должен совпадать (любое сообщение)
      0;                                                               // маска для фильтрации сообщений (0 - прием всех)
    NT_CAN->CAN_Msg[i].MOFGPR = 0;                                     // FIFO не используется
    NT_CAN->CAN_Msg[i].MOIPR  = (i<<CAN_MOIPR_MPN_Pos);                // прерывания, Номер ждущего бита сообщения.
    // указываем бит для ждущего сообщения
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETMSGVAL_Pos);          // разрешаем работу объекта (MSGVAL = 1)
  }
  
  // настройка буферов на передачу
  for (int i = txStartListObject; i <= txMaxListObject; i++){
    NT_CAN->CAN_Msg[i].MOCTR = 0xFFF;                                  // сброс всех флагов (через биты RESET)
    // объект для передачи сообщений (DIR=1)
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETTXEN0_Pos)|           // разрешаем передачу сообщений (TXEN0 = 1)
      (1<<CAN_MOCTR_SETTXEN1_Pos)|                                     // разрешаем передачу сообщений (TXEN1 = 1)
        (1<<CAN_MOCTR_SETDIR_Pos);                                     // разрешаем передачу сообщений (DIR = 1)
    NT_CAN->CAN_Msg[i].MOAR   = (1<<CAN_MOAR_PRI_Pos) |                // тип фильтрации
      (1<<CAN_MOAR_IDE_Pos) |                                          // работа только с расширенными сообщениями
        0x0;
    NT_CAN->CAN_Msg[i].MOAMR  = (0<<CAN_MOAMR_MIDE_Pos) |              // бит IDE при фильтрации должен совпадать
      0x0;                                                             // маска для фильтрации сообщений
    NT_CAN->CAN_Msg[i].MOFGPR = 0;                                     // FIFO не используется
    NT_CAN->CAN_Msg[i].MOIPR  = (i<<CAN_MOIPR_MPN_Pos);                // прерывания, Номер ждущего бита сообщения.
    // указываем бит для ждущего сообщения
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETMSGVAL_Pos);          // разрешаем работу объекта (MSGVAL = 1)
  }
}

/*******************************************************************************
*  @brief   CreateTXFifo - Формируем FIFO структуру для передачи
*******************************************************************************/
void CreateTXFifo (uint32_t base_num, uint32_t end_num){
  // базовый объект
  NT_CAN->CAN_Msg[base_num].MOCTR  = 0xFFF;                                     // сброс всех флагов (через биты RESET)
  NT_CAN->CAN_Msg[base_num].MOCTR  = (1<<CAN_MOCTR_SETTXEN0_Pos)|               // разрешаем передачу сообщений (TXEN0 = 1)
    (1<<CAN_MOCTR_SETTXEN1_Pos)|                                                // разрешаем передачу сообщений (TXEN1 = 1)
      (1<<CAN_MOCTR_SETDIR_Pos);                                                // разрешаем передачу сообщений (DIR = 1)
  NT_CAN->CAN_Msg[base_num].MOFCR  = (2<<CAN_MOFCR_MMC_Pos)|                    // базовый объект передачи (без бита активности)
    (0<<CAN_MOFCR_OVIE_Pos)|                                                    // бит разрешения прерывания по заполнению буфера
      (1<<CAN_MOFCR_STT_Pos);                                                   // в случае неудачной передачи, повторной передачи сообщения не будет
  NT_CAN->CAN_Msg[base_num].MOFGPR = ((base_num/*+1*/)<<CAN_MOFGPR_BOT_Pos) |       // номер первого элемента (базовый  используется)
    (end_num<<CAN_MOFGPR_TOP_Pos) |                                             // номер последнего элемента 
      (base_num<<CAN_MOFGPR_CUR_Pos) |                                          // номер первого элемента для считывания
        (0<<CAN_MOFGPR_SEL_Pos);                                                // номер элемента для контроля за загрузкой FIFO (не обязательно)----попробовать
  NT_CAN->CAN_Msg[base_num].MOIPR  = (base_num<<CAN_MOIPR_MPN_Pos);             // прерывания не используются
  NT_CAN->CAN_Msg[base_num].MOCTR  = (1<<CAN_MOCTR_SETMSGVAL_Pos);              // разрешаем работу объекта (MSGVAL = 1)
  
  // вспомогательные объекты
  for (int i=base_num+1;i<=end_num;i++){
    NT_CAN->CAN_Msg[i].MOCTR  = 0xFFF;                                          // сброс всех флагов (через биты RESET)
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETTXEN0_Pos)|                    // разрешаем передачу сообщений (TXEN0 = 1)
      (1<<CAN_MOCTR_SETDIR_Pos);                                                // разрешаем передачу сообщений (DIR = 1)
    NT_CAN->CAN_Msg[i].MOFCR  = (3<<CAN_MOFCR_MMC_Pos)|                         // вспомогательный объект FIFO передачи (обязательно)
      (0<<CAN_MOFCR_OVIE_Pos)|                                                  // бит разрешения прерывания по заполнению буфера
        (1<<CAN_MOFCR_STT_Pos);                                                 // в случае неудачной передачи, повторной передачи сообщения не будет
    NT_CAN->CAN_Msg[i].MOFGPR = (base_num<<CAN_MOFGPR_CUR_Pos);                 // базовый объект
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETMSGVAL_Pos);                   // разрешаем работу объекта (MSGVAL = 1)
    NT_CAN->CAN_Msg[i].MOFCR_bit.DLC = 8;
  }
  
}

/*******************************************************************************
*  @brief   CreateRXFifo - Формируем FIFO структуру для приема
*******************************************************************************/
void CreateRXFifo (uint32_t base_num, uint32_t end_num){
  // базовый объект
  NT_CAN->CAN_Msg[base_num].MOCTR  = 0xFFF;                                     // сброс всех флагов (через биты RESET)
  NT_CAN->CAN_Msg[base_num].MOCTR  = (1<<CAN_MOCTR_SETRXEN_Pos);                // разрешаем прием сообщений
  NT_CAN->CAN_Msg[base_num].MOFCR  = (1<<CAN_MOFCR_RXIE_Pos)|                   // прерывание по приему
    (1<<CAN_MOFCR_OVIE_Pos)|                                                    // бит разрешения прерывания по заполнению буфера
      (1<<CAN_MOFCR_MMC_Pos);                                                     // базовый объект приема
  NT_CAN->CAN_Msg[base_num].MOFGPR = ((base_num/*+1*/)<<CAN_MOFGPR_BOT_Pos) |       // номер первого элемента (базовый используется)
    (end_num<<CAN_MOFGPR_TOP_Pos) |                                             // номер последнего элемента 
      ((base_num+1)<<CAN_MOFGPR_CUR_Pos) |                                      // номер первого элемента для записи
        (0<<CAN_MOFGPR_SEL_Pos);                                                // номер элемента для контроля за загрузкой FIFO (не обязательно)----попробовать
  NT_CAN->CAN_Msg[base_num].MOCTR  = (1<<CAN_MOCTR_SETMSGVAL_Pos);              // разрешаем работу объекта (MSGVAL = 1)
  
  // вспомогательные объекты
  for (int i=base_num+1;i<=end_num;i++){
    NT_CAN->CAN_Msg[i].MOCTR  = 0xFFF;                                          // сброс всех флагов (через биты RESET)
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_RESTXEN0_Pos);
    NT_CAN->CAN_Msg[i].MOFCR  = (0<<CAN_MOFCR_MMC_Pos);                         // игнорируется
    NT_CAN->CAN_Msg[i].MOFGPR = (base_num<<CAN_MOFGPR_CUR_Pos);                 // базовый объект
    NT_CAN->CAN_Msg[i].MOCTR  = (1<<CAN_MOCTR_SETMSGVAL_Pos);                   // разрешаем работу объекта (MSGVAL = 1)
    
    NT_CAN->CAN_Msg[i].MOFCR_bit.DLC = 8;
    
  }
}
/*******************************************************************************
*  @brief   CAN12_IRQHandler - прерывание входящих сообщений CAN1
*******************************************************************************/
void CAN12_IRQHandler(void){
  CanMsgTypeDef *bufRxMsg;
  uint8_t cntRxBuffCan1;
  if(!isParseCan1){
    bufRxMsg =  &buf1RxMsg[0];
    cntRxBuffCan1 = cnt1RxBuffCan1;
    cnt1RxBuffCan1++;
    // превыщен лимит массива
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
  
  // определяем текущий аппаратно записаный буфер
  int obj = NT_CAN->CAN_Msg[CAN1_RX_START_LIST_OBJECT].MOFGPR_bit.CUR - 1;
  if(obj == CAN1_RX_START_LIST_OBJECT-1)obj = CAN1_RX_MAX_LIST_OBJECT;
  // закидываем в буфер
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
  
  // обнуляем регистр
  NT_CAN->CAN_Msg[obj].MOAR_bit.ID = 0;
  NT_CAN->CAN_Msg[obj].MODATAL = 0;
  NT_CAN->CAN_Msg[obj].MODATAH = 0;
  
  // можем обработку входящих сообщений сделать здесь в прерывании.
  /* Пример:
  if(bufRxMsg[cntRxBuffCan1].id == 0xAAA){
  int data = bufRxMsg[cntRxBuffCan1].data[0];
  flag = 1;
}
  */
}

/*******************************************************************************
*  @brief   CAN15_IRQHandler - прерывание входящих сообщений CAN2
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
  
  // определяем текущий аппаратно записаный буфер
  int obj = NT_CAN->CAN_Msg[CAN2_RX_START_LIST_OBJECT].MOFGPR_bit.CUR - 1;
  if(obj == CAN2_RX_START_LIST_OBJECT-1)obj = CAN2_RX_MAX_LIST_OBJECT;
  // закидываем в буфер
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
  // обнуляем регистр
  NT_CAN->CAN_Msg[obj].MOAR_bit.ID = 0;
  NT_CAN->CAN_Msg[obj].MODATAL = 0;
  NT_CAN->CAN_Msg[obj].MODATAH = 0;
  
  // можем обработку входящих сообщений сделать здесь в прерывании.
  /* Пример:
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
*  @param   TxMessage - заранее подготовленная структура отправляемого сообщения
*  @retval  none
*******************************************************************************/
void sendCanMessage(CAN_e CANx, CanMsgTypeDef* TxMessage){
  uint16_t cur, mbNum;
  if(CANx == CAN1){
    cur = NT_CAN->CAN_Msg[CAN1_TX_START_LIST_OBJECT].MOFGPR_bit.CUR;
    //сделанно так, из-за того, что при использовании проверки регистра NT_CAN->CAN_Msg[mbNum].MOSTAT_bit.TXRQ == 1 
    //                                контроллер не выставляет 1 при передачи второго сообщения
    if(stepSendCan1 == 0){
      if(cur != oldCurCan1){
        mbNumCan1 = cur;
        oldCurCan1 = cur;
        stepSendCan1 = 0;
        cntCurrentCurCan1 = 0;
      }else{
        if(++mbNumCan1 > CAN1_TX_MAX_LIST_OBJECT) mbNumCan1 = CAN1_TX_START_LIST_OBJECT;
        //переходим к ошибкам
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
    //сделанно так, из-за того, что при использовании проверки регистра NT_CAN->CAN_Msg[mbNum].MOSTAT_bit.TXRQ == 1 
    //                                контроллер не выставляет 1 при передачи второго сообщения
    if(stepSendCan2 == 0){
      if(cur != oldCurCan2){
        mbNumCan2 = cur;
        oldCurCan2 = cur;
        stepSendCan2 = 0;
      }else{
        if(++mbNumCan2 > CAN2_TX_MAX_LIST_OBJECT) mbNumCan2 = CAN2_TX_START_LIST_OBJECT;
        //переходим к ошибкам
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
  
  
  // отправляем
  NT_CAN->CAN_Msg[mbNum].MOAR = 0;
  // приоритет отправки оп высокому ID
  NT_CAN->CAN_Msg[mbNum].MOAR_bit.PRI = 2;                                      
  // расширенный/стандартный идентификатор
  if(TxMessage->ide <= 1)
    NT_CAN->CAN_Msg[mbNum].MOAR_bit.IDE = TxMessage->ide;
  // id
  if(NT_CAN->CAN_Msg[mbNum].MOAR_bit.IDE)
    NT_CAN->CAN_Msg[mbNum].MOAR_bit.ID = TxMessage->id;
  else
    NT_CAN->CAN_Msg[mbNum].MOAR_bit.ID = TxMessage->id<<18;
  // длинна 
  if(TxMessage->len <= 8) 
    NT_CAN->CAN_Msg[mbNum].MOFCR_bit.DLC = TxMessage->len;
  // данные
  NT_CAN->CAN_Msg[mbNum].MODATAL = (uint32_t)TxMessage->data[0];
  NT_CAN->CAN_Msg[mbNum].MODATAL |= (uint32_t)TxMessage->data[1]<<8;
  NT_CAN->CAN_Msg[mbNum].MODATAL |= (uint32_t)TxMessage->data[2]<<16;
  NT_CAN->CAN_Msg[mbNum].MODATAL |= (uint32_t)TxMessage->data[3]<<24;
  NT_CAN->CAN_Msg[mbNum].MODATAH = (uint32_t)TxMessage->data[4];
  NT_CAN->CAN_Msg[mbNum].MODATAH |= (uint32_t)TxMessage->data[5]<<8;
  NT_CAN->CAN_Msg[mbNum].MODATAH |= (uint32_t)TxMessage->data[6]<<16;
  NT_CAN->CAN_Msg[mbNum].MODATAH |= (uint32_t)TxMessage->data[7]<<24;
  
  NT_CAN->CAN_Msg[mbNum].MOCTR = CAN_MOCTR_SETMSGVAL_Msk | CAN_MOCTR_SETTXRQ_Msk ; //CAN_MOCTR_SETMSGVAL_Msk - коррктность объекта, CAN_MOCTR_SETTXRQ_Msk - инициирует передачу фрейма
}

/*******************************************************************************
*  @brief   sendCanTest - тестовая отправка сообщения
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
*  @brief   xMakeId - (калькулятор) сборка дискриптора (3-14-6-6)
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
*  @brief   CAN1_Receive - Подготовка буфера приема в общем цикле
*  @retval  none
*******************************************************************************/
void CAN1_Receive(){
  if(cnt1RxBuffCan1 > 0 || cnt2RxBuffCan1 > 0){
    CanMsgTypeDef *CanMsgRX;
    uint8_t cntRxBuffCan1;
    // определяем 2й массив если мы уже парсим 1й
    if(cnt2RxBuffCan1  > 0){
      CanMsgRX = &buf2RxMsg[0];
      cntRxBuffCan1 = cnt2RxBuffCan1;
      cnt2RxBuffCan1 = 0;
      // определяем 1й массив (по умолчанию)
    }else{
      isParseCan1 = 1;
      CanMsgRX = &buf1RxMsg[0];
      cntRxBuffCan1 = cnt1RxBuffCan1;
      cnt1RxBuffCan1 = 0;
    }
    for(int cnt=0; cnt<cntRxBuffCan1; cnt++){
      
      // здесь обработка принятого сообщения
      CAN1_Parse(CanMsgRX, cnt);
      
    }
    isParseCan1 = 0;
  }
}

/*******************************************************************************
*  @brief   CAN2_Receive - Подготовка буфера приема в общем цикле
*  @retval  none
*******************************************************************************/
void CAN2_Receive(){
  if(cnt1RxBuffCan2 > 0 || cnt2RxBuffCan2 > 0){
    CanMsgTypeDef *CanMsgRX;
    uint8_t cntRxBuffCan2;
    // определяем 2й массив если мы уже парсим 1й
    if(cnt2RxBuffCan2  > 0){
      CanMsgRX = &buf4RxMsg[0];
      cntRxBuffCan2 = cnt2RxBuffCan2;
      cnt2RxBuffCan2 = 0;
      // определяем 1й массив (по умолчанию)
    }else{
      isParseCan2 = 1;
      CanMsgRX = &buf3RxMsg[0];
      cntRxBuffCan2 = cnt1RxBuffCan2;
      cnt1RxBuffCan2 = 0;
    }
    for(int cnt=0; cnt<cntRxBuffCan2; cnt++){
      
      // здесь обработка принятого сообщения
      CAN2_Parse(CanMsgRX, cnt);
      
    }
    isParseCan2 = 0;
  }
}

//==============================================================================
//======================== пользовательские функции ============================
//==============================================================================

/*******************************************************************************
*  @brief   CAN1_Receive - Парсинг буфера сообщений в общем цикле
*  @param   CanMsgRX - структура приятого сообщения
*  @param   cnt - номер данных вытаскиваемого из массива
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
*  @brief   CAN2_Receive - Парсинг буфера сообщений в общем цикле
*  @param   CanMsgRX - структура приятого сообщения
*  @param   cnt - номер данных вытаскиваемого из массива
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