/*******************************************************************************
* Project: ���� �������� �������� CAN
*           ��� �������� � CAN1 ��� CAN2 ���������:
*           - � �������� ���������� id = 0x��� 
*             ���������� �������� ��������� 15 ��������� ������  id = 0x1���
*             �� ����� �� ���������� 10-1 �.�. ���������� ����� �� �������� 10 �����, �� ���� �� ��� ����� ��� ������ ������������
*             ��������� �� ������ � ��������� ������, ��������� Can1ErrorType.cntOwerWriteBufTX = 6
*           - ��� �������� ������ ����� ������ ���������� (� �������� ���������)
*             �� ��� ���������� ������ id = 0x777
*             ������� �������� � ������� ��������� id = 0x1777
*           - id = 0x10101010
*             ������� �������� ������ ������������ ������ CAN
* Autor: ������� �.�.(46013)
* Date: 11.10.22
* Version: 2.0.0
*******************************************************************************/

#include "main.h"
#include "K1921VK01T.h"
#include <stdio.h>
#include "CPUconfig.h"
#include "GPIO.h"
#include "CAN.h"
#include "K1921VK01T_it.h"

extern timer_t timer ;
int flag;

int main(){
  initSystemCoreClock();
  initTimer();
  initGpio();
  initCAN(CAN1, SPEED_250kbs);
  initCAN(CAN2, SPEED_250kbs);
  GPIO_SetBits(NT_GPIOG, GPIO_Pin_15);//��������
  
  while (1){
    CAN1_Receive();
    CAN2_Receive();
    Delay(10000000); //�������� �����
    //���� �������� ����� "n" ���������
    int n = 15;
    if(flag){
      flag = 0;
      CanMsgTypeDef CanMsg;
      CanMsg.ide = EXT;
      CanMsg.len = 8;
      CanMsg.id = 0x1AAA;
      CanMsg.data[0] = 0xAA;
      CanMsg.data[1] = 0xAA;
      CanMsg.data[2] = 0xAA;
      CanMsg.data[3] = 0xAA;
      CanMsg.data[4] = 0xAA;
      CanMsg.data[5] = 0xAA;
      CanMsg.data[6] = 0xAA;
      CanMsg.data[7] = 0xAA;
      for(int i = 1; i<=n; i++){
        CanMsg.data[0] = i;
        CanMsg.id = 0x1AAA;
        sendCanMessage(CAN1, &CanMsg);
      }
      Delay(100000);
      for(int i = 1; i<=n; i++){
        CanMsg.data[0] = i;
        CanMsg.id = 0x2AAA;
        sendCanMessage(CAN2, &CanMsg);
      }
    }

    // !!!
    // ��� �������� � ��� can ������ ����� �������� (������ ���, �� ����������)
//    CanMsg.id = 0x1;
//    sendCanMessage(CAN1, &CanMsg);
//    Delay(500);// !!!
//    CanMsg.id = 0x2;
//    sendCanMessage(CAN2, &CanMsg);
  }
}
