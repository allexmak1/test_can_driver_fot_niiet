#include "CPUconfig.h"
#include "K1921VK01T.h"
#include "niietcm4.h"
/*******************************************************************************
*  @brief   initSystemCoreClock - ������������� ������������ CPU
*  @param   none
*  @retval  none
*******************************************************************************/
void initSystemCoreClock(){
  //��������� ������������
  RCC_PLLAutoConfig(RCC_PLLRef_XI_OSC, 100000000); // 100���
  //RCC_SysClkDiv2Out(ENABLE); // ���� H0 = ������� ������������ ������� �� 2
}

/*******************************************************************************
*  @brief   initTimer - ������������� �������
*  @param   none
*  @retval  none
*******************************************************************************/
void initTimer(){
  RCC_PeriphRstCmd(RCC_PeriphRst_Timer0, ENABLE);
  TIMER_FreqConfig(NT_TIMER0, 100000000, 1000); //�� 1���
  TIMER_ITCmd(NT_TIMER0, ENABLE);
  TIMER_Cmd(NT_TIMER0, ENABLE);
  NVIC_EnableIRQ(TIM0_IRQn);
}

/*******************************************************************************
*  @brief   Delay - ��������
*  @param   waitTicks - ���-�� ������
*  @retval  none
*******************************************************************************/
void Delay(int waitTicks){
  int i;
  for (i = 0; i < waitTicks; i++){
    __NOP();
  }	
}
