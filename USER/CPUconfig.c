#include "CPUconfig.h"
#include "K1921VK01T.h"
#include "niietcm4.h"
/*******************************************************************************
*  @brief   initSystemCoreClock - инициализация тактирования CPU
*  @param   none
*  @retval  none
*******************************************************************************/
void initSystemCoreClock(){
  //настройка тактирования
  RCC_PLLAutoConfig(RCC_PLLRef_XI_OSC, 100000000); // 100мГц
  //RCC_SysClkDiv2Out(ENABLE); // нога H0 = частота тактирования делённая на 2
}

/*******************************************************************************
*  @brief   initTimer - инициализация таймера
*  @param   none
*  @retval  none
*******************************************************************************/
void initTimer(){
  RCC_PeriphRstCmd(RCC_PeriphRst_Timer0, ENABLE);
  TIMER_FreqConfig(NT_TIMER0, 100000000, 1000); //на 1млс
  TIMER_ITCmd(NT_TIMER0, ENABLE);
  TIMER_Cmd(NT_TIMER0, ENABLE);
  NVIC_EnableIRQ(TIM0_IRQn);
}

/*******************************************************************************
*  @brief   Delay - задержка
*  @param   waitTicks - кол-во циклов
*  @retval  none
*******************************************************************************/
void Delay(int waitTicks){
  int i;
  for (i = 0; i < waitTicks; i++){
    __NOP();
  }	
}
