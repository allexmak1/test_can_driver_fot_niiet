#include "K1921VK01T.h"
#include "niietcm4.h"
#include "main.h"
#include "K1921VK01T_it.h"

timer_t timer;

void SysTick_Handler(){
}

//таймер на строен на 1мс
void TIM0_IRQHandler(){
//  мигание светодиода G15 1s
  if(timer.cnt_1ms++ > 1000){
    timer.cnt_1ms = 0;
    timer.cnt_1s++;
    GPIO_ToggleBits(NT_GPIOG, GPIO_Pin_15);
  }
  TIMER_ITStatusClear(NT_TIMER0);
}