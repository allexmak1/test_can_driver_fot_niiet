#include "GPIO.h"
#include "niietcm4.h"

/*******************************************************************************
*  @brief   initGpio - инициализация портов ввода вывода
*  @param   none
*  @retval  none
*******************************************************************************/
void initGpio(){
  //выхода
  GPIO_Init_TypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Dir = GPIO_Dir_Out;
  GPIO_InitStruct.GPIO_Out = GPIO_Out_En;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
  GPIO_Init(NT_GPIOG, &GPIO_InitStruct);
  GPIO_SetBits(NT_GPIOG, GPIO_Pin_15);  

  //входа

  
}
