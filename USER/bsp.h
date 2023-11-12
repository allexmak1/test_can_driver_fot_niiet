/*==============================================================================
 * Определения для периферии платы КФДЛ.441546.016Б (NIIET-EVAL-K1921VK01T)
 *------------------------------------------------------------------------------
 * НИИЭТ, Богдан Колбов <kolbov@niiet.ru>
 *==============================================================================
 * ДАННОЕ ПРОГРАММНОЕ ОБЕСПЕЧЕНИЕ ПРЕДОСТАВЛЯЕТСЯ «КАК ЕСТЬ», БЕЗ КАКИХ-ЛИБО
 * ГАРАНТИЙ, ЯВНО ВЫРАЖЕННЫХ ИЛИ ПОДРАЗУМЕВАЕМЫХ, ВКЛЮЧАЯ ГАРАНТИИ ТОВАРНОЙ
 * ПРИГОДНОСТИ, СООТВЕТСТВИЯ ПО ЕГО КОНКРЕТНОМУ НАЗНАЧЕНИЮ И ОТСУТСТВИЯ
 * НАРУШЕНИЙ, НО НЕ ОГРАНИЧИВАЯСЬ ИМИ. ДАННОЕ ПРОГРАММНОЕ ОБЕСПЕЧЕНИЕ
 * ПРЕДНАЗНАЧЕНО ДЛЯ ОЗНАКОМИТЕЛЬНЫХ ЦЕЛЕЙ И НАПРАВЛЕНО ТОЛЬКО НА
 * ПРЕДОСТАВЛЕНИЕ ДОПОЛНИТЕЛЬНОЙ ИНФОРМАЦИИ О ПРОДУКТЕ, С ЦЕЛЬЮ СОХРАНИТЬ ВРЕМЯ
 * ПОТРЕБИТЕЛЮ. НИ В КАКОМ СЛУЧАЕ АВТОРЫ ИЛИ ПРАВООБЛАДАТЕЛИ НЕ НЕСУТ
 * ОТВЕТСТВЕННОСТИ ПО КАКИМ-ЛИБО ИСКАМ, ЗА ПРЯМОЙ ИЛИ КОСВЕННЫЙ УЩЕРБ, ИЛИ
 * ПО ИНЫМ ТРЕБОВАНИЯМ, ВОЗНИКШИМ ИЗ-ЗА ИСПОЛЬЗОВАНИЯ ПРОГРАММНОГО ОБЕСПЕЧЕНИЯ
 * ИЛИ ИНЫХ ДЕЙСТВИЙ С ПРОГРАММНЫМ ОБЕСПЕЧЕНИЕМ.
 *
 *                              2018 АО "НИИЭТ"
 *==============================================================================
 */

#ifndef BSP_H
#define BSP_H

#ifdef __cplusplus
extern "C" {
#endif

//-- Includes ------------------------------------------------------------------
#include "K1921VK01T.h"
#include <stdio.h>

//-- Defines -------------------------------------------------------------------
//LEDs
#define LED_PORT NT_GPIOG
#define LED_PORT_DEN GPIODENG
#define LED_PIN_MSK 0xFF00
#define LED0_POS 8
#define LED1_POS 9
#define LED2_POS 10
#define LED3_POS 11
#define LED4_POS 12
#define LED5_POS 13
#define LED6_POS 14
#define LED7_POS 15
#define LED0_MSK (1 << LED0_POS)
#define LED1_MSK (1 << LED1_POS)
#define LED2_MSK (1 << LED2_POS)
#define LED3_MSK (1 << LED3_POS)
#define LED4_MSK (1 << LED4_POS)
#define LED5_MSK (1 << LED5_POS)
#define LED6_MSK (1 << LED6_POS)
#define LED7_MSK (1 << LED7_POS)

//Buttons SB2, SB3
#define BTN_PORT NT_GPIOE
#define BTN_PORT_EN GPIODENE
#define BTN_IRQ_N GPIOE_IRQn
#define BTN_IRQ_HANDLER GPIOE_IRQHandler
#define BTN_SB2_PIN_POS 9
#define BTN_SB2_PIN_MSK (1 << BTN_SB2_PIN_POS)
#define BTN_SB3_PIN_POS 8
#define BTN_SB3_PIN_MSK (1 << BTN_SB3_PIN_POS)

//-- Types ---------------------------------------------------------------------
typedef enum {
    BSP_Btn_State_None,
    BSP_Btn_State_SB2,
    BSP_Btn_State_SB3,
    BSP_Btn_State_Both
} BSP_Btn_State_TypeDef;

//-- Functions -----------------------------------------------------------------
void BSP_LED_Init(void);
void BSP_LED_On(uint32_t leds);
void BSP_LED_Off(uint32_t leds);
void BSP_LED_Toggle(uint32_t leds);
void BSP_Btn_Init(void);
BSP_Btn_State_TypeDef BSP_Btn_GetState(void);
void BSP_ETH_Init(void);
void BSP_CAN_Init(void);
void BSP_USB_Init(void);

#ifdef __cplusplus
}
#endif

#endif // BSP_H
