/*==============================================================================
 * Управление периферией на плате КФДЛ.441546.016Б (NIIET-EVAL-K1921VK01T)
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

//-- Includes ------------------------------------------------------------------
#include "bsp.h"

//-- Private variables ---------------------------------------------------------
static volatile union {
    struct {
        uint32_t SB2 : 1;
        uint32_t SB3 : 1;
    } State_bits;
    BSP_Btn_State_TypeDef State;
} Buttons;


//-- Functions -----------------------------------------------------------------
void BSP_LED_Init()
{
    LED_PORT->OUTENSET = LED_PIN_MSK;
    NT_COMMON_REG->LED_PORT_DEN |= LED_PIN_MSK;
    BSP_LED_Off(LED_PIN_MSK);
}

void BSP_LED_On(uint32_t leds)
{
    LED_PORT->DATAOUT &= ~leds;
}

void BSP_LED_Off(uint32_t leds)
{
    LED_PORT->DATAOUT |= leds;
}

void BSP_LED_Toggle(uint32_t leds)
{
    LED_PORT->DATAOUT ^= leds;
}

void BSP_Btn_Init()
{
    // Настройка выводов
    // SB2 - E9 (pulled-down)
    // SB3 - E10 (pulled-down)
    BTN_PORT->INTTYPESET = (1 << BTN_SB2_PIN_POS) | (1 << BTN_SB3_PIN_POS); // фронт
    BTN_PORT->INTPOLSET = (1 << BTN_SB2_PIN_POS) | (1 << BTN_SB3_PIN_POS);  // положительный
    BTN_PORT->INTENSET = BTN_SB2_PIN_MSK | BTN_SB3_PIN_MSK;
    NT_COMMON_REG->BTN_PORT_EN = BTN_SB2_PIN_MSK | BTN_SB3_PIN_MSK;
    NVIC_EnableIRQ(BTN_IRQ_N);
}

BSP_Btn_State_TypeDef BSP_Btn_GetState(void)
{
    if (!(BTN_PORT->DATA & BTN_SB2_PIN_MSK))
         Buttons.State_bits.SB2 = 0;
    if (!(BTN_PORT->DATA & BTN_SB3_PIN_MSK))
         Buttons.State_bits.SB3 = 0;

    return Buttons.State;
}

void BSP_ETH_Init(void)
{
    uint32_t tmp;

    // MII_RXER   - PD10 (af1)
    // MII_RXDV   - PD9  (af1)
    // MII_RXD[3] - PD8  (af1)
    // MII_RXD[2] - PD7  (af1)
    // MII_RXD[1] - PD6  (af1)
    // MII_RXD[0] - PD5  (af1)
    // MII_RXCLK  - PD4  (af1)
    // MII_MDIO   - PD3  (af1)
    // MII_MDC    - PD2  (af1)
    // MII_COL    - PD1  (af1)
    // MII_CRS    - PD0  (af1)
    tmp = (1 << 10) | (1 << 9) | (1 << 8) | (1 << 7) | (1 << 6) | (1 << 5) |
          (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0);
    NT_GPIOD->ALTFUNCSET |= tmp;
    NT_COMMON_REG->GPIOPCTLD &= ~((3 << (2 * 10)) |
                                  (3 << (2 * 9)) |
                                  (3 << (2 * 8)) |
                                  (3 << (2 * 7)) |
                                  (3 << (2 * 6)) |
                                  (3 << (2 * 5)) |
                                  (3 << (2 * 4)) |
                                  (3 << (2 * 3)) |
                                  (3 << (2 * 2)) |
                                  (3 << (2 * 1)) |
                                  (3 << (2 * 0)));
    NT_COMMON_REG->GPIODEND |= tmp;

    // MII_TXER   - PC5  (af1)
    // MII_TXEN   - PC4  (af1)
    // MII_TXD[3] - PC3  (af1)
    // MII_TXD[2] - PC2  (af1)
    // MII_TXD[1] - PC1  (af1)
    // MII_TXD[0] - PC0  (af1)
    tmp = (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0);
    NT_GPIOC->ALTFUNCSET |= tmp;
    NT_COMMON_REG->GPIOPCTLC &= ~((3 << (2 * 4)) |
                                  (3 << (2 * 3)) |
                                  (3 << (2 * 2)) |
                                  (3 << (2 * 1)) |
                                  (3 << (2 * 0)));
    NT_COMMON_REG->GPIODENC |= tmp;

    // MII_TXCLK  - PB3 (af1)
    tmp = (1 << 3);
    NT_GPIOB->ALTFUNCSET |= tmp;
    NT_COMMON_REG->GPIOPCTLB &= ~(3 << (2 * 3));
    NT_COMMON_REG->GPIODENB |= tmp;

    //TODO: add PHY interrupt init
    //NT_GPIOC->INTTYPECLR |= (1 << 5); // interrupt on level
    //NT_GPIOC->INTPOLCLR |= (1 << 5);  // negative int level
    //NT_GPIOC->INTENSET |= (1 << 5);   // enable int
    //NVIC_EnableIRQ(GPIOC_IRQn);

    NT_COMMON_REG->PER_RST0_bit.ETHRST = 0;
    NT_COMMON_REG->PER_RST0_bit.ETHRST = 1;
}

void BSP_CAN_Init()
{
    uint32_t tmp;
    // IO init only
    // G7 - CAN0_TX, E2 - CAN0_RX
    // F14 - CAN1_TX, F15 - CAN1_RX
    tmp = (1<<7);
    NT_GPIOG->ALTFUNCSET |= tmp;
    NT_COMMON_REG->GPIOPCTLG &= ~(3 << (2 * 7));
    NT_COMMON_REG->GPIODENG |= tmp;
    tmp = (1<<2);
    NT_GPIOE->ALTFUNCSET |= tmp;
    NT_COMMON_REG->GPIOPCTLE &= ~(3 << (2 * 2));
    NT_COMMON_REG->GPIODENE |= tmp;
    tmp = (1<<15) | (1<<14);
    NT_GPIOF->ALTFUNCSET |= tmp;
    NT_COMMON_REG->GPIOPCTLF &= ~((3 << (2 * 15)) |
                                  (3 << (2 * 14)));
    NT_COMMON_REG->GPIODENF |= tmp;
}

void BSP_USB_Init()
{
    NT_COMMON_REG->USB_CTRL = (0 << COMMON_REG_USB_CTRL_FSEL_Pos) |  // 12 MHz
                              (1 << COMMON_REG_USB_CTRL_CLKEN_Pos) | // clk en
                              (1 << COMMON_REG_USB_CTRL_CLKSEL_Pos); // xtal
    NT_COMMON_REG->PER_RST0_bit.USBPHYRST = 1; // clear reset from usb phy
    for (int i=0;i<10000;i++) {
        __NOP();
    }
}

//-- IRQ handlers --------------------------------------------------------------
void BTN_IRQ_HANDLER()
{
    if (BTN_PORT->INTSTATUS & BTN_SB2_PIN_MSK) {
        Buttons.State_bits.SB2 = 1;
        BTN_PORT->INTSTATUS = BTN_SB2_PIN_MSK;
    }

    if (BTN_PORT->INTSTATUS & BTN_SB3_PIN_MSK) {
        Buttons.State_bits.SB3 = 1;
        BTN_PORT->INTSTATUS = BTN_SB3_PIN_MSK;
    }
}
