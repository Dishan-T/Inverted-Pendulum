#include "balremote.h"

volatile unsigned long millisCount = 0;

//__attribute__((__constructor__)) void initSysTick(void);

void initSysTick(void)
{
    SysTickIntRegister(SysTick_Handler);
    NVIC_ST_CTRL_R = 0;
    NVIC_ST_RELOAD_R = 16000 - 1;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 7;

}

void SysTick_Handler(void)
{
    millisCount++;
    keypadRowSelect();
}

int millis(void)
{
    return millisCount;
}

void initUART0(int BAUD)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, BAUD, 16000000);
}

//void initPORTA(void)
//{
//    SYSCTL_RCGCGPIO_R |= 0b000001;  // Enable clock for Port A
//    while ((SYSCTL_PRGPIO_R & 0b000001) == 0)
//        //peripheral-ready-register
//        ;
//
//    GPIO_PORTA_DIR_R = 0b11110000;  // PA 6-4 as outputs
//    GPIO_PORTA_DEN_R = 0b11110000;   // Enable digital function for PC4, PC5
//
//}
//
//void initPORTB(void)
//{
//    SYSCTL_RCGCGPIO_R |= 0b000010;  // Enable clock for Port B
//    while ((SYSCTL_PRGPIO_R & 0b000010) == 0)
//        ;
//
//    GPIO_PORTB_DIR_R = 0b11111111;  // Set all PB pins as outputs
//    GPIO_PORTB_DEN_R = 0b11111111;  // Enable digital function for PB pins
//
//}

void initPORTC(void)
{
    SYSCTL_RCGCGPIO_R |= 0b000100;  // Enable clock for Port C
    while ((SYSCTL_PRGPIO_R & 0b000100) == 0)
        ;

    GPIO_PORTC_DIR_R = ~0b11110000;
    GPIO_PORTC_DEN_R = 0b11110000;
    GPIO_PORTC_PUR_R = 0b11110000;

}

void initPORTE(void)
{
    SYSCTL_RCGCGPIO_R |= 0b010000;  // Enable clock for Port E
    while ((SYSCTL_PRGPIO_R & 0b010000) == 0)
        ;  // Wait until Port E is ready

    GPIO_PORTE_DIR_R = 0b00001111;   // PE0 as output
    GPIO_PORTE_DEN_R = 0b00001111;   // Enable digital function for PE0

    GPIO_PORTE_ODR_R |= 0b1111;       // Enable open-drain mode for PE3-PE0

}

void initPORTF(void)
{

    SYSCTL_RCGCGPIO_R |= 0b100000;
    while ((SYSCTL_PRGPIO_R & 0b100000) == 0)
    {
    };

    // **Unlock PF0 (SW2)**
    GPIO_PORTF_LOCK_R = 0x4C4F434B;  // Unlock GPIO Port F
    GPIO_PORTF_CR_R |= (1 << 0);  // Allow changes to PF0

    GPIO_PORTF_DIR_R = 0b01110;
    GPIO_PORTF_DEN_R = 0b11111;
    GPIO_PORTF_PUR_R = 0b10001;

}

void initI2C(void)
{
    // Enable I2C0 and GPIOB peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Wait for peripherals to be ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0))
        ;
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
        ;

    // Configure PB2 and PB3 as I2C pins
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Initialize I2C Master
    I2CMasterInitExpClk(I2C_BASE, SysCtlClockGet(), true);  // 400kHz
}
