#include "balbot.h"

volatile unsigned long millisCount = 0;

void initSysTick(void)
{
    SysTickIntRegister(SysTick_Handler);
    NVIC_ST_CTRL_R = 0;
    NVIC_ST_RELOAD_R = 16000 - 1; //1ms
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 7;

}

void SysTick_Handler(void)
{
    millisCount++;
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

void initUART2(int BAUD)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

    GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    UARTClockSourceSet(UART2_BASE, UART_CLOCK_PIOSC);

    UARTStdioConfig(2, BAUD, 16000000);  // Note: 2 refers to UART2
}

void initPORTA(void)
{
    SYSCTL_RCGCGPIO_R |= 0b000001;  // Enable clock for Port A
    while ((SYSCTL_PRGPIO_R & 0b000001) == 0)
        //peripheral-ready-register
        ;

    GPIO_PORTA_DIR_R |= 0b11111100;
    GPIO_PORTA_DEN_R |= 0b11111100;
}

void initPORTC(void)
{
    SYSCTL_RCGCGPIO_R |= 0b000100;  // Enable clock for Port C
    while ((SYSCTL_PRGPIO_R & 0b000100) == 0)
        ;

    GPIO_PORTC_DIR_R = ~0b11110000;  // PC4, PC5 as inputs
    GPIO_PORTC_DEN_R = 0b11110000;   // Enable digital function for PC4, PC5
    GPIO_PORTC_PUR_R = 0b11110000;

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

void initTimer0A(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    uint32_t clock_freq = SysCtlClockGet();
    uint32_t load_value = (clock_freq / 50000) - 1; // 20us

    TimerLoadSet(TIMER0_BASE, TIMER_A, load_value);

    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0A_Handler);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER0A);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void delayMs(int ms)
{
    int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 3180; j++)
        {
        }  // 1 ms delay approx at 16 MHz
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

