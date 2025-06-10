#ifndef BALREMOTE_H
#define BALREMOTE_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include ".\inc\tm4c123gh6pm.h"
#include <inc/hw_memmap.h>
#include <driverlib/gpio.h>
#include <driverlib/uart.h>
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#define UART0_PRIORITY   0  // 0-Highest
#define SYSTICK_PRIORITY 1
#define GPIOF_PRIORITY   2
#define TIMER0A_PRIORITY 2

#define I2C_BASE I2C0_BASE
#define MPU6050_ADDRESS 0x68

#define SW1        (GPIO_PORTF_DATA_R&(1<<4))
#define SW2        (GPIO_PORTF_DATA_R&(1<<0))

#define COL1        (GPIO_PORTC_DATA_R&(1<<4))
#define COL2        (GPIO_PORTC_DATA_R&(1<<5))
#define COL3        (GPIO_PORTC_DATA_R&(1<<6))
#define COL4        (GPIO_PORTC_DATA_R&(1<<7))

#define LED_RED 0x02
#define LED_BLUE  0x04
#define LED_GREEN 0x08
#define LED_CYAN 0x0C
#define LED_MAGENTA 0x06
#define LED_YELLOW 0x0A
#define LED_WHITE 0x0E

#define SEVEN_SEG_NUMS { \
    0b00111111, /* 0 */ \
    0b00000110, /* 1 */ \
    0b01011011, /* 2 */ \
    0b01001111, /* 3 */ \
    0b01100110, /* 4 */ \
    0b01101101, /* 5 */ \
    0b01111101, /* 6 */ \
    0b00000111, /* 7 */ \
    0b01111111, /* 8 */ \
    0b01101111  /* 9 */ \
}

int millis(void);
void initSysTick(void);
void initUART0(int);
void initUART2(int);

void initPORTA(void);
void initPORTB(void);
void initPORTC(void);
void initPORTD(void);
void initPORTE(void);
void initPORTF(void);
void segment7(void);

void initI2C(void);
void I2C_Write(uint8_t reg, uint8_t data);
void I2C_ReadMulti(uint8_t reg, uint8_t length);
void setupMPU6050(void);

void initSW(void);
void SW_Handler(void);

void SysTick_Handler(void);
void UART0_Handler_Interrupt(void);
void initSW1(void);

void addChar(char, char*);
void processString(char*);

void TIMER0A_Handler(void);
void initTimer0(void);

bool ledColor(char*);
bool timerCMD(char*);
bool extract_number_pwmduty(char*);
bool extract_number_timer(char*);

void keypadColRead(void);
void keypadRowSelect(void);

void cancan(void);

void initADC0(void);
uint32_t readADC0(void);

void initPWM_PF2(uint16_t);
void setdutyPWM_PF2(uint32_t);

void initPWM_PF3(uint16_t);
void setdutyPWM_PF3(uint32_t);

void initPWM_PD1(uint16_t);
void setdutyPWM_PD1(uint16_t);

void initPWM_PD0(uint16_t);
void setdutyPWM_PD0(uint16_t);

void initINTPRI(void);

#endif
