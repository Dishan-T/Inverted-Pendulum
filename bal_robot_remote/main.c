#include"balremote.h"

int16_t rawData[14];
bool mode = 0;

int main(void)
{

    SysCtlClockSet(
    SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    initSysTick();

    initPORTC(); //COLUMN
    initPORTE(); //ROW

    initPORTF(); //STATUS LED

    initUART0(9600);

    initI2C();
    setupMPU6050();

    GPIO_PORTF_DATA_R = LED_GREEN;

    while (1)
    {
        volatile unsigned long currentMillis = millis();
        static volatile unsigned long previousMillis = 0;

        if (!SW1)
        {
            UARTCharPut(UART0_BASE, 'B');
            if ((currentMillis - previousMillis) >= 50)
            {

                if (!mode)
                {
                    mode = 1;
                    GPIO_PORTF_DATA_R = LED_MAGENTA;
                }
                else
                {
                    mode = 0;
                    GPIO_PORTF_DATA_R = LED_GREEN;
                }

            }
            previousMillis = currentMillis;
        }

        if (!SW2)
        {
            UARTCharPut(UART0_BASE, 'L');
            if ((currentMillis - previousMillis) >= 50)
            {
                GPIO_PORTF_DATA_R = LED_BLUE;
            }
            previousMillis = currentMillis;

        }

        int32_t acc_x, acc_y;

        I2C_ReadMulti(0x3B, 4);
        acc_x = ((int16_t) (rawData[0] << 8) | rawData[1]);
        acc_y = ((int16_t) (rawData[2] << 8) | rawData[3]);

        //UARTprintf("%d \t %d \n", acc_x, acc_y);

        if (!mode)
            keypadColRead();
        else
        {
            if (acc_x > 4000)
            {
                UARTCharPut(UART0_BASE, '8');
            }
            else if (acc_x < -3000)
            {
                UARTCharPut(UART0_BASE, '5');
            }
            else if (acc_y > 3000)
            {
                UARTCharPut(UART0_BASE, '6');
            }
            else if (acc_y < -3000)
            {
                UARTCharPut(UART0_BASE, '4');
            }
        }

    }

}

void keypadColRead(void)
{
    volatile unsigned long currentMillis = millis();
    static volatile unsigned long previousMillis = 0;

    if (!COL1)
    {

        if ((currentMillis - previousMillis) >= 50)
        {

            if ((GPIO_PORTE_DATA_R & 0b1111) == 0b1110)
            {
            }
            else if ((GPIO_PORTE_DATA_R & 0b1111) == 0b1101)
            {

            }
            else if ((GPIO_PORTE_DATA_R & 0b1111) == 0b1011)
            {
                UARTCharPut(UART0_BASE, '8');

            }
            else if ((GPIO_PORTE_DATA_R & 0b1111) == 0b0111)
            {
            }
            previousMillis = currentMillis;
        }

    }

    else if (!COL2)
    {

        if ((currentMillis - previousMillis) >= 50)
        {
            if ((GPIO_PORTE_DATA_R & 0b1111) == 0b1110)
            {

            }
            else if ((GPIO_PORTE_DATA_R & 0b1111) == 0b1101)
            {

            }
            else if ((GPIO_PORTE_DATA_R & 0b1111) == 0b1011)
            {
                UARTCharPut(UART0_BASE, '5');
            }
            else if ((GPIO_PORTE_DATA_R & 0b1111) == 0b0111)
            {
            }
            previousMillis = currentMillis;
        }

    }
    else if (!COL3)
    {

        if ((currentMillis - previousMillis) >= 50)
        {
            if ((GPIO_PORTE_DATA_R & 0b1111) == 0b1110)
            {
            }
            else if ((GPIO_PORTE_DATA_R & 0b1111) == 0b1101)
            {

            }
            else if ((GPIO_PORTE_DATA_R & 0b1111) == 0b1011)
            {
                UARTCharPut(UART0_BASE, '4');
            }
            else if ((GPIO_PORTE_DATA_R & 0b1111) == 0b0111)
            {
            }
            previousMillis = currentMillis;
        }

    }
    else if (!COL4)
    {

        if ((currentMillis - previousMillis) >= 50)
        {
            if ((GPIO_PORTE_DATA_R & 0b1111) == 0b1110)
            {
            }
            else if ((GPIO_PORTE_DATA_R & 0b1111) == 0b1101)
            {
            }
            else if ((GPIO_PORTE_DATA_R & 0b1111) == 0b1011)
            {
                UARTCharPut(UART0_BASE, '6');
            }
            else if ((GPIO_PORTE_DATA_R & 0b1111) == 0b0111)
            {
            }
            previousMillis = currentMillis;
        }

    }

}

void keypadRowSelect(void)
{
    volatile unsigned long currentMillis = millis();
    static volatile unsigned long previousMillis = 0;

    if ((currentMillis - previousMillis) >= 10)
    {
        static int i = 0;

        GPIO_PORTE_DATA_R = 0b1111 & ~(1 << i);

        i++;
        if (i > 3)
            i = 0;

        previousMillis = currentMillis;

    }

}

void I2C_Write(uint8_t reg, uint8_t data)
{
    I2CMasterSlaveAddrSet(I2C_BASE, MPU6050_ADDRESS, false);

    I2CMasterDataPut(I2C_BASE, reg);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C_BASE))
        ;

    I2CMasterDataPut(I2C_BASE, data);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C_BASE))
        ;
}

void I2C_ReadMulti(uint8_t reg, uint8_t length)
{
    I2CMasterSlaveAddrSet(I2C_BASE, MPU6050_ADDRESS, false);
    I2CMasterDataPut(I2C_BASE, reg);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C_BASE))
        ;

    I2CMasterSlaveAddrSet(I2C_BASE, MPU6050_ADDRESS, true);

    for (int i = 0; i < length; i++)
    {
        if (i == 0)
        {
            I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        }
        else if (i == length - 1)
        {
            I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        }
        else
        {
            I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        }

        while (I2CMasterBusy(I2C_BASE))
            ;
        rawData[i] = I2CMasterDataGet(I2C_BASE);
    }
}

void setupMPU6050(void)
{

    I2C_Write(0x6B, 0x00); // Wake up the MPU6050 (disable sleep mode)
    I2C_Write(0x1B, 0x00); // Set gyro to ±250°/s
    I2C_Write(0x1C, 0x08); // Set accel to ±2g
    I2C_Write(0x1A, 0X03);

}

