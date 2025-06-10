#include"balbot.h"

int32_t acc_calibration_value = 980; //Enter the accelerometer calibration value

float pid_p_gain = 12;                  //P-controller (15)
float pid_i_gain = 1.2;                //I-controller (1.5)
float pid_d_gain = 28;                  //D-controller (30)
float turning_speed = 30;                                   //Turning speed (20)
float max_target_speed = 100;                           //Max target speed (100)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool line_follower_flag = 0;

int16_t rawData[14];

uint8_t start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor,
        throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor,
        throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int32_t gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

int32_t gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output,
        pid_last_d_error;
float pid_output_left, pid_output_right;

int main(void)
{

    SysCtlClockSet(
    SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    initUART0(9600);
    initUART2(9600);

    initPORTA(); //MOTOR PULSE
    initPORTC(); //IR
    initPORTF(); //STATUS LED

    initI2C();
    setupMPU6050();
    initSysTick();
    initTimer0A();

    while (1)
    {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //BLUETOOTH
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (UARTCharsAvail(UART2_BASE))
        {

            char val = UARTCharGetNonBlocking(UART2_BASE);

            //UARTCharPut(UART0_BASE, val);

            if (val == 'B' || val == 'b')
            {
                line_follower_flag = 0;
                GPIO_PORTF_DATA_R = LED_GREEN;
                turning_speed = 30;
                max_target_speed = 100;
            }
            else if (val == 'L' || val == 'l')
            {
                line_follower_flag = 1;
                GPIO_PORTF_DATA_R = LED_BLUE;
                turning_speed = 60;
                max_target_speed = 50;
            }

            if (!line_follower_flag && start)
            {
                if (val == '6')
                    received_byte = 0b00000001;
                else if (val == '4')
                    received_byte = 0b00000010;
                else if (val == '8')
                    received_byte = 0b00000100;
                else if (val == '5')
                    received_byte = 0b00001000;

                receive_counter = 0;
            }

        }

        if (line_follower_flag && start)
        {
            if (!IRL && IRR)
            {
                if (pid_output > 10)
                {
                    received_byte = 0b00000010;
                    receive_counter = 0;
                }
                else if (pid_output < -10)
                {
                    received_byte = 0b00000001;
                    receive_counter = 0;
                }
            }
            else if (IRL && !IRR)
            {
                if (pid_output > 10)
                {
                    received_byte = 0b00000001;
                    receive_counter = 0;
                }
                else if (pid_output < -10)
                {
                    received_byte = 0b00000010;
                    receive_counter = 0;
                }
            }
            else
            {
                received_byte = 0b00000100;
                receive_counter = 0;

//                static int forward_count = 0;
//
//                if (forward_count <= 200)
//                    received_byte = 0b00000100;
//                else if (forward_count <= 250)
//                    received_byte = 0x00;
//                else
//                    forward_count = 0;
//
//                forward_count++;
            }

        }

        if (receive_counter <= 25)
            receive_counter++;
        else
            received_byte = 0x00;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Angle calculations
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        I2C_ReadMulti(0x3F, 2);
        accelerometer_data_raw = ((int16_t) (rawData[0] << 8) | rawData[1]);
        accelerometer_data_raw += acc_calibration_value;

        if (accelerometer_data_raw > 8200)
            accelerometer_data_raw = 8200;
        if (accelerometer_data_raw < -8200)
            accelerometer_data_raw = -8200;

        angle_acc = asin((float) accelerometer_data_raw / 8200.0) * 57.296;

        if (start == 0 && angle_acc > -0.5 && angle_acc < 0.5)
        {
            angle_gyro = angle_acc;
            start = 1;
        }

        I2C_ReadMulti(0x43, 4);
        gyro_yaw_data_raw = ((int16_t) (rawData[0] << 8) | rawData[1]);
        gyro_pitch_data_raw = ((int16_t) (rawData[2] << 8) | rawData[3]);

        gyro_pitch_data_raw -= gyro_pitch_calibration_value;
        angle_gyro += (int16_t) gyro_pitch_data_raw * 0.000031;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //MPU-6050 filter
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        gyro_yaw_data_raw -= gyro_yaw_calibration_value; //Add the gyro calibration value
        //Uncomment the following line to make the compensation active
        angle_gyro -= gyro_yaw_data_raw * 0.00000115; //Compensate the gyro offset when the robot is rotating

        angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;

//        int int_part = (int) angle_acc;
//        int decimal_part = (int) ((angle_acc - int_part) * 1000); // 3 decimals
//
//        UARTprintf("%d.%03d, ", int_part, decimal_part);
//
//        int_part = (int) angle_gyro;
//        decimal_part = (int) ((angle_gyro - int_part) * 1000); // 3 decimals
//
//        UARTprintf("%d.%03d\n", int_part, decimal_part);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //PID controller calculations
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
        if (pid_output > 10 || pid_output < -10)
            pid_error_temp += pid_output * 0.015;

        pid_i_mem += pid_i_gain * pid_error_temp;
        if (pid_i_mem > 400)
            pid_i_mem = 400;
        else if (pid_i_mem < -400)
            pid_i_mem = -400;

        //Calculate the PID output value
        pid_output = pid_p_gain * pid_error_temp + pid_i_mem
                + pid_d_gain * (pid_error_temp - pid_last_d_error);
        if (pid_output > 400)
            pid_output = 400;
        else if (pid_output < -400)
            pid_output = -400;

        pid_last_d_error = pid_error_temp;

        if (pid_output < 5 && pid_output > -5)
            pid_output = 0;

        if (angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1)
        {
            pid_output = 0;
            pid_i_mem = 0;
            start = 0;
            self_balance_pid_setpoint = 0;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Control calculations
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        pid_output_left = pid_output;
        pid_output_right = pid_output;

        if (received_byte & 0b00000001)
        {
            pid_output_left += turning_speed;
            pid_output_right -= turning_speed;
        }
        if (received_byte & 0b00000010)
        {
            pid_output_left -= turning_speed;
            pid_output_right += turning_speed;
        }

        if (received_byte & 0b00000100)
        {
            if (pid_setpoint > -0.5)
                pid_setpoint -= 0.05;
            if (pid_output > max_target_speed * -1)
                pid_setpoint -= 0.005;
        }
        if (received_byte & 0b00001000)
        {
            if (pid_setpoint < 0.5)
                pid_setpoint += 0.05;
            if (pid_output < max_target_speed)
                pid_setpoint += 0.005;
        }

        if (!(received_byte & 0b00001100))
        {
            if (pid_setpoint > 0.5)
                pid_setpoint -= 0.05;
            else if (pid_setpoint < -0.5)
                pid_setpoint += 0.05;
            else
                pid_setpoint = 0;
        }

        if (pid_setpoint == 0)
        {
            if (pid_output < 0)
                self_balance_pid_setpoint += 0.0015;
            if (pid_output > 0)
                self_balance_pid_setpoint -= 0.0015;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Motor pulse calculations
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if (pid_output_left > 0)
            pid_output_left = 405 - (1 / (pid_output_left + 9)) * 5500;
        else if (pid_output_left < 0)
            pid_output_left = -405 - (1 / (pid_output_left - 9)) * 5500;

        if (pid_output_right > 0)
            pid_output_right = 405 - (1 / (pid_output_right + 9)) * 5500;
        else if (pid_output_right < 0)
            pid_output_right = -405 - (1 / (pid_output_right - 9)) * 5500;

        if (pid_output_left > 0)
            left_motor = 400 - pid_output_left;
        else if (pid_output_left < 0)
            left_motor = -400 - pid_output_left;
        else
            left_motor = 0;

        if (pid_output_right > 0)
            right_motor = 400 - pid_output_right;
        else if (pid_output_right < 0)
            right_motor = -400 - pid_output_right;
        else
            right_motor = 0;

        throttle_left_motor = left_motor;
        throttle_right_motor = right_motor;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Loop time timer
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        while (loop_timer > millis())
            ;
        loop_timer += 4;

    }
}

//Every 20uS
void Timer0A_Handler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Clear the interrupt

    //left motor pulse calculations
    throttle_counter_left_motor++;
    if (throttle_counter_left_motor > throttle_left_motor_memory)
    {
        throttle_counter_left_motor = 0;
        throttle_left_motor_memory = throttle_left_motor;
        if (throttle_left_motor_memory < 0)
        {
            GPIO_PORTA_DATA_R &= 0b11110111;
            throttle_left_motor_memory *= -1;
        }
        else
            GPIO_PORTA_DATA_R |= 0b00001000;
    }
    else if (throttle_counter_left_motor == 1)
        GPIO_PORTA_DATA_R |= 0b00000100;
    else if (throttle_counter_left_motor == 2)
        GPIO_PORTA_DATA_R &= 0b11111011;

    //right motor pulse calculations
    throttle_counter_right_motor++;
    if (throttle_counter_right_motor > throttle_right_motor_memory)
    {
        throttle_counter_right_motor = 0;
        throttle_right_motor_memory = throttle_right_motor;
        if (throttle_right_motor_memory < 0)
        {
            GPIO_PORTA_DATA_R |= 0b00100000;
            throttle_right_motor_memory *= -1;
        }
        else
            GPIO_PORTA_DATA_R &= 0b11011111;
    }
    else if (throttle_counter_right_motor == 1)
        GPIO_PORTA_DATA_R |= 0b00010000;
    else if (throttle_counter_right_motor == 2)
        GPIO_PORTA_DATA_R &= 0b11101111;

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
    I2C_Write(0x1B, 0x00); // Set gyro to �250�/s
    I2C_Write(0x1C, 0x08); // Set accel to �2g
    I2C_Write(0x1A, 0X03);

    for (receive_counter = 0; receive_counter < 500; receive_counter++)
    {
        if (receive_counter % 15 == 0)
            GPIO_PORTF_DATA_R ^= LED_RED;

        I2C_ReadMulti(0x43, 4);
        gyro_yaw_calibration_value +=
                ((int16_t) (rawData[0] << 8) | rawData[1]);

        gyro_pitch_calibration_value += ((int16_t) (rawData[2] << 8)
                | rawData[3]);

        delayMs(4);
    }
    gyro_yaw_calibration_value /= 500;
    gyro_pitch_calibration_value /= 500;

//    UARTprintf("%d \t %d \n", gyro_yaw_calibration_value,
//               gyro_pitch_calibration_value);

    GPIO_PORTF_DATA_R = LED_GREEN;
    loop_timer = millis() + 4;
}

