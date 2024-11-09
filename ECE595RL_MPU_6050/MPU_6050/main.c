/**
 * @file MPU_6050_main.c
 *
 * @brief Main source code for the MPU_6050 program.
 *
 * This file contains the main entry point for the MPU_6050 program,
 * which is used to demonstrate the MPU_6050 driver.
 *
 * It interfaces with the MPU-6050 6-DoF Accelerometer and Gyroscope Sensor module, which uses the I2C communication protocol.
 *  - Product Link: https://www.adafruit.com/product/3886
 *  - Datasheet: http://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 *  - Register Map: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 *
 * @author Aaron Nanas
 *
 */

#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/MPU_6050.h"

//#define ACCELEROMETER_ONLY              1
//#define GYROSCOPE_ONLY                  1
#define ACCELEROMETER_AND_GYROSCOPE     1

int main(void)
{
    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();

    // Initialize the MPU-6050 Accelerometer and Gyroscope sensor
    MPU_6050_Init();

    // Enable the interrupts used by the modules
    EnableInterrupts();

    // Print the MPU-6050 device ID to verify that MPU-6050 registers can be read
    printf("MPU-6050 Device ID: 0x%02X\n", MPU_6050_Get_Device_ID());

    int16_t* raw_acceleration_buffer;
    float* acceleration_buffer;

    int16_t* raw_gyroscope_buffer;
    float* gyroscope_buffer;

    while(1)
    {
#if defined ACCELEROMETER_ONLY

        uint8_t accelerometer_range = MPU_6050_Get_Accelerometer_Range();
        int accelerometer_scale = MPU_6050_Get_Accelerometer_Scale(accelerometer_range);
//        printf("Accelerometer Scale: %d\n", accelerometer_scale);

//        raw_acceleration_buffer = MPU_6050_Get_Raw_XYZ_Acceleration();
//        printf("(Accelerometer) X: %d  Y: %d  Z: %d m/s^2\n", raw_acceleration_buffer[0], raw_acceleration_buffer[1], raw_acceleration_buffer[2]);

        acceleration_buffer = MPU_6050_Get_Adjusted_XYZ_Acceleration();
        printf("(Accelerometer) X: %f  Y: %f  Z: %f m/s^2\n", acceleration_buffer[0], acceleration_buffer[1], acceleration_buffer[2]);

#elif defined GYROSCOPE_ONLY
    #if defined ACCELEROMETER_ONLY || ACCELEROMETER_AND_GYROSCOPE
        #error "Only ACCELEROMETER_ONLY, GYROSCOPE_ONLY, or ACCELEROMETER_AND_GYROSCOPE can be active."
    #endif

        uint8_t gyroscope_range = MPU_6050_Get_Gyroscope_Range();
        float gyroscope_scale = MPU_6050_Get_Gyroscope_Scale(gyroscope_range);
//        printf("Gyroscope Scale: %f\n", gyroscope_scale);

//        raw_gyroscope_buffer = MPU_6050_Get_Raw_XYZ_Gyroscope();
//        printf("(Gyroscope) X: %d  Y: %d  Z: %d deg/s\n", raw_gyroscope_buffer[0], raw_gyroscope_buffer[1], raw_gyroscope_buffer[2]);

        gyroscope_buffer = MPU_6050_Get_Adjusted_XYZ_Gyroscope();
        printf("(Gyroscope) X: %f  Y: %f  Z: %f deg/s\n", gyroscope_buffer[0], gyroscope_buffer[1], gyroscope_buffer[2]);

#elif defined ACCELEROMETER_AND_GYROSCOPE
    #if defined ACCELEROMETER_ONLY || GYROSCOPE_ONLY
        #error "Only ACCELEROMETER_ONLY, GYROSCOPE_ONLY, or ACCELEROMETER_AND_GYROSCOPE can be active."
    #endif

        uint8_t accelerometer_range = MPU_6050_Get_Accelerometer_Range();
        int accelerometer_scale = MPU_6050_Get_Accelerometer_Scale(accelerometer_range);

        uint8_t gyroscope_range = MPU_6050_Get_Gyroscope_Range();
        float gyroscope_scale = MPU_6050_Get_Gyroscope_Scale(gyroscope_range);

        acceleration_buffer = MPU_6050_Get_Adjusted_XYZ_Acceleration();
        printf("(Accelerometer) X: %f  Y: %f  Z: %f m/s^2\n", acceleration_buffer[0], acceleration_buffer[1], acceleration_buffer[2]);

        gyroscope_buffer = MPU_6050_Get_Adjusted_XYZ_Gyroscope();
        printf("(Gyroscope) X: %f  Y: %f  Z: %f deg/s\n\n", gyroscope_buffer[0], gyroscope_buffer[1], gyroscope_buffer[2]);

#else
    #error "Define either one of the options: ACCELEROMETER_ONLY, GYROSCOPE_ONLY, or ACCELEROMETER_AND_GYROSCOPE."
#endif

        // OPTIONAL: Add a delay before reading the next set of values
        Clock_Delay1ms(100);
    }
}
