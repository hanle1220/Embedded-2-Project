/*
 * mpu.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include "uart0.h"
#include "clock.h"
#include "tm4c123gh6pm.h"
#include "i2c0.h"
#include "wait.h"
#include "gpio.h"
#include "mpu.h"

void initIMU()
{
    writeI2c0Register(0x68,SMPLRT_DIV, 0x07);
    //writeI2c0Register(0x68,0x37,0xA2);
    writeI2c0Register(0x68,PWR_MGMT_1,0x01);
    waitMicrosecond(1000);
    writeI2c0Register(0x68,CONFIG, 0x03);
    writeI2c0Register(0x68,GYRO_CONFIG,0xFF);
    writeI2c0Register(0x68,ACCEL_CONFIG,0xFF);
    writeI2c0Register(0x0C,0x0A,0x06);
    writeI2c0Register(0x68,INT_ENABLE, 0x01);
    readI2c0Register(0x68, 0x3A);
}

void gyro(uint16_t* data)
{
    data[0] = (readI2c0Register(0x68, GYRO_XOUT_H) << 8 | readI2c0Register(0x68, GYRO_XOUT_L));
    data[1] = (readI2c0Register(0x68, GYRO_YOUT_H) << 8 | readI2c0Register(0x68, GYRO_YOUT_L));
    data[2] = (readI2c0Register(0x68, GYRO_ZOUT_H) << 8 | readI2c0Register(0x68, GYRO_ZOUT_L));
}

void accel(uint16_t* data)
{
    data[0] = (readI2c0Register(0x68, ACCEL_XOUT_H) << 8 | readI2c0Register(0x68, ACCEL_XOUT_L));
    data[1] = (readI2c0Register(0x68, ACCEL_YOUT_H) << 8 | readI2c0Register(0x68, ACCEL_YOUT_L));
    data[2] = (readI2c0Register(0x68, ACCEL_ZOUT_H) << 8 | readI2c0Register(0x68, ACCEL_ZOUT_L));
}

void tempIMU(uint16_t* data)
{
    data[0] = (readI2c0Register(0x68, TEMP_OUT_H) << 8 | readI2c0Register(0x68, TEMP_OUT_L));
}

void compass(uint16_t* data)
{
    data[0] = readI2c0Register(0x0C, 0x04) << 8 | readI2c0Register(0x0C, 0x03);
    data[1] = readI2c0Register(0x0C, 0x04) << 8 | readI2c0Register(0x0C, 0x03);
    data[2] = readI2c0Register(0x0C, 0x04) << 8 | readI2c0Register(0x0C, 0x03);
}

