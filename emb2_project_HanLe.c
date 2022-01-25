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
#include "hibernate.h"
#include "mpu.h"
#include "str.h"
#include "adc0.h"
#include "rtc.h"

#define HB(x) (x >> 8) & 0xFF
#define LB(x) (x) & 0xFF
#define HWREG(x)(*((volatile uint32_t *)(x)))

#define Vcc         PORTD,6
#define AIN3_MASK   PORTE,0
#define GREEN_LED   PORTF,3
#define LESS        1
#define GREATER     2
#define ON          1
#define PERIODIC    1
#define TRIGGER     2

uint32_t LOG = 0;
uint8_t SLE = 0;
uint16_t COUNT = 0;
uint16_t LEVELING = 0;

typedef struct _sample
{
    uint32_t time;
    uint16_t accelData[3];
    uint16_t gyroData[3];
    uint16_t tempData[1];
    uint16_t compassData[3];
} sample;

enum offset {LOG_ADD = 1, SAMPLE_ADD, SLE_ADD, COUNT_ADD, MONTH_ADD, DAY_ADD, TIME_ADD, GATING_ADD, HYST_ADD, MODE_ADD, PERIOD_ADD, LEVEL_ADD};
enum logVar {LOG_COMPASS = 1, LOG_GYRO = 2, LOG_ACCEL = 4, LOG_TEMP = 8};

void initHw()
{
    initSystemClockTo40Mhz();

    // Enable clocks
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);
    _delay_cycles(3);

    selectPinAnalogInput(AIN3_MASK);
    selectPinDigitalInput(PORTF, 4);
    enablePinPullup(PORTF, 4);
    selectPinPushPullOutput(Vcc);
    selectPinPushPullOutput(GREEN_LED);
}

void writeEeprom(uint16_t address, uint8_t value)
{
    uint8_t i2cvalue[] = {LB(address), value };
    writeI2c0Registers(0xA0 >> 1, HB(address), i2cvalue, 2);
}

uint8_t readEeprom(uint16_t address)
{
    uint8_t value = readI2c0Register16(0xA1 >> 1, address);
    return value;
}

void writeToRtc(uint32_t offset, uint32_t data)
{
        HWREG(0x400FC030 + (offset*4)) = data;
        waitUntilWriteComplete();
}

uint32_t readFromRtc(uint32_t offset)
{
    uint32_t data = HWREG(0x400FC030 + (offset*4));
    waitUntilWriteComplete();
    return data;
}

void defaultMode()
{
    writeToRtc(LOG_ADD, 0);
    writeToRtc(SLE_ADD, 0);
    writeToRtc(COUNT_ADD, 0);
    writeToRtc(LEVEL_ADD, 0);
}

void reset()
{
    putsUart0("Rebooting...\n");
    LOG = 0;
    SLE = 0;
    COUNT = 0;
    LEVELING = 0;
    clearHibernationInterrupt();
    NVIC_APINT_R =  NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
}

void temp()
{
    char str[50];
    uint16_t adcCode;
    uint32_t temp;
    setAdc0Ss3Mux(3);
    setAdc0Ss3Log2AverageCount(2);
    adcCode = readAdc0Ss3();
    temp =  147.5 - ((75 * 3.3 * (float)adcCode) / 4096);
    sprintf(str, "%4u degreeC\n", temp);
    putsUart0(str);
    sprintf(str, "ADC:  %4u\n", adcCode);
    putsUart0(str);
}

void setGating(char str[], char ltGT[], uint16_t value)
{
    uint32_t gating = 0;
    uint8_t lg, param;

    if(strcompare("compass", str))
        param = LOG_COMPASS;
    if(strcompare("gyro", str))
        param = LOG_GYRO;
    if(strcompare("accel", str))
        param = LOG_ACCEL;
    if(strcompare("temp", str))
        param = LOG_TEMP;
    if(strcompare("lt", ltGT))
        lg = LESS;
    if(strcompare("gt", ltGT))
        lg = GREATER;

    gating |= param<<24 | lg<<16 | value;
    writeToRtc(GATING_ADD, gating);
    printBinary(gating);
}

void writeSample()
{
    uint8_t j;
    uint8_t index = 0;
    uint32_t N = readFromRtc(SAMPLE_ADD);
    uint32_t T = readFromRtc(PERIOD_ADD);
    uint8_t mode = (uint8_t)readFromRtc(MODE_ADD);
    sample samples;

    uint16_t levelSeed = readFromRtc(LEVEL_ADD);
    LOG = readFromRtc(LOG_ADD);
    printBinary(LOG);
    putsUart0("\nWriting Data...\n");

    switch(mode)
    {
        case PERIODIC:
        {
            while(COUNT < N)
            {
                samples.time = getTime();
                writeEeprom(TIME_ADD | levelSeed + (index++), (samples.time >>24 & 0xFF));
                writeEeprom(TIME_ADD | levelSeed + (index++), (samples.time >>16 & 0xFF));
                writeEeprom(TIME_ADD | levelSeed + (index++), (samples.time >>8 & 0xFF));
                writeEeprom(TIME_ADD | levelSeed + (index++), (samples.time >>0 & 0xFF));
                if(LOG & LOG_COMPASS)
                {
                    compass(samples.compassData);
                    for(j=0; j<3; j++)
                    {
                        writeEeprom(TIME_ADD | levelSeed + (index++), HB(samples.compassData[j]));
                        writeEeprom(TIME_ADD | levelSeed + (index++), LB(samples.compassData[j]));
                    }
                }
                if(LOG & LOG_GYRO)
                {
                    gyro(samples.gyroData);
                    for(j=0; j<3; j++)
                    {
                        writeEeprom(TIME_ADD | levelSeed + (index++), HB(samples.gyroData[j]));
                        writeEeprom(TIME_ADD | levelSeed + (index++), LB(samples.gyroData[j]));
                    }
                }
                if(LOG & LOG_ACCEL)
                {
                    accel(samples.accelData);
                    for(j=0; j<3; j++)
                    {
                        writeEeprom(TIME_ADD | levelSeed + (index++), HB(samples.accelData[j]));
                        writeEeprom(TIME_ADD | levelSeed + (index++), LB(samples.accelData[j]));
                    }
                }
                if(LOG & LOG_TEMP)
                {
                    tempIMU(samples.tempData);
                    writeEeprom(TIME_ADD | levelSeed + (index++), HB(samples.tempData[0]));
                    writeEeprom(TIME_ADD | levelSeed + (index++), LB(samples.tempData[0]));
                }
                COUNT++;
                waitMicrosecond(T*1000);
            }
            writeEeprom(COUNT_ADD, COUNT);
            writeToRtc(COUNT_ADD, COUNT);
        }
        break;
        case TRIGGER:
        {
            uint32_t gating = readFromRtc(GATING_ADD);
            uint16_t hyst = readFromRtc(HYST_ADD);

            //printHex(gating);
            //printHex(hyst);
            while(COUNT < N)
            {
                samples.time = getTime();
                writeEeprom(TIME_ADD | levelSeed + (index++), (samples.time >>24 & 0xFF));
                writeEeprom(TIME_ADD | levelSeed + (index++), (samples.time >>16 & 0xFF));
                writeEeprom(TIME_ADD | levelSeed + (index++), (samples.time >>8 & 0xFF));
                writeEeprom(TIME_ADD | levelSeed + (index++), (samples.time >>0 & 0xFF));
                if(LOG & LOG_COMPASS)
                {
                    uint16_t data[3];
                    compass(samples.compassData);
                    if((((gating&0xFF000000)>>24) & LOG_COMPASS) && (((gating&0x00FF0000)>>16) & LESS))
                    {
                        while(samples.compassData[0] > ((gating&0x0000FFFF)>>0))
                            compass(samples.compassData);
                        data[0] = samples.compassData[0];
                        while(samples.compassData[1] > ((gating&0x0000FFFF)>>0))
                            compass(samples.compassData);
                        data[1] = samples.compassData[1];
                        while(samples.compassData[2] > ((gating&0x0000FFFF)>>0))
                            compass(samples.compassData);
                        data[2] = samples.compassData[2];
                    }
                    if((((gating&0xFF000000)>>24) & LOG_COMPASS) && (((gating&0x00FF0000)>>16) & GREATER))
                    {
                        while(samples.compassData[0] < ((gating&0x0000FFFF)>>0))
                            compass(samples.compassData);
                        data[0] = samples.compassData[0];
                        while(samples.compassData[1] < ((gating&0x0000FFFF)>>0))
                            compass(samples.compassData);
                        data[1] = samples.compassData[1];
                        while(samples.compassData[2] < ((gating&0x0000FFFF)>>0))
                            compass(samples.compassData);
                        data[2] = samples.compassData[2];
                    }
                    for(j=0; j<3; j++)
                    {
                        writeEeprom(TIME_ADD | levelSeed + (index++), HB(data[j]));
                        writeEeprom(TIME_ADD | levelSeed + (index++), LB(data[j]));
                    }
                }
                if(LOG & LOG_GYRO)
                {
                    uint16_t data[3];
                    gyro(samples.gyroData);
                    if((((gating&0xFF000000)>>24) & LOG_GYRO) && (((gating&0x00FF0000)>>16) & LESS))
                    {
                        while(samples.gyroData[0] > ((gating&0x0000FFFF)>>0))
                            gyro(samples.gyroData);
                        data[0] = samples.gyroData[0];
                        while(samples.gyroData[1] > ((gating&0x0000FFFF)>>0))
                            gyro(samples.gyroData);
                        data[1] = samples.gyroData[1];
                        while(samples.gyroData[2] > ((gating&0x0000FFFF)>>0))
                            gyro(samples.gyroData);
                        data[2] = samples.gyroData[2];
                    }
                    if((((gating&0xFF000000)>>24) & LOG_GYRO) && (((gating&0x00FF0000)>>16) & GREATER))
                    {
                        while(samples.gyroData[0] < ((gating&0x0000FFFF)>>0))
                            gyro(samples.gyroData);
                        data[0] = samples.gyroData[0];
                        while(samples.gyroData[1] < ((gating&0x0000FFFF)>>0))
                            gyro(samples.gyroData);
                        data[1] = samples.gyroData[1];
                        while(samples.gyroData[2] < ((gating&0x0000FFFF)>>0))
                            gyro(samples.gyroData);
                        data[2] = samples.gyroData[2];
                    }
                    for(j=0; j<3; j++)
                    {
                        writeEeprom(TIME_ADD | levelSeed + (index++), HB(data[j]));
                        writeEeprom(TIME_ADD | levelSeed + (index++), LB(data[j]));
                    }
                }
                if(LOG & LOG_ACCEL)
                {
                    uint16_t data[3];
                    accel(samples.accelData);
                    if((((gating&0xFF000000)>>24) & LOG_ACCEL) && (((gating&0x00FF0000)>>16) & LESS))
                    {
                        while(samples.accelData[0] > ((gating&0x0000FFFF)>>0))
                            accel(samples.accelData);
                        data[0] = samples.accelData[0];
                        while(samples.accelData[1] > ((gating&0x0000FFFF)>>0))
                            accel(samples.accelData);
                        data[1] = samples.accelData[1];
                        while(samples.accelData[2] > ((gating&0x0000FFFF)>>0))
                            accel(samples.accelData);
                        data[2] = samples.accelData[2];
                    }
                    if((((gating&0xFF000000)>>24) & LOG_ACCEL) && (((gating&0x00FF0000)>>16) & GREATER))
                    {
                        while(samples.accelData[0] < ((gating&0x0000FFFF)>>0))
                            accel(samples.accelData);
                        data[0] = samples.accelData[0];
                        while(samples.accelData[1] < ((gating&0x0000FFFF)>>0))
                            accel(samples.accelData);
                        data[1] = samples.accelData[1];
                        while(samples.accelData[2] < ((gating&0x0000FFFF)>>0))
                            accel(samples.accelData);
                        data[2] = samples.accelData[2];
                    }
                    for(j=0; j<3; j++)
                    {
                        writeEeprom(TIME_ADD | levelSeed + (index++), HB(data[j]));
                        writeEeprom(TIME_ADD | levelSeed + (index++), LB(data[j]));
                    }
                }
                if(LOG & LOG_TEMP)
                {
                    tempIMU(samples.tempData);
                    if((((gating&0xFF000000)>>24) & LOG_TEMP) && (((gating&0x00FF0000)>>16) & LESS))
                        while(samples.tempData[0] > ((gating&0x0000FFFF)>>0))
                            tempIMU(samples.tempData);
                    if((((gating&0xFF000000)>>24) & LOG_TEMP) && (((gating&0x00FF0000)>>16) & GREATER))
                        while(samples.tempData[0] < ((gating&0x0000FFFF)>>0))
                            tempIMU(samples.tempData);
                    writeEeprom(TIME_ADD | levelSeed + (index++), HB(samples.tempData[0]));
                    writeEeprom(TIME_ADD | levelSeed + (index++), LB(samples.tempData[0]));
                }
                COUNT++;
            }
            writeEeprom(COUNT_ADD, COUNT);
            writeToRtc(COUNT_ADD, COUNT);
        }
        break;
    }
}

void readData()
{
    uint32_t count = readFromRtc(COUNT_ADD);
    uint8_t i, j;
    uint8_t index = 0;
    sample samples;
    uint16_t levelSeed = readFromRtc(LEVEL_ADD);
    uint8_t m = (readEeprom(MONTH_ADD));
    uint8_t d = (readEeprom(DAY_ADD));

    LOG = readFromRtc(LOG_ADD);
    printBinary(LOG);
    putsUart0("\nReading data...\n");
    printInfo(LOG);

    for(i=0; i<count; i++)
    {
        putcUart0('\n');
        samples.time |= readEeprom(TIME_ADD | levelSeed + (index++)) & 0xFF << 24;
        samples.time |= readEeprom(TIME_ADD | levelSeed + (index++)) & 0xFF << 16;
        samples.time |= readEeprom(TIME_ADD | levelSeed + (index++)) & 0xFF << 8;
        samples.time |= readEeprom(TIME_ADD | levelSeed + (index++)) & 0xFF << 0;
        printDateTime(12, 6, samples.time);    putsUart0(":\t");
        if(LOG & LOG_COMPASS)
            for(j=0; j<3; j++)
            {
                samples.compassData[j] |= readEeprom(TIME_ADD | levelSeed + (index++)) << 8;
                samples.compassData[j] |= readEeprom(TIME_ADD | levelSeed + (index++));
                printData(LOG_COMPASS, j, samples.compassData[j]);
            }
        if(LOG & LOG_GYRO)
            for(j=0; j<3; j++)
            {
                samples.gyroData[j] |= readEeprom(TIME_ADD | levelSeed + (index++)) << 8;
                samples.gyroData[j] |= readEeprom(TIME_ADD | levelSeed + (index++));
                printData(LOG_GYRO, j, samples.gyroData[j]);
            }
        if(LOG & LOG_ACCEL)
            for(j=0; j<3; j++)
            {
                samples.accelData[j] |= readEeprom(TIME_ADD | levelSeed + (index++)) << 8;
                samples.accelData[j] |= readEeprom(TIME_ADD | levelSeed + (index++));
                printData(LOG_ACCEL, j, samples.accelData[j]);
            }
        if(LOG & LOG_TEMP)
        {
            samples.tempData[0] |= readEeprom(TIME_ADD | levelSeed + (index++)) << 8;
            samples.tempData[0] |= readEeprom(TIME_ADD | levelSeed + (index++));
            samples.tempData[0] = samples.tempData[0]/333.87+21.0;
            printData(LOG_TEMP, j, samples.tempData[0]);
        }
    }
}

void getData()
{
    writeSample();
    waitMicrosecond(10000);
    readData();
}

void shell(void)
{
    USER_DATA data;
    while(true)
    {
        putsUart0("\n\n>> ");
        getsUart0(&data);
        parseFields(&data);

        if(strcompare("reset", getFieldString(&data, 0)) == 1)
        {
            reset();
        }
        else if(strcompare("temp", getFieldString(&data, 0)) == 1)
        {
            putsUart0("Internal Temperature ");
            temp();
        }
        else if(strcompare("time", getFieldString(&data, 0)) == 1)
        {
            if (getFieldInteger(&data, 1) == '\0')
                printTime(getTime());
            else
            {
                uint32_t h = getFieldInteger(&data, 1);
                uint32_t m = getFieldInteger(&data, 2);
                uint32_t s = getFieldInteger(&data, 3);
                char string[100];
                sprintf(string,"Setting time %d:%d:%d\n", h, m, s);
                putsUart0(string);
                setTime(h,m,s);
            }
        }
        else if(strcompare("date", getFieldString(&data, 0)) == 1)
        {
            if (getFieldInteger(&data, 1) == '\0')
            {
                printDate();
            }
            else
            {
                uint8_t m = getFieldInteger(&data, 1);
                uint8_t d = getFieldInteger(&data, 2);
                uint16_t y = getFieldInteger(&data, 3);
                uint16_t levelSeed = readFromRtc(LEVEL_ADD);
                char string[100];
                sprintf(string,"Setting date %d/%d/%d\n", m, d, y);
                putsUart0(string);
                setDate(m,d,y);
                writeEeprom(MONTH_ADD, m);
                writeEeprom(DAY_ADD, d);
            }
        }
        else if(strcompare("log", getFieldString(&data, 0)) == 1)
        {
            if(strcompare("compass", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Logging Compass\n");
                LOG += LOG_COMPASS;
            }
            else if(strcompare("gyro", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Logging Gyroscope\n");
                LOG += LOG_GYRO;
            }
            else if(strcompare("accel", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Logging Acceleration\n");
                LOG += LOG_ACCEL;
            }
            else if(strcompare("temp", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Logging Temperature\n");
                LOG += LOG_TEMP;
            }
            else
                putsUart0("Invalid!\n");
            printHex(LOG);
        }
        else if(strcompare("samples", getFieldString(&data, 0)) == 1)
        {
            uint8_t maxN = getFieldInteger(&data, 1);
            char string[100];
            sprintf(string,"%d Sample Sets", maxN);
            putsUart0(string);
            writeToRtc(SAMPLE_ADD, maxN);
            writeEeprom(SAMPLE_ADD, maxN);
        }
        else if(strcompare("gating", getFieldString(&data, 0)) == 1)
        {
            char* param = getFieldString(&data, 1);
            char* lessOrGreater =  getFieldString(&data, 2);
            uint8_t value = getFieldInteger(&data, 3);
            char string[100];
            sprintf(string,"%s(8) %s(8) %d(16)\n", param, lessOrGreater, value);
            putsUart0(string);
            setGating(param, lessOrGreater, value);
        }
        else if(strcompare("hysteresis", getFieldString(&data, 0)) == 1)
        {
            char* param = getFieldString(&data, 1);
            uint16_t hyst = getFieldInteger(&data, 2);
            char string[100];
            sprintf(string,"Hysteresis for %s(16) is set to %d(16)\n", param, hyst);
            putsUart0(string);
            if(strcompare("compass", param))
                writeToRtc(HYST_ADD, LOG_COMPASS<<16 | hyst);
            else if(strcompare("gyro", param))
                writeToRtc(HYST_ADD, LOG_GYRO<<16 | hyst);
            else if(strcompare("accel", param))
                writeToRtc(HYST_ADD, LOG_ACCEL<<16 | hyst);
            else if(strcompare("temp", param))
                writeToRtc(HYST_ADD, LOG_TEMP<<16 | hyst);
            else
                putsUart0("Invalid!\n");
            printBinary(readFromRtc(HYST_ADD));
        }
        else if(strcompare("sleep", getFieldString(&data, 0)) == 1)
        {
            if(strcompare("on", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Turns Sleep ON\n");
                SLE |= ON << 2;
            }
            else if(strcompare("off", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Turns Sleep OFF\n");
                SLE &= ~(ON << 2);
            }
            else
                putsUart0("Invalid!\n");
            writeToRtc(SLE_ADD, SLE);
            printBinary(readFromRtc(SLE_ADD));
        }
        else if(strcompare("leveling", getFieldString(&data, 0)) == 1)
        {
            if(strcompare("on", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Turns Leveling ON\n");
                SLE |= ON << 1;
                LEVELING = randomAddr();
            }
            else if(strcompare("off", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Turns Leveling OFF\n");
                SLE &= ~(ON << 1);
                LEVELING = 0;
            }
            else
                putsUart0("Invalid!\n");
            writeToRtc(SLE_ADD, SLE);
            writeToRtc(LEVEL_ADD, LEVELING);
            writeEeprom(SLE_ADD, SLE);
            printBinary(readFromRtc(SLE_ADD));
        }
        else if(strcompare("encrypt", getFieldString(&data, 0)) == 1)
        {
            if(strcompare("key", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Turns Encrypt ON\n");
                SLE |= ON << 0;
            }
            else if(strcompare("off", getFieldString(&data, 1)) == 1)
            {
                putsUart0("Turns Encrypt OFF\n");
                SLE &= ~(ON << 0);
            }
            else
                putsUart0("Invalid!\n");
            writeToRtc(SLE_ADD, SLE);
            writeEeprom(SLE_ADD, SLE);
            printBinary(readFromRtc(SLE_ADD));
        }
        else if(strcompare("periodic", getFieldString(&data, 0)) == 1)
        {
            putsUart0("Periodic Mode\n");
            uint32_t T = getFieldInteger(&data, 1); // milliseconds

            writeToRtc(MODE_ADD, PERIODIC);         //record mode as periodic
            writeToRtc(PERIOD_ADD, T);
            writeEeprom(LOG_ADD, LOG);              //write header to eeprom
            writeToRtc(LOG_ADD, LOG);

            printBinary(readFromRtc(MODE_ADD));     putcUart0('\n');
            printBinary(readFromRtc(LOG_ADD));

            uint8_t sleep = readFromRtc(SLE_ADD);
            if(sleep & (ON << 2))
            {
                setPinValue(Vcc,0);                 //turn off power
                hibernate(HIB_RTCC_R + T/1000);     //load into MATCH
            }
        }
        else if(strcompare("trigger", getFieldString(&data, 0)) == 1)
        {
            putsUart0("Trigger Mode\n");
            writeToRtc(MODE_ADD, TRIGGER);          //record mode as triggered
            writeEeprom(LOG_ADD, LOG);              //write header to eeprom
            writeToRtc(LOG_ADD, LOG);
            if(SLE & 0x4)
            {
                setPinValue(Vcc,0);                 //turn off power except [trigger source]
                hibernate(5);
            }
        }
        else if(strcompare("stop", getFieldString(&data, 0)) == 1)
        {
            putsUart0("Stop Sampling\n");
            //record current count in eeprom header
            writeEeprom(COUNT_ADD, COUNT);
        }
        else if(strcompare("data", getFieldString(&data, 0)) == 1)
        {
            getData();
        }
        else
            putsUart0("Invalid!\n");
    }
}

int main(void)
{
    initHw();
    initTimer();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    initI2c0();
    initRTC();
    initAdc0Ss3();
    initIMU();
    if(!checkIfConfigured())
        initHibernationModule();
    setPinValue(Vcc,1);

    writeToRtc(0, 0x10);
    bool sleep = readFromRtc(SLE_ADD) & 0x4;

    if(!wakePinCausedWakeUp() && !rtcCausedWakeUp())
    {
        setTime(1,1,1);
        printInt(getTime());
        printTime(getTime());
        defaultMode();
    }
    if(wakePinCausedWakeUp() || rtcCausedWakeUp())
    {
         putsUart0("\nWake up\n");
         printInt(getTime());
         printTime(getTime());
         putsUart0("\n\n");
         getData();
    }
    if(sleep == ON)
    {
        uint8_t mode = (uint8_t)readFromRtc(MODE_ADD);
        uint16_t hyst = (uint16_t)readFromRtc(HYST_ADD);
        if(mode == PERIODIC || mode == TRIGGER && hyst == 0)
        {
            writeToRtc(SLE_ADD, 0);
            hibernate(5);
        }
        if(mode == TRIGGER && hyst != 0)
        {
            //set var to detect 2nd condition
            writeToRtc(SLE_ADD, 0);
            hibernate(5);
        }
    }

    shell();
}
