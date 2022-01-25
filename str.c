#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "uart0.h"
#include "str.h"
#include "gpio.h"
#include "rtc.h"
#include "tm4c123gh6pm.h"

void printInfo(uint32_t log)
{
    char str[50];
    putsUart0("\nMM/DD/HH:MM:SS:\t");
    if(log & 1) { sprintf(str,"%8s%8s%8s", "Cx", "Cy", "Cz"); putsUart0(str); }
    if(log & 2) { sprintf(str,"%8s%8s%8s", "Gx", "Gy", "Gz"); putsUart0(str); }
    if(log & 4) { sprintf(str,"%8s%8s%8s", "Ax", "Ay", "Az"); putsUart0(str); }
    if(log & 8) { sprintf(str,"%8s", "T"); putsUart0(str); }
}

void printData(uint8_t para, uint8_t xyz, uint16_t value)
{
    char str[50];
    switch(para)
    {
        case 1: //COMPASS
        {
            if(xyz == 0)    sprintf(str,"%8d", value);
            if(xyz == 1)    sprintf(str,"%8d", value);
            if(xyz == 2)    sprintf(str,"%8d", value);
        }
        break;
        case 2: //GYRO
        {
            if(xyz == 0)    sprintf(str,"%8d", value);
            if(xyz == 1)    sprintf(str,"%8d", value);
            if(xyz == 2)    sprintf(str,"%8d", value);
        }
        break;
        case 4: //ACCEL
        {
            if(xyz == 0)    sprintf(str,"%8d", value);
            if(xyz == 1)    sprintf(str,"%8d", value);
            if(xyz == 2)    sprintf(str,"%8d", value);
        }
        break;
        case 8: //TEMP
        {
            sprintf(str,"%8d", value);
        }
        break;
    }
    putsUart0(str);
}

void printBinary(uint32_t num)
{
    putcUart0('\n');
      uint32_t i = 0x80000000;
      while(i)
      {
          putcUart0((num & i) ? '1' : '0');
          i >>= 1;
      }
}

void printHex(uint32_t num)
{
    char str[50];
    sprintf(str,"0x%08X\t",num);
    putsUart0(str);
}

void printInt(uint32_t num)
{
    char str[50];
    sprintf(str,"%d\t",num);
    putsUart0(str);
}

void getsUart0(USER_DATA* data)
{
    char c;
    uint8_t count = 0;
    while(true)
    {
        if(count == MAX_CHARS)
        {
            data->buffer[count] ='\0';
            break;
        }
        c = getcUart0();
        if(c == 8 || c == 127)
        {
            count--;
        }
        else if(c == 13 || c == 10)
        {
            data->buffer[count] ='\0';
            break;
        }
        else if(c >= 32)
        {
            data->buffer[count] = c;
            count++;
        }
    }
    return;
}

void parseFields(USER_DATA* data)
{
    uint8_t pos = 0;
    data->fieldCount = 0;
    while(data->buffer[pos] != '\0')
    {
        if(data->fieldCount > MAX_FIELDS)
        {
            break;
        }
        if(isalpha(data->buffer[pos]) != 0)
        {
             data->fieldType[data->fieldCount] = 'a';
             data->fieldPosition[data->fieldCount] = pos;
             if(data->buffer[pos-1] == '\0')
                 data->fieldCount++;
        }
        else if(isdigit(data->buffer[pos]) != 0)
        {
             data->fieldType[data->fieldCount] = 'n';
             data->fieldPosition[data->fieldCount] = pos;
             if(data->buffer[pos-1] == '\0')
                 data->fieldCount++;
        }
        else if(data->buffer[pos] == 38)
        {
             data->fieldType[data->fieldCount] = 's';
             data->fieldPosition[data->fieldCount] = pos;
             if(data->buffer[pos-1] == '\0')
                 data->fieldCount++;
        }
        else
        {
            data->buffer[pos] = '\0';
        }
        pos++;
    }
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber <= data->fieldCount)
    {
        if(data->fieldType[fieldNumber] == 'a')
        {
            return &data->buffer[data->fieldPosition[fieldNumber]];
        }
        else
            return 0;
    }
    else
        return 0;
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    char str[50] = " ";
    if(data->fieldType[fieldNumber] == 'n')
    {
        strcpy(str, &data->buffer[data->fieldPosition[fieldNumber]]);

        int i = 0, num = 0;
        while(str[i] != '\0')
        {
            num = num*10+(str[i]-48);
            i++;
        }
        return num;
    }
    else
        return 0;
}

bool strcompare(const char strCommand[], char string[])
{
    uint8_t i = 0;
    if(string[i] == '\0') return false;
    while(strCommand[i] != '\0')
    {
        if(strCommand[i] != string[i])
            return false;
        i++;
    }
    return true;
}

void strcopy(char* str1, char* str2)
{
    uint8_t k;
    for (k = 0; str1[k] != '\0'; k++)
       str2[k] = str1[k];
    str2[k] = '\0';
}
