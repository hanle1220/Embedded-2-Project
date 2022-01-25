/*
 * rtc.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "clock.h"
#include "wait.h"
#include "gpio.h"
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "rtc.h"

void initRTC()
{
    HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN;
    while(!HIB_CTL_WRC&HIB_CTL_R);
    return;
}

void initTimer()
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAMR_R = TIMER_TAMR_TACDIR;               // timer counts up
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

uint16_t randomAddr()
{
    return (uint16_t)(0xFFFF & TIMER1_TAV_R);
}

uint32_t getTime()
{
    uint32_t time = HIB_RTCC_R;
    return time;
}

void printTime(uint32_t time)
{
    char str[50];
    sprintf(str,"\n%2d:%2d:%2d",(time/3600)%24, (time/60)%60, time%60);
    putsUart0(str);
}

void printDateTime(uint16_t m, uint16_t d, uint32_t time)
{
    char str[50];
    sprintf(str,"\n%2d/%2d/%2d:%2d:%2d",12,6,(time/3600)%24, (time/60)%60, time%60);
    putsUart0(str);
}

void getDate()
{
    uint32_t dayPerMonth[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint32_t d = getTime();
    dates.day += (d / 86400);
    while(dates.day > dayPerMonth[dates.month])
    {
        if((dates.year % 4 == 0) && (dates.month == 2) && (dates.day >= 29))
        {
            dates.month += 1;
            dates.day -= 29;
        }
        else
        {
            dates.month += 1;
            dates.day -= dayPerMonth[dates.month];
        }
        if(dates.month >= 12)
        {
            dates.year += 1;
            dates.month -= 12;
        }
    }
}

void setTime(uint32_t h, uint32_t m, uint32_t s)
{
    uint32_t convertToSec = h*3600+m*60+s;
    HIB_RTCLD_R = convertToSec;
    while (!(HIB_CTL_R & HIB_CTL_WRC));
    getDate();
}

void setDate(uint32_t m, uint32_t d, uint32_t y)
{
    dates.day = d;
    dates.month = m;
    dates.year = y;
}
void printDate()
{
    getDate();
    char str[50];
    sprintf(str,"\n%d/%d/%d",dates.month, dates.day, dates.year);
    putsUart0(str);
}



