/*
 * Copyright 2012 Jared Boone
 * Copyright 2013 Benjamin Vernoux
 *
 * This file is part of HackRF.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <stddef.h>
#include <stdint.h>

#include "lsm6ds3.h"
#include "max2831.h"
#include "max5865.h"
#include "alarm_timer.h"
#include "tag_core.h"

#include <libopencm3/lpc43xx/rgu.h>
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/creg.h>

/* center frequency */
#define CENTER_FREQ 2462000000U

#define ACCEL_SENSITIVE  25
#define GYRO_SENSITIVE   10

void board_init(void)  
{
    pin_setup();

    enable_1v8_power();
    enable_rf_power();
    delay_ms(10);  /* allows for voltage stabilization */

    cpu_clock_init();

    ssp1_set_mode_lsm6ds3();
    lsm6ds3_setup(&lsm6ds3);

    ssp1_set_mode_max5865();
    max5865_setup(&max5865);
    max5865_standby(&max5865);

    ssp1_set_mode_max2831();
    max2831_setup(&max2831);
    max2831_set_frequency(&max2831, CENTER_FREQ);	    
    max2831_standby(&max2831);	    

    lpc4320_sleep_init();
}

bool checkMotion(void)
{
    int16_t temp;
    uint8_t i;
    int16_t buf[3];
    static int16_t bufXl[3], bufGyro[3];

    ssp1_set_mode_lsm6ds3();   

    memcpy(buf, bufXl, sizeof(buf));
    lsm6ds3_accel_read(&lsm6ds3, bufXl);
    for(i = 0; i < 3; i++)
    {
        temp = bufXl[i] - buf[i];
        if(temp > ACCEL_SENSITIVE || temp < -ACCEL_SENSITIVE)
        {
            return true;       
        }
    }
        
    lsm6ds3_gyro_read(&lsm6ds3, bufGyro);
    for(i = 0; i < 3; i++)
    {
        if(bufGyro[i] > GYRO_SENSITIVE || bufGyro[i] < -GYRO_SENSITIVE)
        {
            return true;   
        }
    }
    return false;    
}

int main()
{
    bool motionFlag = false;
    int16_t motionCnt = 0;

    board_init();  

    while(1)
    {
        if(true == checkMotion())
        {
            motionCnt = 60;
            if(false == motionFlag)
            {
                motionFlag = true;
                alarm_timer_set_freq(10);
            }   
        }  
        else
        {
            motionCnt--;
            if(motionCnt < 0)
            {
                if(true == motionFlag)
                {
                    motionFlag = false;
                    alarm_timer_set_freq(100);
                }
            }
        }
        
        if(true == motionFlag)
        {
            led_on(LED2);
            ssp1_set_mode_max2831();   
            max2831_standby(&max2831);  
            delay_ms(5);    /* wait for max2831 PLL setup */         
            max2831_tx(&max2831);

            ssp1_set_mode_max5865();  
            max5865_tx(&max5865);

            cpld_enable();
            delay_us(1000);           

            cpld_send_reset();
            cpld_wait_done();   
            led_off(LED2);

            ssp1_set_mode_max2831();   
            max2831_shutdown(&max2831); 

            ssp1_set_mode_max5865();  
            max5865_shutdown(&max5865);     

            cpld_disable();   
        }
        
        lpc4320_sleep();       
    }
}


