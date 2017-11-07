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

#include <libopencm3/lpc43xx/m4/nvic.h>
#include <libopencm3/lpc43xx/atimer.h>
#include <libopencm3/lpc43xx/rtc.h>
#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/rgu.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/eventrouter.h>
//#include <libopencmsis/core_cm3.h>

#include "hackrf-ui.h"
#include "max2837.h"
#include "max5864.h"
#include "rf_path.h"
#include "tag_core.h"

/* center frequency */
#define CENTER_FREQ 2462000000U

/* allows for voltage stabilization */
#define RF_POWER_DELAY 100000

/* milliseconds to sleep after transmitting */
#define STANDBY_TIME 10


#define EVRT_Src  4

//PMC寄存器
#define PMC_PD0_SLEEP0_HW_ENA   MMIO32(PMC_BASE+0x00)
#define PMC_PD0_SLEEP0_MODE   MMIO32(PMC_BASE+0x1c)

#define PMC_PWR_DEEP_SLEEP_MODE         0x3000AA
#define PMC_PWR_POWER_DOWN_MODE         0x30FCBA
#define PMC_PWR_DEEP_POWER_DOWN_MODE    0x30FF7F


void enterSleep(void);


void Chip_EVRT_Init(void)
{
	uint8_t i = 0;
	// Clear all register to be default
	EVENTROUTER_HILO      = 0x0000;
	EVENTROUTER_EDGE      = 0x0000;
	EVENTROUTER_CLR_EN    = 0xFFFF;
	do {
		i++;
		EVENTROUTER_CLR_STAT  = 0xFFFFF;
	} while ((EVENTROUTER_STATUS != 0) && (i < 10));
}


void ATIMER_ClearInts(void)
{
	ATIMER_CLR_STAT = 1;  //清状态
	EVENTROUTER_CLR_STAT = (1 << (uint8_t) EVRT_Src);  //清事件源
}

void eventrouter_isr(void)
{
	if (EVENTROUTER_STATUS & (1 << (uint8_t) EVRT_Src))
	{
		ATIMER_ClearInts();
	}

	if (EVENTROUTER_STATUS & (1 << (uint8_t) 0))
	{
		//led_on(LED3);
		EVENTROUTER_CLR_STAT = (1 << (uint8_t) 0);
		nvic_disable_irq(NVIC_EVENTROUTER_IRQ);
	}
}


void sleep_mode_init(void)
{
	//set 32K
	CREG_CREG0 &= ~((1 << 3) | (1 << 2));
	CREG_CREG0 |= (1 << 1) | (1 << 0);

	delay(10000);  //等待晶振稳定

	Chip_EVRT_Init();  //初始化
	EVENTROUTER_HILO |= (1 << (uint8_t) EVRT_Src);   //设置事件源
	EVENTROUTER_EDGE &= ~(1 << (uint8_t) EVRT_Src);

	EVENTROUTER_CLR_STAT = (1 << (uint8_t) EVRT_Src);  //清事件状态
	EVENTROUTER_SET_EN = (1 << (uint8_t) EVRT_Src);  //使能事件源

    nvic_enable_irq(NVIC_EVENTROUTER_IRQ);

	//RTC只有断电才会复位寄存器，并且在有32K时钟和不使能的情况下才能写寄存器
	ATIMER_CLR_EN = 1;
	while((ATIMER_ENABLE & 0x01) == 0x01);    //等待关闭
	ATIMER_CLR_STAT = 1;

	ATIMER_PRESET = 16;  //频率

	ATIMER_CLR_STAT = 1;  //清状态

	ATIMER_SET_EN |= 0x01;  //使能

	while((ATIMER_ENABLE & 0x01) != 0x01);  //等待使能
}

int main(void)
{
    pin_setup();

    enable_1v8_power();
    enable_rf_power();
    delay(RF_POWER_DELAY);  

    // initialize chips
    cpu_clock_init();
    hackrf_ui_init();
    rf_path_init(&rf_path);
    max2837_set_frequency(&max2837, CENTER_FREQ);	

	ssp1_set_mode_max5864();  
	max5864_shutdown(&max5864);

	ssp1_set_mode_max2837();   
	//max2837_set_mode(&max2837, MAX2837_MODE_SHUTDOWN);  //MAX2837_MODE_STANDBY   //MAX2837_MODE_SHUTDOWN
	max2837_stop(&max2837);

	rf_path_set_tx(&rf_path);  //tianxian 

	si5351c_disable_all_outputs(&clock_gen);
	si5351c_power_down_all_clocks(&clock_gen);  //10ma

	delay(10000);

    disable_rf_power();
    disable_1v8_power();

    delay(10000);


    led_off(LED1); 
	led_off(LED2);
	led_off(LED3);  

    gpio_sleep();

	enterSleep();
	SCB_SCR = 0x4;
	PMC_PD0_SLEEP0_MODE = (uint32_t) PMC_PWR_DEEP_POWER_DOWN_MODE;    //PMC_PWR_DEEP_SLEEP_MODE	   //PMC_PWR_POWER_DOWN_MODE    //PMC_PWR_DEEP_POWER_DOWN_MODE
	__asm__("wfi");	    

	while(1);
}

int mainx(void)
{
	uint16_t cnt = 0;
	uint8_t flag = 0;

    pin_setup();
    enable_1v8_power();

    enable_rf_power();
    delay(RF_POWER_DELAY);

    // initialize chips
    cpu_clock_init();
    hackrf_ui_init();
    rf_path_init(&rf_path);
    max2837_set_frequency(&max2837, CENTER_FREQ);

    /* just keep all chips powered on (for now)
     * TODO(fzliu): add standby/shutdown modes
     */
    ssp1_set_mode_max5864();
    max5864_tx(&max5864);
    ssp1_set_mode_max2837();
    max2837_tx(&max2837);
    rf_path_set_tx(&rf_path);

    /*
     * Tag main loop. Performs the following:
     *   1) Starts up all chips, CPLD com
     es up last.
     *   2) Sends reset signal to CPLD.
     *   3) Waits for done signal from CPLD.
     *   4) Powers off or sleeps all chips. CPLD is disabled first.
     */

	sleep_mode_init();

	flag = 2;
	led_on(LED1);  

    while (true)
	{
		//si5351c_enable_clock_outputs(&clock_gen);

		delay(1000);
		switch(flag)
		{
			case 0:
				ssp1_set_mode_max5864();
				max5864_tx(&max5864);			
				break;
			case 1:
				ssp1_set_mode_max2837();
				max2837_tx(&max2837);	
				break;
			case 2:
				ssp1_set_mode_max5864();
				max5864_tx(&max5864);		
					
				ssp1_set_mode_max2837();
				max2837_tx(&max2837);				
				break;
		}
		delay(5000);  //等待所有设备唤醒	

		if(key_press()) 
		{
			delay(50000);
			if(key_press())
			{
				flag = (++flag) % 3;

				switch(flag)
				{
					case 0:
						led_on(LED1); 
						led_off(LED2);
						led_off(LED3);
						break;
					case 1:
						led_on(LED2); 
						led_off(LED1);
						led_off(LED3);
						break;
					case 2:
						led_on(LED3); 
						led_off(LED1);
						led_off(LED2);
						break;
				}
			}
			//delay(500000);
			while(key_press());
			//delay(500000);
		}		
		/*
		//Si5351
		//si5351c_enable_clock_outputs(&clock_gen);

		//delay(1000)  //wait clk 

		ssp1_set_mode_max5864();
		max5864_tx(&max5864);

		ssp1_set_mode_max2837();
		max2837_tx(&max2837);

		//cpld		

		delay(1000);  //等待所有设备唤醒	
		*/

		#if 1

        //led_on(LED3);
        cpld_send_reset();
        cpld_wait_done();

		//delay(5000)
		//led_off(LED3);
		#endif

		switch(flag)
		{
			case 0:
				ssp1_set_mode_max5864();
				max5864_shutdown(&max5864);
				break;
			case 1:
				ssp1_set_mode_max2837();
				max2837_set_mode(&max2837, MAX2837_MODE_SHUTDOWN);
				break;			
			case 2:
				ssp1_set_mode_max5864();
				max5864_shutdown(&max5864);

				ssp1_set_mode_max2837();
				max2837_set_mode(&max2837, MAX2837_MODE_SHUTDOWN);  //MAX2837_MODE_STANDBY   //MAX2837_MODE_SHUTDOWN

				break;
		}

		//si5351c_disable_all_outputs(&clock_gen);
		//si5351c_power_down_all_clocks(&clock_gen);

		//enterSleep();
		SCB_SCR = 0x4;
		PMC_PD0_SLEEP0_MODE = (uint32_t) PMC_PWR_DEEP_SLEEP_MODE;    //PMC_PWR_DEEP_SLEEP_MODE	   //PMC_PWR_POWER_DOWN_MODE    //PMC_PWR_DEEP_POWER_DOWN_MODE
		__asm__("wfi");	

		//exitSleep();
		delay(1000);

		/*
		//max5864 sleep
		ssp1_set_mode_max5864();
		max5864_shutdown(&max5864);

		//max2837 sleep
		ssp1_set_mode_max2837();
		//寄存器
		max2837_set_mode(&max2837, MAX2837_MODE_SHUTDOWN);

		//Si5351 sleep
		//si5351c_disable_all_outputs(&clock_gen);

		//cpld sleep
		*/

		#if 0
		//lpc4320 sleep
		SCB_SCR = 0x4;
		PMC_PD0_SLEEP0_MODE = (uint32_t) PMC_PWR_DEEP_SLEEP_MODE;    //PMC_PWR_DEEP_SLEEP_MODE	   //PMC_PWR_POWER_DOWN_MODE    //PMC_PWR_DEEP_POWER_DOWN_MODE
		__asm__("wfi");
		#endif
    }

	while(1)
	{
		led_on(LED1);
		delay(500000);

		led_off(LED1);
		delay(500000);
	}

    return 0;
}


void enterSleep(void)
{
	//关闭时钟电源
	CCU1_PM = 1;
	CCU2_PM = 1;

	//关闭外设时钟
    CGU_BASE_USB0_CLK = CGU_BASE_USB0_CLK_AUTOBLOCK(1)
            | CGU_BASE_USB0_CLK_CLK_SEL(CGU_SRC_IRC);

    CGU_BASE_PERIPH_CLK = CGU_BASE_PERIPH_CLK_AUTOBLOCK(1)
            | CGU_BASE_PERIPH_CLK_CLK_SEL(CGU_SRC_IRC);

    CGU_BASE_APB1_CLK = CGU_BASE_APB1_CLK_AUTOBLOCK(1)
            | CGU_BASE_APB1_CLK_CLK_SEL(CGU_SRC_IRC);

    CGU_BASE_APB3_CLK = CGU_BASE_APB3_CLK_AUTOBLOCK(1)
            | CGU_BASE_APB3_CLK_CLK_SEL(CGU_SRC_IRC);

    CGU_BASE_SSP0_CLK = CGU_BASE_SSP0_CLK_AUTOBLOCK(1)
            | CGU_BASE_SSP0_CLK_CLK_SEL(CGU_SRC_IRC);

    CGU_BASE_SSP1_CLK = CGU_BASE_SSP1_CLK_AUTOBLOCK(1)
            | CGU_BASE_SSP1_CLK_CLK_SEL(CGU_SRC_IRC);

	CGU_BASE_M4_CLK =  CGU_BASE_M4_CLK_AUTOBLOCK(1)
			| CGU_BASE_M4_CLK_CLK_SEL(CGU_SRC_IRC);

	CGU_BASE_SPIFI_CLK =  CGU_BASE_SPIFI_CLK_AUTOBLOCK(1)
			| CGU_BASE_SPIFI_CLK_CLK_SEL(CGU_SRC_IRC);

	//关闭PLL
	CGU_PLL1_CTRL |= 0x01;
	CGU_PLL0USB_CTRL |= CGU_PLL0USB_CTRL_PD(1);

}

void exitSleep(void)
{
	CGU_PLL1_CTRL &= ~(0x01);
	CGU_PLL0USB_CTRL &= ~CGU_PLL0USB_CTRL_PD(1);

	CGU_BASE_M4_CLK =  CGU_BASE_M4_CLK_AUTOBLOCK(1)
			| CGU_BASE_M4_CLK_CLK_SEL(CGU_SRC_XTAL);

	//CREG_CREG0 &= ~((1 << 3) | (1 << 2));
	//CREG_CREG0 |= (1 << 1) | (1 << 0);

	//delay(10000);


    CGU_BASE_USB0_CLK = CGU_BASE_USB0_CLK_AUTOBLOCK(1)
            | CGU_BASE_USB0_CLK_CLK_SEL(CGU_SRC_PLL1);

    CGU_BASE_PERIPH_CLK = CGU_BASE_PERIPH_CLK_AUTOBLOCK(1)
            | CGU_BASE_PERIPH_CLK_CLK_SEL(CGU_SRC_PLL1);

    CGU_BASE_APB1_CLK = CGU_BASE_APB1_CLK_AUTOBLOCK(1)
            | CGU_BASE_APB1_CLK_CLK_SEL(CGU_SRC_PLL1);

    CGU_BASE_APB3_CLK = CGU_BASE_APB3_CLK_AUTOBLOCK(1)
            | CGU_BASE_APB3_CLK_CLK_SEL(CGU_SRC_PLL1);

    CGU_BASE_SSP0_CLK = CGU_BASE_SSP0_CLK_AUTOBLOCK(1)
            | CGU_BASE_SSP0_CLK_CLK_SEL(CGU_SRC_PLL1);

    CGU_BASE_SSP1_CLK = CGU_BASE_SSP1_CLK_AUTOBLOCK(1)
            | CGU_BASE_SSP1_CLK_CLK_SEL(CGU_SRC_PLL1);

	CGU_BASE_M4_CLK =  CGU_BASE_M4_CLK_AUTOBLOCK(1)
			| CGU_BASE_M4_CLK_CLK_SEL(CGU_SRC_PLL1);

	CGU_BASE_SPIFI_CLK =  CGU_BASE_SPIFI_CLK_AUTOBLOCK(1)
			| CGU_BASE_SPIFI_CLK_CLK_SEL(CGU_SRC_PLL1);

}


#if 0


				ssp1_set_mode_max5864();
				max5864_shutdown(&max5864);

				ssp1_set_mode_max2837();
				max2837_set_mode(&max2837, MAX2837_MODE_SHUTDOWN);

				si5351c_disable_all_outputs(&clock_gen);

				led_on(LED1); 
				led_on(LED2);
				led_on(LED3);
	while(1)
	{
		if(key_press()) 
		{
			delay(50000);
			if(key_press())
			{
				flag = (++flag) % 5;

				switch(flag)
				{
					case 0:
						led_on(LED1); 
						led_off(LED2);
						led_off(LED3);
						break;
					case 1:
						led_on(LED2); 
						led_off(LED1);
						led_off(LED3);
						break;
					case 2:
						led_on(LED3); 
						led_off(LED1);
						led_off(LED2);
						break;
					case 3:
						led_off(LED3); 
						led_off(LED1);
						led_off(LED2);
						break;		
					case 4:
						enterSleep();

						SCB_SCR = 0x4;
						PMC_PD0_SLEEP0_MODE = (uint32_t) PMC_PWR_DEEP_SLEEP_MODE;    //PMC_PWR_DEEP_SLEEP_MODE	   //PMC_PWR_POWER_DOWN_MODE    //PMC_PWR_DEEP_POWER_DOWN_MODE
						__asm__("wfi");							
						break;			
				}
			}
			while(key_press());
		}		
		delay(2000);
	}

#endif
