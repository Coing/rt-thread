/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

#include <rtthread.h>
#ifdef RT_USING_FINSH
#include <finsh.h>
#include <shell.h>
#endif


#include "components.h"

#include "drv_i2c.h"
#include "mpu6050.h"
#include "drv_led.h"

extern int demo_init(void);

void rt_init_thread_entry(void* parameter)
{
	static rt_device_t mpu6050 = RT_NULL;
	
	rt_led_hw_init();
	
	rt_i2c_core_init();
	rt_hw_i2c_init();
	rt_hw_mpu6050_init("i2c0", MPU6050_DEFAULT_ADDRESS);
	
	mpu6050 = rt_device_find("mpu6050");
	rt_device_open(mpu6050,RT_DEVICE_FLAG_RDWR);
	
//	mpu6050_test();
	/* initialization finsh shell Component */
    //finsh_system_init();
	#ifdef RT_USING_COMPONENTS_INIT
	/* initialization RT-Thread Components */
	rt_components_init();
	#endif
	
		demo_init();
	
}

int rt_application_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("init",
        rt_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/3, 20);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
		
		
    return 0;
}
