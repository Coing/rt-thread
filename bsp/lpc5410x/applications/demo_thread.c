#include <rtthread.h>
#include "drv_led.h"
#include "drv_uart.h"
#include "drv_i2c.h"
#include "mpu6050.h"

static void thread1_entry(void* parameter)					
{
    while(1)
    {
			Led_Control(0,1);
			rt_thread_delay(RT_TICK_PER_SECOND);
			Led_Control(0,0);
			rt_thread_delay(RT_TICK_PER_SECOND);
    }
}

static void thread2_entry(void* parameter)					
{
    while(1)
    {
			Led_Control(1,1);
			rt_thread_delay(RT_TICK_PER_SECOND);
			Led_Control(1,0);
			rt_thread_delay(RT_TICK_PER_SECOND);
    }        
}


int demo_init(void)
{
	rt_thread_t  thread1 = RT_NULL;
	rt_thread_t  thread2 = RT_NULL;
	
	static rt_device_t mpu6050 = RT_NULL;
	
	rt_led_hw_init();

	rt_i2c_core_init();
	rt_hw_i2c_init();
	rt_hw_mpu6050_init("i2c0", MPU6050_DEFAULT_ADDRESS);
	
	mpu6050 = rt_device_find("mpu6050");
	rt_device_open(mpu6050,RT_DEVICE_FLAG_RDWR);
	

	mpu6050_test();
	
//	ack = i2cRead(0x68, 0x75, 1, &sig);
//	mpu6050_test();
	
  thread1 = rt_thread_create("t1",thread1_entry, RT_NULL,512,10,5);                             
  if (thread1 != RT_NULL)                 
			rt_thread_startup(thread1);

	thread2 = rt_thread_create("t2",thread2_entry, RT_NULL,512,10,5); 	
	if (thread2 != RT_NULL)                 
			rt_thread_startup(thread2);

	
	return 0;
	
}

