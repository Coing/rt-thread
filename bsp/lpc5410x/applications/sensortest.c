#include <sensor.h>
#include <stdio.h>

#include <finsh.h>
#include <shell.h>

//#include "bmi055_sensor.h"

#include "MPU6050_sensor.h"

		
		MPU6050_Accelerometer acc = MPU6050_Accelerometer("i2c0", MPU6050_ADDRESS_AD0_LOW);
		MPU6050_Gyroscope gryo = MPU6050_Gyroscope("i2c0", MPU6050_ADDRESS_AD0_LOW);
//		BMI055_Accelerometer acc = BMI055_Accelerometer("i2c0", 0x19);
//		BMI055_Gyroscope gryo = BMI055_Gyroscope("i2c0", BMI055_GYRO_I2C_ADDR2);

int sensor_test(int argc, char** argv)
//int sensor_test()
{
		
    rt_sensor_t sensor;
    sensors_event_t event;
    char line[64];
	
    sensor = rt_sensor_get_default(SENSOR_TYPE_ACCELEROMETER);
    if (sensor != RT_NULL)
    {
        int index;
        SensorConfig config = {SENSOR_MODE_NORMAL, SENSOR_DATARATE_400HZ, SENSOR_ACCEL_RANGE_2G};

        rt_sensor_activate(sensor, 1);
        rt_sensor_configure(sensor, &config);

        index = 0;
        index = index;

//        while (1) // for (index = 0; index < 10; index ++)
				for (index = 0; index < 10; index ++)
        {
            rt_sensor_poll(sensor, &event);

            sprintf(line, "type: %d, x:%f, y:%f, z:%f",
                event.type,
                event.acceleration.x,
                event.acceleration.y,
                event.acceleration.z);
            rt_kprintf("%s\n", line);
						rt_thread_delay(100);
        }
    }
		
		sensor = rt_sensor_get_default(SENSOR_TYPE_GYROSCOPE);
    if (sensor != RT_NULL)
    {
        int index;
        SensorConfig config = {SENSOR_MODE_NORMAL, SENSOR_DATARATE_400HZ, SENSOR_GYRO_RANGE_250DPS};

        rt_sensor_activate(sensor, 1);
        rt_sensor_configure(sensor, &config);

        index = 0;
        index = index;

        while (1) // for (index = 0; index < 10; index ++)
        {
            rt_sensor_poll(sensor, &event);

            sprintf(line, "type: %d, x:%f, y:%f, z:%f",
                event.type,
                event.gyro.x,
                event.gyro.y,
                event.gyro.z);
            rt_kprintf("%s\n", line);
        }
    }
		
		
		
		
   
    return 0;
}

MSH_CMD_EXPORT(sensor_test, sensor test);
