#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include "drv_i2c.h"


static struct rt_i2c_bus_device i2c_device;

static void gpio_set_sda(void *data, rt_int32_t state)
{
		/* I2C0_SDA:P0_24 */
    if (state)
    {
        //GPIO_SetBits(GPIO_PORT_I2C_SDA, PIN_I2C_SDA);
				LPC_GPIO->B[0][24] = 1UL;
		}
    else
    {
        //GPIO_ResetBits(GPIO_PORT_I2C_SDA, PIN_I2C_SDA);
				LPC_GPIO->B[0][24] = 0UL;
    }
}

static void gpio_set_scl(void *data, rt_int32_t state)
{
		/* I2C0_SCL:P0_23 */
    if (state)
    {
        //GPIO_SetBits(GPIO_PORT_I2C_SCL, PIN_I2C_SCL);
				LPC_GPIO->B[0][23] = 1UL;
    }
    else
    {
        //GPIO_ResetBits(GPIO_PORT_I2C_SCL, PIN_I2C_SCL);
				LPC_GPIO->B[0][23] = 0UL;
    }
}

static rt_int32_t gpio_get_sda(void *data)
{
    //return GPIO_ReadInputDataBit(GPIO_PORT_I2C_SDA, PIN_I2C_SDA);
	/* I2C0_SDA:P0_24 */
		return LPC_GPIO->B[0][24];
}

static rt_int32_t gpio_get_scl(void *data)
{
    //return GPIO_ReadInputDataBit(GPIO_PORT_I2C_SCL, PIN_I2C_SCL);
	/* I2C0_SCL:P0_23 */
		return LPC_GPIO->B[0][23];
}

static void gpio_udelay(rt_uint32_t us)
{
    volatile rt_int32_t i;
    for (; us > 0; us--)
    {
        //i = 50;
				i = 7;
        while(i--);
    }
}

static const struct rt_i2c_bit_ops bit_ops =
{
    RT_NULL,
    gpio_set_sda,
    gpio_set_scl,
    gpio_get_sda,
    gpio_get_scl,

    gpio_udelay,	

    4,					/* scl and sda line delay */
    0						/* in tick */
};


void rt_hw_i2c_init(void)
{
	
//	int temp = 1;
	
	/* 先用GPIO模拟来做吧 */
	/*I2C0_SCL:P0_23     I2C0_SDA:P0_24  */
	LPC_SYSCON->AHBCLKCTRLSET[0] = (1UL << 14);       /* enable GPIO0 clock*/
	
//	LPC_SYSCON->PRESETCTRLSET[0] = (1UL << 14);				/* Resets a GPIO0 peripheral */
//	LPC_SYSCON->PRESETCTRLCLR[0] = (1UL << 14);
	
	/* set P0.23, P0.24  output. */
	LPC_GPIO->DIR[0] |= 0x03UL << 23;
	
	Chip_GPIO_SetPinState(LPC_GPIO,0,24,0);
	
	rt_memset((void *)&i2c_device, 0, sizeof(struct rt_i2c_bus_device));
	i2c_device.priv = (void *)&bit_ops;
	rt_i2c_bit_add_bus(&i2c_device, "i2c0");

	//Chip_GPIO_SetPinState(LPC_GPIO, 0, ledBits[LEDNumber], (bool) !On);
	Chip_GPIO_SetPinState(LPC_GPIO,0,23,0);
}


/*
 *  参考了basefliy
 */

#define SCL_H         LPC_GPIO->B[0][23] = 1UL;
#define SCL_L         LPC_GPIO->B[0][23] = 0UL;

#define SDA_H         LPC_GPIO->B[0][24] = 1UL;
#define SDA_L         LPC_GPIO->B[0][24] = 0UL;

#define SCL_read      LPC_GPIO->B[0][23]
#define SDA_read      LPC_GPIO->B[0][24]


static void I2C_delay(void)
{
	  volatile int i = 7;
    while (i)
        i--;
}


static bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if (!SDA_read)
        return false;
    SDA_L;
    I2C_delay();
    if (SDA_read)
        return false;
    SDA_L;
    I2C_delay();
    return true;
}

static void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}

static void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

static void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

static bool I2C_WaitAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    if (SDA_read) 
		{
        SCL_L;
        return false;
    }
    SCL_L;
    return true;
}

static void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) {
        SCL_L;
        I2C_delay();
        if (byte & 0x80)
				{
            SDA_H;
				}
        else
				{
            SDA_L;
				}
        byte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}

static uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if (SDA_read) {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}

bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
    int i;
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | 0x00);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (i = 0; i < len; i++) {
        I2C_SendByte(data[i]);
        if (!I2C_WaitAck()) {
            I2C_Stop();
            return false;
        }
    }
    I2C_Stop();
    return true;
}

bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | 0x00);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(data);
    I2C_WaitAck();
    I2C_Stop();
    return true;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | 0x00);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr << 1 | 0x01);
    I2C_WaitAck();
    while (len) {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        buf++;
        len--;
    }
    I2C_Stop();
    return true;
}
