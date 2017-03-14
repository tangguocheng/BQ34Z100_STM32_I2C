#include <stdio.h>
#include "stm32f10x.h"
#include "battery_manage.h"
#include "delay.h"
static battery_t battery_1;
static battery_t battery_2;
static GPIO_TypeDef* BQ34Z100_GPIO_SCL;
static uint16_t BQ34Z100_SCL_pin;
static GPIO_TypeDef* BQ34Z100_GPIO_SDA;
static uint16_t BQ34Z100_SDA_pin;

/**
 * @brief BQ34Z100 字节转u16
 * @param u8* data 字节指针
 * @return NONE
 */
static inline u16 bytes_to_integer(u8* data)
{
	u16 tmp;

	tmp = ((data[1] << 8) & 0xFF00);
	return ((u16)(tmp + data[0]) & 0x0000FFFF);
}

/**
 * @brief BQ34Z100 IO配置
 * @param IO参数
 * @return NONE
 */
void bq34z100_io_config(GPIO_TypeDef* GPIO_SCL, uint16_t SCL_pin,
                        GPIO_TypeDef* GPIO_SDA, uint16_t SDA_pin)
{
	BQ34Z100_GPIO_SCL = GPIO_SCL;
	BQ34Z100_SCL_pin = SCL_pin;
	BQ34Z100_GPIO_SDA = GPIO_SDA;
	BQ34Z100_SDA_pin = SDA_pin;
}

/**
 * @brief BQ34Z100 I2C起始信号
 * @param NONE
 * @return NONE
 */
void bq34z100_i2c_start(void)
{
	SDA_1();
	I2CDELAY();
	SCL_1();
	I2CDELAY();
	SDA_0();
	I2CDELAY();
	SCL_0();
	I2CDELAY();
}

/**
 * @brief BQ34Z100 I2C停止信号
 * @param NONE
 * @return NONE
 */
void bq34z100_i2c_stop(void)
{
	SDA_0();
	I2CDELAY();
	SCL_1();
	I2CDELAY();
	SDA_1();
	I2CDELAY();
}

/**
 * @brief BQ34Z100 I2C发送字节
 * @param u8 data 发送数据
 * @return ACK
 */
u8 bq34z100_i2c_send_byte(u8 data)
{
	u8 bits, temp, ack;
	u16 wait_cnt;
	SCL_0();
	temp = data;
	bits = 0x08;
	while (bits != 0x00) {
		if (temp & 0x80)
			SDA_1();
		else
			SDA_0();
		I2CDELAY();
		SCL_1();
		wait_cnt = 0;
		while ((BQ34Z100_GPIO_SCL->IDR & BQ34Z100_SCL_pin) == 0) {
			wait_cnt++;
			if (wait_cnt > 200) {
				bq34z100_i2c_stop();
				return (0);
			}
		}
		I2CDELAY();
		temp = (temp << 1);
		SCL_0();
		bits = (bits - 1);
	}
	I2CDELAY();
	SDA_1();
	SCL_1();
	wait_cnt = 0;
	while ((BQ34Z100_GPIO_SCL->IDR & BQ34Z100_SCL_pin) == 0) {
		wait_cnt++;
		if (wait_cnt > 200) {
			bq34z100_i2c_stop();
			return (0);
		}
	}
	I2CDELAY();
	ack = (((BQ34Z100_GPIO_SDA->IDR & BQ34Z100_SDA_pin) == 0) ? 0 : 1);
	SCL_0();
	if (ack)
		return (1);
	else
		return (0);
}

/**
 * @brief BQ34Z100 I2C接收字节
 * @param u8 ack 是否响应ACK
 * @return u8 data 接收数据
 */
u8 bq34z100_i2c_rev_byte(u8 ack)
{
	u8 bits, data = 0;

	SDA_1();
	bits = 0x08;
	while (bits > 0) {
		SCL_1();
		while ((BQ34Z100_GPIO_SCL->IDR & BQ34Z100_SCL_pin) == 0)
			I2CDELAY();
		data = (data << 1);
		if (BQ34Z100_GPIO_SDA->IDR & BQ34Z100_SDA_pin)
			data = (data + 1);
		SCL_0();
		I2CDELAY();
		bits = (bits - 1);
	}
	if (ack)
		SDA_0();
	else
		SDA_1();
	SCL_1();
	I2CDELAY();
	SCL_0();
	SDA_1();

	return (data);
}

/**
 * @brief BQ34Z100 写入数据块
 * @param u8 SlaveAddress  设备地址
 * @param u16 numBytes 读取字节
 * @param void* rx_data 数据指针
 * @param unsigned char multi 是否多数据帧
 * @return NONE
 */
void bq34z100_i2c_write_block(u8 SlaveAddress,
                              u16 numBytes, u8 multi,
                              void* TxData)
{
	u16  i;
	u8* temp;

	temp = (u8*)TxData;
	bq34z100_i2c_start();
	bq34z100_i2c_send_byte(SlaveAddress + 0);
	for (i = 0; i < numBytes; i++) {
		bq34z100_i2c_send_byte(*(temp));
		temp++;
	}
	if (multi == 0) {
		bq34z100_i2c_stop();
	}
	I2CDELAY();
}

/**
 * @brief BQ34Z100 读取数据块
 * @param u8 SlaveAddress  设备地址
 * @param u16 numBytes 读取字节
 * @param void* rx_data 数据指针
 * @return NONE
 */
void bq34z100_i2c_read_block(u8 SlaveAddress,
                             u16 numBytes, void* rx_data)
{
	u16  i;
	u8* temp;

	temp = (u8*)rx_data;
	bq34z100_i2c_start();
	bq34z100_i2c_send_byte(SlaveAddress + 1);
	for (i = 0; i < numBytes; i++) {
		if (i == (numBytes - 1))
			*(temp) = bq34z100_i2c_rev_byte(BQ34Z100_NACK);
		else
			*(temp) = bq34z100_i2c_rev_byte(BQ34Z100_ACK);
		temp++;
	}
	bq34z100_i2c_stop();
}

/**
 * @brief BQ34Z100 i2c初始化
 * @param NONE
 * @return NONE
 */
void bq34z100_i2c_init(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = BQ34Z100_SDA_pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(BQ34Z100_GPIO_SDA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = BQ34Z100_SCL_pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(BQ34Z100_GPIO_SCL, &GPIO_InitStructure);
}

/**
 * @brief BQ34Z100寄存器读取
 * @param u8 port_switch 端口选择 1：电池1,2：电池2
 * @param u16 bytes 读取字节
 * @param void* rx_data 数据指针
 * @return NONE
 */
void bq34z100_read_reg(u8 port_switch, u8 cmd, u16 bytes, void* rx_data)
{
	u8 tx[1];
	tx[0] = cmd;

	if (port_switch == 1) {
		bq34z100_io_config(BATTERY_1_I2C_SCL_GPIO, BATTERY_1_I2C_SCL_PIN,
		                   BATTERY_1_I2C_SDA_GPIO, BATTERY_1_I2C_SDA_PIN);
	} else if (port_switch == 2) {
		bq34z100_io_config(BATTERY_2_I2C_SCL_GPIO, BATTERY_2_I2C_SCL_PIN,
		                   BATTERY_2_I2C_SDA_GPIO, BATTERY_2_I2C_SDA_PIN);
	} else {
		return;
	}
	bq34z100_i2c_init();
	delay_nus(200);
	bq34z100_i2c_write_block(BQ34Z100_ADDR, 1, 1, tx);
	bq34z100_i2c_read_block(BQ34Z100_ADDR, bytes, rx_data);
}

/**
 * @brief BQ34Z100 状态标识处理
 * @param u8 battery_num 电池号
 * @return NONE
 */
void bq34z100_flag_process(u8 battery_num)
{

}

/**
 * @brief BQ34Z100信息读取
 * @param u8 battery_num 电池号
 * @return NONE
 */
void BQ34Z100_get_charge_state(u8 battery_num)
{
	u8 tmp[2];
	static u8 step[2] = {0, 0};
    
    battery_t* battery;
    
    if (battery_num == 1)
        battery = &battery_1;
    else if (battery_num == 2)
        battery = &battery_2;
    else 
        return;
    
	switch (step[battery_num - 1]) {
	case 0:
		// 温度 K
		bq34z100_read_reg(battery_num, BQ34Z100_TEMPERATURE_LSB, 2, tmp);
		battery->temperature = bytes_to_integer(tmp);
		printf("[%d] ", battery_num);
		printf("temperature:%d\n\n", (battery->temperature - 2731) / 10);
		break;

	case 1:
		// 电量百分比 0-100%
		bq34z100_read_reg(battery_num, BQ34Z100_STATE_OF_CHARGE, 1, tmp);
		battery->charge_state = tmp[0];
		printf("[%d] ", battery_num);
		printf("Percentage of electricity:%d \n\n", battery->charge_state);
		break;

	case 2:
		// 电池容量 mAh (充满电之后校准)
		bq34z100_read_reg(battery_num, BQ34Z100_FULL_CHAGRE_CAP_LSB, 2, tmp);
		battery->full_charge_cap = bytes_to_integer(tmp);
		printf("[%d] ", battery_num);
		printf("total capacity:%d mAh \n\n", battery->full_charge_cap);
		break;

	case 3:
		// 剩余电池容量 * 1 mAh
		bq34z100_read_reg(battery_num, BQ34Z100_REMAIN_CAP_LSB, 2, tmp);
		battery->remain_cap = bytes_to_integer(tmp);
		printf("[%d] ", battery_num);
		printf("remain capacity:%d mAh \n\n", battery->remain_cap);
		break;

	case 4:
		// 电压 mV
		bq34z100_read_reg(battery_num, BQ34Z100_VOLTAGE_LSB, 2, tmp);
		battery->voltage = bytes_to_integer(tmp);
		printf("[%d] ", battery_num);
		printf("voltage:%d mV \n\n", battery->voltage);
		break;

	case 5:
		// 电流 mA
		bq34z100_read_reg(battery_num, BQ34Z100_CURRENT_LSB, 2, tmp);
		battery->current = bytes_to_integer(tmp);
		printf("[%d] ", battery_num);
		printf("current:%d mA \n\n", battery->current);
		break;

	case 6:
		// 运行标志
		bq34z100_read_reg(battery_num, BQ34Z100_FLAGS_LSB, 2, tmp);
		battery->flag = bytes_to_integer(tmp);
		printf("[%d] ", battery_num);
		printf("run flag:%x \n\n", battery->flag);
		break;

	default:

		break;
	}
    
	(step[battery_num - 1] > 6) ? (step[battery_num - 1] = 0) :
                                   step[battery_num - 1]++;
	
}
