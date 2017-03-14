#ifndef _BATTERY_MANAGE_H
#define _BATTERY_MANAGE_H
#include "type.h"
#include "stm32f10x.h"
#define BATTERY_1_I2C_SCL_GPIO  GPIOB
#define BATTERY_1_I2C_SCL_PIN   GPIO_Pin_8

#define BATTERY_1_I2C_SDA_GPIO  GPIOB
#define BATTERY_1_I2C_SDA_PIN   GPIO_Pin_9

#define BATTERY_2_I2C_SCL_GPIO  GPIOB
#define BATTERY_2_I2C_SCL_PIN   GPIO_Pin_10

#define BATTERY_2_I2C_SDA_GPIO  GPIOB
#define BATTERY_2_I2C_SDA_PIN   GPIO_Pin_11

#define BQ34Z100_ADDR 0xAA

#define SDA_1() do{\
                    GPIO_InitTypeDef  GPIO_InitStructure;\
                    GPIO_InitStructure.GPIO_Pin = BQ34Z100_SDA_pin;\
                    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;\
                    GPIO_Init(BQ34Z100_GPIO_SDA, &GPIO_InitStructure);\
                 } while (0)

#define SDA_0() do{\
                    GPIO_InitTypeDef  GPIO_InitStructure;\
                    GPIO_InitStructure.GPIO_Pin = BQ34Z100_SDA_pin;\
                    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;\
                    GPIO_Init(BQ34Z100_GPIO_SDA, &GPIO_InitStructure);\
                    GPIO_ResetBits(BQ34Z100_GPIO_SDA, BQ34Z100_SDA_pin);\
                 } while (0)

#define SCL_1() do{\
                    GPIO_InitTypeDef  GPIO_InitStructure;\
                    GPIO_InitStructure.GPIO_Pin = BQ34Z100_SCL_pin;\
                    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;\
                    GPIO_Init(BQ34Z100_GPIO_SCL, &GPIO_InitStructure);\
                 } while (0)

#define SCL_0() do{\
                    GPIO_InitTypeDef  GPIO_InitStructure;\
                    GPIO_InitStructure.GPIO_Pin = BQ34Z100_SCL_pin;\
                    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
                    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;\
                    GPIO_Init(BQ34Z100_GPIO_SCL, &GPIO_InitStructure);\
                    GPIO_ResetBits(BQ34Z100_GPIO_SCL, BQ34Z100_SCL_pin);\
                 } while (0)

#define I2CDELAY()  delay_nus(5)
#define BQ34Z100_NACK 0
#define BQ34Z100_ACK 1

typedef struct {
	u16 charge_state;
	u16 remain_cap;
	u16 full_charge_cap;
	u16 voltage;
	u16 temperature;
	u16 current;
	u16 flag;
} battery_t;

#define BQ34Z100_CONTROL_LSB    0x00
#define BQ34Z100_CONTROL_MSB    0x01

#define BQ34Z100_STATE_OF_CHARGE    0x02
#define BQ34Z100_MAX_ERROR      0x03

#define BQ34Z100_REMAIN_CAP_LSB 0x04
#define BQ34Z100_REMAIN_CAP_MSB 0x05

#define BQ34Z100_FULL_CHAGRE_CAP_LSB 0x06
#define BQ34Z100_FULL_CHAGRE_CAP_MSB 0x07

#define BQ34Z100_VOLTAGE_LSB 0x08
#define BQ34Z100_VOLTAGE_MSB 0x09

#define BQ34Z100_AVA_CURRENT_LSB 0x0A
#define BQ34Z100_AVA_CURRENT_MSB 0x0B

#define BQ34Z100_TEMPERATURE_LSB 0x0C
#define BQ34Z100_TEMPERATURE_MSB 0x0D

#define BQ34Z100_FLAGS_LSB      0x0E
#define BQ34Z100_FLAGS_MSB      0x0F

#define BQ34Z100_CURRENT_LSB      0x10
#define BQ34Z100_CURRENT_MSB      0x11

#define BQ34Z100_FLAGSB_LSB      0x12
#define BQ34Z100_FLAGSB_MSB      0x13

#define LSB_BIT0    0x0001
#define LSB_BIT1    0x0002
#define LSB_BIT2    0x0004
#define LSB_BIT3    0x0008
#define LSB_BIT4    0x0010
#define LSB_BIT5    0x0020
#define LSB_BIT6    0x0040
#define LSB_BIT7    0x0080

#define MSB_BIT0    0x0100
#define MSB_BIT1    0x0200
#define MSB_BIT2    0x0400
#define MSB_BIT3    0x0800
#define MSB_BIT4    0x1000
#define MSB_BIT5    0x2000
#define MSB_BIT6    0x4000
#define MSB_BIT7    0x8000

#define FLAG_OTC        MSB_BIT7    // 充电过温
#define FLAG_OTD        MSB_BIT6
#define FLAG_BATHI      MSB_BIT5
#define FLAG_BATLOW     MSB_BIT4
#define FLAG_CHG_INH    MSB_BIT3
#define FLAG_XCHG       MSB_BIT2
#define FLAG_FC         MSB_BIT1
#define FLAG_CHG        MSB_BIT0

#define FLAG_OCVTAKEN   LSB_BIT7
#define FLAG_RSVD0      LSB_BIT6
#define FLAG_RSVD1      LSB_BIT5
#define FLAG_CF         LSB_BIT4
#define FLAG_RSVD2      LSB_BIT3
#define FLAG_SOC1       LSB_BIT2
#define FLAG_SOCF       LSB_BIT1
#define FLAG_DSG        LSB_BIT0

void BQ34Z100_get_charge_state(u8 battery_num);
void battery_init(void);
#endif
