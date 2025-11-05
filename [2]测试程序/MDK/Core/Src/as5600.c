#include "as5600.h"
#include "i2c.h"
//#include "foc_utils.h"
#include "stdio.h"

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1

#define AS5600_WR_ADDRESS 	0x6C
#define AS5600_RD_ADDRESS 	0x6D

#define AS5600_CPR      		4096
#define STATE_ADDRESS				0x0B
#define RAW_ANGLE 					0x0D

/*取绝对值*/
#define abs(x) ((x)>0?(x):-(x))


float I2C_getRawCount(void);

static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
  
  status = HAL_I2C_Master_Transmit(&hi2c3, dev_addr, pData, count, i2c_time_out);
  return status;
}

static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
  
  status = HAL_I2C_Master_Receive(&hi2c3, (dev_addr | 1), pData, count, i2c_time_out);
  return status;
}

uint16_t bsp_as5600GetRawAngle(void) {
  uint16_t raw_angle;
  uint8_t buffer[2] = {0};
  uint8_t raw_angle_register = 0x0c;
  
  i2cWrite(AS5600_WR_ADDRESS, &raw_angle_register, 1);
  i2cRead(AS5600_WR_ADDRESS, buffer, 2);
  raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  return raw_angle&0x0fff;
}



float I2C_getRawCount(void)//获取角度
{ 
  float angle=0.0;
  
  angle=bsp_as5600GetRawAngle()*0.0878f;
  
  return angle;	
}

