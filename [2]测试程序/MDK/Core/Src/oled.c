#include "oled.h"
#include "font.h"
#include "i2c.h"

extern uint8_t oled_data[16];
#define OLED_ADDRESS 0x78


/* 写oled参数 */
void oled_write(uint8_t w_address,uint8_t *pdata,uint16_t length)
{	
	HAL_I2C_Master_Transmit(&hi2c2,OLED_ADDRESS,pdata,length,HAL_MAX_DELAY);
}
/* 写命令 */
void oled_write_command(uint8_t command)
{
	HAL_I2C_Mem_Write(&hi2c2,OLED_ADDRESS,0x00,I2C_MEMADD_SIZE_8BIT,&command,1,HAL_MAX_DELAY);
}
/* 写数据 */
void oled_write_data(uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c2,OLED_ADDRESS,0x40,I2C_MEMADD_SIZE_8BIT,&data,1,HAL_MAX_DELAY);
}
/* 打开显示 */
void oled_display_on(void)
{
//  oled_write_command(0x8D);
//  oled_write_command(0x14);
  oled_write_command(0xAF);//DISPLAY ON
}
/* 关闭显示 */
void oled_display_off(void)
{
//	oled_write_command(0x8D);
//	oled_write_command(0x10);
	oled_write_command(0xAE);//DISPLAY OFF
}
/* 设置像素点，x0-127 y0-31 */
void oled_set_pos(uint8_t x, uint8_t y)
{
    oled_write_command(0xb0 + y);
    oled_write_command(((x & 0xf0) >> 4) | 0x10);
    oled_write_command((x & 0x0f));
}

/* 清屏 */ 
void oled_clear(void)
{
	uint8_t i, n;

	for(i = 0; i < 4; i++)
	{
		oled_write_command (0xb0 + i);
		oled_write_command (0x00);
		oled_write_command (0x10);
		for(n = 0; n < 128; n++)
		{
			oled_write_data(0);
		}
	}
}


/*>
*/
void oled_show_char(uint8_t x, uint8_t y, uint8_t chr)
{
  uint8_t  c = 0, i = 0;
  c = chr - ' ';
	
  if(x > Max_Column - 1)
  {
    x = 0;
		y = y + 2;
  }
	
	oled_set_pos(x, y);
	for(i = 0; i < 8; i++)
	{	
		oled_write_data(F8X16[c * 16 + i]);
	}

	oled_set_pos(x, y + 1);
	for(i = 0; i < 8; i++)
	{	
		oled_write_data(F8X16[c * 16 + i + 8]);
	}
}

/*>
*/
void oled_show_string(uint8_t line, char *chr)
{	
	unsigned char index = 0;
	unsigned char xpos = 0;
	
	while(chr[index] != 0)
	{
		oled_show_char(xpos, line,chr[index]);
		
		xpos += 8;
		
		if(xpos >= 128){
			break;
		}
		
		index++;
	}
}


/*>
*/
void oled_init(void)
{

	/*OLED工作模式配置指令*/
	oled_write_command(0xAE);
	oled_write_command(0xD5);
	oled_write_command(0x80);
	oled_write_command(0xA8);
	oled_write_command(0x1F);
	oled_write_command(0xD3);
	oled_write_command(0x00);
	oled_write_command(0x40);
	oled_write_command(0x8D);
	oled_write_command(0x14);
	oled_write_command(0xA1);
	oled_write_command(0xC8);
	oled_write_command(0xDA);
	oled_write_command(0x00);
	oled_write_command(0x81);
	oled_write_command(0x8F);
	oled_write_command(0xD9);
	oled_write_command(0x1F);
	oled_write_command(0xDB);
	oled_write_command(0x40);
	oled_write_command(0xA4);

	oled_clear();
	oled_write_command(0xAF);
	HAL_Delay(100);
	
}





