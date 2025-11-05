

#include "MyProject.h"

/*****************************************************************************/
#define  SPI_CS0_L   GPIO_ResetBits(GPIOA, GPIO_Pin_0)
#define  SPI_CS0_H   GPIO_SetBits(GPIOA, GPIO_Pin_0)
#define  SPI_CS1_L   GPIO_ResetBits(GPIOA, GPIO_Pin_1)
#define  SPI_CS1_H   GPIO_SetBits(GPIOA, GPIO_Pin_1)

#define  SPI3_TX_OFF() {GPIOC->MODER&=~(3<<(12*2));GPIOC->MODER|=0<<(12*2);}  //PC12(MOSI)输入浮空
#define  SPI3_TX_ON()  {GPIOC->MODER&=~(3<<(12*2));GPIOC->MODER|=2<<(12*2);}  //PC12(MOSI)复用推挽输出
/******************************************************************************/
/******************************************************************************/
void delay_s(uint32_t i)
{
	while(i--);
}
/*****************************************************************************/
uint16_t ReadAS5047P(void)
{
	uint16_t u16Data;
	
	SPI_CS0_L;
	SPIx_ReadWriteByte(0xFFFF);
	SPI_CS0_H;
	delay_s(16);  //1us，实际测试延时4读不出角度，延时8可以
	
	SPI_CS0_L;
	u16Data = SPIx_ReadWriteByte(0);
	SPI_CS0_H;
	delay_s(16);  //1us
	
	return u16Data;
}
/******************************************************************************/
uint16_t ReadTLE5012B(void)
{
	uint16_t u16Data;
	
	SPI_CS0_L;
	SPIx_ReadWriteByte(0x8020);
	SPI3_TX_OFF();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();  //Twr_delay=130ns min，实际测试不加延时也可以
	u16Data = SPIx_ReadWriteByte(0xffff);
	
	SPI_CS0_H;
	SPI3_TX_ON();
	
	return(u16Data);
}
/*****************************************************************************/
//SPI不能用mode1，否则角度读出在0——180°，少了1个bit
uint16_t ReadMA730(void)
{
	uint16_t u16Data;
	
	SPI_CS0_L;
	u16Data = SPIx_ReadWriteByte(0);
	SPI_CS0_H;
	
	return u16Data;
}
/******************************************************************************/
uint16_t ReadMT6701(void)
{
	uint16_t u16Data;
	
	SPI_CS0_L;
	u16Data = SPIx_ReadWriteByte(0);
	SPI_CS0_H;
	
	return (u16Data&0xFFFC);
}
/*****************************************************************************/

