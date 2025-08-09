#include "gw_color_iic.h"
unsigned char IIC_ReadByte_color(unsigned char Salve_Adress)
{
	unsigned char dat;
	HAL_I2C_Master_Receive(&hi2c2,Salve_Adress,&dat,1,1000);
	return dat;
}
unsigned char IIC_ReadBytes_color(unsigned char Salve_Adress,unsigned char Reg_Address,unsigned char *Result,unsigned char len)
{
	return HAL_I2C_Mem_Read(&hi2c2,Salve_Adress,Reg_Address,I2C_MEMADD_SIZE_8BIT,Result,len,1000)==HAL_OK;
}
unsigned char IIC_WriteByte_color(unsigned char Salve_Adress,unsigned char Reg_Address,unsigned char data)
{
	unsigned char dat[2]={Reg_Address,data};
	return HAL_I2C_Master_Transmit(&hi2c2,Salve_Adress,dat,2,1000)==HAL_OK;
}
unsigned char IIC_WriteBytes_color(unsigned char Salve_Adress,unsigned char Reg_Address,unsigned char *data,unsigned char len)
{
	return HAL_I2C_Mem_Write(&hi2c2,Salve_Adress,Reg_Address,I2C_MEMADD_SIZE_8BIT,data,len, 1000)==HAL_OK;
}
//接口
unsigned char Ping_color(void)
{
	unsigned char dat;
	IIC_ReadBytes_color(Color_Adress<<1,PING,&dat,1);
	if(dat==PING_OK)
	{
			return 0;
	}	
	else return 1;
}
unsigned char IIC_Get_Error(void)
{
	unsigned char dat;
	IIC_ReadBytes_color(Color_Adress<<1,Error,&dat,1);
	return dat;
}
unsigned char IIC_Get_RGB(unsigned char * Result,unsigned char len)
{
	if(IIC_ReadBytes_color(Color_Adress<<1,RGB_Reg,Result,len))return 1;
	else return 0;
}
unsigned char IIC_Get_HSL(unsigned char * Result,unsigned char len)
{
	if(IIC_ReadBytes_color(Color_Adress<<1,HSL_Reg,Result,len))return 1;
	else return 0;
}

int Get_GW_Color_RGB(unsigned char *RGB)
{

unsigned char R=RGB[0];
	unsigned char G=RGB[1];
	unsigned char B=RGB[2];
	if (R > 150 &&G > 150 && B > 150)
	{

		// return 135;
		return gw_white_color;
	} // 白色
	  //  if(B-R>=20&&B-G>=20&&B>=60)
//	if (B - R >= 20 && B - G >= 0 && B >= 60)
	if((B > R) && (B > G) && (B > 1.7 * R))

	{

		// return 0;
		return gw_blue_color;
	} // 蓝色
	if (G > B && G > R && G >= 20)
	//   if(G>R&&G>=20)

	{

		// return 180;
		return gw_green_color;
	} // 绿色
	if (R - B >= 40 && R - G >= 40 && R >= 80)
	{

		return gw_red_color;
	} // 红色
	if (R <= 50 && G <= 50 && B <= 50)
	{

		// return 45;
		return gw_black_color;
	} // 黑色

	return -1;
}
int Get_GW_Color_HSL(unsigned char *HSL)
{
unsigned char H=HSL[0];
	unsigned char S=HSL[1];
	unsigned char L=HSL[2];
	if(L>210)
	{
	return gw_white_color;
	}
	if(L<60)
	{
	return gw_black_color;
	}
	
	
if(L<210&&L>60)//红绿蓝
{
if(H>200||H<10)
{
return gw_red_color;
}
if(H>20&&H<100)
{
	return gw_green_color;
}
if(H>100&&H<200)
{
	return gw_blue_color;
}
}	
return -1;
}
