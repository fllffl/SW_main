/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"
#include "math.h"

#define S3010_FTM   FTM1
#define S3010_CH    FTM_CH0
#define S3010_HZ    (100)

#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH4
#define MOTOR2_PWM  FTM_CH3
#define MOTOR3_PWM  FTM_CH2
#define MOTOR4_PWM  FTM_CH1

#define MOTOR1_PWM_IO  FTM0_CH4
#define MOTOR2_PWM_IO  FTM0_CH3
#define MOTOR3_PWM_IO  FTM0_CH2
#define MOTOR4_PWM_IO  FTM0_CH1


#if 0
#define MOTOR_HZ    (50)
#else
#define MOTOR_HZ    (20*1000)
#endif

enum point
{
	none,
	up,
	down
};

struct linepointstruct
{
	uint8 upPoint[5];
	uint8 downPoint[5];
	uint8 upNum;
	uint8 downNum;
};

uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组
uint8 img[CAMERA_W * CAMERA_H];                         //由于鹰眼摄像头是一字节8个像素，因而需要解压为 1字节1个像素，方便处理

//函数声明
void sendimg(uint8 *imgaddr, uint32 imgsize);          //发送图像到上位机
void sendimg_eSmartCameraCar(uint8 *imgaddr, uint32 imgsize); //发送图像到eSmartCameraCar上位机
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void one(uint8 *img);
void two(uint8 *img);

/*!
 *  @brief      main函数
 *  @since      v5.0
 *  @note       山外 DMA 采集摄像头 实验
                注意，此例程 bus频率设为100MHz(50MHz bus频率会太慢而导致没法及时采集图像)
 */
void  main(void)
{
	uint32 i;
	//初始化摄像头
	camera_init(imgbuff);
	ftm_pwm_init(S3010_FTM, S3010_CH, S3010_HZ, 148);    //初始化 舵机 PWM,

	ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM, MOTOR_HZ, 0);    //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM, MOTOR_HZ, 15);    //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM, MOTOR_HZ, 0);    //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM, MOTOR_HZ, 15);    //初始化 电机 PWM
	//初始化LCD
	LCD_Init();
	// LCD_CLS();//清屏
	Draw_LQLogo();
	DELAY_MS(1000);//延时一秒
	LCD_CLS();//清屏

	//配置中断服务函数
	set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //设置LPTMR的中断服务函数为 PORTA_IRQHandler
	set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //设置LPTMR的中断服务函数为 PORTA_IRQHandler

	while (1)
	{
		//获取图像
		pit_time_start(PIT0);
		camera_get_img();                                   //摄像头获取图像
		LCD_PrintU16(80, 0, pit_time_get_us(PIT0));

		//解压图像
		img_extract(img, imgbuff, CAMERA_SIZE);        //往上位机中传数据无需转码
		LCD_PrintU16(80, 2, pit_time_get_us(PIT0));

		//发送图像到上位机
		sendimg(imgbuff, CAMERA_W * CAMERA_H / 8);              //发送到上位机
		//sendimg_eSmartCameraCar(img, CAMERA_W * CAMERA_H);
		LCD_PrintU16(80, 4, pit_time_get_us(PIT0));
		// unsigned char * bmp;

		i = pit_time_get_us(PIT0);
		one(img);
		LCD_PrintU16(0, 0, pit_time_get_us(PIT0) - i);

	}
}

/*!
 *  @brief      发送图像到eSmartCameraCar上位机显示
 *  @param      imgaddr         图像地址
 *  @param      imgsize         图像占用空间大小
 *  @since      v5.0
 *  @note       不同的上位机，不同的命令，这里使用 eSmartCameraCar软件，
                如果使用其他上位机，则需要修改代码。
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //发送到上位机
 */
void sendimg(uint8 *imgaddr, uint32 imgsize)
{
#define CMD_WARE     1
	uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //山外上位机 使用的命令
	uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //山外上位机 使用的命令

	uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //先发送命令

	uart_putbuff(VCAN_PORT, imgaddr, imgsize); //再发送图像

	uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //先发送命令
}
void sendimg_eSmartCameraCar(uint8 *imgaddr, uint32 imgsize)
{
	uint8 cmd[4] = {0, 255, 1, 0};

	uart_putbuff(VCAN_PORT, cmd, sizeof(cmd));    //先发送命令

	uart_putbuff(VCAN_PORT, imgaddr, imgsize); //再发送图像
}

/*!
 *  @brief      二值化图像解压（空间 换 时间 解压）
 *  @param      dst             图像解压目的地址
 *  @param      src             图像解压源地址
 *  @param      srclen          二值化图像的占用空间大小
 *  @since      v5.0            img_extract(img, imgbuff,CAMERA_SIZE);
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //发送到上位机
 */
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
	uint8 colour[2] = {255, 1}; //0 和 1 分别对应的颜色
	//注：山外的摄像头 0 表示 白色，1表示 黑色
	uint8 tmpsrc;
	while (srclen --)
	{
		tmpsrc = *src++;
		*dst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
		*dst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
	}
}

/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
	uint8  n = 0;    //引脚号
	uint32 flag = PORTA_ISFR;
	PORTA_ISFR  = ~0;                                   //清中断标志位

	n = 29;                                             //场中断
	if (flag & (1 << n))                                //PTA29触发中断
	{
		camera_vsync();
	}
#if 0             //鹰眼直接全速采集，不需要行中断
	n = 28;
	if (flag & (1 << n))                                //PTA28触发中断
	{
		camera_href();
	}
#endif
}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
	camera_dma();
}






void one(uint8 *img)
{

	struct linepointstruct changepoint[CAMERA_H] = {{0}, {0}, 1, 1};
	uint8 ii, j;
	int16 i;

	int fangcha;
	uint16 count;


	// 断线标志,必须初始化,左右断线标志均置为图像高+1
	uint8 leftCutFlag = CAMERA_H;
	uint8 rightCutFlag = CAMERA_H;
	// 中线断线标志位，开始与结束
	uint8 middleStartFlag = 0;
	uint8 middleEndFlag = 0;

	uint8 leftpoint[CAMERA_H];
	uint8 rightpoint[CAMERA_H] ;
	uint8 middlePoint[CAMERA_H] = {0} ;
	//中线处理函数
	int16 middlechange[CAMERA_H] = {0};
	int xielv;
	//提取跳变点
	//
	//
	ii = 0;
	j = 0;
	for (i = CAMERA_H * CAMERA_W - 1; i > 0; i--)
	{
		if (ii >= CAMERA_H * CAMERA_W - 1) break;
		if ((i + 1) % CAMERA_W == 0 )
		{
			if (!img[i] && changepoint[ii].upNum < 5)
				changepoint[ii].upPoint[changepoint[ii].upNum++] = i % CAMERA_W;
			continue;
		}
		if (i % CAMERA_W == 0 )
		{
			if (!img[i] && changepoint[ii].downNum < 5)
				changepoint[ii].downPoint[changepoint[ii].downNum++] = i % CAMERA_W;
			ii++;
			continue;
		}
		if (img[i] != img[i + 1] && img[i] == img[i - 1])
		{
			if (img[i] && changepoint[ii].downNum < 5)
				changepoint[ii].downPoint[changepoint[ii].downNum++] = i % CAMERA_W;
			else if (changepoint[ii].upNum < 5)
				changepoint[ii].upPoint[changepoint[ii].upNum++] = i % CAMERA_W;
		}

	}

	//先分析最下一行，判断是否符合条件
	//从中线向两边找寻下降沿作为左边线，寻找上升沿作为右边线
	//共寻找三项
	//目的是滤除干扰，最底行很重要，因为后面的要根据前面的去寻线
	for (ii = 0; ii < CAMERA_H; ii++)
	{
		leftpoint[ii] = 0;
		rightpoint[ii] = CAMERA_W;
		for (j = 0; j < changepoint[ii].downNum; j++)
		{
			if (changepoint[ii].downPoint[j] < CAMERA_W / 2)
			{
				if (leftpoint[ii] < changepoint[ii].downPoint[j])
					leftpoint[ii] = changepoint[ii].downPoint[j];
			}
		}
		for (j = 0; j < changepoint[ii].upNum; j++)
		{
			if (changepoint[ii].upPoint[j] > CAMERA_W / 2)
			{
				if (rightpoint[ii] > changepoint[ii].upPoint[j])
					rightpoint[ii] = changepoint[ii].upPoint[j];
			}
		}
		middleStartFlag++;
		middlePoint[ii] = (leftpoint[ii] + rightpoint[ii]) / 2;

		if (ii > 2 && leftpoint[ii] - leftpoint[ii - 1] < 3 && rightpoint[ii] - rightpoint[ii - 1] < 3) break;
	}


	// 从上面找寻的最后一行的下一行为开始，开始进行寻线，并将之保存在leftpoint与rightpoint数组中
	for (ii++; ii < CAMERA_H; ii++)
	{
		leftpoint[ii] = changepoint[ii].downPoint[0];
		rightpoint[ii] = changepoint[ii].upPoint[0];
		//寻找左边线，若丢线则不进入对应的if结构
		if (ii < leftCutFlag)
		{
			for (j = 1; j < changepoint[ii].downNum; j++)
			{
				if (abs(leftpoint[ii - 1] - changepoint[ii].downPoint[j]) < abs( leftpoint[ii - 1] - leftpoint[ii]  ) )
					leftpoint[ii] = changepoint[ii].downPoint[j];
			}
		}
		if (ii < rightCutFlag)
		{
			for (j = 1; j < changepoint[ii].upNum; j++)
			{
				if (abs(changepoint[ii].upPoint[j] - rightpoint[ii - 1] ) < abs(rightpoint[ii] - rightpoint[ii - 1] ))
					rightpoint[ii] = changepoint[ii].upPoint[j];
			}
		}

		//判断是否丢线,若两行的列数相差大于5，且leftCutFlag==CAMERA_H，则认为从该行丢线
		if ((abs(leftpoint[ii] - leftpoint[ii - 1] ) > 5 || (leftpoint[ii] == 0 && leftpoint[ii - 1] != 0 && leftpoint[ii - 2] != 0) ) && leftCutFlag == CAMERA_H && ii != middleStartFlag)
		{
			leftCutFlag = ii;
		}
		if ((abs(rightpoint[ii] - rightpoint[ii - 1]) > 5 || (rightpoint[ii] == 0 && rightpoint[ii - 1] != 0 && rightpoint[ii - 2] != 0)  ) && rightCutFlag == CAMERA_H && ii != middleStartFlag)
		{
			rightCutFlag = ii;
		}

		// 当左右边线都没有丢线时，计算中线，并实时更新中线断线标志位middleEndFlag()    !!此处可以考虑不实时更新middleEndFlag
		if (leftCutFlag == CAMERA_H && rightCutFlag == CAMERA_H)
		{
			middlePoint[ii] = (leftpoint[ii] + rightpoint[ii]) / 2;
			middleEndFlag = ii - 1;
		}

		// 当一边丢线时
		// 当两边都丢线时
	}

	// 遍历中线，求取方差
	for (i = middleStartFlag; i < middleEndFlag; i++)
	{
		count = middlePoint[i] + count;
	}
	count = count / middleEndFlag;
	fangcha = 0;
	for (i = middleStartFlag; i < middleEndFlag ; i++)
	{
		fangcha += sqrt(abs(middlePoint[i] - count));
	}
	fangcha = fangcha * 100 / (middleEndFlag - middleStartFlag);
	uint8 jiao;

	for (i = 0; i < middleEndFlag ; i++)
	{
		middlechange[i] = (1 + 0.03 * i) * (middlePoint[i] - CAMERA_W / 2);
	}

	xielv = (int)( (middlechange[middleEndFlag - 1] - middlechange[middleStartFlag]) * 100 / (middleEndFlag - middleStartFlag));


	if (fangcha < 190)fangcha = 190;
	jiao = (fangcha - 190) / 100;
	LCD_PrintU16(0, 3, leftCutFlag);
	LCD_PrintU16(0, 5, rightCutFlag);
	i = -1 * xielv / 12 + 148;
	if (i > 165)i = 160;
	if (i < 135)i = 135;
	LCD_PrintU16(0, 2, i);
	ftm_pwm_duty(S3010_FTM, S3010_CH, i);
}

void two(uint8 *src) {
	enum point img[CAMERA_H][CAMERA_W];


}

