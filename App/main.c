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

struct PointStruct
{
	uint8 row;
	uint8 col;
	uint8 pointType;
};

struct CenterPointStruct
{
	int16 row;
	int16 col;
};

enum TiaobianPointEnum
{
	NONE,
	LEFT,//上升沿
	RIGHT,//下降沿
	LEFTBLACK,//左边线,存在黑点
	RIGHTBLACK,//右边线，存在黑点
	LEFTLOST,//左边线，丢边
	RIGHTLOST,//右边线，丢边
};

struct LinePointStruct
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
int16 two(uint8 *img);
void duoji(int16 loca_error, uint32 time);
int16 loca_lasterror;

/*!
 *  @brief      main函数
 *  @since      v5.0
 *  @note       山外 DMA 采集摄像头 实验
                注意，此例程 bus频率设为100MHz(50MHz bus频率会太慢而导致没法及时采集图像)
 */
void  main(void)
{
	int16 loca_error;
	uint32 i;
	//初始化摄像头
	camera_init(imgbuff);
	ftm_pwm_init(S3010_FTM, S3010_CH, S3010_HZ, 1480);    //初始化 舵机 PWM,

	ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM, MOTOR_HZ, 0);    //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM, MOTOR_HZ, 20);    //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM, MOTOR_HZ, 0);    //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM, MOTOR_HZ, 20);    //初始化 电机 PWM
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
		//sendimg(imgbuff, CAMERA_W * CAMERA_H / 8);              //发送到上位机
		//sendimg_eSmartCameraCar(img, CAMERA_W * CAMERA_H);
		LCD_PrintU16(80, 4, pit_time_get_us(PIT0));
		// unsigned char * bmp;

		i = pit_time_get_us(PIT0);
		loca_error = two(img);
		duoji(loca_error, pit_time_get_us(PIT0));
		LCD_PrintU16(0, 0, pit_time_get_us(PIT0) - i);
		//sendimg_eSmartCameraCar(img, CAMERA_W * CAMERA_H);

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

	struct LinePointStruct changepoint[CAMERA_H] = {{0}, {0}, 1, 1};
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
	uint8 rightpoint[CAMERA_H];
	uint8 middlePoint[CAMERA_H] = {0};
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
			if (!img[i] == 1 && changepoint[ii].upNum < 5)
				changepoint[ii].upPoint[changepoint[ii].upNum++] = i % CAMERA_W;
			continue;
		}
		if (i % CAMERA_W == 0 )
		{
			if (!img[i] == 1 && changepoint[ii].downNum < 5)
				changepoint[ii].downPoint[changepoint[ii].downNum++] = i % CAMERA_W;
			ii++;
			continue;
		}
		if (img[i] != img[i + 1] && img[i] == img[i - 1])
		{
			if (img[i] == 1 && changepoint[ii].downNum < 5)
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

int16 two(uint8 *src)
{

	enum TiaobianPointEnum imgchange[CAMERA_H][CAMERA_W];
	uint8 i;
	uint16 ii, j;

	uint8 canFindLeftPointFlag = 0;
	uint8 canFindRightPointFlag = 0;
	uint8 canFindDownPointFlag = 0;

	struct PointStruct leftPoint[CAMERA_H];
	struct PointStruct rightPoint[CAMERA_H];
	struct CenterPointStruct centerPoint[CAMERA_H];

	// 断线标志,必须初始化
	uint8 leftStartFlag = -1;
	uint8 rightStartFlag = -1;
	uint8 leftEndFlag = -1;
	uint8 rightEndFlag = -1;
	// 中线断线标志位，开始与结束
	uint8 middleStartFlag = 2;
	uint8 middleEndFlag = 0;


	//中线处理数据
	short count1, count2;
	int16 loca_error;



	/*****************************先进行图像分析，然后记录各种标志位************************************/

	/*****************************提取跳变点************************************/
	//顺序已经颠倒过来了
	for (ii = 0, i = CAMERA_H - 1, j = 0; ii < CAMERA_H * CAMERA_W; ii++)
	{
		if (i >= CAMERA_H || j >= CAMERA_W) break;
		if (j == 0)
		{
			if (src[ii] == 1) imgchange[i][j] = LEFTBLACK;
			else imgchange[i][j] = LEFTLOST;
			j++;
			continue;
		}
		else if (j == CAMERA_W - 1)
		{
			if (src[ii] == 1) imgchange[i][j] = RIGHTBLACK;
			else imgchange[i][j] = RIGHTLOST;
			j = 0;
			i--;
			continue;
		}
		else if (src[ii] != src[ii - 1] && src[ii] == src[ii + 1] && src[ii] == src[ii + 2])
		{
			if (src[ii] == 1) imgchange[i][j] = RIGHT;
			else imgchange[i][j] = LEFT;
			j++;
			continue;
		}
		else
		{
			imgchange[i][j] = NONE;
			j++;
		}
	}
	ii = 0;
	//跳变点上位机显示
	// for (ii = 0, i = 0, j = 0; ii < CAMERA_H * CAMERA_W; ii++)
	// {
	// 	if (imgchange[i][j] == LEFT)
	// 		src[ii] = 255;
	// 	else
	// 		src[ii] = 0;
	// 	j++;
	// 	if (j == CAMERA_W)
	// 	{
	// 		i++; j = 0;
	// 	}
	// }


	/**************************************检测前三行，排除干扰因素*************************/
	// 方法是：取中点算前三行的平均值
	for (i = 0; i < 3; i++)
	{
		//向右寻边线
		for (j = CAMERA_W / 2; j < CAMERA_W; j++)
		{
			if (imgchange[i][j] == RIGHT || imgchange[i][j] == RIGHTLOST || imgchange[i][j] == RIGHTBLACK)
			{
				rightPoint[i].row = i;
				rightPoint[i].col = j;
				rightPoint[i].pointType = imgchange[i][j];
				//如果检测到边界非空白并且rightStartFlag==-1即未使用过，则将之赋值为i;
				if (imgchange[i][j] == RIGHTBLACK && rightStartFlag == -1)rightStartFlag = i;
				else if (imgchange[i][j] == RIGHTLOST && rightStartFlag != -1 && rightEndFlag == -1)
				{
					if (i == CAMERA_H - 1)rightEndFlag = i;
					else if (imgchange[i + 1][j] == RIGHTLOST)rightEndFlag = i;
				}
				break;
			}
		}

		// 向左寻线
		for (j = CAMERA_W / 2; j >= 0; j--)
		{
			if (imgchange[i][j] == LEFT || imgchange[i][j] == LEFTLOST || imgchange[i][j] == LEFTBLACK)
			{
				leftPoint[i].row = i;
				leftPoint[i].col = j;
				leftPoint[i].pointType = imgchange[i][j];
				//如果检测到边界非空白并且leftStartFlag==-1即未使用过，则将之赋值为i;
				if (imgchange[i][j] == LEFTBLACK && leftStartFlag == -1)leftStartFlag = i;
				else if (imgchange[i][j] == LEFTLOST && leftStartFlag != -1 && leftEndFlag == -1)
				{
					if (i == CAMERA_H - 1)leftEndFlag = i;
					else if (imgchange[i + 1][j] == LEFTLOST)leftEndFlag = i;
				}
				break;
			}
			if (j == 0)break;
		}
		//不管如何，中心点都是要算出来的
		centerPoint[i].col = (rightPoint[i].col + leftPoint[i].col) / 2;
		centerPoint[i].row = i;
		if (i > 0)
		{
			centerPoint[i].col = (centerPoint[i].col + centerPoint[i - 1].col) / 2;
		}
		// // 如果是第一次，那么就继续循环
		// if (i < 2)	continue;

		// //如果两边都不出现转折，则认为取到了合适的底线
		// if ((leftPoint[i].col - leftPoint[i - 1].col) * (leftPoint[i - 1].col - leftPoint[i - 2].col) >= 0 && (rightPoint[i].col - rightPoint[i - 1].col) * (rightPoint[i - 1].col - rightPoint[i - 2].col) >= 0)
		// {
		// 	// canFindDownPointFlag = 1;
		// 	// leftStartFlag = 3;
		// 	// rightStartFlag = 3;
		// 	break;
		// }
	}

	/**********************************************处理剩下的行数，确保边线的准确性***************/
	//处理方法：
	//从上一行的中心开始寻点，范围为上一行的左右边线+-20；
	for (i = 3; i < CAMERA_H; i++)
	{
		// 向右寻线
		for (j = centerPoint[i - 1].col; j < rightPoint[i - 1].col + 20; j++)
		{
			if (j >= CAMERA_W - 1 || imgchange[i][j] == RIGHT || imgchange[i][j] == RIGHTLOST || imgchange[i][j] == RIGHTBLACK)
			{
				rightPoint[i].row = i;
				rightPoint[i].col = j;
				rightPoint[i].pointType = imgchange[i][j];
				// 如果检测到边界非空白并且rightStartFlag==-1即未使用过，则将之赋值为i;
				if (imgchange[i][j] == RIGHTBLACK && rightStartFlag == -1)rightStartFlag = i;
				else if (imgchange[i][j] == RIGHTLOST && rightStartFlag != -1 && rightEndFlag == -1 )
				{
					if (i == CAMERA_H - 1)rightEndFlag = i;
					else if (imgchange[i + 1][j] == RIGHTLOST)rightEndFlag = i;
				}
				break;
			}
		}

		// 向左寻线
		for (j = centerPoint[i - 1].col + 1; j >= leftPoint[i - 1].col - 20; j--)
		{
			if (imgchange[i][j] == LEFT || imgchange[i][j] == LEFTLOST || imgchange[i][j] == LEFTBLACK)
			{
				leftPoint[i].row = i;
				leftPoint[i].col = j;
				leftPoint[i].pointType = imgchange[i][j];
				//如果检测到边界非空白并且leftStartFlag==-1即未使用过，则将之赋值为i;
				if (imgchange[i][j] == LEFTBLACK && leftStartFlag == -1)leftStartFlag = i;
				else if (imgchange[i][j] == LEFTLOST && leftStartFlag != -1 && leftEndFlag == -1 )
				{
					if (i == CAMERA_H - 1)leftEndFlag = i;
					else if (imgchange[i + 1][j] == LEFTLOST)leftEndFlag = i;
				}
				break;
			}
			if (j == 0)break;
		}

		//如果左边线与右边线相差不到2个格，则认为左右边线相交，跳出循环，并记录行数到middleEndFlag中
		centerPoint[i].col = (rightPoint[i].col + leftPoint[i].col) / 2 ;
		centerPoint[i].row = i;
		middleEndFlag = i;
		if (abs(rightPoint[i].col - leftPoint[i].col) < 20) break;
	}
	for (i = middleStartFlag; i <= middleEndFlag; i++)
	{
		src[centerPoint[i].col + (CAMERA_H - 1 - centerPoint[i].row )* CAMERA_W] = 0;
	}


	/********************************提取中线后，使用加权算法，处理中线；并判断各种情况，实现不同路况下的控制操作*************************/
//选取的角度处理方式的要求：滤波性，近似指数曲线
	//每行加权处理，最高行加权为3，最底行加权为1，平均到每行上的（3-1）/CAMERA_H;
	for (i = 0; i <= middleEndFlag; i++)
	{
		centerPoint[i].col = (centerPoint[i].col - (CAMERA_W / 2)) * (1 + i * 2 / CAMERA_H);
	}
	//每20行加权处理
	//
	//选取前40行数据进行处理；
	if (middleEndFlag >= 30)
	{
		count1 = 0;
		for (i = 0; i < 30; i++)
			count1 = count1 + centerPoint[i].col;
		count1 = count1 * 4 / 30;
		count2 = 0;
		for (i = 30; i <= middleEndFlag; i++)
			count2 = count2 + centerPoint[i].col;
		count2 = count2 * 1 / (middleEndFlag - 29);
		loca_error = (count1 + count2) / 5;
	}
	else
	{
		count1 = 0;
		for (i = 0; i <= middleEndFlag; i++)
			count1 = count1 + centerPoint[i].col;
		count1 = count1 / (middleEndFlag + 1);
		loca_error = count1;
	}

	return loca_error;
}
void duoji(int16 loca_error, uint32 time)
{
	uint16 loca_Kp;
	int loca_out;
	uint16 loca_Td = 0;

	loca_Kp = (loca_error * loca_error) / 2;
	loca_out = loca_Kp * (loca_error + loca_Td / time * (loca_error - loca_lasterror));
	loca_lasterror = loca_error;
	loca_out = loca_out / 16 / 16 * -2;
	LCD_PrintU16(0, 4, loca_out+1000);
	if (loca_out > 150)loca_out = 150;
	if (loca_out < -150)loca_out = -150;
	loca_out = loca_out + 1480;
	LCD_PrintU16(0, 6, loca_out);
	ftm_pwm_duty(S3010_FTM, S3010_CH, loca_out); //初始化 舵机 PWM,
}

