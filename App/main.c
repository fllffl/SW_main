/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ������������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      ɽ��K60 ƽ̨������
 * @author     ɽ��Ƽ�
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

uint8 imgbuff[CAMERA_SIZE];                             //����洢����ͼ�������
uint8 img[CAMERA_W * CAMERA_H];                         //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��

//��������
void sendimg(uint8 *imgaddr, uint32 imgsize);          //����ͼ����λ��
void sendimg_eSmartCameraCar(uint8 *imgaddr, uint32 imgsize); //����ͼ��eSmartCameraCar��λ��
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void one(uint8 *img);
void two(uint8 *img);

/*!
 *  @brief      main����
 *  @since      v5.0
 *  @note       ɽ�� DMA �ɼ�����ͷ ʵ��
                ע�⣬������ busƵ����Ϊ100MHz(50MHz busƵ�ʻ�̫��������û����ʱ�ɼ�ͼ��)
 */
void  main(void)
{
	uint32 i;
	//��ʼ������ͷ
	camera_init(imgbuff);
	ftm_pwm_init(S3010_FTM, S3010_CH, S3010_HZ, 148);    //��ʼ�� ��� PWM,

	ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM, MOTOR_HZ, 0);    //��ʼ�� ��� PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM, MOTOR_HZ, 15);    //��ʼ�� ��� PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM, MOTOR_HZ, 0);    //��ʼ�� ��� PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM, MOTOR_HZ, 15);    //��ʼ�� ��� PWM
	//��ʼ��LCD
	LCD_Init();
	// LCD_CLS();//����
	Draw_LQLogo();
	DELAY_MS(1000);//��ʱһ��
	LCD_CLS();//����

	//�����жϷ�����
	set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //����LPTMR���жϷ�����Ϊ PORTA_IRQHandler
	set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //����LPTMR���жϷ�����Ϊ PORTA_IRQHandler

	while (1)
	{
		//��ȡͼ��
		pit_time_start(PIT0);
		camera_get_img();                                   //����ͷ��ȡͼ��
		LCD_PrintU16(80, 0, pit_time_get_us(PIT0));
		//��ѹͼ��
		img_extract(img, imgbuff, CAMERA_SIZE);        //����λ���д���������ת��
		LCD_PrintU16(80, 2, pit_time_get_us(PIT0));
		//����ͼ����λ��
		sendimg(img, CAMERA_W * CAMERA_H);                //���͵���λ��
		LCD_PrintU16(80, 4, pit_time_get_us(PIT0));
		// unsigned char * bmp;
		i = pit_time_get_us(PIT0);
		one(img);
		LCD_PrintU16(0, 0, pit_time_get_us(PIT0) - i);

		// LCD_CLS();//����
		// Draw_BMP(0, 0, 80, 60, imgbuff);

	}
}

/*!
 *  @brief      ����ͼ��eSmartCameraCar��λ����ʾ
 *  @param      imgaddr         ͼ���ַ
 *  @param      imgsize         ͼ��ռ�ÿռ��С
 *  @since      v5.0
 *  @note       ��ͬ����λ������ͬ���������ʹ�� eSmartCameraCar������
                ���ʹ��������λ��������Ҫ�޸Ĵ��롣
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //���͵���λ��
 */
void sendimg(uint8 *imgaddr, uint32 imgsize)
{
#define CMD_WARE     1
	uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //ɽ����λ�� ʹ�õ�����
	uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //ɽ����λ�� ʹ�õ�����

	uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //�ȷ�������

	uart_putbuff(VCAN_PORT, imgaddr, imgsize); //�ٷ���ͼ��

	uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //�ȷ�������
}
void sendimg_eSmartCameraCar(uint8 *imgaddr, uint32 imgsize)
{
#define CMD_WARE     1
	uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //ɽ����λ�� ʹ�õ�����
	uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //ɽ����λ�� ʹ�õ�����

	uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //�ȷ�������

	uart_putbuff(VCAN_PORT, imgaddr, imgsize); //�ٷ���ͼ��

	uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //�ȷ�������
}

/*!
 *  @brief      ��ֵ��ͼ���ѹ���ռ� �� ʱ�� ��ѹ��
 *  @param      dst             ͼ���ѹĿ�ĵ�ַ
 *  @param      src             ͼ���ѹԴ��ַ
 *  @param      srclen          ��ֵ��ͼ���ռ�ÿռ��С
 *  @since      v5.0            img_extract(img, imgbuff,CAMERA_SIZE);
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //���͵���λ��
 */
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
	uint8 colour[2] = {255, 1}; //0 �� 1 �ֱ��Ӧ����ɫ
	//ע��ɽ�������ͷ 0 ��ʾ ��ɫ��1��ʾ ��ɫ
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
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
	uint8  n = 0;    //���ź�
	uint32 flag = PORTA_ISFR;
	PORTA_ISFR  = ~0;                                   //���жϱ�־λ

	n = 29;                                             //���ж�
	if (flag & (1 << n))                                //PTA29�����ж�
	{
		camera_vsync();
	}
#if 0             //ӥ��ֱ��ȫ�ٲɼ�������Ҫ���ж�
	n = 28;
	if (flag & (1 << n))                                //PTA28�����ж�
	{
		camera_href();
	}
#endif
}

/*!
 *  @brief      DMA0�жϷ�����
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


	// ���߱�־,�����ʼ��,���Ҷ��߱�־����Ϊͼ���+1
	uint8 leftCutFlag = CAMERA_H;
	uint8 rightCutFlag = CAMERA_H;
	// ���߶��߱�־λ����ʼ�����
	uint8 middleStartFlag = 0;
	uint8 middleEndFlag = 0;

	uint8 leftpoint[CAMERA_H];
	uint8 rightpoint[CAMERA_H] ;
	uint8 middlePoint[CAMERA_H] = {0} ;
	//���ߴ�������
	int16 middlechange[CAMERA_H] = {0};
	int xielv;
	//��ȡ�����
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

	//�ȷ�������һ�У��ж��Ƿ��������
	//��������������Ѱ�½�����Ϊ����ߣ�Ѱ����������Ϊ�ұ���
	//��Ѱ������
	//Ŀ�����˳����ţ�����к���Ҫ����Ϊ�����Ҫ����ǰ���ȥѰ��
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


	// ��������Ѱ�����һ�е���һ��Ϊ��ʼ����ʼ����Ѱ�ߣ�����֮������leftpoint��rightpoint������
	for (ii++; ii < CAMERA_H; ii++)
	{
		leftpoint[ii] = changepoint[ii].downPoint[0];
		rightpoint[ii] = changepoint[ii].upPoint[0];
		//Ѱ������ߣ��������򲻽����Ӧ��if�ṹ
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

		//�ж��Ƿ���,�����е�����������5����leftCutFlag==CAMERA_H������Ϊ�Ӹ��ж���
		if ((abs(leftpoint[ii] - leftpoint[ii - 1] ) > 5 || (leftpoint[ii] == 0 && leftpoint[ii - 1] != 0 && leftpoint[ii - 2] != 0) ) && leftCutFlag == CAMERA_H && ii != middleStartFlag)
		{
			leftCutFlag = ii;
		}
		if ((abs(rightpoint[ii] - rightpoint[ii - 1]) > 5 || (rightpoint[ii] == 0 && rightpoint[ii - 1] != 0 && rightpoint[ii - 2] != 0)  ) && rightCutFlag == CAMERA_H && ii != middleStartFlag)
		{
			rightCutFlag = ii;
		}

		// �����ұ��߶�û�ж���ʱ���������ߣ���ʵʱ�������߶��߱�־λmiddleEndFlag()    !!�˴����Կ��ǲ�ʵʱ����middleEndFlag
		if (leftCutFlag == CAMERA_H && rightCutFlag == CAMERA_H)
		{
			middlePoint[ii] = (leftpoint[ii] + rightpoint[ii]) / 2;
			middleEndFlag = ii - 1;
		}

		// ��һ�߶���ʱ
		// �����߶�����ʱ
	}

	// �������ߣ���ȡ����
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
