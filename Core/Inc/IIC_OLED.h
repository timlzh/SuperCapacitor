/*
 * @Author: LO_StartNet
 * @Date: 2022-08-15 13:14:01
 * @LastEditors: LO_StartNet
 * @LastEditTime: 2022-08-19 00:42:59
 * @Description: ʹ��IIC����128*64OLED
 * 
 * Copyright (c) 2022 by LO_StartNet, All Rights Reserved. 
 */

#ifndef _IIC_OLED_H_
#define _IIC_OLED_H_

#include "main.h"
/*************************************************************************/
/**
 * ʹ��˵����
 * 1������OLED_Init()��ʼ��
 * 2��������������ʵ����Ӧ����
 * Init����ʲô�ο�ע��
 * �ο�datasheetʹ��
 * delay��time��������ʹ�õ�Ƭ�������޸�
*/
/*****************************USER_DEFINE***********************************/

#define OLED_ADDRESS 0x78

#define OLED_SDA_Port GPIOB 
#define OLED_SDA_Pin GPIO_PIN_7

#define OLED_SCL_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6

/************************************************************************/

#define OLED_SDA_1() OLED_SDA_Port ->BSRR = OLED_SDA_Pin
#define OLED_SDA_0() OLED_SDA_Port ->BSRR = (uint32_t)OLED_SDA_Pin << 16U

#define OLED_SCL_1() OLED_SCL_Port ->BSRR = OLED_SCL_Pin
#define OLED_SCL_0() OLED_SCL_Port ->BSRR = (uint32_t)OLED_SCL_Pin << 16U

#define OLED_READ() ((OLED_SDA_Port->IDR & OLED_SDA_Pin) != (uint32_t)GPIO_PIN_RESET)

/****************************************************************/

typedef enum
{
    OLED_ROLL_5=0x00,
    OLED_ROLL_64=0x01,
    OLED_ROLL_128=0x02,
    OLED_ROLL_256=0x03,
    OLED_ROLL_3=0x04,
    OLED_ROLL_4=0x05,
    OLED_ROLL_25=0x06,
    OLED_ROLL_2=0x07,
}OLED_ROLL_SPEED;//�����ٶ�ѡ��ԽСԽ��

typedef enum{
    OLED_ROLL_LEFT,
    OLED_ROLL_RIGHT,
}OLED_ROLL_DIR;

/****************************************************************/
uint8_t OLED_Init(void);//��ʼ������
void OLED_SetBright(uint8_t lingt);//���ȵ��ں���
void OELD_StartShow(void);//������ʾ����
void OLED_StopShow(void);//�ر���ʾ����
void OLED_ShowALL(uint8_t ch);//ȫ��ʾ����
void OLED_RollBack(void);//��ת��ʾ����
void OLED_HoriChange(void);//���ҷ�ת��ʾ,����ûɶ��
void OLED_VertChange(void);//���·�ת��ʾ
void OLED_Clear(void);//��������
void OLED_StopRoll(void);//ֹͣ����
void OLED_StartHoriRoll(OLED_ROLL_DIR dir,OLED_ROLL_SPEED speed,uint8_t Spage,uint8_t Epage);//ˮƽ����
void OLED_StartHoriAndVertRoll(OLED_ROLL_DIR dir,OLED_ROLL_SPEED speed,uint8_t Spage,uint8_t Epage,uint8_t vert);//ˮƽ�ʹ�ֱ����
void OLED_SetVertRollPage(uint8_t Dpage,uint8_t Rpage);//���ô�ֱ��������,�����Ժ���ûɶ��

void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Size,uint8_t mode);//��ʾ�ַ�
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t* chr,uint8_t Size,uint8_t mode);//��ʾ�ַ���
void OLED_Printf(uint8_t x,uint8_t y,uint8_t Size,uint8_t mode,const char *__format, ...);//����Ļprintf
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t mode,uint8_t no);//��ʾ����
void OLED_ShowBmp(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[]);//��ʾͼƬ

uint8_t OLED_Test(void);//���Ժ���

#endif
