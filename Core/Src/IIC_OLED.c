/*
 * @Author: LO_StartNet
 * @Date: 2022-08-15 13:13:47
 * @LastEditors: LO_StartNet
 * @LastEditTime: 2022-08-19 00:43:32
 * @Description: ʹ��IIC����128*64OLED
 * 
 * Copyright (c) 2022 by LO_StartNet, All Rights Reserved. 
 */
#include "IIC_OLED.h"
#include "OLEDFonts.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#define TX_BUF_LEN 255 
uint8_t TxBuf[TX_BUF_LEN];  /* ���ͻ�����*/
/**********************************************************/

#define OLED_CMD 0x00 //��������ָʾ
#define OLED_DATA 0x40 //��������ָʾ

#define OLED_CMD_SetContrast 0x81 //���öԱȶ�����ͷ���������һ�ֽ�00h-FFh

#define OLED_CMD_StartShow 0xAF //������ʾָ��
#define OLED_CMD_StopShow 0xAE //�ر���ʾָ��

#define OLED_CMD_ChargePumpH 0x8D //������ɱ�����ͷ
#define OLED_CMD_ChargePumpEN 0x14 //��ʼ��ɱ�
#define OLED_CMD_ChargePumpDis 0x10 //�رյ�ɱ�

#define OLED_CMD_EntireDisplayOn 0xA5 //��������ȫ����
#define OLED_CMD_EntireDisplayOff 0xA4 //�����ʾ����

#define OLED_CMD_RollBackShowOff 0xA6 //1��0��
#define OLED_CMD_RollBackShowOn 0xA7 //0��1��

//Addressing Accessing
//����Ѱַģʽ�͵�ַָ��
#define OLED_CMD_SetAddMode 0x20 //�����ڴ��ַģʽ������ͷ
#define OLED_CMD_AddressHori 0x00 //ˮƽѰַģʽ
#define OLED_CMD_AddressVert 0x01 //��ֱѰַģʽ
#define OLED_CMD_AddressPage 0x02 //ҳѰַģʽ(Ĭ��)
//ҳѰַģʽ��
#define OLED_CMD_SetPage 0xB0 //����ҳ��ַ��ʹ��ʱ���ĺ���λ�� ��Ӧ0-7ҳ--ҳѰַģʽ(page addressing)
#define OLED_CMD_SetLineL 0x00 //�����е���λ��ʹ��ʱʹ�õ���λ--ҳ��ַģʽ(page addressing)
#define OLED_CMD_SetLineH 0x10 //�����и���λ��ʹ��ʱʹ�õ���λ--ҳ��ַģʽ(page addressing)
//ˮƽ�ʹ�ֱ��ַģʽ��
#define OLED_CMD_SetPageAddress 0x22 //ˮƽ�ʹ�ֱģʽ��������ʼҳ����ֹҳ��ַ,��������ֽڣ���Ϊ��ʼ��ַ��Ϊ��ֹ��ַ����ΧΪ0~7��Ĭ��Ϊ0-7
#define OLED_CMD_SetColuAddress 0x21 //ˮƽ�ʹ�ֱģʽ��������ʼ�к���ֹ�У�������ֽڣ���ʼ+��ֹ����Χ0~127��Ĭ��0~127

//Hardware Configuration
 #define OLED_CMD_SetDisplayStartLine 0x40 //����λ������ʼ�У�40h-7Fh������ʼ��0-63��Ĭ��0;������RAM�ڼ���ӳ�䵽COM0
//���ҷ���
 #define OLED_CMD_SetSegRemap_0 0xA0 //�е�ַ0ӳ�䵽SEG0(Ĭ��)
 #define OLED_CMD_SetSegRemap_1 0xA1 //�е�ַ0ӳ�䵽SEG1
//���·���
 #define OLED_CMD_SetScanDirNOR 0xC0 //COM0-COM[N-1],NΪ������(Ĭ��)
 #define OLED_CMD_SetScanDirREM 0xC8 //COM[N-1]-COM0

 #define OLED_CMD_SetMultiRatio 0xA8 //���ø����ʣ���Ӹ�����0x0F~0x3F��Ӧ1-64,Ĭ��111111b(64);��������ʹ�����ü���

 #define OLED_CMD_SetDisplayOffset 0xD3 //���ô�ֱƫ����������Ӿ���ƫ������(0-63),Ĭ��0

 #define OLED_CMD_SetCOMPin 0xDA //������Ӳ����������ͷ��

 //Timing Driving Scheme
 #define OLED_CMD_SetClock 0xD5 //����ʱ�ӷ�Ƶ��������Ƶ�ʵ�����ͷ����Ӿ�������A[7:4]��Ƶ��=A[7:4](Ĭ��1000b),A[3:0]��Ƶ��=A[3:0]+1(Ĭ��0000b)
 #define OLED_CMD_SetPreCharge 0xD9 //����Ԥ�����������ͷ�����������(Ĭ��2)
 #define OLED_CMD_SetVcomh 0xDB //����Vcomh�������ͷ,���00h-0.65Vcc;10h-0.77Vcc(Ĭ��);30h-0.83Vcc
 #define OLED_CMD_NOP 0xE3 //�ղ���

 //Scrolling Command Table
 #define OLED_CMD_ENScroll 0x2F //����26H/27H/29H/2AHָ�����õĹ���,����2AH,2FH
 #define OLED_CMD_DISscroll 0x2E //ֹͣ26H/27H/29H/2AHָ�����õĹ���

 #define OLED_CMD_RightRoll 0x26 //���ҹ�����ָ��ͷ

 #define OLED_CMD_LeftRoll 0x27 //�������ָ��ͷ

 #define OLED_CMD_VeRightRoll 0x29 //��ֱ�����ҹ���ָ��ͷ

 #define OLED_CMD_VeLeftRoll 0x2A //��ֱ���������ָ��ͷ

 #define OELD_CMD_SetVerollArea 0xA3 //���ô�ֱ��������


/***********************��������*****************************/

/**
 * @name: OLED_Delay
 * @description: ��ʱ����
 * @return {*}NONE
 */
static void OLED_Delay()
{
    uint16_t time=20;
    volatile uint16_t t=0;
    for(;time>0;time--) t++;
}

/**
 * @name: IIC_Start
 * @description: CPU����I2C���������ź�
 * @return {*}
 */
static void IIC_Start()
{
    //SCL��SDA���ߵ�����£�SDA���ͷ��Ϳ�ʼ�ź�
    OLED_SCL_1();
    OLED_SDA_1();
    OLED_Delay();
    OLED_SDA_0();
    OLED_Delay();
    OLED_SCL_0();
    
}

/**
 * @name: IIC_Stop
 * @description: CPU����I2C����ֹͣ�ź�
 * @return {*}
 */
static void IIC_Stop()
{
    //SCL����ʱ��SDA�ӵͱ�ߣ�����ֹͣ�ź�
    OLED_SDA_0();
    OLED_SCL_1();
    OLED_Delay();
    OLED_SDA_1();
}

/**
 * @name: IIC_SendByte
 * @description: CPU��I2C�����豸����8bit����,����S��P�ź�
 * @param {uint8_t} byte �ȴ����͵��ֽ�
 * @return {*}
 */
static void IIC_SendByte(uint8_t byte)
{
    //��ʼ״̬SCLΪ0����SCL�ͣ��ı�SDA����SCL�ߣ�����SDAֵ
    for(uint8_t i=0;i<8;i++)//��λ�ȷ���
    {
        if(byte & 0x80) OLED_SDA_1();
        else OLED_SDA_0();
				
        OLED_SCL_1();
        OLED_Delay();
        byte <<=1;
        OLED_SCL_0();
        OLED_Delay();

        if(i==7) OLED_SDA_1();//����ʱSDA�ߵ�ƽ
        
    }
}

/**
 * @name: IIC_WaitAck
 * @description: CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
 * @return {uint8_t}����0��ʾ��ȷӦ��1��ʾ��������Ӧ
 */
static uint8_t IIC_WaitAck()
{
    //��̬SCLΪ0��SDA����ν��������SDA�����ͷ�SDA�����ٽ�SCL���ߣ���ȡSDAֵ��SDAΪ0ΪӦ�𣬷����Ӧ��
    uint8_t rec;

    OLED_SDA_1();
    OLED_SCL_1();
    OLED_Delay();
    if(OLED_READ()) rec=1;
    else rec=0;
    OLED_SCL_0();

    return rec;
}




/***************************��������1**********************************/

/**
 * @name: OLED_Check
 * @description: ���I2C�����豸��CPU�����豸��ַ��Ȼ���ȡ�豸Ӧ�����жϸ��豸�Ƿ����
 * @return {uint8_t}1:OK 0:ERROR
 */
static uint8_t OLED_Check()
{
    uint8_t re;
    IIC_Start();
    IIC_SendByte(OLED_ADDRESS);
    re=IIC_WaitAck();
    IIC_Stop();

    return !re;
} 



/**
 * @name: OLED_SendCmd
 * @description: ��OLED����һ�����������ʼ�ͽ����ź�
 * @param {uint8_t} cmd Ҫ���͵�����
 * @return {uint8_t} ���ͳɹ�����1:����ʧ�ܷ���0
 */
static uint8_t OLED_SendCmd(uint8_t cmd)
{
    IIC_Start();

    IIC_SendByte(OLED_ADDRESS);
    if(IIC_WaitAck()) return 0;
    IIC_SendByte(OLED_CMD);
    if(IIC_WaitAck()) return 0;//û���յ���Ӧ��ը��

    IIC_SendByte(cmd);
    if(IIC_WaitAck()) return 0;

    IIC_Stop();
    return 1;
}

/**
 * @name: OLED_SendData
 * @description: ��OLED����һ�����ݣ�������ʼ�ͽ����ź�
 * @param {uint8_t} data Ҫ���͵�����
 * @return {uint8_t}���ͳɹ�����1:����ʧ�ܷ���0
 */
static uint8_t OLED_SendData(uint8_t data)
{
    IIC_Start();

    IIC_SendByte(OLED_ADDRESS);
    if(IIC_WaitAck()) return 0;
    IIC_SendByte(OLED_DATA);
    if(IIC_WaitAck()) return 0;//û���յ���Ӧ��ը��

    IIC_SendByte(data);
    if(IIC_WaitAck()) return 0;

    IIC_Stop();
    return 1;
}


/***************************��������2**********************************/

/**
 * @name: OLED_SetPgaeAddress
 * @description: ����ΪҳѰַģʽ
 * @return {*}
 */
static void OLED_SetPgaeAddress()
{
    OLED_SendCmd(OLED_CMD_SetAddMode);
    OLED_SendCmd(OLED_CMD_AddressPage);
}

/**
 * @name: OLED_SetPos
 * @description: ������ʼλ�õĺ���,datasheet��˵ֻ��ҳѰַ�����ã���ʵ��������ģʽ������ʹ��
 * @param {uint8_t} x �е�ַ,0-127
 * @param {uint8_t} y ҳ��ַ,0-7
 * @return {*}NONE
 */
static void OLED_SetPos(uint8_t x,uint8_t y)
{
    OLED_SendCmd(OLED_CMD_SetPage+y);

    OLED_SendCmd((x & 0xF0) >> 4 | OLED_CMD_SetLineH);//�����и���λ
    OLED_SendCmd((x & 0x0F) | OLED_CMD_SetLineL);//�����е���λ
}


/**
 * @name: OLED_SetHoriAddress
 * @description: Ѱַģʽ�л�ΪˮƽѰַģʽ��Ĭ��ɨ�跶Χ,�з�Χ0-127,ҳ��Χ0-7
 * @return {*}
 */
static void OLED_SetHoriAddress()
{
    OLED_SendCmd(OLED_CMD_SetAddMode);
    OLED_SendCmd(OLED_CMD_AddressHori);
}

/**
 * @name: OLED_SetVertAddress
 * @description: Ѱַģʽ�л�Ϊ��ֱѰַģʽ��Ĭ��ɨ�跶Χ,�з�Χ0-127,ҳ��Χ0-7
 * @return {*}
 */
static void OLED_SetVertAddress()
{
    OLED_SendCmd(OLED_CMD_SetAddMode);
    OLED_SendCmd(OLED_CMD_AddressVert);
}

/**
 * @name: OLED_SetLineAndColu
 * @description: ˮƽɨ��ģʽ�ʹ�ֱɨ��ģʽ������ɨ����ʼҳ��ַ���е�ַ,�з�Χ0-127,ҳ��Χ0-7
 * @param {uint8_t} Scolu ��ʼ�е�ַ
 * @param {uint8_t} Ecolu �����е�ַ
 * @param {uint8_t} Sline ��ʼҳ��ַ
 * @param {uint8_t} Eline ����ҳ��ַ
 * @return {*}
 */
static void OLED_SetLineAndColu(uint8_t Scolu,uint8_t Ecolu,uint8_t Sline,uint8_t Eline)
{
    OLED_SendCmd(OLED_CMD_SetColuAddress);//������ʼ�к���ֹ��
    OLED_SendCmd(Scolu);OLED_SendCmd(Ecolu);
    OLED_SendCmd(OLED_CMD_SetPageAddress);//������ʼҳ����ֹҳ
    OLED_SendCmd(Sline);OLED_SendCmd(Eline);
}
/****************************�û�����1********************************/


uint8_t OLED_Init()
{

    OLED_SendCmd(OLED_CMD_StopShow);//�ر���ʾ

    OLED_SendCmd(OLED_CMD_SetMultiRatio);OLED_SendCmd(0x3F);//����ʹ��64��
    OLED_SendCmd(OLED_CMD_SetDisplayOffset);OLED_SendCmd(0x00);//���ô�ֱƫ��0��
    OLED_SendCmd(OLED_CMD_SetSegRemap_1);//�����ҷ�ת
    OLED_SendCmd(OLED_CMD_SetScanDirREM);//�����·�ת
    OLED_SendCmd(OLED_CMD_SetCOMPin);OLED_SendCmd(0x12);//COM���Ų�����
    OLED_SendCmd(OLED_CMD_SetDisplayStartLine);//������ʼ��ʾ��Ϊ0
    OLED_SendCmd(OLED_CMD_SetAddMode);OLED_SendCmd(OLED_CMD_AddressHori);//����ˮƽѰַģʽ

    OLED_SendCmd(OLED_CMD_SetContrast);OLED_SendCmd(0x7F);//����127���Աȶ�
    OLED_SendCmd(OLED_CMD_EntireDisplayOff);//�ر�ȫ��ʾ
    OLED_SendCmd(OLED_CMD_RollBackShowOff);//�رշ�ת��ʾ
    OLED_SendCmd(OLED_CMD_SetVcomh);OLED_SendCmd(0x30);// set Vcomh
    OLED_SendCmd(OLED_CMD_SetPreCharge);OLED_SendCmd(0xF1);// Set Pre-Charge Period
    OLED_SendCmd(OLED_CMD_SetClock);OLED_SendCmd(0x80);//��������Ƶ��ΪĬ��ֵ
    OLED_SendCmd(OLED_CMD_ChargePumpH);OLED_SendCmd(OLED_CMD_ChargePumpEN);//������ɱ�

    OLED_SendCmd(OLED_CMD_DISscroll);//ֹͣ����

    OLED_Clear();

    OLED_SendCmd(OLED_CMD_StartShow);//������ʾ

    return OLED_Check();
}


/**
 * @name: OLED_SetBright
 * @description: ������Ļ�Աȶ�
 * @param {uint8_t} lingt �Աȶ�0-256
 * @return {*}
 */
void OLED_SetBright(uint8_t lingt)
{
    OLED_SendCmd(OLED_CMD_SetContrast);//��������ָ��
    OLED_SendCmd(lingt);
}

/**
 * @name: OELD_StartShow
 * @description: ������ʾ
 * @return {*}
 */
void OELD_StartShow()
{
    OLED_SendCmd(OLED_CMD_StartShow);
}

/**
 * @name: OLED_StopShow
 * @description: �ر���ʾ
 * @return {*}
 */
void OLED_StopShow()
{
    OLED_SendCmd(OLED_CMD_StopShow);
}

/**
 * @name: OLED_ShowALL
 * @description: ����,ȫ������
 * @param {uint8_t} ch 1:����ȫ���� 0:�ر�ȫ����
 * @return {*}
 */
void OLED_ShowALL(uint8_t ch)
{
    if(ch==1) OLED_SendCmd(OLED_CMD_EntireDisplayOn);
    else OLED_SendCmd(OLED_CMD_EntireDisplayOff);
}

/**
 * @name: OLED_RollBack
 * @description: ��ת��ʾ
 * @return {*}
 */
void OLED_RollBack()
{
    static uint8_t now=0;
    if(now==0)
    {
        OLED_SendCmd(OLED_CMD_RollBackShowOn);
        now=1;
    }
    else 
    {
        OLED_SendCmd(OLED_CMD_RollBackShowOff);
        now=0;
    }
}

/**
 * @name: OLED_HoriChange
 * @description: �ı����ҷ�ת����,�������Ժ���ûɶ��
 * @return {*}
 */
void OLED_HoriChange()
{
    static uint8_t now=0;
    if(now==0)
    { 
        OLED_SendCmd(OLED_CMD_SetSegRemap_0);//���ҷ�ת
        now=1;
    }
    else 
    {
        OLED_SendCmd(OLED_CMD_SetSegRemap_1);//�ر�
        now=0;
    }
}

/**
 * @name: OLED_VertChange
 * @description: �ı䴹ֱ��ת��ʾ
 * @return {*}
 */
void OLED_VertChange()
{
    static uint8_t now=0;
    if(now==0)
    {
        OLED_SendCmd(OLED_CMD_SetScanDirNOR);//���·�ת
        now=1;
    }
    else{
        OLED_SendCmd(OLED_CMD_SetScanDirREM);
        now=0;
    }
}

/**
 * @name: OLED_Clear
 * @description: ��������
 * @return {*}
 */
void OLED_Clear()
{
    for(uint8_t i=0;i<8;i++)
        for(uint8_t j=0;j<128;j++)
        {
            OLED_SendData(0x00);
        }
    //ҳѰַģʽ
    // for(uint8_t i=0;i<8;i++)
    // {
    //     OLED_SendCmd(OLED_CMD_SetPage+i);
    //     OLED_SendCmd(OLED_CMD_SetLineL);
    //     OLED_SendCmd(OLED_CMD_SetLineH);
    //     for(uint8_t j=0;j<128;j++) OLED_SendData(0x00);
    // }
}

/**
 * @name: OLED_StopRoll()
 * @description: ֹͣ����
 * @return {*}
 */
void OLED_StopRoll()
{
    OLED_SendCmd(OLED_CMD_DISscroll);
}

/**
 * @name: OLED_StartHoriRoll
 * @description: ��ʼˮƽ����ָ�page:0-7,Spage����С��Epage
 * @param {OLED_ROLL_DIR} dir ��������
 * @param {OLED_ROLL_SPEED} speed �����ٶ�
 * @param {uint8_t} Spage ��ʼ����ҳ
 * @param {uint8_t} Epage ��������ҳ
 * @return {*}
 */
void OLED_StartHoriRoll(OLED_ROLL_DIR dir,OLED_ROLL_SPEED speed,uint8_t Spage,uint8_t Epage)
{
    
    if(dir==OLED_ROLL_LEFT) OLED_SendCmd(OLED_CMD_LeftRoll);
    else OLED_SendCmd(OLED_CMD_RightRoll);
    OLED_SendCmd(0x00);
    OLED_SendCmd(Spage);
    OLED_SendCmd(speed);
    OLED_SendCmd(Epage);
    OLED_SendCmd(0x00);
    OLED_SendCmd(0xFF);
    OLED_SendCmd(OLED_CMD_ENScroll);
}

/**
 * @name: OLED_StartHoriAndVertRoll
 * @description: ��ʼˮƽ�ʹ�ֱ����ָ�page:0-7,Spage����С��Epage��vert:0-63
 * @param {OLED_ROLL_DIR} dir ��������
 * @param {OLED_ROLL_SPEED} speed �����ٶ�
 * @param {uint8_t} Spage ��ʼ����ҳ
 * @param {uint8_t} Epage ��������ҳ
 * @param {uint8_t} vert ��ֱƫ����0-63,0Ϊ�޴�ֱ����
 * @return {*}
 */
void OLED_StartHoriAndVertRoll(OLED_ROLL_DIR dir,OLED_ROLL_SPEED speed,uint8_t Spage,uint8_t Epage,uint8_t vert)
{
    
    if(dir==OLED_ROLL_LEFT) OLED_SendCmd(OLED_CMD_VeLeftRoll);
    else OLED_SendCmd(OLED_CMD_VeRightRoll);
    OLED_SendCmd(0x00);
    OLED_SendCmd(Spage);
    OLED_SendCmd(speed);
    OLED_SendCmd(Epage);
    OLED_SendCmd(vert);
    OLED_SendCmd(OLED_CMD_ENScroll);
}

/**
 * @name: OLED_SetVertRollPage
 * @description: ���ô�ֱ��������,�����Ժ���ûɶ��
 * @param {uint8_t} Dpage ���ö����̶���������
 * @param {uint8_t} Rpage ����ĩβ������������
 * @return {*}
 */
void OLED_SetVertRollPage(uint8_t Dpage,uint8_t Rpage)
{
    OLED_SendCmd(OELD_CMD_SetVerollArea);
    OLED_SendCmd(Dpage);
    OLED_SendCmd(Rpage);
}


/***************************�û�����2**********************************/

/**
 * @name: OLED_ShowChar
 * @description: ��ָ��λ����ʾһ���ַ�
 * @param {uint8_t} x 0-127
 * @param {uint8_t} y 0-7
 * @param {uint8_t} chr Ҫ��ʾ���ַ�
 * @param {uint8_t} Size ����ѡ��-16/8
 * @param {uint8_t} mode 0,������ʾ;1,������ʾ
 * @return {*}NONE
 */
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Size,uint8_t mode)
{
    uint8_t c=0;

    if(chr>= ' ') c=chr-' ';//�õ�ƫ�ƺ��ֵ����ģ�е�һ��Ϊ�� ��
    else c=chr+16;

    if(x>127)//�������ޣ�ת����һ��
    {
        x=0;
        y=y+2;
    }
    if(Size==16)//8*16,�൱�ڻ�һ��8*16��ͼ
    {
        OLED_SetPos(x,y);
        for(uint8_t i=0;i<8;i++) OLED_SendData(mode==1 ? F8X16[c*16+i]:~F8X16[c*16+i]);
        OLED_SetPos(x,y+1);
        for(uint8_t i=0;i<8;i++) OLED_SendData(mode==1 ? F8X16[c*16+i+8]:~F8X16[c*16+i+8]);
    }
    else //6*8.�൱�ڻ�һ��6*8��ͼ
    {
        OLED_SetPos(x,y);
        for(uint8_t i=0;i<6;i++) OLED_SendData(mode==1 ? F6x8[c*6+i]:~F6x8[c*6+i]);
    }
}

/**
 * @name: OLED_ShowString
 * @description: ��ʾ�ַ���
 * @param {uint8_t} x 0-127
 * @param {uint8_t} y 0-7
 * @param {uint8_t*} chr Ҫ��ʾ���ַ���
 * @param {uint8_t} Size ����ѡ��-16/8
 * @param {uint8_t} mode 0,������ʾ;1,������ʾ
 * @return {*}
 */
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t* chr,uint8_t Size,uint8_t mode)
{
    uint8_t i=0;
    while (chr[i]!='\0')
    {
        OLED_ShowChar(x,y,chr[i],Size,mode);
        x+=8;
        if(x>120)//ת����һ��
        {
            x=0;
            y+= (Size==16 ? 2:1);
        }
        i++;
    }
    
}

/**
 * @name: OLED_Printf
 * @description: ����Ļ��ӡһ���ַ���
 * @param {uint8_t} x 0-127
 * @param {uint8_t} y 0-7
 * @param {uint8_t} Size ����ѡ��-16/8
 * @param {uint8_t} mode 0,������ʾ;1,������ʾ
 * @param {char} *__format
 * @return {*}
 */
void OLED_Printf(uint8_t x,uint8_t y,uint8_t Size,uint8_t mode,const char *__format, ...)
{
    va_list ap;
  va_start(ap, __format);
  
  /* ��շ��ͻ����� */
  memset(TxBuf, 0x0, TX_BUF_LEN);
  
  /* ��䷢�ͻ����� */
  vsnprintf((char*)TxBuf, TX_BUF_LEN, (const char *)__format, ap);
  va_end(ap);
  
  OLED_ShowString(x,y,TxBuf,Size,mode);
}

/**
 * @name: OLED_ShowChinese
 * @description: ��ʾһ��16x16����
 * @param {uint8_t} x 0-127
 * @param {uint8_t} y 0-7
 * @param {uint8_t} mode 0,������ʾ;1,������ʾ
 * @param {uint8_t} no ���ֱ��
 * @return {*}
 */
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t mode,uint8_t no)
{
    OLED_SetPos(x,y);
    for(uint8_t i=0;i<16;i++)
    {
        OLED_SendData(mode==1 ? Hzk[no*2][i]:~Hzk[no*2][i]);
    }
    OLED_SetPos(x,y+1);
    for(uint8_t i=0;i<16;i++)
    {
        OLED_SendData(mode==1 ? Hzk[no*2+1][i]:~Hzk[no*2+1][i]);
    }
}

/**
 * @name: OLED_ShowBmp
 * @description: ��ʾһ��ͼƬ��x:0-127,y:0-7
 * @param {unsigned char} x0 ��ʼ������
 * @param {unsigned char} y0 ��ʼ������
 * @param {unsigned char} x1 ĩβ������
 * @param {unsigned char} y1 ĩβ������
 * @param {unsigned char} BMP ����ͼƬ������
 * @return {*}
 */
void OLED_ShowBmp(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
    uint32_t i=0;
    uint8_t x,y;

    for(y=y0;y<=y1;y++)
    {
        OLED_SetPos(x0,y);
        for(x=x0;x<=x1;x++)
        {
            OLED_SendData(BMP[i++]);
        }
    }
}

uint8_t OLED_Test()
{
    static int now=0;
    //  if(now==0) now=1;
    //  else now=0;
    if(now==0){
    OLED_ShowALL(1);
    OLED_Clear();
    HAL_Delay(1000);
    OLED_ShowALL(0);
    
    OLED_SetPos(0,0);OLED_SendData(0xFF);OLED_SendData(0xFF);OLED_SendData(0xFF);OLED_SendData(0xFF);
    OLED_SetPos(0,7);OLED_SendData(0xFF);OLED_SendData(0xFF);OLED_SendData(0xFF);OLED_SendData(0xFF);
    OLED_SetPos(123,0);OLED_SendData(0xFF);OLED_SendData(0xFF);OLED_SendData(0xFF);OLED_SendData(0xFF);
    OLED_SetPos(123,7);OLED_SendData(0xFF);OLED_SendData(0xFF);OLED_SendData(0xFF);OLED_SendData(0xFF);


    OLED_ShowChar(86,3,'9',16,1);
    OLED_ShowString(10,0,"HELLO_World",16,1);
    OLED_Printf(10,2,16,0,"%d:%d",2,6);
    OLED_ShowChinese(0,5,1,0);OLED_ShowChinese(16,5,1,1);OLED_ShowChinese(32,5,1,2);OLED_ShowChinese(48,5,1,3);
    HAL_Delay(1000);
    OLED_ShowBmp(0,0,127,7,bmp);
    HAL_Delay(1000);
    OLED_StartHoriRoll(OLED_ROLL_RIGHT,OLED_ROLL_3,0,7);
    HAL_Delay(1000);
    OLED_StartHoriAndVertRoll(OLED_ROLL_RIGHT,OLED_ROLL_4,0,7,1);
    HAL_Delay(1000);
    OLED_StopRoll();
    OLED_ShowBmp(0,0,127,7,bmp);
    HAL_Delay(1000);
    OLED_RollBack();
    HAL_Delay(1000);
    OLED_RollBack();
    //OLED_HoriChange();
    HAL_Delay(1000);
    OLED_VertChange();
    HAL_Delay(1000);
    //OLED_HoriChange();
    OLED_VertChange();
    HAL_Delay(5000);
    for(int i=0;i<256;i++)
    {
        OLED_SetBright(i);
        HAL_Delay(100);
    }

    now=1;
    }
    
	return 1;
    
}

