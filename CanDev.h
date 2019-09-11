/*******************************************************************************

			                    CAN总线设备底层驱动模块
此模块实现CAN总线的收发数据功能，收数据应先配置好Filter，以在回调里识别
此模块暂不支持CAN总线时间关联特性,如定时触发，自动触发，接收时间等
此模块独立于硬件，但实现硬件相关
*******************************************************************************/
#ifndef __CAN_DEV_H
#define __CAN_DEV_H	 

/*******************************************************************************
			                              相关配置
*******************************************************************************/

#ifndef CAN_DEV_FILTER_HW_COUNT   //定义硬件(接收)滤波器总数
  #define CAN_DEV_FILTER_HW_COUNT 14
#endif

/*******************************************************************************
			                              相关结构
*******************************************************************************/
#include "CanMsg.h"

//-------------------------------通报函数定义-------------------------------
//相关通报，主要以接收数主,以pRcvMsg为标志:
//pRcvMsg = -1:  其它功能,此时FilterId为符号为：
//               0: 发送完成; 正:待定; 负: 表示故障码,见“定义故障码”部分
//pRcvMsg = NULL:接收数据异常
//接收时：FilterId与设置Filter时对应(0xff无效)，pRcvMsg为NULL时为异常数据
typedef void (*CanDev_cbNotify_t)(unsigned char CanId,
                                   const struct _CanMsg *pRcvMsg,
                                   unsigned char FilterId);

//------------------------------主结构定义-------------------------------
struct _CanDev{
  //硬件基址
  void *pHw; 
  CanDev_cbNotify_t cbNotify;//动态回调函数，NULL时用静态实现
  unsigned char CanId;      //设备分配的ID号
  struct _CanMsg RcvMsgBuf; //最后一次收到数据的缓冲  
};

/*******************************************************************************
			                      定义故障码
*******************************************************************************/
#define CAN_DEV_ERR_FINAL             0   //正确,即0
#define CAN_DEV_ERR_ENTER_INIT        -1   //进入初始化错误
#define CAN_DEV_ERR_QUIT_INIT         -2   //退出初始化错误
#define CAN_DEV_ERR_BAUDRATE          -3   //波特率设置错误
#define CAN_DEV_ERR_BOX_0V            -4   //邮箱值超限
#define CAN_DEV_ERR_BOX_FULL          -5   //邮箱占用中

/*******************************************************************************
			                        主要函数
*******************************************************************************/

//-------------------------------初始化函数----------------------------------
//返回故障码
signed char CanDev_Init(struct _CanDev *pCan, void *pHw,
                         unsigned char CanId,      //设备分配的ID号
                         CanDev_cbNotify_t cbNotify);//为NULL时使用静态回调

//-------------------------------重设回调函数----------------------------------
void CanDev_SetNotifyFun(struct _CanDev *pCan,
                         CanDev_cbNotify_t cbNotify);//为NULL时使用静态回调

//------------------------------设置波特率函数----------------------------------
//返回故障码
signed char CanDev_SetBaudrate(struct _CanDev *pCan,
                               unsigned long mClk,        //主工作时钟
                               unsigned long *pBaudrate);

//----------------------------------启动接收函数--------------------------------
//配置好滤波器后后调用此函数CAN开始工作
void CanDev_StartRcv(struct _CanDev *pCan);

//---------------------------------停止接收函数--------------------------------
//配置好滤波器后后调用此函数CAN开始工作
void CanDev_StopRcv(struct _CanDev *pCan);

//----------------------------------发送消息函数--------------------------------
//返回故障码,正表示在那个邮箱发出
signed char CanDev_SendMsg(struct _CanDev *pCan,
                            signed char MsgBoxId,  //作用于那个发送邮箱,负表示无要求
                            const struct _CanMsg *pMsg); //需发送的消息

//----------------------------中断处理调用函数-------------------------------
//放在对应CAN中断处理程序中以数据处理
//InitFlag定义为:
//6-7Bit为中断类型:
//0b00xxxxxx时： 为接收中断，xxxxxx可于表示接由邮箱号
//0b01xxxxxx时： 为发送中断，xxxxxx由硬件决定
//0b10xxxxxx时： 为CAN系统中断，xxxxxx由硬件决定
void  CanDev_IRQ(struct _CanDev *pCan,unsigned char IntFlag);

/*******************************************************************************
			                        接收用滤波器相关
请在启动CAN总线前，设置好滤波器
*******************************************************************************/
//-------------------------------滤波器相关说明---------------------------------
//●CAN滤波器用于过滤需要的CAN ID以节省MCU资源
//●通常有全滤波(相等时通过)与掩码滤波(掩码位相同即通过)，且有多个,这里用过滤器ID号表示
//●关于接收邮箱ID与过滤器ID，存在两种形式：
//  ■(多数情况)CAN滤波器ID独立于CAN接收邮箱ID，此时实现时需指定此滤波器作于在那个邮箱上
//  ■(少数情况)每个CAN接收邮箱有独立CAN滤波器ID，此时实现时分别处理
//●部分CAN滤波器为提高利用率，在不同工作模式下，同一滤波器允许拆分成最多4个使用
//●这里提供支持并统一定义。

//------------------------------关于过滤器ID号定义-----------------------
//对应硬件ID
#define CAN_DEV_FILATER_ID_HW_MASK      0x3f 
//全滤波时,硬件滤波器可成对拆分时，表示为高位滤波器
#define CAN_DEV_FILATER_ID_EX_LIST      0x80
//标准模式11Bit时，硬件滤波器双可成对拆分时，表示为更高位滤波器
#define CAN_DEV_FILATER_ID_EX_STD       0x40    
//掩码
#define CAN_DEV_FILATER_ID_EX_SHIFT     6   

//--------------------------设置并启动扩展帧CAN过滤器函数-------------------------
//注：ValidMask = 0xffffffff时，表示工作在全比较模式
//返回故障码
signed char CanDev_SetExtFliter(struct _CanDev *pCan,
                                 unsigned char MsgBoxId, //作用于那个接收邮箱
                                 unsigned char FilterId,  //过滤器ID号,见说明
                                 unsigned long Identifier,//识别器
                                 unsigned long ValidMask);//有效位掩码

//--------------------------设置并启动标准帧CAN过滤器函数------------------------
//注：ValidMask = 0xffff时，表示工作在全比较模式
//返回故障码
signed char CanDev_SetStdFliter(struct _CanDev *pCan,
                                 unsigned char MsgBoxId, //作用于那接收邮箱
                                 unsigned char FilterId,  //过滤器ID号
                                 unsigned short Identifier,//识别器
                                 unsigned short ValidMask);//有效位掩码

//--------------------------清除某个滤波器函数----------------------------------
//返回故障码
signed char CanDev_ClrFliter(struct _CanDev *pCan,
                             unsigned char MsgBoxId,  //作用于那个接收邮箱
                             unsigned char FilterId); //过滤器ID号

/*******************************************************************************
			                         回调函数
*******************************************************************************/

//---------------------------------CAN总线静态回调------------------------------
//没有定义动态回调函数时将调用此函数
void CanDev_cbNotify(unsigned char CanId,
                     const struct _CanMsg *pRcvMsg,
                     unsigned char FilterId);

//---------------------------------启动发送-------------------------------------
//可用于控制通讯指示灯等
#include "IoCtrl.h" //直接实现:
#define CanDev_cbSendStartNotify(canId)     SetCanTxLight()   //置发送灯亮

//---------------------------------发送完成-------------------------------------
//可用于控制通讯指示灯等
#define CanDev_cbSendFinalNotify(canId)     SetCanTxLight()   //置发送灯灭

#endif

















