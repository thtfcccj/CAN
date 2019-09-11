/*******************************************************************************

			                    CAN�����豸�ײ�����ģ��
��ģ��ʵ��CAN���ߵ��շ����ݹ��ܣ�������Ӧ�����ú�Filter�����ڻص���ʶ��
��ģ���ݲ�֧��CAN����ʱ���������,�綨ʱ�������Զ�����������ʱ���
��ģ�������Ӳ������ʵ��Ӳ�����
*******************************************************************************/
#ifndef __CAN_DEV_H
#define __CAN_DEV_H	 

/*******************************************************************************
			                              �������
*******************************************************************************/

#ifndef CAN_DEV_FILTER_HW_COUNT   //����Ӳ��(����)�˲�������
  #define CAN_DEV_FILTER_HW_COUNT 14
#endif

/*******************************************************************************
			                              ��ؽṹ
*******************************************************************************/
#include "CanMsg.h"

//-------------------------------ͨ����������-------------------------------
//���ͨ������Ҫ�Խ�������,��pRcvMsgΪ��־:
//pRcvMsg = -1:  ��������,��ʱFilterIdΪ����Ϊ��
//               0: �������; ��:����; ��: ��ʾ������,������������롱����
//pRcvMsg = NULL:���������쳣
//����ʱ��FilterId������Filterʱ��Ӧ(0xff��Ч)��pRcvMsgΪNULLʱΪ�쳣����
typedef void (*CanDev_cbNotify_t)(unsigned char CanId,
                                   const struct _CanMsg *pRcvMsg,
                                   unsigned char FilterId);

//------------------------------���ṹ����-------------------------------
struct _CanDev{
  //Ӳ����ַ
  void *pHw; 
  CanDev_cbNotify_t cbNotify;//��̬�ص�������NULLʱ�þ�̬ʵ��
  unsigned char CanId;      //�豸�����ID��
  struct _CanMsg RcvMsgBuf; //���һ���յ����ݵĻ���  
};

/*******************************************************************************
			                      ���������
*******************************************************************************/
#define CAN_DEV_ERR_FINAL             0   //��ȷ,��0
#define CAN_DEV_ERR_ENTER_INIT        -1   //�����ʼ������
#define CAN_DEV_ERR_QUIT_INIT         -2   //�˳���ʼ������
#define CAN_DEV_ERR_BAUDRATE          -3   //���������ô���
#define CAN_DEV_ERR_BOX_0V            -4   //����ֵ����
#define CAN_DEV_ERR_BOX_FULL          -5   //����ռ����

/*******************************************************************************
			                        ��Ҫ����
*******************************************************************************/

//-------------------------------��ʼ������----------------------------------
//���ع�����
signed char CanDev_Init(struct _CanDev *pCan, void *pHw,
                         unsigned char CanId,      //�豸�����ID��
                         CanDev_cbNotify_t cbNotify);//ΪNULLʱʹ�þ�̬�ص�

//-------------------------------����ص�����----------------------------------
void CanDev_SetNotifyFun(struct _CanDev *pCan,
                         CanDev_cbNotify_t cbNotify);//ΪNULLʱʹ�þ�̬�ص�

//------------------------------���ò����ʺ���----------------------------------
//���ع�����
signed char CanDev_SetBaudrate(struct _CanDev *pCan,
                               unsigned long mClk,        //������ʱ��
                               unsigned long *pBaudrate);

//----------------------------------�������պ���--------------------------------
//���ú��˲��������ô˺���CAN��ʼ����
void CanDev_StartRcv(struct _CanDev *pCan);

//---------------------------------ֹͣ���պ���--------------------------------
//���ú��˲��������ô˺���CAN��ʼ����
void CanDev_StopRcv(struct _CanDev *pCan);

//----------------------------------������Ϣ����--------------------------------
//���ع�����,����ʾ���Ǹ����䷢��
signed char CanDev_SendMsg(struct _CanDev *pCan,
                            signed char MsgBoxId,  //�������Ǹ���������,����ʾ��Ҫ��
                            const struct _CanMsg *pMsg); //�跢�͵���Ϣ

//----------------------------�жϴ������ú���-------------------------------
//���ڶ�ӦCAN�жϴ��������������ݴ���
//InitFlag����Ϊ:
//6-7BitΪ�ж�����:
//0b00xxxxxxʱ�� Ϊ�����жϣ�xxxxxx���ڱ�ʾ���������
//0b01xxxxxxʱ�� Ϊ�����жϣ�xxxxxx��Ӳ������
//0b10xxxxxxʱ�� ΪCANϵͳ�жϣ�xxxxxx��Ӳ������
void  CanDev_IRQ(struct _CanDev *pCan,unsigned char IntFlag);

/*******************************************************************************
			                        �������˲������
��������CAN����ǰ�����ú��˲���
*******************************************************************************/
//-------------------------------�˲������˵��---------------------------------
//��CAN�˲������ڹ�����Ҫ��CAN ID�Խ�ʡMCU��Դ
//��ͨ����ȫ�˲�(���ʱͨ��)�������˲�(����λ��ͬ��ͨ��)�����ж��,�����ù�����ID�ű�ʾ
//����ڽ�������ID�������ID������������ʽ��
//  ��(�������)CAN�˲���ID������CAN��������ID����ʱʵ��ʱ��ָ�����˲����������Ǹ�������
//  ��(�������)ÿ��CAN���������ж���CAN�˲���ID����ʱʵ��ʱ�ֱ���
//�񲿷�CAN�˲���Ϊ��������ʣ��ڲ�ͬ����ģʽ�£�ͬһ�˲���������ֳ����4��ʹ��
//�������ṩ֧�ֲ�ͳһ���塣

//------------------------------���ڹ�����ID�Ŷ���-----------------------
//��ӦӲ��ID
#define CAN_DEV_FILATER_ID_HW_MASK      0x3f 
//ȫ�˲�ʱ,Ӳ���˲����ɳɶԲ��ʱ����ʾΪ��λ�˲���
#define CAN_DEV_FILATER_ID_EX_LIST      0x80
//��׼ģʽ11Bitʱ��Ӳ���˲���˫�ɳɶԲ��ʱ����ʾΪ����λ�˲���
#define CAN_DEV_FILATER_ID_EX_STD       0x40    
//����
#define CAN_DEV_FILATER_ID_EX_SHIFT     6   

//--------------------------���ò�������չ֡CAN����������-------------------------
//ע��ValidMask = 0xffffffffʱ����ʾ������ȫ�Ƚ�ģʽ
//���ع�����
signed char CanDev_SetExtFliter(struct _CanDev *pCan,
                                 unsigned char MsgBoxId, //�������Ǹ���������
                                 unsigned char FilterId,  //������ID��,��˵��
                                 unsigned long Identifier,//ʶ����
                                 unsigned long ValidMask);//��Чλ����

//--------------------------���ò�������׼֡CAN����������------------------------
//ע��ValidMask = 0xffffʱ����ʾ������ȫ�Ƚ�ģʽ
//���ع�����
signed char CanDev_SetStdFliter(struct _CanDev *pCan,
                                 unsigned char MsgBoxId, //�������ǽ�������
                                 unsigned char FilterId,  //������ID��
                                 unsigned short Identifier,//ʶ����
                                 unsigned short ValidMask);//��Чλ����

//--------------------------���ĳ���˲�������----------------------------------
//���ع�����
signed char CanDev_ClrFliter(struct _CanDev *pCan,
                             unsigned char MsgBoxId,  //�������Ǹ���������
                             unsigned char FilterId); //������ID��

/*******************************************************************************
			                         �ص�����
*******************************************************************************/

//---------------------------------CAN���߾�̬�ص�------------------------------
//û�ж��嶯̬�ص�����ʱ�����ô˺���
void CanDev_cbNotify(unsigned char CanId,
                     const struct _CanMsg *pRcvMsg,
                     unsigned char FilterId);

//---------------------------------��������-------------------------------------
//�����ڿ���ͨѶָʾ�Ƶ�
#include "IoCtrl.h" //ֱ��ʵ��:
#define CanDev_cbSendStartNotify(canId)     SetCanTxLight()   //�÷��͵���

//---------------------------------�������-------------------------------------
//�����ڿ���ͨѶָʾ�Ƶ�
#define CanDev_cbSendFinalNotify(canId)     SetCanTxLight()   //�÷��͵���

#endif
















