/*******************************************************************************

			                   CAN������Ϣ�ṹ-��غ���ʵ��
//�ṩ CAN2.0��֧��,ʹ�ô��ļ���ʹ�ú����ӿ�
*******************************************************************************/
#include "CanMsg.h"    

/*******************************************************************************
			                       ��غ����ӿ�ʵ��
*******************************************************************************/
//----------------------------------�õ���׼֡ID��-------------------------------
unsigned short CanMsg_GetStdId(const struct _CanMsg *pMsg)
{
  return ((unsigned short)(pMsg->IdType[1]) << 8) | pMsg->IdType[0];
}
//--------------------------------���ñ�׼֡ID��-------------------------------
void CanMsg_SetStdId(struct _CanMsg *pMsg, unsigned short StdId)
{
  pMsg->IdType[1] = (unsigned char)(StdId >> 8) & 0x07;
  pMsg->IdType[0] = (unsigned char)StdId;
}

//----------------------------------�õ���չ֡ID��-------------------------------
unsigned long CanMsg_GetExId(const struct _CanMsg *pMsg)
{
  unsigned long ExId = ((unsigned long)(pMsg->IdType[3] & 0x1F)) << 24;
  ExId |= (unsigned long)(pMsg->IdType[2]) << 16;  
  ExId |= (unsigned long)(pMsg->IdType[1]) << 8;   
  ExId |= (unsigned long)(pMsg->IdType[0]) << 0;  
  return ExId;
}
//----------------------------------������չ֡ID��-------------------------------
void CanMsg_SetExId(struct _CanMsg *pMsg, unsigned long ExId)
{
  pMsg->IdType[3] = (unsigned char)(ExId >> 24) & 0x1F;
  pMsg->IdType[2] = (unsigned char)(ExId >> 16) & 0xFF;
  pMsg->IdType[1] = (unsigned char)(ExId >> 8) & 0xFF;  
  pMsg->IdType[0] = (unsigned char)(ExId >> 0) & 0xFF;    
}
















