/*******************************************************************************

			                   CAN总线消息结构-相关函数实现
//提供 CAN2.0版支持,使用此文件请使用函数接口
*******************************************************************************/
#include "CanMsg.h"    

/*******************************************************************************
			                       相关函数接口实现
*******************************************************************************/
//----------------------------------得到标准帧ID号-------------------------------
unsigned short CanMsg_GetStdId(const struct _CanMsg *pMsg)
{
  return ((unsigned short)(pMsg->IdType[1]) << 8) | pMsg->IdType[0];
}
//--------------------------------设置标准帧ID号-------------------------------
void CanMsg_SetStdId(struct _CanMsg *pMsg, unsigned short StdId)
{
  pMsg->IdType[1] = (unsigned char)(StdId >> 8) & 0x07;
  pMsg->IdType[0] = (unsigned char)StdId;
}

//----------------------------------得到扩展帧ID号-------------------------------
unsigned long CanMsg_GetExId(const struct _CanMsg *pMsg)
{
  unsigned long ExId = ((unsigned long)(pMsg->IdType[3] & 0x1F)) << 24;
  ExId |= (unsigned long)(pMsg->IdType[2]) << 16;  
  ExId |= (unsigned long)(pMsg->IdType[1]) << 8;   
  ExId |= (unsigned long)(pMsg->IdType[0]) << 0;  
  return ExId;
}
//----------------------------------设置扩展帧ID号-------------------------------
void CanMsg_SetExId(struct _CanMsg *pMsg, unsigned long ExId)
{
  pMsg->IdType[3] = (unsigned char)(ExId >> 24) & 0x1F;
  pMsg->IdType[2] = (unsigned char)(ExId >> 16) & 0xFF;
  pMsg->IdType[1] = (unsigned char)(ExId >> 8) & 0xFF;  
  pMsg->IdType[0] = (unsigned char)(ExId >> 0) & 0xFF;    
}
















