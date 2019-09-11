/*******************************************************************************

			                   CAN总线消息结构定义
//提供 CAN2.0版支持,使用此文件请使用函数接口
此模块独立于硬件
*******************************************************************************/
#ifndef _CAN_MSG_H
#define	_CAN_MSG_H    

/*******************************************************************************
			                          相关定义
*******************************************************************************/

#define CAN_MSG_DATA_LEN      8   //固定为8

/*******************************************************************************
			                          相关结构
*******************************************************************************/
struct _CanMsg{
  unsigned char IdType[4];     //CanId与消息类型，见定义
  unsigned char Data[CAN_MSG_DATA_LEN];      //数据
  unsigned char LenCh;        //数据长度与(可选)通道标识，见定义
};

//IdType[4]定义为:
//IdType[0]~IdType[1]在标准帧中表示11位ID号(MSB排列，高位无效)
//IdType[0]~IdType[3]在扩展帧中表示29位ID号(MSB排列，高位它用)
//IdType[3]对应位定义为:
#define CAN_MSG_IS_EXT        0x80        //是否为扩展帧，否则为标准帧
#define CAN_MSG_IS_REMOTE     0x40        //是否为远程帧，否则为数据帧

//DataLenCh定义为：
#define CAN_MSG_LEN_MASK      0x0F        //数据长度，最大8
#define CAN_MSG_CH_SHIFT      4           //(接收有效)通道标识,0-15;
#define CAN_MSG_CH_MASK       (15 << 4) 

/*******************************************************************************
			                          函数接口
*******************************************************************************/

//-----------------------------扩展帧与标准帧----------------------------------
#define CanMsg_IsExtFrame(msg)  ((msg)->IdType[3] & CAN_MSG_IS_EXT)
#define CanMsg_IsStdFrame(msg)  (!CanMsg_IsExtFrame(msg))
#define CanMsg_SetExtFrame(msg) do{(msg)->IdType[3] |= CAN_MSG_IS_EXT;}while(0)
#define CanMsg_SetStdFrame(msg) do{(msg)->IdType[3] &= ~CAN_MSG_IS_EXT;}while(0)

//-----------------------------远程帧与数据帧----------------------------------
#define CanMsg_IsRemoteFrame(msg)  ((msg)->IdType[3] & CAN_MSG_IS_REMOTE)
#define CanMsg_IsDataFrame(msg)  (!CanMsg_IsRemoteFrame(msg))
#define CanMsg_SetRemoteFrame(msg) do{(msg)->IdType[3] |= CAN_MSG_IS_REMOTE;}while(0)
#define CanMsg_SetDataFrame(msg) do{(msg)->IdType[3] &= ~CAN_MSG_IS_REMOTE;}while(0)

//----------------------------------ID号操作------------------------------------
//标准帧时输出11位:
unsigned short CanMsg_GetStdId(const struct _CanMsg *pMsg);
void CanMsg_SetStdId(struct _CanMsg *pMsg, unsigned short StdId);
//扩展帧时输出29位
unsigned long CanMsg_GetExId(const struct _CanMsg *pMsg);
void CanMsg_SetExId(struct _CanMsg *pMsg, unsigned long ExId);

//----------------------------------数据区操作----------------------------------
//可直接调用pMsg->Data[n]进行读写操作,也可使用下述函数
//缓冲区读写指针:
#define CanMsg_pGetData(msg) ((msg)->Data)
#define CanMsg_pGetDataN(msg,pos) (&(msg)->Data[pos])
//单数据操作:
#define CanMsg_GetDataN(msg, pos) ((msg)->Data[pos])
#define CanMsg_SetDataN(msg, pos, data) do {(msg)->Data[pos] = data;}while(0)

//-----------------------------------长度操作-----------------------------------
#define CanMsg_GetLen(msg) ((msg)->LenCh & CAN_MSG_LEN_MASK)
#define CanMsg_SetLen(msg, len) do {(msg)->LenCh = \
      ((msg)->LenCh & CAN_MSG_CH_MASK) | (len & CAN_MSG_LEN_MASK);}while(0)

//------------------------------(接收有效)通道操作--------------------------------
#define CanMsg_GetChMask(msg) (((msg)->LenCh & CAN_MSG_CH_MASK))  //带掩码
#define CanMsg_GetCh(msg) (((msg)->LenCh & CAN_MSG_CH_MASK) >> CAN_MSG_CH_SHIFT)
#define CanMsg_SetCh(msg, ch) do {(msg)->LenCh = \
      ((msg)->LenCh & CAN_MSG_LEN_MASK) | (ch << CAN_MSG_CH_SHIFT);}while(0)



#endif

















