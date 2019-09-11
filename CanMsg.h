/*******************************************************************************

			                   CAN������Ϣ�ṹ����
//�ṩ CAN2.0��֧��,ʹ�ô��ļ���ʹ�ú����ӿ�
��ģ�������Ӳ��
*******************************************************************************/
#ifndef _CAN_MSG_H
#define	_CAN_MSG_H    

/*******************************************************************************
			                          ��ض���
*******************************************************************************/

#define CAN_MSG_DATA_LEN      8   //�̶�Ϊ8

/*******************************************************************************
			                          ��ؽṹ
*******************************************************************************/
struct _CanMsg{
  unsigned char IdType[4];     //CanId����Ϣ���ͣ�������
  unsigned char Data[CAN_MSG_DATA_LEN];      //����
  unsigned char LenCh;        //���ݳ�����(��ѡ)ͨ����ʶ��������
};

//IdType[4]����Ϊ:
//IdType[0]~IdType[1]�ڱ�׼֡�б�ʾ11λID��(MSB���У���λ��Ч)
//IdType[0]~IdType[3]����չ֡�б�ʾ29λID��(MSB���У���λ����)
//IdType[3]��Ӧλ����Ϊ:
#define CAN_MSG_IS_EXT        0x80        //�Ƿ�Ϊ��չ֡������Ϊ��׼֡
#define CAN_MSG_IS_REMOTE     0x40        //�Ƿ�ΪԶ��֡������Ϊ����֡

//DataLenCh����Ϊ��
#define CAN_MSG_LEN_MASK      0x0F        //���ݳ��ȣ����8
#define CAN_MSG_CH_SHIFT      4           //(������Ч)ͨ����ʶ,0-15;
#define CAN_MSG_CH_MASK       (15 << 4) 

/*******************************************************************************
			                          �����ӿ�
*******************************************************************************/

//-----------------------------��չ֡���׼֡----------------------------------
#define CanMsg_IsExtFrame(msg)  ((msg)->IdType[3] & CAN_MSG_IS_EXT)
#define CanMsg_IsStdFrame(msg)  (!CanMsg_IsExtFrame(msg))
#define CanMsg_SetExtFrame(msg) do{(msg)->IdType[3] |= CAN_MSG_IS_EXT;}while(0)
#define CanMsg_SetStdFrame(msg) do{(msg)->IdType[3] &= ~CAN_MSG_IS_EXT;}while(0)

//-----------------------------Զ��֡������֡----------------------------------
#define CanMsg_IsRemoteFrame(msg)  ((msg)->IdType[3] & CAN_MSG_IS_REMOTE)
#define CanMsg_IsDataFrame(msg)  (!CanMsg_IsRemoteFrame(msg))
#define CanMsg_SetRemoteFrame(msg) do{(msg)->IdType[3] |= CAN_MSG_IS_REMOTE;}while(0)
#define CanMsg_SetDataFrame(msg) do{(msg)->IdType[3] &= ~CAN_MSG_IS_REMOTE;}while(0)

//----------------------------------ID�Ų���------------------------------------
//��׼֡ʱ���11λ:
unsigned short CanMsg_GetStdId(const struct _CanMsg *pMsg);
void CanMsg_SetStdId(struct _CanMsg *pMsg, unsigned short StdId);
//��չ֡ʱ���29λ
unsigned long CanMsg_GetExId(const struct _CanMsg *pMsg);
void CanMsg_SetExId(struct _CanMsg *pMsg, unsigned long ExId);

//----------------------------------����������----------------------------------
//��ֱ�ӵ���pMsg->Data[n]���ж�д����,Ҳ��ʹ����������
//��������дָ��:
#define CanMsg_pGetData(msg) ((msg)->Data)
#define CanMsg_pGetDataN(msg,pos) (&(msg)->Data[pos])
//�����ݲ���:
#define CanMsg_GetDataN(msg, pos) ((msg)->Data[pos])
#define CanMsg_SetDataN(msg, pos, data) do {(msg)->Data[pos] = data;}while(0)

//-----------------------------------���Ȳ���-----------------------------------
#define CanMsg_GetLen(msg) ((msg)->LenCh & CAN_MSG_LEN_MASK)
#define CanMsg_SetLen(msg, len) do {(msg)->LenCh = \
      ((msg)->LenCh & CAN_MSG_CH_MASK) | (len & CAN_MSG_LEN_MASK);}while(0)

//------------------------------(������Ч)ͨ������--------------------------------
#define CanMsg_GetChMask(msg) (((msg)->LenCh & CAN_MSG_CH_MASK))  //������
#define CanMsg_GetCh(msg) (((msg)->LenCh & CAN_MSG_CH_MASK) >> CAN_MSG_CH_SHIFT)
#define CanMsg_SetCh(msg, ch) do {(msg)->LenCh = \
      ((msg)->LenCh & CAN_MSG_LEN_MASK) | (ch << CAN_MSG_CH_SHIFT);}while(0)



#endif

















