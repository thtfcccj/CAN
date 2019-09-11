/*******************************************************************************

			      CAN���ߵײ�����ģ��-��BxCAN IP��(��ҪΪSTM32)�е�ʵ��
  //bxCAN�Ĺ������ο���https://blog.csdn.net/flydream0/article/details/52317532
*******************************************************************************/

#include "CanDev.h"
#include "CanBand2Div.h"
#include <string.h>

/*******************************************************************************
			                      �ڲ�����
*******************************************************************************/

//-------------------------------��������ģʽ----------------------------------
//����0�ɹ�����ʧ��
static signed char _EnterCfg(CAN_TypeDef *pCanHw)
{
	pCanHw->MCR |= CAN_MCR_INRQ;		  //����CAN�����ʼ��ģʽ
  for(unsigned char i = 100; i > 0; i--){
    if(pCanHw->MSR & CAN_MCR_INRQ) return 0;
  }
  return CAN_DEV_ERR_ENTER_INIT;
}

//------------------------------�˳�����ģʽ----------------------------------
//����0�ɹ�����ʧ��
static signed char _ExitCfg(CAN_TypeDef *pCanHw)
{
	pCanHw->MCR &=~ CAN_MCR_INRQ;		//����CAN�˳���ʼ��ģʽ
  for(unsigned short i = 0XFFF0; i > 0; i--){
    if(!(pCanHw->MSR & CAN_MCR_INRQ)) return 0;
  }
  return CAN_DEV_ERR_QUIT_INIT;
}

/*******************************************************************************
			                        ��Ҫ����ʵ��
*******************************************************************************/

//-------------------------------��ʼ������----------------------------------
//����0�ɹ�����ʧ��
signed char CanDev_Init(struct _CanDev *pCan, void *pHw,
                         unsigned char CanId,      //�豸�����ID��
                         CanDev_cbNotify_t cbNotify)//ΪNULLʱʹ�þ�̬�ص�
{
  memset(pCan, 0, sizeof(struct _CanDev));
  pCan->pHw = pHw;
  pCan->CanId = CanId;  
  if(cbNotify == NULL) pCan->cbNotify = CanDev_cbNotify;
  else pCan->cbNotify = cbNotify;  
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pHw;
  
	pCanHw->MCR = 0;		     //�˳�˯��ģʽ(ͬʱ��������λΪ0)  
  signed char Resume = _EnterCfg(pCanHw);
  if(Resume) return Resume;
  
	//pCanHw->MCR &= ~CAN_MCR_TTCM;		  //��ʱ�䴥��ͨ��ģʽ(ʱ�䴥���ݲ�֧��)
	//pCanHw->MCR &= ~CAN_MCR_ABOM;		  //����Զ����߹���
	//pCanHw->MCR &= ~CAN_MCR_AWUM;		 //˯��ģʽͨ���������(���pCanHw->MCR��SLEEPλ)
	pCanHw->MCR |= CAN_MCR_NART;		   //��ֹ�����Զ�����
	//pCanHw->MCR &= ~CAN_MCR_RFLM;    //���Ĳ�����,�µĸ��Ǿɵ�
	//pCanHw->MCR &= ~CAN_MCR_TXFP;	   //���ȼ��ɱ��ı�ʶ������
  
  Resume = _ExitCfg(pCanHw);
  return Resume;
}

//-------------------------------����ص�����----------------------------------
void CanDev_SetNotifyFun(struct _CanDev *pCan,
                         CanDev_cbNotify_t cbNotify)//ΪNULLʱʹ�þ�̬�ص� 
{
  if(cbNotify == NULL) pCan->cbNotify = CanDev_cbNotify;
  else pCan->cbNotify = cbNotify;
}
                        
//------------------------------���ò����ʺ���----------------------------------
//���ع�����
signed char CanDev_SetBaudrate(struct _CanDev *pCan,
                               unsigned long mClk,        //������ʱ��
                               unsigned long *pBaudrate)
{
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
  signed char Resume = _EnterCfg(pCanHw);
  if(Resume) return Resume;
  
  unsigned long BaudCfg = CanBand2Div_Get(mClk, 1, pBaudrate);
  if(BaudCfg == (signed long )-1) return CAN_DEV_ERR_BAUDRATE;//�쳣��
  
	pCanHw->BTR = 0;	      //���ԭ��������.
	//pCanHw->BTR |= CAN_BTR_LBKM;	  //ģʽ���� 0,��ͨģʽ;1,�ػ�ģʽ;
  unsigned long Tsjw = (BaudCfg >> 12) & 0x03 - 1;
  pCanHw->BTR |= Tsjw << 24; 	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  
  unsigned long Tbs2 = (BaudCfg >> 8) & 0x0f - 1;//��Prop
	pCanHw->BTR|= Tbs2 << 20;
  unsigned long Tbs1 = (BaudCfg >> 4) & 0x0f + //��Phase2 + Phase1, 1-16
                        (BaudCfg & 0x0f) - 1;
	pCanHw->BTR |= Tbs1 << 16;
	pCanHw->BTR|= (BaudCfg >> 16);  //��Ƶϵ��(Fdiv)��-1��
  
  return _ExitCfg(pCanHw);  
}		 

//----------------------------------�������պ���--------------------------------
//���ú��˲��������ô˺���CAN��ʼ����
void CanDev_StartRcv(struct _CanDev *pCan)
{
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
	pCanHw->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1;		//FIFO��Ϣ�Һ��ж�����
}

//---------------------------------ֹͣ���պ���--------------------------------
//���ú��˲��������ô˺���CAN��ʼ����
void CanDev_StopRcv(struct _CanDev *pCan)
{
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
	pCanHw->IER &= ~(CAN_IER_FMPIE0 | CAN_IER_FMPIE1);	//FIFO��Ϣ�Һ��жϽ�ֹ  
}

//----------------------------------������Ϣ����--------------------------------
//���ع�����,����ʾ���Ǹ����䷢��
signed char CanDev_SendMsg(struct _CanDev *pCan,
                            signed char MsgBoxId, //�������Ǹ���������,����ʾ��Ҫ��
                            const struct _CanMsg *pMsg)    //��Ϣ
{
  if(MsgBoxId > 2) return CAN_DEV_ERR_BOX_0V;  //����
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
  
	if(MsgBoxId < 0){//ѡ������
    if(pCanHw->TSR & CAN_TSR_TME0) MsgBoxId = 0;		      //����0Ϊ��
    else if(pCanHw->TSR & CAN_TSR_TME1) MsgBoxId = 1;		  //����1Ϊ��
    else if(pCanHw->TSR & CAN_TSR_TME2) MsgBoxId = 2;		  //����1Ϊ��
    else return CAN_DEV_ERR_BOX_FULL; //����
  }
	else{//�̶�����
     if(!(pCanHw->TSR &( 1<< (MsgBoxId + CAN_TSR_TME0))))
       return CAN_DEV_ERR_BOX_FULL; //����
  }
	pCanHw->sTxMailBox[MsgBoxId].TIR = 0;		  //���֮ǰ������
  
  //����ID��
  unsigned long Id;
  if(CanMsg_IsStdFrame(pMsg)){//��׼֡
    Id = CanMsg_GetStdId(pMsg);
    Id <<= 21;
  }//��չ֡
  else{
    Id = CanMsg_GetStdId(pMsg);
    Id <<= 3;
  }
	pCanHw->sTxMailBox[MsgBoxId].TIR |= Id;		 
  //����DLC
	pCanHw->sTxMailBox[MsgBoxId].TDTR &= ~CAN_TDT0R_DLC;
  unsigned char Len = CanMsg_GetLen(pMsg);
	pCanHw->sTxMailBox[MsgBoxId].TDTR |= Len;  
  //��������
  if(Len >= 4){
    pCanHw->sTxMailBox[MsgBoxId].TDHR = 
      ((unsigned long)CanMsg_GetDataN(pMsg, 7) << 24) |
			((unsigned long)CanMsg_GetDataN(pMsg, 6) << 18) |
 			((unsigned long)CanMsg_GetDataN(pMsg, 5) << 8) |
			((unsigned long)CanMsg_GetDataN(pMsg, 4) << 0);  
  }
  pCanHw->sTxMailBox[MsgBoxId].TDLR = 
    ((unsigned long)CanMsg_GetDataN(pMsg, 3) << 24) |
	((unsigned long)CanMsg_GetDataN(pMsg, 2) << 18) |
 	((unsigned long)CanMsg_GetDataN(pMsg, 1) << 8) |
	((unsigned long)CanMsg_GetDataN(pMsg, 0) << 0);
  
  CanDev_cbSendStartNotify(pCan->CanId);  
	pCanHw->sTxMailBox[MsgBoxId].TIR|= CAN_TI0R_TXRQ; //��������������
	pCanHw->IER |= CAN_IER_TMEIE;//��������ж�
  
	return MsgBoxId;
}

/*******************************************************************************
			                        ������Ϣ���ж�
*******************************************************************************/ 

//---------------------------------������Ϣ����--------------------------------
//����FMIֵ
static unsigned char _RcvMsg(const CAN_TypeDef *pCanHw,
                             unsigned char MsgBoxId,//��Fifo��0��1
                             struct _CanMsg *pMsg)    //��Ϣ
{
  //����ͨ��
  CanMsg_SetCh(pMsg, MsgBoxId); 
  unsigned long Data = pCanHw->sFIFOMailBox[MsgBoxId].RIR;
  //Զ�̻�����֡
  if(Data & 0x01) CanMsg_SetRemoteFrame(pMsg);
  else  CanMsg_SetDataFrame(pMsg);
  //ID���׼����չ֡
  if(Data & 0x02){
    CanMsg_SetExtFrame(pMsg);
    CanMsg_SetExId(pMsg, Data >> 3);
  }
  else{
    CanMsg_SetStdFrame(pMsg);
    CanMsg_SetStdId(pMsg, Data >> 21);
  }
  Data = pCanHw->sFIFOMailBox[MsgBoxId].RDTR;
  unsigned char Len = Data & 0x0f;
  unsigned char Fmi = (Data >> 8) & 0xff;
  //���ݳ���������
  CanMsg_SetLen(pMsg, Len);  
  if(Len > 4){
    Data = pCanHw->sFIFOMailBox[MsgBoxId].RDHR;
    CanMsg_SetDataN(pMsg, 4, (Data >> 24) & 0xff);
    CanMsg_SetDataN(pMsg, 4, (Data >> 16) & 0xff);
    CanMsg_SetDataN(pMsg, 4, (Data >> 8) & 0xff);
    CanMsg_SetDataN(pMsg, 4, (Data >> 0) & 0xff);
  }
  Data = pCanHw->sFIFOMailBox[MsgBoxId].RDLR;
  CanMsg_SetDataN(pMsg, 4, (Data >> 24) & 0xff);
  CanMsg_SetDataN(pMsg, 4, (Data >> 16) & 0xff);
  CanMsg_SetDataN(pMsg, 4, (Data >> 8) & 0xff);
  CanMsg_SetDataN(pMsg, 4, (Data >> 0) & 0xff);
  
  return Fmi;
}

//------------------------------��FMIת��ΪFilterId-------------------------
//0xff��Ч
unsigned char _Fmi2FilterId(const CAN_TypeDef *pCanHw,
                             unsigned char MsgBoxId,//��Fifo��0��1
                             unsigned char Fmi)
{
  //��ѯÿ��Ӳ��
  unsigned char HwFilter = 0;
  unsigned long IdMask = 1 << 0;
  unsigned char CurFmi = 0;  
  for(; HwFilter < CAN_DEV_FILTER_HW_COUNT; HwFilter++,IdMask <<= 1){
    //��������Ƿ�ϵ��ϡ�
    if(pCanHw->FFA1R  &IdMask){		//������n������FIFO1
      if(!MsgBoxId) continue; //0�ϲ���
    }
    else{//������n������FIFO0
      if(MsgBoxId) continue; //1�ϲ���
    }
    unsigned char Add;
    if(pCanHw->FM1R & IdMask)//������n�����ڱ�ʶ���б�ģʽ�ӱ�
      Add = 2;
    else Add = 1;
    if(!(pCanHw->FS1R & IdMask))//������λ��Ϊ16λ�ӱ�
      Add *= 2;
    if((CurFmi + Add) >= Fmi){//�ҵ���
      return HwFilter | (Add << CAN_DEV_FILATER_ID_EX_SHIFT);
    }
  }
  return 0xff; //û�ҵ���Ч
}

//----------------------------�жϴ�����ú���-------------------------------
//���ڶ�ӦCAN�жϴ�������������ݴ���
//InitFlag����Ϊ:
//6-7BitΪ�ж�����:
//0b00xxxxxxʱ�� Ϊ�����жϣ�xxxxxx���ڱ�ʾ���������
//0b01xxxxxxʱ�� Ϊ�����жϣ�xxxxxx��Ӳ������
//0b10xxxxxxʱ�� ΪCANϵͳ�жϣ�xxxxxx��Ӳ������
void  CanDev_IRQ(struct _CanDev *pCan,unsigned char IntFlag)
{
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
  
  //==============================�����жϴ���==============================
  if(IntFlag == 0x00){
    while(pCanHw->RF0R & CAN_RF0R_FMP0){//���FIFO0:
      unsigned char Fmi = _RcvMsg(pCanHw, 0, &pCan->RcvMsgBuf);
      pCan->cbNotify(pCan->CanId, &pCan->RcvMsgBuf, _Fmi2FilterId(pCanHw, 0, Fmi));
      pCanHw->RF0R |= CAN_RF0R_RFOM0;//�ͷű��ζ�ȡ��FIFO0���������CAN_RF0R_FMP0
    } 
    return;
  }
  if(IntFlag == 0x01){  
    while(pCanHw->RF1R & CAN_RF1R_FMP1){//���FIFO1:
      unsigned char Fmi = _RcvMsg(pCanHw, 0, &pCan->RcvMsgBuf);
      pCan->cbNotify(pCan->CanId, &pCan->RcvMsgBuf, _Fmi2FilterId(pCanHw, 1, Fmi));
      pCanHw->RF1R |= CAN_RF1R_RFOM1;//�ͷű��ζ�ȡ��FIFO0���������CAN_RF1R_FMP1
    } 
    return;
  }
  //=============================ϵͳ�жϴ���==============================
  if(IntFlag & 0x80){
    
    
    return;
  }
  //=============================�����жϴ���==============================  
  if(IntFlag & 0x40){
    CanDev_cbSendFinalNotify(pCan->CanId); //�������
    pCanHw->IER &= ~CAN_IER_TMEIE;//��������ж�
    pCan->cbNotify(pCan->CanId, (struct _CanMsg *)(-1), 0);//ͨ���������
    return;
  }
}


/*******************************************************************************
			                        �������˲������ʵ��
*******************************************************************************/ 

//------------------------------������չ֡CAN����������----------------------------
//ע��ValidMask = 0xffffffffʱ����ʾ������ȫ�Ƚ�ģʽ
//���ع�����
signed char CanDev_SetExtFliter(struct _CanDev *pCan,
                                 unsigned char MsgBoxId, //�������Ǹ���������
                                 unsigned char FilterId,  //������ID��,��˵��
                                 unsigned long Identifier,//ʶ����
                                 unsigned long ValidMask) //��Чλ����
{
  //˵����bxCan֧��ȫ�˲�ʱ,�˲���֧��˫��ģʽ
  //��ȫ�Ƚ�ģʽ������Ϊ�б�ģʽ,��ʱpCanHw->sFilterRegister.FR1��FR2������ͬ
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;

  unsigned char HwFilter = FilterId & 0x1f;    //Ӳ���˲���
  if(HwFilter >= CAN_DEV_FILTER_HW_COUNT) return CAN_DEV_ERR_BOX_0V; //����
  unsigned long IdMask = 1 << HwFilter;

  signed char Resume = _EnterCfg(pCanHw);
  if(Resume) return Resume;
  
	pCanHw->FMR |= 1 << 0;		    //�������鹤���ڳ�ʼ��ģʽ //FINTλ
	pCanHw->FA1R &= ~IdMask;	  //������n�ȹرգ����������������
  if(MsgBoxId) pCanHw->FFA1R |= IdMask;		//������n������FIFO1
  else pCanHw->FFA1R &= ~IdMask;		//������n������FIFO0  
	pCanHw->FS1R |= IdMask; 		//����չģʽΪ29λ���ʹ�����λ��Ϊ32λ
  if(ValidMask == 0xffffffff){//�б�ģʽʱ
    pCanHw->FM1R |= IdMask;       //������n�����ڱ�ʶ���б�ģʽ
    if(FilterId & CAN_DEV_FILATER_ID_EX_LIST)//���õڶ���
      pCanHw->sFilterRegister[HwFilter].FR2 = Identifier;
    else//���õ�һ��
      pCanHw->sFilterRegister[HwFilter].FR1 = Identifier;
  }
  else{
    pCanHw->FM1R &= ~IdMask;		//������n�����ڱ�ʶ������λģʽ
	  pCanHw->sFilterRegister[HwFilter].FR1 = Identifier;//ʶ����
    pCanHw->sFilterRegister[HwFilter].FR2 = ValidMask; //����
	}
	pCanHw->FA1R |= IdMask;		      //���������n
	pCanHw->FMR &= ~(1<<0);		      //���������������ģʽ
  
  return _ExitCfg(pCanHw); 
}  

//------------------------------���ñ�׼֡CAN����������----------------------------
//ע��ValidMask = 0xffffʱ����ʾ������ȫ�Ƚ�ģʽ
//���ع�����
signed char CanDev_SetStdFliter(struct _CanDev *pCan,
                                 unsigned char MsgBoxId, //�������Ǹ���������
                                 unsigned char FilterId,  //������ID��
                                 unsigned short Identifier,//ʶ����
                                 unsigned short ValidMask) //��Чλ����
{
  //˵����bxCan֧��ȫ�˲�ʱ,�˲���֧��˫��ģʽ,��֧�ֱ�׼ģʽʱ16λ������λ��ӱ�
  //�ڱ�׼ģʽ��ʹ��16λ������λ��,��ÿ��pCanHw->sFilterRegister.FRx����Ϊ����;
  //��ȫ�Ƚ�ģʽ������Ϊ�б�ģʽʱ,��ʱpCanHw->sFilterRegister.FR1��FR2������ͬ
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
  unsigned char HwFilter = FilterId & 0x1f;    //Ӳ���˲���
  if(HwFilter >= CAN_DEV_FILTER_HW_COUNT) return CAN_DEV_ERR_BOX_0V; //����
  unsigned long IdMask = 1 << HwFilter;

  signed char Resume = _EnterCfg(pCanHw);
  if(Resume) return Resume;
  
	pCanHw->FMR |= 1 << 0;		    //�������鹤���ڳ�ʼ��ģʽ //FINTλ
	pCanHw->FA1R &= ~IdMask;	  //������n�ȹرգ����������������
  if(MsgBoxId) pCanHw->FFA1R |= IdMask;		//������n������FIFO1
  else pCanHw->FFA1R &= ~IdMask;		//������n������FIFO0  
	pCanHw->FS1R &= ~IdMask; 		//����չģʽΪ11λ���ʹ�����λ��Ϊ16λ
  
  unsigned long Mask;
  unsigned char Shift;
  if(FilterId & CAN_DEV_FILATER_ID_EX_STD){ //��16Bit
    Mask = 0xffff0000;
    Shift = 8;
  }
  else{
    Mask = 0x0000ffff;
    Shift = 0;
  }
  if(ValidMask == 0xffff){//�б�ģʽʱ
    pCanHw->FM1R |= IdMask;       //������n�����ڱ�ʶ���б�ģʽ
    if(FilterId & CAN_DEV_FILATER_ID_EX_LIST){//���õ�2��
      pCanHw->sFilterRegister[HwFilter].FR2 &= ~Mask;
      pCanHw->sFilterRegister[HwFilter].FR2 |= (unsigned long)Identifier << Shift;       
    }
    else{//���õ�һ��
      pCanHw->sFilterRegister[HwFilter].FR1 &= ~Mask;
      pCanHw->sFilterRegister[HwFilter].FR1 |= (unsigned long)Identifier << Shift;   
    }
  }
  else{
    pCanHw->FM1R &= ~IdMask;		//������n�����ڱ�ʶ������λģʽ
    pCanHw->sFilterRegister[HwFilter].FR1 &= ~Mask;
	  pCanHw->sFilterRegister[HwFilter].FR1 = (unsigned long)Identifier << Shift;//ʶ����
    pCanHw->sFilterRegister[HwFilter].FR2 &= ~Mask;
    pCanHw->sFilterRegister[HwFilter].FR2 = (unsigned long)ValidMask << Shift; //����
	}
	pCanHw->FA1R |= IdMask;		      //���������n
	pCanHw->FMR &= ~(1<<0);		      //���������������ģʽ
  
  return _ExitCfg(pCanHw); 
} 

//--------------------------���ĳ���˲�������----------------------------------
//���ع�����
signed char CanDev_ClrFliter(struct _CanDev *pCan,
                             unsigned char MsgBoxId,  //�������Ǹ���������
                             unsigned char FilterId)  //������ID��
{
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
  unsigned char HwFilter = FilterId & 0x1f;    //Ӳ���˲���
  if(HwFilter >= CAN_DEV_FILTER_HW_COUNT) return CAN_DEV_ERR_BOX_0V; //����
  unsigned long IdMask = 1 << HwFilter;

  signed char Resume = _EnterCfg(pCanHw);
  if(Resume) return Resume;
  
	pCanHw->FA1R &= ~IdMask;	  //������n�ر�
  
  return _ExitCfg(pCanHw); 
}













