/*******************************************************************************

			      CAN总线底层驱动模块-在BxCAN IP核(主要为STM32)中的实现
  //bxCAN的过滤器参考：https://blog.csdn.net/flydream0/article/details/52317532
*******************************************************************************/

#include "CanDev.h"
#include "CanBand2Div.h"
#include <string.h>

/*******************************************************************************
			                      内部函数
*******************************************************************************/

//-------------------------------进入配置模式----------------------------------
//返回0成功，负失败
static signed char _EnterCfg(CAN_TypeDef *pCanHw)
{
	pCanHw->MCR |= CAN_MCR_INRQ;		  //请求CAN进入初始化模式
  for(unsigned char i = 100; i > 0; i--){
    if(pCanHw->MSR & CAN_MCR_INRQ) return 0;
  }
  return CAN_DEV_ERR_ENTER_INIT;
}

//------------------------------退出配置模式----------------------------------
//返回0成功，负失败
static signed char _ExitCfg(CAN_TypeDef *pCanHw)
{
	pCanHw->MCR &=~ CAN_MCR_INRQ;		//请求CAN退出初始化模式
  for(unsigned short i = 0XFFF0; i > 0; i--){
    if(!(pCanHw->MSR & CAN_MCR_INRQ)) return 0;
  }
  return CAN_DEV_ERR_QUIT_INIT;
}

/*******************************************************************************
			                        主要函数实现
*******************************************************************************/

//-------------------------------初始化函数----------------------------------
//返回0成功，负失败
signed char CanDev_Init(struct _CanDev *pCan, void *pHw,
                         unsigned char CanId,      //设备分配的ID号
                         CanDev_cbNotify_t cbNotify)//为NULL时使用静态回调
{
  memset(pCan, 0, sizeof(struct _CanDev));
  pCan->pHw = pHw;
  pCan->CanId = CanId;  
  if(cbNotify == NULL) pCan->cbNotify = CanDev_cbNotify;
  else pCan->cbNotify = cbNotify;  
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pHw;
  
	pCanHw->MCR = 0;		     //退出睡眠模式(同时设置所有位为0)  
  signed char Resume = _EnterCfg(pCanHw);
  if(Resume) return Resume;
  
	//pCanHw->MCR &= ~CAN_MCR_TTCM;		  //非时间触发通信模式(时间触发暂不支持)
	//pCanHw->MCR &= ~CAN_MCR_ABOM;		  //软件自动离线管理
	//pCanHw->MCR &= ~CAN_MCR_AWUM;		 //睡眠模式通过软件唤醒(清除pCanHw->MCR的SLEEP位)
	pCanHw->MCR |= CAN_MCR_NART;		   //禁止报文自动传送
	//pCanHw->MCR &= ~CAN_MCR_RFLM;    //报文不锁定,新的覆盖旧的
	//pCanHw->MCR &= ~CAN_MCR_TXFP;	   //优先级由报文标识符决定
  
  Resume = _ExitCfg(pCanHw);
  return Resume;
}

//-------------------------------重设回调函数----------------------------------
void CanDev_SetNotifyFun(struct _CanDev *pCan,
                         CanDev_cbNotify_t cbNotify)//为NULL时使用静态回调 
{
  if(cbNotify == NULL) pCan->cbNotify = CanDev_cbNotify;
  else pCan->cbNotify = cbNotify;
}
                        
//------------------------------设置波特率函数----------------------------------
//返回故障码
signed char CanDev_SetBaudrate(struct _CanDev *pCan,
                               unsigned long mClk,        //主工作时钟
                               unsigned long *pBaudrate)
{
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
  signed char Resume = _EnterCfg(pCanHw);
  if(Resume) return Resume;
  
  unsigned long BaudCfg = CanBand2Div_Get(mClk, 1, pBaudrate);
  if(BaudCfg == (signed long )-1) return CAN_DEV_ERR_BAUDRATE;//异常。
  
	pCanHw->BTR = 0;	      //清除原来的设置.
	//pCanHw->BTR |= CAN_BTR_LBKM;	  //模式设置 0,普通模式;1,回环模式;
  unsigned long Tsjw = (BaudCfg >> 12) & 0x03 - 1;
  pCanHw->BTR |= Tsjw << 24; 	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  
  unsigned long Tbs2 = (BaudCfg >> 8) & 0x0f - 1;//即Prop
	pCanHw->BTR|= Tbs2 << 20;
  unsigned long Tbs1 = (BaudCfg >> 4) & 0x0f + //即Phase2 + Phase1, 1-16
                        (BaudCfg & 0x0f) - 1;
	pCanHw->BTR |= Tbs1 << 16;
	pCanHw->BTR|= (BaudCfg >> 16);  //分频系数(Fdiv)已-1了
  
  return _ExitCfg(pCanHw);  
}		 

//----------------------------------启动接收函数--------------------------------
//配置好滤波器后后调用此函数CAN开始工作
void CanDev_StartRcv(struct _CanDev *pCan)
{
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
	pCanHw->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1;		//FIFO消息挂号中断允许
}

//---------------------------------停止接收函数--------------------------------
//配置好滤波器后后调用此函数CAN开始工作
void CanDev_StopRcv(struct _CanDev *pCan)
{
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
	pCanHw->IER &= ~(CAN_IER_FMPIE0 | CAN_IER_FMPIE1);	//FIFO消息挂号中断禁止  
}

//----------------------------------发送消息函数--------------------------------
//返回故障码,正表示在那个邮箱发出
signed char CanDev_SendMsg(struct _CanDev *pCan,
                            signed char MsgBoxId, //作用于那个发送邮箱,负表示无要求
                            const struct _CanMsg *pMsg)    //消息
{
  if(MsgBoxId > 2) return CAN_DEV_ERR_BOX_0V;  //超限
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
  
	if(MsgBoxId < 0){//选择邮箱
    if(pCanHw->TSR & CAN_TSR_TME0) MsgBoxId = 0;		      //邮箱0为空
    else if(pCanHw->TSR & CAN_TSR_TME1) MsgBoxId = 1;		  //邮箱1为空
    else if(pCanHw->TSR & CAN_TSR_TME2) MsgBoxId = 2;		  //邮箱1为空
    else return CAN_DEV_ERR_BOX_FULL; //满了
  }
	else{//固定邮箱
     if(!(pCanHw->TSR &( 1<< (MsgBoxId + CAN_TSR_TME0))))
       return CAN_DEV_ERR_BOX_FULL; //满了
  }
	pCanHw->sTxMailBox[MsgBoxId].TIR = 0;		  //清除之前的设置
  
  //设置ID号
  unsigned long Id;
  if(CanMsg_IsStdFrame(pMsg)){//标准帧
    Id = CanMsg_GetStdId(pMsg);
    Id <<= 21;
  }//扩展帧
  else{
    Id = CanMsg_GetStdId(pMsg);
    Id <<= 3;
  }
	pCanHw->sTxMailBox[MsgBoxId].TIR |= Id;		 
  //设置DLC
	pCanHw->sTxMailBox[MsgBoxId].TDTR &= ~CAN_TDT0R_DLC;
  unsigned char Len = CanMsg_GetLen(pMsg);
	pCanHw->sTxMailBox[MsgBoxId].TDTR |= Len;  
  //设置数据
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
	pCanHw->sTxMailBox[MsgBoxId].TIR|= CAN_TI0R_TXRQ; //请求发送邮箱数据
	pCanHw->IER |= CAN_IER_TMEIE;//发送完后中断
  
	return MsgBoxId;
}

/*******************************************************************************
			                        接收消息与中断
*******************************************************************************/ 

//---------------------------------接收消息函数--------------------------------
//返回FMI值
static unsigned char _RcvMsg(const CAN_TypeDef *pCanHw,
                             unsigned char MsgBoxId,//即Fifo，0或1
                             struct _CanMsg *pMsg)    //消息
{
  //邮箱通道
  CanMsg_SetCh(pMsg, MsgBoxId); 
  unsigned long Data = pCanHw->sFIFOMailBox[MsgBoxId].RIR;
  //远程或数据帧
  if(Data & 0x01) CanMsg_SetRemoteFrame(pMsg);
  else  CanMsg_SetDataFrame(pMsg);
  //ID与标准或扩展帧
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
  //数据长度与数据
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

//------------------------------将FMI转换为FilterId-------------------------
//0xff无效
unsigned char _Fmi2FilterId(const CAN_TypeDef *pCanHw,
                             unsigned char MsgBoxId,//即Fifo，0或1
                             unsigned char Fmi)
{
  //查询每个硬件
  unsigned char HwFilter = 0;
  unsigned long IdMask = 1 << 0;
  unsigned char CurFmi = 0;  
  for(; HwFilter < CAN_DEV_FILTER_HW_COUNT; HwFilter++,IdMask <<= 1){
    //检查邮箱是否合的上。
    if(pCanHw->FFA1R  &IdMask){		//过滤器n关联到FIFO1
      if(!MsgBoxId) continue; //0合不上
    }
    else{//过滤器n关联到FIFO0
      if(MsgBoxId) continue; //1合不上
    }
    unsigned char Add;
    if(pCanHw->FM1R & IdMask)//过滤器n工作在标识符列表模式加倍
      Add = 2;
    else Add = 1;
    if(!(pCanHw->FS1R & IdMask))//过滤器位宽为16位加倍
      Add *= 2;
    if((CurFmi + Add) >= Fmi){//找到了
      return HwFilter | (Add << CAN_DEV_FILATER_ID_EX_SHIFT);
    }
  }
  return 0xff; //没找到无效
}

//----------------------------中断处理调用函数-------------------------------
//放在对应CAN中断处理程序中以数据处理
//InitFlag定义为:
//6-7Bit为中断类型:
//0b00xxxxxx时： 为接收中断，xxxxxx可于表示接由邮箱号
//0b01xxxxxx时： 为发送中断，xxxxxx由硬件决定
//0b10xxxxxx时： 为CAN系统中断，xxxxxx由硬件决定
void  CanDev_IRQ(struct _CanDev *pCan,unsigned char IntFlag)
{
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
  
  //==============================接收中断处理==============================
  if(IntFlag == 0x00){
    while(pCanHw->RF0R & CAN_RF0R_FMP0){//检查FIFO0:
      unsigned char Fmi = _RcvMsg(pCanHw, 0, &pCan->RcvMsgBuf);
      pCan->cbNotify(pCan->CanId, &pCan->RcvMsgBuf, _Fmi2FilterId(pCanHw, 0, Fmi));
      pCanHw->RF0R |= CAN_RF0R_RFOM0;//释放本次读取的FIFO0邮箱以清除CAN_RF0R_FMP0
    } 
    return;
  }
  if(IntFlag == 0x01){  
    while(pCanHw->RF1R & CAN_RF1R_FMP1){//检查FIFO1:
      unsigned char Fmi = _RcvMsg(pCanHw, 0, &pCan->RcvMsgBuf);
      pCan->cbNotify(pCan->CanId, &pCan->RcvMsgBuf, _Fmi2FilterId(pCanHw, 1, Fmi));
      pCanHw->RF1R |= CAN_RF1R_RFOM1;//释放本次读取的FIFO0邮箱以清除CAN_RF1R_FMP1
    } 
    return;
  }
  //=============================系统中断处理==============================
  if(IntFlag & 0x80){
    
    
    return;
  }
  //=============================发送中断处理==============================  
  if(IntFlag & 0x40){
    CanDev_cbSendFinalNotify(pCan->CanId); //发送完成
    pCanHw->IER &= ~CAN_IER_TMEIE;//发送完关中断
    pCan->cbNotify(pCan->CanId, (struct _CanMsg *)(-1), 0);//通报发送完成
    return;
  }
}


/*******************************************************************************
			                        接收用滤波器相关实现
*******************************************************************************/ 

//------------------------------设置扩展帧CAN过滤器函数----------------------------
//注：ValidMask = 0xffffffff时，表示工作在全比较模式
//返回故障码
signed char CanDev_SetExtFliter(struct _CanDev *pCan,
                                 unsigned char MsgBoxId, //作用于那个接收邮箱
                                 unsigned char FilterId,  //过滤器ID号,见说明
                                 unsigned long Identifier,//识别器
                                 unsigned long ValidMask) //有效位掩码
{
  //说明：bxCan支持全滤波时,滤波器支持双倍模式
  //在全比较模式下配置为列表模式,此时pCanHw->sFilterRegister.FR1与FR2功能相同
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;

  unsigned char HwFilter = FilterId & 0x1f;    //硬件滤波器
  if(HwFilter >= CAN_DEV_FILTER_HW_COUNT) return CAN_DEV_ERR_BOX_0V; //超限
  unsigned long IdMask = 1 << HwFilter;

  signed char Resume = _EnterCfg(pCanHw);
  if(Resume) return Resume;
  
	pCanHw->FMR |= 1 << 0;		    //过滤器组工作在初始化模式 //FINT位
	pCanHw->FA1R &= ~IdMask;	  //过滤器n先关闭，即不激活才能配置
  if(MsgBoxId) pCanHw->FFA1R |= IdMask;		//过滤器n关联到FIFO1
  else pCanHw->FFA1R &= ~IdMask;		//过滤器n关联到FIFO0  
	pCanHw->FS1R |= IdMask; 		//因扩展模式为29位，故过滤器位宽为32位
  if(ValidMask == 0xffffffff){//列表模式时
    pCanHw->FM1R |= IdMask;       //过滤器n工作在标识符列表模式
    if(FilterId & CAN_DEV_FILATER_ID_EX_LIST)//配置第二组
      pCanHw->sFilterRegister[HwFilter].FR2 = Identifier;
    else//配置第一组
      pCanHw->sFilterRegister[HwFilter].FR1 = Identifier;
  }
  else{
    pCanHw->FM1R &= ~IdMask;		//过滤器n工作在标识符屏蔽位模式
	  pCanHw->sFilterRegister[HwFilter].FR1 = Identifier;//识别器
    pCanHw->sFilterRegister[HwFilter].FR2 = ValidMask; //掩码
	}
	pCanHw->FA1R |= IdMask;		      //激活过滤器n
	pCanHw->FMR &= ~(1<<0);		      //过滤器组进入正常模式
  
  return _ExitCfg(pCanHw); 
}  

//------------------------------设置标准帧CAN过滤器函数----------------------------
//注：ValidMask = 0xffff时，表示工作在全比较模式
//返回故障码
signed char CanDev_SetStdFliter(struct _CanDev *pCan,
                                 unsigned char MsgBoxId, //作用于那个接收邮箱
                                 unsigned char FilterId,  //过滤器ID号
                                 unsigned short Identifier,//识别器
                                 unsigned short ValidMask) //有效位掩码
{
  //说明：bxCan支持全滤波时,滤波器支持双倍模式,又支持标准模式时16位过滤器位宽加倍
  //在标准模式下使用16位过滤器位宽,则每个pCanHw->sFilterRegister.FRx允许为两个;
  //在全比较模式下配置为列表模式时,此时pCanHw->sFilterRegister.FR1与FR2功能相同
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
  unsigned char HwFilter = FilterId & 0x1f;    //硬件滤波器
  if(HwFilter >= CAN_DEV_FILTER_HW_COUNT) return CAN_DEV_ERR_BOX_0V; //超限
  unsigned long IdMask = 1 << HwFilter;

  signed char Resume = _EnterCfg(pCanHw);
  if(Resume) return Resume;
  
	pCanHw->FMR |= 1 << 0;		    //过滤器组工作在初始化模式 //FINT位
	pCanHw->FA1R &= ~IdMask;	  //过滤器n先关闭，即不激活才能配置
  if(MsgBoxId) pCanHw->FFA1R |= IdMask;		//过滤器n关联到FIFO1
  else pCanHw->FFA1R &= ~IdMask;		//过滤器n关联到FIFO0  
	pCanHw->FS1R &= ~IdMask; 		//因扩展模式为11位，故过滤器位宽为16位
  
  unsigned long Mask;
  unsigned char Shift;
  if(FilterId & CAN_DEV_FILATER_ID_EX_STD){ //高16Bit
    Mask = 0xffff0000;
    Shift = 8;
  }
  else{
    Mask = 0x0000ffff;
    Shift = 0;
  }
  if(ValidMask == 0xffff){//列表模式时
    pCanHw->FM1R |= IdMask;       //过滤器n工作在标识符列表模式
    if(FilterId & CAN_DEV_FILATER_ID_EX_LIST){//配置第2组
      pCanHw->sFilterRegister[HwFilter].FR2 &= ~Mask;
      pCanHw->sFilterRegister[HwFilter].FR2 |= (unsigned long)Identifier << Shift;       
    }
    else{//配置第一组
      pCanHw->sFilterRegister[HwFilter].FR1 &= ~Mask;
      pCanHw->sFilterRegister[HwFilter].FR1 |= (unsigned long)Identifier << Shift;   
    }
  }
  else{
    pCanHw->FM1R &= ~IdMask;		//过滤器n工作在标识符屏蔽位模式
    pCanHw->sFilterRegister[HwFilter].FR1 &= ~Mask;
	  pCanHw->sFilterRegister[HwFilter].FR1 = (unsigned long)Identifier << Shift;//识别器
    pCanHw->sFilterRegister[HwFilter].FR2 &= ~Mask;
    pCanHw->sFilterRegister[HwFilter].FR2 = (unsigned long)ValidMask << Shift; //掩码
	}
	pCanHw->FA1R |= IdMask;		      //激活过滤器n
	pCanHw->FMR &= ~(1<<0);		      //过滤器组进入正常模式
  
  return _ExitCfg(pCanHw); 
} 

//--------------------------清除某个滤波器函数----------------------------------
//返回故障码
signed char CanDev_ClrFliter(struct _CanDev *pCan,
                             unsigned char MsgBoxId,  //作用于那个接收邮箱
                             unsigned char FilterId)  //过滤器ID号
{
  CAN_TypeDef *pCanHw = (CAN_TypeDef *)pCan->pHw;
  unsigned char HwFilter = FilterId & 0x1f;    //硬件滤波器
  if(HwFilter >= CAN_DEV_FILTER_HW_COUNT) return CAN_DEV_ERR_BOX_0V; //超限
  unsigned long IdMask = 1 << HwFilter;

  signed char Resume = _EnterCfg(pCanHw);
  if(Resume) return Resume;
  
	pCanHw->FA1R &= ~IdMask;	  //过滤器n关闭
  
  return _ExitCfg(pCanHw); 
}













