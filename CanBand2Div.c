/*******************************************************************************

			                   CAN������Ϣ�ṹ-��غ���ʵ��
//�ṩ CAN2.0��֧��,ʹ�ô��ļ���ʹ�ú����ӿ�
*******************************************************************************/
#include "CanMsg.h"    

/*******************************************************************************
			                       ��غ����ӿ�ʵ��
*******************************************************************************/

//--------------------------- CANӲ���õ������ʲ��ұ�����----------------------------
static const unsigned long _CanBitTimeLUT[] = {
  /* [ 8] */  0x00002211, /*  0 | 2+1 |   2+1  |   1+1  |   1+1  |  8  | 75% */
  /* [ 9] */  0x00001122, /*  0 | 1+1 |   1+1  |   2+1  |   2+1  |  9  | 67% */
  /* [10] */  0x00002222, /*  0 | 2+1 |   2+1  |   2+1  |   2+1  | 10  | 70% */
  /* [11] */  0x00003322, /*  0 | 3+1 |   3+1  |   2+1  |   2+1  | 11  | 72% */
  /* [12] */  0x00002233, /*  0 | 2+1 |   2+1  |   3+1  |   3+1  | 12  | 67% */
  /* [13] */  0x00003333, /*  0 | 3+1 |   3+1  |   3+1  |   3+1  | 13  | 77% */
  /* [14] */  0x00003334, /*  0 | 3+1 |   3+1  |   3+1  |   4+1  | 14  | 64% */
  /* [15] */  0x00003344, /*  0 | 3+1 |   3+1  |   4+1  |   4+1  | 15  | 67% */
  /* [16] */  0x00003444, /*  0 | 3+1 |   4+1  |   4+1  |   4+1  | 16  | 69% */
  /* [17] */  0x00003544, /*  0 | 3+1 |   5+1  |   4+1  |   4+1  | 17  | 71% */
  /* [18] */  0x00003455, /*  0 | 3+1 |   4+1  |   5+1  |   5+1  | 18  | 67% */
  /* [19] */  0x00003555, /*  0 | 3+1 |   5+1  |   5+1  |   5+1  | 19  | 68% */
  /* [20] */  0x00003655, /*  0 | 3+1 |   6+1  |   5+1  |   5+1  | 20  | 70% */
  /* [21] */  0x00003755, /*  0 | 3+1 |   7+1  |   5+1  |   5+1  | 21  | 71% */
  /* [22] */  0x00003766, /*  0 | 3+1 |   7+1  |   6+1  |   6+1  | 22  | 68% */
  /* [23] */  0x00003776, /*  0 | 3+1 |   7+1  |   7+1  |   6+1  | 23  | 70% */
  /* [24] */  0x00003677, /*  0 | 3+1 |   6+1  |   7+1  |   7+1  | 24  | 67% */
  /* [25] */  0x00003777, /*  0 | 3+1 |   7+1  |   7+1  |   7+1  | 25  | 68% */
};

//--------------------------- CANӲ���õ������ʺ���----------------------------
//����ֵ����Ϊ:
//0-3bit: Phase2, ����λ�����2, 1~7
//4-7bit: Phase1, ����λ�����1, 1~7, ע���е�CAN��������λ�������Ǻ϶�Ϊһ��
//8-11bit: Prop,  ��������,1~7
//12-15bit: Sjw,  ����ͬ����ת���ȣ� 1~3
//16-22bit: div:  ��Ԥ��Ƶ��, 0~(��С��Ƶ�� - 1)
//CAN����ÿ��Bitʱ����: (1ͬ����+������+��λ�����1+��λ�����2����)
//������ = CanMck / ((Phase2 + Phase1 + Prop + 1) * div)
//�β�pBaudrate���뵱ǰ������,����ƥ���Ĳ�����,-1��ʾ<��С��Ƶ����Ƶ�ֲ�������
unsigned long CanBand2Div_Get(unsigned long CanMck,    //CANʱ��
                              unsigned char  MixDiv,    //��С��Ƶ��
                              unsigned long *pBaudrate)
{
  //��ʵ����DAP2010��ֵ��
  unsigned long Baudrate = *pBaudrate;
  //�ﵽ��С��������(��Ƶ�ֲ�������)
  if(((CanMck + (Baudrate * 25 - 1)) / (Baudrate * 25)) > MixDiv)
    return (unsigned long)-1;
  
  unsigned long Mod = 0xffffffff; //Ԥ��Ƶ������,��ӽ�ֵ��Ϊ0ʱ��ʾ����ƥ�䲨����
  
  unsigned char TQ = 8; //����СTQֵ������ƥ�䲨���ʵ����ԽС
  for(unsigned char i = 8; i <= 25; i++){
    if((CanMck / (Baudrate * i)) <= MixDiv){//����Ƶʱ
      unsigned long CurMod = CanMck % (Baudrate * i);
      if(CurMod < Mod){
        Mod = CurMod;
        TQ = i;
        if(!Mod) break;    //100%ƥ���趨������
      }
    }
  }
  unsigned long PreScale = CanMck / (Baudrate * TQ);
  *pBaudrate = CanMck / (PreScale * TQ);//����ʵ�ʵĲ�����
  return ((PreScale - 1) << 16) | _CanBitTimeLUT[TQ - 8];
}














