
#ifndef _ISMART_SPI_COMM_H_
#define _ISMART_SPI_COMM_H_

#define SPI_DEV "/dev/ttyUSB0"
#define SIZE_SPI_RECV_BUFF 1024
#define READ_FREQUENCY 10000 /*10����*/
#define WRITE_FREQUENCY 100000 /*100ms*/
#define READ_ERR_NUM 10 //����readʧ��10��,��Ϊ�豸�Ѿ��Ͽ�
#define ACCESS_DEV 3 //ÿ��1s̽���豸�Ƿ�����

typedef int (* CALLBACK_RECV_SPI_FUN)();

int SetCallbackRecvSerialFun(CALLBACK_RECV_SPI_FUN pRecvSerialFun);
int SetCallbackRecvSerialBuff(unsigned char *pRecvSerialBuff);
int SetCallbackRecvSerialBuffSize(unsigned int *pRecvSerialBuffSize);
int SendDataToSerial(char *pData, unsigned int nLen);
int SerialStart();
int SerialStop();
int SerialSetListenOff();
int SerialSetListenOn();
//int SerialInit();
//int SerialOpen();
//int SerialClose();
int InsertUSB(int dev_id);
int PullUSB(int dev_id);
int SetSerialPrint();
int SetSerialNoPrint();

#endif

