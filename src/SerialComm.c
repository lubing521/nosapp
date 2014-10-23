
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>
#include <nc_ipc.h>
#include <sys/types.h>
#include <sys/time.h>
#include "Common.h"
#include "Queue.h"
#include "SerialComm.h"
#include "Protocol.h"


static int g_hSerialDevFd = 0;
static CALLBACK_RECV_SPI_FUN g_pRecvSerialFun = NULL;
static pthread_t g_ptListen = 0;
static unsigned char *g_pRecvSerialBuff = NULL;
static unsigned int *g_pRecvSerialBuffSize = NULL;
//static pthread_mutex_t g_hRdWrMutex = PTHREAD_MUTEX_INITIALIZER;
static int g_PrintSirial = 0; /*串口通信帧,16进制打印*/
static Queue *g_WriteQueue = NULL;
//static int g_nReadErr = 0;
static pthread_t g_ptWriteSerial = 0;
//struct timeval tvs2;
//struct timeval tve2;
static int g_devid;
static int g_stateMachine;
static int g_SerialStart = 0;

int ReadChar(unsigned char *pChar, int offset, int nLen)
{
	if (1 >= g_stateMachine || 0 == g_hSerialDevFd || nLen <= 0 || nLen > PACKAGE_SIZE_ALL)
	{
		return ReturnError;
	}
	/*read 只有一个线程执行，不需要锁*/
	int nRetRead = 0;
	fd_set readfds;
	struct timeval timeout={0,0};
	int ret = 0;
	while(0 == ret && 0 != g_hSerialDevFd)
	{
		FD_ZERO(&readfds);
		FD_SET(g_hSerialDevFd, &readfds);
		TRACE("g_hSerialDevFd:%d\n", g_hSerialDevFd);
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;
		int maxfd = g_hSerialDevFd + 1;
		ret = select(maxfd, &readfds, NULL, NULL, &timeout);
//		perror("select");
		TRACE("select:%d\n", ret);
		if (0 == ret)
		{
			continue;
		}
		else if(-1 == ret)
		{
			break;
		}
		else
		{
			if(FD_ISSET(g_hSerialDevFd,&readfds))
			{
				nRetRead = usb_read(g_hSerialDevFd, (char *)pChar, offset, nLen, 0);
				TRACE("usb_read:%d\n", nRetRead);
				break;
			}
		}
	}
	return nRetRead;
}

static void* pthreadListen (void* arg)
{
	pthread_detach(pthread_self());
	unsigned int nNum = 0;
	while(1 == g_SerialStart)
	{
		if (1 >= g_stateMachine)
		{
			TRACE("g_stateMachine:%d\n", g_stateMachine);
			sleep(1);
		}
		TRACE("g_stateMachine:%d\n", g_stateMachine);
		int nRetRead = ReadChar(g_pRecvSerialBuff, 0, 4);
		if (0 >= nRetRead)
		{
			continue;
		}
		if (g_pRecvSerialBuff[0] != PACKAGE_HEDAH || g_pRecvSerialBuff[1] != PACKAGE_HEDAL)
		{
			continue;
		}
		unsigned int nLen = ntohs(*(uint16_t *)(g_pRecvSerialBuff + 2));
		if (nLen > PACKAGE_SIZE_ALL || nLen < PACKAGE_SIZE_HEAD)
		{
			continue;
		}
		nRetRead = ReadChar(g_pRecvSerialBuff, 4, nLen - 4);
		if (0 >= nRetRead)
		{
			continue;
		}
		TRACE("%u Read a frame\n", nNum);
		nNum++;
		*g_pRecvSerialBuffSize = nLen;
		if (g_PrintSirial)
		{
			TRACE("read serial####\n");
			PrintDataBy16(g_pRecvSerialBuff, nLen);
		}
		(*g_pRecvSerialFun)();

	}
	return NULL;
}

static void* pthreadWriteSerial (void* arg)
{
	unsigned int nNumWr = 0;
	PQueueNode pnode = {0};
	while (1 == g_SerialStart)
	{
		pnode = GetFront(g_WriteQueue);
		if (pnode)
		{
			//if (g_hSerialDevFd > 0)
			if (g_stateMachine >= 2)
			{
				TRACE("%u write serial####%d\n", nNumWr, pnode->nlen);
				nNumWr++;
				if (g_PrintSirial)
				{
					PrintDataBy16((unsigned char *)pnode->data, pnode->nlen);
				}
				int retwrite = usb_write(g_hSerialDevFd, pnode->data, 0, pnode->nlen);
				TRACE("usb_write:%d\n", retwrite);
				//tcflush(g_hSerialDevFd, TCOFLUSH);
				/*出队列,入队列,不能同时执行*/
				//pthread_mutex_lock(&g_hRdWrMutex);
				DeQueue(g_WriteQueue);
				//pthread_mutex_unlock(&g_hRdWrMutex);
				TRACE("DeQueue !!!:%d\n", GetSize(g_WriteQueue));
			}
		}
		/*30~50ms内不允许再次write,cc2530处理能力有限*/
		usleep(50000);
	}
	return NULL;
}

int SetCallbackRecvSerialFun(CALLBACK_RECV_SPI_FUN pRecvSerialFun)
{
	if (NULL == pRecvSerialFun)
	{
		return ReturnError;
	}
	g_pRecvSerialFun = pRecvSerialFun;
	return ReturnSuccess;
}

int SetCallbackRecvSerialBuff(unsigned char *pRecvSerialBuff)
{
	if (NULL == pRecvSerialBuff)
	{
		return ReturnError;
	}
	g_pRecvSerialBuff = pRecvSerialBuff;
	return ReturnSuccess;
}

int SetCallbackRecvSerialBuffSize(unsigned int *pRecvSerialBuffSize)
{
	if (NULL == pRecvSerialBuffSize)
	{
		return ReturnError;
	}
	g_pRecvSerialBuffSize = pRecvSerialBuffSize;
	return ReturnSuccess;
	
}


int SendDataToSerial(char *pData, unsigned int nLen)
{
	if (NULL == pData)
	{
		return ReturnError;
	}
	if (g_stateMachine <= 1)
	{
		/*无设备,不允许加入队列*/
		return ReturnError;
	}

	/*多个线程调用,需要锁*/
	//pthread_mutex_lock(&g_hRdWrMutex);
	/*加入队列尾部*/
	EnQueue(g_WriteQueue, pData, nLen);
	//pthread_mutex_unlock(&g_hRdWrMutex);
	TRACE("EnQueue !!!:%d\n", GetSize(g_WriteQueue));
	//usleep(WRITE_FREQUENCY);
	return ReturnSuccess;
}



#if 0
int SerialOpen() 
{
	if (g_hSerialDevFd <= 0)
	{
		g_hSerialDevFd = open(SPI_DEV, O_RDWR | O_NOCTTY | O_NDELAY);
		if (g_hSerialDevFd <= 0)
		{
			//perror("open");
			return ReturnError;
		}
	}
	struct termios options;
	tcgetattr(g_hSerialDevFd, &options);
    fcntl(g_hSerialDevFd, F_SETFL, 0); /*阻塞*/
    //options.c_cflag |= (CLOCAL | CREAD);
    /*fcntl(g_hSerialDevFd, F_SETFL, FNDELAY);*/ /*非阻塞*/
    /*baud tates*/
    //cfsetispeed(&options,B38400);
    //cfsetospeed(&options,B38400);
    //options.c_cflag &= ~PARENB; /*无奇偶校验*/
    //options.c_cflag &= ~CSTOPB; /*停止位1 */
    //options.c_cflag &= ~CSIZE;
    //options.c_cflag |= CS8;
	options.c_cflag = B38400 | CS8 | CREAD | CLOCAL;
    /*newtio.c_cc[VTIME] =0;*/
    /*newtio.c_cc[VMIN]=0;*/

    //options.c_lflag &=  ~(ICANON | ECHO | ECHOE); /*原始输入*/
	options.c_lflag = 0;
    //options.c_oflag &= ~OPOST; /*原始输出方式*/
	options.c_oflag = 0;
	options.c_iflag = IGNPAR; /*无奇偶校验*/

    tcsetattr(g_hSerialDevFd, TCSANOW, &options);
	return g_hSerialDevFd;
}

int SerialClose()
{
	if (g_hSerialDevFd <= 0)
	{
		return ReturnError;
	}
	close(g_hSerialDevFd);
	g_hSerialDevFd = 0;
	return ReturnSuccess;
}
#endif

int RegisterUSB()
{
	int nRetReg = usb_register(getpid(), USB_TYPE_SERIAL);
	TRACE("usb_register %d:%d\n", getpid(), nRetReg);
	g_stateMachine = 1;
	return ReturnSuccess;
}

int UnRegisterUSB()
{
	int nRetUnreg = usb_unregister(getpid());
	TRACE("usb_unregister %d:%d\n", getpid(), nRetUnreg);
	g_stateMachine = 0;
	return ReturnSuccess;
}

int LockUSB()
{
	g_stateMachine = 3;
	int nRetLock = usb_lock(g_devid);
	TRACE("usb_lock:%d\n", nRetLock);
	return ReturnSuccess;
}

int UnLockUSB()
{
	g_stateMachine = 1;
	if (0 != g_hSerialDevFd)
	{
		int nRetClose = usb_close(g_hSerialDevFd);
		TRACE("usb_close:%d\n", nRetClose);
		g_hSerialDevFd = 0;
	}
	if (0 != g_devid)
	{
		int nRetUnlock = usb_unlock(g_devid);
		TRACE("usb_unlock:%d\n", nRetUnlock);
		g_devid = 0;
	}
	
	ClearInitSourceMac();
	return ReturnSuccess;
}

int InsertUSB(int dev_id)
{
	TRACE("dev_id:%d, g_devid:%d\n", dev_id, g_devid);
	if (g_stateMachine >= 2)
	{
		/*已经有网关正在使用*/
		//int nRetUnlock = usb_unlock(g_devid);
		//TRACE("used, usb_unlock:%d\n", nRetUnlock);
		//UnLockUSB(); //不要调用这个函数，他会ClearInitSourceMac
		TRACE("bundle used!\n");
		return ReturnSuccess;
	}
	g_devid = dev_id;
	g_hSerialDevFd = usb_open(dev_id);
	TRACE("usb_open:%d\n", g_hSerialDevFd);
	if (0 >= g_hSerialDevFd)
	{
		UnLockUSB();
		return ReturnError;
	}
	int nRetSetSerial = usb_set_serial(g_hSerialDevFd, 38400, 0, 8, 1, 0, 0);
	TRACE("usb_set_serial:%d\n", nRetSetSerial);
	if (0 > nRetSetSerial)
	{
		UnLockUSB();
		return ReturnError;
	}

	/*设置串口成功,状态机设置为2*/
	g_stateMachine = 2;

	int nLockFlag = 0;
	int i = 0;
	for (i = 0; i < 10; i++)
	{
		sleep(1);
		if (SourceMacInited == CheckInitSourceMac())
		{
			nLockFlag = 1;
			break;
		}
	}
	TRACE("10s 10s 10s 10s \n");
	if (1 == nLockFlag)
	{
		LockUSB();
	}
	else
	{
		UnLockUSB();
	}
	return ReturnSuccess;
}

int PullUSB(int dev_id)
{
	TRACE("dev_id:%d, g_devid:%d\n", dev_id, g_devid);
	if (g_devid != dev_id || 0 == dev_id)
	{
		return ReturnSuccess;
	}
	UnLockUSB();
	return ReturnSuccess;
}

int SerialStart()
{
	if (NULL == g_pRecvSerialFun || NULL == g_pRecvSerialBuff)
	{
		return ReturnError;
	}
	g_WriteQueue = InitQueue();
	//pthread_mutex_init(&g_hRdWrMutex,NULL);

	g_SerialStart = 1;
	if (0 != pthread_create(&g_ptWriteSerial, NULL, pthreadWriteSerial, NULL))
	{
		perror("pthread_create");
		return ReturnError;
	}

	if (0 != pthread_create(&g_ptListen, NULL, pthreadListen, NULL))
	{
		perror("pthread_create");
		return ReturnError;
	}

	RegisterUSB();

	return ReturnSuccess;
}

int SerialStop()
{
	g_SerialStart = 0;
	UnLockUSB();
	UnRegisterUSB();
	return ReturnSuccess;
}
int SetSerialPrint()
{
	g_PrintSirial = 1;
	return ReturnSuccess;
}
int SetSerialNoPrint()
{
	g_PrintSirial = 0;
	return ReturnSuccess;
}

