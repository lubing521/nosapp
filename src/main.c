
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <norouter.h>
#include <nc_ipc.h>
#include "Common.h"
#include "SerialComm.h"
#include "NetComm.h"
#include "Protocol.h"
#include "cgicomm.h"


static unsigned char g_szRecvSerialBuff[SIZE_SPI_RECV_BUFF] = {0};
static unsigned int g_nRecvSerialBuffSize = 0;

static unsigned char g_szRecvNetBuff[SIZE_NET_RECV_BUFF] = {0};
static unsigned int g_nRecvNetBuffSize = 0;
static struct sockaddr_in g_oRecvNetBuffFrom = {0,0,{0},{0}};

int CallBackRecvSerialFun()
{
	return DealSerialProtocol(g_szRecvSerialBuff, g_nRecvSerialBuffSize);
}

int CallBackRecvNetFun()
{
	return DealNetProtocol(g_szRecvNetBuff, g_nRecvNetBuffSize, &g_oRecvNetBuffFrom);
}


int main(int argc, char *argv[])
{
	int nDaemon = 0; //根据参数，创建守护进程
	int opt;

	//int r,so,max_fd=0;
	int r,max_fd=0;
	struct dev_usb_notify *pmsg;
	char buf[8192] = {0};
	struct igd_netlink_handler nlh;
	fd_set fds;
	struct timeval tv = {10,0};	

	while((opt=getopt(argc,argv, "dnsv:")) != -1)
	{
		switch(opt)
		{
			case 'd':
			{
				nDaemon = 1;
				break;
			}
			case 'v':
			{
				printf_Enable = *optarg - '0';
				break;
			}
			case 's':
			{
				SetSerialPrint();
				break;
			}
			case 'n':
			{
				SetNetPrint();
				break;
			}
			default:
			{
				break;
			}
			
		}
	}
//	printf_Enable = 3;
	if (nDaemon)
	{
		daemon(0, 0);
		//init_daemon();
	}

	//网络相关start
	SetCallBackRecvNet(CallBackRecvNetFun);	
	SetCallBackRecvNetBuff(g_szRecvNetBuff);
	SetCallBackRecvNetBuffSize(&g_nRecvNetBuffSize);
	SetCallBackRecvNetAddr(&g_oRecvNetBuffFrom);
	if (ReturnError == NetStart())
	{
		TRACEERR("net start fail\n");
		exit(ReturnError);
	}
	TRACE("net start success\n");

	//串口相关start
	SetCallbackRecvSerialFun(CallBackRecvSerialFun);
	SetCallbackRecvSerialBuff(g_szRecvSerialBuff);
	SetCallbackRecvSerialBuffSize(&g_nRecvSerialBuffSize);
	if (ReturnError == SerialStart())
	{
		TRACEERR("serial start fail\n");
		exit(ReturnError);
	}
	TRACE("serial start success\n");

	//协议相关start
	if (ReturnError == ProtocolStart())
	{
		TRACEERR("Protocol start fail\n");
		exit(ReturnError);
	}
	TRACE("Protocol start success\n");

	nlk_start_msg(MSG_START_OK,NULL);			
	
	sleep(1);

	/* init. system message */
	nlk_msg_init(&nlh,(0x1 << (NLKMSG_GRP_STOP - 1)) | (0x1 << (NLKMSG_GRP_DEVICE - 1)));

	while(1) {
		FD_ZERO(&fds);
		IGD_FD_SET(nlh.sock_fd, &fds);
		tv.tv_sec = 10;
		tv.tv_usec = 0;

		if ((r = select(max_fd+1, &fds, NULL, NULL, &tv)) < 0) {
			if (errno == EINTR || errno == EAGAIN)
			    continue;
		}

		if (FD_ISSET(nlh.sock_fd,&fds)) {
			//nlk_msg_recv(&nlh,&pmsg->sizeof(msg_app_t));	
			//nlk_msg_recv(&nlh,&pmsg->sizeof(struct dev_usb_notify));	
			nlk_msg_recv(&nlh,buf,sizeof(buf));
			pmsg = (struct dev_usb_notify *)&buf[0];
			printf("type = %d\n",pmsg->dev_type);
			printf("id = %d\n",pmsg->dev_id);
			printf("action = %d\n",pmsg->comm.action);
			printf("key = %d\n",pmsg->comm.key);
			printf("gid = %d\n",pmsg->comm.gid);
			if(NLKMSG_GRP_APP == pmsg->comm.gid && pmsg->comm.key == MSG_STOP){
				TRACE("MSG_STOP\n");
				NetStop();
				SerialStop();
				ProtocolStop();
				close(nlh.sock_fd);
				exit(1);
			}
			else if (NLKMSG_GRP_DEVICE == pmsg->comm.gid && /*USB_SERIAL*/0 == pmsg->dev_type && NLKMSG_GRP_DEV_ACT_IN == pmsg->comm.action)
			{
				TRACE("NLKMSG_GRP_DEV_ACT_IN\n");
				InsertUSB(pmsg->dev_id);
			}
			else if (NLKMSG_GRP_DEVICE == pmsg->comm.gid && /*USB_SERIAL*/0 == pmsg->dev_type && NLKMSG_GRP_DEV_ACT_OUT == pmsg->comm.action)
			{
				TRACE("NLKMSG_GRP_DEV_ACT_OUT\n");
				PullUSB(pmsg->dev_id);
			}
		}
	}

	return ReturnSuccess;
}

