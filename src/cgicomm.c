#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <cgi.h>
#include <fcntl.h>
#include <unistd.h>




int cgi_comm_server_init (char *path)
{
	int sock;
	struct sockaddr_un addr;
	//sigset_t set;
	int len;

	if (!path) 
		return -1;
	/* Create UNIX domain socket.  */
	sock = socket (AF_UNIX, SOCK_STREAM, 0);
	if (sock < 0)
	{
		console_printf("create management socket error.\r\n");	
		return -1;
	}
	/* Unlink. */
	unlink(path);

	/* Prepare accept socket. */
	memset (&addr, 0, sizeof (struct sockaddr_un));
	addr.sun_family = AF_UNIX;
	strcpy (addr.sun_path, path);
	len = sizeof (addr.sun_family) + strlen(addr.sun_path);

	/* Bind socket. */
	if(bind(sock, (struct sockaddr *) &addr, len) < 0)
	{
		console_printf("bind socket error..\n");
		close(sock);
		return -1;
	}

	if(listen(sock,5)<0)
	{
		console_printf("listen on socket error.\n");
		close(sock);
		return -1;
	}
	return sock;
}


int cgi_comm_connect(char *path)
{
  int sock,len,rv;
  struct sockaddr_un addr;

  /* Create UNIX domain socket.  */
  if((sock = socket (AF_UNIX, SOCK_STREAM, 0))<0)
  {
	console_printf("create socket error,errno:%d\n",sock);  	
    return -1;
  }
  /* Prepare accept socket. */
  memset (&addr, 0, sizeof (struct sockaddr_un));
  addr.sun_family = AF_UNIX;
  strcpy (addr.sun_path, path);
  len = sizeof (addr.sun_family) + sizeof(addr.sun_path);
  
  /* Bind socket. */
  if ((rv=connect (sock, (struct sockaddr *) &addr, len)) < 0)
  {
	close (sock);
	console_printf("connect error:%d\n",rv);
      return -1;
  }
  else
  {
  	console_printf("connect OK,socket=%d\n",sock);
  }
  return sock;
}



