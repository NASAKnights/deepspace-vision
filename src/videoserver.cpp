#include "tcplib.h"

#include <stdio.h>
#include <netdb.h> 
#include <netinet/in.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#include <sys/types.h> 
#include <unistd.h>

extern int remoteSocket;
extern int videoPort;
extern int videoError;

void *videoServer(void *arg){
  char host[80];
  sprintf(host,"%s","127.0.0.1");
  int sock = tcp_open(videoPort,host);
  while(1){
    int sockfd = tcp_listen();
    remoteSocket = sockfd;
  /*
  while (!sig_int){
    int sockfd = tcp_listen();
    printf("server accept the client...\n"); 
    if(USESERVER){
      int localSocket;
      struct sockaddr_in localAddr, remoteAddr;

      int addrLen= sizeof(struct sockaddr_in);
      localSocket = socket(AF_INET,SOCK_STREAM,0);
      if(localSocket == -1){
	printf("error");
      }
      localAddr.sin_family = AF_INET;
      localAddr.sin_addr.s_addr = INADDR_ANY;
      localAddr.sin_port = htons( port );
    
      if( bind(localSocket,(struct sockaddr *)&localAddr , sizeof(localAddr)) < 0) {
	perror("Can't bind() socket");
	exit(1);
      }
    
      //Listening
      listen(localSocket , 3);
    
      std::cout <<  "Waiting for connections...\n"
		<<  "Server Port:" << port << std::endl;
    
      //accept connection from an incoming client
      remoteSocket = accept(localSocket, (struct sockaddr *)&remoteAddr, (socklen_t*)&addrLen);  
      //std::cout << remoteSocket<< "32"<< std::endl;
      if (remoteSocket < 0) {
	perror("accept failed!");
	exit(1);
      }
    }
    }
  */
    while(videoError==0) sleep(1);
    //tcp_close(remoteSocket);
    remoteSocket=0;
    videoError=0;
  }
} 
