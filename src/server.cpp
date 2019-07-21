// gcc serV.c tcplib.c -o server.exe

#include <stdio.h>
#include <netdb.h> 
#include <netinet/in.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#include <sys/types.h> 
#include <unistd.h>
#include "tcplib.h"
#include <iostream>
#include "main.h"
#define MAX 80 
#define PORT 6969
#define MLEN 8192
#define SA struct sockaddr
int sig_int, sig_hup, sig_alarm, sig_pipe;   

using namespace std;
 
double data[20];
int debug=0;

int func(int sockfd, Position *pos) 
{
  char mesg[MLEN];
  char buff[MAX]; 
  char line[MAX]; 
  int i,ib,il,n,lflg; 
  int lenbuf=0;
  // infinite loop for chat 
  for (;;) { 
    bzero(line, MAX); 
    lflg=0; il=0;
    while(lflg==0) {
      // read the message from client and copy it in buffer 
      //if (debug>3) printf("new loop lflg=%d len=%d \n",lflg,lenbuf);
      if (lenbuf==0) {
	bzero(buff, MAX); 
	lenbuf = read(sockfd, buff, sizeof(buff)); 
	if (lenbuf <= 0) return 1;
	//tcp_get2(buff, MAX);
	if (debug>3) printf("got: len=%d %s\n", lenbuf,buff);
      }
      ib=0; 
      if (debug>3) printf(" while : len=%d\n",lenbuf);
      while (lenbuf>0) {
	line[il++]=buff[ib];  lenbuf--;
	if (debug>3) printf(" copy line = %d(%c) len=%d\n",buff[ib],buff[ib],lenbuf);
	if (buff[ib]=='\n' || buff[ib]==0 ) {
	  lflg=1;
	  memcpy(buff,&buff[ib+1],lenbuf);
	  line[il++]=0; 
	  if (debug>3) printf(" end line = %c len=%d\n",buff[ib],lenbuf);
	  break;
	}
	ib++;
      }
      if (debug>3) printf(" end while : len=%d  lflg=%d \n",lenbuf,lflg);
      
    } //-- read line --
    /*
      int irnd = rand()%1000; bzero(mesg, MAX);
      sprintf(mesg,"dist %f\n",irnd/3.33);
      // print buffer which contains the client contents 
      */
    bzero(mesg, MLEN);
    if (debug > 0)
      printf("From client: len=%d %s\t To client : size=%d string:%s \n", il,line,strlen(mesg),mesg); 
    //printf("BP4");
    sprintf(&mesg[strlen(mesg)],"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",pos->x, pos->y, pos->z
	    , pos->angle, pos->dist, pos->OffSetx, pos->OffSety);
    if(strlen(mesg)>MLEN){
      printf("..........error........\n");
      exit(1);
    }
	//cout << "THIS IS MESSAGE SENT:\n" << mesg << endl;
    sprintf(&mesg[strlen(mesg)],"\n");
    if(debug > 0)
      printf("numbers: %s\n",mesg);
    // and send that buffer to client 
    //write(sockfd, data, sizeof(data)); 
    int rc = tcp_send2((int*)mesg,strlen(mesg));
    if (rc != 0)
      return 1;
    // if msg contains "Exit" then server exit and chat ended. 
    if (strncmp("exit", line, 4) == 0) { 
      printf("Server Exit...\n"); 
      break; 
    } 
  } // for loop
} 

// Driver function 
void *opentcp(void *arg){
  Position *pos=(Position*)arg;
  printf("x=%f,y=%f,An=%f,Dis=%f\n",pos->x,pos->y,pos->angle,pos->dist);
  char host[80];
  sprintf(host,"%s","127.0.0.1");
  int sock = tcp_open(PORT,host);
  while (!sig_int){
    int sockfd = tcp_listen();
    printf("server accept the client...\n"); 
    
    // Function for chatting between client and server 
    func(sockfd,pos); 
    // After chatting close the socket 
    //close(sockfd); 
    tcp_close(PORT);
  }
} 
