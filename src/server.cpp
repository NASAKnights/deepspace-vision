// gcc serV.c tcplib.c -o server.exe

#include <stdio.h>
#include <netdb.h> 
#include <netinet/in.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#include <sys/types.h> 
#include <unistd.h>
#include <iostream>
#include "main.h"
#include "tcp_thread.h"
#define MAXLINE 80 
#define MLEN 8192
#define SA struct sockaddr
int sig_int, sig_hup, sig_alarm, sig_pipe;   

using namespace std;
extern int buttonPress;
double data[20];
int debug=0;
int rem_port,sock_main;
#define LINFO 128
#define MAXCLNT 32  //--  used 32 bit word   !!!!
static int TCP_FLAG=0;
pthread_t cl_thread[MAXCLNT];

static void *run_cl_thread (void* arg);


struct CLIENT {
  int Kclnt;       //-- client number
  int sd_current;  //-- tcp port number
  int rem_port;
  Position* pos;
};
const int LHOST=128;
struct HOST {
    int PORT;
    char NAME[LHOST];
    int ModID;
};

int func(int sockfd, Position *pos) 
{
  char mesg[MLEN];
  char buff[MAXLINE]; 
  char line[MAXLINE]; 
  int i,ib,il,n,lflg; 
  int lenbuf=0;
  // infinite loop for chat 
  for (;;) { 
    bzero(line, MAXLINE); 
    lflg=0; il=0;
    while(lflg==0) {
      // read the message from client and copy it in buffer 
      //if (debug>3) printf("new loop lflg=%d len=%d \n",lflg,lenbuf);
      if (lenbuf==0) {
	bzero(buff, MAXLINE); 
	lenbuf = read(sockfd, buff, sizeof(buff)); 
	if (lenbuf <= 0) return 1;
	//tcp_get2(buff, MAXLINE);
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
    if(atoi(buff) != -1)
      buttonPress = atoi(buff);
    
    /*
      int irnd = rand()%1000; bzero(mesg, MAXLINE);
      sprintf(mesg,"dist %f\n",irnd/3.33);
      // print buffer which contains the client contents 
      */
    bzero(mesg, MLEN);
    if (debug > 0)
      printf("From client: len=%d %s\t To client : size=%d string:%s \n", il,line,strlen(mesg),mesg); 
    //printf("BP4");
    sprintf(&mesg[strlen(mesg)],"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,",pos->x, pos->z, pos->dist, pos->angle, pos->angle2, pos->OffSetx, pos->speed, pos->turn, pos->gyro);
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
    int rc = tcp_send_th(sockfd,(int*)mesg,strlen(mesg));
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
/*
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
*/

void *opentcp(void *arg){
  Position *pos = (Position*) arg;
  
  unsigned int Client_mask=0,Nclnt=0,Kclnt=0;
  struct HOST host;
  int sd_clnt=0;
  /*-----------  default value for hosts/ports ---------------------------*/
  strncpy(host.NAME,"127.0.0.1",LHOST); host.PORT=6969;

  //============================================================
  //           wait for incoming  connections loop  
  //============================================================
  while (TCP_FLAG==0) {
    sock_main=tcp_open_th(host.PORT,NULL);  TCP_FLAG=1;
    if(sock_main<0){ 
      //tcp_close(0); TCP_FLAG=0;
      sleep(1); 
    }
  }

  printf("==> Wait for clients loop. \n");

  while(!sig_int) {
    printf(" while_loop:: PORT=%d ",host.PORT);
    char host_name[LINFO];
    rem_port=tcp_listen3(sock_main,host_name,LINFO,&sd_clnt);
    printf(" return from listen3 , ADD NEW Client host=%s remport =%d \n", host_name,rem_port);

    if (Nclnt>=MAXCLNT) { printf(" Error :: MAX CLIENTS=%d\n",MAXCLNT); continue; }
    //---   add new client here -----------
    Kclnt=Nclnt++;
    printf(" ADD NEW Client DONE N_clnt=%d Kclnt=%d Mask=%#x !!! \n",Nclnt,Kclnt,Client_mask);
    //---  end add new client -------
    //pid=fork2();
    //--------------------------------------------------------------
    struct CLIENT sclnt;
    sclnt.Kclnt=Kclnt;
    sclnt.sd_current=sd_clnt;
    sclnt.rem_port=rem_port;
    sclnt.pos=pos;
    int ret = pthread_create (&cl_thread[Kclnt], 0, run_cl_thread, &sclnt);
    if (ret) {
      perror ("pthread_create");
    }
    char th_name[64];
    sprintf(th_name,"cl_thread_%d",Kclnt);
    ret=pthread_setname_np(cl_thread[Kclnt],th_name);
    printf("\n detach run_child thread Kclnt=%d\n",Kclnt);   ret = pthread_detach(cl_thread[Kclnt]);
    if (ret!= 0) fprintf(stderr, "detach %d failed %d\n", Kclnt,ret);

    //----------------------------------------------------
    printf(" Create child \n");
    //tcp_close(-1);
  }  //--- end While(1) Loop
 
  /*-----------------------------------------------------------*/
  printf(" Close TCP ports \n");
  //tcp_close(0);
}


static void *run_cl_thread (void* arg) {
    struct CLIENT *clnt = (struct CLIENT*) arg;
    unsigned int Kclnt = clnt->Kclnt;
    int sd_current= clnt->sd_current;
    int rem_port=clnt->rem_port;

    printf("start new client thread , Kclnt=%d sd=%d \n",Kclnt,sd_current);
    //tcp_close_old();  // ???
    func(sd_current,clnt->pos);
    printf("thread return from Client No=%d close port=%d\n",Kclnt,sd_current);
    
    close(sd_current);

    printf("EXIT child !!!!  clean client here k=%d !!!!\n",Kclnt);
    
    return 0;
}
