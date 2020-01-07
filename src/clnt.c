//gcc clnt.c -o clnt.exe
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <time.h>
#include <netdb.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#define MAX 8192 
#define MLEN 8192
#define SA struct sockaddr 

double data[20];
int debug=0;

int func(int sockfd) 
{ 
  char buff[MAX]; 
  char line[MAX]; 
  char *cmd="get dist";
  int i,ib,il,n,lflg; 
  int lenbuf=0;
  char mesg[MLEN];
  for (;;) { 
    bzero(buff, sizeof(buff)); 
    if (debug>3) printf("send cmd :%s\n",cmd); 
    n = 0; 
    //while ((buff[n++] = getchar()) != '\n') ; 
    write(sockfd, cmd, strlen(cmd)+1); 

    //tcp_get(mesg,sizeof(data),sockfd);


    //tcp_get_line();

    // infinite loop for chat 
    bzero(line, MAX); 
    lflg=0; il=0;
    
    while(lflg==0) {
      // read the message from client and copy it in buffer 
      if (debug>3) printf("new loop lflg=%d len=%d \n",lflg,lenbuf);
      if (lenbuf==0) {
	bzero(buff, MAX); 
	lenbuf = read(sockfd, buff, sizeof(buff)); 
	if (lenbuf <= 0) return 1;
	//tcp_get2(buff, MAX);
	if (debug>3) printf("got: len=%d %s\n", lenbuf,buff);
      }
      //sleep(1);
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
	

    time_t tm = time(NULL);
    printf("From Server (%ld)  \n",tm);
    printf("%s", buff);
    printf("\n");
	
    if ((strncmp(buff, "exit", 4)) == 0) { 
      printf("Client Exit...\n"); 
      break; 
    }
    usleep(100000);
  }
} 
  
int main(int argc, const char* argv[])
{ 

  char HOST[256];
  sprintf(HOST,"127.0.0.1");
  int PORT=6969;
  
  if (argc>1) sprintf(HOST,"%s",argv[1]);
  if (argc>2) PORT=atoi(argv[2]);

  //tcp_open(PORT,argv0);
  int sockfd, connfd; 
  struct sockaddr_in servaddr, cli;   
  // socket create and varification 
  sockfd = socket(AF_INET, SOCK_STREAM, 0); 
  if (sockfd == -1) { 
    printf("socket creation failed...\n"); 
    exit(0); 
  } 
  else
    printf("Socket successfully created..\n"); 
  bzero(&servaddr, sizeof(servaddr)); 
  
  // assign IP, PORT 
  servaddr.sin_family = AF_INET; 
  //servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 
  //servaddr.sin_addr.s_addr = inet_addr("10.1.22.20"); 
  servaddr.sin_addr.s_addr = inet_addr(HOST); 
  servaddr.sin_port = htons(PORT); 
  
  // connect the client socket to server socket 
  if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) { 
    printf("connection with the server failed  %s:%d ...\n",HOST,PORT); 
    exit(0); 
  } 
  else
    printf("connected to the server %s:%d ..\n",HOST,PORT); 
  
  // function for chat 
  func(sockfd); 
  
  // close the socket 
  close(sockfd); 
} 
