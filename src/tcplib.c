#include "tcplib.h"
struct sockaddr_in sinn;
struct sockaddr_in pin;
//struct hostent *gethostbyname(char *);
struct hostent *hp;
int	sd, sd_current;
extern int sig_int, sig_hup, sig_alarm, sig_pipe;   
/*====================================================================*/
/*             TCP open                                               */
/*====================================================================*/
int tcp_open(int PORT,char* hostname){
  int rc;
  printf(" tcp_open:: PORT=%d HOST=%s \n",PORT,hostname);
  if (PORT<0) { PORT=-PORT; rc=open_remote(PORT,hostname); }
  else        {             rc=open_local(PORT);  }
  return rc;
}
/*--------------------------------------------------------------------*/
int open_local(int PORT){                      /*--- LISTEN !!! -----*/
    int on = 1;
    
    printf(" open_local:: PORT=%d \n",PORT);
    /* get an internet domain socket */
    if ((sd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
	perror("socket");
	return -1;
    }
    printf("tcplib::open_local:: setsockopt(sock=%d) SO_REUSEADDR  !!!\n",sd);
    if (setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)
    {
	perror("setsockopt(SO_REUSEADDR) failed");
    }
    
    /* complete the socket structure */
    memset(&sinn, 0, sizeof(sinn));
    sinn.sin_family = AF_INET;
    sinn.sin_addr.s_addr = htonl(INADDR_ANY);
    sinn.sin_port = htons(PORT);

    printf("tcplib::open_local::  bind socket=%d  !!!\n",sd);
    /* bind the socket to the port number */
    if (bind(sd, (struct sockaddr *) &sinn, sizeof(sinn)) == -1) {
	perror("bind");
	return -1;
    }
    return sd;
}
/*--------------------------------------------------------------------*/
int open_remote(int PORT,char* hostname){   /*----- CONNECT  !!!  ----*/
  //strcpy(hostname,HOST);
  if(!sig_hup) printf(" go find out about the desired host machine \n");
  if ((hp = gethostbyname(hostname)) == 0) {
    if(!sig_hup) perror("gethostbyname");
    return -1;
  }
  if(!sig_hup) printf(" name=%s \n",hp->h_name);
  if(!sig_hup) printf(" alias=%s \n",(char*)hp->h_aliases);
  if(!sig_hup) printf(" adr_type=0x%x \n",hp->h_addrtype);
  if(!sig_hup) printf(" adr_len=%d \n",hp->h_length);
  if(!sig_hup) printf(" address=%x \n",(unsigned long)hp->h_addr);
  
  if(!sig_hup) printf(" fill in the socket structure with host information \n");
  //        sprintf(hp->h_addr,"131.169.46.71");
  
  /* fill in the socket structure with host information */
  memset(&pin, 0, sizeof(pin));
  
  if(!sig_hup) printf(" after memset info \n");
  
  pin.sin_family = AF_INET;
  pin.sin_addr.s_addr = ((struct in_addr *)(hp->h_addr))->s_addr;
  pin.sin_port = htons(PORT);

  if(!sig_hup) printf(" grab an Internet domain socket\n ");
  
  if ((sd_current = socket(AF_INET, SOCK_STREAM, 0)) == -1 ) {
    if(!sig_hup) perror("socket");
     return -1;
  }
  
  if(!sig_hup) printf(" get socket option \n ");
  // getsockopt(sd,0,SO_SNDBUF,optval,optlen);           
  
  // printf("optval,optlen=  %d  %d \n",*optval,*optlen);
  //printf("optval,optlen=  %s  %d \n",*optval,*optlen);
  
  
  if(!sig_hup) printf(" try to connect \n");
  /* connect to PORT on HOST */
  if (connect(sd_current,(struct sockaddr *)  &pin, sizeof(pin)) == -1) {
    if(!sig_hup) perror("connect");
    return -1;
  }
  return sd_current;
}
/*==================================================================*/
/*               LISTEN TCP SOCKET                                  */
/*==================================================================*/
int tcp_listen(void){
  static socklen_t 	 addrlen;
  int rem_port;
  printf("Waiting for replay\n");
  /* show that we are willing to listen */
  if (listen(sd, 5) == -1) {
    perror("listen");
    return -1;
  }
  /* wait for a client to talk to us */
  addrlen=sizeof(pin);
  if ((sd_current = accept(sd, (struct sockaddr *)  &pin, &addrlen)) == -1) {
    perror("accept");
    return -1;
  }
  rem_port=ntohs(pin.sin_port);

  if ((  hp = gethostbyaddr(&pin.sin_addr,sizeof(pin.sin_addr),AF_INET)    ) == 0) {
    perror("tcp_listen()::gethostbyaddr");
  }
//  printf("listen:: Connection from %s(%s) accept at sock=%d rem_port=%d\n"
//  	 ,hp->h_name,inet_ntoa(pin.sin_addr),sd_current,rem_port);
  printf("listen:: Connection from %s accept at sock=%d rem_port=%d\n"
  	 ,inet_ntoa(pin.sin_addr),sd_current,rem_port);

  //return rem_port;
  return sd_current;
}
/*==================================================================*/
int tcp_listen2(char* host_name, int len){
  static socklen_t 	 addrlen;
  int rem_port;
  printf("Waiting for replay\n");
  /* show that we are willing to listen */
  if (listen(sd, 5) == -1) {
    perror("listen");
    return -1;
  }
  /* wait for a client to talk to us */
  addrlen=sizeof(pin);
  if ((sd_current = accept(sd, (struct sockaddr *)  &pin, &addrlen)) == -1) {
    perror("accept");
    return -1;
  }
  rem_port=ntohs(pin.sin_port);
  printf("listen:: hp->h_name:: rem_port=%d \n",rem_port);

  printf("listen:: try get host by addr %s \n",inet_ntoa(pin.sin_addr));

  if ((  hp = gethostbyaddr(&pin.sin_addr,sizeof(pin.sin_addr),AF_INET)    ) == 0) {
      perror("tcp_listen()::gethostbyaddr");
      strncpy(host_name,inet_ntoa(pin.sin_addr),len); host_name[len-1]=0;
  } else { 
      printf("listen:: hp->h_name:: ptr=%p \n",hp->h_name);   
      strncpy(host_name,hp->h_name,len);              host_name[len-1]=0;
  }
  printf("listen:: Connection from %s(%s) accept at sock=%d rem_port=%d\n"
	 ,host_name,inet_ntoa(pin.sin_addr),sd_current,rem_port);
  return rem_port;
}
/*====================================================================*/
/*             TCP send    data                                       */
/*====================================================================*/
int tcp_send(int *DATA,int lenDATA){
  int cc;
  //-----------------------------------------------------
  if ((cc=send(sd_current, DATA,lenDATA, 0)) == -1) {
    if(!sig_hup) perror("send DATA");
    return -1;
  }
 // if(!sig_hup) printf("sizeof(DATA)=%d (%d)\n",lenDATA,cc);
  //-----------------------------------------------------
  return 0;
}
/*====================================================================*/
/*             TCP send    data    2                                  */
/*====================================================================*/
int tcp_send2(int *DATA,int lenDATA){
   int nleft,nsent;
   char* snd;
  //-----------------------------------------------------
   nleft=lenDATA;
   snd=(char*)DATA;
   while(nleft>0){
       // printf("try to send = %d of %d\n",PACKSIZE,nleft);
       nsent=send(sd_current,snd, nleft, 0);
       if (nsent <=0) { perror("send"); return -1;}
       nleft-=nsent;
       snd+=nsent;
   }
   return nleft; 
}
/*====================================================================*/
/*             TCP receive line                                       */
/*====================================================================*/
int tcp_get_line(char *DATA,int lenDATA){
  int nleft,nread=0;        
  char *rcv;
#define MAX 80 
  static char buff[MAX]; 
  static char line[MAX]; 
  static int i,ib,il,n,lflg; 
  int debug=5;
  static int lenbuf=0;



  bzero(line, MAX); 
  lflg=0; il=0;
  while(lflg==0) {
    // read the message from client and copy it in buffer 
    if (debug>3) printf("new loop lflg=%d len=%d \n",lflg,lenbuf);
    if (lenbuf==0) {
      bzero(buff, MAX); 
      //lenbuf = read(sockfd, buff, sizeof(buff)); 
      lenbuf=recv(sd_current,buff,MAX, 0);
      if (debug>3) printf("got: len=%d %s\n", lenbuf,buff);
      if (nread <=0) {perror("recv"); return -1;}
    }
    sleep(1);
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

      return 0; 
}
/*====================================================================*/
/*             TCP receive data                                       */
/*====================================================================*/
int tcp_get(int *DATA,int lenDATA){
  int nleft,nread=0;        
  char *rcv;
      //-----------------------------------------------------
      nleft=lenDATA;
      rcv=(char*)DATA;
      while(nleft>0){
	nread=recv(sd_current,rcv,nleft, 0);
	if (nread <=0) {perror("recv"); return -1;}
	nleft-=nread;
	rcv+=nread;
      }
      return 0; 
}
/*====================================================================*/
int tcp_get2(int *DATA,int lenDATA){
  int nleft,nread=0;        
  char *rcv;
      //-----------------------------------------------------
      nleft=lenDATA;
      rcv=(char*)DATA;
      while(nleft>0){
//      if(!sig_hup) printf("try to receive = %d of %d\n",nleft,lenDATA);
	nread=recv(sd_current,rcv,nleft, 0);
	if (nread <=0) {perror("recv"); return -1;}
	nleft-=nread;
//        if(!sig_hup)  printf("nread= %d ==== nleft=%d\n",nread,nleft);
	rcv+=nread;
      }
      return nleft; 
}

/*====================================================================*/
/*             TCP close                                              */
/*====================================================================*/
int tcp_close(int PORT){
    close(sd_current);  
    printf("TCPLIB::close_port sd_current=%d PORT_FLG=%d\n",sd_current,PORT);
    if (PORT==0) { 
	close(sd); 
	printf("TCPLIB::close_port sd=%d PORT_FLG=%d\n",sd,PORT); 
    }
    return 0;
}
//-----------------------------
int tcp_close_old(){
  close(sd);
  printf("TCPLIB::close_old port=%d\n",sd);
  return 0;
}
/*====================================================================*/
/* fork2() - like fork, but the new process is immediately orphaned
 *           (won't leave a zombie when it exits)
 * Returns 1 to the parent, not any meaningful pid.
 * The parent cannot wait() for the new process (it's unrelated).
 */

/* This version assumes that you *haven't* caught or ignored SIGCHLD. */
/* If you have, then you should just be using fork() instead anyway.  */

int fork2()
{
    pid_t pid;
    int status;

    if (!(pid = fork()))
    {
        switch (fork())
        {
          case 0:  return 0;
          case -1: _exit(errno);    /* assumes all errnos are <256 */
          default: _exit(0);
        }
    }

    if (pid < 0 || waitpid(pid,&status,0) < 0)
      return -1;

    if (WIFEXITED(status))
      if (WEXITSTATUS(status) == 0)
        return 1;
      else
        errno = WEXITSTATUS(status);
    else
      errno = EINTR;  /* well, sort of :-) */
    return -1;
}
/*====================================================================*/

