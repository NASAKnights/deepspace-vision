//#include "servoControllerNew.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <iostream>
//stty -F /dev/ttyS0 115200
int addr = 0x04;
int file_i2c;
int length; 
char *filename = (char*)"/dev/i2c-1";


bool setAngle(double angle){
  if(angle>90)
    angle=90;
  if(angle<-90)
    angle=-90;
  angle = angle/90. * 350;
  angle += 350;//350 = centered
  int16_t sendAngle = (int) angle;
  printf("buffer sending: %d\n",sendAngle);
  length = sizeof(sendAngle);
  if (write(file_i2c, &sendAngle, length) != length)
    printf("Failed to write to the i2c bus.\n");
}

int16_t readAngle(){
  int16_t i = -1;
  length = sizeof(int16_t);
  if (write(file_i2c, &i, length) != length)
    printf("Failed to write to the i2c bus.\n");
  usleep(50*1000);
  unsigned char buf[32];
  int16_t angle;
  length = 2;
  if (read(file_i2c,buf, length) != length)
    printf("Failed to read from the i2c bus.\n");
  angle = buf[0] | buf[1]<<8;
  angle -= 350;//350 = centered
  angle = angle/350. * 90;
  return angle;//check for returning of 63 degrees or 597 in their system
}


int main(int argc, char* argv[]){

  if ((file_i2c = open(filename, O_RDWR)) < 0){
    printf("Failed to open the i2c bus");
    return false;
  }
  if (ioctl(file_i2c, I2C_SLAVE, addr) < 0){
    printf("Failed to acquire bus access and/or talk to slave.\n");
    return false;
  }

  
  int angle = 0;
  int rAngle = 0;
  if(argc>1)
    angle = atoi(argv[1]);
  printf("angle= %d\n",angle);
  while(true){
    angle = rand() % 180 - 90;
    //std::cin >> angle;
    setAngle(angle);
    while(abs(rAngle-angle)>1){
      usleep(10*1000);
      rAngle = readAngle();
      if(rAngle != 63)
	printf("read: %d\n",rAngle);
    }
  }





  /*
  uint8_t id2;
  printf("moved to %d \n",angle);
  //servo->id_write(2);
  sleep(5);
  while(1) {
    int rc1=servo->id_read(id2);
    printf("read:: id=%d  \n",id2);
    //
    usleep(10000000);
  int16_t angle2;
  //while(1) {
  //int rc2=servo->pos_read(angle2);
  //vprintf("read:: size=%d rc=%d angle=%d \n",sizeof(unsigned long long),rc2,angle2);
  //  usleep(2000000);
  }
  */
  return 0;
}
