//#include "servoControllerNew.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
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
  unsigned char buf[32];
  int16_t angle;
  length = 2;
  if (read(file_i2c,buf, length) != length)
    printf("Failed to read from the i2c bus.\n");
  angle = buf[0] | buf[1]<<8;
  return angle;
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
  if(argc>1)
    angle = atoi(argv[1]);
  printf("angle= %d\n",angle);
  //LX16ABus * bus = new LX16ABus();
  //bus->openBus("/dev/ttyAMA0");
  //LX16AServo * servo = new LX16AServo(bus,1);
  //usleep(1000*1000);
  //servo->setAngle(angle);
  //usleep(15000*1000);
  setAngle(angle);
  usleep(1000*1000);
  printf("read: %d\n",readAngle());
  





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
