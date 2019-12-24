//stty -F /dev/ttyS0 115200

#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <stdint.h>

#define WAITTIME 90 

// g++ -Wall MotorController.cpp -o MotorControl.exe

// gcc rs_port_read.c -o rs_port_read.exe

// linux:
// ------
// stty -F /dev/ttyS0 19200 ; set speed
// stty -F /dev/ttyS0       ; check speed
// screen  /dev/ttyS0 19200






//unsigned long millis();
//uint32_t ttime(uint8_t);


class LX16ABus {
public:
  LX16ABus(){
    _baud=115200;// 9600
  }
  int openBus(const char *port) {
    /*
      _port = port;
      _baud = baud;
      port.begin(baud, SERIAL_8N1, pin, pin);
      pinMode(pin, OUTPUT|PULLUP|OPEN_DRAIN);
      delay(3);
      while (port.available()) port.read();
    */
    ttyFid = open(port, O_RDWR | O_NONBLOCK);
    if (ttyFid == -1){
      printf( "Error unable to open port: %s\n", port );
      return -1;
    }
    printf("open port fd = %d\n",ttyFid);
    //write(ttyFid,"lets go",8);
    return 0;
  }
  
  // methods passed through to Serial
  //bool available() { return _port.available(); }
  int readBus() { 
    //printf("enter readBus\n");
    unsigned char ch;
    int ret;
    int nb = read(ttyFid,&ch,1);
    if(nb != 1){
      //printf("read bus : no data nb=%d\n",nb);      
      ret=nb;
    }else{
      printf("readBus: ch=(0x%02x) \n",ch);
      ret=ch;
    }
    //printf("ret=0x%02x\n",ret);
    return ret;
  }
  void writeBus(const uint8_t *buf, int buflen) { 
    int nb = write(ttyFid, buf ,buflen);
    if (nb != buflen) printf("writeBus ERROR nb=%d lbuf=%d \n",nb,buflen);
    /*
      for(int i=0;i<buflen;i++){
      int nb = write(ttyFid, &buf[i] ,1);
      //printf("sent byte %d\n",i);
      usleep(WAITTIME);
      }
    */
  }
  
  
  uint32_t ttime(uint8_t n) {
    return n*10*1000/_baud; // 10 bits per char
  }
  

  int _baud;
  int ttyFid;
};


class LX16AServo {
public:
  LX16AServo(LX16ABus * port, int id){
    _bus=port;
    _id=id;
    _debug=true;
  }

  unsigned long micros(){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    unsigned long int sec = tv.tv_sec;
    unsigned long int usec = tv.tv_usec;
    unsigned long int timeNow = (sec*1000000+usec);
    return timeNow;
  }
  unsigned long millis(){
    return micros()/1000;
  }
  // write a command with the provided parameters
  // returns true if the command was written without conflict onto the bus
  bool writeLX(uint8_t cmd, const uint8_t *params, int param_cnt){
    if (param_cnt < 0 || param_cnt > 4) return false;
    // prepare packet in a buffer
    int buflen = 6 + param_cnt;
    uint8_t buf[buflen];
    buf[0] = 0x55;
    buf[1] = 0x55;
    buf[2] = _id;
    buf[3] = buflen-3;
    buf[4] = cmd;
    for (int i=0; i<param_cnt; i++) buf[5+i] = params[i];
    uint8_t cksum = 0;
    for (int i=2; i<buflen-1; i++) cksum += buf[i];
    buf[buflen-1] = ~cksum;


    // clear input buffer
    printf("clear bus...\n");
    int ch;
    while ((ch=_bus->readBus()) >= 0) printf("read ch = 0x%02x\n",ch);
    printf("clear bus done.\n");

    if (_debug) {
      printf("SND: ");
      for (int i=0; i<buflen; i++) printf(" 0x%02x", buf[i]);
      printf("\n");
    }
    // send command packet
    unsigned long t0 = millis();
    _bus->writeBus(buf, buflen);

    // expect to read back command by virtue of single-pin loop-back
    uint32_t tout = _bus->ttime(buflen) + 1; // 2ms margin
    int got = 0;
    bool ok = true;
    if (_debug) printf("RCV: (tout=%d t0=%ld now=%ld) readback cmd ",tout,t0,millis());
    while (got < buflen && millis()-t0 < tout) {
      int ch=_bus->readBus();
      if (ch >= 0) {
	if (_debug) printf(" 0x%02x", ch);
	if (ch != buf[got]) ok = false;
	got++;
	printf(" dt=%ld ",millis() -t0);
      } else 
	printf(".");
    }
    if (_debug) { if (ok) printf(" OK\n"); else printf(" ERR\n"); }
    return ok;
  }
  
  // read sends a command to the servo and reads back the response into the params buffer.
  // returns true if everything checks out correctly.
  bool readLX(uint8_t cmd, uint8_t *params, int param_len){
    // send the read command
    if (_debug>0) printf("readLX:: send read command = %d (0x%x)\n",cmd,cmd);
    bool ok = writeLX(cmd, NULL, 0);
    if (!ok) return false;

    //if (_debug>0) printf("readLX:: read back the expected motor response ...\n");
    unsigned long t0 = millis();
    uint32_t tout = _bus->ttime(param_len+6) + 20; // 20ms for the servo to think
    int got = 0;
    uint8_t sum = 0;
    if (_debug) printf("RCV: (tout=%d) ",tout);
    int len = 7; // minimum length
    while (got < len && millis()-t0 < tout) {
      int ch = _bus->readBus();
      if ( ch >= 0) {
	if (_debug) printf(" 0x%02x", ch);
	switch (got) {
	case 0:
	case 1:
	  if (ch != 0x55) { if (_debug) printf(" ERR (hdr)\n"); return false; }
	  break;
	case 2:
	  if (ch != _id && _id != 0xfe) { if (_debug) printf(" ERR (id)\n"); return false; }
	  break;
	case 3:
	  if (ch < 3 || ch > 7) { if (_debug) printf(" ERR (len)\n"); return false; }
	  len = ch+3;
	  if (len > param_len+6) { if (_debug) printf(" ERR (param_len)\n"); return false; }
	  break;
	case 4:
	  if (ch != cmd) { if (_debug) printf(" ERR (cmd)\n"); return false; }
	  break;
	default:
	  if (got == len-1) {
	    if ((uint8_t)ch == (uint8_t)~sum) { if (_debug) printf(" OK\n"); return true; }
	    else { if (_debug) printf(" ERR (cksum!=%02x)\n", ~sum); return false; }
	  }
	  if (got-5 > param_len) { if (_debug) printf(" ERR (cksum)\n"); return false; }
	  params[got-5] = ch;
	}
	if (got > 1) sum += ch;
	got++;
      } //else printf(".");
    }
    if (_debug) printf(" TIMEOUT\n");
    return false;
  }

  // motor_mode causes the motor to rotate at a fixed speed (-1000..1000) and switches to
  // position (servo) mode if speed==0
  bool motor_mode(uint16_t speed) {
    uint8_t params[] = { (uint8_t)(speed==0 ? 0 : 1), 0, (uint8_t)speed, (uint8_t)(speed>>8) };
    return writeLX(29, params, sizeof(params));
  }
  
  // angle_adjust sets the position angle offset in centi-degrees (-3000..3000)
  bool angle_adjust(int16_t angle) {
    uint8_t params[] = { (uint8_t)((int32_t)angle*125/30) };
    return writeLX(17, params, sizeof(params));
  }

  // angle_limit sets the upper and lower position limit in centi-degrees (0..24000)
  bool angle_limit(uint16_t min_angle, uint16_t max_angle) {
    min_angle = min_angle/24;
    max_angle = max_angle/24;
    uint8_t params[] = {
			(uint8_t)min_angle, (uint8_t)(min_angle>>8),
			(uint8_t)max_angle, (uint8_t)(max_angle>>8) };
    return writeLX(20, params, sizeof(params));
  }

  // move_time positions the servo to the angle in centi-degrees (0..24000) in time milliseconds (0..1000)
  bool move_time(uint16_t angle, uint16_t time) {
    angle = angle/24;
    uint8_t params[] = { (uint8_t)angle, (uint8_t)(angle>>8), (uint8_t)time, (uint8_t)(time>>8) };
    return writeLX(1, params, sizeof(params));
  }

  // pos_read returns the servo position in centi-degrees (0..24000)
  bool pos_read(int16_t &angle) {
    uint8_t params[2];
    if (!readLX(28, params, sizeof(params))) return false;
    angle = ( (int16_t)params[0] | ((int16_t)params[1]<<8) ) * 24;
    return true;
  }

  // id_read returns the ID of the servo, useful if the id is 0xfe, which is broadcast...
  bool id_read(uint8_t &id) {
    uint8_t params[1];
    if (!readLX(14, params, sizeof(params))) return false;
    id = params[0];
    return true;
  }

  // id_write sets the id of the servo, updates the object's id if write appears successful
  bool id_write(uint8_t id) {
    uint8_t params[] = { id };
    bool ok = writeLX(13, params, sizeof(params));
    if (ok) _id = id;
    return ok;
  }

  // temp_read returns the servo temperature in centigrade
  bool temp(uint8_t &temp) {
    uint8_t params[1];
    if (!readLX(26, params, sizeof(params))) return false;
    temp = params[0];
    return true;
  }

  // vin_read returns the servo input voltage in millivolts
  bool vin(uint16_t &vin) {
    uint8_t params[2];
    if (!readLX(27, params, sizeof(params))) return false;
    vin = params[0]|((uint16_t)params[1]<<8);
    return true;
  }

  bool setAngle(int angle){
    if(angle>90)
      angle=90;
    if(angle<-90)
      angle=-90;
    angle = angle/90. * 350;
    angle += 350;
    uint16_t angleSend = angle;
    uint8_t params[] = { (uint8_t)angleSend, (uint8_t)(angleSend>>8), 500&0xff, 500>>8 };
    
    bool ok = writeLX(1, params, sizeof(params));
    printf("Move to %d -> %s\n", angleSend, ok?"OK":"ERR");
  }
 
  //private: 
  LX16ABus * _bus;
  uint8_t _id;
  bool _debug;
  char ch;
};
