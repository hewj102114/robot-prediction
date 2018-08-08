#include "robo_control/serial.h"

Serial::Serial(const char* dev_name) {
    fd = open(dev_name, O_RDWR | O_NONBLOCK);  //| O_NONBLOCK  O_NDELAY
    if (fd == -1) {
        printf("open_port: Unable to open /dev/ttyTHS2. \n");
    } else {
        fcntl(fd, F_SETFL, 0);
        printf("%d  port is open.\n", fd);
    }
}
Serial::~Serial() { close(fd); }
void Serial::configurePort() {  // configure the port
  // structure to store the port settings in
  cfsetispeed(&port_settings, B115200);  // set baud rates
  cfsetospeed(&port_settings, B115200);

  port_settings.c_cflag &= ~PARENB;  // set no parity, stop bits, data bits
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;
  port_settings.c_cflag |= CLOCAL | CREAD;
  tcsetattr(fd, TCSANOW, &port_settings);  // apply the settings to the port
}

bool Serial::SendData(struct RobotMsgToMCU msg) {
  unsigned char send_bytes[255] = {0x00};
  send_bytes[0] = 0x7F;
  send_bytes[sizeof(msg) + 1] = 0x7E;
  unsigned char* ptr_send_bytes = send_bytes + 1;
  memcpy(ptr_send_bytes, &msg, sizeof(msg));

  // printf ("%X %X%X %X%X %X%X %X%X %X%X %X%X %X%X
  // %X\n",send_bytes[0],send_bytes[1],send_bytes[2],send_bytes[3],send_bytes[4],send_bytes[5
  // ],send_bytes[6],send_bytes[7],send_bytes[8],send_bytes[9],send_bytes[10],send_bytes[11],send_bytes[12],send_bytes[13],send_bytes[14],send_bytes[15]);
  int len=sizeof(msg)+2;
  if (len == write(fd, send_bytes, len))  // Send data
    return true;
  return false;
}

bool Serial::ReadData(struct RobotMsgFromMCU& msg) {
  char tmp[1] = {0x00};
  char buf[255] = {0x00};
  char* ptr = buf;
  int data_ready = 0;
  int data_start = 0;
  while (!data_ready) {

    int ret = read(fd, tmp, 1);

    if (ret == 0) continue;
    if (tmp[0] == 0x7F) data_start = 1;
    if (data_start) {
      *ptr = tmp[0]; 
      ptr++;
       float  a;
      printf("%d size %X \n",ptr-buf,tmp[0]);
      if (tmp[0] == 0x7E) {
        //printf("%X %X\n",tmp[0],buf[sizeof(msg)+1]);

        if (buf[sizeof(msg)+1] == 0x7E) {
          //printf("ready\n");
          // printf ("%X %X%X %X%X %X%X %X%X %X%X %X%X %X%X
          // %X\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],buf[13],buf[14],buf[15]);

          data_ready = 1;
          memcpy(&msg, buf + 1, sizeof(msg));
           //printf("msg recv:  %d %d %d %d  %d %d  %d\n",msg.remaining_HP,msg.attack_armorID,msg.remaining_bullet,msg.uwb_x,msg.uwb_y,msg.uwb_yaw,msg.gimbal_chassis_angle);
          tcflush(fd, TCIFLUSH);


          //if (msg.uwb_x != 0) 
          return true;
        } else
          return false;
      }
    }
  }
  return false;
}
