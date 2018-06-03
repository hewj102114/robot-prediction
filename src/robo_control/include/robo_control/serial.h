
#include <errno.h>  // Error number definitions
#include <fcntl.h>  // File control definitions
#include <math.h>
#include <stdio.h>    // standard input / output functions
#include <string.h>   // string function definitions
#include <termios.h>  // POSIX terminal control definitionss
#include <unistd.h>   // UNIX standard function definitions

// #define MSGTOMCU_SIZE 7
// #define MSGFROMMCU_SIZE 34
struct RobotMsgToMCU {
  char chassis_mode = 0;
  char gimbal_mode = 0;
  short int velocity_x = 0;
  short int velocity_y = 0;
  short int velocity_yaw = 0;
  short int gimbal_yaw = 0;
  short int gimbal_pitch = 0;
  short int enemy_distance = 0;
  void clear() {
    chassis_mode = 0;
    gimbal_mode=0;
    velocity_x = 0;
    velocity_y = 0;
    velocity_yaw = 0;
    gimbal_yaw = 0;
    gimbal_pitch = 0;
    enemy_distance = 0;
  };
};

struct RobotMsgFromMCU {
  short int remaining_HP = 0;
  short int attack_armorID = 0;
  short int remaining_bullet = 0;
  short int uwb_x = 0;
  short int uwb_y = 0;
  short int gimbal_pitch_angle = 0;
  union union_uwb_yaw{
    float num;
    unsigned char byte[4];
  }uwb_yaw;
  //unsigned char uwb_yaw[4];
  //float uwb_yaw = 0;
  //short int gimbal_chassis_angle = 0;
  short int imu_acceleration_x = 0;
  short int imu_acceleration_y = 0;
  short int imu_acceleration_z = 0;
  short int imu_velocity_x = 0;
  short int imu_velocity_y = 0;
  short int imu_velocity_z = 0;
  short int wheel_odom_x = 0;
  short int wheel_odom_y = 0;
  char uwb_ready_flag = 0;
  char init_flag=0;
  short int bullet_speed=0;
  short int rfid_flag=0;
  short int gimbal_chassis_angle = 0;
};

class Serial {
 public:
  int fd;
  struct termios port_settings;
  Serial(const char* dev_name);
  ~Serial();

  void configurePort();
  bool SendData(struct RobotMsgToMCU msg);
  bool ReadData(struct RobotMsgFromMCU& msg);
};



