#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include <webots/utils/motion.h>

#ifdef _MSC_VER
#define snprintf sprintf_s
#endif

#define MAX_SPEED 4.0

static int time_step = -1;

bool left, right, straight;

static WbDeviceTag us[2];                    // ultra sound sensors
static WbDeviceTag leds[7];                 // controllable led group
static WbDeviceTag LShoulderPitch, RShoulderPitch;
static WbDeviceTag HeadYaw, HeadPitch;
static WbDeviceTag LHipYawPitch, RHipYawPitch;
static WbDeviceTag LHipRoll,LHipPitch,RHipRoll,RHipPitch;
static WbDeviceTag LAnklePitch,LAnkleRoll,RAnklePitch,RAnkleRoll;
static WbDeviceTag LKneePitch,RKneePitch;
static WbDeviceTag lfoot_lbumper, lfoot_rbumper;  // left foot bumpers
static WbDeviceTag rfoot_lbumper, rfoot_rbumper;  // right foot bumpers


// motion file handles
static WbMotionRef hand_wave, stable, ready, move_fast, go_slow;
static WbMotionRef move_back;
static WbMotionRef sturn_left, sturn_right, bturn_left, bturn_right;
static WbMotionRef head_left, head_right;
static WbMotionRef happy, sad, confused;

static WbMotionRef currently_playing = NULL;


static void find_and_enable_devices() {

  // ultrasound sensors
  us[0] = wb_robot_get_device("Sonar/Left");
  us[1] = wb_robot_get_device("Sonar/Right");
  int i;
  for (i = 0; i < 2; i++)
    wb_distance_sensor_enable(us[i], time_step);

  // foot bumpers
  lfoot_lbumper = wb_robot_get_device("LFoot/Bumper/Left");
  lfoot_rbumper = wb_robot_get_device("LFoot/Bumper/Right");
  rfoot_lbumper = wb_robot_get_device("RFoot/Bumper/Left");
  rfoot_rbumper = wb_robot_get_device("RFoot/Bumper/Right");
  wb_touch_sensor_enable(lfoot_lbumper, time_step);
  wb_touch_sensor_enable(lfoot_rbumper, time_step);
  wb_touch_sensor_enable(rfoot_lbumper, time_step);
  wb_touch_sensor_enable(rfoot_rbumper, time_step);

  // There are 7 controlable LED groups in Webots
  leds[0] = wb_robot_get_device("ChestBoard/Led");
  leds[1] = wb_robot_get_device("RFoot/Led");
  leds[2] = wb_robot_get_device("LFoot/Led");
  leds[3] = wb_robot_get_device("Face/Led/Right");
  leds[4] = wb_robot_get_device("Face/Led/Left");
  leds[5] = wb_robot_get_device("Ears/Led/Right");
  leds[6] = wb_robot_get_device("Ears/Led/Left");


  // shoulder pitch motors
  RShoulderPitch = wb_robot_get_device("RShoulderPitch");
  LShoulderPitch = wb_robot_get_device("LShoulderPitch");
  
  HeadYaw = wb_robot_get_device("HeadYaw");
  HeadPitch = wb_robot_get_device("HeadPitch");
  
  LHipYawPitch = wb_robot_get_device("LHipYawPitch");
  RHipYawPitch = wb_robot_get_device("RHipYawPitch");
  
  LHipRoll = wb_robot_get_device("LHipRoll");
  LHipPitch = wb_robot_get_device("LHipPitch");
  RHipRoll = wb_robot_get_device("RHipRoll");
  RHipPitch = wb_robot_get_device("RHipPitch");
  
  LAnklePitch = wb_robot_get_device("LAnklePitch");
  LAnkleRoll = wb_robot_get_device("LAnkleRoll");
  RAnklePitch = wb_robot_get_device("RAnklePitch");
  RAnkleRoll = wb_robot_get_device("RAnkleRoll");
  
  LKneePitch = wb_robot_get_device("LKneePitch");
  RKneePitch = wb_robot_get_device("RKneePitch");
  

  // keyboard
  wb_keyboard_enable(10 * time_step);
}

// load motion files
static void load_motion_files() {
  hand_wave = wbu_motion_new("../../motions/HandWave.motion");
  ready = wbu_motion_new("../../motions/ready.motion");
  stable = wbu_motion_new("../../motions/stable.motion");
  move_fast = wbu_motion_new("../../motions/move_fast.motion");
  go_slow = wbu_motion_new("../../motions/go_slow.motion");
  move_back = wbu_motion_new("../../motions/move_back.motion");
  sturn_left = wbu_motion_new("../../motions/TurnLeft40.motion");
  sturn_right = wbu_motion_new("../../motions/TurnRight40.motion");
  bturn_left = wbu_motion_new("../../motions/TurnLeft60.motion");
  bturn_right = wbu_motion_new("../../motions/TurnRight60.motion");
  head_left = wbu_motion_new("../../motions/head_left.motion");
  head_right = wbu_motion_new("../../motions/head_right.motion");
  happy = wbu_motion_new("../../motions/Happy.motion");
  sad = wbu_motion_new("../../motions/Sad.motion");
  confused = wbu_motion_new("../../motions/think.motion");  
}

static void start_motion(WbMotionRef motion) {
  // interrupt current motion
  if (currently_playing)
    wbu_motion_stop(currently_playing);

  // start new motion
  wbu_motion_play(motion);
  currently_playing = motion;
}

static void loop_motion(WbMotionRef motion,bool a) {
  wbu_motion_set_loop(motion,a);
  if (a) {    
    start_motion(ready);
  }
  else if (! a) {
    wbu_motion_play(stable);
    wbu_motion_stop(motion);
  }
}

static void hit_sad() {
  int ll = (int)wb_touch_sensor_get_value(lfoot_lbumper);
  int lr = (int)wb_touch_sensor_get_value(lfoot_rbumper);
  int rl = (int)wb_touch_sensor_get_value(rfoot_lbumper);
  int rr = (int)wb_touch_sensor_get_value(rfoot_rbumper);
  
  if ((ll==1) || (lr==1) || (rl==1) || (rr==1))
  {  
    printf("|%d\t%d| |%d\t%d|\n", ll, lr, rl, rr);
    loop_motion(move_fast,false);
    loop_motion(go_slow,false);
    wbu_motion_play(sad);
    //wbu_motion_play(sad);
    //wbu_motion_play(backwards);
    //wb_motor_set_position(HeadPitch, 0.4);
    //wb_motor_set_velocity(HeadPitch, MAX_SPEED);    
  }  
}

static void detect_object() {
  double dist[2];
  int i;
  for (i = 0; i < 2; i++)
    dist[i] = wb_distance_sensor_get_value(us[i]);
  //printf("Distance Sensors\n");
  //printf("left: %f m, right %f m\n", dist[0], dist[1]);
  
  straight = (dist[0] <= 2) & (dist[1] <= 2);
  left = (dist[0] <= 0.8) & (dist[1] > 0.8);
  right = (dist[1] <= 0.8) & (dist[0] > 0.8);
  
  if (straight){
    printf("Obstacle Ahead. Please stop\n");
    wb_motor_set_position(HeadYaw, 0);
    wb_motor_set_velocity(HeadYaw, MAX_SPEED);
  }
  else if (left){  
    printf("Left\n");
    wb_motor_set_position(HeadYaw, 1);
    wb_motor_set_velocity(HeadYaw, (0.5*MAX_SPEED));
  }
  else if (right){
    printf("Right\n");
    wb_motor_set_position(HeadYaw, -1);
    wb_motor_set_velocity(HeadYaw, (0.5*MAX_SPEED));
  }
  else{
    wb_motor_set_position(HeadYaw, 0);
    wb_motor_set_velocity(HeadYaw, (0.5*MAX_SPEED));  
  }
}
  
  

static void set_all_leds_color(int rgb) {
  // these leds take RGB values
  int i;
  for (i = 0; i < 5; i++)
    wb_led_set(leds[i], rgb);

  // ear leds are single color (blue)
  // and take values between 0 - 255
  wb_led_set(leds[5], rgb & 0xff);
  wb_led_set(leds[6], rgb & 0xff);
}

static void print_help() {
  printf("----------nao----------\n");
  printf("The 3D window should be focused)\n");
  printf("[Up]: move forward Slow speed\n");
  printf("[Ctrl]+[Up]: move forward Fast speed\n");
  printf("[Down]: move backwards\n");
  printf("[<-][->]: slightly turn left/right (40 degree)\n");
  printf("[Shift]+[<-][->]: turn left/right (60 degree)\n");
  printf("[Alt]+[<-][->]: head movement left/right\n");
  printf("[1]: for happy expression\n");
  printf("[2]: for sad expression\n");
  printf("[3]: for confused and thinking expression\n");
  printf("[L]: look at left side\n");
  printf("[R]: look at right side\n");
  printf("[END]: to stop all motions and stand still\n");
  printf("[H]: print this help message\n");
}

static void terminate() {
  wb_robot_cleanup();
}

static void simulation_step() {
  if (wb_robot_step(time_step) == -1)
    terminate();
  else 
  {
    detect_object();
    hit_sad();
  }
}

static void run_command(int key) {

  switch (key) {
    case WB_KEYBOARD_UP:
      loop_motion(go_slow,true);
      wb_motor_set_position(LHipYawPitch, 0);
      wb_motor_set_position(RHipYawPitch, 0);
      wb_motor_set_velocity(LHipYawPitch, MAX_SPEED);
      wb_motor_set_velocity(RHipYawPitch, MAX_SPEED);
      start_motion(go_slow);
      break;
    case WB_KEYBOARD_DOWN:
      start_motion(move_back);
      break;
    case WB_KEYBOARD_CONTROL | WB_KEYBOARD_UP:
      loop_motion(move_fast,true);
      wb_motor_set_position(LHipYawPitch, 0);
      wb_motor_set_position(RHipYawPitch, 0);
      wb_motor_set_velocity(LHipYawPitch, MAX_SPEED);
      wb_motor_set_velocity(RHipYawPitch, MAX_SPEED);
      start_motion(move_fast);
      break;
    case WB_KEYBOARD_LEFT:
      start_motion(sturn_left);
      break;
    case WB_KEYBOARD_RIGHT:
      start_motion(sturn_right);
      break;
    case WB_KEYBOARD_SHIFT | WB_KEYBOARD_LEFT:
      start_motion(bturn_left);
      break;
    case WB_KEYBOARD_SHIFT | WB_KEYBOARD_RIGHT:
      start_motion(bturn_right);
      break;
    case WB_KEYBOARD_ALT | WB_KEYBOARD_LEFT:
      start_motion(head_left);
      break;
    case WB_KEYBOARD_ALT | WB_KEYBOARD_RIGHT:
      start_motion(head_right);
      break;
    case WB_KEYBOARD_END:
      set_all_leds_color(0x000000); 
      
      /*wb_motor_set_position(LAnklePitch, 0);
      wb_motor_set_position(RAnklePitch, 0);      
      wb_motor_set_position(LAnkleRoll, 0);
      wb_motor_set_position(RAnkleRoll, 0);      
      wb_motor_set_position(LHipRoll, 0);
      wb_motor_set_position(RHipRoll, 0);     
      wb_motor_set_position(LHipPitch, 0);
      wb_motor_set_position(RHipPitch, 0);      
      wb_motor_set_position(LKneePitch, 0);
      wb_motor_set_position(RKneePitch, 0);   */   
      wb_motor_set_position(LHipYawPitch, 0);
      wb_motor_set_position(RHipYawPitch, 0);
            
      /*wb_motor_set_velocity(LAnklePitch, 1);
      wb_motor_set_velocity(RAnklePitch, 1);      
      wb_motor_set_velocity(LAnkleRoll, 1);
      wb_motor_set_velocity(RAnkleRoll, 1);      
      wb_motor_set_velocity(LHipRoll, 1);
      wb_motor_set_velocity(RHipRoll, 1);     
      wb_motor_set_velocity(LHipPitch, 1);
      wb_motor_set_velocity(RHipPitch, 1);      
      wb_motor_set_velocity(LKneePitch, 1);
      wb_motor_set_velocity(RKneePitch, 1); */
      wb_motor_set_velocity(LHipYawPitch, 4.16);
      wb_motor_set_velocity(RHipYawPitch, 4.16);
      start_motion(stable);
      loop_motion(move_fast,false);
      loop_motion(go_slow,false);
      loop_motion(move_back,false);
      break;
    case '1':
      set_all_leds_color(0x00ff00);  // green
      start_motion(happy);
      break;
    case '2':
      set_all_leds_color(0x0000ff);  // blue
      start_motion(sad);
      break;
    case '3':
      set_all_leds_color(0xff0000);  // red
      start_motion(confused);
      break;
    case 'H':
      print_help();
      break;
  }
}

// main function
int main() {
  // call this before any other call to a Webots function
  wb_robot_init();

  // simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();
  //printf("Basic Simulation time: %d",time_step);

  // initialize stuff
  find_and_enable_devices();
  load_motion_files();

  // print instructions
  print_help();

  // walk forwards
  wbu_motion_set_loop(hand_wave, true);
  wbu_motion_play(hand_wave);

  // until a key is pressed
  int key = -1;
  do {
    simulation_step();
    key = wb_keyboard_get_key();
  } while (key >= 0);

  // stop looping this motion
  wbu_motion_set_loop(hand_wave, false);

  // read keyboard and execute user commands
  while (1) {
    if (key >= 0)
      run_command(key);

    simulation_step();
    key = wb_keyboard_get_key();
  }

  return 0;
}
