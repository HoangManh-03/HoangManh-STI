#ifndef APP_GUI_H
#define APP_GUI_H

#include <QWidget>
#include <QDialog>
#include <QString>
#include <QApplication>
#include <QIcon>
#include <ros/ros.h>
#include <qtimer.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "sti_msgs/HC_info.h"
#include "sti_msgs/POWER_info.h"
#include "sti_msgs/Lift_status.h"
#include "sti_msgs/NN_cmdRequest.h"
#include "sti_msgs/NN_infoRequest.h"
#include "sti_msgs/NN_infoRespond.h"
#include "sti_msgs/Status_goal_control.h"

#include <message_pkg/Status_port.h>
#include <message_pkg/Nav350_data.h>
#include <message_pkg/Server_cmdRequest.h>
#include <message_pkg/Status_launch.h>
#include <message_pkg/Reflector_array.h>
#include <message_pkg/App_button.h>
#include <message_pkg/App_color.h>
#include <message_pkg/Driver_respond.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <bits/stdc++.h>
#include <vector>
#include <thread>
#include <signal.h>
#include <csignal>
#include <ctime>
#include <math.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <arpa/inet.h>
#include <netinet/ether.h>
#include <netdb.h>
#include <cstring>
#include <climits>
#include <fstream>
#include <unistd.h>

using namespace std;

struct Reflector{
    double x = 0;
    double y = 0;
    string localID = "0";
    string globalID = "0";
};

struct statusButon{
    bool bt_passAuto = 0;
    bool bt_passHand = 0;
    bool bt_cancelMission = 0;
    bool bt_tryTarget_hide = 0;
    bool bt_setting = 0;
    bool bt_clearError = 0;

    bool bt_forwards = 0;
    bool bt_backwards = 0;
    bool bt_rotation_left = 0;
    bool bt_rotation_right = 0;
    bool bt_stop = 0;

    bool bt_coorAverage = 0;
    bool bt_disPointA = 0;
    bool bt_disPointB = 0;

    bool bt_chg_on = 0;
    bool bt_chg_off = 0;

    bool bt_spk_on = 0;
    bool bt_spk_off = 0;

    bool bt_disableBrake = 0;

    bool bt_lift_up = 0;
    bool bt_lift_down = 0;
    bool bt_lift_reset = 0;

    bool bt_hideSetting = 0;
    // -
    bool bt_resetFrameWork = 0;
    int vs_speed = 50;

    bool bt_tryTarget_start = 0;
    bool bt_tryTarget_stop = 0;
    bool bt_tryTarget_reset = 1;
    bool ck_tryTarget_safety = 0;
};

struct statusColor{
	// --
    bool lbc_safety_up = 0;
    bool lbc_safety_ahead = 0;
    bool lbc_safety_behind = 0;
    // --
    bool cb_status = 0;
    bool lbc_battery = 0;

    // --
    bool lbc_button_clearError = 0;
    bool lbc_button_power = 0;
    bool lbc_blsock = 0;
    bool lbc_emg = 0;

    bool lbc_limit_up = 0;
    bool lbc_limit_down = 0;
    bool lbc_detect_lifter = 0;

    bool lbc_port_rtc = 0;
    bool lbc_port_rs485 = 0;
    bool lbc_port_nav350 = 0;
};

struct valueLable{
    uint8_t modeRuning = 1;
    string lbv_name_agv = "";
    string lbv_ip = "";
    string lbv_battery = "";
    string lbv_date = "";

    string lbv_coordinates_x = "";
    string lbv_coordinates_y = "";
    string lbv_coordinates_r = "";

    string lbv_numbeReflector = "";
    string lbv_pingServer = "";
    string lbv_jobRuning = "";
    string lbv_goalFollow_id = "";
    
    string lbv_route_target = "";
    string lbv_route_point0 = "";
    string lbv_route_point1 = "";
    string lbv_route_point2 = "";
    string lbv_route_point3 = "";
    string lbv_route_point4 = "";
    string lbv_route_job1 = "";
    string lbv_route_job2 = "";
    string lbv_route_job1_mean = "";
    string lbv_route_job2_mean = "";
    string lbv_route_message = "";

    string lbv_conveyorA = "";
    string lbv_conveyorB = "";

    string lbv_coorAverage_x = "";
    string lbv_coorAverage_y = "";
    string lbv_coorAverage_r = "";
    string lbv_coorAverage_times = "";
    string lbv_deltaDistance = "";

    string lbv_velLeft = "";
    string lbv_velRight = "";

    string lbv_mac = "";
    string lbv_namePc = "";

    string lbv_launhing = "";
    string lbv_numberLaunch = "";
    float percentLaunch = 0;
    vector<string> listError {"A", "B", "C"};                    // <<<<<<<<<<<<<<<<<<<<<<<<<<<

    vector<string> listError_pre;

    string lbv_notification_driver1 = "";
    string lbv_notification_driver2 = "";

    string lbv_qualityWifi = "0";
    // -
    vector<string> list_logError;
    vector<string> list_logError_pre;
    // --
    string lbv_navi_job = "";
    string lbv_navi_type = "";
    string lbv_debug1 = "";
    string lbv_debug2 = "";
    // -
    string lbv_reflectorDetect = "";
    // -
    vector<Reflector> arrReflector;
    string angleCompare = "0";
    
    double lbv_tryTarget_x = 0;
    double lbv_tryTarget_y = 0;
    double lbv_tryTarget_r = 0;
    double lbv_tryTarget_d = 0;
};

namespace Ui {
  class AppGui;
}

class AppGui : public QDialog
{
  Q_OBJECT

public:
  // param app interface
  explicit AppGui(QDialog *parent = nullptr);
  ~AppGui();
  statusButon self_statusButton;
  statusColor self_statusColor;
  valueLable self_valueLable; 

  int setting_status;
  double timeSave_setting;
  string password_data;
  string password_right;

  uint8_t modeRuning;
  uint8_t modeRun_launch;
  uint8_t modeRun_byhand;
  uint8_t modeRun_auto;
  uint8_t modeRun_byhand_tryTarget;
  uint8_t modeRun_cancelMission;

  double timeSave_cancelMisson;
  int16_t cancelMission_status;
  bool isShow_setting;
  bool isShow_reflectorCheck;
  bool isShow_tryTarget;
  
  geometry_msgs::Pose robotPoseNow;
  geometry_msgs::Point pointA;  
  geometry_msgs::Point pointB; 

  bool  bt_coorAverage_status;
  int countTime_coorAverage;
  double total_x;
  double total_y;
  double total_angle;
  bool enable_showToyoWrite;
  double timeSave_showToyoWrite;
  bool isShow_moveHand;
  bool flag_updateShowReflector;
  int changeNow;

  //param of app_ros
  bool is_exist;
  std_msgs::Bool status_brake;
  message_pkg::Driver_respond driver1_respond;  
  message_pkg::Driver_respond driver2_respond;
  sti_msgs::HC_info HC_info;
  sti_msgs::POWER_info main_info;  
  sti_msgs::Lift_status OC_status;
  message_pkg::Status_port status_port;
  message_pkg::Nav350_data nav350_data;
  std_msgs::Int8 safety_NAV;
  geometry_msgs::PoseStamped robotPose_nav;
  message_pkg::Server_cmdRequest server_cmdRequest;
  sti_msgs::NN_cmdRequest NN_cmdRequest;
  sti_msgs::NN_infoRequest NN_infoRequest;
  sti_msgs::NN_infoRespond NN_infoRespond;
  sti_msgs::Status_goal_control status_goalControl; 
  message_pkg::Status_launch status_launch;
  std_msgs::Int16 status_cancelMission;
  message_pkg::Reflector_array nav350_reflectors;
  vector<Reflector> arrReflector;
  std_msgs::Int16 cancelMission_control;
  message_pkg::App_button app_button;   
  message_pkg::App_color pre_app_setColor;

  string name_agv;
  string ip_agv;
  string name_card;
  double pre_timePing;
  string address_traffic;
  bool shutdown_flag = 0;

  ros::NodeHandlePtr nh;
  // subscribe topic 
  ros::Subscriber sub_enable_brake;
  ros::Subscriber sub_driver1_respond;
  ros::Subscriber sub_driver2_respond;
  ros::Subscriber sub_HC_info;
  ros::Subscriber sub_POWER_info;
  ros::Subscriber sub_lift_status;
  ros::Subscriber sub_status_port;
  ros::Subscriber sub_nav350_data;
  ros::Subscriber sub_safety_NAV;
  ros::Subscriber sub_robotPose_nav;
  ros::Subscriber sub_server_cmdRequest;
  ros::Subscriber sub_NN_cmdRequest;
  ros::Subscriber sub_NN_infoRequest;
  ros::Subscriber sub_NN_infoRespond;
  ros::Subscriber sub_status_goal_control;
  ros::Subscriber sub_status_launch;
  ros::Subscriber sub_cancelMission_status;
  ros::Subscriber sub_nav350_reflectors;
  // publish topic
  ros::Publisher  pub_cancelMission;
  ros::Publisher  pub_button;

  void show_reflector();
  void show_combox_unitMeter();
  void show_combox_unitDegree();
  void out();
  double quaternion_to_euler(geometry_msgs::Quaternion qua);
  double calculate_distance(geometry_msgs::Point p1, geometry_msgs::Point p2);
  void set_dateTime();
  void set_valueBattery(string str_value);
  void set_labelColor();
  void set_labelValue();
  void controlShow_followMode();
  void coorAverage_run();
  void show_launch();
  void show_password();

  void anlis_ref2();
  void callback_nav350Reflectors(const message_pkg::Reflector_array data);
  void callback_brakeControl(const std_msgs::Bool data);
  void callback_driver1(const message_pkg::Driver_respond data);
  void callback_driver2(const message_pkg::Driver_respond data);
  void callback_HC(const sti_msgs::HC_info data);
  void callback_Main(const sti_msgs::POWER_info data);
  void callback_OC_board(const sti_msgs::Lift_status data);
  void callback_statusPort(const message_pkg::Status_port data);
  void goalControl_callback(const sti_msgs::Status_goal_control data);
  void callBack_cancelMission(const std_msgs::Int16 data);
  void callback_nav350(const message_pkg::Nav350_data data);  
  void callback_safetyNAV(const std_msgs::Int8 data);  
  void callback_robotPose(const geometry_msgs::PoseStamped data);  
  void callback_server_cmdRequest(const message_pkg::Server_cmdRequest data);
  void NN_cmdRequest_callback(const sti_msgs::NN_cmdRequest data);  
  void callback_NN_infoRequest(const sti_msgs::NN_infoRequest data);
  void infoAGV_callback(const sti_msgs::NN_infoRespond data);
  void callback_statusLaunch(const message_pkg::Status_launch data);
  bool getBit_fromInt16(int16_t value_in, int pos);
  tuple<double, double> convert_position(double distance, double angle);
  string ping_traffic(string address);
  string get_ipAuto(string name_card);
  static void run_screen(int argc, char *argv[]);
  void kill_app();
  string get_MAC(string name_card);
  int get_qualityWifi(string name_card); 
  string get_hostname();
  geometry_msgs::Quaternion euler_to_quaternion(double euler);
  double limitAngle(double angle_in);
  string convert_errorAll(int val);
  string show_job(int val);
  string show_misson(int val);

  void controlColor();
  void controlAll();
  void readButton();
  // static void run();
  void run();

  double round_3decimal(double val_in);
  float round_1decimal(float val_in);
  string convertToString(char* a, int size);

public slots:
  void spinOnce();
  void process_fast();
  void process_normal();
  void process_slow();

private slots:
  void on_bt_controlConveyor_show_released();
  void on_bt_controlConveyor_hide_released();
  void on_bt_exit_pressed();
  void on_bt_cancelMission_pressed();
  void on_bt_cancelMission_released();
  void on_bt_passHand_pressed();
  void on_bt_passHand_released();
  void on_bt_passAuto_pressed();
  void on_bt_passAuto_released();
  void on_bt_clearError_pressed();
  void on_bt_clearError_released();
  void on_bt_speaker_on_clicked();
  void on_bt_speaker_off_clicked();
  void on_bt_charger_on_clicked();
  void on_bt_charger_off_clicked();
  void on_bt_disableBrake_on_clicked(); 
  void on_bt_disableBrake_off_clicked(); 
  void on_bt_forwards_clicked();
  void on_bt_backwards_clicked();
  void on_bt_rotation_left_clicked();
  void on_bt_rotation_right_clicked();
  void on_bt_stop_clicked();
  void on_bt_setting_pressed();
  void on_bt_setting_released();
  void on_bt_hideSetting_clicked();
  void on_bt_pw_cancel_clicked();
  void on_bt_pw_agree_clicked();
  void on_bt_pw_0_clicked();
  void on_bt_pw_1_clicked();
  void on_bt_pw_2_clicked();
  void on_bt_pw_3_clicked();
  void on_bt_pw_4_clicked();
  void on_bt_pw_5_clicked();
  void on_bt_pw_6_clicked();
  void on_bt_pw_7_clicked();
  void on_bt_pw_8_clicked();
  void on_bt_pw_9_clicked();
  void on_bt_pw_clear_clicked();
  void on_bt_pw_delete_clicked();
  void on_bt_lift_up_pressed();
  void on_bt_lift_down_pressed();
  void on_bt_lift_reset_pressed();
  void on_bt_coorAverage_pressed();
  void on_bt_coorAverage_released();
  void on_bt_disPointA_clicked();
  void on_bt_disPointB_clicked();  
  void on_bt_upSpeed_pressed();
  void on_bt_reduceSpeed_pressed();
  void on_bt_resetFrameWork_pressed();
  void on_bt_resetFrameWork_released();
  void on_bt_reflectorCheck_pressed();
  void on_bt_hideNav350_pressed();
  void on_bt_refresh_showRelector_pressed();
  void on_bt_tryTarget_show_pressed();
  void on_bt_tryTarget_show_released();
  void on_bt_tryTarget_hide_pressed();
  void on_bt_tryTarget_hide_released();
  void on_bt_tryTarget_up_pressed();
  void on_bt_tryTarget_up_released();
  void on_bt_tryTarget_down_pressed();
  void on_bt_tryTarget_down_released();
  void on_bt_tryTarget_reset_pressed();
  void on_bt_tryTarget_reset_released();
  void on_bt_tryTarget_start_pressed();
  void on_bt_tryTarget_start_released();
  void on_bt_tryTarget_stop_pressed();
  void on_bt_tryTarget_stop_released();
  void on_bt_tryTarget_x_pressed();
  void on_bt_tryTarget_y_pressed();
  void on_bt_tryTarget_r_pressed();
  void on_bt_tryTarget_d_pressed();

private:
  Ui::AppGui *ui;
  QTimer *timer_fast;
  QTimer *timer_normal;
  QTimer *timer_slow;
  QTimer *ros_timer;
};


#endif // APP_GUI_H
