#include "app_gui.h"
#include "ui_app_gui.h"

AppGui::AppGui(QDialog *parent) :QDialog(parent), ui(new Ui::AppGui)
{
	ui->setupUi(this);

	nh.reset(new ros::NodeHandle("~"));

	// setup the timer that will signal ros stuff to happen
	timer_fast = new QTimer(this);
	connect(timer_fast, SIGNAL(timeout()), this, SLOT(process_fast()));
	timer_fast->start(50);  

	timer_normal = new QTimer(this);
	connect(timer_normal, SIGNAL(timeout()), this, SLOT(process_normal()));
	timer_normal->start(996);  

	//timer_slow = new QTimer(this);
	//connect(timer_slow, SIGNAL(timeout()), this, SLOT(process_slow()));
	//timer_slow->start(3000);  

	ros_timer = new QTimer(this);
	connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
	ros_timer->start(100);

	// setup subscriber 
	sub_enable_brake = nh->subscribe<std_msgs::Bool>("/enable_brake", 10, &AppGui::callback_brakeControl, this );
	sub_driver1_respond = nh->subscribe<message_pkg::Driver_respond>("/driver1_respond", 50, &AppGui::callback_driver1, this );
	sub_driver2_respond = nh->subscribe<message_pkg::Driver_respond>("/driver2_respond", 50, &AppGui::callback_driver2, this ); 
	sub_HC_info = nh->subscribe<sti_msgs::HC_info>("/HC_info", 10, &AppGui::callback_HC, this );
	sub_POWER_info = nh->subscribe<sti_msgs::POWER_info>("/POWER_info", 10, &AppGui::callback_Main, this );
	sub_lift_status = nh->subscribe<sti_msgs::Lift_status>("/lift_status", 10, &AppGui::callback_OC_board, this );
	sub_status_port = nh->subscribe<message_pkg::Status_port>("/status_port", 10, &AppGui::callback_statusPort, this );
	sub_nav350_data = nh->subscribe<message_pkg::Nav350_data>("/nav350_data", 10, &AppGui::callback_nav350, this );
	sub_safety_NAV = nh->subscribe<std_msgs::Int8>("/safety_NAV", 10, &AppGui::callback_safetyNAV, this );
	sub_robotPose_nav = nh->subscribe<geometry_msgs::PoseStamped>("/robotPose_nav", 10, &AppGui::callback_robotPose, this );
	sub_server_cmdRequest = nh->subscribe<message_pkg::Server_cmdRequest>("/server_cmdRequest", 10, &AppGui::callback_server_cmdRequest, this );
	sub_NN_cmdRequest = nh->subscribe<sti_msgs::NN_cmdRequest>("/NN_cmdRequest", 10, &AppGui::NN_cmdRequest_callback, this );
	sub_NN_infoRequest = nh->subscribe<sti_msgs::NN_infoRequest>("/NN_infoRequest", 10, &AppGui::callback_NN_infoRequest, this );
	sub_NN_infoRespond = nh->subscribe<sti_msgs::NN_infoRespond>("/NN_infoRespond", 10, &AppGui::infoAGV_callback, this );
	sub_status_goal_control = nh->subscribe<sti_msgs::Status_goal_control>("/status_goal_control", 10, &AppGui::goalControl_callback, this );
	sub_status_launch = nh->subscribe<message_pkg::Status_launch>("/status_launch", 10, &AppGui::callback_statusLaunch, this );
	sub_cancelMission_status = nh->subscribe<std_msgs::Int16>("/cancelMission_status", 10, &AppGui::callBack_cancelMission, this );
	sub_nav350_reflectors = nh->subscribe<message_pkg::Reflector_array>("/nav350_reflectors", 10, &AppGui::callback_nav350Reflectors, this );
	// setup publisher
	pub_cancelMission = nh->advertise<std_msgs::Int16>("/cancelMission_control", 4);
	pub_button = nh->advertise<message_pkg::App_button>("/app_button", 4);

	setting_status = 0;
	timeSave_setting = ros::Time::now().toSec();
	password_data = "";
	password_right = "0111";
	ui->bt_tryTarget_reset->setStyleSheet("background-color: blue;");
	modeRuning = 0;
	modeRun_launch = 0;
	modeRun_byhand = 1;
	modeRun_auto = 2;
	modeRun_cancelMission = 5;
	modeRuning = modeRun_launch;

	timeSave_cancelMisson = ros::Time::now().toSec();
	cancelMission_status = 0;
	isShow_setting = 0;
	isShow_reflectorCheck = 0;
	isShow_tryTarget = 0;
	bt_coorAverage_status = 0;
	countTime_coorAverage = 0;
	total_x = 0.0;
	total_y = 0.0;
	total_angle = 0.0;
	enable_showToyoWrite = 0;
	timeSave_showToyoWrite = ros::Time::now().toSec();
	isShow_moveHand = 1;
	changeNow = 0;
	ui->lbv_tryTarget_x->setText(QString::fromStdString(to_string(self_valueLable.lbv_tryTarget_x)));
	ui->lbv_tryTarget_y->setText(QString::fromStdString(to_string(self_valueLable.lbv_tryTarget_y)));
	ui->lbv_tryTarget_r->setText(QString::fromStdString(to_string(self_valueLable.lbv_tryTarget_r)));
	ui->lbv_tryTarget_d->setText(QString::fromStdString(to_string(self_valueLable.lbv_tryTarget_d)));

	//QString url_agv = R"(/home/archiep/robot_ws/src/app_ros_cpp/include/app_ros_cpp/AGV.png)";
	QString url_agv = R"(/home/stivietnam/catkin_ws/src/cplus_pkg/app_ros_cpp/include/app_ros_cpp/AGV.png)";
	QPixmap img_agv(url_agv);
	ui->lb_agv->setPixmap(img_agv);

	//QString url_logo = R"(/home/archiep/robot_ws/src/app_ros_cpp/include/app_ros_cpp/logoSti.png)";
	QString url_logo = R"(/home/stivietnam/catkin_ws/src/cplus_pkg/app_ros_cpp/include/app_ros_cpp/logoSti.png)";
	QPixmap img_logo(url_logo);
	ui->lb_logo->setPixmap(img_logo);

	ros::param::get("~name_card", name_card);
	name_card = "wlo2";

	address_traffic = "172.21.15.224";
	ros::param::get("~address_traffic", address_traffic);
	pre_timePing = ros::Time::now().toSec();

	is_exist = 1;
	name_agv = "";
	ip_agv = "";

}

AppGui::~AppGui()
{
  delete ui;
  delete timer_fast;
  delete timer_normal;
  delete timer_slow;
}

void AppGui::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
  }
  else
      QApplication::quit();
}

void AppGui::process_fast(){
//   ui->pb_qualityWifi->setValue(stoi(self_valueLable.lbv_qualityWifi));                      <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,
  ui->pb_speed->setValue(self_statusButton.vs_speed);

  // - combo box
  if (self_valueLable.listError != self_valueLable.listError_pre){
    self_valueLable.listError_pre = self_valueLable.listError;
    ui->cb_status->clear();
    int lg = self_valueLable.listError.size(); 
    for(int i = 0; i < lg; i++){
      ui->cb_status->addItem(QString::fromStdString(self_valueLable.listError[i]));
    }
  }
  // -
  if (self_valueLable.list_logError != self_valueLable.list_logError_pre){
    self_valueLable.list_logError_pre = self_valueLable.list_logError;
    ui->cb_logError->clear();
    int lg = self_valueLable.list_logError.size();
    for(int i = 0; i < lg; i++){
      ui->cb_logError->addItem(QString::fromStdString(self_valueLable.list_logError[i]));
    }
  }
  // - 
  self_statusButton.bt_setting = isShow_setting;
  // -- 
  coorAverage_run();
  // // --
  set_labelValue();
  // // --
  set_labelColor();
  // // --k
  controlShow_followMode();
  // --
  self_statusButton.ck_tryTarget_safety = ui->ck_tryTarget_safety->isChecked();

  // -- show check devices
  if (setting_status == 1){
    double delta_t = ros::Time::now().toSec() - timeSave_setting;
    // -- Chi kich hoat khi dang o che do Bang Tay.
    if (delta_t > 1.5 && self_valueLable.modeRuning == 1){
      isShow_setting = 1;
      isShow_reflectorCheck = 0;
      password_data = "";
    }
  }
  else{
    timeSave_setting = ros::Time::now().toSec();
  }

  // -- add 21/01/2022 - show cancelMission
  if (cancelMission_status == 1){
    double delta_c = ros::Time::now().toSec() - timeSave_cancelMisson;
    if (delta_c > 0.5){
      ui->fr_agv->hide();
      ui->fr_password->show();
      on_bt_stop_clicked();
    }
  }
  else{
    timeSave_cancelMisson = ros::Time::now().toSec();
  }

  // --
  show_password();
  // --

  if (flag_updateShowReflector == 1){
    flag_updateShowReflector = 0;
    int length = self_valueLable.arrReflector.size();
    // print ("length: ", length)
    show_reflector();
  }
}

void AppGui::process_normal(){
	//cout << "Process is running with speed normal " << endl;
	set_dateTime();
	ui->lbv_ip->setText(QString::fromStdString(self_valueLable.lbv_ip));
	ui->lbv_name_agv->setText(QString::fromStdString(self_valueLable.lbv_name_agv));
	ui->lbv_mac->setText(QString::fromStdString(self_valueLable.lbv_mac));
	ui->lbv_namePc->setText(QString::fromStdString(self_valueLable.lbv_namePc));
}

void AppGui::process_slow(){
  set_valueBattery(self_valueLable.lbv_battery);
}

void AppGui::on_bt_controlConveyor_show_released(){
  isShow_moveHand = 0;
  // -
  self_statusButton.bt_rotation_left = 0;
  self_statusButton.bt_forwards = 0;
  self_statusButton.bt_backwards = 0;
  self_statusButton.bt_rotation_right = 0;
  self_statusButton.bt_stop = 1;
  ui->bt_rotation_left->setStyleSheet("background-color: white;");
  ui->bt_forwards->setStyleSheet("background-color: white;");
  ui->bt_backwards->setStyleSheet("background-color: white;");
  ui->bt_rotation_right->setStyleSheet("background-color: white;");
  ui->bt_stop->setStyleSheet("background-color: blue;");
}

void AppGui::on_bt_controlConveyor_hide_released(){
	isShow_moveHand = 1;
}

void AppGui::on_bt_exit_pressed(){
	out();
}

void AppGui::on_bt_cancelMission_pressed(){
  ui->bt_cancelMission->setStyleSheet("background-color: blue;");
  cancelMission_status = 1;
}

void AppGui::on_bt_cancelMission_released(){
  ui->bt_cancelMission->setStyleSheet("background-color: white;");
  cancelMission_status = 0;
  isShow_setting = 0;
}

void AppGui::on_bt_passHand_pressed(){
  self_statusButton.bt_passHand = 1;
  ui->bt_passHand->setStyleSheet("background-color: blue;");
}

void AppGui::on_bt_passHand_released(){
  self_statusButton.bt_passHand = 0;
  ui->bt_passHand->setStyleSheet("background-color: white;");
  isShow_setting = 0;
}

void AppGui::on_bt_passAuto_pressed(){
	self_statusButton.bt_passAuto = 1;
	on_bt_stop_clicked();
	// --
	self_statusButton.bt_disableBrake = 0;
	ui->bt_disableBrake_off->setStyleSheet("background-color: blue;");
	ui->bt_disableBrake_on->setStyleSheet("background-color: white;");
}

void AppGui::on_bt_passAuto_released(){
	self_statusButton.bt_passAuto = 0;
	isShow_setting = 0;
}

void AppGui::on_bt_clearError_pressed(){
	self_statusButton.bt_clearError = 1;
	ui->bt_clearError->setStyleSheet("background-color: blue;");
  on_bt_stop_clicked();
}

void AppGui::on_bt_clearError_released(){
	self_statusButton.bt_clearError = 0;
	ui->bt_clearError->setStyleSheet("background-color: white;");
  on_bt_stop_clicked();
}

void AppGui::on_bt_speaker_on_clicked(){
	self_statusButton.bt_spk_on = 1;
	self_statusButton.bt_spk_off = 0;
	ui->bt_speaker_on->setStyleSheet("background-color: blue;");
	ui->bt_speaker_off->setStyleSheet("background-color: white;");
  on_bt_stop_clicked();
}

void AppGui::on_bt_speaker_off_clicked(){
	self_statusButton.bt_spk_on = 0;
	self_statusButton.bt_spk_off = 1;
	ui->bt_speaker_off->setStyleSheet("background-color: blue;");
	ui->bt_speaker_on->setStyleSheet("background-color: white;");
  on_bt_stop_clicked();
}

void AppGui::on_bt_charger_on_clicked(){
	self_statusButton.bt_chg_on = 1;
	self_statusButton.bt_chg_off = 0;
	ui->bt_charger_on->setStyleSheet("background-color: blue;");
	ui->bt_charger_off->setStyleSheet("background-color: white;");
  on_bt_stop_clicked();
}

void AppGui::on_bt_charger_off_clicked(){
	self_statusButton.bt_chg_on = 0;
	self_statusButton.bt_chg_off = 1;
	ui->bt_charger_off->setStyleSheet("background-color: blue;");
	ui->bt_charger_on->setStyleSheet("background-color: white;");
  on_bt_stop_clicked();
}

void AppGui::on_bt_disableBrake_on_clicked(){
	self_statusButton.bt_disableBrake = 1;
	ui->bt_disableBrake_on->setStyleSheet("background-color: blue;");
	ui->bt_disableBrake_off->setStyleSheet("background-color: white;");
  on_bt_stop_clicked();
}

void AppGui::on_bt_disableBrake_off_clicked(){
	self_statusButton.bt_disableBrake = 0;
	ui->bt_disableBrake_off->setStyleSheet("background-color: blue;");
	ui->bt_disableBrake_on->setStyleSheet("background-color: white;");
  	on_bt_stop_clicked();
}

void AppGui::on_bt_forwards_clicked(){
	self_statusButton.bt_forwards = 1;
	self_statusButton.bt_backwards = 0;
	self_statusButton.bt_rotation_left = 0;
	self_statusButton.bt_rotation_right = 0;
	self_statusButton.bt_stop = 0;
	ui->bt_forwards->setStyleSheet("background-color: blue;");
	ui->bt_backwards->setStyleSheet("background-color: white;");
	ui->bt_rotation_left->setStyleSheet("background-color: white;");
	ui->bt_rotation_right->setStyleSheet("background-color: white;");
	ui->bt_stop->setStyleSheet("background-color: white;");
}

void AppGui::on_bt_backwards_clicked(){
	self_statusButton.bt_forwards = 0;
	self_statusButton.bt_backwards = 1;
	self_statusButton.bt_rotation_left = 0;
	self_statusButton.bt_rotation_right = 0;
	self_statusButton.bt_stop = 0;
	ui->bt_forwards->setStyleSheet("background-color: white;");
	ui->bt_backwards->setStyleSheet("background-color: blue;");
	ui->bt_rotation_left->setStyleSheet("background-color: white;");
	ui->bt_rotation_right->setStyleSheet("background-color: white;");
	ui->bt_stop->setStyleSheet("background-color: white;");
}

void AppGui::on_bt_rotation_left_clicked(){
	self_statusButton.bt_forwards = 0;
	self_statusButton.bt_backwards = 0;
	self_statusButton.bt_rotation_left = 1;
	self_statusButton.bt_rotation_right = 0;
	self_statusButton.bt_stop = 0;
	ui->bt_forwards->setStyleSheet("background-color: white;");
	ui->bt_backwards->setStyleSheet("background-color: white;");
	ui->bt_rotation_left->setStyleSheet("background-color: blue;");
	ui->bt_rotation_right->setStyleSheet("background-color: white;");
	ui->bt_stop->setStyleSheet("background-color: white;");
}

void AppGui::on_bt_rotation_right_clicked(){
	self_statusButton.bt_forwards = 0;
	self_statusButton.bt_backwards = 0;
	self_statusButton.bt_rotation_left = 0;
	self_statusButton.bt_rotation_right = 1;
	self_statusButton.bt_stop = 0;
	ui->bt_forwards->setStyleSheet("background-color: white;");
	ui->bt_backwards->setStyleSheet("background-color: white;");
	ui->bt_rotation_left->setStyleSheet("background-color: white;");
	ui->bt_rotation_right->setStyleSheet("background-color: blue;");
	ui->bt_stop->setStyleSheet("background-color: white;");
}

void AppGui::on_bt_stop_clicked(){
	self_statusButton.bt_forwards = 0;
	self_statusButton.bt_backwards = 0;
	self_statusButton.bt_rotation_left = 0;
	self_statusButton.bt_rotation_right = 0;
	self_statusButton.bt_stop = 1;
	ui->bt_forwards->setStyleSheet("background-color: white;");
	ui->bt_backwards->setStyleSheet("background-color: white;");
	ui->bt_rotation_left->setStyleSheet("background-color: white;");
	ui->bt_rotation_right->setStyleSheet("background-color: white;");
	ui->bt_stop->setStyleSheet("background-color: blue;");
}

void AppGui::on_bt_setting_pressed(){
	ui->bt_setting->setStyleSheet("background-color: blue;");	
  setting_status = 1;
}

void AppGui::on_bt_setting_released(){
	ui->bt_setting->setStyleSheet("background-color: white;");	
	setting_status = 0;
}

void AppGui::on_bt_hideSetting_clicked(){
	isShow_setting = 0;
	password_data = "";
	enable_showToyoWrite = 0;
}

void AppGui::on_bt_pw_cancel_clicked(){
	ui->fr_agv->show();
	ui->fr_password->hide();
	password_data = "";
}

void AppGui::on_bt_pw_agree_clicked(){
	ui->fr_agv->show();
	ui->fr_password->hide();
	self_statusButton.bt_cancelMission = 1;
	password_data = "";
}

void AppGui::on_bt_pw_0_clicked(){
	if (password_data.length() < 4)
		password_data.push_back('0');
}

void AppGui::on_bt_pw_1_clicked(){
	if (password_data.length() < 4)
		password_data.push_back('1');
}

void AppGui::on_bt_pw_2_clicked(){
	if (password_data.length() < 4)
		password_data.push_back('2');
}

void AppGui::on_bt_pw_3_clicked(){
	if (password_data.length() < 4)
		password_data.push_back('3');
}

void AppGui::on_bt_pw_4_clicked(){
	if (password_data.length() < 4)
		password_data.push_back('4');
}

void AppGui::on_bt_pw_5_clicked(){
	if (password_data.length() < 4)
		password_data.push_back('5');
}

void AppGui::on_bt_pw_6_clicked(){
	if (password_data.length() < 4)
		password_data.push_back('6');
}

void AppGui::on_bt_pw_7_clicked(){
	if (password_data.length() < 4)
		password_data.push_back('7');
}

void AppGui::on_bt_pw_8_clicked(){
	if (password_data.length() < 4)
		password_data.push_back('8');
}

void AppGui::on_bt_pw_9_clicked(){
	if (password_data.length() < 4)
		password_data.push_back('9');
}

void AppGui::on_bt_pw_clear_clicked(){
	password_data = "";
}

void AppGui::on_bt_pw_delete_clicked(){
	int lenght = password_data.length();
	if (lenght > 0)
		password_data.pop_back();
}

void AppGui::on_bt_lift_up_pressed(){
	ui->bt_lift_up->setStyleSheet("background-color: blue;");
	ui->bt_lift_down->setStyleSheet("background-color: white;");
	ui->bt_lift_reset->setStyleSheet("background-color: white;");
	self_statusButton.bt_lift_up = 1;
	self_statusButton.bt_lift_down = 0;
}

void AppGui::on_bt_lift_down_pressed(){
	ui->bt_lift_up->setStyleSheet("background-color: white;");
	ui->bt_lift_down->setStyleSheet("background-color: blue;");
	ui->bt_lift_reset->setStyleSheet("background-color: white;");
	self_statusButton.bt_lift_up = 0;
	self_statusButton.bt_lift_down = 1;
}

void AppGui::on_bt_lift_reset_pressed(){
	ui->bt_lift_up->setStyleSheet("background-color: white;");
	ui->bt_lift_down->setStyleSheet("background-color: white;");
	ui->bt_lift_reset->setStyleSheet("background-color: blue;");
	self_statusButton.bt_lift_up = 0;
	self_statusButton.bt_lift_down = 0;
}

void AppGui::on_bt_coorAverage_pressed(){
	ui->bt_coorAverage->setStyleSheet("background-color: blue;");
	bt_coorAverage_status = 1;
}

void AppGui::on_bt_coorAverage_released(){
	ui->bt_coorAverage->setStyleSheet("background-color: white;");
	bt_coorAverage_status = 0;
}

void AppGui::on_bt_disPointA_clicked(){
	pointA.x = robotPoseNow.position.x;
	pointA.y = robotPoseNow.position.y;
	ui->lbv_deltaDistance->setText("---");
}

void AppGui::on_bt_disPointB_clicked(){
	pointB.x = robotPoseNow.position.x;
	pointB.y = robotPoseNow.position.y;
	double delta_distance = calculate_distance(pointA, pointB);
	ui->lbv_deltaDistance->setText(QString::fromStdString(to_string(round_3decimal(delta_distance))));
	if (ui->ck_linkOffset->isChecked() == 1){
		self_valueLable.lbv_tryTarget_d = delta_distance;
		ui->lbv_tryTarget_d->setText(QString::fromStdString(to_string(round_3decimal(self_valueLable.lbv_tryTarget_d))));
  }
}

void AppGui::on_bt_upSpeed_pressed(){
	self_statusButton.vs_speed += 10;
	if (self_statusButton.vs_speed >= 100)
		self_statusButton.vs_speed = 100;

	//cout << "Nút tăng tốc được nhấn " << "vs_speed= " << self_statusButton.vs_speed << endl;
}

void AppGui::on_bt_reduceSpeed_pressed(){
	self_statusButton.vs_speed -= 10;
	if (self_statusButton.vs_speed < 5)
		self_statusButton.vs_speed = 5;
	//cout << "Nút tăng tốc được nhấn " << "vs_speed= " << self_statusButton.vs_speed << endl;
}

void AppGui::on_bt_resetFrameWork_pressed(){
	ui->bt_resetFrameWork->setStyleSheet("background-color: blue;");
	self_statusButton.bt_resetFrameWork = 1;
}

void AppGui::on_bt_resetFrameWork_released(){
	ui->bt_resetFrameWork->setStyleSheet("background-color: white;");
	self_statusButton.bt_resetFrameWork = 0;
}

void AppGui::on_bt_reflectorCheck_pressed(){
	isShow_reflectorCheck = 1;
	isShow_setting = 0;
}

void AppGui::on_bt_hideNav350_pressed(){
	isShow_reflectorCheck = 0;
	isShow_setting = 1;
}

void AppGui::on_bt_refresh_showRelector_pressed(){
	flag_updateShowReflector = 1;
}

void AppGui::on_bt_tryTarget_show_pressed(){
	ui->bt_tryTarget_show->setStyleSheet("background-color: blue;");
	isShow_tryTarget = 1;
  	on_bt_stop_clicked();
}

void AppGui::on_bt_tryTarget_show_released(){
	ui->bt_tryTarget_show->setStyleSheet("background-color: white;");
  	on_bt_stop_clicked();
}

void AppGui::on_bt_tryTarget_hide_pressed(){
	ui->bt_tryTarget_show->setStyleSheet("background-color: blue;");
  	on_bt_stop_clicked();
}

void AppGui::on_bt_tryTarget_hide_released(){
	ui->bt_tryTarget_show->setStyleSheet("background-color: white;");
	isShow_tryTarget = 0;
  	on_bt_stop_clicked();
	self_statusButton.bt_tryTarget_start = 0;
	self_statusButton.bt_tryTarget_stop = 0;
	self_statusButton.bt_tryTarget_reset = 1;
	ui->bt_tryTarget_reset->setStyleSheet("background-color: blue;");
	ui->bt_tryTarget_stop->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_start->setStyleSheet("background-color: white;");
}

void AppGui::on_bt_tryTarget_up_pressed(){
	ui->bt_tryTarget_up->setStyleSheet("background-color: blue;");
	if (ui->cb_unit->currentText() != ""){
		float val_change = stof(ui->cb_unit->currentText().toStdString());
		// - X
		if (changeNow == 1)
			self_valueLable.lbv_tryTarget_x += val_change;
		// - Y
		if (changeNow == 2)
			self_valueLable.lbv_tryTarget_y += val_change;
		// - R
		if (changeNow == 3)
			self_valueLable.lbv_tryTarget_r += val_change;
		// - D
		if (changeNow == 4)
			self_valueLable.lbv_tryTarget_d += val_change;
	}
}

void AppGui::on_bt_tryTarget_up_released(){
	ui->bt_tryTarget_up->setStyleSheet("background-color: white;");
	ui->lbv_tryTarget_x->setText(QString::fromStdString(to_string(round_3decimal(self_valueLable.lbv_tryTarget_x))));
	ui->lbv_tryTarget_y->setText(QString::fromStdString(to_string(round_3decimal(self_valueLable.lbv_tryTarget_y))));
	ui->lbv_tryTarget_r->setText(QString::fromStdString(to_string(round_3decimal(self_valueLable.lbv_tryTarget_r))));
	ui->lbv_tryTarget_d->setText(QString::fromStdString(to_string(round_3decimal(self_valueLable.lbv_tryTarget_d))));
}

void AppGui::on_bt_tryTarget_down_pressed(){
	ui->bt_tryTarget_down->setStyleSheet("background-color: blue;");
	if (ui->cb_unit->currentText() != ""){
		float val_change = stof(ui->cb_unit->currentText().toStdString());
		// - X
		if (changeNow == 1)
			self_valueLable.lbv_tryTarget_x -= val_change;
		// - Y
		if (changeNow == 2)
			self_valueLable.lbv_tryTarget_y -= val_change;
		// - R
		if (changeNow == 3)
			self_valueLable.lbv_tryTarget_r -= val_change;
		// - D
		if (changeNow == 4)
			self_valueLable.lbv_tryTarget_d -= val_change;
	}
}

void AppGui::on_bt_tryTarget_down_released(){
	ui->bt_tryTarget_down->setStyleSheet("background-color: white;");
	ui->lbv_tryTarget_x->setText(QString::fromStdString(to_string(round_3decimal(self_valueLable.lbv_tryTarget_x))));
	ui->lbv_tryTarget_y->setText(QString::fromStdString(to_string(round_3decimal(self_valueLable.lbv_tryTarget_y))));
	ui->lbv_tryTarget_r->setText(QString::fromStdString(to_string(round_3decimal(self_valueLable.lbv_tryTarget_r))));
	ui->lbv_tryTarget_d->setText(QString::fromStdString(to_string(round_3decimal(self_valueLable.lbv_tryTarget_d))));
}

void AppGui::on_bt_tryTarget_reset_pressed(){
	ui->bt_tryTarget_reset->setStyleSheet("background-color: blue;");
	self_statusButton.bt_tryTarget_reset = 1;
}

void AppGui::on_bt_tryTarget_reset_released(){
	ui->bt_tryTarget_start->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_stop->setStyleSheet("background-color: white;");
	self_statusButton.bt_tryTarget_start = 0;
	self_statusButton.bt_tryTarget_stop = 0;
}

void AppGui::on_bt_tryTarget_start_pressed(){
	ui->bt_tryTarget_start->setStyleSheet("background-color: blue;");
	self_statusButton.bt_tryTarget_start = 1;
}

void AppGui::on_bt_tryTarget_start_released(){
	ui->bt_tryTarget_reset->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_stop->setStyleSheet("background-color: white;");
	self_statusButton.bt_tryTarget_reset = 0;
	self_statusButton.bt_tryTarget_stop = 0;
}

void AppGui::on_bt_tryTarget_stop_pressed(){
	ui->bt_tryTarget_stop->setStyleSheet("background-color: blue;");
	self_statusButton.bt_tryTarget_stop = 1;
}

void AppGui::on_bt_tryTarget_stop_released(){
	ui->bt_tryTarget_reset->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_start->setStyleSheet("background-color: white;");
	self_statusButton.bt_tryTarget_reset = 0;
	self_statusButton.bt_tryTarget_start = 0;
}

void AppGui::on_bt_tryTarget_x_pressed(){
	ui->bt_tryTarget_x->setStyleSheet("background-color: blue;");
	ui->bt_tryTarget_y->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_r->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_d->setStyleSheet("background-color: white;");
	show_combox_unitMeter();
	changeNow = 1;
}

void AppGui::on_bt_tryTarget_y_pressed(){
	ui->bt_tryTarget_x->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_y->setStyleSheet("background-color: blue;");
	ui->bt_tryTarget_r->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_d->setStyleSheet("background-color: white;");
	show_combox_unitMeter();
	changeNow = 2;
}

void AppGui::on_bt_tryTarget_r_pressed(){
	ui->bt_tryTarget_x->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_y->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_r->setStyleSheet("background-color: blue;");
	ui->bt_tryTarget_d->setStyleSheet("background-color: white;");
	show_combox_unitMeter();
	changeNow = 3;
}

void AppGui::on_bt_tryTarget_d_pressed(){
	ui->bt_tryTarget_x->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_y->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_r->setStyleSheet("background-color: white;");
	ui->bt_tryTarget_d->setStyleSheet("background-color: blue;");
	show_combox_unitMeter();
	changeNow = 4;
}


void AppGui::show_reflector(){
	self_valueLable.angleCompare = ui->dial_angleCompare->value();
	// --
	ui->lb_rf_0->move(0, 0);
	ui->lb_rf_1->move(0, 20);
	ui->lb_rf_2->move(0, 40);
	ui->lb_rf_3->move(0, 60);
	ui->lb_rf_4->move(0, 80);
	ui->lb_rf_5->move(0, 100);
	ui->lb_rf_6->move(0, 120);
	ui->lb_rf_7->move(0, 140);
	ui->lb_rf_8->move(0, 160);
	ui->lb_rf_9->move(0, 180);
	ui->lb_rf_10->move(0, 200);
	ui->lb_rf_11->move(0, 220);
	// --

	int length = self_valueLable.arrReflector.size();
	if (length > 0){
		ui->lb_rf_0->setText(QString::fromStdString(self_valueLable.arrReflector[0].globalID)); // localID
		ui->lb_rf_0->move(self_valueLable.arrReflector[0].x, self_valueLable.arrReflector[0].y);
		if (self_valueLable.arrReflector[0].globalID == "-1")
			ui->lb_rf_0->setStyleSheet("border: 1px solid red; border-radius: 10px");
		else{
			ui->lb_rf_0->setStyleSheet("border: 1px solid green; border-radius: 10px");
		}
	}

	if (length > 1){
		ui->lb_rf_1->setText(QString::fromStdString(self_valueLable.arrReflector[1].globalID));
		ui->lb_rf_1->move(self_valueLable.arrReflector[1].x, self_valueLable.arrReflector[1].y);
		if (self_valueLable.arrReflector[1].globalID == "-1")
			ui->lb_rf_1->setStyleSheet("border: 1px solid red; border-radius: 10px");
		else{
			ui->lb_rf_1->setStyleSheet("border: 1px solid green; border-radius: 10px");
		}
	}

	if (length > 2){
		ui->lb_rf_2->setText(QString::fromStdString(self_valueLable.arrReflector[2].globalID));
		ui->lb_rf_2->move(self_valueLable.arrReflector[2].x, self_valueLable.arrReflector[2].y);
		if (self_valueLable.arrReflector[2].globalID == "-1")
			ui->lb_rf_2->setStyleSheet("border: 1px solid red; border-radius: 10px");
		else{
			ui->lb_rf_2->setStyleSheet("border: 1px solid green; border-radius: 10px");
		}
	}

	if (length > 3){
		ui->lb_rf_3->setText(QString::fromStdString(self_valueLable.arrReflector[3].globalID));
		ui->lb_rf_3->move(self_valueLable.arrReflector[3].x, self_valueLable.arrReflector[3].y);
		if (self_valueLable.arrReflector[3].globalID == "-1")
			ui->lb_rf_3->setStyleSheet("border: 1px solid red; border-radius: 10px");
		else{
			ui->lb_rf_3->setStyleSheet("border: 1px solid green; border-radius: 10px");
		}
	}

	if (length > 4){
		ui->lb_rf_4->setText(QString::fromStdString(self_valueLable.arrReflector[4].globalID));
		ui->lb_rf_4->move(self_valueLable.arrReflector[4].x, self_valueLable.arrReflector[4].y);
		if (self_valueLable.arrReflector[4].globalID == "1")
			ui->lb_rf_4->setStyleSheet("border: 1px solid red; border-radius: 10px");
		else{
			ui->lb_rf_4->setStyleSheet("border: 1px solid green; border-radius: 10px");
		}
	}

	if (length > 5){
		ui->lb_rf_5->setText(QString::fromStdString(self_valueLable.arrReflector[5].globalID));
		ui->lb_rf_5->move(self_valueLable.arrReflector[5].x, self_valueLable.arrReflector[5].y);
		if (self_valueLable.arrReflector[5].globalID == "-1")
			ui->lb_rf_5->setStyleSheet("border: 1px solid red; border-radius: 10px");
		else{
			ui->lb_rf_5->setStyleSheet("border: 1px solid green; border-radius: 10px");
		}
	}

	if (length > 6){
		ui->lb_rf_6->setText(QString::fromStdString(self_valueLable.arrReflector[6].globalID));
		ui->lb_rf_6->move(self_valueLable.arrReflector[6].x, self_valueLable.arrReflector[6].y);
		if (self_valueLable.arrReflector[6].globalID == "-1")
			ui->lb_rf_6->setStyleSheet("border: 1px solid red; border-radius: 10px");
		else{
			ui->lb_rf_6->setStyleSheet("border: 1px solid green; border-radius: 10px");
		}
	}

	if (length > 7){
		ui->lb_rf_7->setText(QString::fromStdString(self_valueLable.arrReflector[7].globalID));
		ui->lb_rf_7->move(self_valueLable.arrReflector[7].x, self_valueLable.arrReflector[7].y);
		if (self_valueLable.arrReflector[7].globalID == "-1")
			ui->lb_rf_7->setStyleSheet("border: 1px solid red; border-radius: 10px");
		else{
			ui->lb_rf_7->setStyleSheet("border: 1px solid green; border-radius: 10px");
		}
	}

	if (length > 8){
		ui->lb_rf_8->setText(QString::fromStdString(self_valueLable.arrReflector[8].globalID));
		ui->lb_rf_8->move(self_valueLable.arrReflector[8].x, self_valueLable.arrReflector[8].y);
		if (self_valueLable.arrReflector[8].globalID == "-1")
			ui->lb_rf_8->setStyleSheet("border: 1px solid red; border-radius: 10px");
		else{
			ui->lb_rf_8->setStyleSheet("border: 1px solid green; border-radius: 10px");
		}
	}

	if (length > 9){
		ui->lb_rf_9->setText(QString::fromStdString(self_valueLable.arrReflector[9].globalID));
		ui->lb_rf_9->move(self_valueLable.arrReflector[9].x, self_valueLable.arrReflector[9].y);
		if (self_valueLable.arrReflector[9].globalID == "-1")
			ui->lb_rf_9->setStyleSheet("border: 1px solid red; border-radius: 10px");
		else{
			ui->lb_rf_9->setStyleSheet("border: 1px solid green; border-radius: 10px");
		}
	}

	if (length > 10){
		ui->lb_rf_10->setText(QString::fromStdString(self_valueLable.arrReflector[10].globalID));
		ui->lb_rf_10->move(self_valueLable.arrReflector[10].x, self_valueLable.arrReflector[10].y);
		if (self_valueLable.arrReflector[10].globalID == "-1")
			ui->lb_rf_10->setStyleSheet("border: 1px solid red; border-radius: 10px");
		else{
			ui->lb_rf_10->setStyleSheet("border: 1px solid green; border-radius: 10px");
		}
	}

	if (length > 11){
		ui->lb_rf_11->setText(QString::fromStdString(self_valueLable.arrReflector[11].globalID));
		ui->lb_rf_11->move(self_valueLable.arrReflector[11].x, self_valueLable.arrReflector[11].y);
		if (self_valueLable.arrReflector[11].globalID == "-1")
			ui->lb_rf_11->setStyleSheet("border: 1px solid red; border-radius: 10px");
		else{
			ui->lb_rf_11->setStyleSheet("border: 1px solid green; border-radius: 10px");
		}
	}

}


void AppGui::show_combox_unitMeter(){
	ui->cb_unit->clear();
	ui->lbv_unit->setText("m");
	ui->cb_unit->addItem("0.001");
	ui->cb_unit->addItem("0.002");
	ui->cb_unit->addItem("0.005");
	ui->cb_unit->addItem("0.01");
	ui->cb_unit->addItem("0.1");
	ui->cb_unit->addItem("1");
	ui->cb_unit->addItem("2");
	ui->cb_unit->addItem("5");
}

void AppGui::show_combox_unitDegree(){
	ui->cb_unit->clear();
	ui->lbv_unit->setText("Độ");
	ui->cb_unit->addItem("0.01");
	ui->cb_unit->addItem("0.1");
	ui->cb_unit->addItem("0.2");
	ui->cb_unit->addItem("0.5");
	ui->cb_unit->addItem("1");
	ui->cb_unit->addItem("2");
	ui->cb_unit->addItem("5");
}

void AppGui::out(){
	QApplication::quit();
	shutdown_flag = 1;
	is_exist = 0;
	cout << "out" << endl;
}

double AppGui::quaternion_to_euler(geometry_msgs::Quaternion qua){
	tf::Quaternion quat(qua.x, qua.y, qua.z, qua.w);
	tf::Matrix3x3 m(quat);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}

double AppGui::calculate_distance(geometry_msgs::Point p1, geometry_msgs::Point p2){
	double x = p2.x - p1.x;
	double y = p2.y - p1.y;
	return sqrt(x*x + y*y);
}

//void AppGui::set_dateTime(){
//	std::ifstream file("/proc/uptime");
//
//	double uptimeSeconds;
//	file >> uptimeSeconds;
//
//	std::time_t lastBootTime = std::time(nullptr) - (std::time_t) uptimeSeconds;
//	char buf[20];
//	strftime(buf, sizeof(buf), "%d-%m-%Y\n%H:%M:%S", localtime(&lastBootTime));
//
//	int dt_size = sizeof(buf) / sizeof(char);
//	string s_dt = convertToString(buf, dt_size);
//	//cout << s_dt << endl;
//	ui->lbv_date->setText(QString::fromStdString(s_dt));
//}

void AppGui::set_dateTime(){
    time_t now = time(0);
    tm *ltm = localtime(&now);	

    char buf[20];
    sprintf(buf, "%d/%d/%d\n%d:%d:%d", ltm->tm_mday + 1, ltm->tm_mon + 1, 1900 + ltm->tm_year , \
   										ltm->tm_hour, ltm->tm_min, ltm->tm_sec);

	int dt_size = sizeof(buf) / sizeof(char);
	string s_dt = convertToString(buf, dt_size);
    // cout << s_dt << endl;
	ui->lbv_date->setText(QString::fromStdString(s_dt));	
}

void AppGui::set_valueBattery(string str_value){
	ui->lbv_battery->setText(QString::fromStdString(str_value));
}

void AppGui::set_labelColor(){
	// ---- Safety
	if (self_statusColor.lbc_safety_up == 0){
		ui->lbc_safety0->setStyleSheet("background-color: green; color: white;");
		ui->lbc_safety_up->setStyleSheet("background-color: green; color: white");
	}
	else if (self_statusColor.lbc_safety_up == 1){
		ui->lbc_safety0->setStyleSheet("background-color: red; color: white");
		ui->lbc_safety_up->setStyleSheet("background-color: red; color: white");
	}
	else{
		ui->lbc_safety0->setStyleSheet("background-color: yellow;");
		ui->lbc_safety_up->setStyleSheet("background-color: yellow;");
	}
	// -
	if (self_statusColor.lbc_safety_ahead == 0){
		ui->lbc_safety1->setStyleSheet("background-color: green; color: white");
		ui->lbc_safety_ahead->setStyleSheet("background-color: green; color: white");
	}
	else if (self_statusColor.lbc_safety_ahead == 1){
		ui->lbc_safety1->setStyleSheet("background-color: red; color: white");
		ui->lbc_safety_ahead->setStyleSheet("background-color: red; color: white");
	}
	else{
		ui->lbc_safety1->setStyleSheet("background-color: yellow;");
		ui->lbc_safety_ahead->setStyleSheet("background-color: yellow;");
	}
	// -
	if (self_statusColor.lbc_safety_behind == 0){
		ui->lbc_safety2->setStyleSheet("background-color: green; color: white");
		ui->lbc_safety_behind->setStyleSheet("background-color: green; color: white");
	}
	else if (self_statusColor.lbc_safety_behind == 1){
		ui->lbc_safety2->setStyleSheet("background-color: red; color: white");
		ui->lbc_safety_behind->setStyleSheet("background-color: red; color: white");
	}
	else{
		ui->lbc_safety2->setStyleSheet("background-color: yellow;");
		ui->lbc_safety_behind->setStyleSheet("background-color: yellow;");
	}

	// ---- Battery
	if (self_statusColor.lbc_battery == 0){
		ui->lbv_battery->setStyleSheet("background-color: white; color: black;");
		ui->lb_v->setStyleSheet("color: black;");
	}
	else if (self_statusColor.lbc_battery == 1){
		ui->lbv_battery->setStyleSheet("background-color: green; color: white;");
		ui->lb_v->setStyleSheet("color: white;");
	}
	else if (self_statusColor.lbc_battery == 2){
		ui->lbv_battery->setStyleSheet("background-color: orange; color: black;");
		ui->lb_v->setStyleSheet("color: black;");
	}
	else if (self_statusColor.lbc_battery == 3){
		ui->lbv_battery->setStyleSheet("background-color: red; color: white;");
		ui->lb_v->setStyleSheet("color: white;");
	}
	else if (self_statusColor.lbc_battery == 4){
		ui->lbv_battery->setStyleSheet("background-color: yellow; color: black;");
		ui->lb_v->setStyleSheet("color: black;");
	}
	else{ // -- charging
		ui->lbv_battery->setStyleSheet("background-color: white; color: black;");
		ui->lb_v->setStyleSheet("color: black;");
	}

	// ---- Thanh trang thai AGV
	if (self_statusColor.cb_status == 0)
		ui->cb_status->setStyleSheet("background-color: green; color: white;");
	else if (self_statusColor.cb_status == 1)
		ui->cb_status->setStyleSheet("background-color: orange; color: black;");
	else if (self_statusColor.cb_status == 2)
		ui->cb_status->setStyleSheet("background-color: red; color: white;");
	else{
		ui->cb_status->setStyleSheet("background-color: white;");
	}

	// ---- Trang thai che do dang hoat dong
	if (self_valueLable.modeRuning == modeRun_byhand){
		ui->bt_passHand->setStyleSheet("background-color: blue;");	
		ui->bt_passAuto->setStyleSheet("background-color: white;");
	}	
	else if (self_valueLable.modeRuning == modeRun_auto){
		ui->bt_passHand->setStyleSheet("background-color: white;");	
		ui->bt_passAuto->setStyleSheet("background-color: blue;");
	}
	else{	
		ui->bt_passHand->setStyleSheet("background-color: white;");	
		ui->bt_passAuto->setStyleSheet("background-color: white;");
	}

	// -- Button Clear error
	if (self_statusColor.lbc_button_clearError == 1)
		ui->lbc_button_clearError->setStyleSheet("background-color: blue;");
	else if (self_statusColor.lbc_button_clearError == 0)
		ui->lbc_button_clearError->setStyleSheet("background-color: white;");

	// -- Button Power
	if (self_statusColor.lbc_button_power == 1)
		ui->lbc_button_power->setStyleSheet("background-color: blue;");
	else if (self_statusColor.lbc_button_power == 0)
		ui->lbc_button_power->setStyleSheet("background-color: white;");

	// -- Blsock
	if (self_statusColor.lbc_blsock == 1)
		ui->lbc_blsock->setStyleSheet("background-color: blue;");
	else if (self_statusColor.lbc_blsock == 0)
		ui->lbc_blsock->setStyleSheet("background-color: white;");

	// -- EMG
	if (self_statusColor.lbc_emg == 1)
		ui->lbc_emg->setStyleSheet("background-color: blue;");
	else if (self_statusColor.lbc_emg == 0)
		ui->lbc_emg->setStyleSheet("background-color: white;");	

	// -- Port: RTC Board
	if (self_statusColor.lbc_port_rtc == 1)
		ui->lbc_port_rtc->setStyleSheet("background-color: blue; color: white");
	else if (self_statusColor.lbc_port_rtc == 0)
		ui->lbc_port_rtc->setStyleSheet("background-color: red; color: white;");

	// -- Port: RS485
	if (self_statusColor.lbc_port_rs485 == 1)
		ui->lbc_port_rs485->setStyleSheet("background-color: blue; color: white");
	else if (self_statusColor.lbc_port_rs485 == 0)
		ui->lbc_port_rs485->setStyleSheet("background-color: red; color: white;");

	// -- Port: NAV350
	if (self_statusColor.lbc_port_nav350 == 1)
		ui->lbc_port_nav350->setStyleSheet("background-color: blue; color: white");
	else if (self_statusColor.lbc_port_nav350 == 0)
		ui->lbc_port_nav350->setStyleSheet("background-color: red; color: white;");

	// -- Sensor Up
	if (self_statusColor.lbc_limit_up == 1)
		ui->lbc_limit_up->setStyleSheet("background-color: blue; color: white;");
	else if (self_statusColor.lbc_limit_up == 0)
		ui->lbc_limit_up->setStyleSheet("background-color: white; color: black;");

	// -- Sensor Down
	if (self_statusColor.lbc_limit_down == 1)
		ui->lbc_limit_down->setStyleSheet("background-color: blue; color: white;");
	else if (self_statusColor.lbc_limit_down == 0)
		ui->lbc_limit_down->setStyleSheet("background-color: white; color: black;");

	// -- Sensor detect lift
	if (self_statusColor.lbc_detect_lifter == 1)
		ui->lbc_detect_lifter->setStyleSheet("background-color: blue; color: white;");
	else if (self_statusColor.lbc_detect_lifter == 0)
		ui->lbc_detect_lifter->setStyleSheet("background-color: white; color: black;");
}

void AppGui::set_labelValue(){
	ui->lbv_angleCompare->setText(QString::fromStdString(to_string(ui->dial_angleCompare->value())));
	// --
	ui->lbv_battery->setText(QString::fromStdString(self_valueLable.lbv_battery));

	ui->lbv_coordinates_x->setText(QString::fromStdString(self_valueLable.lbv_coordinates_x));
	ui->lbv_coordinates_y->setText(QString::fromStdString(self_valueLable.lbv_coordinates_y));
	ui->lbv_coordinates_r->setText(QString::fromStdString(self_valueLable.lbv_coordinates_r));

	ui->lbv_coordinates_x1->setText(QString::fromStdString(self_valueLable.lbv_coordinates_x));
	ui->lbv_coordinates_y1->setText(QString::fromStdString(self_valueLable.lbv_coordinates_y));
	ui->lbv_coordinates_r1->setText(QString::fromStdString(self_valueLable.lbv_coordinates_r));

	ui->lbv_numberReflector->setText(QString::fromStdString(self_valueLable.lbv_numbeReflector));

	ui->lbv_pingServer->setText(QString::fromStdString(self_valueLable.lbv_pingServer));

	ui->lbv_route_target->setText(QString::fromStdString(self_valueLable.lbv_route_target));
	ui->lbv_jobRuning->setText(QString::fromStdString(self_valueLable.lbv_jobRuning));
	ui->lbv_goalFollow_id->setText(QString::fromStdString(self_valueLable.lbv_goalFollow_id));
	ui->lbv_route_point0->setText(QString::fromStdString(self_valueLable.lbv_route_point0));
	ui->lbv_route_point1->setText(QString::fromStdString(self_valueLable.lbv_route_point1));
	ui->lbv_route_point2->setText(QString::fromStdString(self_valueLable.lbv_route_point2));
	ui->lbv_route_point3->setText(QString::fromStdString(self_valueLable.lbv_route_point3));
	ui->lbv_route_point4->setText(QString::fromStdString(self_valueLable.lbv_route_point4));
	ui->lbv_route_job1->setText(QString::fromStdString(self_valueLable.lbv_route_job1));
	ui->lbv_route_job2->setText(QString::fromStdString(self_valueLable.lbv_route_job2));
	ui->lbv_route_job1_mean->setText(QString::fromStdString(self_valueLable.lbv_route_job1_mean));
	ui->lbv_route_job2_mean->setText(QString::fromStdString(self_valueLable.lbv_route_job2_mean));

	ui->lbv_route_message->setText(QString::fromStdString(self_valueLable.lbv_route_message));

	// ui->lbv_coorAverage_x->setText(self_valueLable.lbv_coorAverage_x));
	// ui->lbv_coorAverage_y->setText(self_valueLable.lbv_coorAverage_y));
	// ui->lbv_coorAverage_r->setText(self_valueLable.lbv_coorAverage_r));
	// ui->lbv_coorAverage_times->setText(self_valueLable.lbv_coorAverage_times));

	ui->lbv_velLeft->setText(QString::fromStdString(self_valueLable.lbv_velLeft));
	ui->lbv_velRight->setText(QString::fromStdString(self_valueLable.lbv_velRight));

	ui->lbv_notification_driver1->setText(QString::fromStdString(self_valueLable.lbv_notification_driver1));
	ui->lbv_notification_driver2->setText(QString::fromStdString(self_valueLable.lbv_notification_driver2));

	// - 
	ui->lbv_reflectorLoc->setText(QString::fromStdString(self_valueLable.lbv_numbeReflector));
	ui->lbv_reflectorDetect->setText(QString::fromStdString(self_valueLable.lbv_reflectorDetect));
	ui->lbv_reflectorDetect_1->setText(QString::fromStdString(self_valueLable.lbv_reflectorDetect));
}

void AppGui::controlShow_followMode(){
	if (self_valueLable.modeRuning == modeRun_launch)
		modeRuning = modeRun_launch;
	else if (self_valueLable.modeRuning == modeRun_byhand)
		modeRuning = modeRun_byhand;
	else if (self_valueLable.modeRuning == modeRun_auto)
		modeRuning = modeRun_auto;
	else{
		modeRuning = modeRun_auto;
	}
	// --
	if (modeRuning == modeRun_launch){ // -- Khoi Dong
		ui->fr_launch->show();
		ui->fr_run->hide();
		show_launch();
	}
	else{
		ui->fr_run->show();
		ui->fr_launch->hide();

		if (modeRuning == modeRun_auto){ // -- Tu dong
			ui->fr_control->show();
			ui->fr_setting->hide();
			ui->fr_handMode_conveyor->hide();
			ui->fr_handMode_move->hide();
			ui->fr_listTask->show();
			ui->fr_tryTarget->hide();
			isShow_moveHand = 1;
		}

		else if (modeRuning == modeRun_byhand){
			// - Hiển thị cài đặt chung. 
			if (isShow_setting == 1){
				ui->fr_control->hide();
				ui->fr_setting->show();
				ui->fr_listTask->hide();
				ui->fr_nav350->hide();
			}

			// - Hiển thị kiểm tra gương.
			else if (isShow_reflectorCheck == 1){
				ui->fr_control->hide();
				ui->fr_setting->hide();
				ui->fr_listTask->hide();
				ui->fr_nav350->show();
			}

			// - Hiển thị thử nghiệm điểm.
			else if (isShow_tryTarget == 1){
				ui->fr_control->show();
				ui->fr_setting->hide();
				ui->fr_listTask->hide();
				ui->fr_nav350->hide();

				ui->fr_handMode_move->hide();
				ui->fr_handMode_conveyor->hide();
				ui->fr_tryTarget->show();
			}

			// - Hiển thị chức năng điều khiển tay.
			else{
				ui->fr_control->show();
				ui->fr_setting->hide();
				ui->fr_listTask->hide();
				ui->fr_nav350->hide();
				if (isShow_moveHand == 1){
					ui->fr_handMode_conveyor->hide();
					ui->fr_handMode_move->show();
					ui->fr_tryTarget->hide();
				}
				else{
					ui->fr_handMode_conveyor->show();
					ui->fr_handMode_move->hide();
					ui->fr_tryTarget->hide();
				}
			}
		}
	}
}

void AppGui::coorAverage_run(){
	if (bt_coorAverage_status == 1){
		countTime_coorAverage += 1.;
		total_x += robotPoseNow.position.x;
		total_y += robotPoseNow.position.y;
		double euler = quaternion_to_euler(robotPoseNow.orientation);
		total_angle += euler;
	}

	else{
		double timeSave_coorAverage = ros::Time::now().toSec();
		if (countTime_coorAverage > 4){
			double d_x = total_x/countTime_coorAverage;
			double d_y = total_y/countTime_coorAverage;
			double d_a = total_angle/countTime_coorAverage;
			double d_degree = d_a * 57.2957795;

			ui->lbv_coorAverage_times->setText(QString::fromStdString(to_string(countTime_coorAverage)));
			ui->lbv_coorAverage_x->setText(QString::fromStdString(to_string(round_3decimal(d_x))));
			ui->lbv_coorAverage_y->setText(QString::fromStdString(to_string(round_3decimal(d_y))));
			ui->lbv_coorAverage_r->setText(QString::fromStdString(to_string(round_3decimal(d_degree))));

			if (ui->ck_linkCoor->isChecked() == 1){
				self_valueLable.lbv_tryTarget_x = d_x;
				self_valueLable.lbv_tryTarget_y = d_y;
				self_valueLable.lbv_tryTarget_r = d_degree;
				ui->lbv_tryTarget_x->setText(QString::fromStdString(to_string(round_3decimal(self_valueLable.lbv_tryTarget_x))));
				ui->lbv_tryTarget_y->setText(QString::fromStdString(to_string(round_3decimal(self_valueLable.lbv_tryTarget_y))));
				ui->lbv_tryTarget_r->setText(QString::fromStdString(to_string(round_3decimal(self_valueLable.lbv_tryTarget_r))));
			}
		}
		countTime_coorAverage = 0;
		total_x = 0.0;
		total_y = 0.0;
		total_angle = 0.0;
	}
}

void AppGui::show_launch(){
	ui->lbv_launhing->setText(QString::fromStdString(self_valueLable.lbv_launhing));
	ui->lbv_numberLaunch->setText(QString::fromStdString(self_valueLable.lbv_numberLaunch));
	// --
	int value = self_valueLable.percentLaunch;
	if (value < 0)
		value = 0;

	if (value > 100)
		value = 100;

	ui->pb_launch->setValue(value);
	// --
	// -- Port: RTC Board
	if (self_statusColor.lbc_port_rtc == 1)
		ui->lbc_lh_rtc->setStyleSheet("background-color: blue; color: white");
	else if (self_statusColor.lbc_port_rtc == 0)
		ui->lbc_lh_rtc->setStyleSheet("background-color: red; color: white;");

	// -- Port: RS485
	if (self_statusColor.lbc_port_rs485 == 1)
		ui->lbc_lh_driver->setStyleSheet("background-color: blue; color: white");
	else if (self_statusColor.lbc_port_rs485 == 0)
		ui->lbc_lh_driver->setStyleSheet("background-color: red; color: white;");

	// -- Port: NAV350
	if (self_statusColor.lbc_port_nav350 == 1)
		ui->lbc_lh_lidar->setStyleSheet("background-color: blue; color: white");
	else if (self_statusColor.lbc_port_nav350 == 0)
		ui->lbc_lh_lidar->setStyleSheet("background-color: red; color: white;");
}

void AppGui::show_password(){
	string data = "";
	int length = password_data.length();

	for(int i = 0; i < length; i++)
		data.push_back('*');
	
	ui->lbv_pw_data->setText(QString::fromStdString(data));

	if (password_data == password_right)
		ui->bt_pw_agree->setEnabled(true);
	else{
		ui->bt_pw_agree->setEnabled(false);
	}
}

void AppGui::anlis_ref2(){
	arrReflector.clear();
	double max_x = -1000;
	double max_y = -1000;
	double p_x, p_y;
	double dis, ang0, ang, rate_show, rate_xy;
	int rp_x, rp_y, sh_x, sh_y;

	int length = nav350_reflectors.num_reflector;
	for(int i = 0; i < length; i++){
		dis = nav350_reflectors.reflectors[i].Polar_Dist/1000.;
		ang0 = nav350_reflectors.reflectors[i].Polar_Phi/1000./57.2957795;
		ang = limitAngle(ang0);

		tie(p_x, p_y) = convert_position(dis, ang);
	}

	if (max_x < fabs(p_x))
		max_x = fabs(p_x);

	if (max_y < fabs(p_y))
		max_y = fabs(p_y);

	rate_show = 0.0;
	rate_xy = max_x/max_y;
	if (rate_xy > 0.5)
		rate_show = 1.0; // (max_x*1000)/431.
	else
		rate_show = 1.0; // (max_y*1000)/811.
	// ----
	
	rate_show = 0.07;
	// print ("----------")
	for(int i = 0; i < length; i++){
		dis = nav350_reflectors.reflectors[i].Polar_Dist/1000.;
		ang0 = (nav350_reflectors.reflectors[i].Polar_Phi/1000. + stod(self_valueLable.angleCompare))/57.2957795;
		ang = limitAngle(ang0);

		tie(p_x, p_y) = convert_position(dis, ang);
		rp_x = int(p_x/rate_show);
		rp_y = int(p_y/rate_show);
		sh_x = rp_y + 400;
		sh_y = rp_x + 200;
		
		// print ( str(self.nav350_reflectors.reflectors[i].LocalID) + " | " + str(self.nav350_reflectors.reflectors[i].GlobalID) + " | " + str(round(sh_x, 3)) + " | " + str(round(sh_y, 3)) )
		// --
		Reflector reflector;
		reflector.x = sh_x;
		reflector.y = sh_y;
		reflector.localID  = to_string(nav350_reflectors.reflectors[i].LocalID);
		reflector.globalID = to_string(nav350_reflectors.reflectors[i].GlobalID);

		arrReflector.push_back(reflector);
	}
}

void AppGui::callback_nav350Reflectors(const message_pkg::Reflector_array data){
	nav350_reflectors = data;
	anlis_ref2();
}

void AppGui::callback_brakeControl(const std_msgs::Bool data){
	status_brake = data;
}

void AppGui::callback_driver1(const message_pkg::Driver_respond data){
	driver1_respond = data;
}

void AppGui::callback_driver2(const message_pkg::Driver_respond data){
	driver2_respond = data;
}

void AppGui::callback_HC(const sti_msgs::HC_info data){
	HC_info = data;
}

void AppGui::callback_Main(const sti_msgs::POWER_info data){
	main_info = data;
}

void AppGui::callback_OC_board(const sti_msgs::Lift_status data){
	OC_status = data;
}

void AppGui::callback_statusPort(const message_pkg::Status_port data){
	status_port = data;
}

void AppGui::goalControl_callback(const sti_msgs::Status_goal_control data){
	status_goalControl = data;
}

void AppGui::callBack_cancelMission(const std_msgs::Int16 data){
	status_cancelMission = data;
}

void AppGui::callback_nav350(const message_pkg::Nav350_data data){
	nav350_data = data;
}

void AppGui::callback_safetyNAV(const std_msgs::Int8 data){
	safety_NAV = data;
}

void AppGui::callback_robotPose(const geometry_msgs::PoseStamped data){
	robotPose_nav = data;
}

void AppGui::callback_server_cmdRequest(const message_pkg::Server_cmdRequest data){
	server_cmdRequest = data;
}

void AppGui::NN_cmdRequest_callback(const sti_msgs::NN_cmdRequest data){
	NN_cmdRequest = data;
}

void AppGui::callback_NN_infoRequest(const sti_msgs::NN_infoRequest data){
	NN_infoRequest = data;
}

void AppGui::infoAGV_callback(const sti_msgs::NN_infoRespond data){
	NN_infoRespond = data;
}

void AppGui::callback_statusLaunch(const message_pkg::Status_launch data){
	status_launch = data;
}

bool AppGui::getBit_fromInt16(int16_t value_in, int pos){
	bool bit_out = 0;
	int16_t value_now = value_in;
	for(int i = 0; i < 16; i++){
		bit_out = value_now%2;
		value_now = int(value_now/2);
		if (i == pos)
			return bit_out;

		if (value_now < 1)
			return 0;	
	}	
	return 0;	
}

tuple<double, double> AppGui::convert_position(double distance, double angle){
	double x, y;
	x = distance*cos(angle);
	y = distance*sin(angle);
	// y = distance*cos(angle)
	// x = distance*sin(angle)
	return make_tuple(x, y);
}

// string AppGui::ping_traffic(string address){
// 	try{
// 		ping = subprocess.check_output("ping -c 1 -w 1 {}".format(address), shell=True)
// 		// print(ping)
// 		vitri = str(ping).find("time")
// 		time_ping = str(ping)[(vitri+5):(vitri+9)]
// 		// print (time_ping)
// 		return str(float(time_ping))
// 	}
// 	catch(...){
// 		return "-1";
// 	}
// }

string AppGui::convertToString(char* a, int size)
{
    int i;
    string s = "";
    for (i = 0; i < size; i++) {
        s = s + a[i];
    }
    return s;
}

string AppGui::get_ipAuto(string name_card){
	try{
		int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

		struct ifreq ifr{};
		const char* str = name_card.c_str();
		strcpy(ifr.ifr_name, str);
		ioctl(fd, SIOCGIFADDR, &ifr);
		// close(fd);

		char ip[INET_ADDRSTRLEN];
		strcpy(ip, inet_ntoa(((sockaddr_in *) &ifr.ifr_addr)->sin_addr));
   		std::cout << ip << std::endl;
		int ip_size = sizeof(ip) / sizeof(char);
		string s_ip = convertToString(ip, ip_size);		
		return s_ip;
	}
	catch(...){
		return "-1";
	}
}

void AppGui::kill_app(){
	out();
	is_exist = 0;	
}

string AppGui::get_MAC(string name_card){
	try{
		int fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);

		struct ifreq ifr{};
		const char* str = name_card.c_str();
		strcpy(ifr.ifr_name, str);
		ioctl(fd, SIOCGIFHWADDR, &ifr);
		// close(fd);

		char mac[18];
		strcpy(mac, ether_ntoa((ether_addr *) ifr.ifr_hwaddr.sa_data));

		std::cout << mac << std::endl;
		int mac_size = sizeof(mac) / sizeof(char);
		string s_mac = convertToString(mac, mac_size);
		return s_mac;
	}
	catch(...){
		return "-1";
	}
}

// int AppGui::get_qualityWifi(string name_card){
// 	try{
// 		quality_data = '0'
// 		output = os.popen("iwconfig {}".format(name_card)).read()
// 		pos_quality = str(output).find('Link Quality=')
// 		// -
// 		if pos_quality >= 0:
// 			quality_data = str(output)[pos_quality+13:pos_quality+15]
// 		// print ("quality_data: ", int(quality_data) )
// 		// -
// 		return int(quality_data)
// 	}
// 	catch(...){
// 		return 0;
// 	}
// }

string AppGui::get_hostname(){
	try{
   		char hostname[HOST_NAME_MAX + 1];
		gethostname(hostname, HOST_NAME_MAX + 1);
		std::cout << hostname << std::endl;
		int hostname_size = sizeof(hostname) / sizeof(char);
		string s_hostname = convertToString(hostname, hostname_size);
		return s_hostname;
	}
	catch(...){
		return "-1";
	}
}

geometry_msgs::Quaternion AppGui::euler_to_quaternion(double euler){
	tf::Quaternion odom_quat;
	geometry_msgs::Quaternion quat;
	odom_quat.setRPY(0,0, euler);
	odom_quat = odom_quat.normalize();
	// odom_quat = quaternion_from_euler(0, 0, euler)
	quat.x = odom_quat.x();
	quat.y = odom_quat.y();
	quat.z = odom_quat.z();
	quat.w = odom_quat.w();
	return quat;
}

double AppGui::limitAngle(double angle_in){
	geometry_msgs::Quaternion qua_in;
	qua_in = euler_to_quaternion(angle_in);
	double angle_out = quaternion_to_euler(qua_in);
	return angle_out;
}

string AppGui::convert_errorAll(int val){
	string str_out;
	if(val == 0) str_out = "AGV Hoạt Động Bình Thường";
	else if(val == 311) str_out = "Mất kết nối với Mạch STI-RTC";
	else if(val == 361) str_out = "Mất kết nối với Mạch STI-CPD";	

	else if(val == 351) str_out = "Mất kết nối với Mạch STI-HC";	
	else if(val == 352) str_out = "Không Giao Tiếp CAN Với Mạch STI-HC";

	else if(val == 341) str_out = "Mất Kết Nối Với Mạch STI-OC";		
	else if(val == 342) str_out = "Mất Cổng USB của USB của Mạch STI-OC";	
	else if(val == 343) str_out = "Không Giao Tiếp CAN Với Mạch STI-OC";	
	else if(val == 344) str_out = "Không Giao Tiếp Với Mạch STI-OC1";	
	else if(val == 345) str_out = "Không Giao Tiếp Với Mạch STI-OC2";		
	else if(val == 346) str_out = "Không Giao Tiếp Với Mạch STI-OC3";	

	else if(val == 323) str_out = "Mạng CAN Không Gửi Được";	
	else if(val == 321) str_out = "Mất Kết Nối Với Mạch STI-Main";	
	else if(val == 322) str_out = "Mất Cổng USB của USB của Mạch STI-Main";		
	else if(val == 251) str_out = "Mất Kết Nối Với Driver1";	
	else if(val == 252) str_out = "Lỗi Động Cơ Số 1";	
	else if(val == 261) str_out = "Mất Kết Nối Với Driver2";	
	else if(val == 262) str_out = "Lỗi Động Cơ Số 2";		
	else if(val == 231) str_out = "Mất Kết Nối Với Cảm Biến Góc";	
	else if(val == 232) str_out = "Mất Cổng USB của Cảm Biến IMU";	
	else if(val == 221) str_out = "Mất Kết Nối Với Cảm Biến NAV350";	
	else if(val == 181) str_out = "LoadCell-Ket Noi";		
	else if(val == 182) str_out = "LoadCell-Dau noi";	
	else if(val == 183) str_out = "LoadCell-USB";	
	else if(val == 184) str_out = "Quá Tải 700kg";	
	else if(val == 222) str_out = "Mất Tọa Độ NAV350";		
	else if(val == 141) str_out = "Lỗi Không Chạm Được Cảm Biến Bàn Nâng";	
	else if(val == 121) str_out = "Trạng Thái Dừng Khẩn - EMG";	
	else if(val == 122) str_out = "AGV Bị Chạm Blsock";	
	else if(val == 272) str_out = "Không Phát Hiện Được Đủ Gương";		
	else if(val == 281) str_out = "Mất TF Parking";	
	else if(val == 282) str_out = "Mất Gói GoalControl";	
	else if(val == 441) str_out = "AGV Đã Di Chuyển Hết Điểm";	
	else if(val == 442) str_out = "AGV Đang Dừng Để Nhường Đường Cho AGV Khác";		
	else if(val == 477) str_out = "Không Có Kệ Tại Vị Trí";	
	else if(val == 411) str_out = "Vướng Vật Cản - Di Chuyển Giữa Các Điểm";	
	else if(val == 412) str_out = "Vướng Vật Cản - Di Chuyển Vào Vị Trí Kệ";	
	else if(val == 431) str_out = "AGV Không Giao Tiếp Với Phần Mềm Traffic";		
	else if(val == 451) str_out = "Điện Áp Của AGV Đang Rất Thấp";	
	else if(val == 452) str_out = "AGV Không Sạc Được Pin";	
	else if(val == 453) str_out = "Không Phát Hiện Được Đủ Gương";
	else{
		str_out = "UNK";
	}

	return str_out;
}

string AppGui::show_job(int val){

	string str_out = "";
	if(val == 0) str_out = "...";
	else if(val == 1) str_out = "Kiểm Tra Lại Nhiệm Vụ";
	else if(val == 2) str_out = "Thực Hiện Nhiệm Vụ Trước";
	else if(val == 3) str_out = "Kiểm Tra Trạng Thái Kệ";	

	else if(val == 4) str_out = "Di Chuyển Ra Khởi Vị Trí";	
	else if(val == 5) str_out = "Di Chuyển Giữa Các Điểm";

	else if(val == 6) str_out = "Di Chuyển Vào Vị Trí Thao Tác";		
	else if(val == 7) str_out = "Thực Hiện Nhiệm Vụ Sau";	
	else if(val == 8) str_out = "Đợi Lệnh Mới";	
	else if(val == 9) str_out = "Đợi Hoàn Thành Lệnh Cũ";	
	else if(val == 20) str_out = "Chế Độ Bằng Tay";		
	else if(val == 30) str_out = "Chế Độ Tự Động";	
	else if(val == 50) str_out = "Kiểm Tra Vị Trí Trả Hàng";
	else{
		str_out = "Không\nXác Định";
	}

	return str_out;
}

string AppGui::show_misson(int val){

	string str_out = "";
	if(val == 0) str_out = "...";
	else if(val == 1) str_out = "Nâng Kệ";
	else if(val == 65) str_out = "Nâng Kệ";
	else if(val == 2) str_out = "Hạ Kệ";	
	else if(val == 66) str_out = "Hạ Kệ";	
	else if(val == 6) str_out = "Sạc Pin";
	else if(val == 10) str_out = "Hạ Kệ\nSạc Pin";		
	else{
		str_out = "Không\nXác Định";
	}

	return str_out;
}

void AppGui::controlColor(){
	self_statusColor.lbc_safety_up = safety_NAV.data;
	// -- HC_info
	self_statusColor.lbc_safety_ahead = HC_info.zone_sick_ahead;
	self_statusColor.lbc_safety_behind = HC_info.zone_sick_behind;
	// --
	self_statusColor.lbc_button_clearError = main_info.stsButton_reset;
	self_statusColor.lbc_button_power = main_info.stsButton_power;
	self_statusColor.lbc_emg = main_info.EMC_status;
	self_statusColor.lbc_blsock = HC_info.vacham;

	// -- Port
	self_statusColor.lbc_port_rtc    = status_port.rtc;
	self_statusColor.lbc_port_rs485  = status_port.driverall;
	self_statusColor.lbc_port_nav350 = status_port.nav350;

	// --
	self_statusColor.lbc_limit_up = OC_status.sensorUp.data;
	self_statusColor.lbc_limit_down = OC_status.sensorDown.data;
	self_statusColor.lbc_detect_lifter = OC_status.sensorLift.data;
}

void AppGui::controlAll(){
	// -- Mode show
	if (NN_infoRespond.mode == 0)   // - launch
		self_valueLable.modeRuning = modeRun_launch;

	else if (NN_infoRespond.mode == 1) // -- md_by_hand
		self_valueLable.modeRuning = modeRun_byhand;

	else if (NN_infoRespond.mode == 2) // -- md_auto
		self_valueLable.modeRuning = modeRun_auto;

	// -- Battery
	if (main_info.charge_current > 0.1)
		self_statusColor.lbc_battery = 4;
	else{
		if (main_info.voltages < 23.5)
			self_statusColor.lbc_battery = 3;
		else if (main_info.voltages >= 23.5 && main_info.voltages < 24.5)
			self_statusColor.lbc_battery = 2;
		else
			self_statusColor.lbc_battery = 1;
	}

	float bat = round_1decimal(main_info.voltages);
	//cout << "bat= " << bat << endl;
	if (bat > 25.5)
		bat = 25.5;
	self_valueLable.lbv_battery = to_string(bat);
	self_valueLable.lbv_battery.resize(4);
	//cout << "bat is " << self_valueLable.lbv_battery << endl; 

	// -- status AGV
	self_statusColor.cb_status = NN_infoRespond.status;
	int lg_err = NN_infoRespond.listError.size();
	self_valueLable.listError.clear();
	if (lg_err == 0)
		self_valueLable.listError.push_back(convert_errorAll(0));
	else{
		int length = self_valueLable.list_logError.size();
		if (length > 15)
			self_valueLable.list_logError.clear();
		// -
		for(int i = 0; i < lg_err; i++){
			self_valueLable.listError.push_back(convert_errorAll(NN_infoRespond.listError[i]) );  
			// -
			if (NN_infoRespond.listError[i] < 400)
				self_valueLable.list_logError.push_back(convert_errorAll(NN_infoRespond.listError[i]) );
		}
	}
	// -
	self_valueLable.lbv_name_agv = NN_infoRequest.name_agv;
	self_valueLable.lbv_numbeReflector = to_string(nav350_data.number_reflectors);
	self_valueLable.lbv_reflectorDetect = to_string(nav350_reflectors.num_reflector);

	// -- Ping
	// deltaTime_ping = (time.time() - pre_timePing)%60
	// if (deltaTime_ping > 2.0):
	// 	self.pre_timePing = time.time()
	// 	self.valueLable.lbv_pingServer = self.ping_traffic(self.address_traffic)
	// 	// -
	// 	self.valueLable.lbv_qualityWifi = self.get_qualityWifi(self.name_card)

	// -- 
	self_valueLable.lbv_coordinates_x = to_string(round_3decimal(robotPose_nav.pose.position.x));
	self_valueLable.lbv_coordinates_x.resize(7);
	self_valueLable.lbv_coordinates_y = to_string(round_3decimal(robotPose_nav.pose.position.y));
	self_valueLable.lbv_coordinates_y.resize(7);
	double angle = quaternion_to_euler(robotPose_nav.pose.orientation);
	double angle_robot;
	
	if (angle < 0)
		angle_robot = 2*M_PI + angle;
	else{
		angle_robot = angle;
	}
	self_valueLable.lbv_coordinates_r = to_string(round_3decimal(angle_robot*57.2957795));
	self_valueLable.lbv_coordinates_r.resize(7);
	// --
	//self_valueLable.lbv_route_target = to_string(NN_cmdRequest.target_id) + "\n" + to_string(NN_cmdRequest.target_x) + "\n" + to_string(NN_cmdRequest.target_y) + \
										to_string(round_3decimal(NN_cmdRequest.target_z*57.2957795)) + "\n" + to_string(NN_cmdRequest.offset);
	string a5 = to_string(NN_cmdRequest.target_id);
	string b5 = to_string(NN_cmdRequest.target_x); b5.resize(7);
	string c5 = to_string(NN_cmdRequest.target_y); c5.resize(7);	
	string d5 = to_string(NN_cmdRequest.target_z); d5.resize(7);	
	string e5 = to_string(NN_cmdRequest.offset); e5.resize(5);
	self_valueLable.lbv_route_target = a5 + "\n" + b5 + "\n" + c5 + "\n" + d5 + "\n" + e5;
	
	// // -
	if (NN_cmdRequest.list_id.size() >= 5){
		string a0 = to_string(NN_cmdRequest.list_id[0]);
		string b0 = to_string(NN_cmdRequest.list_x[0]); b0.resize(7);
		string c0 = to_string(NN_cmdRequest.list_y[0]); c0.resize(7);		
		string d0 = to_string(NN_cmdRequest.list_speed[0]); d0.resize(5);
		self_valueLable.lbv_route_point0 = a0 + "\n" + b0 + "\n" + c0 + "\n" + d0;

		string a1 = to_string(NN_cmdRequest.list_id[1]);
		string b1 = to_string(NN_cmdRequest.list_x[1]); b1.resize(7);
		string c1 = to_string(NN_cmdRequest.list_y[1]); c1.resize(7);		
		string d1 = to_string(NN_cmdRequest.list_speed[1]); d1.resize(5);
		self_valueLable.lbv_route_point1 = a1 + "\n" + b1 + "\n" + c1 + "\n" + d1;

		string a2 = to_string(NN_cmdRequest.list_id[2]);
		string b2 = to_string(NN_cmdRequest.list_x[2]); b2.resize(7);
		string c2 = to_string(NN_cmdRequest.list_y[2]); c2.resize(7);		
		string d2 = to_string(NN_cmdRequest.list_speed[2]); d2.resize(5);
		self_valueLable.lbv_route_point2 = a2 + "\n" + b2 + "\n" + c2 + "\n" + d2;

		string a3 = to_string(NN_cmdRequest.list_id[3]);
		string b3 = to_string(NN_cmdRequest.list_x[3]); b3.resize(7);
		string c3 = to_string(NN_cmdRequest.list_y[3]); c3.resize(7);		
		string d3 = to_string(NN_cmdRequest.list_speed[3]); d3.resize(5);
		self_valueLable.lbv_route_point3 = a3 + "\n" + b3 + "\n" + c3 + "\n" + d3;

		string a4 = to_string(NN_cmdRequest.list_id[4]);
		string b4 = to_string(NN_cmdRequest.list_x[4]); b4.resize(7);
		string c4 = to_string(NN_cmdRequest.list_y[4]); c4.resize(7);		
		string d4 = to_string(NN_cmdRequest.list_speed[4]); d4.resize(5);
		self_valueLable.lbv_route_point4 = a4 + "\n" + b4 + "\n" + c4 + "\n" + d4;
		
		//self_valueLable.lbv_route_point0 = to_string(NN_cmdRequest.list_id[0]) + "\n" + to_string(NN_cmdRequest.list_x[0]) + "\n" + \
											to_string(NN_cmdRequest.list_y[0]) + "\n" + to_string(NN_cmdRequest.list_speed[0]);

		//self_valueLable.lbv_route_point1 = to_string(NN_cmdRequest.list_id[1]) + "\n" + to_string(NN_cmdRequest.list_x[1]) + "\n" + \
											to_string(NN_cmdRequest.list_y[1]) + "\n" + to_string(NN_cmdRequest.list_speed[1]);

		//self_valueLable.lbv_route_point2 = to_string(NN_cmdRequest.list_id[2]) + "\n" + to_string(NN_cmdRequest.list_x[2]) + "\n" + \
											to_string(NN_cmdRequest.list_y[2]) + "\n" + to_string(NN_cmdRequest.list_speed[2]);		

		//self_valueLable.lbv_route_point3 = to_string(NN_cmdRequest.list_id[3]) + "\n" + to_string(NN_cmdRequest.list_x[3]) + "\n" + \
											to_string(NN_cmdRequest.list_y[3]) + "\n" + to_string(NN_cmdRequest.list_speed[3]);	

		//self_valueLable.lbv_route_point4 = to_string(NN_cmdRequest.list_id[4]) + "\n" + to_string(NN_cmdRequest.list_x[4]) + "\n" + \
											to_string(NN_cmdRequest.list_y[4]) + "\n" + to_string(NN_cmdRequest.list_speed[4]);																		
	}
	self_valueLable.lbv_route_job1 = to_string(NN_cmdRequest.before_mission);
	self_valueLable.lbv_route_job2 = to_string(NN_cmdRequest.after_mission);

	self_valueLable.lbv_route_job1_mean = show_misson(NN_cmdRequest.before_mission);        
	self_valueLable.lbv_route_job2_mean = show_misson(NN_cmdRequest.after_mission);

	self_valueLable.lbv_route_message = NN_cmdRequest.command;
	self_valueLable.lbv_jobRuning = show_job(NN_infoRespond.process);                 
	// -- 
	self_valueLable.lbv_goalFollow_id = to_string(status_goalControl.ID_follow);

	// -- Launch
	self_valueLable.percentLaunch = status_launch.persent;
	self_valueLable.lbv_launhing = status_launch.notification;
	self_valueLable.lbv_numberLaunch = status_launch.position;

	self_valueLable.lbv_notification_driver1 = driver1_respond.message_error;
	self_valueLable.lbv_notification_driver2 = driver2_respond.message_error;
}

void AppGui::readButton(){

	// cout << "Đã run việc đọc nút nhấn" << endl;
	// -- 
	app_button.bt_cancelMission = self_statusButton.bt_cancelMission;
	app_button.bt_passAuto 	 = self_statusButton.bt_passAuto;
	app_button.bt_passHand 	 = self_statusButton.bt_passHand;
	// self.app_button.bt_setting 		 = self_statusButton.bt_setting
	app_button.bt_clearError 	 = self_statusButton.bt_clearError;
	// --
	app_button.bt_forwards 	  = self_statusButton.bt_forwards;
	app_button.bt_backwards	  = self_statusButton.bt_backwards;
	app_button.bt_rotation_left  = self_statusButton.bt_rotation_left;
	app_button.bt_rotation_right = self_statusButton.bt_rotation_right;
	app_button.bt_stop 		  = self_statusButton.bt_stop;
	// --
	app_button.bt_chg_on	= self_statusButton.bt_chg_on;
	app_button.bt_chg_off	= self_statusButton.bt_chg_off;

	app_button.bt_spk_on  = self_statusButton.bt_spk_on;
	app_button.bt_spk_off  = self_statusButton.bt_spk_off;

	app_button.bt_disableBrake	= self_statusButton.bt_disableBrake;

	// -- 
	app_button.bt_lift_up	 = self_statusButton.bt_lift_up;
	app_button.bt_lift_down = self_statusButton.bt_lift_down;
	// -
	app_button.vs_speed = self_statusButton.vs_speed;
	app_button.bt_resetFrameWork = self_statusButton.bt_resetFrameWork;

	// -
	app_button.bt_tryTarget_start = self_statusButton.bt_tryTarget_start;
	app_button.bt_tryTarget_stop = self_statusButton.bt_tryTarget_stop;
	app_button.bt_tryTarget_reset = self_statusButton.bt_tryTarget_reset;
	app_button.ck_tryTarget_safety = self_statusButton.ck_tryTarget_safety;
	// -
	app_button.tryTarget_x = self_valueLable.lbv_tryTarget_x;
	app_button.tryTarget_y = self_valueLable.lbv_tryTarget_y;
	app_button.tryTarget_r = self_valueLable.lbv_tryTarget_r;
	app_button.tryTarget_d = self_valueLable.lbv_tryTarget_d;
	pub_button.publish(app_button);
}

// void AppGui::run(){
//     AppGui   tmpC1;  // temporary instance here !
//     AppGui*  tmpThis = &tmpC1;

// 	tmpThis->self_valueLable.lbv_ip = tmpThis->get_ipAuto(tmpThis->name_card);
// 	tmpThis->self_valueLable.lbv_mac = tmpThis->get_MAC(tmpThis->name_card);
// 	tmpThis->self_valueLable.lbv_namePc = tmpThis->get_hostname();
// 	ros::Rate loop_rate(40);

// 	while (ros::ok() && tmpThis->shutdown_flag == 0 && tmpThis->is_exist == 1){
// 		tmpThis->controlAll();
// 		tmpThis->controlColor();
// 		// --
// 		tmpThis->readButton();
// 		// tmpThis->pub_button.publish(tmpThis->app_button);

// 		if (tmpThis->status_cancelMission.data == 1){
// 			tmpThis->self_statusButton.bt_cancelMission = 0;
// 			tmpThis->cancelMission_control.data = 0;
// 		}

// 		if (tmpThis->self_statusButton.bt_cancelMission == 1){
// 			tmpThis->cancelMission_control.data = 1;
// 		}

// 		tmpThis->pub_cancelMission.publish(tmpThis->cancelMission_control);

// 		// ----------------------
// 		// self_valueLable = self.valueLable
// 		// self_statusColor = self.statusColor
// 		// -- 
// 		tmpThis->robotPoseNow = tmpThis->robotPose_nav.pose;
// 		// -
// 		tmpThis->self_valueLable.arrReflector = tmpThis->arrReflector; /// <<<<<<<<<<<<<<<<<<<<<<
// 		ros::spinOnce();     // allow receiving callbacks function
// 		loop_rate.sleep();
// 	}
// }

void AppGui::run(){
	self_valueLable.lbv_ip = get_ipAuto(name_card);
	self_valueLable.lbv_mac = get_MAC(name_card);
	self_valueLable.lbv_namePc = get_hostname();
	ros::Rate loop_rate(40);

	while (ros::ok() && shutdown_flag == 0 && is_exist == 1){
		controlAll();
		controlColor();
		// --
		readButton();
		// pub_button.publish(app_button);

		if (status_cancelMission.data == 1){
			self_statusButton.bt_cancelMission = 0;
			cancelMission_control.data = 0;
		}

		if (self_statusButton.bt_cancelMission == 1){
			cancelMission_control.data = 1;
		}

		pub_cancelMission.publish(cancelMission_control);

		// ----------------------
		// self_valueLable = self.valueLable
		// self_statusColor = self.statusColor
		// -- 
		robotPoseNow = robotPose_nav.pose;
		// -
		self_valueLable.arrReflector = arrReflector; /// <<<<<<<<<<<<<<<<<<<<<<
		ros::spinOnce();     // allow receiving callbacks function
		loop_rate.sleep();
	}
	is_exist = 0;
}

float AppGui::round_1decimal(float var)
{   
	float value = round(var * 10) / 10;
	return value;
}

double AppGui::round_3decimal(double var)
{   
	double value = round(var * 1000) / 1000;
	return value;
}

void AppGui::run_screen(int argc, char *argv[]){
    AppGui   tmpC1;  // temporary instance here !
    AppGui*  tmpThis = &tmpC1;

	QApplication a(argc, argv);	
	tmpThis->setWindowTitle(QString::fromStdString(ros::this_node::getName()));

	// load the icon from our qrc file and set it as the application icon
	QIcon icon(":/icons/my_gui_icon.png");
	tmpThis->setWindowIcon(icon);

	tmpThis->show();
	try{
		a.exec();
	}
	catch(...){
		cout << "Cannot run app" << endl;
	}
//   return a.exec();
}
