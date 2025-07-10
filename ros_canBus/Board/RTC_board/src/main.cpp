#include <Arduino.h>
#include "CAN_manager.h"
#include "Main_controller.h"

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <message_pkg/msg/cansend.h>
#include <message_pkg/msg/canreceived.h>

#define FREQUENCY_PUB_POWER_INFO 4
#define FREQUENCY_PUB_OC_INFO    4
#define FREQUENCY_PUB_HC_INFO    24

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\


/* ROS 2 variable */
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;
rcl_allocator_t allocator;
rcl_publisher_t CAN_received_pub;
rcl_subscription_t CAN_send_sub;

/* message */
message_pkg__msg__Cansend CAN_send;
message_pkg__msg__Canreceived CAN_received;


unsigned long saveTime_pub = 0;
unsigned long saveTime_sub = 0;
unsigned long debugTime = 0;

TaskHandle_t Task_Run_CAN; // Task_Run_CAN
CAN_manager* MainCAN = new CAN_manager(CAN_BAUD_SPEED, CAN_TX, CAN_RX, CAN_FRAME, CAN_ID, CAN_SEND_SIZE);
Main_controller* MainCtrl = new Main_controller(MainCAN);

void create_task();
void CanHandle_Run( void* pvParameters ){
  for(;;){
    MainCtrl->Run_Send_CAN();
    // MainCtrl->Test_Send_CAN();
    vTaskDelay(4);
  }
}

/* Trạng thái ROS Agent */
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

/* function callback */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    if (MainCtrl->is_receivedCAN == true){
      MainCtrl->is_receivedCAN = false;
      MainCtrl->sts_LED_ROS_SEND = true;
      rcl_publish(&CAN_received_pub, &CAN_received, NULL);
      saveTime_pub = millis();
    }
  }
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3


void CAN_send_Callback(const void * data)
{
  CAN_send = *((message_pkg__msg__Cansend *)data);
  // - Control main
  MainCtrl->send_CAN->id = CAN_send.id;
  MainCtrl->send_CAN->byte0 = CAN_send.byte0;
  MainCtrl->send_CAN->byte1 = CAN_send.byte1;
  MainCtrl->send_CAN->byte2 = CAN_send.byte2;
  MainCtrl->send_CAN->byte3 = CAN_send.byte3;
  MainCtrl->send_CAN->byte4 = CAN_send.byte4;
  MainCtrl->send_CAN->byte5 = CAN_send.byte5;
  MainCtrl->send_CAN->byte6 = CAN_send.byte6;
  MainCtrl->send_CAN->byte7 = CAN_send.byte7;
  MainCtrl->is_sendCAN = true;
  MainCtrl->sts_LED_ROS_RECEIVED = true;
} 


bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "Can_ros", "", &support));

  // create publisher
  // Cấu hình QoS với RELIABLE (mặc định)
  // -----
  // rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  // qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;  // Mặc định RELIABLE

  // rmw_qos_profile_t qos_profile = rmw_qos_profile_default;  // Dùng profile mặc định
  // qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  // qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  // qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  // qos_profile.depth = 10;

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data; // Dùng profile sensor

  // Thay đổi cấu hình QoS
  RCCHECK(rclc_publisher_init(
    &CAN_received_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(message_pkg, msg, Canreceived),
    "can_received",
    &qos_profile));
  // ---

  // Bật lên để chọn QoS là best_effort
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &CAN_received_pub,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(message_pkg, msg, Canreceived),
  //   "can_received"));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &CAN_send_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(message_pkg, msg, Cansend),
    "can_send"));

  // create timer,
  /*
   * Init timer_callback
   * TODO : change timer_timeout
   * 50ms : 20Hz
   * 20ms : 50Hz
   * 10ms : 100Hz
   */
  const unsigned int timer_timeout = 20;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor_pub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  executor_sub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &CAN_send_sub, &CAN_send, &CAN_send_Callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&CAN_received_pub, &node);
  rcl_subscription_fini(&CAN_send_sub, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor_pub);
  rclc_executor_fini(&executor_sub);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  MainCtrl->init_main();
	MainCtrl->led_start();

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  // set_microros_transports();
  state = WAITING_AGENT;

  MainCAN->CAN_prepare();
  delay(1000);

  create_task();

}

void loop() {

  MainCtrl->CANReceiveHandle();
  /* -- -- -- -- CAN RECEIVED -- -- -- -- */
  CAN_received.idsend = MainCtrl->received_CAN->idSend;
  CAN_received.byte0  = MainCtrl->received_CAN->byte0;
  CAN_received.byte1  = MainCtrl->received_CAN->byte1;
  CAN_received.byte2  = MainCtrl->received_CAN->byte2;
  CAN_received.byte3  = MainCtrl->received_CAN->byte3;
  CAN_received.byte4  = MainCtrl->received_CAN->byte4;
  CAN_received.byte5  = MainCtrl->received_CAN->byte5;
  CAN_received.byte6  = MainCtrl->received_CAN->byte6;
  CAN_received.byte7  = MainCtrl->received_CAN->byte7;

  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(10));
        rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  MainCtrl->led_loop();
}

void create_task() {   
    xTaskCreatePinnedToCore(
        CanHandle_Run, 	/* Task function. */
        "Task_Run_CAN", /* name of task. */
        10000,          /* Stack size of task */
        NULL,         	/* parameter of the task */
        1,            	/* priority of the task */
        &Task_Run_CAN,  /* Task handle to keep track of created task */
        1);           	/* pin task to core 0 */              
}


// #include <Arduino.h>
// #include <micro_ros_platformio.h>

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #include <std_msgs/msg/int32.h>

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

// rcl_publisher_t publisher;
// std_msgs__msg__Int32 msg;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_init_options_t init_options;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;

// const int domain_id = 0;

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// // Error handle loop
// void error_loop() {
//   while(1) {
//     delay(100);
//   }
// }

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//     msg.data++;
//   }
// }

// void setup() {
//   // Configure serial transport
//   Serial.begin(115200);
//   set_microros_serial_transports(Serial);
//   delay(2000);

//   allocator = rcl_get_default_allocator();
//   init_options = rcl_get_zero_initialized_init_options();

//   //create init_options
//   // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//   RCCHECK(rcl_init_options_init(&init_options, allocator));
//   RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id);)
//   RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

//   // create node
//   RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

//   // create publisher
//   RCCHECK(rclc_publisher_init_default(
//     &publisher,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//     "micro_ros_platformio_node_publisher"));

//   // create timer,
//   const unsigned int timer_timeout = 100;
//   RCCHECK(rclc_timer_init_default(
//     &timer,
//     &support,
//     RCL_MS_TO_NS(timer_timeout),
//     timer_callback));

//   // create executor
//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//   RCCHECK(rclc_executor_add_timer(&executor, &timer));

//   msg.data = 0;
// }

// void loop() {
//   delay(100);
//   RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
// }
