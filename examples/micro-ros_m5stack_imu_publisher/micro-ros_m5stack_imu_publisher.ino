#include <micro_ros_arduino.h>

#include <M5Unified.h>

#include <math.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>

#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>

#if !defined(ESP32)
#error This example is only available for ESP32-based M5Stack devices.
#endif

#define AGENT_PORT 8888

#define IMU_FRAME_ID "m5stack_imu"
#define IMU_TOPIC "/m5stack/imu"
#define PUBLISH_PERIOD_MS 50

static const float GRAVITY = 9.80665f;
static const float GYRO_DEG_TO_RAD = 0.017453292519943295f;

char wifi_ssid[] = "WIFI SSID";
char wifi_pass[] = "WIFI PASS";
char agent_ip[] = "192.168.1.57";

rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

uint32_t last_publish_ms = 0;
uint32_t publish_count = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void draw_status(const char * status)
{
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(8, 8);
  M5.Display.println("micro-ROS M5 IMU");
  M5.Display.setTextSize(1);
  M5.Display.setCursor(8, 36);
  M5.Display.println(status);
}

void error_loop()
{
  while (1) {
    M5.update();
    delay(100);
  }
}

void init_imu_message()
{
  sensor_msgs__msg__Imu__init(&imu_msg);
  imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, IMU_FRAME_ID);

  imu_msg.orientation_covariance[0] = -1.0;
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[8] = 0.02;
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[8] = 0.04;
}

void fill_imu_message(const m5::imu_data_t & data)
{
  uint32_t now_ms = millis();
  imu_msg.header.stamp.sec = now_ms / 1000;
  imu_msg.header.stamp.nanosec = (now_ms % 1000) * 1000000;

  imu_msg.angular_velocity.x = data.gyro.x * GYRO_DEG_TO_RAD;
  imu_msg.angular_velocity.y = data.gyro.y * GYRO_DEG_TO_RAD;
  imu_msg.angular_velocity.z = data.gyro.z * GYRO_DEG_TO_RAD;

  imu_msg.linear_acceleration.x = data.accel.x * GRAVITY;
  imu_msg.linear_acceleration.y = data.accel.y * GRAVITY;
  imu_msg.linear_acceleration.z = data.accel.z * GRAVITY;
}

void draw_imu_data(const m5::imu_data_t & data)
{
  M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(8, 56);
  M5.Display.printf("topic: %s\n", IMU_TOPIC);
  M5.Display.printf("count: %lu\n\n", (unsigned long) publish_count);
  M5.Display.printf("acc[g]  %7.3f %7.3f %7.3f\n", data.accel.x, data.accel.y, data.accel.z);
  M5.Display.printf("gyro[dps]%7.2f %7.2f %7.2f\n", data.gyro.x, data.gyro.y, data.gyro.z);
}

void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);
  draw_status("Starting WiFi transport...");

  set_microros_wifi_transports(wifi_ssid, wifi_pass, agent_ip, AGENT_PORT);

  draw_status("Connecting to micro-ROS Agent...");

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "m5stack_imu_node", "", &support));
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    IMU_TOPIC));

  init_imu_message();

  if (!M5.Imu.isEnabled()) {
    draw_status("IMU not found.");
    error_loop();
  }

  draw_status("Publishing IMU data.");
}

void loop()
{
  M5.update();

  uint32_t now_ms = millis();
  if (now_ms - last_publish_ms < PUBLISH_PERIOD_MS) {
    delay(1);
    return;
  }
  last_publish_ms = now_ms;

  if (M5.Imu.update()) {
    auto data = M5.Imu.getImuData();
    fill_imu_message(data);
    RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
    publish_count++;
    draw_imu_data(data);
  }
}
