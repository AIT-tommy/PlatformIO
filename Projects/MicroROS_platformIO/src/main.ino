/* 
Author: Nathan Hung with special guest Dominic Zerillo & tommy for spring cleaning
Updated: 8/11/2022   NH
         9/01/2022   DZ
         9/12/2022   TK
*/




#include <Arduino.h>                                // Arduino Framework Headers
#include <micro_ros_platformio.h>                   // MicroROS for Platform IO
#include <rcl/rcl.h>                                // ROS Client Library
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

// additional message options for SLAM and whatever
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/nav_sat_status.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/int32.h>                     // ROS2 int32 msgs
#include <std_msgs/msg/string.h>                    // ROS2 string msgs
#include <std_msgs/msg/int32_multi_array.h>         // ROS2 int32 arrays
#include <string>                                   // C++ standard string Library

// sensor/mc libraries
#include "freertos/FreeRTOS.h"                      // freeRTOS Libraries 
#include "freertos/task.h"                          // freeRTOS Task Header
#include "esp_system.h"                             
#include "soc/rtc_wdt.h"                            // RTC watchdog Library
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>   // GPS Library
#include <Adafruit_BNO055.h>                        // IMU Library

// I2C Definitions
#define PIN_SDA 4                                  // HT = 4  , TP = 23  , Dev 21
#define PIN_SCL 15                                  // HT = 15 , TP = 22  , Dev 22
#define I2C_CLK_SPEED 800000                        // I2C Clock Speed (fast --> 400000)  

//      || -------------------- microROS -------------------- ||

// ????
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}  

//publisher initialization
rcl_publisher_t imu_pub;
rcl_publisher_t gps_pub;
rcl_publisher_t temp_pub;

// new proper messages
sensor_msgs__msg__NavSatFix gps_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Temperature temp_msg;

// rclc initialization...
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//      || -------------------- FreeRTOS -------------------- ||

// Task Objects
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

// Task counters
int GPScounter = 0;
int IMUcounter = 0;
int LORAcounter = 0;

//ros time synchronization
const int timeout_ms = 1000;
static int64_t time_ns;
static time_t time_seconds;

// Mutex and Semaphores
static SemaphoreHandle_t GPSMutex;
static SemaphoreHandle_t IMUMutex;
static SemaphoreHandle_t LORAMutex;

// Empty Strings
String GPSlist = "emptyGPSlist";
String IMUlist = "emptyIMUlist";
String message = "emptyMessage";

// Task Rates
static const int RATE_GPS = 5000;               //500; //2000;  // ms
static const int RATE_IMU = 1000;                //100; //1000;  // ms
static const int RATE_LORA = 3000;              //500; //1000;  // ms

//      || -------------------- GPS -------------------- ||  

SFE_UBLOX_GNSS myGNSS;                          // M8Q GNSS (GPS) Object
float latitude = 0.0;                           // lat
float longitude = 0.0;                          // long
float altitude = 0.0;                           // alt
float GPSspeed = 0.0;                           // speed
float GPSheading = 0.0;                         // heading
//float position_covariance[9] = { };

//      || -------------------- IMU -------------------- ||

sensors_event_t orientationData, angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);        // Bosch BNO055 IMU

float Ax = 0;               // Acceleration x, y, z
float Ay = 0;
float Az = 0;
float Bx = 0;               // Magnetometer x, y, z, and roll, pitch, yaw
float By = 0;
float Bz = 0;
float Gyrox = 0;            // Gyro
float Gyroy = 0;
float Gyroz = 0;
float Euler_x = 0;          // Euler orientation
float Euler_y = 0;
float Euler_z = 0;
float Quat_x = 0;           // Quaternion orientation
float Quat_y = 0;
float Quat_z = 0;
float Quat_w = 0;
float heading = 0;

//      || --------------------------------------------------- ||
//      || -------------------- FUNCTIONS -------------------- ||
//      || --------------------------------------------------- ||

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

// This is the function where the message data is set and the msgs are actually published.  
// RCLCPP_INFO macro ensures every published msg is printed to the console
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    //RCSOFTCHECK(rcl_publish(&gps_pub, &gps_msg, NULL));
    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
    // RCSOFTCHECK(rcl_publish(&temp_pub, &temp_msg, NULL));
  }
}

// freeRTOS task for GPS
void update_latitude_and_longitude(void *pvParameter) {
  // setup
  if (!myGNSS.begin()) {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    while(1);
  }
  delay(500);
  // loop
  for (;;) {
    xSemaphoreTake(GPSMutex, portMAX_DELAY);
    GPScounter++;
    // gps_msg.latitude  = myGNSS.getLatitude()/10000000.;
    // gps_msg.longitude = myGNSS.getLongitude()/10000000.;
    // gps_msg.altitude  = myGNSS.getAltitude()/1000.;
    // gps_msg.status.status = int(myGNSS.getNAVSTATUS()) - 1;
    // gps_msg.position_covariance_type = 2;
    //position_covariance[0] = myGNSS.getHorizontalAccuracy();
    //position_covariance[4] = myGNSS.getHorizontalAccuracy();
    //position_covariance[8] = myGNSS.getVerticalAccuracy();
    // gps_msg.position_covariance[0] = myGNSS.getHorizontalAccuracy();
    // gps_msg.position_covariance[4] = myGNSS.getHorizontalAccuracy();
    // gps_msg.position_covariance[8] = myGNSS.getVerticalAccuracy();

    // latitude  = myGNSS.getLatitude();
    // longitude = myGNSS.getLongitude();
    // altitude  = myGNSS.getAltitude();
    // GPSspeed = myGNSS.getGroundSpeed();
    // GPSheading = myGNSS.getHeading();
    // Serial.println(String(myGNSS.getLatitude())+ "," + String(myGNSS.getLongitude()));

//    GPSlist = String(myGNSS.getLatitude()) + "," + String(myGNSS.getLongitude());
    // GPSlist = String(GPScounter) + "," + String(myGNSS.getLatitude()) + "," + String(myGNSS.getLongitude());

    // Serial.println(GPSlist);
    //infoBuffer[0] = latitude;
    //infoBuffer[1] = longitude;
    //infoBuffer[2] = altitude;

    rtc_wdt_feed(); //feed watchdog
    xSemaphoreGive(GPSMutex);
    vTaskDelay(RATE_GPS / portTICK_PERIOD_MS);
  }
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
//    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    Ax = x;
    Ay = y;
    Az = z;
    // infoBuffer[3] = Ax;
    // infoBuffer[4] = Ay;
    // infoBuffer[5] = Az;
  }
 else if (event->type == SENSOR_TYPE_ORIENTATION) {
//    Serial.print("Orient:");
   x = event->orientation.x;
   y = event->orientation.y;
   z = event->orientation.z;
   Euler_x = x;
   Euler_y = y;
   Euler_z = z;

 }
//  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
////    Serial.print("Mag:");
//    x = event->magnetic.x;
//    y = event->magnetic.y;
//    z = event->magnetic.z;
//  }
//  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
////    Serial.print("Gyro:");
//    x = event->gyro.x;
//    y = event->gyro.y;
//    z = event->gyro.z;
//  }
//  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
////    Serial.print("Rot:");
//    x = event->gyro.x;
//    y = event->gyro.y;
//    z = event->gyro.z;
//  }
//  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
////    Serial.print("Linear:");
//    x = event->acceleration.x;
//    y = event->acceleration.y;
//    z = event->acceleration.z;
//  }
//  else if (event->type == SENSOR_TYPE_GRAVITY) {
////    Serial.print("Gravity:");
//    x = event->acceleration.x;
//    y = event->acceleration.y;
//    z = event->acceleration.z;
//  }
//  else {
////    Serial.print("Unk:");
//  }
//
//  Serial.print("\tx= ");
//  Serial.print(x);
//  Serial.print(" |\ty= ");
//  Serial.print(y);
//  Serial.print(" |\tz= ");
//  Serial.println(z);
}


// freeRTOS task for GPS
void update_IMU_data(void *pvParameter) {
  // setup
  if (!bno.begin()) {
    Serial.print("Could not connect to the IMU sensor for 3D movement.");
    while(1);
  }

  delay(500);

  // loop
  for (;;) {
    IMUcounter++;
    xSemaphoreTake(IMUMutex, portMAX_DELAY);

  // RCCHECK(rmw_uros_sync_session(timeout_ms));
  // time_ns = rmw_uros_epoch_nanos();    

//    IMUlist = String(IMUcounter);
//    xSemaphoreGive(GPSMutex);
    
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); //Three axis orientation data based on a 360° sphere
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE); //Three axis of 'rotation speed' in rad/s
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL); //Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER); //Three axis of magnetic field sensing in micro Tesla (uT)
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER); //Three axis of acceleration (gravity + linear motion) in m/s^2
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY); //Three axis of gravitational acceleration (minus any movement) in m/s^2
    //printEvent(&linearAccelData);
    //printEvent(&orientationData);
    
    imu_msg.linear_acceleration.x = linearAccelData.acceleration.x;
    imu_msg.linear_acceleration.y = linearAccelData.acceleration.y;
    imu_msg.linear_acceleration.z = linearAccelData.acceleration.z;

    imu_msg.angular_velocity.x = angVelocityData.gyro.x;
    imu_msg.angular_velocity.y = angVelocityData.gyro.y;
    imu_msg.angular_velocity.z = angVelocityData.gyro.z;
    
    imu_msg.orientation.x = orientationData.orientation.x;
    imu_msg.orientation.y = orientationData.orientation.y;
    imu_msg.orientation.z = orientationData.orientation.z;

    // time_seconds = time_ns /1000000000 ;
    // int64_t time_rmd_ns = time_ns % 1000000000 ;


    // imu_msg.header.stamp.nanosec = time_seconds;
    // imu_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    // imu_msg.header.frame_id.data = "camera_imu_optical_frame";

    imu_msg.angular_velocity_covariance[0] = .5;
    imu_msg.angular_velocity_covariance[4] = .5;
    imu_msg.angular_velocity_covariance[8] = .5;
    imu_msg.orientation_covariance[0] = .5;
    imu_msg.orientation_covariance[4] = .5;
    imu_msg.orientation_covariance[8] = .5;
    imu_msg.linear_acceleration_covariance[0] = 2;
    imu_msg.linear_acceleration_covariance[4] = 2;
    imu_msg.linear_acceleration_covariance[8] = 2;





    // IMUlist = String(Ax) + "," + String(Ay) + "," + String(Az) + "||" + String(Gyrox) + "," + String(Gyroy) + "," + String(Gyroz) + "||" + String(heading);
    // IMUlist = String(IMUcounter) + "," + String(Gyrox) + "," + String(Gyroy) + "," + String(Gyroz);
    // Serial.println(IMUlist);
    
//    heap_caps_check_integrity_all(true);
//    heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
    
    xSemaphoreGive(IMUMutex);

    rtc_wdt_feed(); //feed watchdog
    
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}



void setup() {
  // ----------- Configure data buses
  Serial.begin(115200);                         // begin serial
  delay(500);                                   // !!! need time for serial to begin
  set_microros_serial_transports(Serial);       // set microROS transport to:  serial, wifi
  Wire.begin(PIN_SDA, PIN_SCL);                 // begin I2C
//  Wire.begin(23, 22);                 // begin I2C
  delay(100);
  //Wire.setClock(I2C_CLK_SPEED);                 // set I2C clock speed
  Wire.setClock(I2C_CLK_SPEED);                 // set I2C clock speed

  //delay(100);
  delay(1000);
  // create Mutex/Semaphores
  GPSMutex = xSemaphoreCreateMutex();
  IMUMutex = xSemaphoreCreateMutex();

  // freeRTOS tasks for GPS, IMU
  //xTaskCreatePinnedToCore(update_latitude_and_longitude, "update_latitude_and_longitude", 8912, NULL, 1, &Task1, 1);
  //delay(100);                                   //important delay
  xTaskCreatePinnedToCore(update_IMU_data, "update_IMU_data", 8912, NULL, 1, &Task2, 1);
  delay(100);                                   //important delay

  // ========================= ROS2 stuff setup..............===================
  Serial.println("ROS2 Allocator...");
  allocator = rcl_get_default_allocator();
  Serial.println("ROS2 Allocated...");
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("ROS2 Node Creating...");
  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));
  Serial.println("ROS2 Node Created...");
  // create publisher
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  //   "micro_ros_platformio_node_publisher"));

  // //  create gps publisher
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &gps_pub,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
  //   "gps_NavSatFix"));

  //  create imu publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu"));

  // // create temp publisher
  //RCCHECK(rclc_publisher_init_best_effort(
    //&temp_pub,
    //&node,
    //ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
    //"temperature"));    

  // create timer,
  const unsigned int timer_timeout = 1; //this is the speed on which it is published, set to 1 for extra speed
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  //msg.data = 0;

  //Serial.println("Before....");
  //msg2.data.data[0] = 0.1;
  // msg2.data.data[1] = 0.1;
  //Serial.println("After....");

}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}



/**

Platform IO CLI bash commands

|----- list usbs ------|

lsusb
ls /dev/ttyUSB*

|----- permissions ------|          if you don't have permission to use upload port /dev/ttyUSB*

sudo chown <insert_username_here> /dev/ttyUSB0

|----- cleaning -----|              if you are flashing again, clean microros previous builds

pio run --target clean_microros

|----- lib update -----|            if you are flashing again, update the libraries

pio pkg install

|----- building -----|              build for board   **note that your device may not be /dev/ttyUSB0. CHECK your devices

pio run --target /dev/ttyUSB0

|----- flashing -----|              flash to board  

pio run --target upload

**/

/*
                                                   ._____.
                                          _________|     |________
                                         | o       | USB |      o |
                                         |         |_____|        |
                                     3V3 [o]                    [o] VIN
                                     GND [o]                    [o] GND
     Touch3 / HSPI_CS0 / ADC2_3 / GPIO15 [o]                    [o] GPIO13 / ADC2_4 / HSPI_ID / Touch4
 CS / Touch2 / HSPI_WP / ADC2_2 /  GPIO2 [o]        ESP32       [o] GPIO12 / ADC2_5 / HSPI_Q / Touch5
      Touch0 / HSPI_HD / ADC2_0 /  GPIO4 [o]       Dev Kit      [o] GPIO14 / ADC2_6 / HSPI_CLK / Touch6
                         U2_RXD / GPIO16 [o]         V1         [o] GPIO27 / ADC2_7 / Touch7
                         U2_TXD / GPIO17 [o]                    [o] GPIO26 / ADC2_9 / DAC2
                      V_SPI_CS0 /  GPIO5 [o]  ________________  [o] GPIO25 / ADC2_8 / DAC1
                SCK / V_SPI_CLK / GPIO18 [o] |                | [o] GPIO33 / ADC1_5 / Touch8 / XTAL32
        U0_CTS / MSIO / V_SPI_Q / GPIO19 [o] |                | [o] GPIO32 / ADC1_4 / Touch9 / XTAL32
                 SDA / V_SPI_HD / GPIO21 [o] |                | [o] GPIO35 / ADC1_7 
                  CLK2 / U0_RXD /  GPIO3 [o] |                | [o] GPIO34 / ADC1_6 
                  CLK3 / U0_TXD /  GPIO1 [o] |                | [o] GPIO39 / ADC1_3 / SensVN 
        SCL / U0_RTS / V_SPI_WP / GPIO22 [o] |                | [o] GPIO36 / ADC1_0 / SensVP 
                MOSI / V_SPI_WP / GPIO23 [o] |                | [o] EN 
                                         |   |________________|   |
                                         | o |  __    __   __ | o |
                                         |___|_|__|__|__|__|__|___|
                             

*/