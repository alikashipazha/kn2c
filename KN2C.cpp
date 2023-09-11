#include <F:\Mindw\c_library_v2-master\minimal/mavlink.h>
#include <F:\Mindw\c_library_v2-master\minimal/mavlink_msg_heartbeat.h>
#include <F:\Mindw\c_library_v2-master\common/mavlink_msg_set_position_target_local_ned.h>
#include <F:\Mindw\c_library_v2-master\common/common.h>
#include <fstream>
#include <fcntl.h>  
#include <C:\Users\ticktak\Downloads\Compressed\arduino-serial-main/arduino-serial-lib.h>
#include <F:\Mindw\c_library_v2-master\common/mavlink_msg_local_position_ned.h>
#include <iostream>
/////////////////////////////////////////////////////////////////
//  - system_id: شناسه سیستم فرستنده                         //
//  - component_id: شناسه کامپوننت فرستنده                   //
//  - msg: پیام mavlink                                        //
//  - type: نوع سیستم                                         //
//  - autopilot: نوع پلیت خودرو                               //
//  - base_mode: حالت پایه (مثلاً مسلط یا نامسلط)              //
//  - custom_mode: حالت سفارشی                                //
//  - system_status: وضعیت سیستم (مثلاً فعال یا غیرفعال)       //
/////////////////////////////////////////////////////////////////

//ss


mavlink_message_t msg;
 //  system_id ID of this system
 //  component_id ID of this component (e.g. 200 for IMU)
 //  msg The MAVLink message to compress the data into
uint8_t type = MAV_TYPE_QUADROTOR;//;2
uint8_t autopilot = MAV_AUTOPILOT_GENERIC;//;0
uint8_t base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;//64;
uint32_t custom_mode = 0;
uint8_t system_status =MAV_STATE_ACTIVE ;//;4
uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t type, uint8_t autopilot, 
                                    uint8_t base_mode, uint32_t custom_mode, uint8_t system_status);



                                    
////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////
//- system_id: شناسه سیستم فرستنده                                                          //
//- component_id: شناسه کامپوننت فرستنده                                                    //
//- msg: پیام mavlink                                                                        //
//- time_boot_ms: زمان اجرای دستور در سیستم فرستنده (به میلی‌ثانیه)                        //
//- target_system: شناسه سیستم مقصد                                                         //
//- target_component: شناسه کامپوننت مقصد                                                   //
//- coordinate_frame: سیستم مختصات مورد استفاده (مثلاً NED یا MAV_FRAME_BODY_NED)            //
//              __________________________________________________________________           //
//              |   1	MAV_FRAME_LOCAL_NED	                                     |           //
//              |   NED local tangent frame (x: North, y: East, z: Down)         |           //
//              |   with origin fixed relative to earth.                         |           //
//              |   7	MAV_FRAME_LOCAL_OFFSET_NED	                             |           //
//              |   NED local tangent frame (x: North, y: East, z: Down)         |           //
//              |   with origin that travels with the vehicle.                   |           //
//              |   12	MAV_FRAME_BODY_FRD	                                     |           //
//              |   FRD local frame aligned to the vehicle's attitude            |           //
//              |   (x: Forward, y: Right, z: Down)                              |           //
//              |   with an origin that travels with vehicle.                    |           //
//              |________________________________________________________________|           //
//- type_mask: ماسک نوع دستور (مثلاً حرکت در جهت x، y و z)                                   //
//              __________________________________________________________________           //
//              |   Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)      |           //
//              |   Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)      |           //
//              |   Use Acceleration : 0b110000111111 / 0x0C3F / 3135 (decimal)  |           //
//              |   Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)       |           //
//              |   Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal) |           //
//              |   Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)           |           //
//              |   Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)      |           // 
//              |________________________________________________________________|           //
//                                                                                           //
//- x: موقعیت در جهت x (متر)                                                                //
//- y: موقعیت در جهت y (متر)                                                                //
//- z: موقعیت در جهت z (متر)                                                                //
//- vx: سرعت در جهت x (متر بر ثانیه)                                                        //
//- vy: سرعت در جهت y (متر بر ثانیه)                                                        //
//- vz: سرعت در جهت z (متر بر ثانیه)                                                        //
//- afx: شتاب در جهت x (متر بر مربع ثانیه)                                                  //
//- afy: شتاب در جهت y (متر بر مربع ثانیه)                                                  //
//- afz: شتاب در جهت z (متر بر مربع ثانیه)                                                  //
//- yaw: زاویه yaw (درجه)                                                                    //
//- yaw_rate: نرخ تغییر yaw (درجه بر ثانیه)                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////4

uint32_t time_boot_ms = 0;
uint8_t target_system = 1;//0: request ping from all receiving systems. If greater than 0: message is a ping response and number is the system id of the requesting system
uint8_t target_component = 1;
uint8_t coordinate_frame = MAV_FRAME_BODY_FRD;   //7 11
                                //MAV_FRAME_LOCAL_OFFSET_NED:NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.;
uint16_t type_mask ;//MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TYPEMASK_X_VELOCITY | MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TYPEMASK_YAW_RATE;
uint16_t mavlink_msg_set_position_target_local_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t time_boot_ms,
                                                        uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, 
                                                        uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, 
                                                        float afx, float afy, float afz, float yaw, float yaw_rate);

int main()
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
printf("Hello");

    //int fd = open("CAM10",O_RDONLY); // 127.0.0.1:5760تنظیم پورت سریال
    int fd = open("\\\\.\\COM11",   O_RDWR);
    if (fd == -1)  {
        printf("\nserialport_init: Unable to open port =%d",fd);
        return -1;}
    else{printf("\nserialport is open");}

//msg=;
uint8_t len;
while(1){

uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_msg_heartbeat_pack(1, 200, &msg, type, autopilot, base_mode, custom_mode, system_status);
    len=mavlink_msg_to_send_buffer(buf, &msg);
    write( fd, &buf ,MAVLINK_MAX_PACKET_LEN);}

float x = 0;//10
float y = 0;
float z = 0;
float vx = 0.01;//1
float vy = 0;
float vz = 0;
float afx = 0;
float afy = 0;
float afz = 0;
float yaw = 0;
float yaw_rate = 0;
type_mask=3527;//3520

mavlink_msg_set_position_target_local_ned_pack(1, 200, &msg, time_boot_ms, target_system, target_component, coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate);


    
    len=mavlink_msg_to_send_buffer(buf, &msg);
        write( fd, &len ,MAVLINK_MAX_PACKET_LEN);
printf("\n/nHello");
return 0;
 
}