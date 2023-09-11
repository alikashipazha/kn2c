#include <F:\Mindw\c_library_v2-master\minimal/mavlink.h>
#include <F:\Mindw\c_library_v2-master\minimal/mavlink_msg_heartbeat.h>
#include <F:\Mindw\c_library_v2-master\common/mavlink_msg_set_position_target_local_ned.h>
  #include <fstream>
  #include <fcntl.h>
  //#include <io  
  //#include <C:\Users\ticktak\Downloads\Compressed\arduino-serial-main/arduino-serial-lib.h>
#include <F:\Mindw\c_library_v2-master\common/mavlink_msg_local_position_ned.h>
#include <F:\Mindw\c_library_v2-master\common/mavlink_msg_attitude.h>
#include <F:\Mindw\c_library_v2-master/mavlink_helpers.h>
#include <F:\Mindw\c_library_v2-master\common/mavlink_msg_highres_imu.h>

//mavlink_msg_highres_imu.h
#include <iostream>
#include <string.h>
mavlink_message_t msg;
mavlink_attitude_t attitude;
mavlink_local_position_ned_t local_pos_ned;
mavlink_status_t status;
mavlink_highres_imu_t highres_imu;

uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t *r_message, mavlink_status_t *r_mavlink_status);
uint8_t mavlink_frame_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);
/////////////////////////
uint8_t type = 2;//MAV_TYPE_QUADROTOR;
uint8_t autopilot = 0;//MAV_AUTOPILOT_GENERIC;
uint8_t base_mode = 64;//ss//MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
uint32_t custom_mode = 0;
uint8_t system_status = 4;//MAV_STATE_ACTIVE;
uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t type, uint8_t autopilot, 
                                    uint8_t base_mode, uint32_t custom_mode, uint8_t system_status);





                                    
////////////////////////////////////////////////
int main()
{    


    int fd = open("\\\\.\\COM8", O_RDWR);
    if (fd == -1)  {
        printf("\nserialport_init: Unable to open port =%d",fd);
        return -1;}
    else{printf("\nserialport is open");}
    uint32_t b;

uint8_t len;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

mavlink_msg_heartbeat_pack(1, 200, &msg, type, autopilot, base_mode, custom_mode, system_status);
    len=mavlink_msg_to_send_buffer(buf, &msg);
    std::cout.write( fd, &buf ,MAVLINK_MAX_PACKET_LEN);



while (1)
{
    int n = read(fd,&b,1);
//for(int i=0;i<1000;i++){
if(mavlink_frame_char(MAVLINK_COMM_0,b, &msg, &status) != MAVLINK_FRAMING_INCOMPLETE){
     //printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
            switch (msg.msgid) {
                case 105:

 
                    mavlink_msg_highres_imu_decode(&msg,&highres_imu);


                    printf(" \n xacc: %f , yacc: %f , zacc: %f",highres_imu.xacc,highres_imu.yacc,highres_imu.zacc);


                    printf(" \n xgyro: %f , ygyro: %f , zgyro: %f",highres_imu.xgyro,highres_imu.ygyro,highres_imu.zgyro);


                    break;

                    }
}}//}
printf("Hello");

    return 0;
}
















