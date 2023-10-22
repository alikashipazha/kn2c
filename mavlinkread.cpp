#include <F:\Mindw\c_library_v2-master\minimal/mavlink.h>
#include <F:\Mindw\c_library_v2-master\minimal/mavlink_msg_heartbeat.h>
#include <F:\Mindw\c_library_v2-master\common/mavlink_msg_set_position_target_local_ned.h>
  #include <fstream>
  #include <fcntl.h>  
#include <F:\Mindw\c_library_v2-master\common/mavlink_msg_local_position_ned.h>
#include <F:\Mindw\c_library_v2-master\common/mavlink_msg_attitude.h>
#include <iostream>
#include <string.h>

mavlink_message_t msg;
mavlink_attitude_t attitude;
mavlink_local_position_ned_t local_pos_ned;


uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t *r_message, mavlink_status_t *r_mavlink_status);

int main()
{    
    mavlink_message_t msg;
    mavlink_status_t status;
    
    int fd = open("\\\\.\\COM8", O_RDWR  ); // تنظیم پورت سریال
    if (fd == -1)  {
        printf("\nserialport_init: Unable to open port =%d",fd);
        return -1;}
    else{printf("\nserialport is open");}
    uint8_t b;

while (1)
{
 //printf("\n1");
    int n = read(fd,&b,1);
 //printf("\n2");
    mavlink_parse_char(0, b, &msg, &status);
 //("\n3");

         mavlink_parse_char(MAVLINK_COMM_0, b, &msg, &status);//
 //printf("\n4");

printf("\n%d",msg.msgid);//

            switch (msg.msgid) {
                case MAVLINK_MSG_ID_ATTITUDE:
                    mavlink_msg_attitude_decode(&msg, &attitude);
                    printf(" \n roll: %d , pitch: %d , yaw: %d",attitude.roll,attitude.pitch,attitude.yaw);

                    break;
                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                    mavlink_msg_local_position_ned_decode(&msg, &local_pos_ned);
                    printf(" \n x: %d , y: %d , z: %d",local_pos_ned.x,local_pos_ned.y,local_pos_ned.z);

                    break;
printf("Hello");

    return 0;
}
}
}
