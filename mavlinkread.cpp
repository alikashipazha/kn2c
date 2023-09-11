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
mavlink_status_t status;
mavlink_attitude_t attitude;
mavlink_local_position_ned_t local_pos_ned;


/*uint8_t decode(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
        uint8_t msg_received = mavlink_frame_char(chan, c, r_message, r_mavlink_status);
    if (msg_received == MAVLINK_FRAMING_BAD_CRC ||
	msg_received == MAVLINK_FRAMING_BAD_SIGNATURE) {
	    // we got a bad CRC. Treat as a parse failure
	    mavlink_message_t* rxmsg = mavlink_get_channel_buffer(chan);
	    mavlink_status_t* status = mavlink_get_channel_status(chan);
    }
    return msg_received;
}*/


uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t *r_message, mavlink_status_t *r_mavlink_status);
 
int main()
{    
    int fd = open("\\\\.\\COM8", O_RDWR);
    if (fd == -1)  {
        printf("\nserialport_init: Unable to open port =%d",fd);
        return -1;}
    else{printf("\nserialport is open");}/**/
    uint8_t b[500];

while (1)
{
int n = read(fd,&b,500);//1
for(int i=0;i<=499;i++){
        if(mavlink_parse_char(MAVLINK_COMM_0, b[i], &msg, &status)){
            //decode(MAVLINK_COMM_0,b[i],&msg,&status);
            //printf("\n1");
            printf("%x",msg.msgid);//Hello
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_ATTITUDE:
                    mavlink_msg_attitude_decode(&msg, &attitude);
                    printf(" \n roll: %d , pitch: %d , yaw: %d",attitude.roll,attitude.pitch,attitude.yaw);

                    break;
                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                    mavlink_msg_local_position_ned_decode(&msg, &local_pos_ned);
                    printf(" \n x: %d , y: %d , z: %d",local_pos_ned.x,local_pos_ned.y,local_pos_ned.z);

                    break;
                    }}
//printf("Hello");
            }}
    return 0;
}

