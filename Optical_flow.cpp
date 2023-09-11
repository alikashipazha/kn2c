  #include <fstream>
  #include <fcntl.h>  
  #include <iostream>
#include <string.h>
#include <F:\Mindw\c_library_v2-master\minimal/mavlink.h>
#include <F:\visual studio\c_library_v2-master\common/mavlink_msg_optical_flow.h>
mavlink_optical_flow_t optical_flow;
mavlink_message_t msg;

uint16_t mavlink_msg_optical_flow_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_optical_flow_t* optical_flow);

int main()
{
    int fd = open("\\\\.\\COM12", O_RDWR  ); // تنظیم پورت سریال
    if (fd == -1)  {
    printf("\nserialport_init: Unable to open port =%d",fd);
    return -1;}
    else{printf("\nserialport is open");}

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint8_t len;

mavlink_optical_flow_t optical_flow_msg;
// fill in the optical_flow_msg fields
optical_flow_msg.time_usec = get_time_usec();
optical_flow_msg.flow_comp_m_x = get_flow_comp_m_x();
optical_flow_msg.flow_comp_m_y = get_flow_comp_m_y();
optical_flow_msg.flow_x = get_flow_x();
optical_flow_msg.flow_y = get_flow_y();
optical_flow_msg.quality = get_quality();
optical_flow_msg.sensor_id = get_sensor_id();
mavlink_msg_optical_flow_encode(1,200,&msg,&optical_flow);
len=mavlink_msg_to_send_buffer(buf, &msg);
write( fd, &len ,MAVLINK_MAX_PACKET_LEN);
    return 0;
}
