#include <iostream>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include </home/fatemeh/mavlink/lib/c_library_v2/common/mavlink.h>

using namespace std;
void sendMavlinkMessage(int tcpSocket, const mavlink_message_t& msg, const sockaddr_in& targetAddr) {
    // Encode the MAVLink message into a buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int messageLength = mavlink_msg_to_send_buffer(buffer, &msg);

    // Send the MAVLink message over UDP
    ssize_t sentBytes = sendto(tcpSocket, buffer, messageLength, 0, (struct sockaddr*)&targetAddr, sizeof(targetAddr));
    if (sentBytes == -1) {
        std::cerr << "Error: Failed to send MAVLink message." << std::endl;
        close(tcpSocket);
        exit(1);
    }
}

int main() {

uint32_t time_boot_ms = 0;
uint8_t target_system = 1;//0: request ping from all receiving systems. If greater than 0: message is a ping response and number is the system id of the requesting system
uint8_t target_component = 1;
uint8_t coordinate_frame = MAV_FRAME_BODY_FRD;   //7 11
uint16_t type_mask;

    // Configure the IP address and TCP port of the SITL simulator
    const char* targetIp = "127.0.0.1";  // IP address of SITL (localhost in this example)
    int targetPort = 5762;              // TCP port of SITL (default for MAVLink)

    // Create a TCP socket
    int tcpSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcpSocket == -1) {
        std::cerr << "Error: Failed to create TCP socket." << std::endl;
        return 1;
    }

    // Configure the destination address
    struct sockaddr_in targetAddr{};
    targetAddr.sin_family = AF_INET;
    targetAddr.sin_port = htons(targetPort);
    if (inet_pton(AF_INET, targetIp, &(targetAddr.sin_addr)) <= 0) {
        std::cerr << "Error: Invalid target IP address." << std::endl;
        close(tcpSocket);
        return 1;
    }

    // Connect to the SITL simulator
    if (connect(tcpSocket, (struct sockaddr*)&targetAddr, sizeof(targetAddr)) == -1) {
        std::cerr << "Error: Failed to connect to SITL." << std::endl;
        close(tcpSocket);
        return 1;
    }


    // Create a MAVLink message and buffer for communication
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // // Send a heartbeat message to ArduCopter
    // mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_STABILIZE_DISARMED, 0, MAV_STATE_ACTIVE);
    // // uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // // send(sock, buf, len, 0);
    // sendMavlinkMessage(tcpSocket, msg, targetAddr);
    // usleep(1000000);

    // // Change the mode to GUIDED
    // mavlink_msg_set_mode_pack(1, 200, &msg, 1,  MAV_MODE_GUIDED_DISARMED, 0);
    // // len = mavlink_msg_to_send_buffer(buf, &msg);
    // // send(sock, buf, len, 0);
    // sendMavlinkMessage(tcpSocket, msg, targetAddr);
    // usleep(1000000);

    // mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_DISARMED, 0, MAV_STATE_ACTIVE);
    // // uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // // send(sock, buf, len, 0);
    // sendMavlinkMessage(tcpSocket, msg, targetAddr);
    // usleep(1000000);
    int t = 5000000;
    // // Arm the copter
    // mavlink_msg_command_long_pack(1, 200, &msg, 1, 0, MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0, 0);
    // // len = mavlink_msg_to_send_buffer(buf, &msg);
    // // send(sock, buf, len, 0);
    // sendMavlinkMessage(tcpSocket, msg, targetAddr);
    // usleep(t);

    // Take off to 10 meters altitude
cout << "takeoff" << endl;
    mavlink_msg_command_long_pack(1, 200, &msg, 1, 0, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 1);
    // len = mavlink_msg_to_send_buffer(buf, &msg);
    // send(sock, buf, len, 0);
    sendMavlinkMessage(tcpSocket, msg, targetAddr);
    usleep(t);
cout << "go forward" << endl;
//     type_mask = 3527;
//     for(int i = 0; i < t; i++){
        mavlink_msg_set_position_target_local_ned_pack(1, 200, &msg, time_boot_ms, target_system, target_component, coordinate_frame, 0b000111000000, 0, 20, 0, 0, 10, 0, 0, 0, 0, 0, 0);
        sendMavlinkMessage(tcpSocket, msg, targetAddr);
//     }
usleep(t);
cout << "turn to dir" << endl;    
    type_mask = 2559;
    // for(int i = 0; i < t; i++){
        //mavlink_msg_set_position_target_local_ned_pack(1, 200, &msg, time_boot_ms, target_system, target_component, coordinate_frame, type_mask, 0, 0, 0, 0.0, 0.0, 0, 0, 0, 0, 90, 0);
        mavlink_msg_command_long_pack(1, 200, &msg, 1, 0, MAV_CMD_CONDITION_YAW, 0, 0, 10, 0, 0, 0, 0, 0);
        sendMavlinkMessage(tcpSocket, msg, targetAddr);
   // }
   usleep(t);
// cout << "set alt" << endl;
//     type_mask = 3576;
//     for(int i = 0; i < t; i++){
//         mavlink_msg_set_position_target_local_ned_pack(1, 200, &msg, time_boot_ms, target_system, target_component, coordinate_frame, type_mask, 0, 0, 1000, 0.0, 0.0, 0, 0, 0, 0, 0, 0);
//         sendMavlinkMessage(tcpSocket, msg, targetAddr);
//     }
cout << "land" << endl;
    mavlink_msg_command_long_pack(1, 200, &msg, 1, 0, MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 1);
    // len = mavlink_msg_to_send_buffer(buf, &msg);
    // send(sock, buf, len, 0);
    sendMavlinkMessage(tcpSocket, msg, targetAddr);
    usleep(t);
    // Close the socket and exit
    close(tcpSocket);
    return 0;
}