#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include </home/fatemeh/mavlink/lib/c_library_v2/common/mavlink.h>
#include </home/fatemeh/mavlink/lib/c_library_v2/minimal/mavlink.h>
using namespace std;


// Function to send a MAVLink message over UDP
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

    // Create a MAVLink message for heartbeat
    mavlink_message_t heartbeatMsg;
    mavlink_msg_heartbeat_pack(1, 200, &heartbeatMsg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_STABILIZE_ARMED, 0, MAV_STATE_ACTIVE);

    // Create a MAVLink message for takeoff
    mavlink_message_t takeoffMsg;
    mavlink_msg_command_long_pack(1, 200, &takeoffMsg, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    // Create a MAVLink message for velocity
    // mavlink_message_t velocityMsg;
    // mavlink_msg_set_position_target_local_ned_pack(1, 200, &velocityMsg, 
    //     MAVLINK_TARGETING_FRAMES_LOCAL_NED, 0, MAV_FRAME_LOCAL_NED, 
    //     MAVLINK_IGNORE_POSITION, 0, 0, 0, 1.0, 2.0, 3.0, 0, 0, 0, 0, 0);
    sendMavlinkMessage(tcpSocket, heartbeatMsg, targetAddr);
    sendMavlinkMessage(tcpSocket, takeoffMsg, targetAddr);
    usleep(1000000);

    //
    mavlink_message_t msg;
        // Encode the MAVLink message into a buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int messageLength = mavlink_msg_to_send_buffer(buffer, &msg);
    ssize_t sentBytes = sendto(tcpSocket, buffer, messageLength, 0, (struct sockaddr*)&targetAddr, sizeof(targetAddr));

    while (true) {
        // Send the MAVLink heartbeat message
        cout<<"into while"<<endl;
        sendMavlinkMessage(tcpSocket, heartbeatMsg, targetAddr);

        // Delay for a moment
        usleep(500000);  // Sleep for 1 second

        // Send the MAVLink takeoff message
            mavlink_msg_set_position_target_local_ned_pack(1,200,&msg,0,1,1,11,3520,1000,0,0,10,0,0,0,0,0,0,0);

        messageLength = mavlink_msg_to_send_buffer(buffer, &msg);
        sendto(tcpSocket, buffer, messageLength, 0, (struct sockaddr*)&targetAddr, sizeof(targetAddr));
        if (sentBytes == -1) {
            std::cerr << "Error: Failed to send MAVLink message." << std::endl;
            // close(tcpSocket);
            // return 1;
        }

        // Delay for a moment
        usleep(500000);  // Sleep for 1 second

        // Send the MAVLink velocity message
        //sendMavlinkMessage(udpSocket, velocityMsg, targetAddr);

        // Delay for a moment
        //usleep(1000000);  // Sleep for 1 second
    }

    // Close the UDP socket (usually not reached in this example)
    close(tcpSocket);

    return 0;
}