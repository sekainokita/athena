// gnss_pose_to_socket.cpp
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h> // euler 데이터를 포함하는 메시지 타입
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <string>

int sockfd;
std::string position_data;
std::string euler_data;

// /gnss_pose 콜백 함수
void gnssPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    position_data = "Position - x: " + std::to_string(msg->pose.position.x) + 
                    ", y: " + std::to_string(msg->pose.position.y) + 
                    ", z: " + std::to_string(msg->pose.position.z) + "\n";
    std::string data = position_data + euler_data;
    send(sockfd, data.c_str(), data.size(), 0);
}

// /filter/euler 콜백 함수
void eulerCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    euler_data = "Euler - roll: " + std::to_string(msg->vector.x) + 
                 ", pitch: " + std::to_string(msg->vector.y) + 
                 ", yaw: " + std::to_string(msg->vector.z) + "\n";
    std::string data = position_data + euler_data;
    send(sockfd, data.c_str(), data.size(), 0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gnss_pose_to_socket");
    ros::NodeHandle nh;

    if (argc != 3) {
        std::cerr << "Usage: gnss_pose_to_socket <ip> <port>" << std::endl;
        return 1;
    }

    const char* ip = argv[1];
    int port = std::stoi(argv[2]);

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &server_addr.sin_addr);

    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("connect");
        close(sockfd);
        return 1;
    }

    // /gnss_pose 토픽 구독
    ros::Subscriber gnss_pose_sub = nh.subscribe("/gnss_pose", 1000, gnssPoseCallback);
    // /filter/euler 토픽 구독
    ros::Subscriber euler_sub = nh.subscribe("/filter/euler", 1000, eulerCallback);

    ros::spin();

    close(sockfd);
    return 0;
}
