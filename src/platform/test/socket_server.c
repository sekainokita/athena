#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <pthread.h>

#define PORT 8080

typedef struct DI_GPS_DATA_t {
    float fAccX;
    float fAccY;
    float fAccZ;
    float fGyrX;
    float fGyrY;
    float fGyrZ;
    float fMagX;
    float fMagY;
    float fMagZ;
    float fQuaternionW;
    float fQuaternionX;
    float fQuaternionY;
    float fQuaternionZ;
    float fEulerRoll;
    float fEulerPitch;
    float fEulerYaw;
    float fLatitude;
    float fLongitude;
    float fAltitude;
    float fVelocityEast;
    float fVelocityNorth;
    float fVelocityUp;
} DI_GPS_DATA_T;

DI_GPS_DATA_T gps_data;
pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;

void* handle_connection(void* socket_desc) {
    int new_socket = *(int*)socket_desc;
    free(socket_desc);
    char buffer[1024];
    int valread;

    while ((valread = read(new_socket, buffer, 1024)) > 0) {
        buffer[valread] = '\0';

        pthread_mutex_lock(&data_mutex);
        printf("Received data: %s\n", buffer); // 수신된 데이터 로그 추가

        int scanned = sscanf(buffer, "Position - x: %f, y: %f, z: %f\nEuler - roll: %f, pitch: %f, yaw: %f\n",
               &gps_data.fLatitude, &gps_data.fLongitude, &gps_data.fAltitude,
               &gps_data.fEulerRoll, &gps_data.fEulerPitch, &gps_data.fEulerYaw);
        pthread_mutex_unlock(&data_mutex);

        if (scanned == 6) { // 데이터가 성공적으로 파싱되었을 때만 출력
            printf("Updated GPS Data:\n");
            printf("Latitude: %f, Longitude: %f, Altitude: %f\n", gps_data.fLatitude, gps_data.fLongitude, gps_data.fAltitude);
            printf("Roll: %f, Pitch: %f, Yaw: %f\n", gps_data.fEulerRoll, gps_data.fEulerPitch, gps_data.fEulerYaw);
        } else {
            printf("Error parsing data\n");
        }
    }

    close(new_socket);
    return NULL;
}

int main() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    while ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) >= 0) {
        printf("Connection accepted\n"); // 클라이언트 연결 로그 추가
        pthread_t thread_id;
        int* new_sock = malloc(sizeof(int));
        *new_sock = new_socket;

        if (pthread_create(&thread_id, NULL, handle_connection, (void*)new_sock) < 0) {
            perror("could not create thread");
            free(new_sock);
        }

        pthread_detach(thread_id);
    }

    if (new_socket < 0) {
        perror("accept failed");
        exit(EXIT_FAILURE);
    }

    close(server_fd);
    return 0;
}

