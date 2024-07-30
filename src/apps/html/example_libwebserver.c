
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libwebsockets.h>

// Structure to store data for each WebSocket session
struct per_session_data {
};

static FILE *s_file = NULL; // 파일 포인터 선언
static long last_pos = 0; // 파일에서 마지막으로 읽은 위치
static char last_line[1024] = ""; // 마지막으로 읽은 라인 저장

// Callback function for WebSocket server messages
int callback(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len) {
    char line[1024];

    switch (reason) {
        case LWS_CALLBACK_ESTABLISHED: // Handle new connection event
            // Add a timer to send a timestamp every second
            s_file = fopen("/tmp/rx_db_sample_1.csv", "r");
            if (s_file == NULL) {
                perror("Failed to open file");
                return -1;
            }
            fseek(s_file, 0, SEEK_END); // 파일 끝으로 이동
            last_pos = ftell(s_file); // 마지막 위치 저장
            printf("Initial file position: %ld\n", last_pos); // 초기 파일 위치 출력

            lws_callback_on_writable(wsi);
            break;
        case LWS_CALLBACK_SERVER_WRITEABLE: // Handle send data event
            // 파일의 끝에서부터 역방향으로 탐색하여 마지막 줄의 시작을 찾음
            fseek(s_file, 0, SEEK_END);
            long file_size = ftell(s_file);

            // 마지막 줄의 시작 부분 찾기
            for (long i = file_size - 2; i >= 0; i--) {
                fseek(s_file, i, SEEK_SET);
                if (fgetc(s_file) == '\n') {
                    last_pos = ftell(s_file);
                    break;
                }
            }

            // 마지막 줄의 데이터를 읽음
            fseek(s_file, last_pos, SEEK_SET);
            if (fgets(line, sizeof(line), s_file)) {
                printf("Read last line: %s", line);
                strcpy(last_line, line);
                lws_write(wsi, (unsigned char *)line, strlen(line), LWS_WRITE_TEXT);
            } else if (strlen(last_line) > 0) {
                // 새로운 데이터가 없을 경우, 마지막으로 읽은 데이터 전송
                printf("Sending last line again: %s", last_line);
                lws_write(wsi, (unsigned char *)last_line, strlen(last_line), LWS_WRITE_TEXT);
            }

            // 100ms 후 다시 쓰기 가능 상태로 설정
            usleep(1000000);
            lws_callback_on_writable(wsi);
            break;

        case LWS_CALLBACK_CLOSED:
            if (s_file != NULL) {
                fclose(s_file);
                s_file = NULL;
            }
            break;

        default:
            break;
    }

    return 0;
}

int main(int argc, char **argv)
{
    // Create the WebSocket protocol
    static struct lws_protocols protocols[] =
    {
        {
            "demo-protocol", // Protocol name, should match the WebSocket protocol in the frontend code
            callback, // Callback function pointer
            sizeof(struct per_session_data), // Size of data for each session (connection)
            0, // No additional protocol parameters
            NULL, NULL, NULL
        },
        { NULL, NULL, 0, 0 } // Protocol list ends with NULL
    };

    // Create the WebSocket context
    struct lws_context_creation_info info = {
        .port = 3001, // Listening port number
        .protocols = protocols // Protocol list
    };
    struct lws_context *context = lws_create_context(&info);

    // Check if WebSocket context creation was successful
    if (!context) {
        printf("Failed to create WebSocket context.\n");
        return -1;
    }

    // Enter the loop and wait for WebSocket connections
    while (1) {
        lws_service(context, 50);
    }

    // Clean up and close the WebSocket context
    lws_context_destroy(context);

    return 0;
}