#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libwebsockets.h>

// Structure to store data for each WebSocket session
struct per_session_data {
};

//#define USE_TAIL_READING

static FILE *s_file = NULL; // File pointer declaration
static long last_pos = 0;   // Last read position in the file
static char last_line[1024] = ""; // Last read line storage

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

#if defined(USE_TAIL_READING)
            // If USE_TAIL_READING is defined, read from the end of the file
            fseek(s_file, 0, SEEK_END); // Move to end of the file
            last_pos = ftell(s_file);   // Store last position
            printf("Initial file position: %ld\n", last_pos); // Print initial position

#else
            // Otherwise, start from the beginning of the file
            fseek(s_file, 0, SEEK_SET); // Move to the beginning of the file
#endif

            lws_callback_on_writable(wsi);
            break;

        case LWS_CALLBACK_SERVER_WRITEABLE: // Handle send data event
#if defined(USE_TAIL_READING)
            // If USE_TAIL_READING is defined, read from the end of the file
            fseek(s_file, 0, SEEK_END);
            long file_size = ftell(s_file);

            // Find the start of the last line
            for (long i = file_size - 2; i >= 0; i--) {
                fseek(s_file, i, SEEK_SET);
                if (fgetc(s_file) == '\n') {
                    last_pos = ftell(s_file);
                    break;
                }
            }

            fseek(s_file, last_pos, SEEK_SET);
            if (fgets(line, sizeof(line), s_file)) {
                printf("Read last line: %s", line);
                strcpy(last_line, line);
                lws_write(wsi, (unsigned char *)line, strlen(line), LWS_WRITE_TEXT);
            } else if (strlen(last_line) > 0) {
                printf("Sending last line again: %s", last_line);
                lws_write(wsi, (unsigned char *)last_line, strlen(last_line), LWS_WRITE_TEXT);
            }

#else
            if (fgets(line, sizeof(line), s_file)) {
                printf("Read line: %s", line);
                lws_write(wsi, (unsigned char *)line, strlen(line), LWS_WRITE_TEXT);
            } else {
                // If end of file is reached, go back to the beginning
                fseek(s_file, 0, SEEK_SET);
            }
#endif

            //usleep(1000*1000); // Wait for 1 second
            usleep(100*1000); // Wait for 100ms
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
    static struct lws_protocols protocols[] =
    {
        {
            "demo-protocol",
            callback,
            sizeof(struct per_session_data),
            0,
            NULL, NULL, NULL
        },
        { NULL, NULL, 0, 0 }
    };

    struct lws_context_creation_info info = {
        .port = 3001,
        .protocols = protocols
    };
    struct lws_context *context = lws_create_context(&info);

    if (!context) {
        printf("Failed to create WebSocket context.\n");
        return -1;
    }

    while (1) {
        lws_service(context, 50);
    }

    lws_context_destroy(context);

    return 0;
}