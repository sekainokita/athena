
/******************************************************************************
*
* Copyright (C) 2023 - 2028 KETI, All rights reserved.
*                           (Korea Electronics Technology Institute)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running for Korean Government Project, or
* (b) that interact with KETI project/platform.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* KETI BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the KETI shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from KETI.
*
******************************************************************************/
/******************************************************************************/
/**
*
* @file example_libwebserver.c
*
* @note
*
* Library Web Server Example
*
******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libwebsockets.h>

struct per_session_data {

};

#define USE_TAIL_READING
#define EG_WEB_SERVER_FILE_DB_SAMPLE "/tmp/rx_db_sample_1.csv"
#define EG_WEB_SEFVER_FILE_DB_TX     "/tmp/db_v2x_tx_temp_writing.csv"

static FILE *s_file = NULL;
static long last_pos = 0;
static char last_line[1024] = "";

int callback(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len)
{
    char line[1024];

    switch (reason)
    {
        case LWS_CALLBACK_ESTABLISHED:
            s_file = fopen(EG_WEB_SEFVER_FILE_DB_TX, "r");
            if (s_file == NULL)
            {
                perror("Failed to open file");
                return -1;
            }

#if defined(USE_TAIL_READING)
            fseek(s_file, 0, SEEK_END);
            last_pos = ftell(s_file);
            printf("Initial file position: %ld\n", last_pos);

#else
            fseek(s_file, 0, SEEK_SET);
#endif

            lws_callback_on_writable(wsi);
            break;

        case LWS_CALLBACK_SERVER_WRITEABLE:
#if defined(USE_TAIL_READING)
            fseek(s_file, 0, SEEK_END);
            long file_size = ftell(s_file);

            for (long i = file_size - 2; i >= 0; i--)
            {
                fseek(s_file, i, SEEK_SET);
                if (fgetc(s_file) == '\n')
                {
                    last_pos = ftell(s_file);
                    break;
                }
            }

            fseek(s_file, last_pos, SEEK_SET);
            if (fgets(line, sizeof(line), s_file))
            {
                printf("Read last line: %s", line);
                strcpy(last_line, line);
                lws_write(wsi, (unsigned char *)line, strlen(line), LWS_WRITE_TEXT);
            }
            else if (strlen(last_line) > 0)
            {
                printf("Sending last line again: %s", last_line);
                lws_write(wsi, (unsigned char *)last_line, strlen(last_line), LWS_WRITE_TEXT);
            }

#else
            if (fgets(line, sizeof(line), s_file))
            {
                printf("Read line: %s", line);
                lws_write(wsi, (unsigned char *)line, strlen(line), LWS_WRITE_TEXT);
            }
            else
            {
                fseek(s_file, 0, SEEK_SET);
            }
#endif
            usleep(100*1000); // Wait for 100ms
            lws_callback_on_writable(wsi);
            break;

        case LWS_CALLBACK_CLOSED:
            if (s_file != NULL)
            {
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

    while (1)
    {
        lws_service(context, 50);
    }

    lws_context_destroy(context);

    return 0;
}
