#ifndef _DI_DISPLAY_H_
#define _DI_DISPLAY_H_

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
* @file di_display.h
*
* @note
*
* DI Display Header
*
******************************************************************************/

/***************************** Include ***************************************/
#include "type.h"
#include "di.h"
#include "di_camera.h"

/***************************** Definition ************************************/
#define DI_DISPLAY_TASK_MSG_KEY              (0x240533)
#define DI_DISPLAY_MAX_DEVICES               (4)
#define DI_DISPLAY_DEFAULT_WIDTH             (1920)
#define DI_DISPLAY_DEFAULT_HEIGHT            (1080)
#define DI_DISPLAY_OVERLAY_TEXT_MAX          (256)

/***************************** Enum and Structure ****************************/
/**
* @details DI Display Status
* @param DI_DISPLAY_STATUS_E
*/
typedef enum {
    DI_DISPLAY_STATUS_DEINITIALIZED        = 0,
    DI_DISPLAY_STATUS_INITIALIZED          = 1,
    DI_DISPLAY_STATUS_CLOSED               = 2,
    DI_DISPLAY_STATUS_OPENED               = 3,
    DI_DISPLAY_STATUS_STARTED              = 4,
    DI_DISPLAY_STATUS_STOPPED              = 5,
    DI_DISPLAY_STATUS_MAX                  = 255,
} DI_DISPLAY_STATUS_E;

/**
* @details DI_DISPLAY_EVENT_E
* @param DI_DISPLAY_EVENT_START
* @param DI_DISPLAY_EVENT_STOP
*/
typedef enum {
    eDI_DISPLAY_EVENT_UNKNOWN              = 0x0000,
    eDI_DISPLAY_EVENT_START                = 0x0001,
    eDI_DISPLAY_EVENT_STOP                 = 0x0002,
    eDI_DISPLAY_EVENT_RENDER               = 0x0003,
    eDI_DISPLAY_EVENT_OVERLAY              = 0x0004,
    eDI_DISPLAY_EVENT_UNDEFINED_1,
    eDI_DISPLAY_EVENT_UNDEFINED_2,
    eDI_DISPLAY_EVENT_MAX                  = 0xFFFF
} DI_DISPLAY_EVENT_E;

/**
* @details DI_DISPLAY_OVERLAY_TYPE_E
* @param DI_DISPLAY_OVERLAY_TEXT
* @param DI_DISPLAY_OVERLAY_RECT
*/
typedef enum {
    eDI_DISPLAY_OVERLAY_UNKNOWN            = 0,
    eDI_DISPLAY_OVERLAY_TEXT               = 1,
    eDI_DISPLAY_OVERLAY_RECT               = 2,
    eDI_DISPLAY_OVERLAY_BITMAP             = 3,
    eDI_DISPLAY_OVERLAY_MAX                = 255,
} DI_DISPLAY_OVERLAY_TYPE_E;

/**
* @details DI_DISPLAY_OVERLAY_T
* @param eType
* @param unPosX
* @param unPosY
* @param unWidth
* @param unHeight
* @param achText
*/
typedef struct DI_DISPLAY_OVERLAY_t {
    DI_DISPLAY_OVERLAY_TYPE_E   eType;
    uint32_t                    unPosX;
    uint32_t                    unPosY;
    uint32_t                    unWidth;
    uint32_t                    unHeight;
    uint32_t                    unColor;
    uint32_t                    unFontSize;
    uint8_t                     achText[DI_DISPLAY_OVERLAY_TEXT_MAX];
} DI_DISPLAY_OVERLAY_T;

/**
* @details DI_DISPLAY_CONFIG_T
* @param unDeviceId
* @param unWidth
* @param unHeight
* @param bFullScreen
* @param bHardwareAccel
*/
typedef struct DI_DISPLAY_CONFIG_t {
    uint32_t                    unDeviceId;
    uint32_t                    unWidth;
    uint32_t                    unHeight;
    bool                        bFullScreen;
    bool                        bHardwareAccel;
    bool                        bOverlayEnabled;
} DI_DISPLAY_CONFIG_T;

/**
* @details DI_DISPLAY_EVENT_MSG_T
* @param eEventType
* @param unDeviceId
*/
typedef struct DI_DISPLAY_EVENT_MSG_t {
    DI_DISPLAY_EVENT_E          eEventType;
    uint32_t                    unDeviceId;
} DI_DISPLAY_EVENT_MSG_T;

/**
* @details DI_DISPLAY_T
* @param bLogLevel
* @param bDisplayNotAvailable
* @param eDiDisplayStatus
* @param stDiDisplayConfig
* @param stDiDisplayOverlay
*/
typedef struct DI_DISPLAY_t {
    bool                        bLogLevel;
    bool                        bDisplayNotAvailable;
    DI_DISPLAY_STATUS_E         eDiDisplayStatus;
    DI_DISPLAY_CONFIG_T         stDiDisplayConfig;
    DI_DISPLAY_OVERLAY_T        stDiDisplayOverlay;
    uint32_t                    unReserved;
} DI_DISPLAY_T;

/***************************** Function Prototype *****************************/

int32_t DI_DISPLAY_Init(DI_DISPLAY_T *pstDiDisplay);
int32_t DI_DISPLAY_DeInit(DI_DISPLAY_T *pstDiDisplay);

int32_t DI_DISPLAY_SetLog(DI_DISPLAY_T *pstDiDisplay);
int32_t DI_DISPLAY_SetConfig(DI_DISPLAY_T *pstDiDisplay, DI_DISPLAY_CONFIG_T *pstConfig);
int32_t DI_DISPLAY_GetConfig(DI_DISPLAY_T *pstDiDisplay, DI_DISPLAY_CONFIG_T *pstConfig);
int32_t DI_DISPLAY_SetNa(DI_DISPLAY_T *pstDiDisplay, bool bNotAvailable);

int32_t DI_DISPLAY_Open(DI_DISPLAY_T *pstDiDisplay);
int32_t DI_DISPLAY_Close(DI_DISPLAY_T *pstDiDisplay);
int32_t DI_DISPLAY_Start(DI_DISPLAY_T *pstDiDisplay);
int32_t DI_DISPLAY_Stop(DI_DISPLAY_T *pstDiDisplay);

int32_t DI_DISPLAY_RenderFrame(DI_DISPLAY_T *pstDiDisplay, DI_CAMERA_FRAME_T *pstFrame);
int32_t DI_DISPLAY_SetOverlay(DI_DISPLAY_T *pstDiDisplay, DI_DISPLAY_OVERLAY_T *pstOverlay);
int32_t DI_DISPLAY_ClearOverlay(DI_DISPLAY_T *pstDiDisplay);

void DI_DISPLAY_Status(DI_DISPLAY_T *pstDiDisplay);

#endif /* _DI_DISPLAY_H_ */