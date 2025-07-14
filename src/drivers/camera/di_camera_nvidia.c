/******************************************************************************
*
* Copyright (C) 2023 - 2028 KETI, All rights reserved.
*                           (Korea Electronics Technology Institute)
*
******************************************************************************/
/**
* @file di_camera_nvidia.c
* @note DI Camera NVIDIA Implementation
******************************************************************************/

#include "type.h"
#include "di_camera_nvidia.h"
#include "di_error.h"

/*
 * Initialize NVIDIA camera
 */
int32_t DI_CAMERA_NVIDIA_Init(DI_CAMERA_NVIDIA_T *pstCameraDev)
{
    int32_t nRet = DI_ERROR_OK;
    
    if (pstCameraDev == NULL)
    {
        nRet = DI_ERROR_NULL_POINTER;
        goto EXIT;
    }
    
    /* Initialize device structure */
    memset(pstCameraDev, 0, sizeof(DI_CAMERA_NVIDIA_T));
    pstCameraDev->eStatus = DI_CAMERA_NVIDIA_STATUS_STOPPED;
    pstCameraDev->bIsInitialized = TRUE;
    
EXIT:
    return nRet;
}

/*
 * Deinitialize NVIDIA camera
 */
int32_t DI_CAMERA_NVIDIA_DeInit(DI_CAMERA_NVIDIA_T *pstCameraDev)
{
    int32_t nRet = DI_ERROR_OK;
    
    if (pstCameraDev == NULL)
    {
        nRet = DI_ERROR_NULL_POINTER;
        goto EXIT;
    }
    
    /* Reset device structure */
    memset(pstCameraDev, 0, sizeof(DI_CAMERA_NVIDIA_T));
    
EXIT:
    return nRet;
}

/*
 * Open NVIDIA camera
 */
int32_t DI_CAMERA_NVIDIA_Open(DI_CAMERA_NVIDIA_T *pstCameraDev)
{
    int32_t nRet = DI_ERROR_OK;
    
    if (pstCameraDev == NULL)
    {
        nRet = DI_ERROR_NULL_POINTER;
        goto EXIT;
    }
    
    /* TODO: Implement actual camera open */
    pstCameraDev->eStatus = DI_CAMERA_NVIDIA_STATUS_STOPPED;
    
EXIT:
    return nRet;
}

/*
 * Close NVIDIA camera
 */
int32_t DI_CAMERA_NVIDIA_Close(DI_CAMERA_NVIDIA_T *pstCameraDev)
{
    int32_t nRet = DI_ERROR_OK;
    
    if (pstCameraDev == NULL)
    {
        nRet = DI_ERROR_NULL_POINTER;
        goto EXIT;
    }
    
    /* TODO: Implement actual camera close */
    pstCameraDev->eStatus = DI_CAMERA_NVIDIA_STATUS_STOPPED;
    
EXIT:
    return nRet;
}

/*
 * Start NVIDIA camera
 */
int32_t DI_CAMERA_NVIDIA_Start(DI_CAMERA_NVIDIA_T *pstCameraDev)
{
    int32_t nRet = DI_ERROR_OK;
    
    if (pstCameraDev == NULL)
    {
        nRet = DI_ERROR_NULL_POINTER;
        goto EXIT;
    }
    
    /* TODO: Implement actual camera start */
    pstCameraDev->eStatus = DI_CAMERA_NVIDIA_STATUS_RUNNING;
    
EXIT:
    return nRet;
}

/*
 * Stop NVIDIA camera
 */
int32_t DI_CAMERA_NVIDIA_Stop(DI_CAMERA_NVIDIA_T *pstCameraDev)
{
    int32_t nRet = DI_ERROR_OK;
    
    if (pstCameraDev == NULL)
    {
        nRet = DI_ERROR_NULL_POINTER;
        goto EXIT;
    }
    
    /* TODO: Implement actual camera stop */
    pstCameraDev->eStatus = DI_CAMERA_NVIDIA_STATUS_STOPPED;
    
EXIT:
    return nRet;
}

/*
 * Get frame from NVIDIA camera
 */
int32_t DI_CAMERA_NVIDIA_GetFrame(DI_CAMERA_NVIDIA_T *pstCameraDev, 
                                  DI_CAMERA_NVIDIA_FRAME_T *pstFrame)
{
    int32_t nRet = DI_ERROR_OK;
    
    if (pstCameraDev == NULL || pstFrame == NULL)
    {
        nRet = DI_ERROR_NULL_POINTER;
        goto EXIT;
    }
    
    /* TODO: Implement actual frame capture */
    /* For now, return no data available */
    nRet = DI_ERROR_RESOURCE_NOT_AVAILABLE;
    
EXIT:
    return nRet;
}

/*
 * Release frame from NVIDIA camera
 */
int32_t DI_CAMERA_NVIDIA_ReleaseFrame(DI_CAMERA_NVIDIA_T *pstCameraDev, 
                                      DI_CAMERA_NVIDIA_FRAME_T *pstFrame)
{
    int32_t nRet = DI_ERROR_OK;
    
    if (pstCameraDev == NULL || pstFrame == NULL)
    {
        nRet = DI_ERROR_NULL_POINTER;
        goto EXIT;
    }
    
    /* TODO: Implement actual frame release */
    
EXIT:
    return nRet;
}