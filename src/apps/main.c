/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 5.5
 * C/C++ source code generated on  : 10-Nov-2022 01:04:32
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "GenTxSignal.h"
#include "GenTxSignal_emxAPI.h"
#include "GenTxSignal_types.h"
#include "ChannelEstimation_SL_prop_v5_types.h"

#include "rt_nonfinite.h"
#include <stdio.h>

#include "type.h"
#include <sys/time.h>

/* Define Definitions */
#define RXSIGNAL_BUF_SIZE    15360
#define CIR_BUF_SIZE         1440
//#define CONFIG_RESULT_FILE (1)
//#define CONFIG_TIME_CHECK_MS (1)
/* Function Definitions */
/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    InitAlgorithms();
    GenTxSignal_terminate();
    return 0;
}

long long current_timestamp()
{
    struct timeval te;
    gettimeofday(&te, NULL); // get current time

#if defined(CONFIG_TIME_CHECK_MS)
    long long llMilliSeconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    return llMilliSeconds;
#else
    // default us
    long long llMicroSeconds = te.tv_sec*1000000LL + te.tv_usec; // calculate microseconds
    return llMicroSeconds;
#endif
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void InitAlgorithms(void)
{
    static struct0_T params;
    creal_T RxSignal[RXSIGNAL_BUF_SIZE];
    emxArray_creal_T *RxFreqSignal;
    int idx;
    static struct0_T r;
    struct1_T rxparams;
    creal_T CIR[CIR_BUF_SIZE];
    FILE *pfd_GenTxSignal;
    FILE *pfd_InDataFFT;
    FILE *pfd_ChannelEstimationCIR;
    long long llStartTime;
    long long llEndTime;
    long long llDiffTime;

    PrintEnter("Successfully Start");

#if defined(CONFIG_RESULT_FILE)
    PrintDebug("CONFIG_RESULT_FILE is ENABLED");
#else
    PrintDebug("CONFIG_RESULT_FILE is DISABLED");
#endif
    memset(&RxSignal[0], 0, 15360U * sizeof(creal_T));

    /* Create TxSignal */
    llStartTime = current_timestamp();
    GenTxSignal(RxSignal, &params);
#if defined(CONFIG_RESULT_FILE)
    idx = RXSIGNAL_BUF_SIZE;
    pfd_GenTxSignal = fopen("GenTxSignal.txt","w");

    for(int i=0; i<idx; i++)
    {
        fprintf(pfd_GenTxSignal, "%d %e %e\r\n", i, RxSignal[i].re, RxSignal[i].im);
    }

    fclose(pfd_GenTxSignal);
#endif
    llEndTime = current_timestamp();
    llDiffTime = llEndTime-llStartTime;
    PrintExit("Start-End time of GenTxSignal (%lld)us", llDiffTime);

    /* Create RxFreqSignal */
    llStartTime = current_timestamp();
    InDataFFT_emxInitArray_creal_T(&RxFreqSignal, 2);
    InDataFFT(&params, RxSignal, RxFreqSignal);
#if defined(CONFIG_RESULT_FILE)
    idx = RxFreqSignal->size[0] * RxFreqSignal->size[1];

    pfd_InDataFFT = fopen("InDataFFT.txt","w");
    for(int i=0; i<idx; i++)
    {
        fprintf(pfd_InDataFFT, "%d %e %e\r\n", i, RxFreqSignal->data[i].re, RxFreqSignal->data[i].im);
    }
    fclose(pfd_InDataFFT);
#endif
    llEndTime = current_timestamp();
    llDiffTime = llEndTime-llStartTime;
    PrintExit("Start-End time of InDataFFT (%lld)us", llDiffTime);

    /* Channel Estimation */ // 1us
    llStartTime = current_timestamp();
    CE_emxInit_struct1_T(&rxparams);
    ChannelEstimation_SL_prop_v5(&params, RxFreqSignal, &rxparams, CIR);
#if defined(CONFIG_RESULT_FILE)
    idx = CIR_BUF_SIZE;

    pfd_ChannelEstimationCIR = fopen("ChannelEstimationCIR.txt","w");
    for(int i=0; i<idx; i++)
    {
        fprintf(pfd_ChannelEstimationCIR, "%d %e %e\r\n", i, CIR[i].re, CIR[i].im);
    }
    fclose(pfd_ChannelEstimationCIR);
#endif
    llEndTime = current_timestamp();
    llDiffTime = llEndTime-llStartTime;
    PrintExit("Start-End time of ChannelEstimation_SL_prop_v5 (%lld)us", llDiffTime);

    PrintWarn("Destroy functions");
    /* Destroy Channel Estimation */
    CE_emxDestroy_struct1_T(rxparams);

    /* Destroy RxFreqSignal */
    InDataFFT_emxDestroyArray_creal_T(RxFreqSignal);

    PrintExit("Successfully End");

}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
