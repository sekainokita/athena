#ifndef TYPE_H
#define TYPE_H

/* Include Files */
#ifdef __cplusplus
extern "C" {
#endif

#define COLOR_RED     "\x1b[31m"
#define COLOR_GREEN   "\x1b[32m"
#define COLOR_YELLOW  "\x1b[33m"
#define COLOR_BLUE    "\x1b[34m"
#define COLOR_MAGENTA "\x1b[35m"
#define COLOR_CYAN    "\x1b[36m"
#define COLOR_RESET   "\x1b[0m"

#define PrintError(fmt , args...) \
    do { \
        printf(COLOR_RED "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintNotice(fmt , args...) \
    do { \
        printf(COLOR_MAGENTA "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintWarn(fmt , args...) \
    do { \
        printf(COLOR_YELLOW "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintTrace(fmt , args...) \
    do { \
        printf(COLOR_BLUE "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintEnter(fmt , args...) \
    do { \
        printf(COLOR_GREEN "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintExit(fmt , args...) \
    do { \
        printf(COLOR_CYAN "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
    } while (0)

#define PrintDebug(fmt , args...) \
			do { \
				printf(COLOR_RESET "[%s][%d] """fmt"\n" COLOR_RESET , __FUNCTION__, __LINE__, ##args); \
			} while (0)

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for type.h
 *
 * [EOF]
 */
