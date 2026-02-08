#ifndef ERROR_H
#define ERROR_H


/* Define error types */

/* clocksetup error type 
* @param clocksetup_success: clocks setup successfully
* @param clocksetup_fail: clock setup has failed
*/
typedef enum
{
    clocksetup_success = 0,
    clocksetup_fail = 1,

}clocksetup_err_t;


/* Define error types */

/* devicesetup error type 
* @param devicesetup_success: device setup successfully
* @param devicesetup_fail: device setup has failed
*/
typedef enum
{
    devicesetup_success = 0,
    devicesetup_fail = 1,

}devicesetup_err_t;
#endif