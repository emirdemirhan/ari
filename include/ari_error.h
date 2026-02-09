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

/* devicesetup error type 
* @param devicesetup_success: device setup successfully
* @param devicesetup_fail: device setup has failed
*/
typedef enum
{
    devicesetup_success = 0,
    devicesetup_fail = 1,

}devicesetup_err_t;

/* nrf24 error type 
* @param nrfsetup_success: device setup successfully
* @param nrfsetup_fail: device setup has failed
*/
typedef enum
{
    nrfsetup_success = 0,
    nrfsetup_fail = 1,

}nrf_err_t;



#endif