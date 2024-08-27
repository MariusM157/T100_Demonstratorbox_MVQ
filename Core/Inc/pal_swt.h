/** @file pal_swt.h
 *
 *  @brief  software timer 
    The resolution of the software timer is 100 µs. this is the intervall between calls of 
    pal_swt_inc_clock_count().

    The timer counter variable is uint32_t (4.294.967.295 * 100µs = 4.294.96 s until overrun)

 *
 *  $Rev: 14503 $
 *
 *  $Author: dekestjo $
 *
 *  $Date: 2018-09-11 13:08:42 +0200 (Di, 11. Sep 2018) $
 * 
 *  Copyright (C) 2015 ifm efector gmbh
 * 
 */
/* $Id: pal_swt.h 14503 2018-09-11 11:08:42Z dekestjo $ */

/* include guard *************************************************************/
#ifndef PAL_SWT_H
#define PAL_SWT_H

/* compiler switches *********************************************************/

/* includes ******************************************************************/

#include "pal_config.h"
/* defines *******************************************************************/
#define PAL_SWT_COUNT_PER_MS  ((uint16_t)20)  /*the counts increment every 50µs => 1 ms = 20 counts*/
/* macros ********************************************************************/

/* types *********************************************************************/
/**  @brief status of SW timer*/
typedef enum {
    PAL_SWT_INIT = 2 ,       /**< initialisation state */
    PAL_SWT_RUNNING = 1,     /**<timer is running */
    PAL_SWT_ELLAPSED = 0,     /**<timer has ellapsed */
} pal_swt_status_t;

/**  @brief Timer structure.
 *   To use a timer, define a variable of this type. Use a pointer to it to
 *   reference the timer, when calling any software timer function.
 */
typedef struct {
    pal_swt_status_t eStatus;   /**< current status of the timer*/
    uint32_t dwStartTime;        /**< timer value, when the timer was started */
    uint32_t dwRunTime;          /**< this is the value, the timer should run */
    uint32_t dwEndTime;          /**< this is the value, the timer end */
    bool_t bOverflow;           /**< flag to indicate that the timer will overflow during run*/
    bool_t bOverflowState;      /**< flag to indicate if the actual overflow state of the timer at start*/
} pal_swt_timer_t;


/* variables *****************************************************************/
			
extern int32_t gdiSwtHrCnt;   		
     
/* function prototypes *******************************************************/

/**  @brief  increment clock counter \n
 *   call this function periodiacally in timer ISR\n
 *   @param[in]   void
 *   @return      void
 */
extern void pal_swt_inc_clock_count(void);

/**  @brief Start timer.
 *   To use a timer, define a variable of the type pal_softwaretimer_t. Use a \n
 *   pointer to it when calling this function. The software timer will \n
 *   be initialized with the given parameters. It's status will be \n
 *   PAL_iTIMER_RUNNING. \n
 *   @param[in] pal_softwaretimer_t* psTimer: is a pointer a timer structure \n
 *              uint32_t dwTime: is the desired running time of the timer. \n
 *              The unit of this value is dependend on the clock timebase. \n
 *              To avoid this dependency, this parameter should be normalized by \n
 *              using one of the macros PAL_USEC() or PAL_MSEC(). If, for example, \n
 *              the timer sould run 200us, the call would look like \n
 *               <code> pal_timer_start( &myTimer, PAL_USEC(200) ); </code>
 *   @return void
 */
extern void pal_swt_start( pal_swt_timer_t* psTimer );

/**  @brief set thr runtime of the timer &psTimer \n
 *   set the runtime to dwDurationUs us
 *   @param[in] psTimer is a pointer a timer structure
 *   @return   void
 */

extern void  pal_swt_set_runtime (pal_swt_timer_t* psTimer, uint32_t dwRunTime50Us );

/**  @brief Check if the timer is running.
 *   This function has to be called, to check, if the timer is running.
 *   @param[in] psTimer is a pointer a timer structure
 *   @return     void
 */
extern void pal_swt_poll( pal_swt_timer_t* psTimer );

#ifdef USE_HR_COUNTER
/**  @brief initialise hour counter
 *   @param[in]  void
 *   @return     void
 */
extern void pal_swt_hr_cnt_init(void);

/**  @brief check if one full hour has ellapsed since last\n
 *   increment of hour counter and increment hour counter \n
 *   safe read and safe write to EEPROM \n
 *   @param[in]  void
 *   @return     void
 */
extern void pal_swt_hr_cnt(void);
#endif
/* include guard *************************************************************/

#endif /*PAL_SWT_H*/

/*************************** End of file *************************************/
