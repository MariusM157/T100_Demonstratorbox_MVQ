/** @file pal_swt.c
 *
 *  @brief  software timer
 *
 *  $Rev: 22096 $
 *
 *  $Author: dekestjo $
 *
 *  $Date: 2021-01-28 17:41:45 +0100 (Do, 28. Jan 2021) $
 *   
 *
 *  Copyright (C) 2015 ifm efector gmbh
 *
 *  
 */
/* $Id: pal_swt.c 22096 2021-01-28 16:41:45Z dekestjo $ */

/* local compiler switches ***************************************************/

/* local includes ************************************************************/

#include "pal_swt.h"                 /*include corresponding header file */
#ifdef USE_HR_COUNTER
	#include "pal_eep_man.h"
	#include "pal_eep_map.h"
#endif
/* local defines *************************************************************/
#define PAL_dwMAX_IR_CNT UINT32_MAX
#define PAL_SWT_dwONE_HR  ((uint32_t)72000000) /*one hour = 60*60*1000 *20/100µs = 36000000 2/100µs*/ 
/* local macros **************************************************************/

/* local variables ***********************************************************/

/*flag to prevent reading from g_dwClockCounter or g_bOverflowToggle 
during increment*/
static volatile bool_t gbSkipIncrement = FALSE;
static volatile bool_t gbSkipIncrementWasSkipped = FALSE;

/** @brief This module variable is used to generate a clock.\n
 *  The timing is dependent on the hardware timer 0, where this counter \n
 *  incremented.\n
 *  It needs to be volatile, because it is used inside and outside the timer \n
 *  interrupt service routine. \n
 */
static volatile uint32_t g_dwClockCounter = dwNULL;

/** @brief This module global varable toggles \n
 *  at every overrun of the counter for the software timer \n
 *  gdwClockCounter
 */
static volatile bool_t g_bOverflowToggle = FALSE;

#ifdef USE_HR_COUNTER
	static pal_swt_timer_t gsHrTimer = { PAL_SWT_INIT, wNULL, wNULL, wNULL, FALSE, FALSE};
#endif

/* global variables **********************************************************/
			
int32_t gdiSwtHrCnt = diNULL;    		


/* local function prototypes *************************************************/
/**  @brief initalise (clear) software timer
 *   @param[in] psTimer is a pointer a timer structure
 *   @return     void
 */
static void pal_swt_clear(pal_swt_timer_t* psTimer);

#ifdef USE_HR_COUNTER
/**  @brief init gsHrTimer with PAL_SWT_dwONE_HR in order to allapse after 1hour \n
 *   @param[in]  void
 *   @return     void
 */
static void pal_delay_start_hr(void);
#endif
/* function definitions ******************************************************/

/*timer IR */
void pal_swt_inc_clock_count(void){
	if (gbSkipIncrement == FALSE) {
	   /* interrupt flag is cleared by hardware*/
		if ( g_dwClockCounter == PAL_dwMAX_IR_CNT) {
			if(g_bOverflowToggle == FALSE) {
				g_bOverflowToggle = TRUE;
			} else {
				g_bOverflowToggle = FALSE;
			}		
			g_dwClockCounter = dwNULL;
		} else {
			 g_dwClockCounter++; 
		} 
		gbSkipIncrementWasSkipped = FALSE;
	} else {
		/*read of variabeles g_dwClockCounter or  g_bOverflowToggle
		is in progress => skip increment*/
		gbSkipIncrementWasSkipped = TRUE;
	}
}

void  pal_swt_set_runtime (pal_swt_timer_t* psTimer, uint32_t dwRunTime50Us ){
    pal_swt_clear(psTimer);
    /*set runtime of timer */
    psTimer->dwRunTime = dwRunTime50Us;
}

void  pal_swt_start( pal_swt_timer_t* psTimer ){
	
	uint32_t dwBuffer = dwNULL;
	bool_t bOverflowBuffer = FALSE;
	/*set flag to pal_swt_inc_clock_count() to skip increment
	when called in ISR
	do not disable timer ISR*/
	gbSkipIncrement = TRUE;
	dwBuffer = g_dwClockCounter;	
	bOverflowBuffer = g_bOverflowToggle;
	/*re-enable increment in ISR*/
	gbSkipIncrement = FALSE;
	
	if (gbSkipIncrementWasSkipped == TRUE) {
		/*increment was skipped during read
		call increment "by hand"*/
		pal_swt_inc_clock_count();
	} else {
		/*nothing*/
	}
	
    /*for safety: add check of psTimer for NULLPOINTER */
    psTimer->eStatus = PAL_SWT_RUNNING;
    /* get the start time */
    psTimer->dwStartTime = dwBuffer;
    /* if the starttime/2 + runtime/2 are greater than PAL_wMAX_IR_CNT/2,
     * then the timer will overflow during runtime. 
     * => wait for overflow of the timer AND time > endtime  */
    if ( ((psTimer->dwStartTime)/2 + psTimer->dwRunTime /2) > (PAL_dwMAX_IR_CNT/2)) {

        psTimer->dwEndTime = (psTimer->dwStartTime + psTimer->dwRunTime ) - PAL_dwMAX_IR_CNT;
        psTimer->bOverflow = TRUE; /*overflow will occure during duration of sw timer*/

    } else {
        psTimer->dwEndTime= (psTimer->dwStartTime + psTimer->dwRunTime);
        psTimer->bOverflow = FALSE; /*no overflow will occure during duration of sw timer*/
    }

    psTimer->bOverflowState = bOverflowBuffer;
}


void pal_swt_poll( pal_swt_timer_t* psTimer ){
	/*disable counter ISR
	get counter variable
	re-enable counter ISR*/
	uint32_t dwBuffer = dwNULL;
	bool_t bOverflowBuffer = FALSE;
	
	/*set flag to pal_swt_inc_clock_count() to skip increment
	when called in ISR
	do not disable timer ISR*/
	gbSkipIncrement = TRUE;
	
	dwBuffer = g_dwClockCounter;	
	bOverflowBuffer = g_bOverflowToggle;
	/*re-enable increment in ISR*/
	gbSkipIncrement = FALSE;
	if (gbSkipIncrementWasSkipped == TRUE) {
		/*increment was skipped during read
		call increment "by hand"*/
		pal_swt_inc_clock_count();
	} else {
		/*nothing*/
	}	
	
    if( psTimer->eStatus == PAL_SWT_RUNNING ) {

        /* no overflow */
        if( psTimer->bOverflow == FALSE ) {
			if (psTimer->bOverflowState != bOverflowBuffer ) {
				/*although no Overflow was expected, an overflow did occur
				=> timer is ellapsed*/
				psTimer->eStatus = PAL_SWT_ELLAPSED;
			} else {
				if (dwBuffer >= psTimer->dwEndTime) {
					psTimer->eStatus = PAL_SWT_ELLAPSED;
				} else {
					/* keep running */
					psTimer->eStatus = PAL_SWT_RUNNING;
				}
			}			
						
        /* overflow during runtime */
        } else {
            
            if( psTimer->bOverflowState == bOverflowBuffer ) {
                /* overflow did not happen until now */
                psTimer->eStatus = PAL_SWT_RUNNING;

            } else {

                psTimer->bOverflow = FALSE;

                if (dwBuffer >= psTimer->dwEndTime) {
                    psTimer->eStatus = PAL_SWT_ELLAPSED;
                } else {
                    /* keep running */
                    psTimer->eStatus = PAL_SWT_RUNNING;
                }
            }
        }

    } else {

        if( psTimer->eStatus == PAL_SWT_INIT ) {
            /*timer has not been started yet
             pal_swt_start() must be called before polling*/
        } else {
            /* timer is already stopped */
            psTimer->eStatus = PAL_SWT_ELLAPSED;
        }
    }

}

#ifdef USE_HR_COUNTER
void pal_swt_hr_cnt_init(void){
    /*read hour count from EEPROM and start software timer with 
    one hour*/  
    pal_eep_man_di_rd_safe(PAL_EEP_OP_HR_ADR, &gdiSwtHrCnt);
    pal_delay_start_hr();
}

void pal_swt_hr_cnt(void){
    /*poll hour timer*/
    pal_swt_poll(&gsHrTimer);
    if (gsHrTimer.eStatus == PAL_SWT_RUNNING) {
        /*hour  not ellapsed*/
    } else {
        /*hour ellapsed 
        start next hour
        and increment hour counter */

        pal_delay_start_hr();   
        pal_eep_man_di_rd_safe(PAL_EEP_OP_HR_ADR, &gdiSwtHrCnt);

        if ((gdiSwtHrCnt+1) > (int32_t)PAL_dw_2_MIO) {
            /*maximum reached
            => do not increment hour counter
			only 2 million write cycles in the EEPROM*/            
        } else {
            /*still hours to count*/
			gdiSwtHrCnt++;
			pal_eep_man_di_wrt_safe(PAL_EEP_OP_HR_ADR, gdiSwtHrCnt);
        }        
    }
}  
#endif		


/* local function definitions ************************************************/

static void pal_swt_clear(pal_swt_timer_t* psTimer){
    if (psTimer != vNULL) {
        psTimer->eStatus = PAL_SWT_INIT;
        psTimer->dwStartTime = dwNULL;
        psTimer->dwRunTime = dwNULL;
        psTimer->dwEndTime = dwNULL;
        psTimer->bOverflow = FALSE;
    } else {
        /*psTimer is a nullpointer. do not manipulate *psTimer */
    }
}

#ifdef USE_HR_COUNTER
static void pal_delay_start_hr(void){
    pal_swt_set_runtime (&gsHrTimer, PAL_SWT_dwONE_HR);
    pal_swt_start( &gsHrTimer);
}
#endif

/*************************** End of file *************************************/
