/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_sleep_handler.c
 *
 * DESCRIPTION:        ZLO Demo : Manages sleep configuration.
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5179].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2016. All rights reserved
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <string.h>
#include "dbg.h"
#include "ZTimer.h"
#include "app_main.h"
#include "pwrm.h"
#include "AppHardwareApi.h"
#include "pdum_gen.h"
#include "PDM.h"
#include "pdum_gen.h"
#include "app_common.h"
#include "PDM_IDs.h"
#include "zcl_options.h"
#include "app_zbp_utilities.h"
#include "zcl_common.h"
#include "app_sleep_handler.h"
#include "app_zcl_tick_handler.h"
#include "app_zcl_sensor_task.h"
#include "app_sensor_state_machine.h"
#include "App_LightTemperatureOccupancySensor.h"
#include "GenericBoard.h"
#include "app_zlo_sensor_node.h"
#include "app_nwk_event_handler.h"
#include "app_events.h"
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef DEBUG_SLEEP_HANDLER
    #define TRACE_SLEEP_HANDLER   FALSE
#else
    #define TRACE_SLEEP_HANDLER   TRUE
#endif

#if ZLO_MAX_REPORT_INTERVAL == 0
    #define MAXIMUM_TIME_TO_SLEEP APP_OCCUPANCY_SENSOR_OCCUPIED_TO_UNOCCUPIED_DELAY + 1
#else
    #define MAXIMUM_TIME_TO_SLEEP ZLO_MAX_REPORT_INTERVAL
#endif
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE uint8 u8NumberOfTimersTaskTimers(void);
PRIVATE void vScheduleSleep(bool_t bDeepSleep);
PRIVATE void vWakeCallBack(void);
PRIVATE void vStopNonSleepPreventingTimers(void);
PRIVATE void vStartNonSleepPreventingTimers(void);
PRIVATE uint8 u8NumberOfNonSleepPreventingTimers(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE pwrm_tsWakeTimerEvent    sWake;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC uint32 u32SleepTime()
{
	return (MAXIMUM_TIME_TO_SLEEP - u32GetNumberOfZCLTicksSinceReport());
}

/****************************************************************************
 *
 * NAME:        vAttemptToSleep
 *
 * DESCRIPTION: Checks to see if any software timers are running that may
 * prevent us from going to sleep. If there is none, if wake timer 0 is
 * running, schedule none deep sleep, if there is schedule deep sleep
 * which is checked if its enabled in vScheduleSleep.
 *
 ****************************************************************************/
PUBLIC void vAttemptToSleep(void)
{
    DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: Activity Count = %d", PWRM_u16GetActivityCount());
    DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: Task Timers = %d", u8NumberOfTimersTaskTimers());

    /* Only enter here if the activity count is equal to the number of non sleep preventing timers (in other words, the activity count
     * will become zero when we stop them) */
    if ((PWRM_u16GetActivityCount() == u8NumberOfNonSleepPreventingTimers()) &&
        (0 == u8NumberOfTimersTaskTimers()))
    {
        /* Stop any background timers that are non sleep preventing*/
        vStopNonSleepPreventingTimers();

        /* Check if Wake timer 0 is running.*/
        if (u8AHI_WakeTimerStatus() & E_AHI_WAKE_TIMER_MASK_0)
        {
            vScheduleSleep(FALSE);
        }
        else
        {
            vScheduleSleep(TRUE);
        }
    }
}

/****************************************************************************/
/***        Local Function                                                ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME:        u8NumberOfTimersTaskTimers
 *
 * DESCRIPTION: Checks to see if any timers are running that shouldn't be
 * interrupted by sleeping.
 *
 ****************************************************************************/
PRIVATE uint8 u8NumberOfTimersTaskTimers(void)
{
    uint8 u8NumberOfRunningTimers = 0;

    if (ZTIMER_eGetState(u8TimerButtonScan) == E_ZTIMER_STATE_RUNNING)
    {
        DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: APP_ButtonsScanTimer");
        u8NumberOfRunningTimers++;
    }

    if (ZTIMER_eGetState(u8TimerPoll) == E_ZTIMER_STATE_RUNNING)
    {
        DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: u8TimerPoll");
        u8NumberOfRunningTimers++;
    }

    if (ZTIMER_eGetState(u8TimerBlink) == E_ZTIMER_STATE_RUNNING)
    {
        DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: u8TimerBlink");
        u8NumberOfRunningTimers++;
    }

    if (ZTIMER_eGetState(u8TimerPowerOnCount) == E_ZTIMER_STATE_RUNNING)
    {
        DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: u8TimerPowerOnCount");
        u8NumberOfRunningTimers++;
    }

#if (defined APP_NTAG_ICODE) || (defined APP_NTAG_AES)
    if (ZTIMER_eGetState(u8TimerNtag) == E_ZTIMER_STATE_RUNNING)
    {
        DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: APP_TimerNtag");
        u8NumberOfRunningTimers++;
    }
#endif
    return u8NumberOfRunningTimers;
}

/****************************************************************************
 *
 * NAME:        vScheduleSleep
 *
 * DESCRIPTION: If we have deep sleep enabled and we attempting to deep sleep
 * then re-initialise the power manager for deep sleep
 *
 ****************************************************************************/
PRIVATE void vScheduleSleep(bool_t bDeepSleep)
{
	DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler ");

#ifdef DEEP_SLEEP_ENABLE
    if (bDeepSleep)
    {
        PWRM_vInit(E_AHI_SLEEP_DEEP);
        DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: Deep Sleep");
    }
    else
    {
    	PWRM_eScheduleActivity(&sWake, 29700, vWakeCallBack);
        DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: Osc on");
    }
#else
    /* Needs to find the exact value to be used in order to get exact one sec as otherwise
     * the reports are getting delayed with 32768 as value
     */
    PWRM_eScheduleActivity(&sWake, 29700, vWakeCallBack);
    DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: Osc on");
#endif
}

/****************************************************************************
 *
 * NAME:        vStopNonSleepPreventingTimers
 *
 * DESCRIPTION: The timers in this function should not stop us from sleep.
 * Stop the timers to reduce the activity count which will prevent sleep.
 *
 ****************************************************************************/
PRIVATE void vStopNonSleepPreventingTimers()
{
#ifndef DEEP_SLEEP_ENABLE
    if (ZTIMER_eGetState(u8TimerTick) != E_ZTIMER_STATE_STOPPED)
        ZTIMER_eStop(u8TimerTick);

    if (ZTIMER_eGetState(u8TimerLightTempSensorSample) != E_ZTIMER_STATE_STOPPED)
    	ZTIMER_eStop(u8TimerLightTempSensorSample);
#endif

}

/****************************************************************************
 *
 * NAME:        vStartNonSleepPreventingTimers
 *
 * DESCRIPTION: Start the timers that wont stop us in vAttemptToSleep
 *
 ****************************************************************************/
PRIVATE void vStartNonSleepPreventingTimers(void)
{
#ifndef DEEP_SLEEP_ENABLE
    if (ZTIMER_eGetState(u8TimerTick) != E_ZTIMER_STATE_RUNNING)
        ZTIMER_eStart(u8TimerTick, ZCL_TICK_TIME);
#endif
}
/****************************************************************************
 *
 * NAME:        u8NumberOfNonSleepPreventingTimers
 *
 * DESCRIPTION: Returns the number of timers that are running that we are
 * prepared to stop before going to sleep.
 *
 ****************************************************************************/
PRIVATE uint8 u8NumberOfNonSleepPreventingTimers(void)
{
    uint8 u8NumberOfRunningTimers = 0;

    if (ZTIMER_eGetState(u8TimerTick) == E_ZTIMER_STATE_RUNNING)
    {
        DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: u8TimerTick");
        u8NumberOfRunningTimers++;
    }
    if (ZTIMER_eGetState(u8TimerLightTempSensorSample) == E_ZTIMER_STATE_RUNNING)
    {
    	DBG_vPrintf(TRACE_SLEEP_HANDLER, "\r\nSLEEP: u8TimerLightTempSensorSample");
    	u8NumberOfRunningTimers++;
    }
    return u8NumberOfRunningTimers;
}

/****************************************************************************
 *
 * NAME: vWakeCallBack
 *
 * DESCRIPTION:
 * Wake up call back called upon wake up by the schedule activity event.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vWakeCallBack(void)
{
    DBG_vPrintf(TRACE_SLEEP_HANDLER, "\nAPP Sleep Handler: vWakeCallBack\n");

#ifdef CLD_OTA
    
    {
    	vStopPollTimerTask();
     }
#endif

    /*Start the APP_TickTimer to continue the ZCL tasks */
    vStartNonSleepPreventingTimers();
}


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
