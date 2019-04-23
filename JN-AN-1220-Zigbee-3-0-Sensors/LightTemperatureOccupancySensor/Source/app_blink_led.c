/*****************************************************************************
 *
 * MODULE:          JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:       app_blink_led.c
 *
 * DESCRIPTION:     ZLO Demo: Blinks an LED a number of pre defined ticks
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
#include "dbg.h"
#include "App_LightTemperatureOccupancySensor.h"
#include "LightingBoard.h"
#include "AppHardwareApi.h"
#include "ZTimer.h"
#include "app_main.h"
#include "pwrm.h"
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_BLINK_LED
    #define TRACE_BLINK_LED   TRUE
#else
    #define TRACE_BLINK_LED   FALSE
#endif
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
#define BLINK_LED   GEN_BOARD_LED_D3_VAL
/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vToggleLED(void);
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE bool_t bDIO1State = FALSE;
PRIVATE uint32 u32BlinkTickTime = 0;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: APP_BlinkLED
 *
 * DESCRIPTION:
 * Toggles LED 2 and restarts the timer.
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vAPP_cbBlinkLED( void *pvParams)
{
    if (0 != u32BlinkTickTime)
    {
        DBG_vPrintf(TRACE_BLINK_LED, "\nAPP Blink LED: Task Started: Blink Time = %dmS", u32BlinkTickTime);
        vToggleLED();
        ZTIMER_eStart(u8TimerBlink, u32BlinkTickTime);
    }
}

/****************************************************************************
 *
 * NAME: vStartBlinkTimer
 *
 * DESCRIPTION:
 * Starts the software timer and blinks it the desired amount of ticks. It also
 * stores the time so the Task can restart it with the defined time
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vStartBlinkTimer(uint32 u32Ticks)
{
    DBG_vPrintf(TRACE_BLINK_LED, "\nAPP Blink LED: Starting Blink Timer value = %d", u32Ticks);
    u32BlinkTickTime = u32Ticks;
    bRGB_LED_SetLevel(LED_MAX_LEVEL,LED_MIN_LEVEL,LED_MIN_LEVEL);
    ZTIMER_eStart(u8TimerBlink, u32Ticks);
}

/****************************************************************************
 *
 * NAME: vStopBlinkTimer
 *
 * DESCRIPTION:
 * Stops the blink timer and turns the LED off
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vStopBlinkTimer(void)
{
    DBG_vPrintf(TRACE_BLINK_LED, "\nAPP Blink LED: Stopping Blink Timer");
    ZTIMER_eStop(u8TimerBlink);
    u32BlinkTickTime = 0;
    bRGB_LED_Off();
}

/****************************************************************************/
/***        Local Function                                                ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vToggleLED
 *
 * DESCRIPTION:
 * Changes the state of LED 1
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vToggleLED(void)
{
    DBG_vPrintf(TRACE_BLINK_LED, "\nAPP Blink LED: Toggle LED to %d", !bDIO1State);
    bDIO1State = !bDIO1State;
    if (bDIO1State)
    {
        bRGB_LED_SetGroupLevel(LED_LEVEL);
        bRGB_LED_SetLevel(LED_MAX_LEVEL,LED_MIN_LEVEL,LED_MIN_LEVEL);
    }
    else
    {
        bRGB_LED_Off();
    }
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
