/*****************************************************************************
 *
 * MODULE:          JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:       app_event_handler.h
 *
 * DESCRIPTION:     ZLO Demo: Handles all the different type of Application events
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
#ifndef APP_EVENT_HANDLER_H_
#define APP_EVENT_HANDLER_H_

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define BUTTON_RELEASED_OFFSET 0x80

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/* Kindly Maintain the order as the button numbers are assigned directly */

typedef enum{
    COMM_BUTTON_PRESSED,
    SW1_PRESSED,
    SW2_PRESSED,
    SW3_PRESSED,
    SW4_PRESSED,
#if (defined APP_NTAG_ICODE) || (defined APP_NTAG_AES)
    FD_PRESSED,
#endif
    // Make sure each switch value matches its counterpart
    COMM_BUTTON_RELEASED = BUTTON_RELEASED_OFFSET,
    SW1_RELEASED,
    SW2_RELEASED,
    SW3_RELEASED,
    SW4_RELEASED,
#if (defined APP_NTAG_ICODE) || (defined APP_NTAG_AES)
    FD_RELEASED,
#endif
    NUMBER_OF_TRANSITION_CODE
}te_TransitionCode;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void vAppHandleAppEvent(APP_tsEvent sButton);
PUBLIC void vEventStopFindAndBind(void);
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#endif /* APP_EVENT_HANDLER_H_ */
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
