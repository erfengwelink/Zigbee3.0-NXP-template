/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_events.h
 *
 * DESCRIPTION:        Application Events
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

#ifndef APP_GENERIC_EVENTS_H_
#define APP_GENERIC_EVENTS_H_

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef enum
{
    APP_E_EVENT_NONE = 0,
    APP_E_EVENT_BUTTON_UP,
    APP_E_EVENT_BUTTON_DOWN,
    APP_E_EVENT_PERIODIC_REPORT,
    APP_E_EVENT_SEND_REPORT,
    APP_E_EVENT_WAKE_TIMER,
    APP_E_EVENT_KEEPALIVE_START,
    APP_E_EVENT_KEEPALIVE_STOP,
    APP_E_EVENT_FIND_BIND_START,
    APP_E_EVENT_FIND_BIND_STOP,
    APP_E_EVENT_MAX,
} APP_teEventType;

typedef struct
{
    uint8 u8Button;
    uint32 u32DIOState;
} APP_tsEventButton;

typedef struct
{
    APP_teEventType eType;
    union
    {
        APP_tsEventButton                   sButton;
    }uEvent;
} APP_tsEvent;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#endif /*APP_GENERIC_EVENTS_H_*/
