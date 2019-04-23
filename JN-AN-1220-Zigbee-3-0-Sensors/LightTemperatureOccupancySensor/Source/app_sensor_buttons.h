/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_sensor_buttons.h
 *
 * DESCRIPTION:        DK4 (DR1175) Button Press detection (Implementation)
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
 * Copyright NXP B.V. 2017. All rights reserved
 *
 ***************************************************************************/

#ifndef APP_BUTTONS_H
#define APP_BUTTONS_H

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

#if (defined BUTTON_MAP_DR1175)
    #if (defined APP_NTAG_ICODE) || (defined APP_NTAG_AES)

        typedef enum {
            APP_E_BUTTONS_BUTTON_1 = 0,
            APP_E_BUTTONS_NFC_FD
        } APP_teButtons;

        #define APP_BUTTONS_NUM                         (2UL)
        #if (JENNIC_CHIP_FAMILY == JN517x)
            #define APP_BUTTONS_BUTTON_1                (4)
            #define APP_BUTTONS_NFC_FD                  (17)
        #elif (JENNIC_CHIP_FAMILY == JN516x)
            #define APP_BUTTONS_BUTTON_1                (8)
            #define APP_BUTTONS_NFC_FD                  (0)
        #endif
        #define APP_BUTTONS_DIO_MASK                    ((1 << APP_BUTTONS_BUTTON_1) | (1 << APP_BUTTONS_NFC_FD))
    #else
        typedef enum {
            APP_E_BUTTONS_BUTTON_1
        } APP_teButtons;

        #define APP_BUTTONS_NUM                         (1UL)
        #if (JENNIC_CHIP_FAMILY == JN517x)
            #define APP_BUTTONS_BUTTON_1                (4)
        #elif (JENNIC_CHIP_FAMILY == JN516x)
            #define APP_BUTTONS_BUTTON_1                (8)
        #endif
        #define APP_BUTTONS_DIO_MASK                    (1 << APP_BUTTONS_BUTTON_1)
    #endif
#endif

typedef enum {
    E_INTERRUPT_UNKNOWN,
    E_INTERRUPT_BUTTON,
    E_INTERRUPT_WAKE_TIMER_EXPIRY
} teInterruptType;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void APP_bButtonInitialise(void);
PUBLIC void vActionOnButtonActivationAfterDeepSleep(void);
PUBLIC void vSaveDioStateBeforeDeepSleep(void);

PUBLIC void APP_cbTimerButtonScan(void *pvParam);

#if (JENNIC_CHIP_FAMILY == JN516x)
PUBLIC void vISR_SystemController(void);
#endif
#if (JENNIC_CHIP_FAMILY == JN517x)
PUBLIC void vISR_SystemController(uint32 u32DeviceId, uint32 u32ItemBitMap);
#endif

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#endif /*APP_BUTTONS_H*/
