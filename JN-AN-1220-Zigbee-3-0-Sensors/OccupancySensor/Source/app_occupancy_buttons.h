/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_occupancy_buttons.h
 *
 * DESCRIPTION:        DK4 (DR1175/DR1199) Button Press detection (Implementation)
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

#ifndef APP_OCCUPANCY_BUTTONS_H_
#define APP_OCCUPANCY_BUTTONS_H_

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

#if (defined BUTTON_MAP_DR1199)
    #if (defined APP_NTAG_ICODE) || (defined APP_NTAG_AES)
        typedef enum {
            APP_E_BUTTONS_BUTTON_1 = 0,
            APP_E_BUTTONS_BUTTON_SW1,
            APP_E_BUTTONS_BUTTON_SW2,
            APP_E_BUTTONS_BUTTON_SW3,
            APP_E_BUTTONS_BUTTON_SW4,
            APP_E_BUTTONS_NFC_FD
        } APP_teButtons;

        #if (JENNIC_CHIP_FAMILY == JN516x)
            #define APP_BUTTONS_NUM             (6UL)
            #define APP_BUTTONS_BUTTON_1        (8)
            #define APP_BUTTONS_BUTTON_SW1      (11)
            #define APP_BUTTONS_BUTTON_SW2      (12)
            #define APP_BUTTONS_BUTTON_SW3      (17)
            #define APP_BUTTONS_BUTTON_SW4      (1)
            #define APP_BUTTONS_NFC_FD          (0)
        #endif

        #if (JENNIC_CHIP_FAMILY == JN517x)
            #define APP_BUTTONS_NUM             (6UL)
            #define APP_BUTTONS_BUTTON_1        (4)
            #define APP_BUTTONS_BUTTON_SW1      (12)
            #define APP_BUTTONS_BUTTON_SW2      (13)
            #define APP_BUTTONS_BUTTON_SW3      (18)
            #define APP_BUTTONS_BUTTON_SW4      (5)
            #define APP_BUTTONS_NFC_FD          (17)
        #endif

        #define APP_BUTTONS_DIO_MASK                    ((1 << APP_BUTTONS_BUTTON_1)|(1 << APP_BUTTONS_BUTTON_SW4) | (1 << APP_BUTTONS_BUTTON_SW3) | (1 << APP_BUTTONS_BUTTON_SW2) | (1 << APP_BUTTONS_BUTTON_SW1) | (1 << APP_BUTTONS_NFC_FD))
        #define APP_BUTTONS_DIO_MASK_FOR_DEEP_SLEEP     ((1 << APP_BUTTONS_BUTTON_SW4) | (1 << APP_BUTTONS_BUTTON_SW3) | (1 << APP_BUTTONS_BUTTON_SW2) | (1 << APP_BUTTONS_BUTTON_SW1) | (1 << APP_BUTTONS_NFC_FD))
    #else
        // No ntag
        typedef enum {
            APP_E_BUTTONS_BUTTON_1 = 0,
            APP_E_BUTTONS_BUTTON_SW1,
            APP_E_BUTTONS_BUTTON_SW2,
            APP_E_BUTTONS_BUTTON_SW3,
            APP_E_BUTTONS_BUTTON_SW4
        } APP_teButtons;

        #if (JENNIC_CHIP_FAMILY == JN516x)
            #define APP_BUTTONS_NUM             (5UL)
            #define APP_BUTTONS_BUTTON_1        (8)
            #define APP_BUTTONS_BUTTON_SW1      (11)
            #define APP_BUTTONS_BUTTON_SW2      (12)
            #define APP_BUTTONS_BUTTON_SW3      (17)
            #define APP_BUTTONS_BUTTON_SW4      (1)
        #endif

        #if (JENNIC_CHIP_FAMILY == JN517x)
            #define APP_BUTTONS_NUM             (5UL)
            #define APP_BUTTONS_BUTTON_1        (4)
            #define APP_BUTTONS_BUTTON_SW1      (12)
            #define APP_BUTTONS_BUTTON_SW2      (13)
            #define APP_BUTTONS_BUTTON_SW3      (18)
            #define APP_BUTTONS_BUTTON_SW4      (5)
        #endif

        #define APP_BUTTONS_DIO_MASK        ((1 << APP_BUTTONS_BUTTON_1)|(1 << APP_BUTTONS_BUTTON_SW4) | (1 << APP_BUTTONS_BUTTON_SW3) | (1 << APP_BUTTONS_BUTTON_SW2) | (1 << APP_BUTTONS_BUTTON_SW1))
        #define APP_BUTTONS_DIO_MASK_FOR_DEEP_SLEEP        ((1 << APP_BUTTONS_BUTTON_SW4) | (1 << APP_BUTTONS_BUTTON_SW3) | (1 << APP_BUTTONS_BUTTON_SW2) | (1 << APP_BUTTONS_BUTTON_SW1))
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
PUBLIC bool_t bGetPreSleepOccupancyState(void);
#if (JENNIC_CHIP_FAMILY == JN516x)
PUBLIC void vISR_SystemController(void);
#endif
#if (JENNIC_CHIP_FAMILY == JN517x)
PUBLIC void vISR_SystemController(uint32 u32DeviceId, uint32 u32ItemBitMap);
#endif
PUBLIC bool_t bButtonDebounceInProgress(void);

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#endif /*APP_OCCUPANCY_BUTTONS_H_*/
