/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          App_LightSensor.c
 *
 * DESCRIPTION:        ZLO Demo Light Sensor -Implementation
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
#include "zps_gen.h"
#include "App_LightSensor.h"
#include "dbg.h"
#include <string.h>
#include "LightingBoard.h"
#include "TSL2550.h"
#include "app_common.h"
#include "app_main.h"
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_LIGHT_SENSOR
    #define TRACE_LIGHT_SENSOR TRUE
#else
    #define TRACE_LIGHT_SENSOR FALSE
#endif

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
const uint8 u8MyEndpoint = LIGHTSENSOR_SENSOR_ENDPOINT;
tsZLO_LightSensorDevice sSensor;

/* define the default reports */
tsReports asDefaultReports[ZCL_NUMBER_OF_REPORTS] = \
{\
    {MEASUREMENT_AND_SENSING_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,{0, E_ZCL_UINT16,E_CLD_ILLMEAS_ATTR_ID_MEASURED_VALUE,ZLO_MIN_REPORT_INTERVAL,ZLO_MAX_REPORT_INTERVAL,0,{LIGHT_SENSOR_MINIMUM_REPORTABLE_CHANGE}}},\
};

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: eApp_ZCL_RegisterEndpoint
 *
 * DESCRIPTION:
 * Register ZLO endpoints
 *
 * PARAMETER
 * Type                        Name                  Descirption
 * tfpZCL_ZCLCallBackFunction  fptr                  Pointer to ZCL Callback function
 *
 * RETURNS:
 * teZCL_Status
 *
 ****************************************************************************/
teZCL_Status eApp_ZCL_RegisterEndpoint(tfpZCL_ZCLCallBackFunction fptr)
{
    return eZLO_RegisterLightSensorEndPoint(LIGHTSENSOR_SENSOR_ENDPOINT,
                                              fptr,
                                              &sSensor);
}

/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_Init
 *
 * DESCRIPTION:
 * ZCL Device Specific initialization
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
void vAPP_ZCL_DeviceSpecific_Init(void)
{
    /* Initialise the strings in Basic */
    memcpy(sSensor.sBasicServerCluster.au8ManufacturerName, "NXP", CLD_BAS_MANUF_NAME_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "ZLO-LightSensor", CLD_BAS_MODEL_ID_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8DateCode, "20160210", CLD_BAS_DATE_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8SWBuildID, "4000-0001", CLD_BAS_SW_BUILD_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8ProductURL, "www.nxp.com", CLD_BAS_URL_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8ProductCode, "1234", CLD_BAS_PCODE_SIZE);
    sSensor.sBasicServerCluster.eGenericDeviceType = E_CLD_BAS_GENERIC_DEVICE_TYPE_MOTION_OR_LIGHT_SENSOR;

    /* Initialise the attribute in illuminance Measurement */
    sSensor.sIlluminanceMeasurementServerCluster.u16MeasuredValue = 0;
    sSensor.sIlluminanceMeasurementServerCluster.eLightSensorType = E_CLD_ILLMEAS_LST_CMOS;
    sSensor.sIlluminanceMeasurementServerCluster.u16MinMeasuredValue = LIGHT_SENSOR_MINIMUM_MEASURED_VALUE;
    sSensor.sIlluminanceMeasurementServerCluster.u16MaxMeasuredValue = LIGHT_SENSOR_MAXIMUM_MEASURED_VALUE;
}

/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_SetIdentifyTime
 *
 * DESCRIPTION:
 * ZCL Device Specific setting of identify time
 *
 * PARAMETER:
 * uint16 u16Time Identify time duration
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC void vAPP_ZCL_DeviceSpecific_SetIdentifyTime(uint16 u16Time)
{
    sSensor.sIdentifyServerCluster.u16IdentifyTime=u16Time;
}
/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_UpdateIdentify
 *
 * DESCRIPTION:
 * ZCL Device Specific identify updates
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC void vAPP_ZCL_DeviceSpecific_UpdateIdentify(void)
{
    if(sSensor.sIdentifyServerCluster.u16IdentifyTime%2)
    {
        bRGB_LED_SetGroupLevel(LED_LEVEL);
        bRGB_LED_On();
    }
    else
    {
        bRGB_LED_Off();
    }
}
/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_IdentifyOff
 *
 * DESCRIPTION:
 * ZCL Device Specific stop identify
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC void vAPP_ZCL_DeviceSpecific_IdentifyOff(void)
{
    vAPP_ZCL_DeviceSpecific_SetIdentifyTime(0);
    bRGB_LED_Off();
}

/****************************************************************************
 *
 * NAME: APP_cbTimerLightSensorSample
 *
 * DESCRIPTION:
 * CallBack For Light sensor sampling
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC void APP_cbTimerLightSensorSample(void *pvParam)
{
    /* Start Sampling timer only when you are keeping alive */
    if(APP_bPersistantPolling == TRUE)
        vAPP_LightSensorSample();
}

/****************************************************************************
 *
 * NAME: vAPP_LightSensorSample
 *
 * DESCRIPTION:
 * Light sensor sampling
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC void vAPP_LightSensorSample(void)
{
    bool_t bStatus;
    uint16 u16ALSResult;

    bStatus = bTSL2550_StartRead(TSL2550_CHANNEL_0);
    DBG_vPrintf(TRACE_LIGHT_SENSOR,"\nStart Read Status =%d",bStatus);

    u16ALSResult = u16TSL2550_ReadResult();
    DBG_vPrintf(TRACE_LIGHT_SENSOR,"\nResult = %d",u16ALSResult);

    if(u16ALSResult > (LIGHT_SENSOR_MINIMUM_MEASURED_VALUE - 1))
        sSensor.sIlluminanceMeasurementServerCluster.u16MeasuredValue = u16ALSResult;
    else
        sSensor.sIlluminanceMeasurementServerCluster.u16MeasuredValue = LIGHT_SENSOR_MINIMUM_MEASURED_VALUE;

    /* Start sample timer so that you keep on sampling if KEEPALIVE_TIME is too high*/
    ZTIMER_eStart(u8TimerLightSensorSample, ZTIMER_TIME_MSEC(1000 * LIGHT_SENSOR_SAMPLING_TIME_IN_SECONDS));
}

/****************************************************************************
 *
 * NAME: app_u8GetDeviceEndpoint
 *
 * DESCRIPTION:
 * Returns the application endpoint
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC uint8 app_u8GetDeviceEndpoint( void)
{
    return LIGHTSENSOR_SENSOR_ENDPOINT;
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
