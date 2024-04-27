#include "AF.h"
#include "OSAL.h"
#include "ZComDef.h"
#include "ZDConfig.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ms.h"
#include "zcl_ha.h"

#include "zcl_app.h"

#include "battery.h"
#include "version.h"
/*********************************************************************
 * CONSTANTS
 */

#define APP_DEVICE_VERSION 2
#define APP_FLAGS 0

#define APP_HWVERSION 1
#define APP_ZCLVERSION 1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Global attributes
const uint16 zclApp_clusterRevision_all = 0x0001;

int16 zclApp_Temperature_Sensor_MeasuredValue = 0;
int16 zclApp_PressureSensor_MeasuredValue = 0;
int16 zclApp_PressureSensor_ScaledValue = 0;
int8 zclApp_PressureSensor_Scale = -1;

uint16 zclApp_HumiditySensor_MeasuredValue = 0;

uint16 zclApp_SoilHumiditySensor_MeasuredValue = 0;
uint16 zclApp_SoilHumiditySensor_MeasuredValueRawAdc = 0;
bool zclApp_SoilHumiditySensor_Output = 0;

int16 zclApp_DS18B20_MeasuredValue = 0;

uint16 zclApp_IlluminanceSensor_MeasuredValue = 0;
uint16 zclApp_IlluminanceSensor_MeasuredValueRawAdc = 0;

// Basic Cluster
const uint8 zclApp_HWRevision = APP_HWVERSION;
const uint8 zclApp_ZCLVersion = APP_ZCLVERSION;
const uint8 zclApp_ApplicationVersion = 3;
const uint8 zclApp_StackVersion = 4;

//{lenght, 'd', 'a', 't', 'a'}
const uint8 zclApp_ManufacturerName[] = {7, 'B', 'a', 'c', 'c', 'h', 'u', 's'};
const uint8 zclApp_ModelId[] = {16, 'F', 'l', 'o', 'w', 'e', 'r', '_',  'S', 'e', 'n', 's', 'o', 'r', '_', 'v', '2'};
const uint8 zclApp_PowerSource = POWER_SOURCE_BATTERY;


#define DEFAULT_Threshold       50
#define DEFAULT_Interval        30


application_config_t zclApp_Config = {
    .Threshold =      DEFAULT_Threshold,
    .Interval =       DEFAULT_Interval,
};

/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */


CONST zclAttrRec_t zclApp_AttrsFirstEP[] = {
    {BASIC, {ATTRID_BASIC_ZCL_VERSION, ZCL_UINT8, R, (void *)&zclApp_ZCLVersion}},
    {BASIC, {ATTRID_BASIC_APPL_VERSION, ZCL_UINT8, R, (void *)&zclApp_ApplicationVersion}},
    {BASIC, {ATTRID_BASIC_STACK_VERSION, ZCL_UINT8, R, (void *)&zclApp_StackVersion}},
    {BASIC, {ATTRID_BASIC_HW_VERSION, ZCL_UINT8, R, (void *)&zclApp_HWRevision}},
    {BASIC, {ATTRID_BASIC_MANUFACTURER_NAME, ZCL_DATATYPE_CHAR_STR, R, (void *)zclApp_ManufacturerName}},
    {BASIC, {ATTRID_BASIC_MODEL_ID, ZCL_DATATYPE_CHAR_STR, R, (void *)zclApp_ModelId}},
    {BASIC, {ATTRID_BASIC_DATE_CODE, ZCL_DATATYPE_CHAR_STR, R, (void *)zclApp_DateCode}},
    {BASIC, {ATTRID_BASIC_POWER_SOURCE, ZCL_DATATYPE_ENUM8, R, (void *)&zclApp_PowerSource}},
    {BASIC, {ATTRID_BASIC_SW_BUILD_ID, ZCL_DATATYPE_CHAR_STR, R, (void *)zclApp_DateCode}},
    {BASIC, {ATTRID_CLUSTER_REVISION, ZCL_DATATYPE_UINT16, R, (void *)&zclApp_clusterRevision_all}},

    {POWER_CFG, {ATTRID_POWER_CFG_BATTERY_VOLTAGE, ZCL_UINT8, RR, (void *)&zclBattery_Voltage}},
    {POWER_CFG, {ATTRID_POWER_CFG_BATTERY_PERCENTAGE_REMAINING, ZCL_UINT8, RR, (void *)&zclBattery_PercentageRemainig}},
    {POWER_CFG, {ATTRID_POWER_CFG_BATTERY_VOLTAGE_RAW_ADC, ZCL_UINT16, RR, (void *)&zclBattery_RawAdc}},

    {ILLUMINANCE, {ATTRID_MS_ILLUMINANCE_MEASURED_VALUE, ZCL_UINT16, RR, (void *)&zclApp_IlluminanceSensor_MeasuredValue}},
    {TEMP, {ATTRID_MS_TEMPERATURE_MEASURED_VALUE, ZCL_INT16, RR, (void *)&zclApp_DS18B20_MeasuredValue}},

    {SOIL_MOISTURE, {ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE, ZCL_UINT16, RR, (void *)&zclApp_SoilHumiditySensor_MeasuredValue}},
    {SOIL_MOISTURE, {ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE_RAW_ADC, ZCL_UINT16, RR, (void *)&zclApp_SoilHumiditySensor_MeasuredValueRawAdc}},
    {SOIL_MOISTURE, {ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE_BATTERY_RAW_ADC, ZCL_UINT16, RR, (void *)&zclBattery_RawAdc}},
    {SOIL_MOISTURE, {ATTRID_MS_THRESHOLD, ZCL_UINT16, RW, (void *)&zclApp_Config.Threshold}},
    {SOIL_MOISTURE, {ATTRID_MS_INTERVAL,  ZCL_UINT16, RW, (void *)&zclApp_Config.Interval}},

    {GEN_ON_OFF, {ATTRID_ON_OFF, ZCL_BOOLEAN, RR, (void *)&zclApp_SoilHumiditySensor_Output}},
};


uint8 CONST zclApp_AttrsFirstEPCount = (sizeof(zclApp_AttrsFirstEP) / sizeof(zclApp_AttrsFirstEP[0]));


const cId_t zclApp_InClusterListFirstEP[] = {ZCL_CLUSTER_ID_GEN_BASIC, POWER_CFG, ILLUMINANCE, TEMP, SOIL_MOISTURE, GEN_ON_OFF};

#define APP_MAX_INCLUSTERS_FIRST_EP (sizeof(zclApp_InClusterListFirstEP) / sizeof(zclApp_InClusterListFirstEP[0]))

const cId_t zclApp_OutClusterListFirstEP[] = {SOIL_MOISTURE};

#define APP_MAX_OUTCLUSTERS_FIRST_EP (sizeof(zclApp_OutClusterListFirstEP) / sizeof(zclApp_OutClusterListFirstEP[0]))

SimpleDescriptionFormat_t zclApp_FirstEP = {
    1,                                                  //  int Endpoint;
    ZCL_HA_PROFILE_ID,                                  //  uint16 AppProfId[2];
    ZCL_HA_DEVICEID_SIMPLE_SENSOR,                      //  uint16 AppDeviceId[2];
    APP_DEVICE_VERSION,                                 //  int   AppDevVer:4;
    APP_FLAGS,                                          //  int   AppFlags:4;
    APP_MAX_INCLUSTERS_FIRST_EP,                        //  byte  AppNumInClusters;
    (cId_t *)zclApp_InClusterListFirstEP,               //  byte *pAppInClusterList;
    APP_MAX_OUTCLUSTERS_FIRST_EP,                       //  byte  AppNumOutClusters;
    (cId_t *)zclApp_OutClusterListFirstEP               //  byte *pAppOutClusterList;  
};



void zclApp_ResetAttributesToDefaultValues(void) {
    zclApp_Config.Threshold =     DEFAULT_Threshold;
    zclApp_Config.Interval =     DEFAULT_Interval;
}