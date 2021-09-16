/*
 * Copyright (c) 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/// @file mc3672.c

/* Standard and Zephyr libraries */
#include <zephyr/types.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <soc.h>
#include <devicetree.h>

#include "mc3672.h"

/* Logging module */
#include <logging/log.h>
LOG_MODULE_REGISTER(MC3672, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT mcube_mc3672 ///< Reference to the binding and compatible on the Device Tree.

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "RESP SIMU driver enabled without any devices"
#endif

/*******************************************************************************
 *** UTILITY
 *******************************************************************************/
#define M_DRV_MC_UTL_AXIS_X 0
#define M_DRV_MC_UTL_AXIS_Y 1
#define M_DRV_MC_UTL_AXIS_Z 2
#define M_DRV_MC_UTL_AXES_NUM 3

typedef struct
{
    signed char bSign[M_DRV_MC_UTL_AXES_NUM];
    unsigned char bMap[M_DRV_MC_UTL_AXES_NUM];
} S_M_DRV_MC_UTIL_OrientationReMap;

typedef enum
{
    E_M_DRV_UTIL_ORIENTATION_TOP_LEFT_DOWN = 0,
    E_M_DRV_UTIL_ORIENTATION_TOP_RIGHT_DOWN,
    E_M_DRV_UTIL_ORIENTATION_TOP_RIGHT_UP,
    E_M_DRV_UTIL_ORIENTATION_TOP_LEFT_UP,
    E_M_DRV_UTIL_ORIENTATION_BOTTOM_LEFT_DOWN,
    E_M_DRV_UTIL_ORIENTATION_BOTTOM_RIGHT_DOWN,
    E_M_DRV_UTIL_ORIENTATION_BOTTOM_RIGHT_UP,
    E_M_DRV_UTIL_ORIENTATION_BOTTOM_LEFT_UP,
    E_M_DRV_UTIL_ORIENTATION_TOTAL_CONFIG
} E_M_DRV_UTIL_OrientationConfig;

extern const S_M_DRV_MC_UTIL_OrientationReMap g_MDrvUtilOrientationReMap[E_M_DRV_UTIL_ORIENTATION_TOTAL_CONFIG];

const S_M_DRV_MC_UTIL_OrientationReMap
    g_MDrvUtilOrientationReMap[E_M_DRV_UTIL_ORIENTATION_TOTAL_CONFIG] =
        {
            {{1, 1, 1}, {M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_Z}},
            {{-1, 1, 1}, {M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Z}},
            {{-1, -1, 1}, {M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_Z}},
            {{1, -1, 1}, {M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Z}},

            {{-1, 1, -1}, {M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_Z}},
            {{1, 1, -1}, {M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Z}},
            {{1, -1, -1}, {M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_Z}},
            {{-1, -1, -1}, {M_DRV_MC_UTL_AXIS_Y, M_DRV_MC_UTL_AXIS_X, M_DRV_MC_UTL_AXIS_Z}},
};

/******************************************************************************
 *** CONFIGURATION
 *****************************************************************************/
/** !!! DO NOT use both I2C and SPI at the same time !!! */
#define M_DRV_MC36XX_CFG_BUS_I2C
//#define M_DRV_MC36XX_CFG_BUS_SPI

#if (!defined(M_DRV_MC36XX_CFG_BUS_SPI) && !defined(M_DRV_MC36XX_CFG_BUS_I2C))
#error "MUST use one bus to access register!"
#endif

#if (defined(M_DRV_MC36XX_CFG_BUS_SPI) && defined(M_DRV_MC36XX_CFG_BUS_I2C))
#error "DO NOT use both SPI and I2C simultaneously!"
#endif

#define M_DRV_MC36XX_CFG_I2C_ADDR (0x4C)
//#define M_DRV_MC36XX_CFG_I2C_ADDR    (0x6C)

#define M_DRV_MC36XX_OPERATE_MODE_WAKE_WHEN_READ
//#define M_DRV_MC36XX_SUPPORT_LPF

//#define DEBUG_MODE

#define M_DRV_MC36XX_CFG_SAMPLE_RATE_CWAKE_DEFAULT \
    E_M_DRV_MC36XX_CWAKE_SR_LP_54Hz
#define M_DRV_MC36XX_CFG_SAMPLE_RATE_SNIFF_DEFAULT \
    E_M_DRV_MC36XX_SNIFF_SR_6Hz
#define M_DRV_MC36XX_CFG_RANGE \
    E_M_DRV_MC36XX_RANGE_4G
#define M_DRV_MC36XX_CFG_RESOLUTION \
    E_M_DRV_MC36XX_RES_12BIT
#define M_DRV_MC36XX_CFG_ORIENTATION_MAP \
    E_M_DRV_UTIL_ORIENTATION_TOP_RIGHT_UP
#define M_DRV_MC36XX_CFG_WAKE_GAIN_DEFAULT \
    E_M_DRV_MC36XX_WAKE_GAIN_LOW
#define M_DRV_MC36XX_CFG_SNIFF_GAIN_DEFAULT \
    E_M_DRV_MC36XX_SNIFF_GAIN_HIGH
#define M_DRV_MC36XX_CFG_INT_MODE_DEFAULT \
    M_DRV_MC36XX_INTR_C_IAH_ACTIVE_LOW    \
    | M_DRV_MC36XX_INTR_C_IPP_MODE_PUSH_PULL

#ifdef M_DRV_MC36XX_SUPPORT_LPF
/** (Hz) Sampling frequency */
#define SAMPLE_RATE 50
/** Sampling interval */
#define SAMPLE_INTERVAL 1 / SAMPLE_RATE
/** (Hz) Cut-off frequency */
#define CUT_OFF_FREQUENCY 4
#define PI 3.14
/** LPF coefficient */
#define ALPHA \
    SAMPLE_INTERVAL / (1 / ((2 * PI * CUT_OFF_FREQUENCY) + SAMPLE_INTERVAL))
#endif

#define _M_DRV_MC36XX_REG_STATUS_1_MODE(bRegStatus1) \
    (bRegStatus1 & 0x07)
#define _M_DRV_MC36XX_REG_STATUS_1_NEW_DATA(bRegStatus1) \
    (bRegStatus1 & 0x08)
#define _M_DRV_MC36XX_REG_STATUS_1_FIFO_EMPTY(bRegStatus1) \
    (bRegStatus1 & 0x10)
#define _M_DRV_MC36XX_REG_STATUS_1_FIFO_FULL(bRegStatus1) \
    (bRegStatus1 & 0x20)
#define _M_DRV_MC36XX_REG_STATUS_1_FIFO_THRESH(bRegStatus1) \
    (bRegStatus1 & 0x40)
#define _M_DRV_MC36XX_REG_STATUS_1_INT_PEND(bRegStatus1) \
    (bRegStatus1 & 0x80)

#define _M_DRV_MC36XX_REG_STATUS_2_INT_WAKE(bRegStatus2) \
    ((bRegStatus2 >> 2) & 0x01)
#define _M_DRV_MC36XX_REG_STATUS_2_INT_ACQ(bRegStatus2) \
    ((bRegStatus2 >> 3) & 0x01)
#define _M_DRV_MC36XX_REG_STATUS_2_INT_FIFO_EMPTY(bRegStatus2) \
    ((bRegStatus2 >> 4) & 0x01)
#define _M_DRV_MC36XX_REG_STATUS_2_INT_FIFO_FULL(bRegStatus2) \
    ((bRegStatus2 >> 5) & 0x01)
#define _M_DRV_MC36XX_REG_STATUS_2_INT_FIFO_THRESH(bRegStatus2) \
    ((bRegStatus2 >> 6) & 0x01)
#define _M_DRV_MC36XX_REG_STATUS_2_INT_SWAKE_SNIFF(bRegStatus2) \
    ((bRegStatus2 >> 7) & 0x01)

#define _M_DRV_MC36XX_REG_MODE_C_MODE(bRegMODE_C) \
    (bRegMODE_C & 0x07)
#define _M_DRV_MC36XX_REG_RANGE_C_RES(bRegRANGE_C) \
    (bRegRANGE_C & 0x07)
#define _M_DRV_MC36XX_REG_RANGE_C_RANGE(bRegRANGE_C) \
    ((bRegRANGE_C >> 4) & 0x07)

#define _M_DRV_MC36XX_REG_FIFO_C_FIFO_EN(bRegFIFO_C) \
    (bRegFIFO_C & 0x40)
#define _M_DRV_MC36XX_FIFO_VDD_EN(bRegPwrCtrl) \
    (bRegPwrCtrl | 0x42);

#ifdef M_DRV_MC36XX_SUPPORT_LPF
#define _M_DRV_MC36XX_SENSOR_FILTER(last_data, curr_data) \
    ((ALPHA * (last_data)) + ((1 - ALPHA) * (curr_data)))
#endif

/******************************************************************************
 *** STATIC VARIABLE
 *****************************************************************************/
/** unit: SI/LSB, SI: m/s^2 */
static float s_fMC36XX_Sensitivity = 0.0f;

static e_m_drv_mc36xx_range_t s_eRange = M_DRV_MC36XX_CFG_RANGE;
static e_m_drv_mc36xx_res_t s_eRes = M_DRV_MC36XX_CFG_RESOLUTION;
static e_m_drv_mc36xx_cwake_sr_t s_eSR_CWAKE = M_DRV_MC36XX_CFG_SAMPLE_RATE_CWAKE_DEFAULT;
static e_m_drv_mc36xx_sniff_sr_t s_eSR_SNIFF = M_DRV_MC36XX_CFG_SAMPLE_RATE_SNIFF_DEFAULT;
static e_m_drv_mc36xx_wake_gain_t s_eGAIN_WAKE = M_DRV_MC36XX_CFG_WAKE_GAIN_DEFAULT;
static e_m_drv_mc36xx_sniff_gain_t s_eGAIN_SNIFF = M_DRV_MC36XX_CFG_SNIFF_GAIN_DEFAULT;

static uint_dev s_bCfgRngResol = 0x00;
static uint_dev s_bCfgSniffThr = 0x00;
static uint_dev s_bCfgSniffCfg = 0x00;
static uint_dev s_bCfgFifo = 0x00;
static uint_dev s_bCfgINT = 0x00;
static uint_dev s_bCfgFifoVdd = 0x42;
static uint_dev s_bCfgWakeSRMode = 0xFF;
static uint_dev s_bCfgSniffSRMode = 0xFF;
static uint_dev s_bCfgHSMode = 0x00;

static int g_bIsMC3610 = 0;

#ifdef M_DRV_MC36XX_SUPPORT_LPF
static short _saLPFPrevData[M_DRV_MC36XX_AXES_NUM] = {0};
#endif

/******************************************************************************
 *** NON API FUNCTIONS
 *****************************************************************************/
static int _M_DRV_MC36XX_REG_READ(const struct device *dev, uint8_t reg_addr, uint8_t *value)
{
    uint8_t buf[1];
    if (i2c_burst_read(dev, DT_INST_REG_ADDR(0), reg_addr, (uint8_t *)buf, 1) < 0)
    {
        LOG_ERR("Error reading register.");
        return -1;
    }
    *value = buf[0];
    return 0;
}

static int _M_DRV_MC36XX_REG_WRITE(const struct device *dev, uint8_t reg_addr, uint8_t value)
{
    if (i2c_burst_write(dev, DT_INST_REG_ADDR(0), reg_addr, &value, 1) < 0)
    {
        LOG_ERR("Error writing register.");
        return -1;
    }
    return 0;
}

/******************************************************************************
 *** _M_DRV_MC36XX_SetSniffAGAIN
 *****************************************************************************/
static int _M_DRV_MC36XX_SetSniffAGAIN(const struct device *dev, uint8_t SniffGain)
{
    LOG_DBG("[%s] SniffGain= 0x%02X\r\n", "SetSniffAGAIN", SniffGain);

    uint_dev _bRegAGain = 0x00;

    if (SniffGain > E_M_DRV_MC36XX_SNIFF_GAIN_END)
    {
        LOG_DBG("[%s] SniffGain > %d\r\n", "SetSniffAGAIN", SniffGain);
        return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);
    }
    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_DMX, _bRegAGain);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "SetSniffAGAIN",
            E_M_DRV_MC36XX_REG_DMX, _bRegAGain);

    _bRegAGain = 0x00;
    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_DMY, &_bRegAGain);
    _bRegAGain &= 0x3F;
    _bRegAGain |= (SniffGain << 6);

    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_DMY, _bRegAGain);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "SetSniffAGAIN",
            E_M_DRV_MC36XX_REG_DMY, _bRegAGain);

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** _M_DRV_MC36XX_SetWakeAGAIN
 *****************************************************************************/
static int _M_DRV_MC36XX_SetWakeAGAIN(const struct device *dev, uint8_t WakeGain)
{
    LOG_DBG("[%s] WakeGain= 0x%02X\r\n", "SetWakeAGAIN", WakeGain);

    uint_dev _bRegAGain = 0x01;

    if (WakeGain > E_M_DRV_MC36XX_WAKE_GAIN_END)
    {
        LOG_ERR("[%s] WakeGain > %d\r\n", "SetWakeAGAIN", WakeGain);
        return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);
    }

    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_DMX, _bRegAGain);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "SetWakeAGAIN",
            E_M_DRV_MC36XX_REG_DMX, _bRegAGain);

    _bRegAGain = 0x00;
    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_DMY, &_bRegAGain);

    _bRegAGain &= 0x3F;
    _bRegAGain |= (WakeGain << 6);
    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_DMY, _bRegAGain);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "SetWakeAGAIN",
            E_M_DRV_MC36XX_REG_DMY, _bRegAGain);

    _bRegAGain = 0x00;
    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_DMX, _bRegAGain);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "SetWakeAGAIN",
            E_M_DRV_MC36XX_REG_DMX, _bRegAGain);

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** _M_DRV_MC36XX_ValidateSensorIC
 *****************************************************************************/
static int _M_DRV_MC36XX_ValidateSensorIC(const struct device *dev)
{
    uint8_t _bRegData = 0;

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_PROD, &_bRegData);
    LOG_DBG("%s: 0x18[0x%x]\r\n", "Validate Sensor IC", _bRegData);

    _bRegData &= 0xff;

    if (0x71 == _bRegData)
    {
        _M_DRV_MC36XX_REG_READ(dev, 0x19, &_bRegData);
        if (_bRegData < 3)
        {
            LOG_DBG("[%s] Orion. \r\n", "Validate Sensor IC");
            return (M_DRV_MC36XX_RETCODE_SUCCESS);
        }
    }
    else if (0x70 == _bRegData)
    {
        _M_DRV_MC36XX_REG_READ(dev, 0x34, &_bRegData);
        if (0x30 == _bRegData)
        {
            _M_DRV_MC36XX_REG_READ(dev, 0x3B, &_bRegData);
            if (0x10 == (_bRegData & 0xF3))
            {
                g_bIsMC3610 = 1;
                LOG_DBG("[%s] Orion mc3610. \r\n", "Validate Sensor IC");
                return (M_DRV_MC36XX_RETCODE_SUCCESS);
            }
        }
    }

    return (M_DRV_MC36XX_RETCODE_ERROR_IDENTIFICATION);
}

/******************************************************************************
 *** _M_DRV_MC36XX_ResetChip
 *****************************************************************************/
static int _M_DRV_MC36XX_ResetChip(const struct device *dev)
{
    uint8_t _bRegData = 0x01;

    if (_M_DRV_MC36XX_REG_WRITE(dev, 0x10, _bRegData) < 0)
    {
        LOG_ERR("Failed to 1.\n");
        return -EINVAL;
    }

    k_busy_wait(10000);

    _bRegData = 0x40;
    if (_M_DRV_MC36XX_REG_WRITE(dev, 0x24, _bRegData) < 0)
    {
        LOG_ERR("Failed to 1.\n");
        return -EINVAL;
    }

    k_busy_wait(50000);

    /** dummy write */
    _bRegData = 0x00;
    if (_M_DRV_MC36XX_REG_WRITE(dev, 0x09, _bRegData) < 0)
    {
        LOG_ERR("Failed to 1.\n");
        return -EINVAL;
    }
    _bRegData = 0x42;
    if (_M_DRV_MC36XX_REG_WRITE(dev, 0x0F, _bRegData) < 0)
    {
        LOG_ERR("Failed to 1.\n");
        return -EINVAL;
    }
    _bRegData = 0x01;
    if (_M_DRV_MC36XX_REG_WRITE(dev, 0x20, _bRegData) < 0)
    {
        LOG_ERR("Failed to 1.\n");
        return -EINVAL;
    }
    _bRegData = 0x80;
    if (_M_DRV_MC36XX_REG_WRITE(dev, 0x21, _bRegData) < 0)
    {
        LOG_ERR("Failed to 1.\n");
        return -EINVAL;
    }
    _bRegData = 0x00;
    if (_M_DRV_MC36XX_REG_WRITE(dev, 0x28, _bRegData) < 0)
    {
        LOG_ERR("Failed to 1.\n");
        return -EINVAL;
    }
    _bRegData = 0x00;
    if (_M_DRV_MC36XX_REG_WRITE(dev, 0x1a, _bRegData) < 0)
    {
        LOG_ERR("Failed to 1.\n");
        return -EINVAL;
    }

    _bRegData = 0x01;
    if (_M_DRV_MC36XX_REG_WRITE(dev, 0x10, _bRegData) < 0)
    {
        LOG_ERR("Failed to 1.\n");
        return -EINVAL;
    }

    k_busy_wait(10000);

    return 0;
}

#ifdef M_DRV_MC36XX_SUPPORT_LPF
/******************************************************************************
 *** _M_DRV_MC36XX_LowPassFilter
 *****************************************************************************/
static void _M_DRV_MC36XX_LowPassFilter(signed short _saData[M_DRV_MC36XX_AXES_NUM])
{
    LOG_DBG("[%s]\r\n", "Low Pass Filter");
    LOG_DBG("[CurrData]     %d    %d    %d",
            _saData[M_DRV_MC36XX_AXIS_X],
            _saData[M_DRV_MC36XX_AXIS_Y],
            _saData[M_DRV_MC36XX_AXIS_Z]);

    LOG_DBG("[PreData]     %d    %d    %d",
            _saLPFPrevData[M_DRV_MC36XX_AXIS_X],
            _saLPFPrevData[M_DRV_MC36XX_AXIS_Y],
            _saLPFPrevData[M_DRV_MC36XX_AXIS_Z]);

    _saData[M_DRV_MC36XX_AXIS_X] =
        _M_DRV_MC36XX_SENSOR_FILTER(_saLPFPrevData[M_DRV_MC36XX_AXIS_X],
                                    _saData[M_DRV_MC36XX_AXIS_X]);
    _saData[M_DRV_MC36XX_AXIS_Y] =
        _M_DRV_MC36XX_SENSOR_FILTER(_saLPFPrevData[M_DRV_MC36XX_AXIS_Y],
                                    _saData[M_DRV_MC36XX_AXIS_Y]);
    _saData[M_DRV_MC36XX_AXIS_Z] =
        _M_DRV_MC36XX_SENSOR_FILTER(_saLPFPrevData[M_DRV_MC36XX_AXIS_Z],
                                    _saData[M_DRV_MC36XX_AXIS_Z]);

    _saLPFPrevData[M_DRV_MC36XX_AXIS_X] = _saData[M_DRV_MC36XX_AXIS_X];
    _saLPFPrevData[M_DRV_MC36XX_AXIS_Y] = _saData[M_DRV_MC36XX_AXIS_Y];
    _saLPFPrevData[M_DRV_MC36XX_AXIS_Z] = _saData[M_DRV_MC36XX_AXIS_Z];

    LOG_DBG("[M_DRV_MC36XX_SUPPORT_LPF]     %d    %d    %d",
            _saData[M_DRV_MC36XX_AXIS_X],
            _saData[M_DRV_MC36XX_AXIS_Y],
            _saData[M_DRV_MC36XX_AXIS_Z]);
}
#endif // #ifdef M_DRV_MC36XX_SUPPORT_LPF

/******************************************************************************
 *** _M_DRV_MC36XX_SetBusIF
 *****************************************************************************/
static int _M_DRV_MC36XX_SetBusIF(const struct device *dev)
{
    LOG_DBG("[%s]\r\n", "Set Bus IF");

    /*Orion I2C/SPI interface Setup */
    uint_dev _bRegIO_C = 0;
    uint_dev _bRegSNIFFTH_C = 0;

    if (g_bIsMC3610 == 0)
    {
        LOG_DBG("Sensor MC36XX.\r\n");
        _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_FEATURE_C_1, &_bRegIO_C);
#ifdef M_DRV_MC36XX_CFG_BUS_I2C
        _bRegIO_C &= 0x3F;
        _bRegIO_C |= 0x40;
#else
        _bRegIO_C &= 0x3F;
        _bRegIO_C |= 0x80;
#endif
        _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_FEATURE_C_1, _bRegIO_C);
    }
    else
    {
        LOG_DBG("Sensor MC3610.\r\n");
        _M_DRV_MC36XX_REG_READ(dev, 0x13, &_bRegSNIFFTH_C);
        _M_DRV_MC36XX_REG_READ(dev, 0x14, &_bRegIO_C);
#ifdef M_DRV_MC36XX_CFG_BUS_I2C
        _bRegSNIFFTH_C |= 0x80;
        _bRegIO_C &= 0x7F;
#else
        _bRegSNIFFTH_C &= 0x7F;
        _bRegIO_C |= 0x80;
#endif
        _M_DRV_MC36XX_REG_WRITE(dev, 0x13, _bRegSNIFFTH_C);
        _M_DRV_MC36XX_REG_WRITE(dev, 0x14, _bRegIO_C);
    }

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** _M_DRV_MC36XX_SetSniffOverSR
 *****************************************************************************/
static void _M_DRV_MC36XX_SetSniffOverSR(const struct device *dev, uint8_t bDesiredSRMode)
{
    LOG_DBG("[%s]\r\n", "Set Sniff Over SR");

    uint_dev _bRegData = 0x00;

    if (s_bCfgSniffSRMode == bDesiredSRMode)
        return;

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_PMCR, &_bRegData);
    _bRegData &= 0x8F;
    _bRegData |= (s_bCfgHSMode | (bDesiredSRMode << 4));
    s_bCfgHSMode = _bRegData;

    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_PMCR, _bRegData);
    LOG_DBG("[%s] REG[0x1C] 0x%02X\r\n", "Set Sniff Over SR", _bRegData);
    s_bCfgSniffSRMode = bDesiredSRMode;
}

/******************************************************************************
 *** _M_DRV_MC36XX_SetWakeOverSR
 *****************************************************************************/
static void _M_DRV_MC36XX_SetWakeOverSR(const struct device *dev, uint8_t bDesiredSRMode)
{
    LOG_DBG("[%s]\r\n", "Set Wake Over SR");

    uint_dev _bRegData = 0x00;

    if (s_bCfgWakeSRMode == bDesiredSRMode)
        return;

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_PMCR, &_bRegData);
    _bRegData &= 0xF8;
    _bRegData |= (s_bCfgHSMode | bDesiredSRMode);
    s_bCfgHSMode = _bRegData;

    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_PMCR, _bRegData);
    LOG_DBG("[%s] REG[0x1C] 0x%02X\r\n", "Set Wake Over SR", _bRegData);
    s_bCfgWakeSRMode = bDesiredSRMode;
}

/******************************************************************************
 *** _M_DRV_MC36XX_SetCwakeSR
 *****************************************************************************/
static int _M_DRV_MC36XX_SetCwakeSR(const struct device *dev, e_m_drv_mc36xx_cwake_sr_t eSR)
{
    LOG_DBG("[%s]\r\n", "Set Cwake SR");

    uint_dev _bRegMODE = ((eSR & 0x70) >> 4);
    uint_dev _bRegWAKE = (eSR & 0x0f);

    _M_DRV_MC36XX_SetWakeOverSR(dev, _bRegMODE);
    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_WAKE_C, _bRegWAKE);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "Set Cwake SR",
            E_M_DRV_MC36XX_REG_WAKE_C, _bRegWAKE);

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** _M_DRV_MC36XX_CheckCwakeSR
 *****************************************************************************/
static int _M_DRV_MC36XX_CheckCwakeSR(e_m_drv_mc36xx_cwake_sr_t eSR)
{
    LOG_DBG("[%s]\r\n", "Check Cwake SR");

    uint_dev _bRegMODE = ((eSR & 0x70) >> 4);

    if (_bRegMODE > E_M_DRV_MC36XX_WAKE_SR_MODE_HIGH_PRECISION)
        return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** _M_DRV_MC36XX_SetSniffSR
 *****************************************************************************/
static int _M_DRV_MC36XX_SetSniffSR(const struct device *dev, e_m_drv_mc36xx_sniff_sr_t eSR)
{
    LOG_DBG("[%s]\r\n", "Set Sniff SR");

    uint_dev _bRegWAKE = 0x00;
    uint_dev _bRegSNIFF = 0xC0;

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_WAKE_C, &_bRegWAKE);
    _M_DRV_MC36XX_SetSniffOverSR(dev, E_M_DRV_MC36XX_SNIFF_SR_MODE_LOW_POWER);

    _bRegSNIFF |= eSR;
    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_SNIFF_C, _bRegSNIFF);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "Set Sniff SR",
            E_M_DRV_MC36XX_REG_SNIFF_C, _bRegSNIFF);

    _bRegWAKE &= 0x7F;
    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_WAKE_C, _bRegWAKE);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "Set Sniff SR",
            E_M_DRV_MC36XX_REG_WAKE_C, _bRegWAKE);

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** _M_DRV_MC36XX_SetMode
 *****************************************************************************/
static int _M_DRV_MC36XX_SetMode(const struct device *dev, e_m_drv_mc36xx_mode_t eNextMode)
{
    uint_dev _bCurrMode = 0;
    uint_dev _bRegMODE_C = 0;
    uint_dev _bGuard = 0;

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_STATUS_1, &_bCurrMode);

    if (eNextMode == _M_DRV_MC36XX_REG_STATUS_1_MODE(_bCurrMode))
        return (M_DRV_MC36XX_RETCODE_ERROR_STATUS);

    switch ((int)eNextMode)
    {
    case E_M_DRV_MC36XX_MODE_SNIFF:
        _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_PWR_CONTROL,
                                s_bCfgFifoVdd);
        LOG_DBG("[%s] MODE_SNIFF\r\n", "Set Mode");
        break;
    case E_M_DRV_MC36XX_MODE_SLEEP:
        _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_PWR_CONTROL,
                                s_bCfgFifoVdd);
        LOG_DBG("[%s] MODE_SLEEP\r\n", "Set Mode");
        break;
    case E_M_DRV_MC36XX_MODE_STANDBY:
        LOG_DBG("[%s] MODE_STANDBY\r\n", "Set Mode");
        break;
    default:
        LOG_DBG("[%s] MODE WAKE\r\n", "Set Mode");
        if (E_M_DRV_MC36XX_MODE_CWAKE == eNextMode)
        {
            _M_DRV_MC36XX_SetCwakeSR(dev, s_eSR_CWAKE);
        }
        break;
    }

    _bRegMODE_C |= eNextMode;

    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_MODE_C, _bRegMODE_C);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "Set Mode",
            E_M_DRV_MC36XX_REG_MODE_C, _bRegMODE_C);

    while (1)
    {
        _bGuard++;
        //_M_DRV_MC36XX_Delay(1);
        k_busy_wait(1000);
        _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_STATUS_1, &_bCurrMode);
        if (eNextMode == _M_DRV_MC36XX_REG_STATUS_1_MODE(_bCurrMode))
        {
            LOG_DBG("SET MODE SUCCESS!!\r\n");
            LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "Set Mode",
                    E_M_DRV_MC36XX_REG_MODE_C, _bCurrMode & 0xFF);
            break;
        }
        if (_bGuard > 100)
        {
            LOG_ERR("M_DRV_MC36XX_RETCODE_ERROR_SETUP\r\n");
            return (M_DRV_MC36XX_RETCODE_ERROR_SETUP);
        }
    }

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** _M_DRV_MC36XX_ReadRawData
 *****************************************************************************/
static int _M_DRV_MC36XX_ReadRawData(const struct device *dev, signed short Output[M_DRV_MC36XX_AXES_NUM])
{
    LOG_DBG("[%s]\r\n", "Read Raw Data");

    signed short _waRaw[M_DRV_MC36XX_AXES_NUM] = {0};
    m_drv_buffer(180) _baData = {0};
    const S_M_DRV_MC_UTIL_OrientationReMap *_ptOrienMap = &g_MDrvUtilOrientationReMap[M_DRV_MC36XX_CFG_ORIENTATION_MAP];

    i2c_burst_read(dev, DT_INST_REG_ADDR(0), E_M_DRV_MC36XX_REG_XOUT_LSB,
                   (uint_dev *)_baData.words, 6);

    _waRaw[M_DRV_MC36XX_AXIS_X] = ((signed short)((_baData.bytes[0]) | (_baData.bytes[1] << 8)));
    _waRaw[M_DRV_MC36XX_AXIS_Y] = ((signed short)((_baData.bytes[2]) | (_baData.bytes[3] << 8)));
    _waRaw[M_DRV_MC36XX_AXIS_Z] = ((signed short)((_baData.bytes[4]) | (_baData.bytes[5] << 8)));

#ifdef M_DRV_MC36XX_SUPPORT_LPF
    _M_DRV_MC36XX_LowPassFilter(_waRaw);
#endif

    Output[M_DRV_MC36XX_AXIS_X] =
        ((_ptOrienMap->bSign[M_DRV_MC36XX_AXIS_X] * _waRaw[_ptOrienMap->bMap[M_DRV_MC36XX_AXIS_X]]));
    Output[M_DRV_MC36XX_AXIS_Y] =
        ((_ptOrienMap->bSign[M_DRV_MC36XX_AXIS_Y] * _waRaw[_ptOrienMap->bMap[M_DRV_MC36XX_AXIS_Y]]));
    Output[M_DRV_MC36XX_AXIS_Z] =
        ((_ptOrienMap->bSign[M_DRV_MC36XX_AXIS_Z] * _waRaw[_ptOrienMap->bMap[M_DRV_MC36XX_AXIS_Z]]));

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** _M_DRV_MC36XX_ReadData
 *****************************************************************************/
static int _M_DRV_MC36XX_ReadData(const struct device *dev, float faOutput[M_DRV_MC36XX_AXES_NUM])
{
    LOG_DBG("[%s]\r\n", "Read Data");

    signed short _waRaw[M_DRV_MC36XX_AXES_NUM] = {0};

    _M_DRV_MC36XX_ReadRawData(dev, _waRaw);

    faOutput[M_DRV_MC36XX_AXIS_X] = (float)((_waRaw[M_DRV_MC36XX_AXIS_X] * s_fMC36XX_Sensitivity));
    faOutput[M_DRV_MC36XX_AXIS_Y] = (float)((_waRaw[M_DRV_MC36XX_AXIS_Y] * s_fMC36XX_Sensitivity));
    faOutput[M_DRV_MC36XX_AXIS_Z] = (float)((_waRaw[M_DRV_MC36XX_AXIS_Z] * s_fMC36XX_Sensitivity));
    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** M_DRV_MC36XX_SetMode
 *****************************************************************************/
int M_DRV_MC36XX_SetMode(const struct device *dev, e_m_drv_mc36xx_mode_t eNextMode)
{
    LOG_DBG("[%s]\r\n", "Set Mode sans _");

    if (!(E_M_DRV_MC36XX_MODE_STANDBY ^ eNextMode))
        _M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_STANDBY);
    _M_DRV_MC36XX_SetMode(dev, eNextMode);

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** M_DRV_MC36XX_ConfigRegRngResCtrl
 *****************************************************************************/
int M_DRV_MC36XX_ConfigRegRngResCtrl(const struct device *dev, e_m_drv_mc36xx_range_t eCfgRange,
                                     e_m_drv_mc36xx_res_t eCfgResolution)
{
    LOG_DBG("[%s]\r\n", "Config Reg Rng Res Ctrl");

    uint_dev _bPreMode = 0;
    float _faRange[E_M_DRV_MC36XX_RANGE_END] = {19.614f, 39.228f, 78.456f, 156.912f, 117.684f, 235.368f};
    float _faResolution[E_M_DRV_MC36XX_RES_END] = {32.0f, 64.0f, 128.0f, 512.0f, 2048.0f, 8192.0f};

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_MODE_C, &_bPreMode);
    _M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_STANDBY);

    if (E_M_DRV_MC36XX_RES_12BIT < eCfgResolution)
    {
        s_bCfgFifo = 0x80;
        _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_FIFO_C, s_bCfgFifo);
        s_bCfgFifo = 0x00;
        _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_FIFO_C, s_bCfgFifo);
    }

    s_bCfgRngResol = (((eCfgRange << 4) & 0x70) | eCfgResolution);
    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_RANGE_C, s_bCfgRngResol);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "Config Reg Rng Res Ctrl",
            E_M_DRV_MC36XX_REG_RANGE_C, s_bCfgRngResol);

    s_fMC36XX_Sensitivity = (_faRange[eCfgRange] / _faResolution[eCfgResolution]);
    LOG_DBG("[%s] s_fMC36XX_Sensitivity=%f\r\n", "Config Reg Rng Res Ctrl",
            s_fMC36XX_Sensitivity);
    _M_DRV_MC36XX_SetMode(dev, (e_m_drv_mc36xx_mode_t)
                                   _M_DRV_MC36XX_REG_MODE_C_MODE(_bPreMode));
    s_eRange = eCfgRange;
    s_eRes = eCfgResolution;

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/*****************************************
 *** M_DRV_MC36XX_SetSampleRate
 *****************************************/
int M_DRV_MC36XX_SetSampleRate(const struct device *dev, e_m_drv_mc36xx_cwake_sr_t eCwakeSR,
                               e_m_drv_mc36xx_sniff_sr_t eSniffSR)
{
    LOG_DBG("[%s]\r\n", "Set sample Rate");

    uint_dev _bPreMode = 0;

    if (M_DRV_MC36XX_RETCODE_SUCCESS != _M_DRV_MC36XX_CheckCwakeSR(eCwakeSR))
        return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

    s_eSR_CWAKE = eCwakeSR;
    s_eSR_SNIFF = eSniffSR;

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_MODE_C, &_bPreMode);
    _M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_STANDBY);

    _M_DRV_MC36XX_SetSniffSR(dev, s_eSR_SNIFF);
    _M_DRV_MC36XX_SetCwakeSR(dev, s_eSR_CWAKE);

    _M_DRV_MC36XX_SetMode(dev, (e_m_drv_mc36xx_mode_t)
                                   _M_DRV_MC36XX_REG_MODE_C_MODE(_bPreMode));

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** M_DRV_MC36XX_SetGain
 *****************************************************************************/
int M_DRV_MC36XX_SetGain(const struct device *dev, e_m_drv_mc36xx_wake_gain_t eWakeGain,
                         e_m_drv_mc36xx_sniff_gain_t eSniffGain)
{
    LOG_DBG("[%s]\r\n", "Set Gain");

    uint_dev _bPreMode = 0x00;

    s_eGAIN_WAKE = eWakeGain;
    s_eGAIN_SNIFF = eSniffGain;

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_MODE_C, &_bPreMode);
    _M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_STANDBY);

    _M_DRV_MC36XX_SetWakeAGAIN(dev, s_eGAIN_WAKE);
    _M_DRV_MC36XX_SetSniffAGAIN(dev, s_eGAIN_SNIFF);

    _M_DRV_MC36XX_SetMode(dev, (e_m_drv_mc36xx_mode_t)
                                   _M_DRV_MC36XX_REG_MODE_C_MODE(_bPreMode));

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** M_DRV_MC36XX_SetSniffThreshold
 *****************************************************************************/
int M_DRV_MC36XX_SetSniffThreshold(const struct device *dev, uint8_t axis, uint8_t sniff_thr)
{
    LOG_DBG("[%s]\r\n", "Set Sniff Threshold");

    uint_dev _bRegSniff_addr = 0;

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_SNIFFTH_C, &s_bCfgSniffThr);

    if (axis < M_DRV_MC36XX_AXES_NUM)
        _bRegSniff_addr = axis + 1;
    else
        return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_SNIFF_CFG, _bRegSniff_addr);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "Set Sniff Threshold",
            E_M_DRV_MC36XX_REG_SNIFF_CFG, _bRegSniff_addr);

    s_bCfgSniffThr |= sniff_thr;
    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_SNIFFTH_C, s_bCfgSniffThr);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "Set Sniff Threshold",
            E_M_DRV_MC36XX_REG_SNIFFTH_C, s_bCfgSniffThr);

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** M_DRV_MC36XX_SetSniffDetectCount
 *****************************************************************************/
int M_DRV_MC36XX_SetSniffDetectCount(const struct device *dev, uint8_t axis, uint8_t SniffCount)
{
    LOG_DBG("[%s]\r\n", "Set Sniff Detect Count");

    /** unsigned SNIFF event count, 1 to 62 events,
        independent from other channels */
    uint_dev _bRegSniff_Count = SniffCount;
    uint_dev _bRegSniff_Axis = 0;
    uint_dev _bRegSniff_Count_En = 0x08;

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_SNIFFTH_C, &s_bCfgSniffThr);
    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_SNIFF_CFG, &s_bCfgSniffCfg);

    if (axis < M_DRV_MC36XX_AXES_NUM)
        _bRegSniff_Axis = axis + 5;
    else
        return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

    s_bCfgSniffCfg |= _bRegSniff_Axis;
    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_SNIFF_CFG, s_bCfgSniffCfg);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "Set Sniff Detect Count",
            E_M_DRV_MC36XX_REG_SNIFF_CFG, s_bCfgSniffCfg);

    /** Set detection count as (count +1) */
    s_bCfgSniffThr |= _bRegSniff_Count;
    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_SNIFFTH_C, s_bCfgSniffThr);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "Set Sniff Detect Count",
            E_M_DRV_MC36XX_REG_SNIFFTH_C, s_bCfgSniffThr);

    /** Enable SNIFF detection counts, required for valid SNIFF wake-up */
    s_bCfgSniffCfg |= _bRegSniff_Count_En;
    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_SNIFF_CFG, s_bCfgSniffCfg);
    LOG_DBG("0x%x : s_bCfgSniffCfg:%x\r\n",
            E_M_DRV_MC36XX_REG_SNIFF_CFG, s_bCfgSniffCfg);
    LOG_DBG("0x%x : s_bCfgSniffThr:%x\r\n",
            E_M_DRV_MC36XX_REG_SNIFFTH_C, s_bCfgSniffThr);
    LOG_DBG("[%s] REG[0x%02X] 0x%02X", "Set Sniff Detect Count",
            E_M_DRV_MC36XX_REG_SNIFF_CFG, s_bCfgSniffCfg);

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** M_DRV_MC36XX_SetSniffAndOrN
 *****************************************************************************/
int M_DRV_MC36XX_SetSniffAndOrN(const struct device *dev, e_m_drv_mc36xx_andorn_t LogicAndOr)
{
    LOG_DBG("[%s]\r\n", "Set Sniff And Or N");

    uint_dev _bRegAndOrN = 0x00;

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_SNIFFTH_C, &_bRegAndOrN);

    if (LogicAndOr == 1)
        _bRegAndOrN |= (LogicAndOr << 6);
    else if (LogicAndOr == 0)
        _bRegAndOrN &= 0xBF;
    else
        return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_SNIFFTH_C, _bRegAndOrN);
    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** M_DRV_MC36XX_SetSniffDeltaMode
 *****************************************************************************/
int M_DRV_MC36XX_SetSniffDeltaMode(const struct device *dev, e_m_drv_mc36xx_delta_mode_t Mode)
{
    LOG_DBG("[%s]\r\n", "Set Sniff Delta Mode");

    uint_dev _bRegDeltaMode = 0x00;

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_SNIFFTH_C, &_bRegDeltaMode);

    /** mode(0) is C2P, mode(1) is C2B*/
    if (Mode == 1)
        _bRegDeltaMode |= 0x80;
    else if (Mode == 0)
        _bRegDeltaMode &= 0x7F;
    else
        return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_SNIFFTH_C, _bRegDeltaMode);
    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** M_DRV_MC36XX_EnableFIFO
 *****************************************************************************/
int M_DRV_MC36XX_EnableFIFO(const struct device *dev, e_m_drv_mc36xx_fifo_ctl_t eCtrl,
                            e_m_drv_mc36xx_fifo_mode_t eMode,
                            uint8_t bThreshold)
{
    LOG_DBG("[%s]\r\n", "Enable FIFO");

    uint_dev _bPreMode = 0;

    if (eCtrl >= E_M_DRV_MC36XX_FIFO_CTL_END)
        return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

    if (eMode >= E_M_DRV_MC36XX_FIFO_MODE_END)
        return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

    if (bThreshold > (M_DRV_MC36XX_FIFO_DEPTH - 1))
        bThreshold = (M_DRV_MC36XX_FIFO_DEPTH - 1);

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_MODE_C, &_bPreMode);
    _M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_STANDBY);

    uint_dev _bRegRANGE_C = 0;
    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_RANGE_C, &_bRegRANGE_C);

    if (E_M_DRV_MC36XX_RES_12BIT < _M_DRV_MC36XX_REG_RANGE_C_RES(_bRegRANGE_C))
        M_DRV_MC36XX_ConfigRegRngResCtrl(dev, (e_m_drv_mc36xx_range_t)_M_DRV_MC36XX_REG_RANGE_C_RANGE(_bRegRANGE_C),
                                         E_M_DRV_MC36XX_RES_12BIT);

    s_bCfgFifo = ((eCtrl << 6) | (eMode << 5) | (bThreshold));
    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_FIFO_C, s_bCfgFifo);
    _M_DRV_MC36XX_SetMode(dev, (e_m_drv_mc36xx_mode_t)
                                   _M_DRV_MC36XX_REG_MODE_C_MODE(_bPreMode));

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** M_DRV_MC36XX_ReadData
 *****************************************************************************/
int M_DRV_MC36XX_ReadData(
    const struct device *dev, float faOutput[M_DRV_MC36XX_FIFO_DEPTH][M_DRV_MC36XX_AXES_NUM],
    int nNumOfSample)
{
    int _nDataCount = 0;
    uint_dev _bRegStatus1 = 0;
    uint_dev _bRegFIFO_C = 0;

    if ((M_DRV_MC36XX_NULL_ADDR == faOutput) || (0 == nNumOfSample))
        return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

#ifdef M_DRV_MC36XX_OPERATE_MODE_WAKE_WHEN_READ
    _M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_STANDBY);
    _M_DRV_MC36XX_SetCwakeSR(dev, s_eSR_CWAKE);
    _M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_CWAKE);
#endif

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_STATUS_1, &_bRegStatus1);
    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_FIFO_C, &_bRegFIFO_C);

    /** FIFO Mode */
    if (_M_DRV_MC36XX_REG_FIFO_C_FIFO_EN(_bRegFIFO_C))
    {
        if (M_DRV_MC36XX_FIFO_DEPTH < nNumOfSample)
            nNumOfSample = M_DRV_MC36XX_FIFO_DEPTH;

        LOG_DBG("[%s] FIFO mode read data\r\n", "REad Data");

        if (_M_DRV_MC36XX_REG_STATUS_1_FIFO_EMPTY(_bRegStatus1))
            return (M_DRV_MC36XX_RETCODE_ERROR_NO_DATA);

        for (_nDataCount = 0; _nDataCount < nNumOfSample; _nDataCount++)
        {
            _M_DRV_MC36XX_ReadData(dev, faOutput[_nDataCount]);
            _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_STATUS_1,
                                   &_bRegStatus1);

            if (_M_DRV_MC36XX_REG_STATUS_1_FIFO_EMPTY(_bRegStatus1))
            {
                _nDataCount++;
                break;
            }
        }
    }
    /** Normal Mode */
    else
    {
        LOG_DBG("[%s] Normal mode read data", "REad Data");
        /**
         * 0: No new sample data has arrived since last read.
         * 1: New sample data has arrived and has been written to FIFO/registers
         */
        if (!_M_DRV_MC36XX_REG_STATUS_1_NEW_DATA(_bRegStatus1))
            return (M_DRV_MC36XX_RETCODE_ERROR_NO_DATA);

        _M_DRV_MC36XX_ReadData(dev, faOutput[0]);
        _nDataCount = 1;
    }

    return (_nDataCount);
}

/******************************************************************************
 *** M_DRV_MC36XX_ReadRawData
 *****************************************************************************/
int M_DRV_MC36XX_ReadRawData(
    const struct device *dev, short Output[M_DRV_MC36XX_FIFO_DEPTH][M_DRV_MC36XX_AXES_NUM],
    int nNumOfSample)
{

    int _nDataCount = 0;
    uint_dev _bRegStatus1 = 0;
    uint_dev _bRegFIFO_C = 0;

    if ((M_DRV_MC36XX_NULL_ADDR == Output) || (0 == nNumOfSample))
        return (M_DRV_MC36XX_RETCODE_ERROR_WRONG_ARGUMENT);

#ifdef M_DRV_MC36XX_OPERATE_MODE_WAKE_WHEN_READ
    _M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_STANDBY);
    _M_DRV_MC36XX_SetCwakeSR(dev, s_eSR_CWAKE);
    _M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_CWAKE);
#endif

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_STATUS_1, &_bRegStatus1);
    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_FIFO_C, &_bRegFIFO_C);

    /** FIFO Mode */
    if (_M_DRV_MC36XX_REG_FIFO_C_FIFO_EN(_bRegFIFO_C))
    {
        if (M_DRV_MC36XX_FIFO_DEPTH < nNumOfSample)
            nNumOfSample = M_DRV_MC36XX_FIFO_DEPTH;

        LOG_DBG("[%s] FIFO mode read data\r\n", "Read raw data");

        if (_M_DRV_MC36XX_REG_STATUS_1_FIFO_EMPTY(_bRegStatus1))
            return (M_DRV_MC36XX_RETCODE_ERROR_NO_DATA);

        for (_nDataCount = 0; _nDataCount < nNumOfSample; _nDataCount++)
        {
            _M_DRV_MC36XX_ReadRawData(dev, Output[_nDataCount]);
            _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_STATUS_1,
                                   &_bRegStatus1);

            if (_M_DRV_MC36XX_REG_STATUS_1_FIFO_EMPTY(_bRegStatus1))
            {
                _nDataCount++;
                break;
            }
        }
    }
    /** Normal Mode */
    else
    {
        LOG_DBG("[%s] Normal mode read data", "Read raw data");
        /**
         * 0: No new sample data has arrived since last read.
         * 1: New sample data has arrived and has been written to FIFO/registers
         */
        if (!_M_DRV_MC36XX_REG_STATUS_1_NEW_DATA(_bRegStatus1))
            return (M_DRV_MC36XX_RETCODE_ERROR_NO_DATA);

        _M_DRV_MC36XX_ReadRawData(dev, Output[0]);
        _nDataCount = 1;
    }

    return (_nDataCount);
}

/******************************************************************************
 *** M_DRV_MC36XX_ConfigINT
 *****************************************************************************/
int M_DRV_MC36XX_ConfigINT(const struct device *dev, uint8_t bFifoThreshEnable,
                           uint8_t bFifoFullEnable,
                           uint8_t bFifoEmptyEnable,
                           uint8_t bACQEnable,
                           uint8_t bWakeEnable)
{
    LOG_DBG("[%s]\r\n", "Config INT");

    uint_dev _bPreMode = 0;

    _M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_MODE_C, &_bPreMode);
    _M_DRV_MC36XX_SetMode(dev, E_M_DRV_MC36XX_MODE_STANDBY);

    s_bCfgINT = (((bFifoThreshEnable & 0x01) << 6) | ((bFifoFullEnable & 0x01) << 5) | ((bFifoEmptyEnable & 0x01) << 4) | ((bACQEnable & 0x01) << 3) | ((bWakeEnable & 0x01) << 2) | M_DRV_MC36XX_CFG_INT_MODE_DEFAULT);

    _M_DRV_MC36XX_REG_WRITE(dev, E_M_DRV_MC36XX_REG_INTR_C, s_bCfgINT);
    _M_DRV_MC36XX_SetMode(dev, (e_m_drv_mc36xx_mode_t)
                                   _M_DRV_MC36XX_REG_MODE_C_MODE(_bPreMode));

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** M_DRV_MC36XX_HandleINT
 *****************************************************************************/
int M_DRV_MC36XX_HandleINT(const struct device *dev, s_m_drv_mc36xx_int_t *ptINT_Event)
{
    uint_dev _bRegStatus2 = 0;

    if (_M_DRV_MC36XX_REG_READ(dev, E_M_DRV_MC36XX_REG_STATUS_2, &_bRegStatus2) < 0)
    {
        LOG_ERR("Could not read Reg Status 2 in Interrupt.\n");
    }

    ptINT_Event->bWAKE =
        _M_DRV_MC36XX_REG_STATUS_2_INT_WAKE(_bRegStatus2);
    ptINT_Event->bACQ =
        _M_DRV_MC36XX_REG_STATUS_2_INT_ACQ(_bRegStatus2);
    ptINT_Event->bFIFO_EMPTY =
        _M_DRV_MC36XX_REG_STATUS_2_INT_FIFO_EMPTY(_bRegStatus2);
    ptINT_Event->bFIFO_FULL =
        _M_DRV_MC36XX_REG_STATUS_2_INT_FIFO_FULL(_bRegStatus2);
    ptINT_Event->bFIFO_THRESHOLD =
        _M_DRV_MC36XX_REG_STATUS_2_INT_FIFO_THRESH(_bRegStatus2);
    ptINT_Event->bSWAKE_SNIFF =
        _M_DRV_MC36XX_REG_STATUS_2_INT_SWAKE_SNIFF(_bRegStatus2);

    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** M_DRV_MC36XX_ReadReg
 *****************************************************************************/
uint8_t M_DRV_MC36XX_ReadReg(const struct device *dev, uint8_t bRegAddr)
{
    uint_dev _bData = 0;

    _M_DRV_MC36XX_REG_READ(dev, bRegAddr, &_bData);

    LOG_DBG("[%s] REG[0x%02X] 0x%02X\r\n", "Read Reg", bRegAddr, _bData);
    return (_bData);
}

/******************************************************************************
 *** M_DRV_MC36XX_ReadRegMap
 *****************************************************************************/
int M_DRV_MC36XX_ReadRegMap(const struct device *dev, uint8_t baRegMap[M_DRV_MC36XX_REG_MAP_SIZE])
{
    uint8_t _bIndex = 0;
    uint_dev _bRegData = 0;

    for (_bIndex = 0; _bIndex < M_DRV_MC36XX_REG_MAP_SIZE; _bIndex++)
    {
        _M_DRV_MC36XX_REG_READ(dev, _bIndex, &_bRegData);
        LOG_DBG("REG[0x%02X] 0x%02X\r\n", _bIndex, _bRegData);

        if (0 != baRegMap)
            baRegMap[_bIndex] = _bRegData;
    }
    return (M_DRV_MC36XX_RETCODE_SUCCESS);
}

/******************************************************************************
 *** API FUNCTIONS
 *****************************************************************************/
static void mc3672_convert_accel(struct sensor_value *val, float raw_val)
{
    double conv_val;
    conv_val = ((double)raw_val);
    sensor_value_from_double(val, conv_val);
    //val->val1 = conv_val / 1000000;
    //val->val2 = conv_val % 1000000;
}

static int mc3672_sample_fetch(const struct device *dev,
                               enum sensor_channel chan)
{
    struct mc3672_data *drv_data = dev->data;
    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

    float faAccels[M_DRV_MC36XX_FIFO_DEPTH][M_DRV_MC36XX_AXES_NUM];

    M_DRV_MC36XX_ReadData(drv_data->i2c, faAccels, M_DRV_MC36XX_FIFO_DEPTH);

    uint_dev _bRegFIFO_C = 0;
    _M_DRV_MC36XX_REG_READ(drv_data->i2c, E_M_DRV_MC36XX_REG_FIFO_C, &_bRegFIFO_C);

    /* FIFO Mode */
    if (_M_DRV_MC36XX_REG_FIFO_C_FIFO_EN(_bRegFIFO_C))
    {
        for (int i = 0; i < M_DRV_MC36XX_FIFO_DEPTH; i++)
        {
            //memcpy(drv_data->accel_x_fifo, &faAccels[0][M_DRV_MC36XX_AXIS_X], M_DRV_MC36XX_FIFO_DEPTH*sizeof(float));
            //memcpy(drv_data->accel_y_fifo, &faAccels[0][M_DRV_MC36XX_AXIS_Y], M_DRV_MC36XX_FIFO_DEPTH*sizeof(float));
            //memcpy(drv_data->accel_z_fifo, &faAccels[0][M_DRV_MC36XX_AXIS_Z], M_DRV_MC36XX_FIFO_DEPTH*sizeof(float));
            drv_data->accel_x_fifo[i] = faAccels[i][M_DRV_MC36XX_AXIS_X];
            drv_data->accel_y_fifo[i] = faAccels[i][M_DRV_MC36XX_AXIS_Y];
            drv_data->accel_z_fifo[i] = faAccels[i][M_DRV_MC36XX_AXIS_Z];
        }
    }
    /* Normal Mode */
    else
    {
        drv_data->accel_x = faAccels[0][M_DRV_MC36XX_AXIS_X];
        drv_data->accel_y = faAccels[0][M_DRV_MC36XX_AXIS_Y];
        drv_data->accel_z = faAccels[0][M_DRV_MC36XX_AXIS_Z];
    }

    return 0;
}

static int mc3672_channel_get(const struct device *dev,
                              enum sensor_channel chan,
                              struct sensor_value *val)
{
    struct mc3672_data *drv_data = dev->data;

    uint_dev _bRegFIFO_C = 0;
    _M_DRV_MC36XX_REG_READ(drv_data->i2c, E_M_DRV_MC36XX_REG_FIFO_C, &_bRegFIFO_C);

    switch (chan)
    {
    case SENSOR_CHAN_ACCEL_XYZ:
        /* FIFO Mode */
        if (_M_DRV_MC36XX_REG_FIFO_C_FIFO_EN(_bRegFIFO_C))
        {
            LOG_ERR("Chan Accel XYZ not supported in FIFO mode.");
            return -EINVAL;
        }
        /* Normal Mode */
        else
        {
            mc3672_convert_accel(val, drv_data->accel_x);
            mc3672_convert_accel(val + 1, drv_data->accel_y);
            mc3672_convert_accel(val + 2, drv_data->accel_z);
        }
        break;
    case SENSOR_CHAN_ACCEL_X:
        /* FIFO Mode */
        if (_M_DRV_MC36XX_REG_FIFO_C_FIFO_EN(_bRegFIFO_C))
        {
            for (int i = 0; i < M_DRV_MC36XX_FIFO_DEPTH; i++)
            {
                mc3672_convert_accel(val + i, drv_data->accel_x_fifo[i]);
            }
        }
        /* Normal Mode */
        else
        {
            mc3672_convert_accel(val, drv_data->accel_x);
        }
        break;
    case SENSOR_CHAN_ACCEL_Y:
        /* FIFO Mode */
        if (_M_DRV_MC36XX_REG_FIFO_C_FIFO_EN(_bRegFIFO_C))
        {
            for (int i = 0; i < M_DRV_MC36XX_FIFO_DEPTH; i++)
            {
                mc3672_convert_accel(val + i, drv_data->accel_y_fifo[i]);
            }
        }
        /* Normal Mode */
        else
        {
            mc3672_convert_accel(val, drv_data->accel_y);
        }
        break;
    case SENSOR_CHAN_ACCEL_Z:
        /* FIFO Mode */
        if (_M_DRV_MC36XX_REG_FIFO_C_FIFO_EN(_bRegFIFO_C))
        {
            for (int i = 0; i < M_DRV_MC36XX_FIFO_DEPTH; i++)
            {
                mc3672_convert_accel(val + i, drv_data->accel_z_fifo[i]);
            }
        }
        /* Normal Mode */
        else
        {
            mc3672_convert_accel(val, drv_data->accel_z);
        }
        break;
    default:
        LOG_ERR("Unsupported Channel");
        return -EINVAL;
    }
    return 0;
}

static const struct sensor_driver_api mc3672_driver_api = {
#if CONFIG_MC3672_TRIGGER
    .trigger_set = mc3672_trigger_set,
#endif
    .sample_fetch = mc3672_sample_fetch,
    .channel_get = mc3672_channel_get,
};

static int mc3672_init(const struct device *dev)
{
    struct mc3672_data *drv_data = dev->data;
    const struct mc3672_config *cfg = dev->config;
    //uint8_t tmp = 0;

    drv_data->i2c = device_get_binding(cfg->i2c_label);

    /* GET THE I2C BUS */
    if (drv_data->i2c == NULL)
    {
        LOG_ERR("Failed to get pointer to %s device!",
                cfg->i2c_label);
        return -EINVAL;
    }

    if (M_DRV_MC36XX_RETCODE_SUCCESS != _M_DRV_MC36XX_ValidateSensorIC(drv_data->i2c))
        return (M_DRV_MC36XX_RETCODE_ERROR_IDENTIFICATION);

    // RESET THE CHIP
    if (_M_DRV_MC36XX_ResetChip(drv_data->i2c) < 0)
    {
        LOG_ERR("Failed to reset device.\n");
        return -EINVAL;
    }

    /** Config Bus Type either SPI or I2C */
    if (_M_DRV_MC36XX_SetBusIF(drv_data->i2c) < 0)
    {
        LOG_ERR("Failed to set bus to i2c.\n");
        return -EINVAL;
    }
    _M_DRV_MC36XX_REG_WRITE(drv_data->i2c, E_M_DRV_MC36XX_REG_PWR_CONTROL, s_bCfgFifoVdd);

    /** Config Range and Resolution */
    if (M_DRV_MC36XX_ConfigRegRngResCtrl(drv_data->i2c, s_eRange, s_eRes) < 0)
    {
        LOG_ERR("Failed to config range and resolution.\n");
        return -EINVAL;
    }

    /** Config Sniff and CWake Sample Rate */
    if (_M_DRV_MC36XX_SetSniffSR(drv_data->i2c, s_eSR_SNIFF) < 0)
    {
        LOG_ERR("Failed to set Sniff Sample Rate.\n");
        return -EINVAL;
    }
    if (_M_DRV_MC36XX_SetCwakeSR(drv_data->i2c, s_eSR_CWAKE) < 0)
    {
        LOG_ERR("Failed to set CWake Sample Rate.\n");
        return -EINVAL;
    }

    /* Config Sniff and CWake Analog Gain */
    if (_M_DRV_MC36XX_SetWakeAGAIN(drv_data->i2c, s_eGAIN_WAKE) < 0)
    {
        LOG_ERR("Failed to set CWake Analog Gain.\n");
        return -EINVAL;
    }
    if (_M_DRV_MC36XX_SetSniffAGAIN(drv_data->i2c, s_eGAIN_SNIFF) < 0)
    {
        LOG_ERR("Failed to set Sniff Analog Gain.\n");
        return -EINVAL;
    }

#ifdef M_DRV_MC36XX_OPERATE_MODE_WAKE_WHEN_READ
    if (_M_DRV_MC36XX_SetMode(drv_data->i2c, E_M_DRV_MC36XX_MODE_SLEEP) < 0)
    {
        LOG_ERR("Failed to set Operate Mode Wake when read.\n");
        return -EINVAL;
    }
#endif

    _M_DRV_MC36XX_REG_READ(drv_data->i2c, E_M_DRV_MC36XX_REG_FIFO_C, &s_bCfgFifo);
    _M_DRV_MC36XX_REG_READ(drv_data->i2c, E_M_DRV_MC36XX_REG_INTR_C, &s_bCfgINT);

    LOG_DBG("Initialized the MC3672 !\n");
    return 0;
}

static struct mc3672_data mc3672_driver;
static const struct mc3672_config mc3672_cfg = {
    .i2c_label = DT_INST_BUS_LABEL(0),
    .i2c_addr = DT_INST_REG_ADDR(0),
#ifdef CONFIG_MC3672_TRIGGER
    .int_pin = DT_INST_GPIO_PIN(0, int_gpios),
    .int_flags = DT_INST_GPIO_FLAGS(0, int_gpios),
    .int_label = DT_INST_GPIO_LABEL(0, int_gpios),
#endif /* CONFIG_MC3672_TRIGGER */
};

DEVICE_DT_INST_DEFINE(0, mc3672_init, NULL,
                      &mc3672_driver, &mc3672_cfg,
                      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
                      &mc3672_driver_api);
