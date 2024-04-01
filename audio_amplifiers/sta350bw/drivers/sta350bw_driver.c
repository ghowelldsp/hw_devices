/***********************************************************************************************************************
 * 
 * @file    sta350bw.c
 *
 * @brief   STA350BW Driver
 * 
 * 2.1-channel high-efficiency digital audio system Sound Terminal
 *  
 * @par
 * @author	G. Howell
 * 
 **********************************************************************************************************************/

/*------------------------------------------- INCLUDES ---------------------------------------------------------------*/

#include <string.h>
#include <math.h>

#include "sta350bw_driver.h"

/*------------------------------------------- EXTERN VARIABLES -------------------------------------------------------*/
/*------------------------------------------- PRIVATE MACROS AND DEFINES ---------------------------------------------*/
/*------------------------------------------- PRIVATE TYPEDEFS -------------------------------------------------------*/
/*------------------------------------------- STATIC VARIABLES -------------------------------------------------------*/
/*------------------------------------------- GLOBAL VARIABLES -------------------------------------------------------*/
/*------------------------------------------- STATIC FUNCTION PROTOTYPES ---------------------------------------------*/

static sta350bw_status_t write(
    sta350bw_handle_t *H,
    uint8_t regAddr,
    uint8_t *pData,
    size_t nBytes);

static sta350bw_status_t read(
    sta350bw_handle_t *H,
    uint8_t regAddr,
    uint8_t *pData,
    size_t nBytes);

static sta350bw_status_t setMClk(
    sta350bw_handle_t *H);

static sta350bw_status_t setI2sCfg(
    sta350bw_handle_t *H);

static sta350bw_status_t readStatus(
    sta350bw_handle_t *H);

static sta350bw_status_t enablePower(
    sta350bw_handle_t *H);

/*------------------------------------------- STATIC FUNCTIONS -------------------------------------------------------*/

/** write function */
static sta350bw_status_t write(
    sta350bw_handle_t *H,
    uint8_t regAddr,
    uint8_t *pData,
    size_t nBytes)
{
    sta350bw_status_t ret;
    uint8_t *pTmp;

    // create array to hold both the register address and the data
    pTmp = (uint8_t*)calloc(nBytes+1, sizeof(uint8_t));

    // write reg address followed by all the data
    *pTmp = regAddr;
    memcpy(pTmp+1, pData, nBytes);

    // write subaddress
    if ((ret = H->write(H->deviceAddr, &regAddr, 1)) != STA350BW_STATUS_OK)
    {
        free(pTmp);
        return ret;
    }

    free(pTmp);

    return ret;
}

/** read function */
static sta350bw_status_t read(
    sta350bw_handle_t *H,
    uint8_t regAddr,
    uint8_t *pData,
    size_t nBytes)
{
    sta350bw_status_t ret;

    // write subaddress
    if ((ret = H->write(H->deviceAddr, &regAddr, 1)) != STA350BW_STATUS_OK)
    {
        return ret;
    }

    // read data from subaddress
    if ((ret = H->read(H->deviceAddr, pData, nBytes)) != STA350BW_STATUS_OK) 
    {
        return ret;
    }

    return ret;
}

/** set sample rate */
static sta350bw_status_t setMClk(
    sta350bw_handle_t *H) 
{
    uint8_t regVal;
    uint32_t sampleRate;
    sta350bw_i2s_mclkSampRate_t mclkSampRate;
    sta350bw_status_t ret;

    if ((ret = read(H, STA350BW_REG_CONF_A, &regVal, 1)) != STA350BW_STATUS_OK) 
    {
        return ret;
    }

    sampleRate = H->i2sCfg.sampleRate;
    mclkSampRate = H->i2sCfg.mclkSampRate;

    // clear MCS and IR registers
    regVal &= ~0x1F;

    if (sampleRate == STA350BW_I2S_FS_32000 || 
        sampleRate == STA350BW_I2S_FS_44100 || 
        sampleRate == STA350BW_I2S_FS_48000)
    {
        if      (mclkSampRate == STA350BW_I2S_MCLK_FS_X_576)    { regVal |= STA350BW_MCLK_FS1_X_576; }
        else if (mclkSampRate == STA350BW_I2S_MCLK_FS_X_128)    { regVal |= STA350BW_MCLK_FS1_X_128; }
        else if (mclkSampRate == STA350BW_I2S_MCLK_FS_X_256)    { regVal |= STA350BW_MCLK_FS1_X_256; }
        else if (mclkSampRate == STA350BW_I2S_MCLK_FS_X_384)    { regVal |= STA350BW_MCLK_FS1_X_384; }
        else if (mclkSampRate == STA350BW_I2S_MCLK_FS_X_512)    { regVal |= STA350BW_MCLK_FS1_X_512; }
        else if (mclkSampRate == STA350BW_I2S_MCLK_FS_X_768)    { regVal |= STA350BW_MCLK_FS1_X_768; }
        else                                                    { return STA350BW_STATUS_ERROR; }
    } 
    else if (sampleRate == STA350BW_I2S_FS_88200 || 
             sampleRate == STA350BW_I2S_FS_96000) 
    {
        // TODO
    } 
    else if (sampleRate == STA350BW_I2S_FS_176400 ||
             sampleRate == STA350BW_I2S_FS_192000)
    {
        // TODO
    }
    else 
    {
        return STA350BW_STATUS_ERROR;
    }

    if ((ret = write(H, STA350BW_REG_CONF_A, &regVal, 1)) != STA350BW_STATUS_OK) 
    {
        return ret;
    }

    return STA350BW_STATUS_OK;
}

/** set i2s mode */
static sta350bw_status_t setI2sCfg(
    sta350bw_handle_t *H)
{
    uint8_t regVal = 0;
    sta350bw_status_t ret = STA350BW_STATUS_OK;
    sta350bw_i2s_dataBits_t dataBits;
    sta350bw_i2s_dataFirstBit_t dataFirstBit;
    sta350bw_i2s_format_t format;
    sta350bw_i2s_delayBitEn_t delayBitEn;

    // read register
    if ((ret = read(H, STA350BW_REG_CONF_B, &regVal, 1)) != STA350BW_STATUS_OK)
    {
        return ret;
    }

    // get variable from handle
    dataFirstBit = H->i2sCfg.dataFirstBit;
    dataBits = H->i2sCfg.dataBits;
    format = H->i2sCfg.format;
    delayBitEn = H->i2sCfg.delayBitEn;

    // clear the SAIX, SAIFB and DSCKE bits
    regVal &= STA350BW_CONF_REGB_C1IM_MASK;

    // write serial data first bits
    if      (dataFirstBit == STA350BW_I2S_MSB_FIRST)    { regVal |= STA350BW_I2S_SDF_MSB; }
    else if (dataFirstBit == STA350BW_I2S_LSB_FIRST)    { regVal |= STA350BW_I2S_SDF_LSB; }
    else                                                { return STA350BW_STATUS_ERROR; }

    // write serial data audio input bits
    if      (format == STA350BW_I2S_FMT_I2S)                { regVal |= STA350BW_I2S_FMT_TYPE_I2S; }
    else if (format == STA350BW_I2S_FMT_LEFT_JUSTIFIED)     { regVal |= STA350BW_I2S_FMT_TYPE_LJ; }
    else if (format == STA350BW_I2S_FMT_RIGHT_JUSTIFIED)
    {
        if      (dataBits == STA350BW_I2S_DATA_BITS_24)     { regVal |= STA350BW_I2S_FMT_TYPE_RJ_24BITS; }
        else if (dataBits == STA350BW_I2S_DATA_BITS_20)     { regVal |= STA350BW_I2S_FMT_TYPE_RJ_20BITS; }
        else if (dataBits == STA350BW_I2S_DATA_BITS_18)     { regVal |= STA350BW_I2S_FMT_TYPE_RJ_18BITS; }
        else if (dataBits == STA350BW_I2S_DATA_BITS_16)     { regVal |= STA350BW_I2S_FMT_TYPE_RJ_16BITS; }
        else                                                { return STA350BW_STATUS_ERROR; } 
    }
    else { return STA350BW_STATUS_ERROR; }

    // write delay serial clock enable bits
    if      (delayBitEn == STA350BW_I2S_DELAY_BIT_DISABLE)  { regVal |= STA350BW_I2S_DELAY_BIT_DIS; }
    else if (delayBitEn == STA350BW_I2S_DELAY_BIT_ENABLE)   { regVal |= STA350BW_I2S_DELAY_BIT_EN; }
    else                                                    { return STA350BW_STATUS_ERROR; } 

    // write register
    if ((ret = write(H, STA350BW_REG_CONF_B, &regVal, 1)) != STA350BW_STATUS_OK)
    {
        return ret;
    }

    return ret;
}

/** read status */
static sta350bw_status_t readStatus(
    sta350bw_handle_t *H)
{
    uint8_t regVal = 0;
    sta350bw_status_t ret = STA350BW_STATUS_OK;

    // read reg
    if ((ret = read(H, STA350BW_REG_STATUS, &regVal, 1)) != STA350BW_STATUS_OK) 
    {
        return ret;                          
    }
    
    // status register highlights undesired behaviour (PLL not locked, ...)
    // TODO
    if (regVal != 0x7F) 
    {
        return STA350BW_STATUS_ERROR;
    }

    return ret;
}

/** enable power out */
static sta350bw_status_t enablePower(
    sta350bw_handle_t *H)
{
    uint8_t tmp = 0;
    sta350bw_status_t ret;

    if ((ret = read(H, STA350BW_REG_CONF_F, &tmp, 1)) != STA350BW_STATUS_OK) 
    {
        return ret;
    }
    
    // TODO
    tmp &= ~0x80;
    tmp |= 0x80;
    
    /* enable power out stage */
    if ((ret = write(H, STA350BW_REG_CONF_F, &tmp, 1)) != STA350BW_STATUS_OK) 
    {
        return ret;
    }

    return ret;
}

/*------------------------------------------- GLOBAL FUNCTIONS -------------------------------------------------------*/

/** init */
sta350bw_status_t sta350bw_init(
    sta350bw_handle_t *H) 
{
    sta350bw_status_t ret;

    // sets master clock rate
    if ((ret = setMClk(H)) != STA350BW_STATUS_OK) { return ret; }
    
    // setup i2s
    if ((ret = setI2sCfg(H)) != STA350BW_STATUS_OK) { return ret; }

    // read status register
    if ((ret = readStatus(H)) != STA350BW_STATUS_OK) { return ret; }

    // setup master volume
    if ((ret = sta350bw_setVolume(H, STA350BW_VOL_MASTER, H->masterVolume)) != STA350BW_STATUS_OK)
    {
        return ret;
    }
    
    // enable power output
    if ((ret = enablePower(H)) != STA350BW_STATUS_OK) { return ret; }

    return ret;
}

/** set volume */
sta350bw_status_t sta350bw_setVolume(
    sta350bw_handle_t *H,
    sta350bw_channelVol_t channel, 
    float volume)
{
    uint8_t regVal;

    // check for null pointers
    if (NULL == H)
    {
        return STA350BW_STATUS_ERROR;
    }

    // check value limits
    if (channel == STA350BW_VOL_MASTER)
    {
        if ((volume < STA350BW_MVOL_MIN) || (volume > STA350BW_MVOL_MAX))
        {
            return STA350BW_STATUS_ERROR;
        }

        // convert decibel value to hex
        regVal = (uint8_t)(fabsf(2.0f * volume));
    }
    else
    {
        // check value limits
        if ((volume < STA350BW_CVOL_MIN) || (volume > STA350BW_CVOL_MAX))
        {
            return STA350BW_STATUS_ERROR;
        }

        // the values between 0 and -60 dB increment by 0.5, whereas the values below -60 dB only increment by 1 dB
        if (volume > -60.0f)
        {
            // convert decibel value to hex
            regVal = (uint8_t)(fabsf(2.0f * (volume - 48.0f)));
        }
        else
        {
            // convert decibel value to hex
            regVal = (uint8_t)(fabsf(volume + 60.0f)) + 0xd8;
        }
    }

    return write(H, channel, &regVal, 1);
}
