/***********************************************************************************************************************
 * 
 * @file    sta350bw.h
 *
 * @brief   STA350BW Driver
 * 
 * 2.1-channel high-efficiency digital audio system Sound Terminal. Modified version of the ST's original header.
 *  
 * @par
 * @author 	G. Howell
 * 
 **********************************************************************************************************************/

#ifndef STA350BW_H
#define STA350BW_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------- INCLUDES ---------------------------------------------------------------*/

#include <stdint.h>

/*------------------------------------------- MACROS AND DEFINES -----------------------------------------------------*/

/* i2c device addresses */
#define STA350BW_DEV_ADDR_1                     ((uint8_t)0x38)
#define STA350BW_DEV_ADDR_2                     ((uint8_t)0x3A)

/* registers */
#define STA350BW_REG_CONF_A                     ((uint8_t)0x00)      /* Configuration A */
#define STA350BW_REG_CONF_B                     ((uint8_t)0x01)      /* Configuration B */
#define STA350BW_REG_CONF_F                     ((uint8_t)0x05)      /* Configuration F */
#define STA350BW_REG_MUTE                       ((uint8_t)0x06)      /* MUTE / Lineout configuration */
#define STA350BW_REG_MVOL                       ((uint8_t)0x07)      /* Master Volume */
#define STA350BW_REG_C1VOL                      ((uint8_t)0x08)      /* Channel 1 volume */
#define STA350BW_REG_C2VOL                      ((uint8_t)0x09)      /* Channel 2 volume */
#define STA350BW_REG_C3VOL	                    ((uint8_t)0x0A)      /* Channel 3 volume */
#define STA350BW_REG_STATUS                     ((uint8_t)0x2D)      /* Status */

/** config register b mask bits */
#define STA350BW_CONF_REGB_SAIX_MASK            ((uint8_t)0x0F)
#define STA350BW_CONF_REGB_SAIFB_MASK           ((uint8_t)0x10)
#define STA350BW_CONF_REGB_DSCKE_MASK           ((uint8_t)0x20)
#define STA350BW_CONF_REGB_C1IM_MASK            ((uint8_t)0xC0)

/** master clock oversampling rate for 32 kHz, 44.1 kHz and 48 kHz sample rates */
#define STA350BW_MCLK_FS1_X_576                 ((uint8_t)0x5)
#define STA350BW_MCLK_FS1_X_128                 ((uint8_t)0x4)
#define STA350BW_MCLK_FS1_X_256                 ((uint8_t)0x3)
#define STA350BW_MCLK_FS1_X_384                 ((uint8_t)0x2)
#define STA350BW_MCLK_FS1_X_512                 ((uint8_t)0x1)
#define STA350BW_MCLK_FS1_X_768                 ((uint8_t)0x0)

/** master clock oversampling rate for 88.2 kHz and 96 kHz sample rates */
#define STA350BW_MCLK_FS2_X_64                  ((uint8_t)0xC)
#define STA350BW_MCLK_FS2_X_128                 ((uint8_t)0xB)
#define STA350BW_MCLK_FS2_X_192                 ((uint8_t)0xA)
#define STA350BW_MCLK_FS2_X_256                 ((uint8_t)0x9)
#define STA350BW_MCLK_FS2_X_384                 ((uint8_t)0x8)

/** master clock oversampling rate for 176.4 kHz and 192 kHz sample rates */
#define STA350BW_MCLK_FS3_X_32                  ((uint8_t)0x14)
#define STA350BW_MCLK_FS3_X_64                  ((uint8_t)0x13)
#define STA350BW_MCLK_FS3_X_96                  ((uint8_t)0x12)
#define STA350BW_MCLK_FS3_X_128                 ((uint8_t)0x11)
#define STA350BW_MCLK_FS3_X_192                 ((uint8_t)0x10)

/** serial data first */
#define STA350BW_I2S_SDF_MSB                    ((uint8_t)0x00)
#define STA350BW_I2S_SDF_LSB                    ((uint8_t)0x10)

/** serial data format - TODO (for MSB first only atm) */
#define STA350BW_I2S_FMT_TYPE_I2S               ((uint8_t)0x0)
#define STA350BW_I2S_FMT_TYPE_LJ                ((uint8_t)0x1)
#define STA350BW_I2S_FMT_TYPE_RJ_24BITS         ((uint8_t)0x2)
#define STA350BW_I2S_FMT_TYPE_RJ_20BITS         ((uint8_t)0x6)
#define STA350BW_I2S_FMT_TYPE_RJ_18BITS         ((uint8_t)0x9)
#define STA350BW_I2S_FMT_TYPE_RJ_16BITS         ((uint8_t)0x13)

/** delay serial clock enable */
#define STA350BW_I2S_DELAY_BIT_DIS              ((uint8_t)0x00)
#define STA350BW_I2S_DELAY_BIT_EN               ((uint8_t)0x20)

/*------------------------------------------- TYPEDEFS ---------------------------------------------------------------*/

/** master clock oversample rate */
typedef enum
{
    STA350BW_I2S_MCLK_FS_X_32           = 32,
    STA350BW_I2S_MCLK_FS_X_64           = 64,
    STA350BW_I2S_MCLK_FS_X_128          = 128,
    STA350BW_I2S_MCLK_FS_X_192          = 192,
    STA350BW_I2S_MCLK_FS_X_256          = 256,
    STA350BW_I2S_MCLK_FS_X_384          = 384,
    STA350BW_I2S_MCLK_FS_X_512          = 512,
    STA350BW_I2S_MCLK_FS_X_576          = 512,
    STA350BW_I2S_MCLK_FS_X_768          = 768,
} sta350bw_i2s_mclkSampRate_t;

/* i2s sample rates  */
typedef enum
{
    STA350BW_I2S_FS_32000               = 32000,
    STA350BW_I2S_FS_44100               = 44100,
    STA350BW_I2S_FS_48000               = 48000,
    STA350BW_I2S_FS_88200               = 88200,
    STA350BW_I2S_FS_96000               = 96000,
    STA350BW_I2S_FS_176400              = 176400,
    STA350BW_I2S_FS_192000              = 192000
} sta350bw_i2s_sampleRate_t;

/** i2s data first bit */
typedef enum 
{
    STA350BW_I2S_MSB_FIRST,
    STA350BW_I2S_LSB_FIRST
} sta350bw_i2s_dataFirstBit_t;

/** i2s delay serial clock bit */
typedef enum 
{
    STA350BW_I2S_DELAY_BIT_DISABLE,
    STA350BW_I2S_DELAY_BIT_ENABLE
} sta350bw_i2s_delayBitEn_t;

/** i2s bits per frame */
typedef enum
{
    STA350BW_I2S_32_BITS_PER_FRAME      = 32,
    STA350BW_I2S_48_BITS_PER_FRAME      = 48,
    STA350BW_I2S_64_BITS_PER_FRAME      = 64
} sta350bw_i2s_bitsPerFrame_t;

/** i2s format type */
typedef enum
{
    STA350BW_I2S_FMT_I2S,
    STA350BW_I2S_FMT_LEFT_JUSTIFIED,
    STA350BW_I2S_FMT_RIGHT_JUSTIFIED
} sta350bw_i2s_format_t;

/** i2s number of data bits per frame */
typedef enum
{
    STA350BW_I2S_DATA_BITS_16           = 16,
    STA350BW_I2S_DATA_BITS_18           = 18,
    STA350BW_I2S_DATA_BITS_20           = 20,
    STA350BW_I2S_DATA_BITS_24           = 24
} sta350bw_i2s_dataBits_t;

/** channel volumes */
typedef enum
{
    STA350BW_VOL_MASTER                 = STA350BW_REG_MVOL,
    STA350BW_VOL_CHANNEL_1              = STA350BW_REG_C1VOL,
    STA350BW_VOL_CHANNEL_2              = STA350BW_REG_C2VOL,
    STA350BW_VOL_CHANNEL_3              = STA350BW_REG_C3VOL,
} sta350bw_channelVol_t;

/** sta350bw status definition */
typedef enum
{
    STA350BW_STATUS_ERROR               = -1,
    STA350BW_STATUS_OK                  = 0,
    STA350BW_STATUS_I2C_ERROR           = 1
} sta350bw_status_t;

/*------------------------------------------- EXPORTED VARIABLES -----------------------------------------------------*/
/*------------------------------------------- GLOBAL FUNCTION PROTOTYPES ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* STA350BW_H */
