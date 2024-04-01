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

#ifndef STA350BW_DRIVER_H
#define STA350BW_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------- INCLUDES ---------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>

#include "sta350bw.h"

/*------------------------------------------- MACROS AND DEFINES -----------------------------------------------------*/

/* volume limits */
#define STA350BW_MVOL_MIN               (-127.0F)
#define STA350BW_MVOL_MAX               (0.0F)
#define STA350BW_CVOL_MIN               (-80.0F)
#define STA350BW_CVOL_MAX               (48.0F)

/*------------------------------------------- TYPEDEFS ---------------------------------------------------------------*/

typedef struct sta350bw_i2s_cfg_t
{
    sta350bw_i2s_sampleRate_t sampleRate;
    sta350bw_i2s_dataFirstBit_t dataFirstBit;
    sta350bw_i2s_delayBitEn_t delayBitEn;
    sta350bw_i2s_bitsPerFrame_t bitsPerFrame;
    sta350bw_i2s_format_t format;
    sta350bw_i2s_mclkSampRate_t mclkSampRate;
    sta350bw_i2s_dataBits_t dataBits;
} sta350bw_i2s_cfg_t;

typedef struct sta350bw_handle_t
{
    uint8_t deviceAddr;
    float masterVolume;
    sta350bw_i2s_cfg_t i2sCfg;
    sta350bw_status_t (*write)(uint8_t, uint8_t*, size_t);
    sta350bw_status_t (*read)(uint8_t, uint8_t*, size_t);
} sta350bw_handle_t;

/*------------------------------------------- EXPORTED VARIABLES -----------------------------------------------------*/
/*------------------------------------------- GLOBAL FUNCTION PROTOTYPES ---------------------------------------------*/

/**
 * @brief 	Initialise STA350BW Device
 * 
 * @param   H           Handle
 *
 * @return  status
 */
sta350bw_status_t sta350bw_init(
    sta350bw_handle_t *H);

/**
 * @brief 	Set Volume
 * 
 * Enables setting the master and channel volume levels. The master volume has a dynamic range of 0 dBFS to -127 dBFS,
 * in increments of -0.5 dB, a linear ramp applied to transition to the next level of a period of 4096 samples. A value
 * of -127.5 dB forces the amp into a 'hard mute' mode, which has no ramp. The channel volumes have a dynamic range of 
 * +48 dBFS down to -80 dBFS, where values above -60 dBFS have an increment of -0.5 dB, and below have an increment of
 * -1 dB. Again, the volume uses the same linear ramp to transistion to the next level, and the value of -80 dBFS
 * results in a 'hard mute'.
 * 
 * @param   H           Handle
 * @param   channel     Channel
 * @param   volume      Volume in dB
 *
 * @return  status
 */
sta350bw_status_t sta350bw_setVolume(
    sta350bw_handle_t *H,
    sta350bw_channelVol_t channel, 
    float volume);

#ifdef __cplusplus
}
#endif

#endif /* STA350BW_DRIVER_H */