/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "fsl_ftm_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
void FTM_HAL_Init(uint32_t ftmBaseAddr)
{
    /* Use FTM mode */
    FTM_HAL_Enable(ftmBaseAddr, true);
    FTM_HAL_SetClockPs(ftmBaseAddr, kFtmDividedBy2);
}

void FTM_HAL_SetSyncMode(uint32_t ftmBaseAddr, uint32_t syncMethod)
{
    assert(syncMethod & (BM_FTM_SYNC_TRIG0 | BM_FTM_SYNC_TRIG1 | BM_FTM_SYNC_TRIG2 | BM_FTM_SYNC_SWSYNC));

    uint32_t channel = 0;

    /* Use the Enhanced PWM synchronization method */
    FTM_HAL_SetPwmSyncModeCmd(ftmBaseAddr, true);

    FTM_HAL_SetCntinPwmSyncModeCmd(ftmBaseAddr, true);
    FTM_HAL_SetInvctrlPwmSyncModeCmd(ftmBaseAddr, true);
    FTM_HAL_SetSwoctrlPwmSyncModeCmd(ftmBaseAddr, true);
    FTM_HAL_SetOutmaskPwmSyncModeCmd(ftmBaseAddr, true);

    for (channel = 0; channel < (FSL_FEATURE_FTM_CHANNEL_COUNT / 2); channel++)
    {
        FTM_HAL_SetDualChnPwmSyncCmd(ftmBaseAddr, channel, true);
    }
    if (syncMethod & BM_FTM_SYNC_SWSYNC)
    {
        /* Enable needed bits for software trigger to update registers with its buffer value */
        FTM_HAL_SetCounterSoftwareSyncModeCmd(ftmBaseAddr, true);
        FTM_HAL_SetModCntinCvSoftwareSyncModeCmd(ftmBaseAddr, true);
        FTM_HAL_SetInvctrlSoftwareSyncModeCmd(ftmBaseAddr, true);
        FTM_HAL_SetSwoctrlSoftwareSyncModeCmd(ftmBaseAddr, true);
        FTM_HAL_SetOutmaskSoftwareSyncModeCmd(ftmBaseAddr, true);
    }
    if (syncMethod & (BM_FTM_SYNC_TRIG0 | BM_FTM_SYNC_TRIG1 | BM_FTM_SYNC_TRIG2))
    {
        /* Enable needed bits for hardware trigger to update registers with its buffer value */
        FTM_HAL_SetCounterHardwareSyncModeCmd(ftmBaseAddr, true);
        FTM_HAL_SetModCntinCvHardwareSyncModeCmd(ftmBaseAddr, true);
        FTM_HAL_SetInvctrlHardwareSyncModeCmd(ftmBaseAddr, true);
        FTM_HAL_SetSwoctrlHardwareSyncModeCmd(ftmBaseAddr, true);
        FTM_HAL_SetOutmaskHardwareSyncModeCmd(ftmBaseAddr, true);
        if (syncMethod & BM_FTM_SYNC_TRIG0)
        {
            BW_FTM_SYNC_TRIG0(ftmBaseAddr, 1);
        }

        if (syncMethod & BM_FTM_SYNC_TRIG1)
        {
            BW_FTM_SYNC_TRIG1(ftmBaseAddr, 1);
        }
        if (syncMethod & BM_FTM_SYNC_TRIG2)
        {
            BW_FTM_SYNC_TRIG2(ftmBaseAddr, 1);
        }
    }
}

void FTM_HAL_EnablePwmMode(uint32_t ftmBaseAddr, ftm_pwm_param_t *config, uint8_t channel)
{
    uint8_t chnlPairnum = FTM_HAL_GetChnPairIndex(channel);

    FTM_HAL_SetDualEdgeCaptureCmd(ftmBaseAddr, chnlPairnum, false);
    FTM_HAL_SetChnEdgeLevel(ftmBaseAddr, channel, config->edgeMode ? 1 : 2);
    switch(config->mode)
    {
        case kFtmEdgeAlignedPWM:
            FTM_HAL_SetDualChnCombineCmd(ftmBaseAddr, chnlPairnum, false);
            FTM_HAL_SetCpwms(ftmBaseAddr, 0);
            FTM_HAL_SetChnMSnBAMode(ftmBaseAddr, channel, 2);
            break;
        case kFtmCenterAlignedPWM:
            FTM_HAL_SetDualChnCombineCmd(ftmBaseAddr, chnlPairnum, false);
            FTM_HAL_SetCpwms(ftmBaseAddr, 1);
            break;
        case kFtmCombinedPWM:
            FTM_HAL_SetCpwms(ftmBaseAddr, 0);
            FTM_HAL_SetDualChnCombineCmd(ftmBaseAddr, chnlPairnum, true);
            break;
        default:
            assert(0);
            break;
    }
}

void FTM_HAL_DisablePwmMode(uint32_t ftmBaseAddr, ftm_pwm_param_t *config, uint8_t channel)
{
    uint8_t chnlPairnum = FTM_HAL_GetChnPairIndex(channel);

    FTM_HAL_SetChnCountVal(ftmBaseAddr, channel, 0);
    FTM_HAL_SetChnEdgeLevel(ftmBaseAddr, channel, 0);
    FTM_HAL_SetChnMSnBAMode(ftmBaseAddr, channel, 0);
    FTM_HAL_SetCpwms(ftmBaseAddr, 0);
    FTM_HAL_SetDualChnCombineCmd(ftmBaseAddr, chnlPairnum, false);
}

void FTM_HAL_Reset(uint32_t ftmBaseAddr)
{
    HW_FTM_SC_WR(ftmBaseAddr, 0);
    HW_FTM_CNT_WR(ftmBaseAddr, 0);
    HW_FTM_MOD_WR(ftmBaseAddr, 0);

    HW_FTM_CNTIN_WR(ftmBaseAddr, 0);
    HW_FTM_STATUS_WR(ftmBaseAddr, 0);
    HW_FTM_MODE_WR(ftmBaseAddr, 0x00000004);
    HW_FTM_SYNC_WR(ftmBaseAddr, 0);
    HW_FTM_OUTINIT_WR(ftmBaseAddr, 0);
    HW_FTM_OUTMASK_WR(ftmBaseAddr, 0);
    HW_FTM_COMBINE_WR(ftmBaseAddr, 0);
    HW_FTM_DEADTIME_WR(ftmBaseAddr, 0);
    HW_FTM_EXTTRIG_WR(ftmBaseAddr, 0);
    HW_FTM_POL_WR(ftmBaseAddr, 0);
    HW_FTM_FMS_WR(ftmBaseAddr, 0);
    HW_FTM_FILTER_WR(ftmBaseAddr, 0);
    HW_FTM_FLTCTRL_WR(ftmBaseAddr, 0);
    /*HW_FTM_QDCTRL_WR(instance, 0);*/
    HW_FTM_CONF_WR(ftmBaseAddr, 0);
    HW_FTM_FLTPOL_WR(ftmBaseAddr, 0);
    HW_FTM_SYNCONF_WR(ftmBaseAddr, 0);
    HW_FTM_INVCTRL_WR(ftmBaseAddr, 0);
    HW_FTM_SWOCTRL_WR(ftmBaseAddr, 0);
    HW_FTM_PWMLOAD_WR(ftmBaseAddr, 0);
}

void FTM_HAL_SetHardwareSyncTriggerSrc(uint32_t ftmBaseAddr, uint32_t trigger_num, bool enable)
{
    switch(trigger_num)
    {
        case 0:
            BW_FTM_SYNC_TRIG0(ftmBaseAddr, enable ? 1 : 0);
            break;
        case 1:
            BW_FTM_SYNC_TRIG1(ftmBaseAddr, enable ? 1 : 0);
            break;
        case 2:
            BW_FTM_SYNC_TRIG2(ftmBaseAddr, enable ? 1 : 0);
            break;
        default:
            assert(0);
            break;
    }
}

void FTM_HAL_SetChnTriggerCmd(uint32_t ftmBaseAddr, uint8_t channel, bool val)
{
    assert(channel < HW_CHAN6);

    uint8_t bit = val ? 1 : 0;
    uint32_t value = (channel > 1U) ? (uint8_t)(bit << (channel - 2U)) : (uint8_t)(bit << (channel + 4U));

    val ? HW_FTM_EXTTRIG_SET(ftmBaseAddr, value) : HW_FTM_EXTTRIG_CLR(ftmBaseAddr, value);
}

void FTM_HAL_SetChnInputCaptureFilter(uint32_t ftmBaseAddr, uint8_t channel, uint8_t val)
{
    assert(channel < HW_CHAN4);

    switch(channel)
    {
        case HW_CHAN0:
            BW_FTM_FILTER_CH0FVAL(ftmBaseAddr, val);
            break;
        case HW_CHAN1:
            BW_FTM_FILTER_CH1FVAL(ftmBaseAddr, val);
            break;
        case HW_CHAN2:
            BW_FTM_FILTER_CH2FVAL(ftmBaseAddr, val);
            break;
        case HW_CHAN3:
            BW_FTM_FILTER_CH3FVAL(ftmBaseAddr, val);
            break;
        default:
            assert(0);
            break;
    }
}

uint32_t FTM_HAL_GetChnPairIndex(uint8_t channel)
{
    if((channel == HW_CHAN0) || (channel == HW_CHAN1))
    {
        return 0;
    }
    else if((channel == HW_CHAN2) || (channel == HW_CHAN3))
    {
        return 1;
    }
    else if((channel == HW_CHAN4) || (channel == HW_CHAN5))
    {
        return 2;
    }
    else
    {
        return 3;
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

