/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the AQ32 Flight Control Board

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)Paparazzi UAV
  5)S.O.H. Madgwick
  6)UAVX

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

#define FLASH_WRITE_EEPROM_ADDR  0x08004000  // FLASH_Sector_1

const char rcChannelLetters[] = "AERT1234";

static uint8_t checkNewEEPROMConf = 1;

///////////////////////////////////////////////////////////////////////////////

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++)
    {
        s = strchr(rcChannelLetters, *c);
        if (s)
            eepromConfig.rcMap[s - rcChannelLetters] = c - input;
    }
}

///////////////////////////////////////////////////////////////////////////////

uint32_t crc32bEEPROM(eepromConfig_t *e, int includeCRCAtEnd)
{
    return crc32B((uint32_t*)e, includeCRCAtEnd ? (uint32_t*)(e + 1) : e->CRCAtEnd);
}

///////////////////////////////////////////////////////////////////////////////

enum { eepromConfigNUMWORD =  sizeof(eepromConfig_t)/sizeof(uint32_t) };

///////////////////////////////////////////////////////////////////////////////

void readEEPROM(void)
{
    eepromConfig_t *dst = &eepromConfig;

    *dst = *(eepromConfig_t*)FLASH_WRITE_EEPROM_ADDR ;

    if ( crcCheckVal != crc32bEEPROM(dst, true) )
    {
        evrPush(EVR_FlashCRCFail,0);
        dst->CRCFlags |= CRC_HistoryBad;
    }
    else if ( dst->CRCFlags & CRC_HistoryBad )
      evrPush(EVR_ConfigBadHistory,0);

    ///////////////////////////////////

    accConfidenceDecay = 1.0f / sqrt(eepromConfig.accelCutoff);

    if (eepromConfig.yawDirection >= 0)
        eepromConfig.yawDirection = 1.0f;
    else
    	eepromConfig.yawDirection = -1.0f;
}

///////////////////////////////////////////////////////////////////////////////

uint8_t writeEEPROM(void)
{
    // there's no reason to write these values to EEPROM, they'll just be noise
    zeroPIDintegralError();
    zeroPIDstates();

    FLASH_Status status;

    int i;
    uint32_t       *dst = (uint32_t*)FLASH_WRITE_EEPROM_ADDR;
    eepromConfig_t *src = &eepromConfig;

    if ( src->CRCFlags & CRC_HistoryBad )
        evrPush(EVR_ConfigBadHistory,0);

    src->CRCAtEnd[0] = crc32B( (uint32_t*)&src[0], src->CRCAtEnd);

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR  | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    i = -1;

    status = FLASH_EraseSector(FLASH_Sector_1, VoltageRange_3);

    while ( FLASH_COMPLETE == status && i++ < eepromConfigNUMWORD )
        status = FLASH_ProgramWord((uint32_t)&dst[i], ((uint32_t*)src)[i]);

    if ( FLASH_COMPLETE != status )
        evrPush( -1 == i ? EVR_FlashEraseFail : EVR_FlashProgramFail, status);

    FLASH_Lock();

    readEEPROM();

    return status;
}

///////////////////////////////////////////////////////////////////////////////

void checkFirstTime(bool eepromReset)
{
    uint8_t test_val;

    test_val = *(uint8_t *)FLASH_WRITE_EEPROM_ADDR;

    if (eepromReset || test_val != checkNewEEPROMConf)
    {
		// Default settings
        eepromConfig.version = checkNewEEPROMConf;

	    ///////////////////////////////

        eepromConfig.accelBiasMXR[XAXIS]        = 2048.0f;
        eepromConfig.accelBiasMXR[YAXIS]        = 2048.0f;
        eepromConfig.accelBiasMXR[ZAXIS]        = 2048.0f;

        ///////////////////////////////

        eepromConfig.accelScaleFactorMXR[XAXIS] = 0.04937965f;  // (3.3 / 4096) / 0.16 * 9.8065
        eepromConfig.accelScaleFactorMXR[YAXIS] = 0.04937965f;  // (3.3 / 4096) / 0.16 * 9.8065
        eepromConfig.accelScaleFactorMXR[ZAXIS] = 0.04937965f;  // (3.3 / 4096) / 0.16 * 9.8065

        ///////////////////////////////

        eepromConfig.accelTCBiasSlope[XAXIS] = 0.0f;
        eepromConfig.accelTCBiasSlope[YAXIS] = 0.0f;
        eepromConfig.accelTCBiasSlope[ZAXIS] = 0.0f;

        ///////////////////////////////

        eepromConfig.accelTCBiasIntercept[XAXIS] = 0.0f;
        eepromConfig.accelTCBiasIntercept[YAXIS] = 0.0f;
        eepromConfig.accelTCBiasIntercept[ZAXIS] = 0.0f;

        ///////////////////////////////

        eepromConfig.gyroTCBiasSlope[ROLL ] = 0.0f;
        eepromConfig.gyroTCBiasSlope[PITCH] = 0.0f;
        eepromConfig.gyroTCBiasSlope[YAW  ] = 0.0f;

	    ///////////////////////////////

	    eepromConfig.gyroTCBiasIntercept[ROLL ] = 0.0f;
	    eepromConfig.gyroTCBiasIntercept[PITCH] = 0.0f;
	    eepromConfig.gyroTCBiasIntercept[YAW  ] = 0.0f;

	    ///////////////////////////////

	    eepromConfig.magBias[XAXIS] = 0.0f;
	    eepromConfig.magBias[YAXIS] = 0.0f;
	    eepromConfig.magBias[ZAXIS] = 0.0f;

		///////////////////////////////

		eepromConfig.accelCutoff = 1.0f;

		///////////////////////////////

	    eepromConfig.KpAcc = 5.0f;    // proportional gain governs rate of convergence to accelerometer
	    eepromConfig.KiAcc = 0.0f;    // integral gain governs rate of convergence of gyroscope biases
	    eepromConfig.KpMag = 5.0f;    // proportional gain governs rate of convergence to magnetometer
	    eepromConfig.KiMag = 0.0f;    // integral gain governs rate of convergence of gyroscope biases

	    ///////////////////////////////

	    eepromConfig.compFilterA =  2.0f;
		eepromConfig.compFilterB =  1.0f;

	    ///////////////////////////////

	    eepromConfig.dlpfSetting = BITS_DLPF_CFG_98HZ;

	    ///////////////////////////////////

        eepromConfig.rateScaling     = 300.0 / 180000.0 * PI;  // Stick to rate scaling for 300 DPS

        eepromConfig.attitudeScaling = 60.0  / 180000.0 * PI;  // Stick to att scaling for 60 degrees

        eepromConfig.nDotEdotScaling = 0.009f;      // Stick to nDot/eDot scaling (9 mps)/(1000 RX PWM Steps) = 0.009

        eepromConfig.hDotScaling     = 0.003f;      // Stick to hDot scaling (3 mps)/(1000 RX PWM Steps) = 0.003

        ///////////////////////////////

	    eepromConfig.receiverType  = SPEKTRUM;

	    eepromConfig.slaveSpektrum = false;

	    parseRcChannels("TAER2134");

	    eepromConfig.escPwmRate   = 450;
        eepromConfig.servoPwmRate = 50;

        eepromConfig.mixerConfiguration = MIXERTYPE_QUADX;
        eepromConfig.yawDirection = 1.0f;

        eepromConfig.midCommand   = 3000.0f;
        eepromConfig.minCheck     = (float)(MINCOMMAND + 200);
        eepromConfig.maxCheck     = (float)(MAXCOMMAND - 200);
        eepromConfig.minThrottle  = (float)(MINCOMMAND + 200);
        eepromConfig.maxThrottle  = (float)(MAXCOMMAND);

        eepromConfig.PID[ROLL_RATE_PID].B               =   1.0f;
        eepromConfig.PID[ROLL_RATE_PID].P               = 250.0f;
        eepromConfig.PID[ROLL_RATE_PID].I               = 100.0f;
        eepromConfig.PID[ROLL_RATE_PID].D               =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].iTerm           =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].windupGuard     = 100.0f;  // PWMs
        eepromConfig.PID[ROLL_RATE_PID].lastDcalcValue  =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].lastDterm       =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].lastLastDterm   =   0.0f;
        eepromConfig.PID[ROLL_RATE_PID].dErrorCalc      =   D_ERROR;
        eepromConfig.PID[ROLL_RATE_PID].type            =   OTHER;

        eepromConfig.PID[PITCH_RATE_PID].B              =   1.0f;
        eepromConfig.PID[PITCH_RATE_PID].P              = 250.0f;
        eepromConfig.PID[PITCH_RATE_PID].I              = 100.0f;
        eepromConfig.PID[PITCH_RATE_PID].D              =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].iTerm          =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].windupGuard    = 100.0f;  // PWMs
        eepromConfig.PID[PITCH_RATE_PID].lastDcalcValue =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].lastDterm      =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].lastLastDterm  =   0.0f;
        eepromConfig.PID[PITCH_RATE_PID].dErrorCalc     =   D_ERROR;
        eepromConfig.PID[PITCH_RATE_PID].type           =   OTHER;

        eepromConfig.PID[YAW_RATE_PID].B                =   1.0f;
        eepromConfig.PID[YAW_RATE_PID].P                = 350.0f;
        eepromConfig.PID[YAW_RATE_PID].I                = 100.0f;
        eepromConfig.PID[YAW_RATE_PID].D                =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].iTerm            =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].windupGuard      = 100.0f;  // PWMs
        eepromConfig.PID[YAW_RATE_PID].lastDcalcValue   =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].lastDterm        =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].lastLastDterm    =   0.0f;
        eepromConfig.PID[YAW_RATE_PID].dErrorCalc       =   D_ERROR;
        eepromConfig.PID[YAW_RATE_PID].type             =   OTHER;

        eepromConfig.PID[ROLL_ATT_PID].B                =   1.0f;
        eepromConfig.PID[ROLL_ATT_PID].P                =   2.0f;
        eepromConfig.PID[ROLL_ATT_PID].I                =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].D                =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].iTerm            =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].windupGuard      =   0.5f;  // radians/sec
        eepromConfig.PID[ROLL_ATT_PID].lastDcalcValue   =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].lastDterm        =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].lastLastDterm    =   0.0f;
        eepromConfig.PID[ROLL_ATT_PID].dErrorCalc       =   D_ERROR;
        eepromConfig.PID[ROLL_ATT_PID].type             =   ANGULAR;

        eepromConfig.PID[PITCH_ATT_PID].B               =   1.0f;
        eepromConfig.PID[PITCH_ATT_PID].P               =   2.0f;
        eepromConfig.PID[PITCH_ATT_PID].I               =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].D               =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].iTerm           =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].windupGuard     =   0.5f;  // radians/sec
        eepromConfig.PID[PITCH_ATT_PID].lastDcalcValue  =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].lastDterm       =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].lastLastDterm   =   0.0f;
        eepromConfig.PID[PITCH_ATT_PID].dErrorCalc      =   D_ERROR;
        eepromConfig.PID[PITCH_ATT_PID].type            =   ANGULAR;

        eepromConfig.PID[HEADING_PID].B                 =   1.0f;
        eepromConfig.PID[HEADING_PID].P                 =   3.0f;
        eepromConfig.PID[HEADING_PID].I                 =   0.0f;
        eepromConfig.PID[HEADING_PID].D                 =   0.0f;
        eepromConfig.PID[HEADING_PID].iTerm             =   0.0f;
        eepromConfig.PID[HEADING_PID].windupGuard       =   0.5f;  // radians/sec
        eepromConfig.PID[HEADING_PID].lastDcalcValue    =   0.0f;
        eepromConfig.PID[HEADING_PID].lastDterm         =   0.0f;
        eepromConfig.PID[HEADING_PID].lastLastDterm     =   0.0f;
        eepromConfig.PID[HEADING_PID].dErrorCalc        =   D_ERROR;
        eepromConfig.PID[HEADING_PID].type              =   ANGULAR;

        eepromConfig.PID[NDOT_PID].B                    =   1.0f;
        eepromConfig.PID[NDOT_PID].P                    =   3.0f;
        eepromConfig.PID[NDOT_PID].I                    =   0.0f;
        eepromConfig.PID[NDOT_PID].D                    =   0.0f;
        eepromConfig.PID[NDOT_PID].iTerm                =   0.0f;
        eepromConfig.PID[NDOT_PID].windupGuard          =   0.5f;
        eepromConfig.PID[NDOT_PID].lastDcalcValue       =   0.0f;
        eepromConfig.PID[NDOT_PID].lastDterm            =   0.0f;
        eepromConfig.PID[NDOT_PID].lastLastDterm        =   0.0f;
        eepromConfig.PID[NDOT_PID].dErrorCalc           =   D_ERROR;
        eepromConfig.PID[NDOT_PID].type                 =   OTHER;

        eepromConfig.PID[EDOT_PID].B                    =   1.0f;
        eepromConfig.PID[EDOT_PID].P                    =   3.0f;
        eepromConfig.PID[EDOT_PID].I                    =   0.0f;
        eepromConfig.PID[EDOT_PID].D                    =   0.0f;
        eepromConfig.PID[EDOT_PID].iTerm                =   0.0f;
        eepromConfig.PID[EDOT_PID].windupGuard          =   0.5f;
        eepromConfig.PID[EDOT_PID].lastDcalcValue       =   0.0f;
        eepromConfig.PID[EDOT_PID].lastDterm            =   0.0f;
        eepromConfig.PID[EDOT_PID].lastLastDterm        =   0.0f;
        eepromConfig.PID[EDOT_PID].dErrorCalc           =   D_ERROR;
        eepromConfig.PID[EDOT_PID].type                 =   OTHER;

        eepromConfig.PID[HDOT_PID].B                    =   1.0f;
        eepromConfig.PID[HDOT_PID].P                    =   2.0f;
        eepromConfig.PID[HDOT_PID].I                    =   0.0f;
        eepromConfig.PID[HDOT_PID].D                    =   0.0f;
        eepromConfig.PID[HDOT_PID].iTerm                =   0.0f;
        eepromConfig.PID[HDOT_PID].windupGuard          =   5.0f;
        eepromConfig.PID[HDOT_PID].lastDcalcValue       =   0.0f;
        eepromConfig.PID[HDOT_PID].lastDterm            =   0.0f;
        eepromConfig.PID[HDOT_PID].lastLastDterm        =   0.0f;
        eepromConfig.PID[HDOT_PID].dErrorCalc           =   D_ERROR;
        eepromConfig.PID[HDOT_PID].type                 =   OTHER;

        eepromConfig.PID[N_PID].B                       =   1.0f;
        eepromConfig.PID[N_PID].P                       =   3.0f;
        eepromConfig.PID[N_PID].I                       =   0.0f;
        eepromConfig.PID[N_PID].D                       =   0.0f;
        eepromConfig.PID[N_PID].iTerm                   =   0.0f;
        eepromConfig.PID[N_PID].windupGuard             =   0.5f;
        eepromConfig.PID[N_PID].lastDcalcValue          =   0.0f;
        eepromConfig.PID[N_PID].lastDterm               =   0.0f;
        eepromConfig.PID[N_PID].lastLastDterm           =   0.0f;
        eepromConfig.PID[N_PID].dErrorCalc              =   D_ERROR;
        eepromConfig.PID[N_PID].type                    =   OTHER;

        eepromConfig.PID[E_PID].B                       =   1.0f;
        eepromConfig.PID[E_PID].P                       =   3.0f;
        eepromConfig.PID[E_PID].I                       =   0.0f;
        eepromConfig.PID[E_PID].D                       =   0.0f;
        eepromConfig.PID[E_PID].iTerm                   =   0.0f;
        eepromConfig.PID[E_PID].windupGuard             =   0.5f;
        eepromConfig.PID[E_PID].lastDcalcValue          =   0.0f;
        eepromConfig.PID[E_PID].lastDterm               =   0.0f;
        eepromConfig.PID[E_PID].lastLastDterm           =   0.0f;
        eepromConfig.PID[E_PID].dErrorCalc              =   D_ERROR;
        eepromConfig.PID[E_PID].type                    =   OTHER;

        eepromConfig.PID[H_PID].B                       =   1.0f;
        eepromConfig.PID[H_PID].P                       =   2.0f;
        eepromConfig.PID[H_PID].I                       =   0.0f;
        eepromConfig.PID[H_PID].D                       =   0.0f;
        eepromConfig.PID[H_PID].iTerm                   =   0.0f;
        eepromConfig.PID[H_PID].windupGuard             =   5.0f;
        eepromConfig.PID[H_PID].lastDcalcValue          =   0.0f;
        eepromConfig.PID[H_PID].lastDterm               =   0.0f;
        eepromConfig.PID[H_PID].lastLastDterm           =   0.0f;
        eepromConfig.PID[H_PID].dErrorCalc              =   D_ERROR;
        eepromConfig.PID[H_PID].type                    =   OTHER;

        eepromConfig.magVar                 =  9.033333f * D2R;  // Albuquerque, NM Mag Var 9 degrees 2 minutes (+ East, - West)

        eepromConfig.batteryCells           = 3;
        eepromConfig.voltageMonitorScale    = 11.5f / 1.5f;
        eepromConfig.voltageMonitorBias     = 0.0f;

        eepromConfig.armCount               =  50;
        eepromConfig.disarmCount            =  0;

        eepromConfig.activeTelemetry        =  0;

        eepromConfig.verticalVelocityHoldOnly = true;

        writeEEPROM();
	}
}

///////////////////////////////////////////////////////////////////////////////
