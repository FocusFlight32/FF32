/*
  August 2013

  Focused Flight32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Designed to run on the Naze32Pro Flight Control Board

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

float   attCmd[3];

float   attPID[3];

float   axisPID[3];

float   rateCmd[3];

float   headingReference;

float   altitudeHoldReference;

float   throttleReference;

float   verticalVelocityCmd;

///////////////////////////////////////////////////////////////////////////////
// Compute Axis Commands
///////////////////////////////////////////////////////////////////////////////

void computeAxisCommands(float dt)
{
    if (flightMode == ATTITUDE)
    {
        attCmd[ROLL ] = rxCommand[ROLL ] * eepromConfig.attitudeScaling;
        attCmd[PITCH] = rxCommand[PITCH] * eepromConfig.attitudeScaling;
    }

    if (flightMode >= ATTITUDE)
    {
        attPID[ROLL]  = updatePID( attCmd[ROLL ],  sensors.attitude500Hz[ROLL ], dt, holdIntegrators, &eepromConfig.PID[ROLL_ATT_PID ] );
        attPID[PITCH] = updatePID( attCmd[PITCH], -sensors.attitude500Hz[PITCH], dt, holdIntegrators, &eepromConfig.PID[PITCH_ATT_PID] );
    }

    if (flightMode == RATE)
    {
        rateCmd[ROLL ] = rxCommand[ROLL ] * eepromConfig.rateScaling;
        rateCmd[PITCH] = rxCommand[PITCH] * eepromConfig.rateScaling;
    }
    else
    {
        rateCmd[ROLL ] = attPID[ROLL ];
        rateCmd[PITCH] = attPID[PITCH];
    }

    ///////////////////////////////////

    if (headingHoldEngaged == true)  // Heading Hold is ON
        rateCmd[YAW] = updatePID( headingReference, heading.mag, dt, holdIntegrators, &eepromConfig.PID[HEADING_PID] );
    else                             // Heading Hold is OFF
        rateCmd[YAW] = rxCommand[YAW] * eepromConfig.rateScaling;

    ///////////////////////////////////

    axisPID[ROLL ] = updatePID( rateCmd[ROLL ],  sensors.gyro500Hz[ROLL ], dt, holdIntegrators, &eepromConfig.PID[ROLL_RATE_PID ] );
    axisPID[PITCH] = updatePID( rateCmd[PITCH], -sensors.gyro500Hz[PITCH], dt, holdIntegrators, &eepromConfig.PID[PITCH_RATE_PID] );
    axisPID[YAW  ] = updatePID( rateCmd[YAW  ],  sensors.gyro500Hz[YAW  ], dt, holdIntegrators, &eepromConfig.PID[YAW_RATE_PID  ] );

    ///////////////////////////////////

	if (verticalModeState == ALT_DISENGAGED_THROTTLE_ACTIVE)            // Manual Mode is ON
        throttleCmd = rxCommand[THROTTLE];

    else
    {
		if ((verticalModeState == ALT_HOLD_FIXED_AT_ENGAGEMENT_ALT) ||  // Altitude Hold is ON
            (verticalModeState == ALT_HOLD_AT_REFERENCE_ALTITUDE)   ||
            (verticalModeState == ALT_DISENGAGED_THROTTLE_INACTIVE))
        {

			verticalVelocityCmd = updatePID( altitudeHoldReference, hEstimate, dt, holdIntegrators, &eepromConfig.PID[H_PID] );
		}
        else                                                            // Vertical Velocity Hold is ON
        {
            verticalVelocityCmd = verticalReferenceCommand * eepromConfig.hDotScaling;
        }

    	throttleCmd = throttleReference + updatePID( verticalVelocityCmd, hDotEstimate, dt, holdIntegrators, &eepromConfig.PID[HDOT_PID] );
	}
}

///////////////////////////////////////////////////////////////////////////////
