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
// Vertical Complementary Filter Defines and Variables
///////////////////////////////////////////////////////////////////////////////

float   accelZ;
float   estimationError = 0.0f;
float   hDotEstimate    = 0.0f;
float   hEstimate;
uint8_t previousExecUp  = false;

///////////////////////////////////////////////////////////////////////////////
// Vertical Complementary Filter
///////////////////////////////////////////////////////////////////////////////

void vertCompFilter(float dt)
{
    if ((execUp == true) && (previousExecUp == false))
    	hEstimate = sensors.pressureAlt50Hz;

    previousExecUp = execUp;

	if (execUp == true)
    {
    	accelZ = -earthAxisAccels[ZAXIS] + eepromConfig.compFilterB * estimationError;

        hDotEstimate += accelZ * dt;

        hEstimate += (hDotEstimate + eepromConfig.compFilterA * estimationError) * dt;

        estimationError = sensors.pressureAlt50Hz - hEstimate;
    }
}

///////////////////////////////////////////////////////////////////////////////




