/**
  \brief      Watch Dog Timers.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for Focused Flight 32.
*/

///////////////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////////////

typedef void (*timeout_fp)(void); /*!< prototype for timeout functions */

int  watchDogRegister(uint32_t*, uint32_t, timeout_fp,int);
void watchDogsTick(void);
void watchDogDisable(uint32_t);
void watchDogReset(uint32_t hnd);

///////////////////////////////////////////////////////////////////////////////
