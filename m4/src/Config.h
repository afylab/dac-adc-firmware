#pragma once

#define NUM_CHANNELS_PER_DAC_BOARD 4
#define NUM_CHANNELS_PER_ADC_BOARD 4
#define NUM_ADC_BOARDS 2

#ifdef __OLD_SHIELD__
#define NUM_DAC_CHANNELS 4
#else
#define NUM_DAC_CHANNELS 8
#endif
