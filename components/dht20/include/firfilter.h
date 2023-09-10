#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FILTER_TAP_NUM 10
#define FIR_FILTER_LENGTH FILTER_TAP_NUM

typedef struct {
    float buf[FIR_FILTER_LENGTH];
    uint8_t bufIndex;

    float out;
} FIRFilter;

void FIRFilter_Init(FIRFilter *fir);
float FIRFilter_Update(FIRFilter *fir, float inp);

/*
Filter Tools

https://fiiir.com
http://t-filter.engineerjs.com
https://tomroelandts.com/articles/how-to-create-a-simple-low-pass-filter
https://wirelesslibrary.labs.b-com.com/FIRfilterdesigner/#/#result-container
https://github.com/pms67/HadesFCS/tree/master/Filtering [Compile with free ms visual studio 2022 c#]
*/

/* FIR FIlter Designer [Philip Salmony] - FIRWinSyncDesign
Filter Order: 32
Sampling Frequency (Hz): 100.000000
Cut-Off Frequency Lo (Hz): 20.000000
Cut-Off Frequency Hi (Hz): 20.000000
*/
// static __attribute__((aligned(16))) float filter_taps[FILTER_TAP_NUM] = { 0.0000000f, 0.0000000f, -0.0008230f, -0.0012128f, 0.0022833f, 0.0061156f, 0.0000000f, -0.0135373f, -0.0116936f, 0.0159713f, 0.0348817f, 0.0000000f, -0.0645992f, -0.0571107f, 0.0899884f, 0.2998222f, 0.4000000f, 0.2998222f, 0.0899884f, -0.0571107f, -0.0645992f, 0.0000000f, 0.0348817f, 0.0159713f, -0.0116936f, -0.0135373f, 0.0000000f, 0.0061156f, 0.0022833f, -0.0012128f, -0.0008230f, 0.0000000f };
// Filter Order: 16 Sampling Frequency (Hz): 100.000000 Cut-Off Frequency Lo (Hz): 10.000000 Cut-Off Frequency Hi (Hz): 20.000000
// static __attribute__((aligned(16))) float filter_taps[FILTER_TAP_NUM] = {0.0000000f, -0.0006327f, -0.0020720f, 0.0000000f, 0.0159033f, 0.0559823f, 0.1170892f, 0.1755929f, 0.2000000f, 0.1755929f, 0.1170892f, 0.0559823f, 0.0159033f, 0.0000000f, -0.0020720f, -0.0006327f};

#ifdef __cplusplus
}
#endif