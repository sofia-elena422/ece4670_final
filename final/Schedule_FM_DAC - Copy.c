/*********************************************************************
 *
 *  FM synth to SPI to  MCP4822 dual channel 12-bit DAC
 *
/* ====== MCP4822 control word ==============
bit 15 A/B: DACA or DACB Selection bit
1 = Write to DACB
0 = Write to DACA
bit 14 ? Don?t Care
bit 13 GA: Output Gain Selection bit
1 = 1x (VOUT = VREF * D/4096)
0 = 2x (VOUT = 2 * VREF * D/4096), where internal VREF = 2.048V.
bit 12 SHDN: Output Shutdown Control bit
1 = Active mode operation. VOUT is available. ?
0 = Shutdown the selected DAC channel. Analog output is not available at the channel that was shut down.
VOUT pin is connected to 500 k???typical)?
bit 11-0 D11:D0: DAC Input Data bits. Bit x is ignored.
*/
////////////////////////////////////
// clock AND protoThreads configure!
// threading library
// for sine
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/sync.h"

#include "pt_cornell_rp2040_v1.h"

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000
#define Fs 4000.0
#define two32 4294967296.0 // 2^32 

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
//static struct pt pt_cmd, pt_tick;
// uart control threads
//static struct pt pt_input, pt_output, pt_DMA_output ;

// system 1 second interval tick
int sys_time_seconds ;

// === GCC s16.15 format ===============================================
#define float2Accum(a) ((_Accum)(a))
#define Accum2float(a) ((float)(a))
#define int2Accum(a) ((_Accum)(a))
#define Accum2int(a) ((int)(a))
// the native type is _Accum but that is ugly
typedef _Accum fixAccum  ;
#define onefixAccum int2Accum(1)
//#define sustain_constant float2Accum(256.0/20000.0) ; // seconds per decay update

// === GCC 0.16 format ===============================================
#define float2Fract(a) ((_Fract)(a))
#define Fract2float(a) ((float)(a))
// the native type is _Accum but that is ugly
typedef _Fract fixFract  ;
fixFract onefixFract = float2Fract(0.9999);
#define sustain_constant float2Accum(256.0/20000.0)  // seconds per decay update

// === 16:16 fixed point macros ==========================================
typedef signed int fix16 ;
#define multfix16(a,b) ((fix16)(((( signed long long)(a))*(( signed long long)(b)))>>16)) //multiply two fixed 16:16
#define float2fix16(a) ((fix16)((a)*65536.0)) // 2^16
#define fix2float16(a) ((float)(a)/65536.0)
#define fix2int16(a)    ((int)((a)>>16))
#define int2fix16(a)    ((fix16)((a)<<16))
#define divfix16(a,b) ((fix16)((((signed long long)(a)<<16)/(b)))) 
#define sqrtfix16(a) (float2fix16(sqrt(fix2float16(a)))) 
#define absfix16(a) abs(a)
#define onefix16 0x00010000 // int2fix16(1)

// actual scaled DAC 
uint16_t DAC_data;
// the DDS units: 1=FM, 2=main frequency
volatile unsigned int phase_accum_fm, phase_incr_fm=1.8*261.0*two32/Fs ;// 
volatile unsigned int phase_accum_main, phase_incr_main=261.0*two32/Fs ;//
//volatile int spiClkDiv = 2 ; // 20 MHz max speed for this DAC

// DDS sine table
#define sine_table_size 256
volatile fix16 sine_table[sine_table_size];


// envelope: 1=FM, 2=main frequency
// envelope: FM and main frequency
volatile fixAccum env_fm, wave_fm, dk_state_fm, attack_state_fm;
volatile fixAccum env_main, wave_main,  dk_state_main, attack_state_main;
// define the envelopes and tonal quality of the instruments
#define n_synth 8 
// 0 plucked string-like
// 1 slow rise 
// 2 string-like lower pitch
// 3 low drum
// 4 medium drum
// 5 snare
// 6 chime
// 7 low, harsh string-like
//                                          0     1      2      3      4      5      6
volatile fixAccum  attack_main[n_synth] = {0.001, 0.9,  0.001, 0.001, 0.001, 0.001, 0.001, .005};
volatile fixAccum   decay_main[n_synth] = {0.98,  0.97, 0.98,  0.98,  0.98,  0.80,  0.98, 0.98};
//
volatile fixAccum     depth_fm[n_synth] = {2.00,  2.5,  2.0,   3.0,   1.5,   10.0,  1.0,  2.0};
volatile fixAccum    attack_fm[n_synth] = {0.001, 0.9,  0.001, 0.001, 0.001, 0.001, 0.001, 0.005};
volatile fixAccum     decay_fm[n_synth] = {0.80,  0.8,  0.80,  0.90,  0.90,  0.80,  0.98,  0.98};
//                          0    1    2    3     4    5     6     7
float freq_main[n_synth] = {1.0, 1.0, 0.5, 0.25, 0.5, 1.00, 1.0,  0.25};
float   freq_fm[n_synth] = {3.0, 1.1, 1.5, 0.4,  0.8, 1.00, 1.34, 0.37};
// the current setting for instrument (index into above arrays)
int current_v1_synth=0, current_v2_synth=0 ;

// pentatonic scale
// Transposing the pitches to fit into one octave rearranges 
// the pitches into the major pentatonic scale: C, D, E, G, A, C.
//  C4   262 Hz (middle C)   
//  C4S  277
//  D4   294
//  D4S  311
//  E4   330
//  F4   349
//  F4S  370
//  G4   392
//  G4S  415
//  A4   440  
//  A4S  466 
//  B4   494
//  C5   523  
//  C5S  554  
//  D5   587
//  D5S  622
//  E5   659
//  F5   698
//  F5S  740
//  G5   784
//  G5S  831
//  A5   880
//  A5S  932
// see: http://www.phy.mtu.edu/~suits/notefreqs.html
float notes[23] = {262,277,294,311,330,349,370,392,415,440,466,494,523,554,587,622,659,698,740,784,831,880,932} ; 
float notes_model[23][23] = {{0.36666666666666664, 0.0, 0.06666666666666667, 0.0, 0.16666666666666666, 0.3, 0.0, 0.03333333333333333, 0.0, 0.03333333333333333, 0.0, 0.0, 0.03333333333333333, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.36666666666666664, 0.03333333333333333, 0.06666666666666667, 0.0, 0.13333333333333333, 0.03333333333333333, 0.0, 0.23333333333333334, 0.0, 0.0, 0.0, 0.03333333333333333, 0.03333333333333333, 0.0, 0.06666666666666667, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0.0, 0.0, 0.19696969696969696, 0.0, 0.3939393939393939, 0.15151515151515152, 0.0, 0.15151515151515152, 0.0, 0.045454545454545456, 0.0, 0.015151515151515152, 0.0, 0.0, 0.030303030303030304, 0.0, 0.015151515151515152, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.057971014492753624, 0.0, 0.10144927536231885, 0.0, 0.2028985507246377, 0.30434782608695654, 0.0, 0.21739130434782608, 0.0, 0.057971014492753624, 0.0, 0.043478260869565216, 0.014492753623188406, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.02247191011235955, 0.0, 0.0, 0.0, 0.11235955056179775, 0.16853932584269662, 0.011235955056179775, 0.1797752808988764, 0.0, 0.42696629213483145, 0.0, 0.0, 0.056179775280898875, 0.0, 0.011235955056179775, 0.0, 0.011235955056179775, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 
0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.011904761904761904, 0.0, 0.03571428571428571, 0.0, 0.0, 0.10714285714285714, 0.0, 0.25, 0.011904761904761904, 0.13095238095238096, 0.0, 0.34523809523809523, 0.10714285714285714, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0.015873015873015872, 0.0, 0.015873015873015872, 0.0, 0.031746031746031744, 0.0, 0.0, 0.15873015873015872, 0.0, 0.38095238095238093, 0.0, 0.07936507936507936, 0.2698412698412698, 0.015873015873015872, 0.031746031746031744, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0273972602739726, 0.0410958904109589, 0.0, 0.1506849315068493, 0.0, 0.0410958904109589, 0.0, 0.1780821917808219, 0.4246575342465753, 0.0, 0.1095890410958904, 0.0, 0.0136986301369863, 0.0136986301369863, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 
0.0, 0.0, 0.034482758620689655, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3793103448275862, 0.20689655172413793, 0.0, 0.27586206896551724, 0.0, 0.06896551724137931, 0.034482758620689655, 0.0, 0.0, 0.0, 0.0, 0.0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0.0, 0.0, 0.09090909090909091, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.09090909090909091, 0.0, 0.2727272727272727, 0.0, 0.36363636363636365, 0.18181818181818182, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.5, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

float notes_cdf[23][23] = {{0.36666666666666664, 0.36666666666666664, 0.4333333333333333, 0.4333333333333333, 0.6, 0.8999999999999999, 0.8999999999999999, 0.9333333333333332, 0.9333333333333332, 0.9666666666666666, 0.9666666666666666, 0.9666666666666666, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999}, {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0.36666666666666664, 0.39999999999999997, 0.4666666666666666, 0.4666666666666666, 0.6, 0.6333333333333333, 0.6333333333333333, 0.8666666666666667, 0.8666666666666667, 0.8666666666666667, 0.8666666666666667, 0.9, 0.9333333333333333, 0.9333333333333333, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0.0, 0.0, 0.19696969696969696, 0.19696969696969696, 0.5909090909090908, 0.7424242424242423, 0.7424242424242423, 0.8939393939393938, 0.8939393939393938, 0.9393939393939392, 0.9393939393939392, 0.9545454545454544, 0.9545454545454544, 0.9545454545454544, 0.9848484848484846, 0.9848484848484846, 0.9999999999999998, 0.9999999999999998, 0.9999999999999998, 0.9999999999999998, 0.9999999999999998, 0.9999999999999998, 0.9999999999999998}, {0.057971014492753624, 0.057971014492753624, 0.15942028985507248, 0.15942028985507248, 0.3623188405797102, 0.6666666666666667, 0.6666666666666667, 0.8840579710144928, 0.8840579710144928, 0.9420289855072465, 0.9420289855072465, 0.9855072463768116, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0.02247191011235955, 0.02247191011235955, 0.02247191011235955, 0.02247191011235955, 0.1348314606741573, 0.3033707865168539, 0.3146067415730337, 0.4943820224719101, 0.4943820224719101, 0.9213483146067416, 0.9213483146067416, 0.9213483146067416, 0.9775280898876405, 0.9775280898876405, 0.9887640449438203, 0.9887640449438203, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0.011904761904761904, 0.011904761904761904, 0.047619047619047616, 0.047619047619047616, 0.047619047619047616, 0.15476190476190477, 0.15476190476190477, 0.40476190476190477, 0.4166666666666667, 0.5476190476190477, 0.5476190476190477, 0.8928571428571429, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0.015873015873015872, 0.015873015873015872, 0.031746031746031744, 0.031746031746031744, 0.06349206349206349, 0.06349206349206349, 0.06349206349206349, 0.2222222222222222, 0.2222222222222222, 0.6031746031746031, 0.6031746031746031, 0.6825396825396826, 0.9523809523809523, 0.9682539682539681, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999, 0.9999999999999999}, {0.0, 0.0, 0.0, 0.0, 0.0273972602739726, 0.0684931506849315, 0.0684931506849315, 0.2191780821917808, 0.2191780821917808, 0.2602739726027397, 0.2602739726027397, 0.4383561643835616, 0.8630136986301369, 0.8630136986301369, 0.9726027397260273, 0.9726027397260273, 0.9863013698630136, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.034482758620689655, 0.034482758620689655, 0.034482758620689655, 0.034482758620689655, 0.034482758620689655, 0.034482758620689655, 0.41379310344827586, 0.6206896551724138, 0.6206896551724138, 0.896551724137931, 0.896551724137931, 0.9655172413793104, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0}, {0.0, 0.0, 0.09090909090909091, 0.09090909090909091, 0.09090909090909091, 0.09090909090909091, 0.09090909090909091, 0.09090909090909091, 0.09090909090909091, 0.09090909090909091, 0.09090909090909091, 0.09090909090909091, 0.18181818181818182, 0.18181818181818182, 0.45454545454545453, 0.45454545454545453, 0.8181818181818181, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.25, 0.75, 0.75, 0.75, 1.0, 1.0, 1.0, 1.0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

                    //  full, half dot, half, quarter dot, quarter, eighth, sixteenth
int lengths[7] = { 16, 12, 8, 6, 4, 2, 1} ; 
float lengths_model[7][7] = {{0.0, 0.0, 0.0, 0.0, 0.6666666666666666, 0.3333333333333333, 0.0}, {0.0, 0.058823529411764705, 0.29411764705882354, 0.11764705882352941, 0.23529411764705882, 0.29411764705882354, 0.0}, {0.0, 0.0, 0.08333333333333333, 0.125, 0.7916666666666666, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.12280701754385964, 0.3157894736842105, 0.5614035087719298, 0.0}, {0.010135135135135136, 0.05067567567567568, 0.05067567567567568, 0.07094594594594594, 0.581081081081081, 0.23648648648648649, 0.0}, {0.0, 0.025, 0.02, 0.11, 0.39, 0.455, 0.0}, {0, 0, 0, 0, 0, 0, 0}};
float lengths_cdf[7][7] = {{0.0, 0.0, 0.0, 0.0, 0.6666666666666666, 1.0, 1.0}, {0.0, 0.058823529411764705, 0.35294117647058826, 0.47058823529411764, 0.7058823529411764, 1.0, 1.0}, {0.0, 0.0, 0.08333333333333333, 0.20833333333333331, 1.0, 1.0, 1.0}, {0.0, 0.0, 0.0, 0.12280701754385964, 0.43859649122807015, 1.0, 1.0}, {0.010135135135135136, 0.060810810810810814, 0.11148648648648649, 0.18243243243243243, 0.7635135135135135, 1.0, 1.0}, {0.0, 0.025, 0.045, 0.155, 0.545, 1.0, 1.0}, {0, 0, 0, 0, 0, 0, 0}};

// note transition rate in ticks of the ISR
// rate is 20/mSec. So 250 mS is 5000 counts
// 8/sec, 4/sec, 2/sec, 1/sec
int tempo[4] = {2500, 5000, 10000, 20000};
int tempo_v1_flag, tempo_v2_flag ;
int current_v1_tempo=1, current_v2_tempo=2;
int tempo_v1_count, tempo_v2_count ;

// beat/rest patterns
#define n_beats 11
int beat[n_beats] = {
		0b0101010101010101, // 1-on 1-off phase 2
		0b1111111011111110, // 7-on 1-off
		0b1110111011101110, // 3-on 1-off
		0b1100110011001100, // 2-on 2-off phase 1
		0b1010101010101010, // 1-on 1-off phase 1
		0b1111000011110000, // 4-on 4-off phase 1
		0b1100000011000000, // 2-on 6-off 
		0b0011001100110011, // 2-on 2-off phase 2
		0b1110110011101100, // 3-on 1-off 2-on 2-off 3-on 1-off 2-on 2-off 
		0b0000111100001111, // 4-on 4-off phase 2
		0b1111111111111111  // on
	} ;
// max-beats <= 16 the length of the beat vector in bits
#define max_beats 16
int current_v1_beat=1, current_v2_beat=2;
int beat_v1_count, beat_v2_count ;
//
// random number
volatile int rand_raw ;

// time scaling for decay calculation
volatile int dk_interval; // wait some samples between decay calcs
// play flag
volatile int play_trigger;
volatile fixAccum sustain_state, sustain_interval=0;
// profiling of ISR
volatile int isr_time, isr_start_time, isr_count=0;

volatile int current_note=0;
volatile int current_length=1;

//SPI configurations (for Pico hardware)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define SPI_PORT spi0 

//=============================
bool repeating_timer_callback(struct repeating_timer *t)
{
    // time to get into ISR
    isr_start_time = time_us_32(); 
    tempo_v1_count++ ;
    if (tempo_v1_count>=tempo[current_v1_tempo]*(lengths[current_length])) {
        tempo_v1_flag = 1;
        tempo_v1_count = 0;
    }
    
    //mT2ClearIntFlag();
    
    // FM phase
    phase_accum_fm += (int)(phase_incr_fm ); 
    // main phase
    phase_accum_main += phase_incr_main + (Accum2int(sine_table[phase_accum_fm>>24] * env_fm)<<16) ;
     
     // init the exponential decays
     // by adding energy to the exponential filters 
    if (play_trigger) {
        dk_state_fm = depth_fm[current_v1_synth]; 
        dk_state_main = onefixAccum; 
        attack_state_fm = depth_fm[current_v1_synth]; 
        attack_state_main = onefixAccum; 
        play_trigger = 0; 
        phase_accum_fm = 0;
        phase_accum_main = 0;
        dk_interval = 0;
        sustain_state = 0;
    }
    
    // envelope calculations are 256 times slower than sample rate
    // computes 4 exponential decays and builds the product envelopes
    if ((dk_interval++ & 0xff) == 0){
        // approximate the first order FM decay  ODE
        dk_state_fm = dk_state_fm * decay_fm[current_v1_synth] ;
        //  approximate the first order main waveform decay  ODE
        dk_state_main = dk_state_main * decay_main[current_v1_synth] ;
        // approximate the ODE for the exponential rise FM/main waveform
        attack_state_fm = attack_state_fm * attack_fm[current_v1_synth] ;
        attack_state_main = attack_state_main * attack_main[current_v1_synth] ;
        // product of rise and fall is the FM envelope
        // fm_depth is the current value of the function
        env_fm = (depth_fm[current_v1_synth] - attack_state_fm) * dk_state_fm ;
        // product of rise and fall is the main envelope
        env_main = (onefixAccum - attack_state_main) * dk_state_main ;
     
    }

    // === Channel A =============
    // CS low to start transaction
    gpio_put(PIN_CS, 0);
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);
     //WriteSPI2(DAC_data); 
    
    wave_main = (sine_table[phase_accum_main>>24]) * env_main ;
    // truncate to 12 bits, read table, convert to int and add offset
    DAC_data = DAC_config_chan_A | (Accum2int(wave_main) + 2048) ; 
    // test for done
    while (spi_is_busy(SPI_PORT)); // wait for end of transaction
     // CS high
     gpio_put(PIN_CS, 1);
     
     // time to get into ISR is the same as the time to get out so add it again
     isr_time = fmax(isr_time, time_us_32()+isr_start_time) ; // - isr_time;
     return true;
} // end ISR TIMER2

// === Serial Thread ======================================================
// revised commands:
// i index (0-7) -- choose instrument index, default=0 string
// pn note (0-7)  -- play note default=C
// ps -- play scale
// t -- print current instrument parameters
// 
static PT_THREAD (protothread_serial(struct pt *pt))
{
    // The serial interface
    static char cmd[16]; 
    static float value;
    static float f_fm, f_main;
    

    PT_BEGIN(pt);
    
      while(1) {
            rand_raw = rand();
            
            // send the prompt to serial
            sprintf(pt_serial_out_buffer,"\n\rcmd>");
            // by spawning a print thread
            serial_write;
            serial_read;
            //parse the string
             sscanf(pt_serial_in_buffer, "%s %f", cmd, &value);

             switch(cmd[0]){
                 case 'p': // value is note index
                     if(cmd[1]=='n') {
                         phase_incr_fm = freq_fm[current_v1_synth]*notes[(int)value]*(float)two32/Fs; 
                         phase_incr_main = freq_main[current_v1_synth]*notes[(int)value]*(float)two32/Fs; 
                         play_trigger = 1;
                        sprintf(pt_serial_out_buffer, "isr time=%d \n\r", isr_time);
                        serial_write;
                        break ;
                     }
                     
                     // value is ignored
                     if(cmd[1]=='s') {
                         static int i;
                         tempo_v1_count = 0;
                         tempo_v1_flag = 0;
                         for (i=0; i<8; i++){
                            current_length=1;
                            PT_YIELD_UNTIL(pt,tempo_v1_flag==1);
                            phase_incr_fm = freq_fm[current_v1_synth]*notes[i]*(float)two32/Fs; 
                            phase_incr_main = freq_main[current_v1_synth]*notes[i]*(float)two32/Fs; 
                            play_trigger = 1;
                            tempo_v1_flag = 0;
                         }
                        break;
                     }
                     
                     
                 case 'i': // value is instrument index
                     current_v1_synth = (int)(value);
                     break;
                
                 case 't': // value is tempo index 0-3
                     current_v1_tempo = (int)(value);
                     break;

             }
             
            // never exit while
      } // END WHILE(1)
  PT_END(pt);
} // thread 1

// === Thread 2 ======================================================
// update a 1 second tick counter
static PT_THREAD (protothread_tick(struct pt *pt))
{
    PT_BEGIN(pt);

      while(1) {
            // yield time 1 second
            PT_YIELD_usec(1000000) ;
            sys_time_seconds++ ;
            rand_raw = rand();
            // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // thread 2

// === Main  ======================================================
// set up UART, timer2, threads
// then schedule them as fast as possible

int main(void)
{
  srand((unsigned) time(NULL));
  // === config the uart, DMA, vref, timer5 ISR =============
    stdio_init_all();

   // === setup system wide interrupts  ====================
  //INTEnableSystemMultiVectoredInt();
    
    // 400 is 100 ksamples/sec
    // 1000 is 40 ksamples/sec
    // 2000 is 20 ksamp/sec
    // 1000 is 40 ksample/sec
    // 2000 is 20 ks/sec
    //OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 2000);
    // set up the timer interrupt with a priority of 2
    //ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    //mT2ClearIntFlag(); // and clear the interrupt flag

    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_init(SPI_PORT, 20000000);
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

  //initialize timer
  struct repeating_timer timer;
  add_repeating_timer_us(-25, repeating_timer_callback, NULL, &timer);
  // init the threads
  pt_add_thread(protothread_cmd);
  pt_add_thread(protothread_tick);

  // turn off the sustain until triggered
  sustain_state = float2Accum(100.0);
  
  // build the sine lookup table
   // scaled to produce values between 0 and 4095
   int i;
   for (i = 0; i < sine_table_size; i++){
         sine_table[i] =  float2Accum(2047.0*sin((float)i*6.283/(float)sine_table_size));
    }
    
  pt_schedule_start;
} // main

////////////////////////////////////////////////////////
/*
Examples:      
Chime:
 * freq_main = 261
 * freq_FM = 350
 * depthFM =  1.0
 * decay_main = 0.99
 * decay_FM = 0.99
 * attack_main = 0.001
 * attack_fM = 0.001
 * sustain = 0
 * VARIANTS
 * depthFM = 3 gives struck stiff string
 * doubling both frequenies also sounds like chime
 * 
Plucked String:
 * freq_main = 261
 * freq_FM = 3*361 = 783
 * depthFM =  1.0 - 2.5
 * decay_main = 0.99
 * decay_FM = 0.5 - 0.8
 * attack_main = 0.001
 * attack_fM = 0.001
 * sustain = 0
 * VARIANTS
	
Bowed string
 * freq_main = 261
 * freq_FM = 261
 * depthFM =  2 - 3
 * decay_main = 0.99
 * decay_FM = 0.5 - 0.8
 * attack_main = 0.9
 * attack_fM = 0.9
 * sustain = 0.1
 * VARIANTS
Bell/chime
 * freq_main = 1440
 * freq_FM  = 600 (400 - 800) 
 * depthFM =  1
 * decay_main = 0.99
 * decay_FM = 0.99
 * attack_main = 0.001
 * attack_fM = 0.001
 * sustain = 
 * VARIANTS
 * freq_FM  = 50, 100,  2000 
 * small drum
f_fm=180.0 f_main=100.0
fm_depth=1.000
sustain=0
dk_fm=0.89999 dk_main=0.89999
attack_fm=0.00098 attack_main=0.00098
 * big drum
f_fm= 90.0 f_main= 50.0
fm_depth=0.600
sustain=0
dk_fm=0.98999 dk_main=0.98999
attack_fm=0.00098 attack_main=0.00098
 * snare/cymbal
 f_fm=200.0 f_main= 50.0
fm_depth=50.000
sustain=0
dk_fm=0.79999 dk_main=0.79999
attack_fm=0.00098 attack_main=0.00098
 * 
*/

/*
================================================================
% compute normalized markov matrix
% 8x8 for test
clear all
clc
s = 1/2 ; % the ratio of one element to the next in a row
A = [ 1 s s^2 s^3 s^4 s^5 s^6 s^7; 
      s 1 s s^2 s^3 s^4 s^5 s^6 ;
      s^2 s 1 s s^2 s^3 s^4 s^5 ;
      s^3 s^2 s 1 s s^2 s^3 s^4 ;
      s^4 s^3 s^2 s 1 s s^2 s^3 ;
      s^5 s^4 s^3 s^2 s 1 s s^2 ;
      s^6 s^5 s^4 s^3 s^2 s 1 s ;
      s^7 s^6 s^5 s^4 s^3 s^2 s 1 ;] ;
  
for i=1:8
    norm = 127/sum(A(i,:));
    Anorm(i,:) = A(i,:)*norm ;
end
Anorm = round(Anorm);
for i=1:8
   d = sum(Anorm(i,:)) - 127 ;
   Anorm(i,i) = Anorm(i,i) - d ;
   fprintf('%d,%d,%d,%d,%d,%d,%d,%d,\n', Anorm(i,:))
end
*/