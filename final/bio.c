// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
// Include custom libraries
#include "vga_graphics.h"
#include "pt_cornell_rp2040_v1.h"

//bio_input gpio
#define BIO_IN     26
fix15 bio_arr[10] = {0,0,0,0,0,0,0,0,0,0};
uint16 output =0;
//vga initialization
char screentext[40];
static struct pt_sem vga_semaphore ;

// semaphore
static struct pt_sem vga_semaphore ;

// Protothread for writing diagnostic data for the input voltage term on the screen
static PT_THREAD (protothread_timer(struct pt *pt)){
  PT_BEGIN(pt);
  while(1){
    fillRect(0, 0, 640, 62, BLACK); 
    char strs[50];
    sprintf(strs,"Voltage %d", output);
    setCursor(0,10);
    writeString(strs);
   // printf("sum: %f", integral_sum);
    PT_YIELD_usec(100000) ;
   
  }
  PT_END(pt);
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange_a = 500. ; // (+/- 250)
    static float NewRange_a = 150. ; // (looks nice on VGA)
    static float OldMin_a = 0. ;
    static float OldMax_a = 180. ;

    static float OldRange_m = 500. ; // (+/- 250)
    static float NewRange_m = 150. ; // (looks nice on VGA)
    static float OldMin_m = 0. ;
    static float OldMax_m = 5000. ;

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "5000") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot
    // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "180") ;
    setCursor(50, 75) ;
    writeString(screentext) ;

    sprintf(screentext, "90") ;
    setCursor(50, 150) ;
    writeString(screentext) ;

    sprintf(screentext, "0") ;
    setCursor(45, 225) ;
    writeString(screentext) ;

    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK) ;

            // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            drawPixel(xcoord, 430 - (int)(NewRange_a*((float)((fix2float15(output)*0.1))/OldRange_a)), WHITE) ;
           // drawPixel(xcoord, 355 - (int)(NewRange_a*((float)(((k_p * fix2float15(angle_err))*0.05))/OldRange_a)), RED) ;
           // drawPixel(xcoord, 355 - (int)(NewRange_a*((float)(((k_i*integral_sum)*0.05))/OldRange_a)), BLUE) ;
           // drawPixel(xcoord, 355 - (int)(NewRange_a*((float)(((k_d*fix2float15(angle_rate))*0.05))/OldRange_a)), GREEN) ;

            // Draw top plot
            //drawPixel(xcoord, 230 - (int)(NewRange_a*((float)((fix2float15(angle)*2.8)-OldMin_a)/OldRange_a)), WHITE) ;


            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

static PT_THREAD (protothread_bio(struct pt *pt)){
    PT_BEGIN(pt) ;
    while(1){
        uint16 input =0;
        uint16 mean =0;
        for(i=0; i=10; i++){
            input = adc_read();
            mean = input+mean;
        }
        mean = mean/10;
        output =mean;
        PT_SEM_SIGNAL(pt, &vga_semaphore);

    }
    PT_END(pt) ;

}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}

int main() {

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    adc_init();
    adc_gpio_init(BIO_IN);
    adc_select_input(BIO_IN);


    //start protothreads

    // start core 1 for vga screen
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0 
    pt_add_thread(protothread_serial) ;
    pt_add_thread(protothread_bio);
    //pt_add_thread(protothread_button);

    pt_schedule_start ;

}