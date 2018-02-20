/*
    
  harness-216 derived from example ledramp in simavr to
  construct a system that provides timed uart serial input
  to the 32u4 and traces the output signals using vcd and
  captures serial output for postprocessing by python or
  ruby.

  it may not actually do any of that.  

  Copyright 2017 Neil Spring 

*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <libgen.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/stat.h>

#include "sim_avr.h"
#include "avr_ioport.h"
#include "sim_elf.h"
#include "sim_gdb.h"
#include "sim_vcd_file.h"
#include "avr_adc.h"
#include "avr_acomp.h"

#define F_CPU 8000000U

avr_t * avr = NULL;
avr_vcd_t vcd_output_file;
avr_vcd_t vcd_input_file;
uint8_t	pin_state = 0;	// current port B

float pixsize = 64;
int window;

/*
 * called when the AVR change any of the pins on port C (c bit 6 is the red led)
 * so lets update our buffer
 */
void pin_changed_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
	pin_state = (pin_state & ~(1 << irq->irq)) | (value << irq->irq);
    fprintf(stderr, "pin_state %x\n", pin_state);
}

int led_flipped_count;
void led_changed_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
  static unsigned char led_is_on;
  if(value != led_is_on) led_flipped_count++;
  led_is_on = value;
}

void adc_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
  printf("adc is hooked!\n");
}

void neopixel_changed_hook(struct avr_irq_t * irq, uint32_t value, void * param) {
  static unsigned long start_cycle, last_cycle;
  unsigned long position;
  static unsigned char Pixels[10][3];

  /* if we haven't seen a transition for 200 cycles, assume
     we're starting over.  Even on init, we only blit zeroes
     out at cycle 255 after the pin is set to output
     low.. */
  if(avr->cycle > last_cycle + 200) {
    start_cycle = avr->cycle;
    memset(Pixels, 0, sizeof(Pixels));
    // fprintf(stderr, "starting pixel set at cycle %llu\n", avr->cycle);
  }
  last_cycle = avr->cycle;
  position = avr->cycle - start_cycle;

  // fprintf(stderr, "position: %lu %u\n", position, value);
  /* it takes 2400 cycles to program the bits.  That's 
     240 cycles per led. 80 cycles per color.
     10 cycles per bit. 8 bits per pixel. */
  /* the bit at a multiple of 10 is always 1; if the
     transition is at 2, then the bit is zero; if the
     transition is at 8, the bit is one. */
  /* this means that we really just need to set the matrix
     to zeroes, and if position % 10 == 7, set the bit at
     position / 10. */

  /* undoubtedly there is a clever way to do this. */
  /* I welcome the cleverness. */
  unsigned char bit, color, pixel;
  if(position % 10 != 2 && position % 10 != 0) {
    bit = position / 10;
    pixel = bit / 24;
    color = (bit % 24) / 8;
    Pixels[pixel][color] |= (1 << (7 - (bit % 8))); /* the zero'th output bit is the highest bit */
    // fprintf(stderr, "found a bit at %d!\n", position % 10);
  }
  
  if(position > 2390) {
    printf("pixel dump at cycle %llu: ", avr->cycle);
    for(pixel=0; pixel<10; pixel++) {
      for(color=0; color<3; color++) {
        /* TODO: consider RGB for diminished confusion? */
        printf("%02x", Pixels[pixel][color]);
      }
      printf(" ");
    }
    printf("\n");
  }

}


int main(int argc, char *argv[])
{
	elf_firmware_t f;
	const char * fname;
    unsigned long step, step_limit = F_CPU * 15; /* 15 seconds, 120,000,000 */
    struct timeval start_time, end_time, delta_t;

    if (argc > 1) { 
      fname = argv[1]; //  "atmega48_ledramp.axf";
    } else {
      fprintf(stderr, "need firmware to test as first argument\n");
      exit(EXIT_FAILURE);
    }

	elf_read_firmware(fname, &f);


    if(f.mmcu[0] == '\0') { 
      fprintf(stderr, "Failed to find simavr_tracing.h defined processor infomation in %s; using default atmega32u4 at 8Mhz\n", fname);
      strcpy(f.mmcu, "atmega32u4");
      f.frequency = F_CPU;
    } else  {
      fprintf(stderr, "firmware %s f=%d mmcu=%s\n", fname, (int)f.frequency, f.mmcu);
    }

	avr = avr_make_mcu_by_name(f.mmcu);
	if (!avr) {
		fprintf(stderr, "%s: AVR '%s' not known\n", argv[0], f.mmcu);
		exit(1);
	}
	avr_init(avr);
	avr_load_firmware(avr, &f);

    /* register for a callback whenever the red led changes
       state, in order to count how often it toggles.. */
    avr_irq_register_notify( avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('C'), 7),
                             led_changed_hook, 
                             NULL);
    avr_irq_register_notify( avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 0),
                             neopixel_changed_hook, 
                             NULL);

	// even if not setup at startup, activate gdb if crashing
	avr->gdb_port = 1234;
	if (0) {
		avr->state = cpu_Stopped;
		avr_gdb_init(avr);
	}

    /* gets the B pins */
	avr_vcd_init(avr, "gtkwave_output-B.vcd", &vcd_output_file, 100000 /* usec */);
	avr_vcd_add_signal(&vcd_output_file, 
		avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), IOPORT_IRQ_PIN_ALL), 8 /* bits */ ,
		"portb" );

    struct stat dummy_stat;
    if(stat("gtkwave_input.vcd", &dummy_stat) == 0) {
      printf("avr_vcd_init_input returned %d\n",
             avr_vcd_init_input(avr, "gtkwave_input.vcd", &vcd_input_file) );
    }

    /* something for setting the analog light value */
    /* this is from one of the tests. */
    /* the schematic shows A5 on PF0, ADC0 */
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ACOMP_GETIRQ, ACOMP_IRQ_ADC0), 3000); 

    avr_irq_register_notify(avr_io_getirq( avr, AVR_IOCTL_ADC_GETIRQ,
                                           ADC_IRQ_OUT_TRIGGER ),
                            adc_hook,
                            NULL); 

    /* nstodo: investigate using built in
       avr_cycle_timer_register() to set an expiration time
       within a general mechanism */
    gettimeofday(&start_time, NULL);

    int state = cpu_Running;
    for(step=0;
        step < step_limit && state != cpu_Done && state != cpu_Crashed;
        step++) { 
		state = avr_run(avr);
    }

    gettimeofday(&end_time, NULL);
    timersub(&end_time, &start_time, &delta_t);

    fprintf(stderr,
            "simulation terminated after %lu steps, %lu.%06d seconds\n",
            step, delta_t.tv_sec, delta_t.tv_usec);
    fprintf(stderr,
            "led_flipped_count: %d\n",
            led_flipped_count);
}


/* cruft from https://groups.google.com/forum/#!topic/simavr/S0hHPd6Y99Q 
avr_irq_register_notify(
                        avr_io_getirq( Avr, AVR_IOCTL_ADC_GETIRQ,
                                       ADC_IRQ_OUT_TRIGGER ),
                        adc_hook,
                        this);

m_adc_irq = avr_alloc_irq(0, 1);
avr_connect_irq( m_adc_irq, avr_io_getirq( AvrProcessor,
                                           AVR_IOCTL_ADC_GETIRQ, m_pin ) );
*/

