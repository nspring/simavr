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
#include <inttypes.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <assert.h>
#include <errno.h>

#include "sim_avr.h"
#include "avr_ioport.h"
#include "sim_elf.h"
#include "sim_gdb.h"
#include "sim_vcd_file.h"
#include "avr_adc.h"
#include "avr_acomp.h"
#include "avr_uart.h"
#include "avr_twi.h"

#include "commando.h"

#define F_CPU 8000000U

/* command-line configurable parameters */
static boolean disable_neopixel;
static boolean disable_statistics;
static FILE *neopixel_log_fp;
static boolean enable_gdb_on_crash;
static boolean enable_gdb_now;
static int set_log_level = 1;

static void open_neopixel_log(const char *str_arg, /*@unused@*/ void *a) {
  neopixel_log_fp = fopen(str_arg,"w");
  if(!neopixel_log_fp) {
    fprintf(stderr, "unable to open %s as neopixel log; reverting to stdout\n", str_arg);
    neopixel_log_fp = stdout;
  }
}

/* probably should make parallel with open_neopixel_log */
static char *i2c_log_filename = NULL;
static FILE *i2c_file = NULL;
static void set_i2c_log_filename(const char *trace_filename, void *dummy /*@unused@*/) {
  i2c_log_filename = strdup(trace_filename);
}

static const char * _twi216_irq_names[2] = {
		[TWI_IRQ_INPUT] = "8>twi216.out",
		[TWI_IRQ_OUTPUT] = "32<twi216.in",
};
static struct { 
  avr_irq_t *irq;
  uint8_t addr_base;
  uint8_t addr_mask;
} twi_listener;

void twi216_in_hook(struct avr_irq_t * irq, uint32_t value, void * param) {
  /* this does get invoked.  Unclear if better than twi_changed_hook */
  /* fprintf(stderr, "twi216 in hook\n"); */
}

/* this twi_listener is modeled after i2c_eeprom */
static void twi_listener_init(struct avr_t *avr,
                              uint8_t addr,
                              uint8_t mask) {
  twi_listener.irq = avr_alloc_irq(&avr->irq_pool, 0,2,_twi216_irq_names);
  avr_irq_register_notify(twi_listener.irq + TWI_IRQ_OUTPUT, twi216_in_hook, &twi_listener);
  avr_connect_irq(twi_listener.irq + TWI_IRQ_INPUT,
                  avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT));
  avr_connect_irq(avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_OUTPUT),
                  twi_listener.irq + TWI_IRQ_OUTPUT );
}

static void twi_ack(struct avr_irq_t *irq, uint8_t addr) { 
  avr_raise_irq(irq - TWI_IRQ_OUTPUT + TWI_IRQ_INPUT,
                avr_twi_irq_msg(TWI_COND_ACK, addr, 1));
}

static void twi_sendit(struct avr_t *avr) {
  avr_irq_t *irq = avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT);
  unsigned int i, d=20;
  fprintf(stderr, "i2c start from simavr-216\n");
  avr_raise_irq(irq, avr_twi_irq_msg(TWI_COND_START | TWI_COND_WRITE | TWI_COND_ADDR, 12 << 1, 1));

  /* TODO: more nicely handle receipt of ACK and transmission of the thing to send */

  /* 
  for(i=0;i<d;i++) avr_run(avr);
  fprintf(stderr, "i2c write from simavr-216\n");
  avr_raise_irq(irq, avr_twi_irq_msg(TWI_COND_WRITE | TWI_COND_ADDR, 11, 66));
  for(i=0;i<d;i++) avr_run(avr);
  fprintf(stderr, "i2c stop from simavr-216\n");
  avr_raise_irq(irq, avr_twi_irq_msg(TWI_COND_STOP | TWI_COND_ADDR, 11, 1));
  */
}

unsigned long long twi_send_cb(struct avr_t *avr, unsigned long long ull, void *p) {
  twi_sendit(avr);
  return 0;
}

/* this twi_changed_hook method is modeled after other change hooks. */
void twi_changed_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
  /* decode the value through an avr_twi_msg_irq_t */
  avr_twi_msg_irq_t v;
  uint8_t address;
  struct avr_t *avr = (struct avr_t *)param;

  v.u.v = value;

  /* if our simulated sytem employs i2c, 1) ack I2C writes (can't 
     easily simulate i2c reads), 2) log I2C activity. */

  /* the address provided in the message is the shifted left
     version that is in the twdr register where the
     low bit would have been read/write (but the low bit is
     removed?)  */
  address = v.u.twi.addr >> 1;

  /* uncomment - handy for debugging to keep stderr/stdout in line with the log */
  /*  i2c_file = stderr;  */
  if(!i2c_file && i2c_log_filename) {
    i2c_file = fopen(i2c_log_filename, "w");
    if(!i2c_file) {
      fprintf(stderr, "unable to open %s for writing: %s\n", i2c_log_filename,
              strerror(errno));
      exit(1);
    }
  }


  /* Can be STOP, START, WRITE (don't support READ) */
  if (v.u.twi.msg & TWI_COND_STOP) {
    fprintf(i2c_file, "%llu twi STOP %d %d %d\n", avr->cycle, v.u.twi.msg, address,
            v.u.twi.data);
    /* stop is not ack'd */
  } else if (v.u.twi.msg & TWI_COND_START) {
    fprintf(i2c_file, "%llu twi START %d %d %d\n", avr->cycle, v.u.twi.msg, address,
            v.u.twi.data
            );
    twi_ack(irq, v.u.twi.addr);
  } else if (v.u.twi.msg & TWI_COND_WRITE) {
    fprintf(i2c_file, "%llu twi WRITE %d %d\n", avr->cycle, address, v.u.twi.data);
    twi_ack(irq, v.u.twi.addr);
  } else if (v.u.twi.msg & TWI_COND_READ) {
    fprintf(i2c_file, "%llu twi READ %x %d %d\n", avr->cycle, value,  address, v.u.twi.data);
    //     twi_ack(irq, v.u.twi.addr);

#define BUILD(x,y) (((x)<<17) | (y))
    /* part of trying to send via i2c to the chip */
    if(value == BUILD(12, 0x2400)) { 
      /* ack of start, not really an ack? */
      fprintf(stderr, "simavr-216 sending data byte start (a)?\n");
      avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT),
                    avr_twi_irq_msg(TWI_COND_WRITE | TWI_COND_ADDR, 12 << 1, 66));
    }
  } else {
    // fprintf(i2c_file, "%llu twi %x\n", avr->cycle, value);

    /* part of trying to send via i2c to the chip */
    if(value == BUILD(12, 0x2400) || value == BUILD(12,0x1000c00)) { 
      /* ack of start, not really an ack? */
      fprintf(stderr, "simavr-216 sending data byte start?\n");
      avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT),
                    avr_twi_irq_msg(TWI_COND_WRITE | TWI_COND_ADDR, 12 << 1, 66));
    } else if(value == BUILD(12,0x42002800) || value == BUILD(12, 0x42000c00) ) { 
      /* ack of data?  or straight up read command  */
      fprintf(stderr, "sending stop?\n");
      /* adding WRITE seems to keep the avr_twi in the right mode. */
      /* leavig it off gets the stop to complete and allow writing */
      avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT),
                    avr_twi_irq_msg(TWI_COND_STOP | TWI_COND_WRITE | TWI_COND_ADDR, 12 << 1, 1));
    }
  }
  /* print the ACK after START/WRITE/READ */
  if (v.u.twi.msg & TWI_COND_ACK) {
    fprintf(i2c_file, "%llu twi ACK %d %d %d\n", avr->cycle, v.u.twi.msg, address,
            v.u.twi.data);
  }
    
  fflush(i2c_file); /* to permit debuging */
}

static void print_version_and_exit(/*@unused@*/ const char *dummy, /*@unused@*/ void *a) {
  printf("simavr-216 version 0.1.1 compiled on " __DATE__ "\n");
  exit(0);
}

static struct commandos commands[] = {
  { "Disable neopixel printing",
    "disable-neopixel", 'N', no_argument,
    commando_boolean, &disable_neopixel },
  { "Disable statistics (cycles taken, led flip count)",
    "disable-statistics", 'S', no_argument,
    commando_boolean, &disable_statistics },
  { "Dump neopixel trace to file named by argument",
    "neopixel-file", 'n', required_argument,
    open_neopixel_log, NULL },
  { "Enable gdb on illegal instruction / crash",
    "gdb-crash", 'G', no_argument,
    commando_boolean, &enable_gdb_on_crash },
  { "Enable gdb immediately",
    "gdb", 'g', no_argument,
    commando_boolean, &enable_gdb_now },
  { "Set AVR log level, 0 is none, 4 is max",
    "log-level", 'l', required_argument,
    commando_int, &set_log_level },
  { "Dump i2c write trace to file named by argument",
    "i2c-file", '2', required_argument,
    set_i2c_log_filename, NULL }, 
  { "Show version",
    "version", 'V', no_argument,
    print_version_and_exit, NULL },
  { HELP_COMMANDO(commands) },
  { END_COMMANDO }
};

static avr_t * avr = NULL;
static avr_vcd_t vcd_output_file;
static avr_vcd_t vcd_input_file;
static uint8_t	pin_state = 0;	// current port B

/* Custom logger function doesn't print random stuff to
   stdout / stderr.  log 0 goes to stdout, but initial 
   stuff while loading the firmware goes nowhere. */
static void simavr_216_logger(avr_t * avr, const int level,
                              const char * format, va_list ap) {
  /* before avr is set, use the global log level in this file. */
  if (!avr && level <= set_log_level) {
    vfprintf((level > 0) ? stderr : stdout, format, ap);
  } else if (avr && avr->log >= level) {
    /* after avr is set, use the avr->log level, likely identical. */
    vfprintf((level > 0) ? stderr : stdout, format, ap);
  }
}

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
static void led_changed_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
  static unsigned char led_is_on;
  if(value != led_is_on) led_flipped_count++;
  led_is_on = value;
}

void adc_hook(struct avr_irq_t * irq, uint32_t value, void * param)
{
  printf("adc is hooked!\n");
}

static void neopixel_changed_hook(struct avr_irq_t * irq, uint32_t value, void * param) {
  static avr_cycle_count_t start_cycle, last_cycle;
  unsigned long position;
  static unsigned char Pixels[10][3];

  /* if we haven't seen a transition for 200 cycles, assume
     we're starting over.  Even on init, we only blit zeroes
     out at cycle 255 after the pin is set to output
     low.. */
  if(avr->cycle > last_cycle + 100) {
    /* should make sure this is a low to high transition */
    if(value != 1) {
      if (last_cycle != 0) {
        fprintf(stderr, "unexpected high to low transition on neopixel pin, %llu cycles after low to high\n", (unsigned long long)(avr->cycle - last_cycle));
      } /* else this is the first transition, setting to low initially */
      return;
    }
    start_cycle = avr->cycle;
    memset(Pixels, 0, sizeof(Pixels));
    // fprintf(stderr, "starting pixel set at cycle %llu\n", avr->cycle);
  }
  last_cycle = avr->cycle;
  position = avr->cycle - start_cycle;

  if(position > 2400) {
    fprintf(stderr, "lost sync with neopixel signal, likely due to extra signal on pin 17 from a conflicting library\n");
    return;
  }

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
    fprintf(neopixel_log_fp, "pixel dump at cycle %llu: ", (unsigned long long)avr->cycle);
    for(pixel=0; pixel<10; pixel++) {
      for(color=0; color<3; color++) {
        /* TODO: consider RGB for diminished confusion? */
        fprintf(neopixel_log_fp, "%02x", Pixels[pixel][color]);
      }
      fprintf(neopixel_log_fp, " ");
    }
    fprintf(neopixel_log_fp, "\n");
  }

}

int main(int argc, char *argv[])
{
	elf_firmware_t f;
	const char * fname;
    unsigned long step, step_limit = F_CPU * 15; /* 15 seconds, 120,000,000 */
    struct timeval start_time, end_time, delta_t;
    int zsuccess;
    int fname_arg_index;

    neopixel_log_fp = stdout; /* default, but not compile time constant */
    avr_global_logger_set(simavr_216_logger);

    fname_arg_index = commando_parse(argc,argv,commands);

    if (argc > fname_arg_index) { 
      fname = argv[fname_arg_index]; 
    } else {
      fprintf(stderr, "need firmware to test as last argument\n");
      exit(EXIT_FAILURE);
    }

    if((zsuccess = elf_read_firmware(fname, &f)) != 0) {
      fprintf(stderr, "elf_read_firmware returned nonzero code: %d\n", zsuccess);
      exit(1);
    }

    if(f.mmcu[0] == '\0') { 
      AVR_LOG(avr, LOG_WARNING, "Failed to find simavr_tracing.h defined processor infomation in %s; using default atmega32u4 at 8Mhz\n", fname);
      strcpy(f.mmcu, "atmega32u4");
      f.frequency = F_CPU;
    } else  {
      AVR_LOG(avr, LOG_DEBUG, "firmware %s f=%d mmcu=%s\n", fname, (int)f.frequency, f.mmcu);
    }

	avr = avr_make_mcu_by_name(f.mmcu);
	if (!avr) {
		fprintf(stderr, "%s: AVR '%s' not known\n", argv[0], f.mmcu);
		exit(1);
	}
	avr_init(avr);
    avr->log = set_log_level;
	avr_load_firmware(avr, &f);

    /* register for a callback whenever the red led changes
       state, in order to count how often it toggles.. */
    if(!disable_statistics) {
      avr_irq_register_notify( avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('C'), 7),
                               led_changed_hook, 
                               NULL);
    }

    if(!disable_neopixel) {
      avr_irq_register_notify( avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 0),
                               neopixel_changed_hook, 
                               NULL);
    }

    if(enable_gdb_on_crash) { 
      avr->gdb_port = 1234;
    } else { 
      /* zero should cause avr_sadly_crashed to not call avr_gdb_init */
      avr->gdb_port = 0;
    }

    /* gets the B pins */
#ifdef UNUSED_REALLY
	avr_vcd_init(avr, "gtkwave_output-B.vcd", &vcd_output_file, 100000 /* usec */);
	avr_vcd_add_signal(&vcd_output_file, 
		avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), IOPORT_IRQ_PIN_ALL), 8 /* bits */ ,
		"portb" );
#endif

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

    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_OUTPUT),
                            twi_changed_hook, avr);

    /* TWI_vect_num, defined in iom32u4 
    for (int vi = 0; vi < avr->interrupts.vector_count; vi++)
      if(avr->interrupts.vector[vi]->vector == 36)
        avr->interrupts.vector[vi]->trace = 1;
    */
    twi_listener_init(avr,11,3);

    if(enable_gdb_now) {
      avr->gdb_port = 1234;
      avr->state = cpu_Stopped;
      avr_gdb_init(avr);
    }

    avr_cycle_timer_register(avr, 150000, twi_send_cb, NULL);


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

    if(!disable_statistics) { 
      /* avr->cycles is ultimately a uint64_t, which may not be a  */
      fprintf(stderr,
              "simulation terminated after %" PRIu64 " cycles, %lu steps, %lu.%06d real seconds\n",
              avr->cycle, step,
              (unsigned long)delta_t.tv_sec, (int)delta_t.tv_usec);
      fprintf(stderr,
              "led_flipped_count: %d\n",
              led_flipped_count);
    }
}
