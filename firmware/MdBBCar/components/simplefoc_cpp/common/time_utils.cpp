#include "time_utils.h"
#include "rtthread.h"
#include "bsp_tick.h"

// function buffering delay()
// arduino uno function doesn't work well with interrupts
void _delay(unsigned long ms){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__)  || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
  // if arduino uno and other atmega328p chips
  // use while instad of delay,
  // due to wrong measurement based on changed timer0
  unsigned long t = _micros() + ms*1000;
  while( _micros() < t ){};
#else
  // regular micros
  // rt_thread_mdelay(ms);
  // delay(ms);
  bsp_tick_delay_ms(ms);
#endif
}

#include "board.h"
#include "bsp_foc.h"

// function buffering _micros()
// arduino function doesn't work well with interrupts
unsigned long _micros(){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__)  || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
// if arduino uno and other atmega328p chips
    //return the value based on the prescaler
    if((TCCR0B & 0b00000111) == 0x01) return (micros()/32);
    else return (micros());
#else
  // regular micros
  return bsp_tick_get_us();
  // return board_get_cur_us();
  // return (unsigned long)bsp_foc_cur_us();
  // return micros();
#endif
}
