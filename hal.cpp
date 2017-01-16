#include <avr/io.h>
#include <avr/interrupt.h>
//#include "hal.h"
#define _NOP() asm("nop")

void SetupExternalInterrupt(uint8_t mode)
{
    /*Init pin interrupt*/
    //MCUCR = (MCUCR & 0xFC) | mode; //Falling edge INT0 generates interrupt; Table 9-2
//    EICRA &= ~(bit(ISC00) | bit (ISC01)); 
//    EICRA |= (1 << ISC10);
    EICRA &= ~3; 
    EICRA |= 2;
}

void Setup105usclock(void)
{
    /*Timer0 CTC mode, overrun at OCR0A, CLK/8*/
    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS01);        // CTC mode, TOP is OCR0A, CLK = CLKio/8
//    TCCR0B = _BV(CS01) | _BV(CS00);        // CTC mode, TOP is OCR0A, CLK = CLKio/64
    TCNT0  = 0;
    OCR0A  = 210; //105us
    TIMSK0 |= _BV(OCIE0A); //Output compare interrupt enabled
}

