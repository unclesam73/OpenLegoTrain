#ifndef HAL_H_
#define HAL_H_

#include <avr/io.h>
#include "openpf.h"

#define ExternalInterruptLowLevel	0b00
#define ExternalInterruptHighLevel	0b01
#define ExternalInterruptFalling	0b10
#define ExternalInterruptRising		0b11

#define TIMER_105US   TIMER0_COMPA_vect
#define IR_INTERRUPT  INT0_vect

#define RESET_IR_TIMER    TCNT0 = 0

extern volatile uint8_t externalint;
extern volatile uint8_t timerflag105us;

void SetupExternalInterrupt(uint8_t mode);
void Setup105usclock(); //105us clock

//#define DISABLE_IR_INT   GIMSK &= ~(_BV(INT0))
#define ENABLE_IR_INT    EIMSK |= (1 << INT0);
#define RESET_IR_TIMER    TCNT0 = 0
//GIMSK |= _BV(INT0)

ISR(IR_INTERRUPT, ISR_NOBLOCK) //External Interrupt Handler
{    
	//DISABLE_IR_INT; //Disable pin change interrupt. Enabled in timer interrupt routine.    
 //RESET_IR_TIMER    TCNT0 = 0
	RESET_IR_TIMER;    
 
	externalint = 1;
}

ISR(TIMER_105US, ISR_NOBLOCK)
{
    OpenPfRx105usState();
    timerflag105us = 1;
}

#endif

