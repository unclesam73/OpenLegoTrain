#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "hal.h"
//#include "openpf.h"

#define IR_PIN 2
#define PIN_STATUS 13

volatile uint8_t externalint = 0;
volatile uint8_t timerflag105us = 0;
//uint8_t pwmport, servoatrip, servobtrip;
//uint16_t pwma,pwmb;

uint8_t channelnumber;//, servo_channel_mode;
struct OpenPfRx_channel channel_pwm;
//  struct OpenPfRx_channel channel_servo;
uint16_t legochannel[8] = {0,0,0,0,0,0,0,0}; //can be used to store retrieved data

static void UpdateOutputValues(struct OpenPfRx_output * );

void setup() {

  // put your setup code here, to run once:
  Serial.begin( 115000 );
  pinMode(PIN_STATUS, OUTPUT);
  
  IoInit();
  OpenPfRx_channel_init(  (struct OpenPfRx_channel *)&channel_pwm, 0);
//  while(BUTTON_PUSHED);
//  power_adc_disable();
//  power_usi_disable();
//  ocr1_mask_a = ocr1_mask_b = 0xFF;

  //Setup external Interrupt
  pinMode(IR_PIN, INPUT_PULLUP);
  cli();
  SetupExternalInterrupt(ExternalInterruptFalling);
  //Attach external Interrupt
  //attachInterrupt(digitalPinToInterrupt(IR_PIN, , FALL);
  //Enable external Interrupt
  ENABLE_IR_INT;
  SetupPWMTimer();
  ENABLE_PWMA_INTERRUPT;
  ENABLE_PWMB_INTERRUPT;

  Setup105usclock();
  OpenPfRx_rx.newdata = 0;
  
  sei();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//  Serial.println( TCCR0B, BIN );
//  delay(100);
}

void loop() {

        uint8_t c_sreg;

        if(OpenPfRx_rx.newdata)
        {
            OpenPfRx_rx.newdata = 0;
//OpenPfRx_rx.rxdata = 0xB646;
            if(OpenPfRxVerifyChecksum(OpenPfRx_rx.rxdata))                         //if Checksum is OK continue processing data
            {
              //Serial.println( OpenPfRx_rx.rxdata, BIN );
              channelnumber = OpenPfRxGetChannelNumber(OpenPfRx_rx.rxdata);
              //Serial.println( channelnumber, BIN );
              legochannel[channelnumber] = OpenPfRx_rx.rxdata;
              if(channelnumber == channel_pwm.channel_number)                 //if data is for own channel
              {
                uint8_t a_mask = 0xff,b_mask = 0xff;
                uint16_t a_pwm = 0xF0, b_pwm = 0xF0;
                uint8_t enablepwma = 1, enablepwmb  = 1;
                uint8_t temp_var= ~(A_C1|A_C2|B_C1|B_C2);
                channel_pwm.timeout = channel_pwm.timeout_limit;                       //reset timeout
                OpenPfRxInterpreter((const uint16_t *)&legochannel[channelnumber] , &channel_pwm);
//                      Serial.println( channel_pwm.A.output_mode, BIN );
//                      Serial.println( OM_FWD, BIN );
//                      Serial.println( OM_BWD, BIN );
////            channel_pwm.A.pwmvalue = 1;
                if (channel_pwm.A.output_mode == (OM_FWD) || channel_pwm.A.output_mode == (OM_BWD))
                {
//                      Serial.println( channel_pwm.A.pwmvalue, BIN );
//                      Serial.println( OpenPfRx_MIN_PWM_VALUE, BIN );
//                      Serial.println( OpenPfRx_MAX_PWM_VALUE, BIN );
                  if(channel_pwm.A.pwmvalue == OpenPfRx_MIN_PWM_VALUE || channel_pwm.A.pwmvalue == OpenPfRx_MAX_PWM_VALUE)
                  {
                    enablepwma = 0;
                  }
                }
                else
                  enablepwma = 0; //OM_BRAKE, OM_BRAKE_THEN_FLOAT, OM_INDEPENDENT, OM_FLOAT
                UpdateOutputValues(&channel_pwm.A); //Calculate new values for output A, C1 and C2
                if (channel_pwm.A.C1)
                  temp_var |= A_C1;
                if (channel_pwm.A.C2)
                  temp_var |= A_C2;
                c_sreg = SREG;
                if (enablepwma)
                {
                  a_mask = ~(A_C1|A_C2);
                  a_pwm = (channel_pwm.A.pwmvalue) <<1;
                }
                else
                {
                  a_mask = 0xff;
                  a_pwm = 0x30;
                }
                cli();
                pwma  = a_pwm;
                pwmb  = b_pwm;
                ocr1_mask_a = a_mask;
                ocr1_mask_b = b_mask;
                pwmport = temp_var;
                sei();
                SREG = c_sreg;
              }
            }
        }
        if(externalint) //check if external interrupt has taken place
        {
          externalint = 0; //clear flag
          OpenPfRxPinInterruptState(); //process counters / pin state to gather IR data
        }
        if(timerflag105us)
        {
            timerflag105us = 0;
            channel_pwm.A.brakethenfloatcount++;
            channel_pwm.B.brakethenfloatcount++;
            if(channel_pwm.timeout)
                --channel_pwm.timeout;
        }
        if(channel_pwm.timeout == 0 && channel_pwm.timeout_action)
        {
            ResetPWMChannel(&channel_pwm);
            //asm("nop");
        }

        if( (channel_pwm.A.output_mode == OM_BRAKE_THEN_FLOAT) && channel_pwm.A.brakethenfloatcount >= 2000)
        {
            channel_pwm.A.output_mode = OM_FLOAT;
            channel_pwm.A.pwmindex    = PWM_FLOAT;
            pwmport &= ~(A_C1|A_C2);
            ocr1_mask_a = 0xFF;

        }
/*        if( (channel_pwm.B.output_mode == OM_BRAKE_THEN_FLOAT) && channel_pwm.B.brakethenfloatcount >= 2000)
        {
            channel_pwm.B.output_mode = OM_FLOAT;
            channel_pwm.B.pwmindex    = PWM_FLOAT;
            pwmport &= ~(B_C1|B_C2);
            ocr1_mask_b = 0xFF;
        }
*/        
/*  if(OpenPfRx_rx.newdata)
  {
    OpenPfRx_rx.newdata = 0;
                channelnumber = OpenPfRxGetChannelNumber(OpenPfRx_rx.rxdata);      //Read channelnumber from received data
                legochannel[channelnumber] = OpenPfRx_rx.rxdata;
    digitalWrite(PIN_STATUS, HIGH);
    delay(10);
    digitalWrite(PIN_STATUS, LOW);
    Serial.println("OpenPfRx_rx.rxdata");
    Serial.println(OpenPfRx_rx.rxdata, BIN);
//    OpenPfRx_rx.rxdata
  }
  if(externalint) //check if external interrupt has taken place
  {
    externalint = 0; //clear flag
//    mtime1 = mtime2;
//    Serial.println("Before----");
//    Serial.println("OpenPfRx_rx.state");
//    Serial.println(OpenPfRx_rx.state, BIN);
//    Serial.println("OpenPfRx_rx.rxdata");
//    Serial.println(OpenPfRx_rx.rxdata, BIN);
//    Serial.println("OpenPfRx_rx.periodcounter");
//    Serial.println(OpenPfRx_rx.periodcounter, DEC);
    OpenPfRxPinInterruptState(); //process counters / pin state to gather IR data
//    Serial.println("-");
//    mtime1 = micros();
//    Serial.println( mtime1 );
//    Serial.println("OpenPfRx_rx.state");
//    Serial.println(OpenPfRx_rx.state, BIN);
//    Serial.println("OpenPfRx_rx.periodcounter");
//    Serial.println(OpenPfRx_rx.periodcounter, DEC);
*/
/*    Serial.println("OpenPfRx_rx.rxdata");
    Serial.println(OpenPfRx_rx.rxdata, BIN);
    Serial.println("OpenPfRx_rx.bit_count");
    Serial.println(OpenPfRx_rx.bit_count, BIN);
    Serial.println("OpenPfRx_rx.irstate");
    Serial.println(OpenPfRx_rx.irstate, BIN);
*/
// }

//  int i = externalint;
//  sei();
//  if (i == 1) 
//  {
//    cli();
//    externalint = 0; 
//    Serial.println( 1, BIN );
//    sei();
//    Serial.println( EIFR, BIN );
//    Serial.println( EIMSK, BIN );
//    Serial.println( EICRA, BIN );
//  }
}

static void UpdateOutputValues(struct OpenPfRx_output * output)
{
    switch(output->output_mode)
    {
    case OM_FWD:
    {
        if(output->pwmvalue != OpenPfRx_MIN_PWM_VALUE)
        {
            output->C1 = 1;
            output->C2 = 0;
        }
        else
        {
            //FLOAT
            output->C1 = 0;
            output->C2 = 0;
        }
        break;
    }
    case OM_BWD:
    {
        if(output->pwmvalue != OpenPfRx_MIN_PWM_VALUE)
        {
            output->C1 = 0;
            output->C2 = 1;
        }
        else
        {
            //FLOAT
            output->C1 = 0;
            output->C2 = 0;
        }
        break;
    }
    case OM_FLOAT:
    {
        output->C1 = 0;
        output->C2 = 0;
        break;
    }
    case OM_BRAKE_THEN_FLOAT:
        output->brakethenfloatcount= 0;
    case OM_BRAKE:
    {
        output->C1 = 1;
        output->C2 = 1;
        break;
    }
    case OM_INDEPENDENT:
    default:
        break;
    }
}

static void init_wdt()
{
    WDTCSR = (1<<WDIE)|(1<<WDCE)|(1<<WDE)|(1<<WDP1)|(1<<WDP0);//0.125s cycle, interrupt, no reset
}

