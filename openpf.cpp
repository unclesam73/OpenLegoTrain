 //Copyright (c) 2012, vsluiter <info-at- hackvandedam.nl>
//
//Permission to use, copy, modify, and/or distribute this software for any
//purpose with or without fee is hereby granted, provided that the above
//copyright notice and this permission notice appear in all copies.
//
//THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
//WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
//AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
//INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
//OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
//TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
//OF THIS SOFTWARE.

#include "openpf.h"
//#include <avr/io.h>

//TODO: richting verandering bij numerical PWM
struct OpenPfRx_receiver OpenPfRx_rx;

#define OpenPfRx_TIMEOUTMARGIN 10

uint8_t OpenPfRxOutputModePwmLUT[16][2]=  //Used for single output mode (pwm) and for combopwm mode
{
    {PWM_OFF  ,OM_FLOAT},
    {PWM_STEP1,OM_FWD},
    {PWM_STEP2,OM_FWD},
    {PWM_STEP3,OM_FWD},
    {PWM_STEP4,OM_FWD},
    {PWM_STEP5,OM_FWD},
    {PWM_STEP6,OM_FWD},
    {PWM_STEP7,OM_FWD},
    {PWM_BRAKE_THEN_FLOAT,OM_BRAKE_THEN_FLOAT},
    {PWM_STEP7,OM_BWD},
    {PWM_STEP6,OM_BWD},
    {PWM_STEP5,OM_BWD},
    {PWM_STEP4,OM_BWD},
    {PWM_STEP3,OM_BWD},
    {PWM_STEP2,OM_BWD},
    {PWM_STEP1,OM_BWD},
};

uint8_t OpenPfRxComboDirectModeLUT[4][2]=
{
    {PWM_FLOAT,OM_FLOAT},
    {PWM_FULL ,OM_FWD},
    {PWM_FULL ,OM_BWD},
    {PWM_BRAKE_THEN_FLOAT,OM_BRAKE_THEN_FLOAT}
};

uint8_t OpenPfRx_pwmvalues[] = {   OpenPfRx_MIN_PWM_VALUE,
                                   OpenPfRx_PWM_VALUE_STEP,     //PWM_STEP1
                                   (OpenPfRx_PWM_VALUE_STEP*2), //PWM_STEP2
                                   (OpenPfRx_PWM_VALUE_STEP*3), //PWM_STEP3
                                   (OpenPfRx_PWM_VALUE_STEP*4), //PWM_STEP4
                                   (OpenPfRx_PWM_VALUE_STEP*5), //PWM_STEP5
                                   (OpenPfRx_PWM_VALUE_STEP*6), //PWM_STEP6
                                   (OpenPfRx_PWM_VALUE_STEP*7), //PWM_STEP7
                                   OpenPfRx_MAX_PWM_VALUE
                               };

//#define TIMEOUT_MARGIN 200
void OpenPfRx_channel_init(struct OpenPfRx_channel *channel, uint8_t number)
{
    channel->channel_number    = number;
    channel->A.pwmvalue        = PWM_OFF;
    channel->B.pwmvalue        = PWM_OFF;
    channel->A.pwmindex        = (OpenPfRx_PWM_INDEX)0;
    channel->B.pwmindex        = (OpenPfRx_PWM_INDEX)0;
    channel->A.output_mode     = OM_FLOAT;
    channel->B.output_mode     = OM_FLOAT;
    channel->A.brakethenfloatcount = 0;
    channel->B.brakethenfloatcount = 0;
    channel->timeout_limit = 11428;
    channel->timeout_action    = 0;
    channel->toggle            = 0;
}

uint8_t OpenPfRxVerifyChecksum(uint16_t data)
{
    uint8_t counter, lrc;
    uint16_t temp;
    temp = 0;
    lrc = data & 0x0F;
    for(counter=1 ; counter < 4 ; counter++)
    {
        temp ^= (data >> (counter*4));
    }
    temp ^= 0xF;
    if(((uint8_t)(temp&0x0F)) == lrc)
        return 1;
    return 0;
}

void OpenPfRxPinInterruptState(void)
{
    switch(OpenPfRx_rx.state)
    {
    case IDLE:
    {
        OpenPfRx_rx.state = WAIT_FOR_START;
        break;
    }
    case WAIT_FOR_START:
    {
        if(OpenPfRx_rx.periodcounter >= IR_LENGTH_START_STOP) //if no flanks have been detected between IDLE and START_STOP*105us
        {
            OpenPfRx_rx.state = WAIT_FOR_BIT;       //start reading data
            OpenPfRx_rx.bit_count = 0;
            OpenPfRx_rx.rxdata = 0;
        }
        else
            OpenPfRx_rx.state = IDLE;               //reset statemachine if unexpected flank is detected
        break;
    }
    case WAIT_FOR_BIT:
    {
        if( (OpenPfRx_rx.periodcounter >= IR_LENGTH_LO) && (OpenPfRx_rx.periodcounter <= IR_LENGTH_START_STOP) ) //if pulse length within expected datalength
        {
            if(OpenPfRx_rx.periodcounter > IR_LENGTH_HI)
                OpenPfRx_rx.rxdata |= 1 ; //set bit in correct position
            OpenPfRx_rx.bit_count++;
            if(OpenPfRx_rx.bit_count == 16) //last bit
                OpenPfRx_rx.state = WAIT_FOR_STOP;
            else
                OpenPfRx_rx.rxdata = (OpenPfRx_rx.rxdata << 1) & 0xFFFE;
        }
        else
            OpenPfRx_rx.state = IDLE;
        break;
    }
    case WAIT_FOR_STOP: //during Stop bit no IR data should be received. STOP bit length is monitored by timer interrupt function.
    {
        OpenPfRx_rx.state = IDLE;
        break;
    }
    default:
    {
        OpenPfRx_rx.state = IDLE;
    }
    }
    OpenPfRx_rx.periodcounter = IR_LENGTH_INIT;
}



void OpenPfRxInterpreter(const uint16_t * rxdata, struct OpenPfRx_channel *channel)
{
    if(*rxdata & OpenPfRx_ESCAPE_MASK)     //if Extended mode because 'E' bit is set
    {
        OpenPfRxComboPWMMode(rxdata, channel);
    }
    else
    {
        uint8_t mode;
        mode = (*rxdata & OpenPfRx_MODE_MASK) >> 8;
        if(mode == OpenPfRx_EXTENDED_MODE)
            OpenPfRxExtendedMode(rxdata, channel);
        else if(mode == OpenPfRx_COMBO_DIRECT_MODE)
            OpenPfRxComboDirectMode(rxdata, channel);
        else if(mode & OpenPfRx_SINGLE_OUTPUT_MODE)
            OpenPfRxSingleOutputMode(rxdata, channel);
        else if(mode == OpenPfRx_RESERVED_1)
            return;
        else if(mode == OpenPfRx_RESERVED_2)
            return;
    }
}

void OpenPfRxComboPWMMode(const uint16_t * rxdata, struct OpenPfRx_channel *channel)
{
    //use lookup table for pwm mode
    uint8_t dataA, dataB;
    dataA = (*rxdata & OpenPfRx_OUTPUTA_MASK) >> 4;
    dataB = (*rxdata & OpenPfRx_OUTPUTB_MASK) >> 8;
    channel->timeout_action = 1;
    //channel->timeout = channel->timeout_limit;
    channel->A.pwmindex    = (OpenPfRx_PWM_INDEX)(OpenPfRxOutputModePwmLUT[dataA][0]);
    channel->A.output_mode = (OpenPfRx_OUTPUT_MODES)(OpenPfRxOutputModePwmLUT[dataA][1]);
    channel->A.pwmvalue    = OpenPfRx_pwmvalues[channel->A.pwmindex];
    channel->B.pwmindex    = (OpenPfRx_PWM_INDEX)(OpenPfRxOutputModePwmLUT[dataB][0]);
    channel->B.output_mode = (OpenPfRx_OUTPUT_MODES)(OpenPfRxOutputModePwmLUT[dataB][1]);
    channel->B.pwmvalue    = OpenPfRx_pwmvalues[channel->B.pwmindex];
}

void OpenPfRxExtendedMode(const uint16_t * rxdata, struct OpenPfRx_channel *channel)
{
    //Data determines function used
    //Toggle bit is verified
    //No timeout
    uint8_t function;
    function = (*rxdata & OpenPfRx_DATA_MASK) >> 4;
    if(!OpenPfRxVerifyToggleBit(rxdata,channel))
        return;    //skip function if toggle bit is not different than previous message. Timeout not observed.
    switch(function)
    {
    case 0b0000: //brake then float   output A
    {
        channel->timeout_action= 0;
        channel->A.pwmindex    = PWM_BRAKE_THEN_FLOAT;
        channel->A.pwmvalue    = OpenPfRx_pwmvalues[channel->A.pwmindex];
        channel->A.output_mode = OM_BRAKE_THEN_FLOAT;
        break;
    }
    case 0b0001: //increment speed on output A
    {
        channel->timeout_action = 0;
        if(channel->A.pwmindex < PWM_FULL)
        {
            channel->A.pwmindex = (OpenPfRx_PWM_INDEX)((int)channel->A.pwmindex+1);
            channel->A.pwmvalue = OpenPfRx_pwmvalues[channel->A.pwmindex];
            if(!(channel->A.output_mode == OM_BWD || channel->A.output_mode == OM_FWD )) //If output not already delivering power
                channel->A.output_mode = OM_FWD;
        }
        break;
    }
    case 0b0010: //decrement next_speed on output A
    {
        channel->timeout_action = 0;
        if(channel->A.pwmindex > PWM_OFF)
        {
            channel->A.pwmindex = (OpenPfRx_PWM_INDEX)((int)channel->A.pwmindex-1);
            channel->A.pwmvalue = OpenPfRx_pwmvalues[channel->A.pwmindex];
        }
        break;
    }
    case 0b0100: //Toggle forward/float on output B
    {
        channel->timeout_action = 0;
        if(channel->B.pwmindex == PWM_FLOAT)
        {
            channel->B.pwmindex = PWM_FULL;
            channel->B.pwmvalue = OpenPfRx_pwmvalues[channel->B.pwmindex];
            channel->B.output_mode = OM_FWD;
        }
        else
        {
            channel->B.pwmindex = PWM_FLOAT;
            channel->B.pwmvalue = OpenPfRx_pwmvalues[channel->B.pwmindex];
            channel->B.output_mode = OM_FLOAT;
        }
        break;
    }
    case 0b0110: //toggle address bit
    {
        uint8_t temp;
        temp= channel->channel_number;
        if(temp & 0b100)
            temp = temp & 0b0011;
        else
            temp =  temp | 0b100;
        //OpenPfRx_channel_init(channel, temp);    //reset channel  //TODO: opnieuw enablen
        OpenPfRx_channel_init(channel, temp);    //reset channel
        break;
    }
    case 0b0111: //align toggle bit; already done in intro
    {
        break;
    }
    default:
        OpenPfRx_channel_init(channel, channel->channel_number);    //reset channel
    }

}

void OpenPfRxComboDirectMode(const uint16_t * rxdata, struct OpenPfRx_channel *channel)
{
    // timeout
    // toggle bit not verified
    // *LEGO8879 uses this mode when both red buttons are pressed simultaneously
    uint8_t data, dataA, dataB;
    data = (*rxdata & OpenPfRx_DATA_MASK) >> 4;
    dataA = data&0x03;
    dataB = (data >> 2) & 0x03;
    channel->timeout_action = 1;
    channel->A.pwmindex    = (OpenPfRx_PWM_INDEX)OpenPfRxComboDirectModeLUT[dataA][0];
    channel->A.output_mode = (OpenPfRx_OUTPUT_MODES)OpenPfRxComboDirectModeLUT[dataA][1];
    channel->A.pwmvalue    = OpenPfRx_pwmvalues[channel->A.pwmindex];
    channel->B.pwmindex    = (OpenPfRx_PWM_INDEX)OpenPfRxComboDirectModeLUT[dataB][0];
    channel->B.output_mode = (OpenPfRx_OUTPUT_MODES)OpenPfRxComboDirectModeLUT[dataB][1];
    channel->B.pwmvalue    = OpenPfRx_pwmvalues[channel->B.pwmindex];
}

void OpenPfRxSingleOutputMode(const uint16_t * rxdata,  struct OpenPfRx_channel *channel)
{
// Protocol used by LEGO 8879 remote control (inc/dec pwm, break then float)
// Toggle bit verified for increment / decrement / toggle
// Timeout ONLY for full forward and full backward
    uint8_t DDDD; //data value
    uint8_t verify_toggle;
    struct OpenPfRx_output * target_output;
    if(*rxdata & OpenPfRx_SINGLEOUTPUT_OUTPUT_MASK)
        target_output = &(channel->B);
    else
        target_output = &(channel->A);
    verify_toggle = OpenPfRxVerifyToggleBit(rxdata,channel); //take care that toggle bit remains synchronized, even if it isn't used.
    DDDD = (*rxdata & OpenPfRx_SINGLEOUTPUT_DDDD_MASK)>>4;
    if(*rxdata & OpenPfRx_SINGLEOUTPUT_CSTIDMODE_MASK) //Clear/Set/Toggle/Inc/Dec mode
    {
        if(DDDD == 0110 || DDDD == 0111) //Full Forward and Full Backward have timeout
            channel->timeout_action = 1;
        else                             // others haven't
            channel->timeout_action = 0;
        switch(DDDD)
        {
        case 0b0000:        //Toggle Full Forward
        {
            //TODO: verify with lego behavior; I guess that from any output state except full forward this goes to Full FWD, and toggles to stop next press
            if(!verify_toggle)
                return;
            switch(target_output->output_mode)
            {
            case(OM_FWD):
            {
                target_output->pwmindex    = PWM_OFF;
                target_output->pwmvalue    = OpenPfRx_pwmvalues[target_output->pwmindex];
                target_output->output_mode = OM_BRAKE;
                break;
            }
            default:
            {
                target_output->pwmindex    = PWM_FULL;
                target_output->pwmvalue    = OpenPfRx_pwmvalues[target_output->pwmindex];
                target_output->output_mode = OM_FWD;
            }
            }
            break;
        }
        case 0b0001:        //Toggle Direction
        {
            if(!verify_toggle)
                return;
            switch(target_output->output_mode)
            {
            case(OM_FWD):
            {
                //do not change PWM value
                target_output->output_mode = OM_BWD;
                break;
            }
            case(OM_BWD):
            {
                //do not change PWM value
                target_output->output_mode = OM_FWD;
                break;
            }
            default:
                break; // do nothing if not driving, or independent.
            }

        }
        case 0b0010:        //Increment Numerical PWM
        {
            if(!verify_toggle)
                return;
            if(target_output->pwmvalue < OpenPfRx_MAX_PWM_VALUE)
                target_output->pwmvalue++;
            break;
        }
        case 0b0011:        //Decrement Numerical PWM
        {
            if(!verify_toggle)
                return;
            if(target_output->pwmvalue > OpenPfRx_MIN_PWM_VALUE)
                target_output->pwmvalue--;
            break;
        }
        case 0b0100:        //Increment PWM *LEGO8879
        {
            if(!verify_toggle)
                return;
            if((target_output->output_mode != OM_FWD) && (target_output->output_mode != OM_BWD)) //if floating, braking, etc.
            {
                target_output->output_mode = OM_FWD;    //set output mode on 'running forward'
                target_output->pwmindex    = PWM_OFF;   //set pwmindex to 'doing nothing'
            }
            if((target_output->pwmindex < PWM_FULL) && target_output->output_mode == OM_FWD)
            {
                target_output->pwmindex = (OpenPfRx_PWM_INDEX)((int)target_output->pwmindex+1); //increase speed forward
            }
            if((target_output->pwmindex > PWM_OFF)  && target_output->output_mode == OM_BWD)
            {
                target_output->pwmindex = (OpenPfRx_PWM_INDEX)((int)target_output->pwmindex-1); //decrease speed backward
            }
            else if((target_output->pwmindex == PWM_OFF)  && target_output->output_mode == OM_BWD)
            {
                //switch direction
                target_output->pwmindex = PWM_STEP1;
                target_output->output_mode = OM_FWD;
            }
            target_output->pwmvalue = OpenPfRx_pwmvalues[target_output->pwmindex]; //update also if not changing pwmindex; align to PWM_step if previously 'numerical' value was changed.
            break;
        }
        case 0b0101:        //Decrement PWM *LEGO8879
        {
            if(!verify_toggle)
                return;
            if((target_output->output_mode != OM_FWD) && (target_output->output_mode != OM_BWD))
            {
                target_output->output_mode = OM_BWD;
                target_output->pwmindex    = PWM_OFF;
            }
            if((target_output->pwmindex < PWM_FULL) && target_output->output_mode == OM_BWD)
            {
               target_output->pwmindex = (OpenPfRx_PWM_INDEX)((int)target_output->pwmindex+1); //increase speed backward
            }
            if((target_output->pwmindex > PWM_OFF)  && target_output->output_mode == OM_FWD)
            {
               target_output->pwmindex = (OpenPfRx_PWM_INDEX)((int)target_output->pwmindex-1); //decrease speed forward
            }
            else if((target_output->pwmindex == PWM_OFF)  && target_output->output_mode == OM_FWD)
            {
                //switch direction
                target_output->pwmindex = PWM_STEP1;
                target_output->output_mode = OM_BWD;
            }
            target_output->pwmvalue = OpenPfRx_pwmvalues[target_output->pwmindex];
            break;
        }
        case 0b0110:        //Full Forward (timeout)
        {
            target_output->pwmindex = PWM_FULL;
            target_output->pwmvalue = OpenPfRx_pwmvalues[target_output->pwmindex];
            target_output->output_mode = OM_FWD;
            break;
        }
        case 0b0111:        //Full Backward(timeout)
        {
            target_output->pwmindex = PWM_FULL;
            target_output->pwmvalue = OpenPfRx_pwmvalues[target_output->pwmindex];
            target_output->output_mode = OM_BWD;
            break;
        }
        case 0b1000:        //Toggle Full Forward / Backward (default Forward)
        {
            if(!verify_toggle)
                return;
            target_output->pwmindex = PWM_FULL;
            target_output->pwmvalue = OpenPfRx_pwmvalues[target_output->pwmindex];
            if(target_output->output_mode == OM_FWD)
                target_output->output_mode = OM_BWD;
            else
                target_output->output_mode = OM_FWD;
            break;
        }
        case 0b1001:        //Clear C1 (negative logic, C1 High)
        {
            //TODO: what happens to C2 after full forward / backward / brake? unknown....
            target_output->output_mode = OM_INDEPENDENT;
            target_output->C1          = 1;
            break;
        }
        case 0b1010:        //Set   C1 (negative logic, C1 Low)
        {
            //TODO: what happens to C2 after full forward / backward / brake? unknown....
            target_output->output_mode = OM_INDEPENDENT;
            target_output->C1          = 0;
            break;
        }
        case 0b1011:        //ToggleC1
        {
            if(!verify_toggle)
                return;
            if(target_output->C1)
                target_output->C1 = 0;
            else
                target_output->C1 = 1;
            target_output->output_mode = OM_INDEPENDENT;
        }
        case 0b1100:        //Clear C2 (negative logic, C2 High)
        {
            //TODO: what happens to C1 after full forward / backward / brake? unknown....
            target_output->output_mode = OM_INDEPENDENT;
            target_output->C2          = 1;
            break;
        }
        case 0b1101:        //Set   C2 (negative logic, C2 Low)
        {
            //TODO: what happens to C1 after full forward / backward / brake? unknown....
            target_output->output_mode = OM_INDEPENDENT;
            target_output->C2          = 0;
            break;
        }
        case 0b1110:        //ToggleC2
        {
            if(!verify_toggle)
                return;
            if(target_output->C2)
                target_output->C2 = 0;
            else
                target_output->C2 = 1;
            target_output->output_mode = OM_INDEPENDENT;
        }
        case 0b1111:        //Toggle Full Backward
        {
            if(!verify_toggle)
                return;
            switch(target_output->output_mode)
            {
            case(OM_BWD):
            {
                target_output->pwmindex    = PWM_OFF;
                target_output->pwmvalue    = OpenPfRx_pwmvalues[target_output->pwmindex];
                target_output->output_mode = OM_BRAKE;
                break;
            }
            default:
            {
                target_output->pwmindex    = PWM_FULL;
                target_output->pwmvalue    = OpenPfRx_pwmvalues[target_output->pwmindex];
                target_output->output_mode = OM_BWD;
            }
            }
            break;
        }
        default:
            return;
        }

    }
    else                                        //PWM Mode
    {
        target_output->pwmindex    = (OpenPfRx_PWM_INDEX)(OpenPfRxOutputModePwmLUT[DDDD][0]);
        target_output->output_mode = (OpenPfRx_OUTPUT_MODES)(OpenPfRxOutputModePwmLUT[DDDD][1]);
        target_output->pwmvalue    = OpenPfRx_pwmvalues[target_output->pwmindex];
        channel->timeout_action = 0;
    }
}

//returns 1 if toggle bit is valid (new message)
uint8_t OpenPfRxVerifyToggleBit(const uint16_t * rxdata, struct OpenPfRx_channel *channel)
{
    uint8_t toggle_bit = 0;
    if(*rxdata & OpenPfRx_TOGGLE_MASK) //toggle bit set at message
    {
        toggle_bit = 1;
    }
    if(toggle_bit == channel->toggle)
    {
        return 0;
    }
    channel->toggle = toggle_bit;
    return 1;
}

uint8_t OpenPfRxGetChannelNumber(uint16_t rxdata)
{
    uint8_t temp;
    temp =  ((rxdata & OpenPfRx_CHANNEL_MASK)>>12) ; //create address from CC bits
    if(rxdata & OpenPfRx_ADDRESS_MASK)
    {
        temp |= 0x04;                       // with 'a' bit as MSB. Preparation for address extention!
    }
    return temp;
}
