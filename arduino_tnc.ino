/* 
    Arduino TNC
    
    This is a TNC based on Arduino hardware, intended primarily for APRS.
    Wherever possible, low-level AVR registers are used instead of the high-level Wiring
    libraries, in order to minimize the program size and maximize performance. This code
    is experimental, buggy, and is probably not suitable for any purpose. That said, it 
    is also small, fast, heavily commented, and might even decode a packet occasionally.

    This software is based heavily on the (far better) works of:
      Bob Bruninga, WB4APR
      Dennis Seguine
      Scott Miller, N1VG
      Gary Dion, N4TXI
      John Hansen, W2FS
      Thomas Sailer, HB9JNX/AE4WA
      
    Anyone wanting to use this code is strongly encouraged to:
      1. Reconsider.
      2. Buy a pre-made device from "argentdata.com" or "tncx.com" instead.
      3. At the very least, download the datasheet for the ATmega328P and/or ATmega1280.
      4. Bookmark "http://www.nongnu.org/avr-libc/".
    
    If you do find anything useful about this code, please let me know, at:   
    Kilo India Four Mike Charlie Whiskey at-sign Golf Mike Alpha India Lima Dot Com.    
    Robert Marshall, KI4MCW    
        
    Transmit code based on  Whereavr http://www.garydion.com/projects/whereavr/
    added by Kieran Levin, KC9BZY kieranlevin.com kilo india ralf alpha Mike 9 at-sign yahoo dot com
     
Version history:
20170102 (0.15.2) SQ9MDD clean up the code, reformating comments, delete not used variables, add compiler options, thanx to Luca SQ5RWU,
                  removed Mega option, some small fixes.
20150214 (0.15.0) SQ9MDD add watchdog
20150201 (0.14.1) SQ5RWU Checking for CRC in received data before sending it to host
20100602 (0.14) Kiss transmit added, increased buffer size changed uart speed to 19200 to match arduino bootloader, changed heartbeat off
                ADCREF is set to VCC, DISCONNECT AREF from power supply to prevent damage to chip and only connect external cap ~0.1uF
20100409 (0.13) Auto-bias, KISS output.
20100408 (0.12) Switch to Seguine math, handle Due vs Mega hardware thru config options.
20100401 (0.06) Last attempt with Fourier math.
20100323 (0.01) First draft.
*/

// compiler options // used for debug random hags
//#pragma GCC push_options
//#pragma GCC optimize ("O0")

// ==== defs
#define WELCOME_MSG           "Arduino TNC v.0.15.3"
#define MIN_PACKET_LEN        10
#define PACKET_SIZE           340
#define AX25_MARK             0
#define AX25_SPACE            1
#define MAX_SYNC_ERRS         5
#define MIN_DCD               20
#define T3TOP                 1212
#define READY_TO_SEND         (UCSR0A & (1<<UDRE0))
#define CHECK_CRC_BY_DEFAULT  1                   // enable frame check sequence
#define BIT_DELAY             205                 // 189 Delay for 0.833 ms (189 for 14.7456 MHz) 205 for 16mhz
#define TXDELAY               50                  // Number of 6.7ms delay cycles (send flags)
#define SPACE                 (55)                // gives us 2228hz
#define MARK                  (103)               // gives us 1200.98hz close enough with 16mhz clock 
#define TRUE                  (1)
#define FALSE                 (0)
#define SET_DDRB              (DDRB = 0x3F)
#define DCD_ON                (PORTB |=  0x01)
#define DCD_OFF               (PORTB &= ~0x01)

// ==== includes
#include "Arduino.h"
#include <avr/wdt.h>                              // watchdog lib
#include <util/delay.h>

// ==== protos
void send_serial_str(const char *inputstr);
void Serial_Processes(void);
void MsgHandler(uint8_t data);
void mainTransmit(void);
void ax25sendHeader(void);
void ax25sendByte(uint8_t txbyte);
void ax25crcBit(uint16_t lsb_int);
void decode_ax25(void);
void ax25sendFooter(void);
void mainReceive(void);

// variables
int8_t adcval;                          // zero-biased ADC input
int16_t mult_cb[7],                       // circular buffer for adc*delay values
        mult_sum,                         // sum of mult_cb values
        bias_sum;                        // sum of last 128 ADC readings
uint16_t msg_pos;                        // bytes rec'd, next array element to fill
uint8_t rawadc,                           // value read directly from ADCH register
        since_last_chg,                   // samples since the last MARK/SPACE symbol change
        phase_err,                        // symbol transition timing error, in samples
        current_symbol,                   // MARK or SPACE
        last_symbol,                      // MARK or SPACE from previous reading
        last_symbol_inaframe,             // MARK or SPACE from one bit-duration ago
        inaframe,                         // rec'd start-of-frame marker
        bittimes,                         // bit durations that have elapsed
        bitqlen,                          // number of rec'd bits in bitq
        popbits,                          // number of bits to pop (drop) off the end
        byteval,                          // rec'd byte, read from bitq
        cb_pos,                           // position within the circular buffer
        msg[PACKET_SIZE + 1],             // rec'd data
        test,                             // temp variable
        decode_state,                     // section of rec'd data being parsed
        bias_cnt,                         // number of ADC samples collected (toward goal of 128)
        adc_bias,                         // center-value for ADC, self-adjusting
        calc_crc = CHECK_CRC_BY_DEFAULT;  // check crc before sending to host
uint8_t sync_err_cnt,                     // number of sync errors in this frame (so far)
        bit_timer,                        // countdown one bit duration
        thesebits;                       // number of elapsed bit durations
int8_t adc_delay[6];                    // delay line for adc readings
uint32_t bitq;                            // rec'd bits awaiting grouping into bytes

static uint16_t crc;
static uint8_t sine[16] = {58, 22, 46, 30, 62, 30, 46, 22, 6, 42, 18, 34, 2, 34, 18, 42};
static uint8_t sine_index;                  // Index for the D-to-A sequence
volatile uint8_t inbuf[PACKET_SIZE + 1];      // USART input buffer array
volatile uint16_t inhead;                      // USART input buffer head pointer
volatile uint16_t intail;                      // USART input buffer tail pointer
static uint8_t prevdata;
volatile uint8_t transmit;                 // Keeps track of TX/RX state
volatile uint8_t txtone;                   // Used in main.c SIGNAL(SIG_OVERFLOW2)
volatile uint8_t maindelay;                 // State of mainDelay function

void setup(void) {
    wdt_enable(WDTO_8S);                          // reset after eight second, if no "pat the dog" received
    UBRR0H = 0;                                  // timer value of 19.2kbps serial output to match bootloader
    UBRR0L = 103;
    UCSR0A |= (1 << U2X0);
    UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
    UCSR0B |= (1 << RXCIE0);                        // enable rx interrupt

    ADMUX = (1 << REFS0);                        // channel0, ref to external input (Aref)
    ADMUX |= (1 << ADLAR);                        // left-justified (only need 8 bits)
    ADCSRA = (1 << ADPS2);                        // pre-scale 16
    ADCSRA |= (1 << ADATE);                        // auto-trigger (free-run)
    ADCSRB = 0x00;                              // free-running
    DIDR0 |= (1 << ADC0D);                        // disable digital driver on ADC0
    ADCSRA |= (1 << ADEN);                         // enable ADC
    ADCSRA |= (1 << ADSC);                         // trigger first conversion

    // use 16-bit timer to check the ADC at 13200 Hz.
    // this is 11x the 1200Hz MARK freq, and 6x the 2200Hz SPACE freq.
    // use Timer1 on Arduino Nano (ATmega328P)
    TCCR1A = 0x00;
    TCCR1B = (1 << WGM12) | (1 << CS10);
    TCCR1C = 0x00;
    OCR1A = T3TOP;                               //set timer trigger frequency
    TIMSK1 = (1 << OCIE1A);
    TCCR2A = 0;                                   // use timer2 for a symbol (bit) timer
    // Initialize the 8-bit Timer2 to clock at 1200hz
    TCCR2B = 0x04;                                // Timer2 clock prescale of
    TIFR2 = (1 << TOV2);                            // enable overflow interrupt flag to trigger on overflow
                                                    // use blinky on "pin 13" as DCD light, use "pin 12" as sample clock heartbeat
                                                    // the port/bit designation for these Arduino pins varies with the chip used
                                                    // see the chip-specific DEFINEs above for details
    SET_DDRB;
    PORTB &= 0x00;                                // set
    _delay_ms(1000);                           // pause to settle
    send_serial_str(WELCOME_MSG);              // announce ourselves
    send_serial_str("\n\n\n");
    sei();                                       // enable interrupts
    _delay_ms(1000);                           // pause again for ADC bias to settle
}

void loop(void) {
    Serial_Processes();
    wdt_reset();                                    // watchdog give me another second to do stuff (pat the dog)
}

// functions
ISR(TIMER1_COMPA_vect) {
    if (transmit)                                    // if transmitting output our sine tone
    {
        ++sine_index;                                 // Increment index
        sine_index &= 15;                             // And wrap to a max of 15
        PORTB = sine[sine_index];                     // Load next D-to-A sinewave value
        OCR1A = txtone;                               // Preload counter based on freq.
    } else {
        // calculate ADC bias (average of last 128 ADC samples)
        // this input is decoulped from the receiver with a capacitor,
        // and is re-biased to half of the Arduino regulated +3.3V bus
        // with a voltage divider. therefore the net bias should equal
        // (more or less) the settle point of the voltage divider.
        // doing this in software also means that the calculated bias
        // will re-center automatically if the resistors are not
        // perfectly matched, etc.
        rawadc = ADCH;
        bias_sum += rawadc;
        if (++bias_cnt == 128) {
            adc_bias = (uint8_t) (bias_sum >> 7);
            bias_cnt = 0;
            bias_sum = 0;
        }
        //=========================================================
        // Seguine math
        // for details, see http://www.cypress.com/?docID=2328
        //=========================================================
        adcval = rawadc - adc_bias;
        // circle buffer is just 7 elements (about half of a bit duration)
        if (++cb_pos == 7) {
            cb_pos = 0;
        }
        // back out old value from sum
        mult_sum -= mult_cb[cb_pos];
        // multiply the latest ADC value by the value ~1/2 lo-freq duration ago. if the
        // incoming audio is the 1200 Hz MARK tone, the two samples will (usually) have
        // opposite phases/signs, so multiplying the two values will give a negative result.
        // If the incoming audio is 2200 Hz, the two samples usu. have the same phase/sign,
        // so multiplying them will give a positve result.
        // subscript 5 = six samples ago-------v
        mult_cb[cb_pos] = adcval * adc_delay[5];
        // add this result to get the sum of the last 7 multipliers (1st LPF)
        mult_sum += mult_cb[cb_pos];
        // force a definitive answer with a hysteresis zone around zero (2nd LPF)
        if (mult_sum >= 100) { current_symbol = AX25_SPACE; }
        else if (mult_sum <= -100) { current_symbol = AX25_MARK; }
        else { ; } // inconclusive - dont change
        // increment # of samples since last symbol change, enforce a max
        if (++since_last_chg > 200) { since_last_chg = 200; }
        thesebits = 0;
        // rotate the delay
        uint8_t x;
        for (x = 5; x >= 1; x--) {
            adc_delay[x] = adc_delay[x - 1];
        }
        adc_delay[0] = adcval;
        //=============================
        //   clock and bit recovery
        //=============================
        // the in-a-frame and seeking-frame-start modes require different strategies
        // let's split them up here
        if (inaframe) {
            //================================
            // clock recovery within a frame
            //================================

            // check symbol state only once per bit time (3rd LPF)
            bit_timer--;

            if (current_symbol != last_symbol) {
                // save the new symbol
                last_symbol = current_symbol;
                // reset counter
                since_last_chg = 0;

                // Ideally, frequency transitions will occur on a since-last-change
                // count that is an exact bit duration at the 1200 Hz signaling
                // rate - that is, an exact multiple of 11 samples (11 samples = 1 bit,
                // 22 = 2 bits, 33 = 3, etc). To give some extra settle time, we
                // don't attempt to read the symbol value at the exact moment of
                // transition; instead, we give it 4 extra beats. Thus as bit_timer
                // is counting down to the next symbol check, its value should ideally
                // be 4 when the symbol change actually takes place. If the symbol
                // change is a little early or late, we can tweak the bit_timer to
                // tolerate some drift and still keep sync.
                // By those rules, an SLC of 4 is perfect, 2 through 6 are fine and
                // need no adjustment, 7,8 and 0,1 need adjustment, 9,10 are timing
                // errors - we can accept a certain number of those before aborting.
                if ((bit_timer == 7) || (bit_timer == 8)) {
                    // other station is slow or we're fast. nudge timer back.
                    bit_timer -= 1;
                } else if ((bit_timer == 0) || (bit_timer == 1)) {
                    // they're fast or we're slow - nudge timer forward
                    bit_timer += 1;
                } else if ((bit_timer == 9) || (bit_timer == 10)) {
                    // too much error
                    if (++sync_err_cnt > MAX_SYNC_ERRS) {
                        sync_err_cnt = 0;
                        msg_pos = 0;
                        inaframe = 0;
                        bitq = 0;
                        bitqlen = 0;
                        bit_timer = 0;
                        bittimes = 0;
                        // turn off DCD light
                        DCD_OFF;
                        return;
                    }
                } // end bit_timer cases
            } // end if symbol change
            //=============================
            // bit recovery within a frame
            //=============================
            if (bit_timer == 0) {
                // time to check for new bits
                // reset timer for the next bit
                bit_timer = 11;
                // another bit time has elapsed
                bittimes++;
                // wait for a symbol change decide what bits to clock in,
                // and how many
                if (current_symbol != last_symbol_inaframe) {
                    // add one as ready-to-decode flag
                    thesebits = bittimes + 1;
                    bittimes = 0;
                    last_symbol_inaframe = current_symbol;
                }
            } // end if bit_timer==0
        } // end if inaframe
        else {
            //=================
            // not in a frame
            //=================
            // housekeeping
            // phase_err = since_last_change MOD 11, except that the "%" operator is =very slow=
            phase_err = since_last_chg;
            while (phase_err >= 11) {
                phase_err -= 11;
            }
            // elapsed bittimes = round (since_last_chg / 11)
            bittimes = 0;
            test = since_last_chg + 5;
            while (test > 11) {
                test -= 11;
                bittimes++;
            }
            thesebits = 0;
            //====================================
            // clock recovery NOT within a frame
            //====================================
            // our bit clock is not synced yet, so we will need a symbol transition
            // to either establish sync or to clock in bits (no transition? exit ISR)
            if (current_symbol == last_symbol) {
                return;
            }
            // symbol change
            // save the new symbol, reset counter
            last_symbol = current_symbol;
            since_last_chg = 0;
            // check bit sync
            if ((phase_err >= 4) && (phase_err <= 7)) {
                // too much error
                bitq = 0;
                bitqlen = 0;
                // turn off the DCD light
                DCD_OFF;
            }
            // save these bits
            thesebits = bittimes + 1;
        } // end else ( = not inaframe)
        //========================================
        //   bit recovery, in or out of a frame
        //========================================
        if (thesebits == 0) {
            return;                                         // if no bit times have elapsed, nothing to do
        } else {
            thesebits--;                                    // remove the "ready" flag
        }
        // determine incoming bit values based on how many bit times have elapsed.
        // whatever the count was, the last bit involved a symbol change, so must be zero.
        // all other elapsed bits must be ones. AX.25 is transmitted LSB first, so in
        // adding bits to the incoming bit queue, we add them right-to-left (ie, new bits
        // go on the left). this lets us ready incoming bytes directly from the lowest
        // eight bits of the bit queue (once we have that many bits).
        // the act of adding bits to the queue is in two parts - (a) OR in any one bits,
        // shifting them to the left as required prior to the OR operation, and (b) update
        // the number of bits stored in the queue. with zero bits, there's nothing to
        // OR into place, so they are taken care of when we update the queue length, and
        // when we shift the queue to the right as bytes are popped off the end later on.
        switch (thesebits) {
            case 1:
                break;    // no ones to add ------> binary       "0"

            case 2:
                bitq |= (0x01 << bitqlen);     // binary      "01"
                break;

            case 3:
                bitq |= (0x03 << bitqlen);     // binary     "011"
                break;

            case 4:
                bitq |= (0x07 << bitqlen);     // binary    "0111"
                break;

            case 5:
                bitq |= (0x0F << bitqlen);     // binary   "01111"
                break;

                // "six" is a special case ("bitstuff"): drop the zero, and only add the 5 one bits
            case 6:
                bitq |= (0x1F << bitqlen);     // binary   "11111"
                thesebits = 5;
                break;

                // "seven" is another special case - only legal for an HDLC byte
            case 7:                                   // binary "0111111"
                if ((bitqlen == 1) && (bitq == 0)) {
                    // only one bit pending, and it's a zero
                    // this is the ideal situation to receive a "seven".
                    // complete the HDLC byte
                    bitq = 0x7E;
                    bitqlen = 8;
                } else if (bitqlen < 4) {
                    // 0-3 bits still pending, but not the ideal single-zero.
                    // this is kinda ugly. let's dump whatever is pending,
                    // and close the frame with a tidy right-justified HDLC.
                    bitq = 0x7E;
                    bitqlen = 8;
                } else if (bitqlen >= 4) {
                    // also kinda ugly - half or more of an unfinished byte.
                    // lets hang onto the pending bits by fill out this
                    // unfinished byte with zeros, and append the full HDLC
                    // char, so that we can close the frame neatly.
                    bitq = (bitq & 0xFF) | 0x7E00;
                    bitqlen = 16;
                } else {
                    // huh?!? ok, well, let's clean up
                    bitq = 0x7E;
                    bitqlen = 8;
                }
                // we've already made the necessary adjustments to bitqlen,
                // so do not add anything else (below)
                thesebits = 0;
                break;

            default:
                // less than a full bit, or more than seven have elapsed
                // clear buffers
                msg_pos = 0;
                inaframe = 0;
                bitq = 0;
                bitqlen = 0;
                // do not add to bitqlen (below)
                thesebits = 0;
                // turn off DCD light
                DCD_OFF;
                break;

        } // end switch
        // how many bits did we add?
        bitqlen += thesebits;
        //===================
        //   byte recovery
        //===================
        // got our bits in a row. now let's talk about bytes.
        // check the bit queue, more than once if necessary
        while (bitqlen >= 8) {
            // take the bottom 8 bits to make a byte
            byteval = bitq & 0xFF;
            // special case - HDLC frame marker
            if (byteval == 0x7E) {
                if (inaframe == 0) {
                    // marks start of a new frame
                    inaframe = 1;
                    last_symbol_inaframe = current_symbol;
                    sync_err_cnt = 0;
                    bit_timer = 15;
                    bittimes = 0;
                    // pop entire byte (later)
                    popbits = 8;
                } else if (msg_pos < MIN_PACKET_LEN) {
                    // We are already in a frame, but have not rec'd any/enough data yet.
                    // AX.25 preamble is sometimes a series of HDLCs in a row, so
                    // let's assume that's what this is, and just drop this byte.
                    popbits = 8;
                } else {
                    // in a frame, and have some data, so this HDLC is probably
                    // a frame-ender (and maybe also a starter)
                    if (msg_pos > 0) {   /*
            printf( "Message was:" ) ;
            for ( x=0 ; x < msg_pos ; x++ )
            {
                printf( " %02X", msg[x] ) ;
            }    
            printf( "\n" ) ; 
            printf( "Which decodes as:\n" ) ;
            */
                        // send frame data out the serial port
                        decode_ax25();
                    }
                    // stay in inaframe-mode, in case a new frame is starting
                    msg_pos = 0;
                    sync_err_cnt = 0;
                    bittimes = 0;
                    // pop entire byte (later)
                    popbits = 8;
                }  // end else for "if not inaframe"
            }  // end if byteval = 0x7E
            else if (inaframe == 1) {
                // not an HDLC frame marker, but =is= a data character
                // add it to the incoming message
                msg[msg_pos] = byteval;
                msg_pos++;
                if (msg_pos > PACKET_SIZE) {
                    sync_err_cnt = 0;
                    msg_pos = 0;
                    inaframe = 0;
                    bitq = 0;
                    bitqlen = 0;
                    bit_timer = 0;
                    bittimes = 0;
                    // turn off DCD light
                    DCD_OFF;
                    return;
                }
                // is this good enough of a KISS frame to turn on the carrier-detect light?
                // we know we have an HDLC (because we're in a frame);
                // check for end-of-header marker
                if (byteval == 0x03) {
                    DCD_ON;
                }
                // pop entire byte (later)
                popbits = 8;
            } else {
                // not already in a frame, and this byte is not a frame marker.
                // It is possible (likely) that when an HDLC byte arrives, its 8 bits will not
                // align perfectly with the 8 we just checked. So instead of dropping all 8 of
                // these random bits, let's just drop one, and re-check the rest again later.
                // This increases our chances of seeing the HDLC byte in the incoming bitstream
                // amid noise (esp. if we're running open squelch).
                popbits = 1;
            }
            // pop the used bits off the end
            // (i know, not really a "pop", more of a "drop")
            bitq = (bitq >> popbits);
            bitqlen -= popbits;
        } // end while bitqueue >= 8
        sei();
    }
    return;
}  // end timerX interrupt

/**********************************************************************
*  Set up a generic timer that we can call (only enabled in transmit mode) 
*  that we use as a symbol timer so that it ticks at ~1200hz 
*
**********************************************************************/
SIGNAL(TIMER2_OVF_vect) {
    //PORTB ^= 0x3C;//sine[sine_index];             // Load next D-to-A sinewave value
    maindelay = FALSE;                              // Clear condition holding up mainDelay
    TCNT2 = 0;                                      // Make long as possible delay
}

/******************************************************************************/
SIGNAL(USART_RX_vect) {
    if (++inhead == PACKET_SIZE) inhead = 0;           // Advance and wrap buffer pointer
    inbuf[inhead] = UDR0;                           // Transfer the byte to the input buffer
    return;
}                                                 // End SIGNAL(SIG_UART_RECV)

/******************************************************************************/
inline void Serial_Processes(void) {
    PORTD ^= 0x04;
    if (intail != inhead)                           // If there are incoming bytes pending
    {
        if (++intail == PACKET_SIZE) intail = 0;         // Advance and wrap pointer
        MsgHandler(inbuf[intail]);                    // And pass it to a handler
    }
    return;
}                                                 // End Serial_Processes(void)

/*******************************************************************************
*  MsgHandler(char data) 
*  Takes in a byte at a time and determins what we should do with it from the 
*  serial port. This is what translates kiss data and spits it out the modem 
*  taking care of special characters 
*******************************************************************************/
inline void MsgHandler(uint8_t data) {
    if (0xC0 == prevdata) {
        if ((0x00 == data)) { //we have the start of a new message
            mainTransmit();
        }
        //TODO setup other modem parameters here... such as txtail etc....
    } else if ((0xC0 == data) && (transmit == TRUE))// now we have the end of a message
    {
        mainReceive();
    } else if (0xDC == data) // we may have an escape byte
    {
        if (0xDB == prevdata) {
            ax25sendByte(0xC0);
        }
    } else if (0xDD == data) // we may have an escape byte
    {
        if (0xDB == prevdata) {
            ax25sendByte(0xDB);
        }
    } else if (TRUE == transmit)ax25sendByte(data); // if we are transmitting then just send the data
    prevdata = data; // copy the data for our state machine
}

void decode_ax25(void) {
    // Take the data in the msg array, and send it out the serial port.
    uint16_t x = 0;
    decode_state = 0;     // 0=just starting, 1=header, 2=got 0x03, 3=got 0xF0 (payload data)
    if (calc_crc) {
        crc = 0xFFFF;
        //(ax25crcBit(bit_zero));
        for (x = 0; x < (msg_pos - 2); x++) {
            uint8_t crcbyte = msg[x];
            for (char y = 0; y < 8; y++) {
                (ax25crcBit(crcbyte & 0x01));
                crcbyte >>= 1;
            }
        } // end for
        if (msg[msg_pos - 2] != (lowByte(crc) ^ 0xFF) || msg[msg_pos - 1] != (highByte(crc) ^ 0xFF)) {
            //crc error!
            return;
        }
    }
    // lop off last 2 bytes (FCS checksum, which we're not sending to the PC)
    for (x = 0; x < (msg_pos - 2); x++) {
        switch (decode_state) {
            // note the goofy order!!
            case 0:
                // just starting
                while (!READY_TO_SEND);
                UDR0 = 0xC0;                 // frame start/end marker
                while (!READY_TO_SEND);
                UDR0 = 0x00;                 // data on port 0
                while (!READY_TO_SEND);
                UDR0 = msg[x];
                decode_state = 1;
                break;

            case 2:
                // got the 0x03, waiting for the 0xF0
                if (msg[x] == 0xF0) {
                    while (!READY_TO_SEND);
                    UDR0 = msg[x];
                    decode_state = 3;
                } else {
                    // wow - corrupt packet? abort
                    while (!READY_TO_SEND);
                    UDR0 = 13;
                    while (!READY_TO_SEND);
                    UDR0 = 10;
                    return;
                }
                break;

            case 1:
                // in the header
                if (msg[x] == 0x03) {
                    while (!READY_TO_SEND);
                    UDR0 = msg[x];
                    decode_state = 2;
                    break;
                }
                // else fall through

            default:
                // payload or header
                if (msg[x] == 0xC0) {
                    while (!READY_TO_SEND);
                    UDR0 = 0xDB;
                }
                while (!READY_TO_SEND);
                UDR0 = msg[x];
                if (msg[x] == 0xDB) {
                    while (!READY_TO_SEND);
                    UDR0 = 0xDD;
                }
                break;
        }

    } // end for
    while (!READY_TO_SEND);
    UDR0 = 0xC0;  // end of frame
    while (!READY_TO_SEND);
    UDR0 = 13;    // CR
    while (!READY_TO_SEND);
    UDR0 = 10;    // LF
}

void send_serial_str(const char *inputstr) {
    uint16_t x;
    for (x = 0; x < strlen(inputstr); x++) {
        if (inputstr[x] == 0) {
            return;
        }
        while (!READY_TO_SEND);
        UDR0 = (uint8_t) inputstr[x];
    }
}

void mainTransmit(void) {
    //UCSR0B &= ~((1<<RXCIE0)|(1<<TXCIE0));               // Disable the serial interrupts
    //TCCR2B = 0x02;                                      // Timer2 clock prescale of 8
    sine_index = 0;                                       // set our transmitter so it always starts the sine generator from 0
    txtone = MARK;                                        // set up the bits so we always start with the same tone for the header (otherwise it alternates)
    // enable overflow interrupt
    TIMSK2 |= (1 << TOIE2);                                 // enable timer 2
    TCCR1B = (1 << WGM12) | (2 << CS10);                     // setup timer 1
    TCNT2 = BIT_DELAY;                                    // setup timer two to trigger at 1200 times a second
    transmit = TRUE;                                      // Enable the transmitter
    ax25sendHeader();                                     // Send APRS header
    return;
}                                                       // End mainTransmit(void)

void mainReceive(void) {
    ax25sendFooter();                                     // Send APRS footer
    transmit = FALSE;                                     // Disable transmitter
    PORTB &= 0x00;                                        // Make sure the transmitter is disabled by turning off transmit pin was 0x3D to turn off ptt
    TCCR1B = (1 << WGM12) | (1 << CS10);
    TIMSK2 &= ~(1 << TOIE2);                                // disable timer two interrupt
    OCR1A = T3TOP;                                       // set timer 1 back to triggering at the recieve sample frequency
    sine_index = 0;                                       // set the sine back to 0 (redundant)
    return;
}                                                       // End mainReceive(void)

void mainDelay(uint8_t timeout) {
    maindelay = TRUE;                                     // Set the condition variable
    TCNT2 = 255 - timeout;                                // Set desired delay
    while (maindelay) {
        //asm("nop");                               //do something TODO process serial data here...
    }
    //TCNT2 = (255 - BIT_DELAY);
    return;
}                                                       // End mainDelay(unsigned int timeout)

void ax25sendHeader(void) {
    static uint8_t loop_delay;
    crc = 0xFFFF;                                         // Initialize the crc register
    // Transmit the Flag field to begin the UI-Frame
    // Adjust length for txdelay (each one takes 6.7ms)
    for (loop_delay = 0; loop_delay < TXDELAY; loop_delay++) {
        (ax25sendByte(0x7E));                               //send the sync header byte
    }
    return;
}                                                       // End ax25sendHeader(void)

void ax25sendFooter(void) {
    static uint8_t crchi;
    crchi = (crc >> 8) ^ 0xFF;
    ax25sendByte(crc ^ 0xFF);                               // Send the low byte of the crc
    ax25sendByte(crchi);                                  // Send the high byte of the crc
    ax25sendByte(0x7E);                                   // Send a flag to end the packet
    return;
}                                                       // End ax25sendFooter(void)

void ax25sendByte(uint8_t txbyte) {
    static uint8_t mloop;
    static uint8_t bitbyte;
    static uint8_t bit_zero;
    static uint8_t sequential_ones;
    bitbyte = txbyte;                                     // Bitbyte will be rotated through
    for (mloop = 0; mloop < 8; mloop++)                 // Loop for eight bits in the byte
    {
        bit_zero = bitbyte & 0x01;                          // Set aside the least significant bit
        if (txbyte == 0x7E)                                 // Is the transmit character a flag?
        {
            sequential_ones = 0;                              // it is immune from sequential 1's
        } else                                                // The transmit character is not a flag
        {
            (ax25crcBit(bit_zero));                           // So modify the checksum
        }
        if (!(bit_zero))                                    // Is the least significant bit low?
        {
            sequential_ones = 0;                              // Clear the number of ones we have sent
            txtone = (txtone == MARK) ? SPACE : MARK;          // Toggle transmit tone
        } else                                                // Else, least significant bit is high
        {
            if (++sequential_ones == 5)                       // Is this the 5th "1" in a row?
            {
                mainDelay(BIT_DELAY);                           // Go ahead and send it

                txtone = (txtone == MARK) ? SPACE : MARK;        // Toggle transmit tone
                sequential_ones = 0;                            // Clear the number of ones we have sent
            }
        }
        bitbyte >>= 1;                                      // Shift the reference byte one bit right
        mainDelay(BIT_DELAY);                               // Pause for the bit to be sent
    }
    return;
}                                                       // End ax25sendByte(uint8_t txbyte)

void ax25crcBit(uint16_t lsb_int) {
    static uint16_t xor_int;
    xor_int = crc ^ lsb_int;                              // XOR lsb of CRC with the latest bit
    crc >>= 1;                                            // Shift 16-bit CRC one bit to the right
    if (xor_int & 0x0001)                                 // If XOR result from above has lsb set
    {
        crc ^= 0x8408;                                      // Shift 16-bit CRC one bit to the right
    }
    return;
}                                                       // End ax25crcBit(int lsb_int)
//#pragma GCC pop_options
// end of file
