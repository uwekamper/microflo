/* MicroFlo - Flow-Based Programming for microcontrollers
 * Copyright (c) 2013 Jon Nordby <jononor@gmail.com>
 * MicroFlo may be freely distributed under the MIT license
 */

#include "microflo.h"

#include <avr/io.h>
#include <util/atomic.h>

// Datasheets
// atmega328: http://www.atmel.com/Images/Atmel-8271-8-bit-AVR-Microcontroller-ATmega48A-48PA-88A-88PA-168A-168PA-328-328P_datasheet.pdf
// at90usb1287: http://www.atmel.com/Images/doc7593.pdf
// at90usbkey: http://www.atmel.com/Images/doc7627.pdf‎

// Those not familiar with bitmasking and register twiddling should see
// http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=37871

// Timers, see
// http://www.adnbr.co.uk/articles/counting-milliseconds
// PWM, see
// http://maxembedded.com/2011/08/07/avr-timers-pwm-mode-part-i/
// http://maxembedded.com/2012/01/07/avr-timers-pwm-mode-part-ii/


#ifndef F_CPU
#error "F_CPU not defined!"
#endif

#define MILLISECOND_OVERFLOW ((F_CPU / 1000) / 8)

#define avrPinSet(REG, pinNumber) *ports[pinNumber/8].REG |= _BV(pinNumber % 8)
#define avrPinClear(REG, pinNumber) *ports[pinNumber/8].REG &= ~_BV(pinNumber % 8)
#define avrPinGet(REG, pinNumber) *ports[pinNumber/8].REG & _BV(pinNumber % 8)

struct AvrPort {
    volatile uint8_t * DDR;
    volatile uint8_t * PIN;
    volatile uint8_t * PORT;
};

// PERFORMANCE: use a C++ const-expression for the pin to register mapping?

static const AvrPort ports[6] = {
    {&DDRA, &PINA, &PORTA},
    {&DDRB, &PINB, &PORTB},
    {&DDRC, &PINC, &PORTC},
    {&DDRD, &PIND, &PORTD},
    {&DDRE, &PINE, &PORTE},
    {&DDRF, &PINF, &PORTF},
};

static const int8_t ADC_START_PIN = 5*8; // PORTF0, correct for AT90USB1287

static volatile long g_millis = 0;
static volatile uint8_t g_adc_values[8] = {};
static volatile int8_t g_adc_channel = -1;

ISR (TIMER1_COMPA_vect)
{
    g_millis++;
}

ISR(ADC_vect)
{
    if (g_adc_channel > 0) {
        g_adc_values[g_adc_channel] = ADCH;
        g_adc_channel++
        if (g_adc_channel >= 7) {
            g_adc_channel = 0;
        }
        ADMUX = 0xE0 + g_adc_channel;
        //ADMUX |= (0b00011111 | g_adc_channel);
    }
    ADCSRA |= 1<<ADSC;
}


class Avr8IO : public IO {
public:


public:
    Avr8IO() {
        // Clear on match mode, Clock/8
        TCCR1B = (1 << WGM12) | (1 << CS11);
        // Overflow every 1ms, enable interrupt
        OCR1AH = (MILLISECOND_OVERFLOW >> 8);
        OCR1AL = MILLISECOND_OVERFLOW;
        TIMSK1 |= (1 << OCIE1A);

        // Enable ADC, ADC interrupt. Pre-scale=clk/128
        ADCSRA = 0x8F;
        // Vref=2.56V internal, left justify data, ADC0 as input channel
        ADMUX = 0xE0;
        ADCSRA |= 1<<ADSC; // Start Conversion



    }
    ~Avr8IO() {
    }

    // Serial
    // FIXME: implement
    virtual void SerialBegin(int serialDevice, int baudrate) {

    }
    virtual long SerialDataAvailable(int serialDevice) {
        return false;
    }
    virtual unsigned char SerialRead(int serialDevice) {
        return 0;
    }
    virtual void SerialWrite(int serialDevice, unsigned char b) {

    }

    // Pin config
    virtual void PinSetMode(int pin, IO::PinMode mode) {
        if (mode == IO::OutputPin) {
            avrPinSet(DDR, pin);
        } else if (mode == IO::InputPin) {
            avrPinClear(DDR, pin);
        }
    }
    virtual void PinEnablePullup(int pin, bool enable) {
        // assumes pin has been configured as input
        if (enable)
            avrPinSet(PORT, pin);
        else
            avrPinClear(PORT, pin);
    }

    // Digital
    virtual void DigitalWrite(int pin, bool val) {
        if (val)
            avrPinSet(PORT, pin);
        else
            avrPinClear(PORT, pin);
    }
    virtual bool DigitalRead(int pin) {
        return avrPinGet(PIN, pin);
    }

    // Analog
    virtual long AnalogRead(int pin) {
        pin = pin-ADC_START_PIN;
        if (pin < 0 || pin > 7 || g_adc_channel < 0) {
            return -1;
        }

        long val;
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            val = g_adc_values[pin];
        }
        return val;
    }
    // FIXME: implement
    virtual void PwmWrite(int pin, long dutyPercent) {

    }

    // Timer
    virtual long TimerCurrentMs() {
        long millis;
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            millis = g_millis;
        }
        return millis;
    }

    virtual void AttachExternalInterrupt(int interrupt, IO::Interrupt::Mode mode,
                                         IOInterruptFunction func, void *user) {

    }
};

