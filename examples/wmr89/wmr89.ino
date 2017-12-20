// Modified for use with the WeeWX and the WMR89a - Ray Kinsella
//  - Check for the sync bits and then remove. 
//  - Modified the interrupt handler to make it robust (interrupt safe).
//  - Added flexibily so bitstream length can vary with the sensor.
//  - Output to serial in binary, removes the need to do nasty string conversion.
//  - Fixed the bit ordering in the serial output. 
// Oregon V2 decoder added - Dominique Pierre
// Oregon V3 decoder revisited - Dominique Pierre
// New code to decode OOK signals from weather sensors, etc.
// 2017-11-03 <raykinsella78@gmail.com> 
// 2010-04-11 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: ookDecoder.pde 5331 2010-04-17 10:45:17Z jcw $

// Define OUTPUT_ASCII to debug on arduino serial console. 
//#define OUTPUT_ASCII
#define OUTPUT_LEDS

// Decoding algorthim uses the change from continuous bit flipping (all 1s)
// to the a constant flip bit (gives 1010) to detect the start of real sensor data.
// This has the effect of capturing the sync bits, as though it where useful data.
#define IGNORE_SYNC_BITS
#define NUMBER_OF_SYNC_BITS 4

class BoardLEDs {
private:
  const    byte BASEPIN = 2;
  const    byte THROTTLEVALUE = 100;
  unsigned long LEDTimer[3];
  byte     checktimer;

public:
  BoardLEDs() 
  {
     memset(&LEDTimer,0, sizeof(LEDTimer));
     checktimer = 0;
  }

  Setup()
  {
#ifdef OUTPUT_LEDS
    for(int i=0; i < 3; i++)
      pinMode(BASEPIN + i, OUTPUT);
#endif
  }

  void SetLED(byte LED)
  {
#ifdef OUTPUT_LEDS
    //Set LED ON
    digitalWrite(BASEPIN + LED, HIGH);
    LEDTimer[LED] = millis() + 2000;
#endif
  }

  void CheckTimer()
  {
    unsigned long now;
    
#ifdef OUTPUT_LEDS
    // Now need to do it everytime we are poked.
    checktimer++;
    if(checktimer > THROTTLEVALUE)
      checktimer = 0;
    else
      return;
    
    now = millis();
    
    for(int i=0; i < 3; i++)
    {
       if(LEDTimer[i] && LEDTimer[i] < now)
       {
        //Turn off LED
        digitalWrite(BASEPIN + i, LOW);
        LEDTimer[i] = 0;
       }
    }
#endif
  }
};

class DecodeOOK {
protected:

    byte total_bits, bits, bitlen, flip, state, pos, syncbits, data[25];

    virtual char decode (word width) =0;
    
public:

    enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };

    DecodeOOK () { resetDecoder(); }

    bool nextPulse (word width) {
        if (state != DONE)
        
            switch (decode(width)) {
                case -1: resetDecoder(); break;
                case 1:  done(); break;
            }
        return isDone();
    }
    
    bool isDone () const { return state == DONE; }

    const byte* getData (byte& count) const {
        count = pos;
        return data; 
    }
    
    void resetDecoder () {
        bitlen = total_bits = bits = pos = flip = 0;
        syncbits = NUMBER_OF_SYNC_BITS;
        state = UNKNOWN;
    }
        
    virtual void gotBit (char value) = 0;
    
    // store a bit using Manchester encoding
    void manchester (char value) {
        flip ^= value; // manchester code, long pulse flips the bit
        gotBit(flip);
    }
    
    // move bits to the front so that all the bits are aligned to the end
    void alignTail (byte max =0) {
        // align bits
        if (bits != 0) {
            data[pos] >>= 8 - bits;
            for (byte i = 0; i < pos; ++i)
                data[i] = (data[i] >> bits) | (data[i+1] << (8 - bits));
            bits = 0;
        }
        // optionally shift bytes down if there are too many of 'em
        if (max > 0 && pos > max) {
            byte n = pos - max;
            pos = max;
            for (byte i = 0; i < pos; ++i)
                data[i] = data[i+n];
        }
    }
    
    void reverseBits () {
        for (byte i = 0; i < pos; ++i) {
            byte b = data[i];
            for (byte j = 0; j < 8; ++j) {
                data[i] = (data[i] << 1) | (b & 1);
                b >>= 1;
            }
        }
    }
    
    void reverseNibbles () {
        for (byte i = 0; i < pos; ++i)
            data[i] = (data[i] << 4) | (data[i] >> 4);
    }
    
    void done () {
        while (bits)
            gotBit(0); // padding
        state = DONE;
    }
};

class OregonDecoderV3 : public DecodeOOK {

private:

    byte SensorLED;

    const byte sensorlookup[3][4] = 
    { { 0x8F, 0x42, 0x48, 0x00} , // Sensor ID = F8 24
      { 0x91, 0x48, 0x50, 0x01} , // Sensor ID = 91 48
      { 0x0, 0x0, 0x0, 0x02}  };

    virtual byte lookupLength(byte sensorcode0, byte sensorcode1)
    {
      for(int i = 0; i < 3; i++)
      {      
        if( sensorlookup[i][0] == sensorcode0 &&
            sensorlookup[i][1] == sensorcode1)
            {
              SensorLED = sensorlookup[i][3];
              return sensorlookup[i][2];
            }
      }

      return -1;
    }

public:
    OregonDecoderV3() {}
    
    // add one bit to the packet data buffer
    virtual void gotBit (char value) {
#ifdef IGNORE_SYNC_BITS
        if (syncbits > 0)
        {
          syncbits--;
          state = OK;

          return;
        }
#endif
        data[pos] = (data[pos] >> 1) | (value ? 0x80 : 00);
        total_bits++;
        pos = total_bits >> 3;
        if (pos >= sizeof data) {
            resetDecoder();
            return;
        }
        state = OK;
    }

    virtual char decode (word width) {
        if (200 <= width && width < 1200) {
            byte w = width >= 700;
            switch (state) {
                case UNKNOWN:
                    if (w == 0)
                        ++flip;
                    // first first long pulse indictates a state transition.
                    else if (32 <= flip) {
                        flip = 1;
                        manchester(1);
                    } else
                        return -1;
                    break;
                case OK:
                    if (w == 0)
                        state = T0;
                    else
                        manchester(1);
                    break;
                case T0:
                    if (w == 0)
                        manchester(0);
                    else
                        return -1;
                    break;                   
            }
        } else {
            return -1;
        }

        // initial 32 bits contain the sensor id
        if ( total_bits < 32 ) 
        {
          return 0;
        }
        else if ( total_bits == 32 )
        {
          bitlen = lookupLength(data[0], data[1]);
          if (bitlen == -1) return -1;

          return 0;
        }
        else
        {
          return  total_bits == bitlen ? 1: 0;
        }
    }

    byte getSensorLED()
    {
      return SensorLED;
    }
};


OregonDecoderV3 orscV3;
BoardLEDs     ledMgr;

enum eErrors 
  {
    eNoMoreSlots,
    eErrorMax,
  };

#define PORT 2
#define VECTORLEN 32
#define VECTORNUM 4

volatile word pulsetab[VECTORNUM][VECTORLEN];

volatile byte vectoridx;
volatile byte vectorslot;
word errors[eErrorMax];

#if defined(__AVR_ATmega1280__)
void ext_int_1(void) {
#else
ISR(ANALOG_COMP_vect) {
#endif
    static word last;
    word pulse; 
    
    // determine the pulse length in microseconds, for either polarity
    pulse = micros() - last;
    last += pulse;

    if(VECTORLEN <= vectorslot)
    {
      errors[eNoMoreSlots]++;
    }
    else
    {
      pulsetab[vectoridx][vectorslot] = pulse;
      vectorslot++;
    }
}

void reportSerial (class DecodeOOK& decoder) {
    byte pos;
    const byte* data = decoder.getData(pos);

    

    for (byte i = 0; i < pos; ++i) {
#ifdef OUTPUT_ASCII
        Serial.print((data[i] & 0x0F), HEX);
        Serial.print(data[i] >> 4, HEX);
#else 
        //Sensor encoding is nibble orientated, but we captured in bytes.
        //Compensate by output LSB first, then the MSB. 
        //Serial.write((data[i] & 0x0F) << 4 | data[i] >> 4);
        Serial.write(data[i]);
#endif
    }

#ifdef OUTPUT_ASCII
    Serial.println();
#endif
        
    decoder.resetDecoder();
}


void setup () {
    Serial.begin(115200);

#ifdef OUTPUT_LEDS
    ledMgr.Setup();
#endif

#ifdef OUTPUT_ASCII
    Serial.println("\n[ookDecoder]");
#endif
 
    vectoridx = vectorslot = 0;
    
#if !defined(__AVR_ATmega1280__)
    pinMode(13 + PORT, INPUT);  // use the AIO pin
    digitalWrite(13 + PORT, 1); // enable pull-up

    // use analog comparator to switch at 1.1V bandgap transition
    ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);

    // set ADC mux to the proper port
    ADCSRA &= ~ bit(ADEN);
    ADCSRB |= bit(ACME);
    ADMUX = PORT - 1;
#else
   attachInterrupt(1, ext_int_1, CHANGE);

   DDRE  &= ~_BV(PE5);
   PORTE &= ~_BV(PE5);
#endif
}

void loop () {

    byte    slot, slots, idx;
    word    localtab[VECTORLEN];
    char    message[20];
    
    cli();
    
    idx = vectoridx;
    slots = vectorslot;
    
    vectorslot = 0;
    if((vectoridx + 1) >= VECTORNUM)
      vectoridx = 0;
    else
      vectoridx++;
      
    sei();

    for(slot=0; slot < slots; slot++)
    {
      if (pulsetab[idx][slot] != 0) {
          if (orscV3.nextPulse(pulsetab[idx][slot]))
          {
              ledMgr.SetLED(orscV3.getSensorLED());
              reportSerial(orscV3);
          }
      }
   }

   ledMgr.CheckTimer();
}


