/*
 * Copyright (c) 2014, Majenko Technologies
 * Adapted for the MLX90316 by niktwo17, 2020
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 *  1. Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 * 
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of Majenko Technologies nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without 
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



#ifdef DEBUGEncoderActive
 #define Debugln(x)  Serial.println (x)
 #define Debug(x)  Serial.print (x)
 #define DebugHex(x) Serial.print(x, HEX)
#else
 #define Debugln(x)
 #define Debug(x)
 #define DebugHex(x)
#endif



#include "MLX.h"

MLX::MLX() {
    _delay = 2;
    _cke = 0;
    _ckp = 0;
    _order = MSBFIRST;
}

void MLX::begin() {
    #ifdef DEBUGEncoderActive
    Serial.begin(115200);
    #endif
    pinMode(MOSIMLX, OUTPUT);
    pinMode(SCKMLX, OUTPUT);
    pinMode(SSMLX, OUTPUT);
}

void MLX::end() {
    pinMode(MOSIMLX, INPUT);
    pinMode(SCKMLX, INPUT);
    pinMode(SSMLX, INPUT);
}

void MLX::setBitOrder(uint8_t order) {
    _order = order & 1;
}

void MLX::setDataMode(uint8_t mode) {
    switch (mode) {
        case SPI_MODE0:
            _ckp = 0;
            _cke = 0;
            break;
        case SPI_MODE1:
            _ckp = 0;
            _cke = 1;
            break;
        case SPI_MODE2:
            _ckp = 1;
            _cke = 0;
            break;
        case SPI_MODE3:
            _ckp = 1;
            _cke = 1;
            break;
    }

    digitalWrite(SCKMLX, _ckp ? HIGH : LOW);
}

void MLX::setClockDivider(uint8_t div) {
    switch (div) {
        case SPI_CLOCK_DIV2:
            _delay = 2;
            break;
        case SPI_CLOCK_DIV4:
            _delay = 4;
            break;
        case SPI_CLOCK_DIV8:
            _delay = 8;
            break;
        case SPI_CLOCK_DIV16:
            _delay = 16;
            break;
        case SPI_CLOCK_DIV32:
            _delay = 32;
            break;
        case SPI_CLOCK_DIV64:
            _delay = 64;
            break;
        case SPI_CLOCK_DIV128:
            _delay = 128;
            break;
        default:
            _delay = 128;
            break;
    }
}

void MLX::wait(uint_fast8_t del) {
    for (uint_fast8_t i = 0; i < del; i++) {
        asm volatile("nop");
    }
}

uint8_t MLX::transfer(uint8_t val) {
    pinMode(MOSIMLX, OUTPUT);
    uint8_t out = 0;
    if (_order == MSBFIRST) {
        uint8_t v2 = 
            ((val & 0x01) << 7) |
            ((val & 0x02) << 5) |
            ((val & 0x04) << 3) |
            ((val & 0x08) << 1) |
            ((val & 0x10) >> 1) |
            ((val & 0x20) >> 3) |
            ((val & 0x40) >> 5) |
            ((val & 0x80) >> 7);
        val = v2;
    }

    uint8_t del = _delay >> 1;

    uint8_t bval = 0;
    /*
     * CPOL := 0, CPHA := 0 => INIT = 0, PRE = Z|0, MID = 1, POST =  0
     * CPOL := 1, CPHA := 0 => INIT = 1, PRE = Z|1, MID = 0, POST =  1
     * CPOL := 0, CPHA := 1 => INIT = 0, PRE =  1 , MID = 0, POST = Z|0
     * CPOL := 1, CPHA := 1 => INIT = 1, PRE =  0 , MID = 1, POST = Z|1
     */

    int sck = (_ckp) ? HIGH : LOW;

    for (uint8_t bit = 0u; bit < 8u; bit++)
    {
        if (_cke) {
            sck ^= 1;
            digitalWrite(SCKMLX, sck);
            wait(del);
        }

        /* ... Write bit */
        digitalWrite(MOSIMLX, ((val & (1<<bit)) ? HIGH : LOW));

        delayMicroseconds(15);

        sck ^= 1u; digitalWrite(SCKMLX, sck);

        /* ... Read bit */
        {
            bval = digitalRead(_miso);

            if (_order == MSBFIRST) {
                out <<= 1;
                out |= bval;
            } else {
                out >>= 1;
                out |= bval << 7;
            }
        }

        delayMicroseconds(15);

        if (!_cke) {
            sck ^= 1u;
            digitalWrite(SCKMLX, sck);
        }
    }

    return out;
}

/**
 * Returns the Angle in radians
 * If defined in MLX.h already mapped to range
 */
float MLX::getAngle()
{
    float returnVal = 0;
    digitalWrite(SSMLX, 0);
	transfer(0xAA);
    delayMicroseconds(40);
    transfer(0xFF);
    delayMicroseconds(40);
    pinMode(MOSIMLX,INPUT_PULLUP);    // switch MOSI pin to input
    long int ret=-1;
    unsigned int rr=-1;
    unsigned int cc=-1;
    uint8_t firstByte = readByte();
    uint8_t secondByte = readByte();
    uint8_t thirdByte = readByte();
    uint8_t fourthByte = readByte();
    uint8_t fifthByte = readByte();
    uint8_t thirdByteInv = 0xFF - thirdByte;
    uint8_t fourthByteInv = 0xFF - fourthByte;
    if((secondByte &0x003) == 1)
    {
        if(((firstByte - thirdByteInv) | (secondByte - fourthByteInv)) == 0)
        {
            uint32_t angleValTest = (uint32_t)firstByte;
            angleValTest = angleValTest <<8;
            angleValTest = angleValTest + secondByte;
            angleValTest = (angleValTest >>2);
            angleValDegree = angleValTest;
            angleValDegree = (angleValDegree *  3600) / 16384;	// scale output to 360 deg, unit in degrees*10

            angleRadVal = angleValDegree *(PI/1800);
            #if defined(CONVERTCROSSOVER)
            angleRadVal = convertCrossover();
            #elif defined(QUADRANTSUSED)
            angleRadVal = convertQuadrants();
            #endif

            if((millis() - lastSerialPrint) >=1000)
            {
            Debug("AngleRadianVal: ");
            Debugln(angleRadVal);
            Debug("AngleValDegree: ");
            Debugln((angleToDisplay /10));
            lastSerialPrint = millis();
            }
        }
        else
        {
            Debugln("Bit error");
            return -1;
        }
        
    }
    else
    {
        Debugln("Encoder in errorMode");
        return -2;
    }
    
delayMicroseconds(80);
digitalWrite(SSMLX,1);
return angleRadVal;
}



/**
 * Maps the radian value to the corresponding range 
 */
float MLX::convertCrossover()
{
    float returnRad;           
    if(angleValDegree <= ANGLEREADCROSSOVER)
    {
        returnRad = map (angleValDegree, 0, ANGLEREADCROSSOVER, 0, MINANGLECORRESP);
    }
    else
    {
        returnRad = map (angleValDegree, 3600, ANGLEREADCROSSOVER, 0, MAXANGLECORRESP);
    }
    return returnRad;
}


/**
 * Maps the Radian value to the corresponding range considering the current quadrant
 * If used method needs to be updated with high frequency, otherwise crossover may be missed
 */
float MLX::convertQuadrants()
{
    float returnRadVal;
    if((angleValDegree<=800)&&(angleValDegree >=0))
    {
        QRight = true;
    }
    else if( (angleValDegree <=3600)&& (angleValDegree >= 3200) )
    {
        QRight = false;
    }
    if(QRight ==true)
    {
        angleToDisplay = map (angleValDegree, 0, 3200, 0, 1000);
        returnRadVal = angleToDisplay *(PI/1800);         
    }
    else
    {
        angleToDisplay = map (angleValDegree, 3600, 800, 0, -1000);
        returnRadVal = angleToDisplay *(PI/1800); 
    }
    return returnRadVal;
}



/**
 * Neutral is defined as 0 rad
 */
bool MLX::isNeutral()
{
    if(abs(angleRadVal) <= ANGLELOCKTOLERANCEABS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t MLX::readByte()
{
    uint8_t out = 0;
    uint8_t bval = 0;
    /*
     * CPOL := 0, CPHA := 0 => INIT = 0, PRE = Z|0, MID = 1, POST =  0
     * CPOL := 1, CPHA := 0 => INIT = 1, PRE = Z|1, MID = 0, POST =  1
     * CPOL := 0, CPHA := 1 => INIT = 0, PRE =  1 , MID = 0, POST = Z|0
     * CPOL := 1, CPHA := 1 => INIT = 1, PRE =  0 , MID = 1, POST = Z|1
     */

    int sck = (_ckp) ? HIGH : LOW;

    for (uint8_t bit = 0u; bit < 8u; bit++)
    {
        if (_cke) {
            sck ^= 1;
            digitalWrite(SCKMLX, sck);
            //delayMicroseconds(100);            
            //wait(del);
        }
        delayMicroseconds(15);
        sck ^= 1u; digitalWrite(SCKMLX, sck);
        delayMicroseconds(1);

        /* ... Read bit */
        {
            bval = digitalRead(MOSIMLX);

            if (_order == MSBFIRST) {
                out <<= 1;
                out |= bval;
            } else {
                out >>= 1;
                out |= bval << 7;
            }
        }

        //wait(del);
        delayMicroseconds(15);

        if (!_cke) {
            sck ^= 1u;
            digitalWrite(SCKMLX, sck);
        }
    }

    return out;


}

uint16_t MLX::transfer16(uint16_t data)
{
	union {
		uint16_t val;
		struct {
			uint8_t lsb;
			uint8_t msb;
		};
	} in, out;
  
	in.val = data;

	if ( _order == MSBFIRST ) {
		out.msb = transfer(in.msb);
		out.lsb = transfer(in.lsb);
	} else {
		out.lsb = transfer(in.lsb);
		out.msb = transfer(in.msb);
	}

	return out.val;
}
