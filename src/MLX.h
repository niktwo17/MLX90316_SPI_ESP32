/*
 * Copyright (c) 2014, Majenko Technologies
 * Adapted for the MLX90316 by niktwo17, 2021
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


#ifndef _MLX_H
#define _MLX_H
#include <SPI.h>

#if (ARDUINO >= 100) 
# include <Arduino.h>
#else
# include <WProgram.h>
#endif



/// In Radians
#define ANGLELOCKTOLERANCEABS 0.05

/// Only needed if conversion should be done here
#define MAXANGLECORRESP 700
#define MINANGLECORRESP -700
#define ANGLEREADCROSSOVER 2000
#define QRIGHTVAL 1200

/// Allows to map the value to the right area
//#define CONVERTCROSSOVER

/// Needed if the sensor is geared and has multiple turns
//#define QUADRANTSUSED

/// For serial prints
//#define DEBUGEncoderActive

/// Pin definitions
#define MOSIMLX 2
#define SCKMLX 16
#define SSMLX 17



class MLX : public SPIClass {
    private:
        void wait(uint_fast8_t del);

    private:
        uint8_t _cke;
        uint8_t _ckp;
        uint8_t _delay;
        uint8_t _miso;
        uint8_t _order;
        void setBitOrder(uint8_t);
        void setDataMode(uint8_t);
        void setClockDivider(uint8_t);
		uint16_t transfer16(uint16_t data);
        uint8_t transfer(uint8_t);
        uint8_t readByte();
        float convertCrossover();
        float convertQuadrants();
        bool QRight = false;
        float lastSerialPrint = 0;
        float angleRadVal = 0;
        float angleValDegree = 0;
        int16_t angleToDisplay = 0;

    public:
        MLX();
        ~MLX();
        void begin();
        void end();
        float getAngle();
        bool isNeutral();
};
#endif