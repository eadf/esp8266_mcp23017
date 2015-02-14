/*
* mcp23017.h
*
* Copyright (c) 2015, eadf (https://github.com/eadf)
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef DRIVER_MCP23017_INCLUDE_MCP23017_MCP23017_H_
#define DRIVER_MCP23017_INCLUDE_MCP23017_MCP23017_H_

#include "i2c/i2c.h"
#include "c_types.h"

typedef struct {
  I2C_Self i2c;
} MCP23017_Self;

typedef enum {
  MCP23017_INPUT=0,
  MCP23017_OUTPUT=1
} MCP23017_PinMode;

/**
 * initiates the device. Sets the SCL and SDA pins
 */
bool mcp23017_init(MCP23017_Self *self, uint8_t scl_pin, uint8_t sda_pin);

/**
 * Sets the pin mode to either MCP23017_INPUT or MCP23017_OUTPUT
 */
bool mcp23017_pinMode(MCP23017_Self *self, uint8_t deviceAddr, uint8_t pin, MCP23017_PinMode pinmode);

/**
 * Sets the output data of the pin
 */
bool mcp23017_digitalWrite(MCP23017_Self *self, uint8_t deviceAddr, uint8_t pin, bool data);

/**
 * reads the value of the pin
 */
bool mcp23017_digitalRead(MCP23017_Self *self, uint8_t deviceAddr, uint8_t pin, bool* data);

/**
 * Sets the pin mode on all the pins on bank A to MCP23017_INPUT or MCP23017_OUTPUT
 */
bool mcp23017_pinModeA(MCP23017_Self *self, uint8_t deviceAddr, MCP23017_PinMode pinmode);

/**
 * Sets the output data of whole A bank
 */
bool mcp23017_digitalWriteA(MCP23017_Self *self, uint8_t deviceAddr, uint8_t data);

/**
 * reads the value of the whole A bank
 */
bool mcp23017_digitalReadA(MCP23017_Self *self, uint8_t deviceAddr, uint8_t* data);

/**
 * Sets the pin mode on all the pins on bank B to MCP23017_INPUT or MCP23017_OUTPUT
 */
bool mcp23017_pinModeB(MCP23017_Self *self, uint8_t deviceAddr, MCP23017_PinMode pinmode);

/**
 * Sets the output data of whole B bank
 */
bool mcp23017_digitalWriteB(MCP23017_Self *self, uint8_t deviceAddr, uint8_t data);

/**
 * reads the value of the whole B bank
 */
bool mcp23017_digitalReadB(MCP23017_Self *self, uint8_t deviceAddr, uint8_t* data);


/**
 * Sets the pin mode on all the pins to MCP23017_INPUT or MCP23017_OUTPUT
 */
bool mcp23017_pinModeAB(MCP23017_Self *self, uint8_t deviceAddr, MCP23017_PinMode pinmode);

/**
 * writes to all the pins at once
 */
bool mcp23017_digitalWriteAB(MCP23017_Self *self, uint8_t deviceAddr, uint16_t data);

/**
 * reads the value of all the pins at once
 */
bool mcp23017_digitalReadAB(MCP23017_Self *self, uint8_t deviceAddr, uint16_t* data);


#endif /* DRIVER_MCP23017_INCLUDE_MCP23017_MCP23017_H_ */
