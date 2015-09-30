/*
   Board configuration header.
 
 Copyright (c) 2015, Edouard Lafargue, ed@wizkers.io
 All rights reserved
  
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _PM_CONFIG_H_
#define _PM_CONFIG_H_

// Global configuration of the board: pin attributions and so on.

// IO LED
#define IO_LED_PIN 13    // PC7 - Arduino IO13 (default LED)
#define STAT_LED_PIN 5   // PC6 - Arduino D5 

//#define DPIN_OOK_TX         3
//#define DPIN_STARTTX_BUTTON 6
//#define DPIN_RF69_RESET     7
#define DPIN_OOK_RX         8
#define DPIN_LED            13

// The default ID of the transmitter to decode/encode from/to
#define DEFAULT_TX_ID 0x71e0

// If defined will dump all the encoded RX data, and partial decode fails
//#define DUMP_RX


// Board settings (saved in EEPROM)
#define SETTINGS_MAGIC 0xbabebabe

// Our settings (saved to EEPROM)
typedef struct {
  unsigned long magic;          // set at first run
  
  float power_factor;             // The power meter power factor
  uint16_t tx_id;
  bool debug;
  
} settings_t;


#endif
