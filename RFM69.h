// **********************************************************************************
// Driver definition for HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
// **********************************************************************************
// Copyright Felix Rusu (2014), felix@lowpowerlab.com
// http://lowpowerlab.com/
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#ifndef RFM69_h
#define RFM69_h

#include <stdint.h>

#define GLUE(a, b)      a##b
#define PORT(x)         GLUE(PORT, x)
#define PIN(x)          GLUE(PIN, x)
#define DDR(x)          GLUE(DDR, x)

#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead)
#define RF69_CS_PORT            B // SS is the SPI slave select pin, for instance D10 on Atmega328
#define RF69_CS                 PB2

#define RF69_SDI_PORT           B
#define RF69_SDI                PB3
#define RF69_SDO_PORT           B
#define RF69_SDO                PB4
#define RF69_SCK_PORT           B
#define RF69_SCK                PB5

// INT0 on AVRs should be connected to RFM69's DIO0 (ex on Atmega328 it's D2, on Atmega644/1284 it's D2)
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
  #define RF69_IRQ_PORT         D
  #define RF69_IRQ              PD2
  #define RF69_IRQ_NUM          0
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  #define RF69_IRQ_PORT         D
  #define RF69_IRQ              PD2
  #define RF69_IRQ_NUM          2
#elif defined(__AVR_ATmega32U4__)
  #define RF69_IRQ_PORT         D
  #define RF69_IRQ              PD0
  #define RF69_IRQ_NUM          0
#endif

#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625 // == FXOSC/2^19 = 32mhz/2^19 (p13 in DS)

extern volatile uint8_t rfm69_DATA[RF69_MAX_DATA_LEN]; // recv/xmit buf, including hdr & crc bytes
extern volatile uint8_t rfm69_DATALEN;
extern volatile uint8_t rfm69_SENDERID;
extern volatile uint8_t rfm69_TARGETID; // should match _address
extern volatile uint8_t rfm69_PAYLOADLEN;
extern volatile uint8_t rfm69_ACK_REQUESTED;
extern volatile uint8_t rfm69_ACK_RECEIVED; // should be polled immediately after sending a packet with ACK request
extern volatile int rfm69_RSSI; // most accurate RSSI during reception (closest to the reception)

bool rfm69_initialize(uint8_t freqBand, uint8_t ID, uint8_t networkID);
void rfm69_setAddress(uint8_t addr);
void rfm69_setNetwork(uint8_t networkID);
bool rfm69_canSend(void);
void rfm69_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK);
bool rfm69_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime); // 40ms roundtrip req for  61byte packets
bool rfm69_receiveDone(void);
bool rfm69_ACKReceived(uint8_t fromNodeID);
bool rfm69_ACKRequested(void);
void rfm69_sendACK(const void* buffer, uint8_t bufferSize);
uint32_t rfm69_getFrequency(void);
void rfm69_setFrequency(uint32_t freqHz);
void rfm69_encrypt(const char* key);
int rfm69_readRSSI(bool forceTrigger);
void rfm69_promiscuous(bool onOff);
void rfm69_setHighPower(bool onOFF); // have to call it after initialize for RFM69HW
void rfm69_setPowerLevel(uint8_t level); // reduce/increase transmit power level
void rfm69_sleep(void);
uint8_t rfm69_readTemperature(uint8_t calFactor); // get CMOS temperature (8bit)
void rfm69_rcCalibration(void); // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]

// allow hacking registers by making these public
uint8_t rfm69_readReg(uint8_t addr);
void rfm69_writeReg(uint8_t addr, uint8_t val);

#endif
