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
#include <RFM69.h>
#include <RFM69registers.h>
#include <SPI.h>

volatile uint8_t rfm69_DATA[RF69_MAX_DATA_LEN];
volatile uint8_t rfm69_DATALEN;
volatile uint8_t rfm69_SENDERID;
volatile uint8_t rfm69_TARGETID;      // should match _address
volatile uint8_t rfm69_PAYLOADLEN;
volatile uint8_t rfm69_ACK_REQUESTED;
volatile uint8_t rfm69_ACK_RECEIVED;  // should be polled immediately after sending a packet with ACK request
volatile int rfm69_RSSI;              // most accurate RSSI during reception (closest to the reception)

// internal private variables and functions
void _rfm69_isr0(void);
void _rfm69_interruptHandler(void);
void _rfm69_sendFrame(uint8_t toAddress, const void* buffer, uint8_t size, bool requestACK, bool sendACK);

static volatile uint8_t _mode = RF69_MODE_STANDBY; // current transceiver state
static uint8_t _address;
static bool _promiscuousMode = false;
static uint8_t _powerLevel = 31;
static bool _isRFM69HW = false;
static uint8_t _SPCR;
static uint8_t _SPSR;

void _rfm69_receiveBegin(void);
void _rfm69_setMode(uint8_t mode);
void _rfm69_setHighPowerRegs(bool onOff);
void _rfm69_select(void);
void _rfm69_unselect(void);

bool rfm69_initialize(uint8_t freqBand, uint8_t nodeID, uint8_t networkID)
{
  const uint8_t CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default:4.8 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default:5khz, (FDEV + BitRate/2 <= 500Khz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

    /* 0x07 */ { REG_FRFMSB, (uint8_t)(freqBand==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
    /* 0x08 */ { REG_FRFMID, (uint8_t)(freqBand==RF69_315MHZ ? RF_FRFMID_315 : (freqBand==RF69_433MHZ ? RF_FRFMID_433 : (freqBand==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
    /* 0x09 */ { REG_FRFLSB, (uint8_t)(freqBand==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout=-18+OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout=-14+OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout=-11+OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4khz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2) - default is 0xE4=228 so -114dBm
    ///* 0x2d */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    /* 0x2e */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2f */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
    /* 0x30 */ { REG_SYNCVALUE2, networkID }, // NETWORK ID
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
    ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
    /* 0x3d */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    //for BR-19200: /* 0x3d */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    ///* 0x6F */ { REG_TESTDAGC, RF_DAGC_CONTINUOUS }, // run DAGC continuously in RX mode
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode, recommended default for AfcLowBetaOn=0
    {255, 0}
  };

  DDR(RF69_CS_PORT) |= (1<<RF69_CS);
  SPI.begin();

  do rfm69_writeReg(REG_SYNCVALUE1, 0xaa); while (rfm69_readReg(REG_SYNCVALUE1) != 0xaa);
  do rfm69_writeReg(REG_SYNCVALUE1, 0x55); while (rfm69_readReg(REG_SYNCVALUE1) != 0x55);

  for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
    rfm69_writeReg(CONFIG[i][0], CONFIG[i][1]);

  // Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
  rfm69_encrypt(0);

  rfm69_setHighPower(_isRFM69HW); // called regardless if it's a RFM69W or RFM69HW
  _rfm69_setMode(RF69_MODE_STANDBY);
  while ((rfm69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  attachInterrupt(RF69_IRQ_NUM, _rfm69_isr0, RISING);

  _address = nodeID;
  return true;
}

// return the frequency (in Hz)
uint32_t rfm69_getFrequency(void)
{
  return RF69_FSTEP * (((uint32_t)rfm69_readReg(REG_FRFMSB)<<16) + ((uint16_t)rfm69_readReg(REG_FRFMID)<<8) + rfm69_readReg(REG_FRFLSB));
}

// set the frequency (in Hz)
void rfm69_setFrequency(uint32_t freqHz)
{
  // TODO: p38 hopping sequence may need to be followed in some cases
  freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
  rfm69_writeReg(REG_FRFMSB, freqHz >> 16);
  rfm69_writeReg(REG_FRFMID, freqHz >> 8);
  rfm69_writeReg(REG_FRFLSB, freqHz);
}

void _rfm69_setMode(uint8_t newMode)
{
  if (newMode == _mode) return; // TODO: can remove this?

  switch (newMode) {
    case RF69_MODE_TX:
      rfm69_writeReg(REG_OPMODE, (rfm69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      if (_isRFM69HW) _rfm69_setHighPowerRegs(true);
      break;
    case RF69_MODE_RX:
      rfm69_writeReg(REG_OPMODE, (rfm69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      if (_isRFM69HW) _rfm69_setHighPowerRegs(false);
      break;
    case RF69_MODE_SYNTH:
      rfm69_writeReg(REG_OPMODE, (rfm69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
      rfm69_writeReg(REG_OPMODE, (rfm69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
      rfm69_writeReg(REG_OPMODE, (rfm69_readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default: return;
  }

  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == RF69_MODE_SLEEP && (rfm69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  _mode = newMode;
}

void rfm69_sleep(void) {
  _rfm69_setMode(RF69_MODE_SLEEP);
}

void rfm69_setAddress(uint8_t addr)
{
  _address = addr;
  rfm69_writeReg(REG_NODEADRS, _address);
}

void rfm69_setNetwork(uint8_t networkID)
{
  rfm69_writeReg(REG_SYNCVALUE2, networkID);
}

// set output power: 0=min, 31=max
// this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
void rfm69_setPowerLevel(uint8_t powerLevel)
{
  _powerLevel = powerLevel;
  rfm69_writeReg(REG_PALEVEL, (rfm69_readReg(REG_PALEVEL) & 0xE0) | (_powerLevel > 31 ? 31 : _powerLevel));
}

bool rfm69_canSend(void)
{
  if (_mode == RF69_MODE_RX && rfm69_PAYLOADLEN == 0 && rfm69_readRSSI(false) < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
  {
    _rfm69_setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

void rfm69_send(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK)
{
  rfm69_writeReg(REG_PACKETCONFIG2, (rfm69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  unsigned long now = millis();
  while (!rfm69_canSend() && millis()-now < RF69_CSMA_LIMIT_MS) rfm69_receiveDone();
  _rfm69_sendFrame(toAddress, buffer, bufferSize, requestACK, false);
}

// to increase the chance of getting a packet across, call this function instead of rfm69_send
// and it handles all the ACK requesting/retrying for you :)
// The only twist is that you have to manually listen to ACK requests on the other side and send back the ACKs
// The reason for the semi-automaton is that the lib is interrupt driven and
// requires user action to read the received data and decide what to do with it
// replies usually take only 5-8ms at 50kbps@915Mhz
bool rfm69_sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries, uint8_t retryWaitTime) {
  unsigned long sentTime;
  for (uint8_t i = 0; i <= retries; i++)
  {
    rfm69_send(toAddress, buffer, bufferSize, true);
    sentTime = millis();
    while (millis()-sentTime<retryWaitTime)
    {
      if (rfm69_ACKReceived(toAddress))
      {
        //Serial.print(" ~ms:");Serial.print(millis()-sentTime);
        return true;
      }
    }
    //Serial.print(" RETRY#");Serial.println(i+1);
  }
  return false;
}

// should be polled immediately after sending a packet with ACK request
bool rfm69_ACKReceived(uint8_t fromNodeID) {
  if (rfm69_receiveDone())
    return (rfm69_SENDERID == fromNodeID || fromNodeID == RF69_BROADCAST_ADDR) && rfm69_ACK_RECEIVED;
  return false;
}

// check whether an ACK was requested in the last received packet (non-broadcasted packet)
bool rfm69_ACKRequested(void) {
  return rfm69_ACK_REQUESTED && (rfm69_TARGETID != RF69_BROADCAST_ADDR);
}

// should be called immediately after reception in case sender wants ACK
void rfm69_sendACK(const void* buffer, uint8_t bufferSize) {
  uint8_t sender = rfm69_SENDERID;
  int _RSSI = rfm69_RSSI; // save payload received RSSI value
  rfm69_writeReg(REG_PACKETCONFIG2, (rfm69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  unsigned long now = millis();
  while (!rfm69_canSend() && millis()-now < RF69_CSMA_LIMIT_MS) rfm69_receiveDone();
  _rfm69_sendFrame(sender, buffer, bufferSize, false, true);
  rfm69_RSSI = _RSSI; // restore payload RSSI
}

void _rfm69_sendFrame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK)
{
  _rfm69_setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((rfm69_readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  rfm69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;

  // write to FIFO
  _rfm69_select();
  SPI.transfer(REG_FIFO | 0x80);
  SPI.transfer(bufferSize + 3);
  SPI.transfer(toAddress);
  SPI.transfer(_address);

  // control byte
  if (sendACK)
    SPI.transfer(0x80);
  else if (requestACK)
    SPI.transfer(0x40);
  else SPI.transfer(0x00);

  for (uint8_t i = 0; i < bufferSize; i++)
    SPI.transfer(((uint8_t*)buffer)[i]);
  _rfm69_unselect();

  /* no need to wait for transmit mode to be ready since its handled by the radio */
  _rfm69_setMode(RF69_MODE_TX);
  unsigned long txStart = millis();
  while (!(PIN(RF69_IRQ_PORT) & (1<<RF69_IRQ)) && millis()-txStart < RF69_TX_LIMIT_MS); // wait for DIO0 to turn HIGH signalling transmission finish
  //while (rfm69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); // wait for ModeReady
  _rfm69_setMode(RF69_MODE_STANDBY);
}

void _rfm69_interruptHandler(void) {
  if (_mode == RF69_MODE_RX && (rfm69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    //_rfm69_RSSI = rfm69_readRSSI();
    _rfm69_setMode(RF69_MODE_STANDBY);
    _rfm69_select();
    SPI.transfer(REG_FIFO & 0x7f);
    rfm69_PAYLOADLEN = SPI.transfer(0);
    rfm69_PAYLOADLEN = rfm69_PAYLOADLEN > 66 ? 66 : rfm69_PAYLOADLEN; // precaution
    rfm69_TARGETID = SPI.transfer(0);
    if(!(_promiscuousMode || rfm69_TARGETID == _address || rfm69_TARGETID == RF69_BROADCAST_ADDR) // match this node's address, or broadcast address or anything in promiscuous mode
       || rfm69_PAYLOADLEN < 3) // address situation could receive packets that are malformed and don't fit this libraries extra fields
    {
      rfm69_PAYLOADLEN = 0;
      _rfm69_unselect();
      _rfm69_receiveBegin();
      return;
    }

    rfm69_DATALEN = rfm69_PAYLOADLEN - 3;
    rfm69_SENDERID = SPI.transfer(0);
    uint8_t CTLbyte = SPI.transfer(0);

    rfm69_ACK_RECEIVED = CTLbyte & 0x80; // extract ACK-received flag
    rfm69_ACK_REQUESTED = CTLbyte & 0x40; // extract ACK-requested flag

    for (uint8_t i = 0; i < rfm69_DATALEN; i++)
    {
      rfm69_DATA[i] = SPI.transfer(0);
    }
    if (rfm69_DATALEN<RF69_MAX_DATA_LEN) rfm69_DATA[rfm69_DATALEN] = 0; // add null at end of string
    _rfm69_unselect();
    _rfm69_setMode(RF69_MODE_RX);
  }
  rfm69_RSSI = rfm69_readRSSI(false);
}

void _rfm69_isr0(void) { _rfm69_interruptHandler(); }

void _rfm69_receiveBegin(void) {
  rfm69_DATALEN = 0;
  rfm69_SENDERID = 0;
  rfm69_TARGETID = 0;
  rfm69_PAYLOADLEN = 0;
  rfm69_ACK_REQUESTED = 0;
  rfm69_ACK_RECEIVED = 0;
  rfm69_RSSI = 0;
  if (rfm69_readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    rfm69_writeReg(REG_PACKETCONFIG2, (rfm69_readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  rfm69_writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
  _rfm69_setMode(RF69_MODE_RX);
}

bool rfm69_receiveDone(void) {
//ATOMIC_BLOCK(ATOMIC_FORCEON)
//{
  noInterrupts(); // re-enabled in _rfm69_unselect() via _rfm69_setMode() or via _rfm69_receiveBegin()
  if (_mode == RF69_MODE_RX && rfm69_PAYLOADLEN>0)
  {
    _rfm69_setMode(RF69_MODE_STANDBY); // enables interrupts
    return true;
  }
  else if (_mode == RF69_MODE_RX)  // already in RX no payload yet
  {
    interrupts(); // explicitly re-enable interrupts
    return false;
  }
  _rfm69_receiveBegin();
  return false;
//}
}

// To enable encryption: rfm69_encrypt("ABCDEFGHIJKLMNOP");
// To disable encryption: rfm69_encrypt(null) or rfm69_encrypt(0)
// KEY HAS TO BE 16 bytes !!!
void rfm69_encrypt(const char* key) {
  _rfm69_setMode(RF69_MODE_STANDBY);
  if (key != 0)
  {
    _rfm69_select();
    SPI.transfer(REG_AESKEY1 | 0x80);
    for (uint8_t i = 0; i < 16; i++)
      SPI.transfer(key[i]);
    _rfm69_unselect();
  }
  rfm69_writeReg(REG_PACKETCONFIG2, (rfm69_readReg(REG_PACKETCONFIG2) & 0xFE) | (key ? 1 : 0));
}

int rfm69_readRSSI(bool forceTrigger) {
  int rssi = 0;
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    rfm69_writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((rfm69_readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
  }
  rssi = -rfm69_readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}

uint8_t rfm69_readReg(uint8_t addr)
{
  _rfm69_select();
  SPI.transfer(addr & 0x7F);
  uint8_t regval = SPI.transfer(0);
  _rfm69_unselect();
  return regval;
}

void rfm69_writeReg(uint8_t addr, uint8_t value)
{
  _rfm69_select();
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  _rfm69_unselect();
}

// select the transceiver
void _rfm69_select(void) {
  noInterrupts();
  // save current SPI settings
  _SPCR = SPCR;
  _SPSR = SPSR;
  // set RFM69 SPI settings
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4); // decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
  PORT(RF69_CS_PORT) &= ~(1<<RF69_CS);
}

// UNselect the transceiver chip
void _rfm69_unselect(void) {
  PORT(RF69_CS_PORT) |= (1<<RF69_CS);
  // restore SPI settings to what they were before talking to RFM69
  SPCR = _SPCR;
  SPSR = _SPSR;
  interrupts();
}

// ON  = disable filtering to capture all frames on network
// OFF = enable node+broadcast filtering to capture only frames sent to this/broadcast address
void rfm69_promiscuous(bool onOff) {
  _promiscuousMode = onOff;
  //rfm69_writeReg(REG_PACKETCONFIG1, (rfm69_readReg(REG_PACKETCONFIG1) & 0xF9) | (onOff ? RF_PACKET1_ADRSFILTERING_OFF : RF_PACKET1_ADRSFILTERING_NODEBROADCAST));
}

void rfm69_setHighPower(bool onOff) {
  _isRFM69HW = onOff;
  rfm69_writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (_isRFM69HW) // turning ON
    rfm69_writeReg(REG_PALEVEL, (rfm69_readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
  else
    rfm69_writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // enable P0 only
}

void _rfm69_setHighPowerRegs(bool onOff) {
  rfm69_writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  rfm69_writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

// for debugging
void rfm69_readAllRegs(void)
{
  uint8_t regVal;

  for (uint8_t regAddr = 1; regAddr <= 0x4F; regAddr++)
  {
    _rfm69_select();
    SPI.transfer(regAddr & 0x7f); // send address + r/w bit
    regVal = SPI.transfer(0);
    _rfm69_unselect();

    Serial.print(regAddr, HEX);
    Serial.print(" - ");
    Serial.print(regVal,HEX);
    Serial.print(" - ");
    Serial.println(regVal,BIN);
  }
  _rfm69_unselect();
}

uint8_t rfm69_readTemperature(uint8_t calFactor) // returns centigrade
{
  _rfm69_setMode(RF69_MODE_STANDBY);
  rfm69_writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((rfm69_readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
  return ~rfm69_readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement'corrects the slope, rising temp = rising val
} // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction

void rfm69_rcCalibration(void)
{
  rfm69_writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((rfm69_readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}
