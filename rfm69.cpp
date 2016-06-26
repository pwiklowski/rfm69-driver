#include "rfm69.h"
#include "rfm69hal.h"
#include "rfm69registers.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

extern "C" {
    #include "printf.h"
}


#define log(line, ...) printf(line, ##__VA_ARGS__ )

#define TIMEOUT_MODE_READY    1000000 ///< Maximum amount of time until mode switch [ms]
#define TIMEOUT_PACKET_SENT   100 ///< Maximum amount of time until packet must be sent [ms]
#define TIMEOUT_CSMA_READY    500 ///< Maximum CSMA wait time for channel free detection [ms]
#define CSMA_RSSI_THRESHOLD   -85 ///< If RSSI value is smaller than this, consider channel as free [dBm]

uint8_t syncSentence[] = { 'w', 'i', 'k', 'l', 'o', 's', 'o', 'f', 't', 0, 0 };

static const uint8_t config[][2] = { { REG_OPMODE, 0x04 }, // RegOpMode: Standby Mode
		{ REG_DATAMODUL, 0x00 }, // RegDataModul: Packet mode, FSK, no shaping
		{ REG_BITRATEMSB, 0x00 }, // RegBitrateMsb: 10 kbps
		{ REG_BITRATELSB, 0x80 }, // RegBitrateLsb

		{ REG_FDEVMSB, 0x10 }, // RegFdevMsb: 20 kHz
		{ REG_FDEVLSB, 0x00 }, // RegFdevLsb

		{ REG_FRFMSB, 0xD9 }, // RegFrfMsb: 868,15 MHz
		{ REG_FRFMID, 0x09 }, // RegFrfMid
		{ REG_FRFLSB, 0x9A }, // RegFrfLsb
		{ REG_LNA, 0x88 }, // RegLNA: 200 Ohm impedance, gain set by AGC loop

		{ REG_RXBW, 0xe8 }, // RegRxBw: 25 kHz
		{ REG_AFCBW, 0xe0 },

		{ REG_PREAMBLEMSB, 0x00 }, // RegPreambleMsb: 3 bytes preamble
		{ REG_PREAMBLELSB, 0x03 }, // RegPreambleLsb
		{ REG_SYNCCONFIG, 0x88 }, // RegSyncConfig: Enable sync word, 2 bytes sync word
		{ REG_SYNCVALUE1, 0x41 }, // RegSyncValue1: 0x4148
		{ REG_SYNCVALUE2, 0x48 }, // RegSyncValue2
		{ REG_PACKETCONFIG1, 0xD0 }, // RegPacketConfig1: Variable length, CRC on, whitening
		{ REG_PAYLOADLENGTH, RFM69_MAX_PAYLOAD }, // RegPayloadLength: 64 bytes max payload
		{ REG_FIFOTHRESH, 0x8F }, // RegFifoThresh: TxStart on FifoNotEmpty, 15 bytes FifoLevel
		{ 0x58, 0x1B }, // RegTestLna: Normal sensitivity mode
		{ REG_TESTDAGC, 0x30 }, // RegTestDagc: Improved margin, use if AfcLowBetaOn=0 (default)

		};

// Clock constants. DO NOT CHANGE THESE!
#define RFM69_XO               32000000    ///< Internal clock frequency [Hz]
#define RFM69_FSTEP            61.03515625 ///< Step width of synthesizer [Hz]

Rfm69::Rfm69() {
	m_mode = RFM69_MODE_STANDBY;
	m_powerLevel = 0;
	_rssi = -127;
	_ookEnabled = false;
	_autoReadRSSI = false;
	_dataMode = RFM69_DATA_MODE_PACKET;
	_csmaEnabled = false;
	_rxBufferLength = 0;
}

Rfm69::~Rfm69() {

}

void Rfm69::reset() {
	rfm69hal_enable(true);
	rfm69hal_delay_ms(1);
	rfm69hal_enable(false);

	rfm69hal_delay_ms(10);
	m_mode = RFM69_MODE_STANDBY;
}

bool Rfm69::init() {
	rfm69hal_init();
	bool res = setConfig(config, sizeof(config) / 2);
	clearFIFO();
	return res;
}

uint8_t Rfm69::readRegister(uint8_t reg) {
	rfm69hal_enable(true);

	unsigned char data[2];
	data[0] = reg & 0x7F;
	data[1] = 0;

	rfm69hal_transfer(data, 2);

	rfm69hal_enable(false);

	return data[1];
}

void Rfm69::writeRegister(uint8_t reg, uint8_t value) {
	rfm69hal_enable(true);

	unsigned char data[2];
	data[0] = reg | 0x80;
	data[1] = value;

	rfm69hal_transfer(data, 2);
	rfm69hal_enable(false);
}

void Rfm69::setMode(Rfm69Mode mode) {
	if ((mode == m_mode) || (mode > RFM69_MODE_RX))
		return;

	writeRegister(REG_OPMODE, mode << 2);
	m_mode = mode;
}

void Rfm69::setPowerLevel(uint8_t power) {
	if (power > 31)
		power = 31;

	writeRegister(REG_PALEVEL, (readRegister(0x11) & 0xE0) | power);

	m_powerLevel = power;
}

bool Rfm69::setConfig(const uint8_t config[][2], unsigned int length) {
	for (unsigned int i = 0; i < length; i++) {
		writeRegister(config[i][0], config[i][1]);
		uint8_t val = readRegister(config[i][0]);

		if (config[i][1] != val) {
			return false;
		}
	}
	return false;
}

int Rfm69::sendPacket(uint8_t* packet, uint16_t len) {
	uint8_t sequence = 0;

    log("sendPacket %d\n", len);

	uint16_t bytesToBeSent = len;
	uint8_t* data = packet;

	syncSentence[9] = len >> 8;
	syncSentence[10] = len;

	if (sendWithAck(syncSentence, 11, sequence++) < 0)
		return -1;

	while (bytesToBeSent > 0) {
		if (bytesToBeSent > (RFM69_MAX_PAYLOAD - 1)) {
			if (sendWithAck(data, RFM69_MAX_PAYLOAD - 1, sequence++) < 0)
				return -1;
			bytesToBeSent -= RFM69_MAX_PAYLOAD - 1;
			data += RFM69_MAX_PAYLOAD - 1;
		} else {
			if (sendWithAck(data, bytesToBeSent, sequence++) < 0)
				return -1;
			bytesToBeSent = 0;
		}
	}

}

int Rfm69::receivePacket(uint8_t* buf, uint16_t maxSize) {
	uint8_t ack = 0;
	uint8_t expectedSeqNumber = 0;
	uint8_t* bufPos = buf;
	uint8_t chunk[RFM69_MAX_PAYLOAD+2];
	uint16_t bytesToReceive = sizeof(syncSentence);
	uint16_t bytesReceived = 0;

	int l = -1;
	while (bytesToReceive > 0) {

	    for (int i = 0; i < 20; i++) {
			l = read(chunk, RFM69_MAX_PAYLOAD+2, &ack);
			if (l > 0) break;
			rfm69hal_delay_ms(1);
		}
	    if (l<0) return -1;


	    log("received size=%d %d\n", l, ack);
		if (expectedSeqNumber != ack) {
            log("invalid ack %d %d\n", expectedSeqNumber, ack);
		} else if ((l == 11) && (ack == 0)){
			bytesToReceive = (chunk[9] << 8 | chunk[10]);
			bytesReceived = 0;
			expectedSeqNumber = 1;
			send(0, 0, ack);
			log("received header size=%d\n", bytesToReceive);
		} else {
			log("received size=%d\n", bytesToReceive);
			memcpy(bufPos, chunk, l);
			bufPos += l;
			bytesToReceive -= l;
			bytesReceived += l;
			send(0, 0, ack);
			expectedSeqNumber++;
		}
	}

	log("receivePacket %d\n", bytesReceived);
	return bytesReceived;
}

int Rfm69::sendWithAck(uint8_t* data, uint16_t len, uint8_t sequence) {
	uint8_t buf[RFM69_MAX_PAYLOAD + 2];
	for (int i = 0; i < 10; i++) {
        log("sendWithAck try=%d len=%d seq=%d\n", i, len, sequence);
		send(data, len, sequence);

		uint8_t ackSeq;
		int l;
        log("read ack\n");
		for(int t=0; t<10; t++) {
			l = read(buf, RFM69_MAX_PAYLOAD, &ackSeq);
			if (l == 0) break;
            log("waiting for ack\n");
			rfm69hal_delay_ms(1);
		}
        log("read ack %d \n", l);

		if (l==0 && ackSeq == sequence) {
            log("received ack =%d\n", ackSeq);
			return len;
		} else {
            log("wrong ack received, try again %d\n", i);
		}
	}
	return -1;
}

int Rfm69::send(uint8_t* data, unsigned int dataLength, uint8_t sequence) {
    log("send %d\n", dataLength);
	if (RFM69_MODE_SLEEP != m_mode) {
		setMode(RFM69_MODE_STANDBY);
		waitForModeReady();
	}

//  clearFIFO();

	/* Wait for a free channel, if CSMA/CA algorithm is enabled.
	 * This takes around 1,4 ms to finish if channel is free */
//  if (true == _csmaEnabled)
//  {
//    // Restart RX
//    writeRegister(0x3D, (readRegister(0x3D) & 0xFB) | 0x20);
//
//    // switch to RX mode
//    setMode(RFM69_MODE_RX);
//
//    // wait until RSSI sampling is done; otherwise, 0xFF (-127 dBm) is read
//
//    // RSSI sampling phase takes ~960 µs after switch from standby to RX
//    uint32_t timeEntry = mstimer_get();
//    while (((readRegister(0x23) & 0x02) == 0) && ((mstimer_get() - timeEntry) < 10));
//
//    while ((false == channelFree()) && ((mstimer_get() - timeEntry) < TIMEOUT_CSMA_READY))
//    {
//      // wait for a random time before checking again
//      delay_ms(10);
//
//      /* try to receive packets while waiting for a free channel
//       * and put them into a temporary buffer */
//      int bytesRead;
//      if ((bytesRead = _receive(_rxBuffer, RFM69_MAX_PAYLOAD)) > 0)
//      {
//        _rxBufferLength = bytesRead;
//
//        // module is in RX mode again
//
//        // Restart RX and wait until RSSI sampling is done
//        writeRegister(0x3D, (readRegister(0x3D) & 0xFB) | 0x20);
//        uint32_t timeEntry = mstimer_get();
//        while (((readRegister(0x23) & 0x02) == 0) && ((mstimer_get() - timeEntry) < 10));
//      }
//    }
//
//    setMode(RFM69_MODE_STANDBY);
//  }
	// transfer packet to FIFO
	rfm69hal_enable(true);

	uint8_t p[RFM69_MAX_PAYLOAD + 3];

	p[0] = 0x00 | 0x80;
	p[1] = dataLength + 1;
	p[2] = sequence;

	for (int i = 0; i < dataLength; i++) {
		p[3 + i] = data[i];
	}

	rfm69hal_transfer(p, dataLength + 3);

	rfm69hal_enable(false);
	setMode(RFM69_MODE_TX);

	waitForPacketSent();
	setMode(RFM69_MODE_STANDBY);
	return dataLength;
}

void Rfm69::clearFIFO() {
	writeRegister(0x28, 0x10);
}

void Rfm69::waitForModeReady() {
	uint32_t timeEntry = rfm69hal_get_timer_ms();
	while (((readRegister(0x27) & 0x80) == 0) && ((rfm69hal_get_timer_ms() - timeEntry) < TIMEOUT_MODE_READY));
}

void Rfm69::sleep() {
	setMode(RFM69_MODE_SLEEP);
}

int Rfm69::receive(uint8_t* data, unsigned int dataLength) {
	// check if there is a packet in the internal buffer and copy it
	if (_rxBufferLength > 0) {
		// copy only until dataLength, even if packet in local buffer is actually larger
		memcpy(data, _rxBuffer, dataLength);

		unsigned int bytesRead = _rxBufferLength;

		// empty local buffer
		_rxBufferLength = 0;

		return bytesRead;
	} else {
		// regular receive
		return read(data, dataLength, 0);
	}
}

bool Rfm69::isFifoNotEmpty() {
	uint8_t reg = readRegister(0x28);
	return reg & 0x40;
}

bool Rfm69::isPacketReady() {
	if (RFM69_MODE_RX != m_mode) {
		setMode(RFM69_MODE_RX);
		waitForModeReady();
	}

	uint8_t reg = readRegister(0x28);
	return reg & 0x4;
}

int Rfm69::read(uint8_t* data, unsigned int dataLength, uint8_t* sequence) {
	// go to RX mode if not already in this mode
	if (RFM69_MODE_RX != m_mode) {
		setMode(RFM69_MODE_RX);
		waitForModeReady();
	}

	// check for flag PayloadReady
	if (readRegister(0x28) & 0x04) {
		setMode(RFM69_MODE_STANDBY);
		unsigned int bytesRead = 0;
		uint8_t bytesInChunk = readRegister(0) - 1; //minus seq byte
		*sequence = readRegister(0);

		for (int i = 0; i < bytesInChunk; i++) {
			data[i] = readRegister(0);
		}

		if (true == _autoReadRSSI) {
			readRSSI();
		}

		setMode(RFM69_MODE_RX);
        log("read %d\n", bytesInChunk);
		return bytesInChunk;
	} else
		return -1;
}

bool Rfm69::setAESEncryption(const void* aesKey, unsigned int keyLength) {
	bool enable = false;

	// check if encryption shall be enabled or disabled
	if ((0 != aesKey) && (16 == keyLength))
		enable = true;

	// switch to standby
	setMode(RFM69_MODE_STANDBY);

	if (true == enable) {
		// transfer AES key to AES key register
		rfm69hal_enable(true);

		// address first AES MSB register

		uint8_t p[keyLength + 1];

		p[0] = 0x3E | 0x80;

		for (unsigned int i = 0; i < keyLength; i++)
			p[1 + i] = (((uint8_t*) aesKey)[i]);

		rfm69hal_transfer(p, keyLength + 1);

		rfm69hal_enable(false);
	}

	// set/reset AesOn Bit in packet config
	writeRegister(0x3D, (readRegister(0x3D) & 0xFE) | (enable ? 1 : 0));

	return enable;
}

/**
 * Wait until packet has been sent over the air or timeout.
 */
void Rfm69::waitForPacketSent() {
	uint32_t timeEntry = rfm69hal_get_timer_ms();

	while (((readRegister(0x28) & 0x08) == 0)
			&& ((rfm69hal_get_timer_ms() - timeEntry) < TIMEOUT_PACKET_SENT))
		;
}
int Rfm69::readRSSI() {
	_rssi = -readRegister(0x24) / 2;

	return _rssi;
}

int Rfm69::setPowerDBm(int8_t dBm) {
	/* Output power of module is from -18 dBm to +13 dBm
	 * in "low" power devices, -2 dBm to +20 dBm in high power devices */
	if (dBm < -18 || dBm > 20)
		return -1;

	uint8_t powerLevel = powerLevel = dBm + 18;

	writeRegister(0x11, 0x80 | powerLevel);
	return 0;
}

bool Rfm69::isChannelFree() {
	if (readRSSI() < CSMA_RSSI_THRESHOLD) {
		return true;
	} else {
		return false;
	}
}

