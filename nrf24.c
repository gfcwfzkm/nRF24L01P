#include "nrf24.h"

/* All timing values are times 10us */
#define DELAY_STARTUP	15
#define DELAY_POWERON	500

#define SPIBUF_LEN	33
static uint8_t nspiBuf[SPIBUF_LEN];	// STATUS + 32 Byte RxPacket is the largest packet size

static int8_t _isPowerOfTwo(uint8_t n)
{
	return n && (!(n & (n - 1)));
}

static int8_t _findBitPosition(uint8_t n)
{
	uint8_t counter = 0;

	if (!_isPowerOfTwo(n))
	{
		return -1;
	}

	while(n)
	{
		n = n >> 1;
		counter++;
	}

	return counter;
}

void nrf24_setupStruct(nrf24_t *nrf, void *ioInt, uint8_t (*startTrans)(void*),
		uint8_t (*transBytes)(void*,uint8_t,uint8_t*,uint16_t), uint8_t (*endTrans)(void*),
		void (*highActiveCE)(void*), void (*lowInactiveCE)(void*), void (*delay10us)(void))
{
	nrf->status = NRF24_NO_STATUS;
	nrf->ioInt = ioInt;
	nrf->startTrans = startTrans;
	nrf->transBytes = transBytes;
	nrf->endTrans = endTrans;
	nrf->highActiveCE = highActiveCE;
	nrf->lowInactiveCE = lowInactiveCE;
	nrf->delay10us = delay10us;
}

uint8_t nrf24_init(nrf24_t *nrf)
{
	uint8_t tmpReg, oldReg;

	if (nrf->delay10us != NULL)
	{
		for (uint8_t i = DELAY_STARTUP; i > 0; i--)
		{
			nrf->delay10us();
		}
	}

	nrf24_writeReg(nrf, NRF24_R_CONFIG, 0);

	// Set retries and timing for the worst-case scenario (32B payload @ 250kB/s)
	nrf24_setRetries(nrf, 15, 5);

	// Enable EnhancedShockburst on All Pipes
	nrf24_enableAutoAck(nrf);
	// Disable all receive pipes except pipe 0 and 1
	nrf24_setEnRxPipe(nrf, NRF24_RXPIPE_0 | NRF24_RXPIPE_1);

	// Disable dynamic payloads, set a size of 32 bytes
	nrf24_setRxPayloadSize(nrf, NRF24_RXPIPE_ALL, NRF24_PLSIZE_32);
	nrf24_setTxDynamicPayload(nrf, 0);

	// Use 5-Bytes Addressing
	nrf24_setAddressWidth(nrf, NRF24_ADDR_5BYTES);

	// Set default speed to 1 MHz and low strength
	nrf24_writeReg(nrf, NRF24_R_RF_SETUP, NRF24_POWER_LOW | NRF24_DR_1MBIT);
	
	// Set channel to 76
	nrf24_setChannel(nrf, 76);

	// Reset all current statuses 
	nrf24_writeReg(nrf, NRF24_R_STATUS, (NRF24_R_STATUS_RX_DR | NRF24_R_STATUS_TX_DS | NRF24_R_STATUS_MAX_RT));

	// Flush both FIFOs
	nrf24_flushRx(nrf);
	nrf24_flushTx(nrf);

	// Reset the config registers to PTX, Power-Up & 16bit CRC
	oldReg = (NRF24_R_CONFIG_CRCO | NRF24_R_CONFIG_EN_CRC | NRF24_R_CONFIG_PWR_UP);
	nrf24_writeReg(nrf, NRF24_R_CONFIG, oldReg);
	tmpReg = nrf24_readReg(nrf, NRF24_R_CONFIG);

	return (tmpReg == oldReg ? 0 : 1);
}

/* BASIC I/O FUNCTIONS */
void nrf24_writeRegs(nrf24_t* nrf, uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t spiCnt;
	if (len > 32)	len = 32;
	spiCnt = len + 1;

	while(spiCnt--)
	{
		if (spiCnt)
		{
			nspiBuf[spiCnt] = buf[spiCnt - 1];
		}
		else
		{
			nspiBuf[0] = NRF24_C_W_REG | (reg & NRF24_C_RW_MASK);
		}
	}

	if (nrf->startTrans(nrf->ioInt))
	{
		nrf->status = NRF24_IOERROR;
	}
	else
	{
		nrf->transBytes(nrf->ioInt, 0, nspiBuf, len + 1);
		nrf->status = nspiBuf[0];
	}
	nrf->endTrans(nrf->ioInt);
}

void nrf24_writeReg(nrf24_t* nrf, uint8_t reg, uint8_t regVal)
{
	nrf24_writeRegs(nrf, reg, &regVal, 1);
}

void nrf24_writeCommand(nrf24_t *nrf, uint8_t cmd)
{
	if (nrf->startTrans(nrf->ioInt))
	{
		nrf->status = NRF24_IOERROR;
	}
	else
	{
		nrf->transBytes(nrf->ioInt, 0, &cmd, 1);
		nrf->status = cmd;
	}
	nrf->endTrans(nrf->ioInt);
}

uint8_t nrf24_readReg(nrf24_t *nrf, uint8_t reg)
{
	uint8_t regVal[2] = {NRF24_C_R_REG | (reg & NRF24_C_RW_MASK), NRF24_C_NOP};

	if (nrf->startTrans(nrf->ioInt))
	{
		nrf->status = NRF24_IOERROR;
	}
	else
	{
		nrf->transBytes(nrf->ioInt, 0, regVal, 2);
		nrf->status = regVal[0];
	}
	nrf->endTrans(nrf->ioInt);

	return regVal[1];
}

void nrf24_readRegs(nrf24_t *nrf, uint8_t reg, uint8_t *buf, uint8_t len)
{
	nspiBuf[0] = NRF24_C_R_REG | (reg & NRF24_C_RW_MASK);
	uint8_t spiCnt = len + 1;

	for (uint8_t i = 1; i < (len+1); i++)
	{
		nspiBuf[i] = NRF24_C_NOP;
	}

	if (nrf->startTrans(nrf->ioInt))
	{
		nrf->status = NRF24_IOERROR;
	}
	else
	{
		nrf->transBytes(nrf->ioInt, 0, nspiBuf, len);
		nrf->status = nspiBuf[0];

		while((spiCnt--) - 1)
		{
			buf[spiCnt - 1] = nspiBuf[spiCnt];
		}
	}
	nrf->endTrans(nrf->ioInt);
}


void nrf24_sendMessage(nrf24_t *nrf, uint8_t *txAddress, nrf24_message_t *message)
{
	// Set destination address
	nrf24_setTxAddress(nrf, txAddress);

	// Write into FIFO
	nrf24_writePayload(nrf, message->payload, message->length, message->writeType);
	
	// Setup Interrupts
	uint8_t tmpReg = nrf24_readReg(nrf, NRF24_R_CONFIG);
	tmpReg |= NRF24_R_CONFIG_MASK_RX_DR;
	tmpReg &=~ (NRF24_R_CONFIG_PRIM_RX | NRF24_R_CONFIG_MASK_MAX_RT | NRF24_R_CONFIG_MASK_TX_DS);
	nrf24_writeReg(nrf, NRF24_R_CONFIG, tmpReg);

	// Ensure that no Interrupts are open / pending
	nrf24_writeReg(nrf, NRF24_R_STATUS, NRF24_R_STATUS_RX_DR | NRF24_R_STATUS_TX_DS | NRF24_R_STATUS_MAX_RT);

	// Short burst
	nrf->highActiveCE(nrf->ioInt);
	if (nrf->delay10us)
	{
		nrf->delay10us();
		nrf->lowInactiveCE(nrf->ioInt);
	}
}

uint8_t nrf24_recvMessage(nrf24_t *nrf, nrf24_message_t *message)
{
	uint8_t regVal[2] = {NRF24_C_R_RX_PL_WID, NRF24_C_NOP};
	uint8_t rxpipe = nrf24_getStatus(nrf) & NRF24_R_STATUS_RX_P_NO_MASK;
	
	// check if the message is on a valid pipe
	if (rxpipe >= 0b110)
	{
		return 1;
	}

	message->rxPipe = (1<<rxpipe);

	// Check if dynamic payload size is enabled for that pipe
	if (nrf->PRXdynamicPayload & message->rxPipe)
	{
		// Get the size of the received payload
		if (nrf->startTrans(nrf->ioInt))
		{
			nrf->status = NRF24_IOERROR;
		}
		else
		{
			nrf->transBytes(nrf->ioInt, 0, regVal, 2);
			nrf->status = regVal[0];
			message->length = regVal[1];
		}
		nrf->endTrans(nrf->ioInt);
	}
	else
	{
		message->length = nrf24_readReg(nrf, _findBitPosition(message->rxPipe) + NRF24_R_RX_PW_0);
	}

	if ((nrf->status & NRF24_IOERROR) || (message->length > 32))
	{
		return 2;
	}

	nrf24_readPayload(nrf, message->payload, message->length);

	return 0;
}

void nrf24_startListening(nrf24_t *nrf)
{
	// Turn off the TX interrupts
	uint8_t tmpReg = nrf24_readReg(nrf, NRF24_R_CONFIG);
	tmpReg |= (NRF24_R_CONFIG_MASK_MAX_RT | NRF24_R_CONFIG_MASK_TX_DS);
	nrf24_writeReg(nrf, NRF24_R_CONFIG, tmpReg);
	
	// Ensure that no Interrupts are open / pending
	nrf24_writeReg(nrf, NRF24_R_STATUS, NRF24_R_STATUS_RX_DR | NRF24_R_STATUS_TX_DS | NRF24_R_STATUS_MAX_RT);
	
	// Set the radio to receive data
	tmpReg |= (NRF24_R_CONFIG_PRIM_RX);
	tmpReg &=~ (NRF24_R_CONFIG_MASK_RX_DR);
	nrf24_writeReg(nrf, NRF24_R_CONFIG, tmpReg);

	// Start Listening
	nrf->highActiveCE(nrf->ioInt);
}

void nrf24_stopListening(nrf24_t *nrf)
{
	// Stop Listening
	nrf->lowInactiveCE(nrf->ioInt);

	// Ensure that 4us have passed before we access the SPI bus
	if (nrf->delay10us)	nrf->delay10us();

	// Set up the radio chip to transmit data in order to get to Standby-I
	// Disable the receive interrupts
	uint8_t tmpReg = nrf24_readReg(nrf, NRF24_R_CONFIG);
	tmpReg |= ( NRF24_R_CONFIG_MASK_RX_DR | NRF24_R_CONFIG_PRIM_RX);
	nrf24_writeReg(nrf, NRF24_R_CONFIG, tmpReg);

	// Ensure that no Interrupts are open / pending
	nrf24_writeReg(nrf, NRF24_R_STATUS, NRF24_R_STATUS_RX_DR);
}

nrf24_status_e nrf24_checkIRQ(nrf24_t *nrf)
{
	nrf24_status_e status = nrf24_getStatus(nrf);
	return (status & (NRF24_IRQ_DATATX_SENT | NRF24_IRQ_DATARX_RECV | NRF24_IRQ_MAX_RETR));
}

void nrf24_setTxAddress(nrf24_t *nrf, uint8_t *txAddress)
{
	nrf24_writeRegs(nrf, NRF24_R_TX_ADDR, txAddress, nrf->addrWidth + 2);
}

void nrf24_setRxAddress(nrf24_t *nrf, uint8_t *rxAddress, nrf24_RxPipe_e pipe)
{
	int8_t pipePos = _findBitPosition(pipe);
	if ((pipePos < 0) || (pipePos > 5))		return;
	
	nrf24_writeRegs(nrf, NRF24_R_RX_PW_0 + pipePos, rxAddress, (pipePos > 1) ? 1 : (nrf->addrWidth + 2));
}

void nrf24_writePayload(nrf24_t *nrf, uint8_t *dataBuffer, uint8_t bufLen, nrf24_writeType_e type)
{
	uint8_t spiCnt;
	if (bufLen > 32)	bufLen = 32;
	spiCnt = bufLen + 1;

	while(spiCnt--)
	{
		if (spiCnt)
		{
			nspiBuf[spiCnt] = dataBuffer[spiCnt - 1];
		}
		else
		{
			nspiBuf[0] = type;
		}
	}

	if (nrf->startTrans(nrf->ioInt))
	{
		nrf->status = NRF24_IOERROR;
	}
	else
	{
		nrf->transBytes(nrf->ioInt, 0, nspiBuf, bufLen + 1);
		nrf->status = nspiBuf[0];
	}
	nrf->endTrans(nrf->ioInt);
}

void nrf24_readPayload(nrf24_t *nrf, uint8_t *dataBuffer, uint8_t len)
{
	nspiBuf[0] = NRF24_C_R_RX_PAYLOAD;
	uint8_t spiCnt = len + 1;

	if (nrf->startTrans(nrf->ioInt))
	{
		nrf->status = NRF24_IOERROR;
	}
	else
	{
		nrf->transBytes(nrf->ioInt, 0, nspiBuf, len);
		nrf->status = nspiBuf[0];

	}
	nrf->endTrans(nrf->ioInt);
	
	while((spiCnt--) - 1)
	{
		dataBuffer[spiCnt - 1] = nspiBuf[spiCnt];
	}
}

nrf24_status_e nrf24_getStatus(nrf24_t *nrf)
{
	nrf24_writeCommand(nrf, NRF24_C_NOP);
	return nrf->status;
}

void nrf24_powerUp(nrf24_t *nrf)
{
	uint8_t tmpReg = nrf24_readReg(nrf, NRF24_R_CONFIG);
	tmpReg |= NRF24_R_CONFIG_PWR_UP;
	nrf24_writeReg(nrf, NRF24_R_CONFIG, tmpReg);

	if (nrf->delay10us != NULL)
	{
		for (uint16_t i = DELAY_POWERON; i > 0; i--)
		{
			nrf->delay10us();
		}
	}
}

void nrf24_powerDown(nrf24_t *nrf)
{
	uint8_t tmpReg;
	nrf->lowInactiveCE(nrf->ioInt);

	tmpReg = nrf24_readReg(nrf, NRF24_R_CONFIG);
	tmpReg &= ~NRF24_R_CONFIG_PWR_UP;
	nrf24_writeReg(nrf, NRF24_R_CONFIG, tmpReg);
}

uint8_t nrf24_isConnected(nrf24_t *nrf)
{
	uint8_t tmpReg = nrf24_readReg(nrf, NRF24_R_SETUP_AW);
	if (tmpReg > NRF24_R_SETUP_AW_5BYTES || tmpReg < NRF24_R_SETUP_AW_3BYTES)
	{
		return 0;
	}
	return 1;
}

void nrf24_setAddressWidth(nrf24_t *nrf, nrf24_addrWidth_e addrWidth)
{
	nrf24_writeReg(nrf, NRF24_R_SETUP_AW, addrWidth);
	nrf->addrWidth = addrWidth;
}

void nrf24_setRetries(nrf24_t *nrf, uint8_t delays, uint8_t retries)
{
	uint8_t tmpReg = (((delays & 0x0F) << 4) | (retries & 0x0F));
	nrf24_writeReg(nrf, NRF24_R_SETUP_RETR, tmpReg);
}

void nrf24_setChannel(nrf24_t *nrf, uint8_t channel)
{
	uint8_t tmpReg = channel & NRF24_R_RF_CH_MASK;
	nrf24_writeReg(nrf, NRF24_R_RF_CH, tmpReg);
}

uint8_t nrf24_getChannel(nrf24_t *nrf)
{
	return nrf24_readReg(nrf, NRF24_R_RF_CH);
}

void nrf24_setRxPayloadSize(nrf24_t *nrf, nrf24_RxPipe_e pipes, nrf24_payloadSize_e plSize)
{
	uint8_t baseRegister = NRF24_R_RX_PW_0;

	pipes &= NRF24_RXPIPE_ALL;

	while (pipes)
	{
		if (pipes & 0x01)
		{
			nrf24_writeReg(nrf, baseRegister, plSize);
		}

		baseRegister++;
		pipes >>= 1;
	}
}

uint8_t nrf24_getRxPayloadSize(nrf24_t *nrf, nrf24_RxPipe_e pipe)
{
	int8_t tmpReg = _findBitPosition(pipe);

	if (tmpReg < 0)	return 0;

	tmpReg += NRF24_R_RX_PW_0;

	return nrf24_readReg(nrf, tmpReg);
}

uint8_t nrf24_getRxDynPayloadSize(nrf24_t* nrf)
{
	nspiBuf[0] = NRF24_C_R_RX_PL_WID;
	nspiBuf[1] = NRF24_C_NOP;

	if (nrf->startTrans(nrf->ioInt))
	{
		nrf->transBytes(nrf->ioInt, 0, nspiBuf, 2);

		nrf->status = nspiBuf[0];
	}
	nrf->endTrans(nrf->ioInt);

	return nspiBuf[1];
}

void nrf24_setTxDynamicPayload(nrf24_t *nrf, uint8_t onBit)
{
	uint8_t tmpReg = nrf24_readReg(nrf, NRF24_R_FEATURE);
	
	if (onBit)
	{
		tmpReg |= NRF24_R_FEATURE_EN_DPL;
	}
	else
	{
		tmpReg &=~ NRF24_R_FEATURE_EN_DPL;
	}

	nrf24_writeReg(nrf, NRF24_R_FEATURE, tmpReg);
	nrf->PTXdynamicPayload = onBit & 1;
}

void nrf24_enableAutoAck(nrf24_t *nrf)
{
	nrf24_writeReg(nrf, NRF24_R_EN_AA, NRF24_RXPIPE_ALL);
	nrf->PRXdynamicPayload = NRF24_RXPIPE_ALL;
}
void nrf24_disableAutoAck(nrf24_t *nrf)
{
	nrf24_writeReg(nrf, NRF24_R_EN_AA, 0);
	nrf->PRXdynamicPayload = 0;
}

void nrf24_setAutoAckPipe(nrf24_t *nrf, nrf24_RxPipe_e pipes)
{
	nrf24_writeReg(nrf, NRF24_R_EN_AA, pipes & NRF24_RXPIPE_ALL);
	nrf->PRXdynamicPayload = pipes & NRF24_RXPIPE_ALL;
}

void nrf24_setEnRxPipe(nrf24_t *nrf, nrf24_RxPipe_e pipes)
{
	nrf24_writeReg(nrf, NRF24_R_EN_RXADDR, pipes & NRF24_RXPIPE_ALL);
}

void nrf24_setRFPower(nrf24_t *nrf, nrf24_power_e pwrLevel)
{
	uint8_t tmpReg = nrf24_readReg(nrf, NRF24_R_RF_SETUP);

	tmpReg &= ~(NRF24_POWER_MAX);
	tmpReg |= pwrLevel;

	nrf24_writeReg(nrf, NRF24_R_RF_SETUP, tmpReg);
}

nrf24_power_e nrf24_getRFPower(nrf24_t *nrf)
{
	return (nrf24_readReg(nrf,NRF24_R_RF_SETUP) & (NRF24_POWER_MAX));
}

void nrf24_getTXCounters(nrf24_t *nrf, uint8_t *lostPacketsCnt, uint8_t *retransmittedCnt)
{
	uint8_t tmpReg = nrf24_readReg(nrf, NRF24_R_OBSERVE_TX);

	if (lostPacketsCnt != NULL)
		*lostPacketsCnt = (tmpReg & NRF24_R_OBSERVE_TX_PLOS_CNT) >> 4;
	
	if (retransmittedCnt != NULL)
		*retransmittedCnt = tmpReg & NRF24_R_OBSERVE_TX_ARC_CNT;
}

void nrf24_setDataRate(nrf24_t *nrf, nrf24_datarate_e dataRate)
{
	uint8_t tmpReg = nrf24_readReg(nrf, NRF24_R_RF_SETUP);

	tmpReg &=~ (NRF24_R_RF_SETUP_RF_DR_HIGH | NRF24_R_RF_SETUP_RF_DR_LOW);
	tmpReg |= dataRate;

	nrf24_writeReg(nrf, NRF24_R_RF_SETUP, tmpReg);
}

nrf24_addrWidth_e nrf24_getDataRate(nrf24_t *nrf)
{
	uint8_t tmpReg = nrf24_readReg(nrf, NRF24_R_RF_SETUP);

	tmpReg &= (NRF24_R_RF_SETUP_RF_DR_HIGH | NRF24_R_RF_SETUP_RF_DR_LOW);

	return tmpReg;
}

void nrf24_setCRCType(nrf24_t *nrf, nrf24_crc_e crcType)
{
	uint8_t tmpReg = nrf24_readReg(nrf, NRF24_R_CONFIG);

	tmpReg &=~ NRF24_CRC_MASK;
	tmpReg |= (crcType & NRF24_CRC_MASK);

	nrf24_writeReg(nrf, NRF24_R_CONFIG, tmpReg);
}

nrf24_crc_e nrf24_getCRCType(nrf24_t *nrf)
{
	uint8_t tmpReg = nrf24_readReg(nrf, NRF24_R_CONFIG);

	return tmpReg &= NRF24_CRC_MASK;
}

void nrf24_flushRx(nrf24_t *nrf)
{
	nrf24_writeCommand(nrf, NRF24_C_FLUSH_RX);
}

void nrf24_flushTx(nrf24_t *nrf)
{
	nrf24_writeCommand(nrf, NRF24_C_FLUSH_TX);
}
