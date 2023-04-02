#ifndef _NRF24_H_
#define _NRF24_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <inttypes.h>
#include <stddef.h>

/* -------- nRF24 Commands -------- */
#define NRF24_C_R_REG				0x00	// Read command and status reg. 
											// Lower 5 bits = Register Map Address
#define NRF24_C_W_REG				0x20	// Write command and status reg. 
											// Lower 5 bits = Register Map Address
											// Executable in power down or standby modes only
#define NRF24_C_RW_MASK				0x1F
#define NRF24_C_R_RX_PAYLOAD		0x61	// Read RX-payload: 1 - 32 Bytes. Payload is
											// deleted from FIFO after read. (Used in RX mode)
#define NRF24_C_W_TX_PAYLOAD		0xA0	// Write TX-payload: 1 - 32 Bytes.
#define NRF24_C_FLUSH_TX			0xE1	// Flush TX FIFO, used in TX mode
#define NRF24_C_FLUSH_RX			0xE0	// Flush RX FIFO, used in RX mode
#define NRF24_C_REUSE_TX_PL			0xE3	// Used for a PTX device. Reuse last transmitted
											// payload. Active until W_TX_PAYLOAD or FLUSH_TX
											// is executed. Don't (de)activate during action
#define NRF24_C_R_RX_PL_WID			0x60	// Read RX payload width for the top R_RX_PAYLOAD
											// in the FIFO. Flush RX FIFO if larger than 32 B
#define NRF24_C_W_ACK_PAYLOAD		0xA8	// Used in RX mode. Write Payload together with ACK
											// packet on PIPE (lower three bits). Max 3 ACK
											// packets can be pending. 
#define NRF24_C_W_TX_PAYLOAD_NACK	0xB0    // Used in TX mode. Disables AUTOACK on this
											// specific packet.
#define NRF24_C_NOP					0xFF    // No Operation - Can be used to get STATUS reg.

/* -------- nRF24 Registers -------- */
#define NRF24_R_CONFIG				0x00	// Configuration Register (Default: 0x00)
#define NRF24_R_CONFIG_MASK_RX_DR		0x40	// Mask interrupt caused by RX_DR
#define NRF24_R_CONFIG_MASK_TX_DS		0x20	// Mask interrupt caused by TX_DS
#define NRF24_R_CONFIG_MASK_MAX_RT		0x10	// Mask Interrupt caused by MAX_RT
#define NRF24_R_CONFIG_EN_CRC			0x08	// Enable CRC. Forced high if EN_AA > 1
#define NRF24_R_CONFIG_CRCO				0x04	// CRC encoding scheme: 0 = 1 Byte, 1 = 2 Bytes
#define NRF24_R_CONFIG_PWR_UP			0x02	// 1: Power UP, 0: Power DOWN
#define NRF24_R_CONFIG_PRIM_RX			0x01	// Rx/Tx control (1: PRX, 0:PTX)
#define NRF24_R_EN_AA				0x01	// EnhancedShockBurst AutoAcknowledgement(D:0x3F)
#define NRF24_R_EN_AA_ENAA_P5			0x20	// Enable AutoAck on data pipe 5
#define NRF24_R_EN_AA_ENAA_P4			0x10	// Enable AutoAck on data pipe 4
#define NRF24_R_EN_AA_ENAA_P3			0x08	// Enable AutoAck on data pipe 3
#define NRF24_R_EN_AA_ENAA_P2			0x04	// Enable AutoAck on data pipe 2
#define NRF24_R_EN_AA_ENAA_P1			0x02	// Enable AutoAck on data pipe 1
#define NRF24_R_EN_AA_ENAA_P0			0x01	// Enable AutoAck on data pipe 0
#define NRF24_R_EN_RXADDR			0x02	// Enable RX Addresses (Default: 0x03)
#define NRF24_R_EN_RXADDR_ERX_P5		0x20	// Enable data pipe 5
#define NRF24_R_EN_RXADDR_ERX_P4		0x10	// Enable data pipe 4
#define NRF24_R_EN_RXADDR_ERX_P3		0x08	// Enable data pipe 3
#define NRF24_R_EN_RXADDR_ERX_P2		0x04	// Enable data pipe 2
#define NRF24_R_EN_RXADDR_ERX_P1		0x02	// Enable data pipe 1
#define NRF24_R_EN_RXADDR_ERX_P0		0x01	// Enable data pipe 0
#define NRF24_R_SETUP_AW			0x03	// Setup of Address Width for all pipes (D:0x03)
#define NRF24_R_SETUP_AW_MASK			0x03	// Rx / Tx Address Field width:
#define NRF24_R_SETUP_AW_3BYTES			0x01	// '01' - 3 Bytes
#define NRF24_R_SETUP_AW_4BYTES			0x02	// '10' - 4 Bytes
#define NRF24_R_SETUP_AW_5BYTES			0x03	// '11' - 5 Bytes
#define NRF24_R_SETUP_RETR			0x04	// Setup of Automatic Retransmission (D:0x03)
#define NRF24_R_SETUP_RETR_ARD_BIT3		0x80	// 0 to 15, 250uS steps starting from 250uS wait time
#define NRF24_R_SETUP_RETR_ARD_BIT2		0x40
#define NRF24_R_SETUP_RETR_ARD_BIT1		0x20
#define NRF24_R_SETUP_RETR_ARD_BIT0		0x10
#define NRF24_R_SETUP_RETR_ARC_BIT3		0x08	// Auto Retransmit Count (0 to 15)
#define NRF24_R_SETUP_RETR_ARC_BIT2		0x04
#define NRF24_R_SETUP_RETR_ARC_BIT1		0x02
#define NRF24_R_SETUP_RETR_ARC_BIT0		0x01
#define NRF24_R_RF_CH				0x05	// RF Channel (Default: 0x02)
#define NRF24_R_RF_CH_MASK				0x7F	// Sets frequency channel the nRF24 operates on
#define NRF24_R_RF_SETUP			0x06	// RF Setup Register (Default: 0x0E)
#define NRF24_R_RF_SETUP_CONT_WAVE		0x80	// Enables continous carrier transmit when high
#define NRF24_R_RF_SETUP_RF_DR_LOW		0x20	// Set RF Data Rate High Bit (see detail below)
#define NRF24_R_RF_SETUP_PLL_LOCK		0x10	// Force PLL lock signal (test only!)
#define NRF24_R_RF_SETUP_RF_DR_HIGH		0x08	// Set RF Data Rate Low Bit (see detail below)
/* RF_DR_LOW + RF_DR_HIGH set the bitrate:
 * RF_DR_LOW: 0, RF_DR_HIGH: 0 - 1Mbps
 * RF_DR_LOW: 0, RF_DR_HIGH: 1 - 2Mbps
 * RF_DR_LOW: 1, RF_DR_HIGH: 0 - 250kbps */
#define NRF24_R_RF_SETUP_RF_PWR_1		0x04	// Set RF output power in TX mode
#define NRF24_R_RF_SETUP_RF_PWR_0		0x02	// Set RF output power in TX mode
/* Set RF output power in TX mode:
 * '00' : -18dBm
 * '01' : -12dBm
 * '10' :  -6dBm
 * '11' :   0dBm */
#define NRF24_R_STATUS				0x07	// Status Register (shifted out during SPI CMD)
#define NRF24_R_STATUS_RX_DR			0x40	// Data Ready RX FIFO IRQ. Write 1 to clear
#define NRF24_R_STATUS_TX_DS			0x20	// Data Sent TX FIFO IRQ. Write 1 to clear
#define NRF24_R_STATUS_MAX_RT			0x10	// Max num of TX retransmit IRQ. Write 1 to clr
#define NRF24_R_STATUS_RX_P_NO_MASK		0x0E	// Data Pipe number for payload available for
												// read: 000-101: Data Pipe num, 111: RX empty
#define NRF24_R_STATUS_TX_FULL			0x01	// TX FIFO Full flag
#define NRF24_R_OBSERVE_TX			0x08	// Transmit observe register (Default: 0x00)
#define NRF24_R_OBSERVE_TX_PLOS_CNT		0xF0	// Count lost packets, 0 to 15
#define NRF24_R_OBSERVE_TX_ARC_CNT		0x0F	// Count retransmitted packets
#define NRF24_R_RPD					0x09	// Receive Power Detector (Bit 0 only)

/* Following Registers are the Receive Address Data Pipes. LSByte is written
 * first. Write the number of bytes defined by SETUP_AW.
 * Receive Addr Pipe 2 to 5 are special, only LSByte can be written there.
 * The remaining MSBytes are equal to Receive Address Pipe 1! */
#define NRF24_R_RX_ADDR_P0			0x0A	// Receive Address Data Pipe 0, D: 0xE7E7E7E7E7
#define NRF24_R_RX_ADDR_P1			0x0B	// Receive Address Data Pipe 1, D: 0xC2C2C2C2C2
#define NRF24_R_RX_ADDR_P2			0x0C	// Receive Address Data Pipe 2, Default: 0xC3
#define NRF24_R_RX_ADDR_P3			0x0D	// Receive Address Data Pipe 3, Default: 0xC4
#define NRF24_R_RX_ADDR_P4			0x0E	// Receive Address Data Pipe 4, Default: 0xC5
#define NRF24_R_RX_ADDR_P5			0x0F	// Receive Address Data Pipe 5, Default: 0xC6
#define NRF24_R_TX_ADDR				0x10	// Transmit Address, LSByte first, D:E7E7E7E7E7
/* Set RX_ADDR_P0 equal to this address to handle auto-ack for enhanced shockburst        */
#define NRF24_R_RX_PW_0				0x11	// Number of Bytes in RX Payload in Data Pipe 0
#define NRF24_R_RX_PW_1				0x12	// Number of Bytes in RX Payload in Data Pipe 1
#define NRF24_R_RX_PW_2				0x13	// Number of Bytes in RX Payload in Data Pipe 2
#define NRF24_R_RX_PW_3				0x14	// Number of Bytes in RX Payload in Data Pipe 3
#define NRF24_R_RX_PW_4				0x15	// Number of Bytes in RX Payload in Data Pipe 4
#define NRF24_R_RX_PW_5				0x16	// Number of Bytes in RX Payload in Data Pipe 5
#define NRF24_R_FIFO_STATUS			0x17	// FIFO STATUS Register
#define NRF24_R_FIFO_STATUS_TX_REUSE	0x40	// Reuse last TX payload (in PTX device)
#define NRF24_R_FIFO_STATUS_TX_FULL		0x20	// TX FIFO Full flag
#define NRF24_R_FIFO_STATUS_TX_EMPTY	0x10	// TX FIFO empty flag
#define NRF24_R_FIFO_STATUS_RX_FULL		0x02	// RX FIFO Full flag
#define NRF24_R_FIFO_STATUS_RX_EMPTY	0x01	// RX FIFO empty flag
#define NRF24_R_DYNPD				0x1C	// Enable dynamic payload length (Default: 0x00)
#define NRF24_R_DYNPD_DPL_P5			0x20	// Enable dynamic payload length data pipe 5
#define NRF24_R_DYNPD_DPL_P4			0x10	// Enable dynamic payload length data pipe 4
#define NRF24_R_DYNPD_DPL_P3			0x08	// Enable dynamic payload length data pipe 3
#define NRF24_R_DYNPD_DPL_P2			0x04	// Enable dynamic payload length data pipe 2
#define NRF24_R_DYNPD_DPL_P1			0x02	// Enable dynamic payload length data pipe 1
#define NRF24_R_DYNPD_DPL_P0			0x01	// Enable dynamic payload length data pipe 0
#define NRF24_R_FEATURE				0x1D	// Feature Register (Default: 0x00)
#define NRF24_R_FEATURE_EN_DPL			0x04	// Enables Dynamic Payload Length
#define NRF24_R_FEATURE_EN_ACK_PAY		0x02	// Enables Payload with ACK
#define NRF24_R_FEATURE_EN_DYN_ACK		0x01	// Enables the W_TX_PAYLOAD_NOACK command
/* -------- END Registers -------- */
 
typedef enum  {
	NRF24_IOERROR			= 0x80,
	NRF24_TX_FIFO_FULL		= NRF24_R_STATUS_TX_FULL,
	NRF24_RX_FIFO_EMPTY		= NRF24_R_STATUS_RX_P_NO_MASK,
	NRF24_IRQ_MAX_RETR		= NRF24_R_STATUS_MAX_RT,
	NRF24_IRQ_DATATX_SENT	= NRF24_R_STATUS_TX_DS,
	NRF24_IRQ_DATARX_RECV	= NRF24_R_STATUS_RX_DR,
	NRF24_NO_STATUS 		= 0
}nrf24_status_e;

typedef enum {
	NRF24_POWER_LOW 	= 0,
	NRF24_POWER_MID 	= NRF24_R_RF_SETUP_RF_PWR_0,
	NRF24_POWER_HIGH 	= NRF24_R_RF_SETUP_RF_PWR_1,
	NRF24_POWER_MAX 	= NRF24_R_RF_SETUP_RF_PWR_1 | NRF24_R_RF_SETUP_RF_PWR_0
}nrf24_power_e;

typedef enum  {
	NRF24_DR_250KBIT	= NRF24_R_RF_SETUP_RF_DR_LOW,
	NRF24_DR_1MBIT		= 0,
	NRF24_DR_2MBIT		= NRF24_R_RF_SETUP_RF_DR_LOW
}nrf24_datarate_e;

typedef enum {
	NRF24_RXPIPE_0	= 0x01,
	NRF24_RXPIPE_1	= 0x02,
	NRF24_RXPIPE_2	= 0x04,
	NRF24_RXPIPE_3	= 0x08,
	NRF24_RXPIPE_4	= 0x10,
	NRF24_RXPIPE_5	= 0x20,
	NRF24_RXPIPE_ALL= 0x2F
}nrf24_RxPipe_e;

typedef enum {
	NRF24_CRC_OFF		= 0,
	NRF24_CRC_1B 		= NRF24_R_CONFIG_EN_CRC,
	NRF24_CRC_2B 		= NRF24_R_CONFIG_EN_CRC | NRF24_R_CONFIG_CRCO,
	NRF24_CRC_MASK 		= NRF24_R_CONFIG_EN_CRC | NRF24_R_CONFIG_CRCO
}nrf24_crc_e;

typedef enum {
	NRF24_ADDR_3BYTES	= NRF24_R_SETUP_AW_3BYTES,
	NRF24_ADDR_4BYTES	= NRF24_R_SETUP_AW_4BYTES,
	NRF24_ADDR_5BYTES	= NRF24_R_SETUP_AW_5BYTES
}nrf24_addrWidth_e;

typedef enum {
	NRF24_WRITE_WITHACK	= NRF24_C_W_TX_PAYLOAD,
	NRF24_WRITE_NOACK	= NRF24_C_W_TX_PAYLOAD_NACK,
	NRF24_ACK_PAYLOAD	= NRF24_C_W_ACK_PAYLOAD
}nrf24_writeType_e;

typedef enum {
	NRF24_PLSIZE_DYNAMIC	= -1,
	NRF24_PLSIZE_1			= 1,
	NRF24_PLSIZE_2,
	NRF24_PLSIZE_3,
	NRF24_PLSIZE_4,
	NRF24_PLSIZE_5,
	NRF24_PLSIZE_6,
	NRF24_PLSIZE_7,
	NRF24_PLSIZE_8,
	NRF24_PLSIZE_9,
	NRF24_PLSIZE_10,
	NRF24_PLSIZE_11,
	NRF24_PLSIZE_12,
	NRF24_PLSIZE_13,
	NRF24_PLSIZE_14,
	NRF24_PLSIZE_15,
	NRF24_PLSIZE_16,
	NRF24_PLSIZE_17,
	NRF24_PLSIZE_18,
	NRF24_PLSIZE_19,
	NRF24_PLSIZE_20,
	NRF24_PLSIZE_21,
	NRF24_PLSIZE_22,
	NRF24_PLSIZE_23,
	NRF24_PLSIZE_24,
	NRF24_PLSIZE_25,
	NRF24_PLSIZE_26,
	NRF24_PLSIZE_27,
	NRF24_PLSIZE_28,
	NRF24_PLSIZE_29,
	NRF24_PLSIZE_30,
	NRF24_PLSIZE_31,
	NRF24_PLSIZE_32,
	NRF24_PLSIZE_INVALID
}nrf24_payloadSize_e;

typedef struct {
	nrf24_status_e status;
	nrf24_addrWidth_e addrWidth:2;
	uint8_t PTXdynamicPayload:1;
	nrf24_RxPipe_e PRXdynamicPayload:6;
	void *ioInt;
	uint8_t (*startTrans)(void*);
	uint8_t (*transBytes)(void*,uint8_t,uint8_t*,uint16_t);
	uint8_t (*endTrans)(void*);
	void (*highActiveCE)(void*);
	void (*lowInactiveCE)(void*);
	void (*delay10us)(void);
}nrf24_t;

typedef struct {
	uint8_t payload[32];
	nrf24_payloadSize_e length;
	union{
		nrf24_RxPipe_e 		rxPipe;
		nrf24_writeType_e	writeType;
	};
}nrf24_message_t;

void nrf24_setupStruct(nrf24_t *nrf, void *ioInt, uint8_t (*startTrans)(void*),
		uint8_t (*transBytes)(void*,uint8_t,uint8_t*,uint16_t), uint8_t (*endTrans)(void*),
		void (*highActiveCE)(void*), void (*lowInactiveCE)(void*), void (*delay10us)(void));
uint8_t nrf24_init(nrf24_t *nrf);

/* BASIC I/O FUNCTIONS */
void nrf24_writeRegs(nrf24_t* nrf24, uint8_t reg, uint8_t *buf, uint8_t len);
void nrf24_writeReg(nrf24_t* nrf, uint8_t reg, uint8_t regVal);
void nrf24_writeCommand(nrf24_t *nrf, uint8_t cmd);
uint8_t nrf24_readReg(nrf24_t *nrf, uint8_t reg);
void nrf24_readRegs(nrf24_t *nrf, uint8_t reg, uint8_t *buf, uint8_t len);

void nrf24_sendMessage(nrf24_t *nrf, uint8_t *txAddress, nrf24_message_t *message);
uint8_t nrf24_recvMessage(nrf24_t *nrf, nrf24_message_t *message);
void nrf24_startListening(nrf24_t *nrf);
void nrf24_stopListening(nrf24_t *nrf);
nrf24_status_e nrf24_checkIRQ(nrf24_t *nrf);

void nrf24_setTxAddress(nrf24_t *nrf, uint8_t *txAddress);
void nrf24_setRxAddress(nrf24_t *nrf, uint8_t *rxAddress, nrf24_RxPipe_e pipe);
void nrf24_writePayload(nrf24_t *nrf, uint8_t *dataBuffer, uint8_t bufLen, nrf24_writeType_e type);
void nrf24_readPayload(nrf24_t *nrf, uint8_t *dataBuffer, uint8_t len);
nrf24_status_e nrf24_getStatus(nrf24_t *nrf);
void nrf24_powerUp(nrf24_t *nrf);
void nrf24_powerDown(nrf24_t *nrf);
uint8_t nrf24_isConnected(nrf24_t *nrf);
void nrf24_setAddressWidth(nrf24_t *nrf, nrf24_addrWidth_e addrWidth);
void nrf24_setRetries(nrf24_t *nrf, uint8_t delays, uint8_t retries);
void nrf24_setChannel(nrf24_t *nrf, uint8_t channel);
uint8_t nrf24_getChannel(nrf24_t *nrf);
void nrf24_setRxPayloadSize(nrf24_t *nrf, nrf24_RxPipe_e pipes, nrf24_payloadSize_e plSize);
uint8_t nrf24_getRxPayloadSize(nrf24_t *nrf, nrf24_RxPipe_e pipe);
uint8_t nrf24_getRxDynPayloadSize(nrf24_t* nrf);
void nrf24_setTxDynamicPayload(nrf24_t *nrf, uint8_t onBit);
void nrf24_enableAutoAck(nrf24_t *nrf);
void nrf24_disableAutoAck(nrf24_t *nrf);
void nrf24_setAutoAckPipe(nrf24_t *nrf, nrf24_RxPipe_e pipes);
void nrf24_setEnRxPipe(nrf24_t *nrf, nrf24_RxPipe_e pipes);
void nrf24_setRFPower(nrf24_t *nrf, nrf24_power_e pwrLevel);
nrf24_power_e nrf24_getRFPower(nrf24_t *nrf);
void nrf24_getTXCounters(nrf24_t *nrf, uint8_t *lostPacketsCnt, uint8_t *retransmittedCnt);
void nrf24_setDataRate(nrf24_t *nrf, nrf24_datarate_e dataRate);
nrf24_addrWidth_e nrf24_getDataRate(nrf24_t *nrf);
void nrf24_setCRCType(nrf24_t *nrf, nrf24_crc_e crcType);
nrf24_crc_e nrf24_getCRCType(nrf24_t *nrf);
void nrf24_flushRx(nrf24_t *nrf);
void nrf24_flushTx(nrf24_t *nrf);

#ifdef __cplusplus
}
#endif

#endif /* _NRF24_H_ */