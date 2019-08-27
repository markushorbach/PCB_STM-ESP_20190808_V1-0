/*		NXP MFRC522 SPI Mifare driver
 *		for STM32 CubeMX projects
 *      Author: Markus Horbach
 *      Version 1.0
 *      Date : 26.08.2019
 */

#include "main.h"

#define MFRC522_SPI						&hspi1
#define	MFRC522_RST_Pin					RFID_RST_Pin
#define	MFRC522_RST_Port				RFID_RST_GPIO_Port
#define	MFRC522_IRQ_Pin					RFID_IRQ_Pin
#define	MFRC522_IRQ_Port				RFID_IRQ_GPIO_Port

#define MFRC522_DUMMY					(uint8_t)0x00

/* MFRC522 Commands */
#define MFRC522_CMD_IDLE				(uint8_t)0x00	// no action, cancel the current command
#define MFRC522_CMD_MEM					(uint8_t)0x01	// store 25 Byte in internal buffer
#define MFRC522_CMD_GenRandID			(uint8_t)0x02	// generate random 10 Byte ID
#define MFRC522_CMD_CALCCRC				(uint8_t)0x03	// CRC Calculate
#define MFRC522_CMD_TRANSMIT			(uint8_t)0x04	// Transmit data from FIFO buffer
//#define MFRC522_CMD_tbd_05				(uint8_t)0x05	// tbd
//#define MFRC522_CMD_tbd_06				(uint8_t)0x06	// tbd
#define MFRC522_CMD_NoCmdChange			(uint8_t)0x07	// no command change, used to modify Command register bits
#define MFRC522_CMD_RECEIVE				(uint8_t)0x08	// activate receiver circuits
//#define MFRC522_CMD_tbd_09				(uint8_t)0x09	// tbd
//#define MFRC522_CMD_tbd_0A				(uint8_t)0x0A	// tbd
//#define MFRC522_CMD_tbd_0B				(uint8_t)0x0B	// tbd
#define MFRC522_CMD_TRANSCEIVE			(uint8_t)0x0C	// transceive data from FIFO buffer to antenna and activate the receiver
//#define MFRC522_CMD_tbd_0D				(uint8_t)0x0D	// tbd
#define MFRC522_CMD_AUTHENT				(uint8_t)0x0E	// perform the mifare standard authentication as a reader
#define MFRC522_CMD_RESETPHASE			(uint8_t)0x0F	// reset the MFRC522

/* MFRC522 Registers */
//Page 0: Command and Status
#define MFRC522_REG_RESERVED00			(uint8_t)0x00
#define MFRC522_REG_COMMAND				(uint8_t)0x01
#define MFRC522_REG_COMM_I_EN			(uint8_t)0x02
#define MFRC522_REG_DIV_I_EN				(uint8_t)0x03
#define MFRC522_REG_COMM_IRQ			(uint8_t)0x04
#define MFRC522_REG_DIV_IRQ				(uint8_t)0x05
#define MFRC522_REG_ERROR				(uint8_t)0x06	// read only
#define MFRC522_REG_STATUS1				(uint8_t)0x07	// read only
#define MFRC522_REG_STATUS2				(uint8_t)0x08
#define MFRC522_REG_FIFO_DATA			(uint8_t)0x09
#define MFRC522_REG_FIFO_LEVEL			(uint8_t)0x0A
#define MFRC522_REG_WATER_LEVEL			(uint8_t)0x0B
#define MFRC522_REG_CONTROL				(uint8_t)0x0C
#define MFRC522_REG_BIT_FRAMING			(uint8_t)0x0D
#define MFRC522_REG_COLL				(uint8_t)0x0E
//#define MFRC522_REG_RESERVED0F			(uint8_t)0x0F
//Page 1: Command
//#define MFRC522_REG_RESERVED10			(uint8_t)0x10
#define MFRC522_REG_MODE				(uint8_t)0x11	// init CMD 07
#define MFRC522_REG_TX_MODE				(uint8_t)0x12
#define MFRC522_REG_RX_MODE				(uint8_t)0x13
#define MFRC522_REG_TX_CONTROL			(uint8_t)0x14	// init CMD 08
#define MFRC522_REG_TX_AUTO				(uint8_t)0x15	// init CMD 06
#define MFRC522_REG_TX_SEL				(uint8_t)0x16
#define MFRC522_REG_RX_SEL				(uint8_t)0x17
#define MFRC522_REG_RX_THRESHOLD		(uint8_t)0x18
#define MFRC522_REG_DEMOD				(uint8_t)0x19
//#define MFRC522_REG_RESERVED1A			(uint8_t)0x1A
//#define MFRC522_REG_RESERVED1B			(uint8_t)0x1B
#define MFRC522_REG_MIFARE_Tx			(uint8_t)0x1C
#define MFRC522_REG_MIFARE_Rx			(uint8_t)0x1D
//#define MFRC522_REG_RESERVED1E			(uint8_t)0x1E
#define MFRC522_REG_SERIALSPEED			(uint8_t)0x1F
//Page 2: CFG
//#define MFRC522_REG_RESERVED20			(uint8_t)0x20
#define MFRC522_REG_CRC_RESULT_H		(uint8_t)0x21
#define MFRC522_REG_CRC_RESULT_L		(uint8_t)0x22
//#define MFRC522_REG_RESERVED23			(uint8_t)0x23
#define MFRC522_REG_MOD_WIDTH			(uint8_t)0x24
//#define MFRC522_REG_RESERVED25			(uint8_t)0x25
#define MFRC522_REG_RF_CFG				(uint8_t)0x26	// init CMD 05
#define MFRC522_REG_GS_N				(uint8_t)0x27
#define MFRC522_REG_CWGS_PREG			(uint8_t)0x28
#define MFRC522_REG__MODGS_PREG			(uint8_t)0x29
#define MFRC522_REG_T_MODE				(uint8_t)0x2A	// init CMD 01
#define MFRC522_REG_T_PRESCALER			(uint8_t)0x2B	// init CMD 02
#define MFRC522_REG_T_RELOAD_H			(uint8_t)0x2C	// init CMD 04
#define MFRC522_REG_T_RELOAD_L			(uint8_t)0x2D	// init CMD 03
#define MFRC522_REG_T_COUNTER_VALUE_H	(uint8_t)0x2E
#define MFRC522_REG_T_COUNTER_VALUE_L	(uint8_t)0x2F
//Page 3:TestRegister
//#define MFRC522_REG_RESERVED30			(uint8_t)0x30
#define MFRC522_REG_TEST_SEL1			(uint8_t)0x31
#define MFRC522_REG_TEST_SEL2			(uint8_t)0x32
#define MFRC522_REG_TEST_PIN_EN			(uint8_t)0x33
#define MFRC522_REG_TEST_PIN_VALUE		(uint8_t)0x34
#define MFRC522_REG_TEST_BUS			(uint8_t)0x35
#define MFRC522_REG_AUTO_TEST			(uint8_t)0x36
#define MFRC522_REG_VERSION				(uint8_t)0x37
#define MFRC522_REG_ANALOG_TEST			(uint8_t)0x38
#define MFRC522_REG_TEST_DAC1			(uint8_t)0x39
#define MFRC522_REG_TEST_DAC2			(uint8_t)0x3A
#define MFRC522_REG_TEST_ADC0			(uint8_t)0x3B
//#define MFRC522_REG_RESERVED3C			(uint8_t)0x3C
//#define MFRC522_REG_RESERVED3D			(uint8_t)0x3D
//#define MFRC522_REG_RESERVED3E			(uint8_t)0x3E
//#define MFRC522_REG_RESERVED3F			(uint8_t)0x3F

/* Mifare_One card command word */
#define PICC_REQIDL						(uint8_t)0x26	// find the antenna area does not enter hibernation
#define PICC_REQALL						(uint8_t)0x52	// find all the cards antenna area
#define PICC_ANTICOLL					(uint8_t)0x93	// anti-collision
#define PICC_SElECTTAG					(uint8_t)0x93	// election card
#define PICC_AUTHENT1A					(uint8_t)0x60	// authentication key A
#define PICC_AUTHENT1B					(uint8_t)0x61	// authentication key B
#define PICC_READ						(uint8_t)0x30	// read Block
#define PICC_WRITE						(uint8_t)0xA0	// write block
#define PICC_DECREMENT					(uint8_t)0xC0	// debit
#define PICC_INCREMENT					(uint8_t)0xC1	// recharge
#define PICC_RESTORE					(uint8_t)0xC2	// transfer block data to the buffer
#define PICC_TRANSFER					(uint8_t)0xB0	// save the data in the buffer
#define PICC_HALT						(uint8_t)0x50	// sleep

// Hard Reset
void MFRC522_HardReset(void);
// Soft Reset
void MFRC522_SoftReset(void);
// Initialisierung nach Hard Reset
void MFRC522_Init(void);
// einzelnen Befehl senden
void MFRC522_SendCMD(uint8_t cmd);
// einzelnes Register lesen
uint8_t MFRC522_Read_Register(uint8_t addr);
// einzelenes Register schreiben
void MFRC522_Write_Register(uint8_t addr, uint8_t val);
// Chip version
uint8_t MFRC522_ChipVersion(void);
// digitaler Selbsttest
void MFRC522_SelfTest(void);


