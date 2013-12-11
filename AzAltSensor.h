/*

Arduino library header for interfacing with the Azimuth/Inclination sensor

See README.txt and LICENSE.txt

*/

#ifndef __AZINCSENSOR_H__
#define __AZINCSENSOR_H__


#include "Arduino.h"
#include <Spi.h>


/* sensor board I2C address */
#define BOARD_ADDRESS		0x20


/* pin definitions */
#define PIN_SENSE_RST		10
#define PIN_I2C_SDA		A4
#define PIN_I2C_SCL		A5


/* timing constants */
#define T_RESET			20
#define T_WD_FLASH		5
#define T_WD_EEPROM		4
#define T_WD_ERASE		9
#define T_WD_FUSE		5
#define T_I2CREAD		1


/* programmer state */
#define SENSE_STATE_IDLE	0
#define SENSE_STATE_PROG	1


class AzIncSensor
{
	public:

	AzIncSensor();

	void init();
	void reset();


	/* ----- sampling related ----- */

	void sampleStart();
	void sampleEnd();

	unsigned int sample();

	byte sampleBuffer[32];


	/* ----- programming commands ----- */

	boolean pmodeStart();
	void pmodeEnd();

	void erase();

	void writeProgPages(word *buf, word pageAddress, unsigned int numWords);
	void readProgWords(word *buf, word wordAddress, unsigned int numWords);

	void writeEepromPages(byte *buf, word pageAddress, unsigned int numBytes);
	void readEepromBytes(byte *buf, word byteAddress, unsigned int numBytes);

	word readProgWord(word address);
	void loadProgWord(word address, word memWord);
	void writeProgPage(word address);

	byte readEepromByte(word address);
	void writeEepromByte(word address, byte memByte);
	void loadEepromByte(word address, byte memByte);
	void writeEepromPage(word address);

	byte readLockBits();
	void writeLockBits(byte lockBits);

	long int readSignature();

	void writeFuseBits(byte lowFuseBits, byte highFuseBits, byte extFuseBits);
	long int readFuseBits();

	word readCalWord();

	boolean pollRdyBsy();
	/* -------------------------------- */


	private:

	void doCommand(byte a, byte b, byte c, byte d);

	Spi spi;

	byte buffer[4];
	byte state;
};


#endif /*__AZINCSENSOR_H__*/