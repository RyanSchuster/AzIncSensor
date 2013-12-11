/*

Arduino library for interfacing with the Azimuth/Inclination sensor

See README.txt and LICENSE.txt

*/

#include "AzIncSensor.h"

#include <Wire.h>


#define EEPROM_BYTES_PER_PAGE	4
#define EEPROM_FULL_MASK	0x007F
#define EEPROM_BYTE_MASK	0x0003
#define EEPROM_PAGE_MASK	0x007C

#define FLASH_WORDS_PER_PAGE	16
#define FLASH_FULL_MASK		0x03FF
#define FLASH_WORD_MASK		0x000F
#define FLASH_PAGE_MASK		0x03F0


AzIncSensor::AzIncSensor()
{
}


void AzIncSensor::init()
{
	/* set up spi for our board */
	spi.off();
	spi.setMode(0x53);

	/* make sure i2c is off */
	sampleEnd();

	/* reset the microcontroller */
	pinMode(PIN_SENSE_RST, OUTPUT);
	reset();
}


void AzIncSensor::reset()
{
	digitalWrite(PIN_SENSE_RST, LOW);
	delay(T_RESET);

	digitalWrite(PIN_SENSE_RST, HIGH);
	delay(T_RESET);
}


boolean AzIncSensor::pmodeStart()
{
	byte test;

	/* turn on spi */
	spi.on();

	/* activate the device */
	digitalWrite(PIN_SENSE_RST, LOW);
	delay(T_RESET);

	/* send the program enable command */
	doCommand(0xAC, 0x53, 0x00, 0x00);

	/* test for success */
	if (buffer[2] == 0x53)
	{
		state = SENSE_STATE_PROG;
		return true;
	}

	pmodeEnd();
	return false;
}


void AzIncSensor::pmodeEnd()
{
	/* release the device */
	digitalWrite(PIN_SENSE_RST, HIGH);
	delay(T_RESET);	/* probably don't need this */

	/* turn spi off */
	spi.off();

	state = SENSE_STATE_IDLE;
}


void AzIncSensor::sampleStart()
{
	Wire.begin();
}


void AzIncSensor::sampleEnd()
{
	pinMode(PIN_I2C_SDA, INPUT);
	pinMode(PIN_I2C_SCL, INPUT);
}


unsigned int AzIncSensor::sample()
{
	unsigned int i;

	/* send command */
	Wire.beginTransmission(BOARD_ADDRESS);
	Wire.write("A");
	Wire.endTransmission();

	/* give the board some time to process */
	delay(T_I2CREAD);

	/* read bytes back */
	Wire.requestFrom(BOARD_ADDRESS, 6);
	i = 0;
	while(Wire.available())
	{
		sampleBuffer[i] = Wire.read();
		i++;
	}

	return i;
}


void AzIncSensor::doCommand(byte a, byte b, byte c, byte d)
{
	buffer[0] = spi.transfer(a);
	buffer[1] = spi.transfer(b);
	buffer[2] = spi.transfer(c);
	buffer[3] = spi.transfer(d);
}


void AzIncSensor::erase()
{
	doCommand(0xAC, 0x80, 0x00, 0x00);
	delay(T_WD_ERASE);
}


void AzIncSensor::writeProgPages(word *buf, word pageAddress, unsigned int numWords)
{
	unsigned int numPages = (numWords / FLASH_WORDS_PER_PAGE) + ((numWords % FLASH_WORDS_PER_PAGE) ? 1 : 0);

	unsigned int thisPage;
	unsigned int thisWord;

	unsigned int wordIdx = 0;

	for (thisPage = 0; thisPage < numPages; thisPage++)
	{
		for (thisWord = 0; thisWord < FLASH_WORDS_PER_PAGE; thisWord++)
		{
			loadProgWord(thisWord, buf[wordIdx++]);
			if (wordIdx >= numWords)
				break;
		}
		writeProgPage(pageAddress);
		pageAddress += FLASH_WORDS_PER_PAGE;
	}
}


void AzIncSensor::writeEepromPages(byte *buf, word pageAddress, unsigned int numBytes)
{
	unsigned int numPages = (numBytes / EEPROM_BYTES_PER_PAGE) + ((numBytes % EEPROM_BYTES_PER_PAGE) ? 1 : 0);

	unsigned int thisPage;
	unsigned int thisByte;

	unsigned int byteIdx = 0;

	for (thisPage = 0; thisPage < numPages; thisPage++)
	{
		for (thisByte = 0; thisByte < EEPROM_BYTES_PER_PAGE; thisByte++)
		{
			loadEepromByte(thisByte, buf[byteIdx++]);
			if (byteIdx >= numBytes)
				break;
		}
		writeEepromPage(pageAddress);
		pageAddress += EEPROM_BYTES_PER_PAGE;
	}
}


void AzIncSensor::readProgWords(word *buf, word wordAddress, unsigned int numWords)
{
	for (; numWords > 0; numWords--)
		buf[numWords - 1] = readProgWord(wordAddress + numWords - 1);
}


void AzIncSensor::readEepromBytes(byte *buf, word byteAddress, unsigned int numBytes)
{
	for (; numBytes > 0; numBytes--)
		buf[numBytes - 1] = readEepromByte(byteAddress + numBytes - 1);
}


word AzIncSensor::readProgWord(word address)
{
	word memWord = 0;

	address &= FLASH_FULL_MASK;

	doCommand(0x20, highByte(address), lowByte(address), 0x00);
	memWord |= buffer[3];
	doCommand(0x28, highByte(address), lowByte(address), 0x00);
	memWord |= buffer[3] << 8;

	return memWord;
}

void AzIncSensor::loadProgWord(word address, word memWord)
{
	address &= FLASH_WORD_MASK;

	doCommand(0x40, highByte(address), lowByte(address), lowByte(memWord));
	doCommand(0x48, highByte(address), lowByte(address), highByte(memWord));
}


void AzIncSensor::writeProgPage(word address)
{
	address &= FLASH_PAGE_MASK;

	doCommand(0x4C, highByte(address), lowByte(address), 0x00);
	delay(T_WD_FLASH);
}


byte AzIncSensor::readEepromByte(word address)
{
	address &= EEPROM_FULL_MASK;

	doCommand(0xA0, highByte(address), lowByte(address), 0x00);

	return buffer[3];
}


void AzIncSensor::writeEepromByte(word address, byte memByte)
{
	address &= EEPROM_FULL_MASK;

	doCommand(0xC0, highByte(address), lowByte(address), memByte);
	delay(T_WD_EEPROM);
}


void AzIncSensor::loadEepromByte(word address, byte memByte)
{
	address &= EEPROM_BYTE_MASK;

	doCommand(0xC1, highByte(address), lowByte(address), memByte);
}


void AzIncSensor::writeEepromPage(word address)
{
	address &= EEPROM_PAGE_MASK;

	doCommand(0xC2, highByte(address), lowByte(address), 0x00);
	delay(T_WD_EEPROM);
}


byte AzIncSensor::readLockBits()
{
	doCommand(0x58, 0x00, 0x00, 0x00);

	return buffer[3];
}


void AzIncSensor::writeLockBits(byte lockBits)
{
	doCommand(0xA3, 0xE0, 0x00, lockBits);
}


long int AzIncSensor::readSignature()
{
	long int sig = 0;

	doCommand(0x30, 0x00, 0x00, 0x00);
	sig |= buffer[3];
	doCommand(0x30, 0x00, 0x01, 0x00);
	sig |= buffer[3] << 8;
	doCommand(0x30, 0x00, 0x02, 0x00);
	sig |= buffer[3] << 16;
	doCommand(0x30, 0x00, 0x03, 0x00);
	sig |= buffer[3] << 24;

	return sig;
}


void AzIncSensor::writeFuseBits(byte lowFuseBits, byte highFuseBits, byte extFuseBits)
{
	doCommand(0xAC, 0xA0, 0x00, lowFuseBits);
	delay(T_WD_FUSE);
	doCommand(0xAC, 0xA8, 0x00, highFuseBits);
	delay(T_WD_FUSE);
	doCommand(0xAC, 0xA4, 0x00, extFuseBits);
	delay(T_WD_FUSE);
}


long int AzIncSensor::readFuseBits()
{
	long int fuseBits = 0;

	doCommand(0x50, 0x00, 0x00, 0x00);
	fuseBits |= buffer[3];
	doCommand(0x58, 0x08, 0x00, 0x00);
	fuseBits |= buffer[3] << 8;
	doCommand(0x50, 0x08, 0x00, 0x00);
	fuseBits |= buffer[3] << 16;

	return fuseBits;
}


word AzIncSensor::readCalWord()
{
	word calWord = 0;

	doCommand(0x38, 0x00, 0x00, 0x00);
	calWord |= buffer[3];
	doCommand(0x38, 0x00, 0x01, 0x00);
	calWord |= buffer[3] << 8;

	return calWord;
}


boolean AzIncSensor::pollRdyBsy()
{
	doCommand(0xF0, 0x00, 0x00, 0x00);

	return buffer[3] ? true : false;
}