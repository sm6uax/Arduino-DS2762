/*
    Arduino DS2762 Library
    Copyright (C) 2013  nigelb, Scott Mills

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#include "DS2762.h"

DS2762::DS2762(OneWire* bus)
{
	this->bus = bus;
	this->memory = NULL;
}

DS2762::~DS2762()
{
	reset();
}

void DS2762::_read_device(uint8_t* address,uint8_t* buf, uint8_t start, uint8_t count)
{
	this->bus->reset();
	this->bus->select(address);
	this->bus->write(DS2762_READ_DATA);
	this->bus->write(start);
	this->bus->read_bytes(buf, count);
}

void DS2762::_write_device(uint8_t* address,uint8_t* buf, uint8_t start, uint8_t count)
{
	this->bus->reset();
	this->bus->select(address);
	this->bus->write(DS2762_WRITE_DATA);
	this->bus->write(start);
	this->bus->write_bytes(buf, count,false);
	reset();
}

void DS2762::readDevice(uint8_t* address)
{
	memory = (uint8_t*)calloc(sizeof(uint8_t), 255);
	_read_device(address,memory, DS2762_PROTECTION_REG, 255);
}

boolean DS2762::_has_buffer()
{
	return this->memory != NULL;
}

int16_t DS2762::_read_int16(uint8_t* address,uint8_t addr_msb, uint8_t addr_lsb, int shift)
{
	uint8_t MSB, LSB;
	if(_has_buffer()){
		MSB = this->memory[addr_msb];
		LSB = this->memory[addr_lsb];
	}
	else
	{
		uint8_t data[2];
		_read_device(address,data, addr_msb, 2);
		MSB = data[0];
		LSB = data[1];
	}
	return (int16_t)(((MSB << 8) | LSB) >> shift);
}

int16_t DS2762::readADCRaw(uint8_t* address)
{
	return _read_int16(address,DS2762_VOLTAGE_MSB, DS2762_VOLTAGE_LSB, 5);
}

double DS2762::readADC(uint8_t* address)
{
	return (double(readADCRaw( address)) * 4880) / 1000000; // 4.88mV per count.
}

int16_t DS2762::readTempRaw(uint8_t* address)
{
	return _read_int16(address,DS2762_TEMP_MSB, DS2762_TEMP_LSB, 5);
}

double DS2762::readTempC(uint8_t* address)
{
	return (double(readTempRaw(address)) * 125) / 1000;
}

double DS2762::readTempF(uint8_t* address)
{
	return (readTempC(address) * 1.8) + 32;
}

int16_t DS2762::readCurrentRaw(uint8_t* address)
{
	return _read_int16(address,DS2762_CURRENT_MSB, DS2762_CURRENT_LSB, 3);
}

double DS2762::readCurrent(uint8_t* address,bool internal_sense_resistor = 0)
{
	uint16_t count = readCurrentRaw(address);

	if (internal_sense_resistor)
	{
		// For the internal sense resistor configuration, the DS2762 maintains
		// the current register in units of amps, with a resolution of 0.625mA
		// and full-scale range of no less than 1.9A. The DS2762 automatically
		// compensates for internal sense resistor process variations
		// and temperature effects when reporting current. 
		return (double(count) * 6.25 ) / 10000; // 0.625mA per count.
	}
	else
	{
		// For the external sense resistor configuration, the DS2762 writes the
		// measured VIS voltage to the current register,
		// with a 15.625 microV resolution and a full-scale 64mV range.
		return (double(count) * 15.625) / 100000;
	}
}

void DS2762::resetProtectionRegister(uint8_t* address)
{
	uint8_t _register;
	if(_has_buffer()){
		_register = this->memory[DS2762_PROTECTION_REG];
	}else
	{
		_read_device(address,&_register, DS2762_PROTECTION_REG, 1);
	}
	_register = _register & (!(DS2762_PR_OV_FLAG | DS2762_PR_UV_FLAG | DS2762_PR_COC_FLAG | DS2762_PR_DOC_FLAG));
	_write_device(address,&_register, DS2762_PROTECTION_REG, 1);
}

void DS2762::reset()
{
	if(memory != NULL)
	{
		free (memory);
		memory = NULL;
	}
}

void DS2762::copyEEPROMBlock(uint8_t* address,DS2762_EEPROM block)
{
	this->bus->reset();
	this->bus->select(address);
	this->bus->write(DS2762_COPY_DATA);
	this->bus->write(block);
	delay(5);
}
void DS2762::recallEEPROMBlock(uint8_t* address,DS2762_EEPROM block)
{
	this->bus->reset();
	this->bus->select(address);
	this->bus->write(DS2762_RECALL_DATA);
	this->bus->write(block);
}



uint8_t DS2762::_read_register_bit(uint8_t* address,uint8_t register_address, uint8_t pin)
{
	uint8_t _register;
	if(_has_buffer()){
		_register = this->memory[register_address];
	}else
	{
		_read_device(address,&_register, register_address, 1);
	}
	return _register & pin ;
}

boolean DS2762::isOverVoltage(uint8_t* address){
	return _read_register_bit(address,DS2762_PROTECTION_REG, DS2762_PR_OV_FLAG) > 0;
}

boolean DS2762::isUnderVoltage(uint8_t* address){
	return _read_register_bit(address,DS2762_PROTECTION_REG, DS2762_PR_UV_FLAG) > 0;
}

boolean DS2762::isChargeOverCurrent(uint8_t* address){
	return _read_register_bit(address,DS2762_PROTECTION_REG, DS2762_PR_COC_FLAG) > 0;
}

boolean DS2762::isDischargeOverCurrent(uint8_t* address){
	return _read_register_bit(address,DS2762_PROTECTION_REG, DS2762_PR_DOC_FLAG) > 0;
}

boolean DS2762::isCCPin(uint8_t* address){
	return _read_register_bit(address,DS2762_PROTECTION_REG, DS2762_PR_CCPIN_FLAG) > 0;
}

boolean DS2762::isDCPin(uint8_t* address){
	return _read_register_bit(address,DS2762_PROTECTION_REG, DS2762_PR_DCPIN_FLAG) > 0;
}

boolean DS2762::isChargeEnable(uint8_t* address){
	return _read_register_bit(address,DS2762_PROTECTION_REG, DS2762_PR_CE_FLAG) > 0;
}

boolean DS2762::isDischargeEnable(uint8_t* address){
	return _read_register_bit(address,DS2762_PROTECTION_REG, DS2762_PR_DE_FLAG) > 0;
}


boolean DS2762::isSleepModeEnabled(uint8_t* address){
	return _read_register_bit(address,DS2762_STATUS_REG, DS2762_STATUS_PMOD) > 0;
}
boolean DS2762::isReadNetAddressOpcode(uint8_t* address){
	return _read_register_bit(address,DS2762_STATUS_REG, DS2762_STATUS_RNAOP) > 0;
}
boolean DS2762::isSWAPEnabled(uint8_t* address){
	return _read_register_bit(address,DS2762_STATUS_REG, DS2762_STATUS_SWEN) > 0;
}
boolean DS2762::isInterruptEnabled(uint8_t* address){
	return _read_register_bit(address,DS2762_STATUS_REG, DS2762_STATUS_IE) > 0;
}

boolean DS2762::isPSPinLatch(uint8_t* address){
	return _read_register_bit(address,DS2762_SPECIAL_FEATURE_REG, DS2762_SF_PS_FLAG) > 0;
}

boolean DS2762::isPIO(uint8_t* address){
	return _read_register_bit(address,DS2762_SPECIAL_FEATURE_REG, DS2762_SF_PIO_FLAG) > 0;
}
boolean DS2762::isSWAPMasterStatusBit(uint8_t* address){
	return _read_register_bit(address,DS2762_SPECIAL_FEATURE_REG, DS2762_SF_MSTR_FLAG) > 0;
}


void DS2762::setSWAPEnabled(uint8_t* address,boolean enabled)
{
	_set_register_bit(address,DS2762_STATUS_REG, DS2762_STATUS_WRITE_REG, DS2762_STATUS_SWEN, BLOCK1, enabled);
}

void DS2762::setSleepMode(uint8_t* address,boolean enabled)
{
	_set_register_bit(address,DS2762_STATUS_REG, DS2762_STATUS_WRITE_REG, DS2762_STATUS_PMOD, BLOCK1, enabled);
}

void DS2762::_set_register_bit(uint8_t* address,uint8_t read_register_address, uint8_t write_register_address, uint8_t bit, DS2762_EEPROM block, boolean enabled)
{
	uint8_t _register;
	if(_has_buffer()){
		_register = this->memory[read_register_address];
	}else
	{
		_read_device(address,&_register, read_register_address, 1);
	}

	if(enabled)
	{
		_register = _register | bit;
	}else
	{
		_register = _register & (!bit);
	}

	this->bus->reset();
	this->bus->select(address);
	this->bus->write(DS2762_WRITE_DATA);
	this->bus->write(write_register_address);
	this->bus->write(_register);

	copyEEPROMBlock(address,BLOCK1);
	recallEEPROMBlock(address,BLOCK1);


}

uint32_t  DS2762::writeEEPROM(uint8_t* address,byte* buf, uint32_t length, uint32_t eeprom_offset)
{
	if((DS2762_EEPROM_BLOCK0 + length + eeprom_offset) > (0x2F + 1))
	{
		return 0;
	}
	_write_device(address,buf, DS2762_EEPROM_BLOCK0 + eeprom_offset, length);
	copyEEPROMBlock(address,BLOCK0);
	recallEEPROMBlock(address,BLOCK0);
	return length;
}

uint32_t  DS2762::readEEPROM(uint8_t* address,byte* buf, uint32_t length, uint32_t eeprom_offset)
{
	Serial.println(DS2762_EEPROM_BLOCK0 + length + eeprom_offset, HEX);
	if((DS2762_EEPROM_BLOCK0 + length + eeprom_offset) > (0x2F + 1))
	{
		return 0;
	}
	recallEEPROMBlock(address,BLOCK0);
	_read_device(address,buf, DS2762_EEPROM_BLOCK0 + eeprom_offset, length);
	return length;
}


