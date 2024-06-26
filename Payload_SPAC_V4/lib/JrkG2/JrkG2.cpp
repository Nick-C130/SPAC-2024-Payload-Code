#include "JrkG2.h"
#include <Arduino.h>
/**** JrkG2Serial ****/

void JrkG2Serial::commandW7(uint8_t cmd, uint8_t val)
{
  sendCommandHeader(cmd);
  serialW7(val);

  _lastError = 0;
}

void JrkG2Serial::commandWs14(uint8_t cmd, int16_t val)
{
  sendCommandHeader(cmd);
  serialW7(val);  // lower 7 bits
  serialW7(val >> 7);  // upper 7 bits

  _lastError = 0;
}

uint8_t JrkG2Serial::commandR8(uint8_t cmd)
{
  uint8_t val;

  sendCommandHeader(cmd);

  uint8_t byteCount = _stream->readBytes(&val, 1);
  if (byteCount != 1)
  {
    _lastError = JrkG2CommReadError;
    return 0;
  }

  _lastError = 0;
  return val;
}

uint16_t JrkG2Serial::commandR16(uint8_t cmd)
{
  uint8_t buffer[2];

  sendCommandHeader(cmd);

  uint8_t byteCount = _stream->readBytes(buffer, 2);
  if (byteCount != 2)
  {
    _lastError = JrkG2CommReadError;
    return 0;
  }

  _lastError = 0;
  return ((uint16_t)buffer[0] << 0) | ((uint16_t)buffer[1] << 8);
}

void JrkG2Serial::segmentRead(uint8_t cmd, uint8_t offset,
  uint8_t length, uint8_t * buffer)
{
  // The Jrk does not allow reads longer than 15 bytes.
  if (length > 15) { length = 15; }

  sendCommandHeader(cmd);
  serialW7(offset);
  serialW7(length);

  uint8_t byteCount = _stream->readBytes(buffer, length);
  if (byteCount != length)
  {
    _lastError = JrkG2CommReadError;

    // Set the buffer bytes to 0 so the program will not use an uninitialized
    // variable.
    memset(buffer, 0, length);
    return;
  }

  _lastError = 0;
}

void JrkG2Serial::segmentWrite(uint8_t cmd, uint8_t offset,
  uint8_t length, uint8_t * buffer)
{
  // The Jrk does not accept writes longer than 7 bytes over serial.
  if (length > 7) { length = 7; }

  sendCommandHeader(cmd);
  serialW7(offset);
  serialW7(length);

  // bit i = most-significant bit of buffer[i]
  uint8_t msbs = 0;
  for (uint8_t i = 0; i < length; i++)
  {
    serialW7(buffer[i]);
    msbs |= (buffer[i] >> 7 & 1) << i;
  }
  serialW7(msbs);

  _lastError = 0;
}

void JrkG2Serial::sendCommandHeader(uint8_t cmd)
{
  if (_deviceNumber == 255)
  {
    // Compact protocol
    _stream->write((uint8_t)cmd);
  }
  else
  {
    // Pololu protocol
    _stream->write(0xAA);
    serialW7(_deviceNumber);
    serialW7((uint8_t)cmd);
  }
  _lastError = 0;
}

/**** JrkG2I2C ****/

void JrkG2I2C::commandQuick(uint8_t cmd)
{
  AccWire.beginTransmission(_address);
  AccWire.write(cmd);
  _lastError = AccWire.endTransmission();
}

void JrkG2I2C::commandW7(uint8_t cmd, uint8_t val)
{
  AccWire.beginTransmission(_address);
  AccWire.write(cmd);
  AccWire.write(val & 0x7F);
  _lastError = AccWire.endTransmission();
}

void JrkG2I2C::commandWs14(uint8_t cmd, int16_t val)
{
  uint16_t v = val;
  AccWire.beginTransmission(_address);
  AccWire.write(cmd);
  AccWire.write(v & 0xFF);
  AccWire.write(v >> 8 & 0xFF);
  _lastError = AccWire.endTransmission();
}

uint8_t JrkG2I2C::commandR8(uint8_t cmd)
{
  AccWire.beginTransmission(_address);
  AccWire.write(cmd);
  _lastError = AccWire.endTransmission(false);  // no stop (repeated start)
  if (_lastError) { return 0; }

  uint8_t byteCount = AccWire.requestFrom(_address, (uint8_t)1);
  if (byteCount != 1)
  {
    _lastError = JrkG2CommReadError;
    return 0;
  }

  _lastError = 0;
  uint8_t val = AccWire.read();
  return val;
}

uint16_t JrkG2I2C::commandR16(uint8_t cmd)
{
  AccWire.beginTransmission(_address);
  AccWire.write(cmd);
  _lastError = AccWire.endTransmission(false);  // no stop (repeated start)
  if (_lastError) { return 0; }

  uint8_t byteCount = AccWire.requestFrom(_address, (uint8_t)2);
  if (byteCount != 2)
  {
    _lastError = JrkG2CommReadError;
    return 0;
  }

  _lastError = 0;
  uint8_t valL = AccWire.read();
  uint8_t valH = AccWire.read();
  return (uint16_t)valL | ((uint16_t)valH << 8);
}

void JrkG2I2C::segmentRead(uint8_t cmd, uint8_t offset,
  uint8_t length, uint8_t * buffer)
{
  // The Jrk does not allow reads longer than 15 bytes.
  if (length > 15) { length = 15; }

  AccWire.beginTransmission(_address);
  AccWire.write(cmd);
  AccWire.write(offset);
  _lastError = AccWire.endTransmission(false);  // no stop (repeated start)
  if (_lastError)
  {
    // Set the buffer bytes to 0 so the program will not use an uninitialized
    // variable.
    memset(buffer, 0, length);
    return;
  }

  uint8_t byteCount = AccWire.requestFrom(_address, (uint8_t)length);
  if (byteCount != length)
  {
    _lastError = JrkG2CommReadError;
    memset(buffer, 0, length);
    return;
  }

  _lastError = 0;
  for (uint8_t i = 0; i < length; i++)
  {
    buffer[i] = AccWire.read();
  }
}

void JrkG2I2C::segmentWrite(uint8_t cmd, uint8_t offset,
  uint8_t length, uint8_t * buffer)
{
  // The Jrk does not accept writes longer than 13 bytes over I2C.
  if (length > 13) { length = 13; }

  AccWire.beginTransmission(_address);
  AccWire.write((uint8_t)cmd);
  AccWire.write(offset);
  AccWire.write(length);
  for (uint8_t i = 0; i < length; i++)
  {
    AccWire.write(buffer[i]);
  }
  _lastError = AccWire.endTransmission();
}
