#include "encoder_as5600.h"
#include "config.h"
#include <Wire.h>

namespace
{
  uint16_t lastRawAngle = 0;
  int64_t accumPos = 0;

  int64_t sumAngle = 0;
  uint8_t sampleCount = 0;
  float prevMean = 0.0f;

  unsigned long lastSampleTime = 0;

  uint8_t readStatusRegister()
  {
    Wire.beginTransmission(Enc::AS5600_ADDR);
    Wire.write(Enc::REG_STATUS);
    if (Wire.endTransmission() != 0)
      return 0xFF;
    Wire.requestFrom(Enc::AS5600_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
  }

  uint16_t readRawAngle()
  {
    Wire.beginTransmission(Enc::AS5600_ADDR);
    Wire.write(0x0E);
    Wire.endTransmission();
    Wire.requestFrom(Enc::AS5600_ADDR, (uint8_t)2);
    if (Wire.available() < 2)
      return lastRawAngle;
    const uint16_t high = Wire.read();
    const uint16_t low = Wire.read();
    return ((high << 8) | low) & 0x0FFF;
  }

  int16_t readDeltaAngle()
  {
    const uint16_t raw = readRawAngle();
    int16_t diff = (int16_t)lastRawAngle - (int16_t)raw;
    if (diff > 2048)
      diff -= 4096;
    else if (diff < -2048)
      diff += 4096;
    lastRawAngle = raw;
    return diff;
  }

  int64_t readAccumulatedAngle()
  {
    accumPos += (int64_t)readDeltaAngle();
    return accumPos;
  }

} // namespace

void initEncoder()
{
  Wire.begin();
  lastRawAngle = readRawAngle();
  accumPos = 0;
  sumAngle = 0;
  sampleCount = 0;
  prevMean = 0.0f;
  lastSampleTime = micros();
  Serial.println("Encoder listo");
  Serial.println(lastRawAngle);
}

void centerPosition()
{
  lastRawAngle = readRawAngle();
  accumPos = 0;
  sumAngle = 0;
  sampleCount = 0;
  prevMean = 0.0f;
}

bool magnetPresent()
{
  const uint8_t status = readStatusRegister();
  return (status != 0xFF) && (status & 0x20); // MD bit
}

bool sampleBlockAndGetDelta(int16_t &delta_out)
{
  const unsigned long now = micros();
  if ((now - lastSampleTime) < Enc::SAMPLE_US || !magnetPresent())
  {
    return false;
  }

  if (!magnetPresent())
    Serial.println("magnet not detected");

  lastSampleTime += Enc::SAMPLE_US;

  const int64_t pos = readAccumulatedAngle();
  sumAngle += pos;
  ++sampleCount;

  if (sampleCount < Enc::SAMPLES_PER_BLOCK)
  {
    return false;
  }

  const float mean = float(sumAngle) / Enc::SAMPLES_PER_BLOCK;
  float deltaF = mean - prevMean;
  prevMean = mean;

  sumAngle = 0;
  sampleCount = 0;

  if (deltaF > Ui::DELTA_MAX)
    deltaF = Ui::DELTA_MAX;
  if (deltaF < -Ui::DELTA_MAX)
    deltaF = -Ui::DELTA_MAX;

  delta_out = (int16_t)deltaF;
  return true;
}
