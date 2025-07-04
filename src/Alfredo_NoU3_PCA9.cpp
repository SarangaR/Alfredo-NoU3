#include "Alfredo_NoU3_PCA9.h"

#include <Arduino.h>
#include <Wire.h>

PCA9685::PCA9685()
{
  device_count_ = 0;
  for (DeviceIndex device_index=0; device_index<DEVICE_COUNT_MAX; ++device_index)
  {
    device_addresses_[device_index] = GENERAL_CALL_DEVICE_ADDRESS;
  }
}

void PCA9685::setupSingleDevice(TwoWire & wire,
  DeviceAddress device_address,
  bool fast_mode_plus)
{
  wire_ptr_ = &wire;
  //setWire(wire,fast_mode_plus);
  addDevice(device_address);
  resetAllDevices();
}

void PCA9685::setupOutputEnablePin(Pin output_enable_pin)
{
  pinMode(output_enable_pin,OUTPUT);
  digitalWrite(output_enable_pin,HIGH);
}

void PCA9685::enableOutputs(Pin output_enable_pin)
{
  digitalWrite(output_enable_pin,LOW);
}

void PCA9685::disableOutputs(Pin output_enable_pin)
{
  digitalWrite(output_enable_pin,HIGH);
}

PCA9685::Frequency PCA9685::getFrequencyMin()
{
  return MICROSECONDS_PER_SECOND / PWM_PERIOD_MAX_US;
}

PCA9685::Frequency PCA9685::getFrequencyMax()
{
  return MICROSECONDS_PER_SECOND / PWM_PERIOD_MIN_US;
}

void PCA9685::setToFrequency(Frequency frequency)
{
  setAllDevicesToFrequency(frequency);
}

PCA9685::Frequency PCA9685::getFrequency()
{
  Frequency frequency = 0;
  if (device_count_ > 0)
  {
    frequency = getSingleDeviceFrequency(device_addresses_[0]);
  }
  return frequency;
}

void PCA9685::setToServoFrequency()
{
  setAllDevicesToServoFrequency();
}

PCA9685::Frequency PCA9685::getServoFrequency()
{
  return SERVO_FREQUENCY;
}

PCA9685::ChannelCount PCA9685::getChannelCount()
{
  return CHANNELS_PER_DEVICE * device_count_;
}

PCA9685::Percent PCA9685::getDutyCycleMin()
{
  return PERCENT_MIN;
}

PCA9685::Percent PCA9685::getDutyCycleMax()
{
  return PERCENT_MAX;
}

PCA9685::Percent PCA9685::getPercentDelayMin()
{
  return PERCENT_MIN;
}

PCA9685::Percent PCA9685::getPercentDelayMax()
{
  return PERCENT_MAX;
}

void PCA9685::setChannelDutyCycle(Channel channel,
  Percent duty_cycle,
  Percent percent_delay)
{
  Duration pulse_width;
  Duration phase_shift;
  dutyCycleAndPercentDelayToPulseWidthAndPhaseShift(duty_cycle,percent_delay,pulse_width,phase_shift);
  setChannelPulseWidth(channel,pulse_width,phase_shift);
}

void PCA9685::getChannelDutyCycle(Channel channel,
  Percent & duty_cycle,
  Percent & percent_delay)
{
  Duration pulse_width;
  Duration phase_shift;
  getChannelPulseWidth(channel,pulse_width,phase_shift);
  pulseWidthAndPhaseShiftToDutyCycleAndPercentDelay(pulse_width,phase_shift,duty_cycle,percent_delay);
}

void PCA9685::setAllChannelsDutyCycle(Percent duty_cycle,
  Percent percent_delay)
{
  setAllDeviceChannelsDutyCycle(DEVICE_ADDRESS_ALL,duty_cycle,percent_delay);
}

PCA9685::Duration PCA9685::getPulseWidthMin()
{
  return TIME_MIN;
}

PCA9685::Duration PCA9685::getPulseWidthMax()
{
  return TIME_MAX;
}

PCA9685::Duration PCA9685::getPhaseShiftMin()
{
  return TIME_MIN;
}

PCA9685::Duration PCA9685::getPhaseShiftMax()
{
  return 0xFFFF;
}

void PCA9685::setChannelPulseWidth(Channel channel,
  Duration pulse_width,
  Duration phase_shift)
{
  Time on_time;
  Time off_time;
  pulseWidthAndPhaseShiftToOnTimeAndOffTime(pulse_width,phase_shift,on_time,off_time);
  setChannelOnAndOffTime(channel,on_time,off_time);
}

void PCA9685::getChannelPulseWidth(Channel channel,
  Duration & pulse_width,
  Duration & phase_shift)
{
  Time on_time = 0;
  Time off_time = 0;
  getChannelOnAndOffTime(channel,on_time,off_time);
  onTimeAndOffTimeToPulseWidthAndPhaseShift(on_time,off_time,pulse_width,phase_shift);
}

void PCA9685::setAllChannelsPulseWidth(Duration pulse_width,
  Duration phase_shift)
{
  setAllDeviceChannelsPulseWidth(DEVICE_ADDRESS_ALL,pulse_width,phase_shift);
}

void PCA9685::setChannelServoPulseDuration(Channel channel,
  DurationMicroseconds pulse_duration_microseconds)
{
  Duration pulse_width;
  Duration phase_shift;
  servoPulseDurationToPulseWidthAndPhaseShift(pulse_duration_microseconds,pulse_width,phase_shift);
  setChannelPulseWidth(channel,pulse_width,phase_shift);
}

void PCA9685::getChannelServoPulseDuration(Channel channel,
  DurationMicroseconds & pulse_duration_microseconds)
{
  Duration pulse_width;
  Duration phase_shift;
  getChannelPulseWidth(channel,pulse_width,phase_shift);
  pulseWidthAndPhaseShiftToServoPulseDuration(pulse_width,phase_shift,pulse_duration_microseconds);
}

void PCA9685::setAllChannelsServoPulseDuration(DurationMicroseconds pulse_duration_microseconds)
{
  setAllDeviceChannelsServoPulseDuration(DEVICE_ADDRESS_ALL,pulse_duration_microseconds);
}

PCA9685::Time PCA9685::getTimeMin()
{
  return TIME_MIN;
}

PCA9685::Time PCA9685::getTimeMax()
{
  return TIME_MAX;
}

void PCA9685::setChannelOnAndOffTime(Channel channel,
  Time on_time,
  Time off_time)
{
  if (channel >= getChannelCount())
  {
    return;
  }
  DeviceIndex device_index = channelToDeviceIndex(channel);
  Channel device_channel = channelToDeviceChannel(channel);
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  uint32_t data = (off_time << BITS_PER_TWO_BYTES) | on_time;
  write(device_addresses_[device_index],register_address,data);
}

void PCA9685::getChannelOnAndOffTime(Channel channel,
  Time & on_time,
  Time & off_time)
{
  if (channel >= getChannelCount())
  {
    return;
  }
  DeviceIndex device_index = channelToDeviceIndex(channel);
  Channel device_channel = channelToDeviceChannel(channel);
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  uint32_t data;
  read(device_index,register_address,data);
  on_time = data & TWO_BYTE_MAX;
  off_time = (data >> BITS_PER_TWO_BYTES) & TWO_BYTE_MAX;
}

void PCA9685::setAllChannelsOnAndOffTime(Time on_time,
  Time off_time)
{
  setAllDeviceChannelsOnAndOffTime(DEVICE_ADDRESS_ALL,on_time,off_time);
}

void PCA9685::setChannelOnTime(Channel channel,
  Time on_time)
{
  if (channel >= getChannelCount())
  {
    return;
  }
  DeviceIndex device_index = channelToDeviceIndex(channel);
  Channel device_channel = channelToDeviceChannel(channel);
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  write(device_addresses_[device_index],register_address,on_time);
}

void PCA9685::getChannelOnTime(Channel channel,
  Time & on_time)
{
  if (channel >= getChannelCount())
  {
    return;
  }
  DeviceIndex device_index = channelToDeviceIndex(channel);
  Channel device_channel = channelToDeviceChannel(channel);
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  read(device_index,register_address,on_time);
}

void PCA9685::setAllChannelsOnTime(Time on_time)
{
  setAllDeviceChannelsOnTime(DEVICE_ADDRESS_ALL,on_time);
}

void PCA9685::setChannelOffTime(Channel channel,
  Time off_time)
{
  if (channel >= getChannelCount())
  {
    return;
  }
  DeviceIndex device_index = channelToDeviceIndex(channel);
  Channel device_channel = channelToDeviceChannel(channel);
  uint8_t register_address = LED0_OFF_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  write(device_addresses_[device_index],register_address,off_time);
}

void PCA9685::getChannelOffTime(Channel channel,
  Time & off_time)
{
  if (channel >= getChannelCount())
  {
    return;
  }
  DeviceIndex device_index = channelToDeviceIndex(channel);
  Channel device_channel = channelToDeviceChannel(channel);
  uint8_t register_address = LED0_OFF_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  read(device_index,register_address,off_time);
}

void PCA9685::setAllChannelsOffTime(Time off_time)
{
  setAllDeviceChannelsOffTime(DEVICE_ADDRESS_ALL,off_time);
}

void PCA9685::setOutputsInverted()
{
  setAllDevicesOutputsInverted();
}

void PCA9685::setOutputsNotInverted()
{
  setAllDevicesOutputsNotInverted();
}

void PCA9685::setOutputsToTotemPole()
{
  setAllDevicesOutputsToTotemPole();
}

void PCA9685::setOutputsToOpenDrain()
{
  setAllDevicesOutputsToOpenDrain();
}

void PCA9685::setOutputsLowWhenDisabled()
{
  setAllDevicesOutputsLowWhenDisabled();
}

void PCA9685::setOutputsHighWhenDisabled()
{
  setAllDevicesOutputsHighWhenDisabled();
}

void PCA9685::setOutputsHighImpedanceWhenDisabled()
{
  setAllDevicesOutputsHighImpedanceWhenDisabled();
}

void PCA9685::setWire(TwoWire & wire,
  bool fast_mode_plus)
{
  wire_ptr_ = &wire;
  wire_ptr_->begin();
  if (fast_mode_plus)
  {
    wire_ptr_->setClock(FAST_MODE_PLUS_CLOCK_FREQUENCY);
  }

}

void PCA9685::addDevice(DeviceAddress device_address)
{
  if ((device_count_ >= DEVICE_COUNT_MAX) ||
    (device_address < DEVICE_ADDRESS_MIN) ||
    (device_address > DEVICE_ADDRESS_MAX))
  {
    return;
  }
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index != DEVICE_INDEX_NONE)
  {
    return;
  }
  device_addresses_[device_count_++] = device_address;
}

void PCA9685::resetAllDevices()
{
  wire_ptr_->beginTransmission(GENERAL_CALL_DEVICE_ADDRESS);
  wire_ptr_->write(SWRST);
  wire_ptr_->endTransmission();
  delay(10);
  wakeAll();
}

void PCA9685::addDeviceToGroup0(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sub1 = DOES_RESPOND;
  write(device_address,MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::removeDeviceFromGroup0(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sub1 = DOES_NOT_RESPOND;
  write(device_address,MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::addDeviceToGroup1(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sub2 = DOES_RESPOND;
  write(device_address,MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::removeDeviceFromGroup1(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sub2 = DOES_NOT_RESPOND;
  write(device_address,MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::addDeviceToGroup2(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sub3 = DOES_RESPOND;
  write(device_address,MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::removeDeviceFromGroup2(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sub3 = DOES_NOT_RESPOND;
  write(device_address,MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::setSingleDeviceToFrequency(DeviceAddress device_address,
  Frequency frequency)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  uint8_t prescale = frequencyToPrescale(frequency);
  setPrescale(device_index,prescale);
}

PCA9685::Frequency PCA9685::getSingleDeviceFrequency(DeviceAddress device_address)
{
  Frequency frequency = 0;
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index >= 0)
  {
    uint8_t prescale;
    getPrescale(device_index,prescale);
    frequency = prescaleToFrequency(prescale);
  }
  return frequency;
}

void PCA9685::setAllDevicesToFrequency(Frequency frequency)
{
  uint8_t prescale = frequencyToPrescale(frequency);
  for (DeviceIndex device_index=0; device_index<device_count_; ++device_index)
  {
    setPrescale(device_index,prescale);
  }
}

void PCA9685::setSingleDeviceToServoFrequency(DeviceAddress device_address)
{
  setSingleDeviceToFrequency(device_address,SERVO_FREQUENCY);
}

PCA9685::Frequency PCA9685::getSingleDeviceServoFrequency(DeviceAddress device_address)
{
  return SERVO_FREQUENCY;
}

void PCA9685::setAllDevicesToServoFrequency()
{
  setAllDevicesToFrequency(SERVO_FREQUENCY);
}

uint8_t PCA9685::getDeviceChannelCount()
{
  return CHANNELS_PER_DEVICE;
}

void PCA9685::setDeviceChannelDutyCycle(DeviceAddress device_address,
  Channel device_channel,
  Percent duty_cycle,
  Percent percent_delay)
{
  Duration pulse_width;
  Duration phase_shift;
  dutyCycleAndPercentDelayToPulseWidthAndPhaseShift(duty_cycle,percent_delay,pulse_width,phase_shift);
  setDeviceChannelPulseWidth(device_address,device_channel,pulse_width,phase_shift);
}

void PCA9685::setAllDeviceChannelsDutyCycle(DeviceAddress device_address,
  Percent duty_cycle,
  Percent percent_delay)
{
  Duration pulse_width;
  Duration phase_shift;
  dutyCycleAndPercentDelayToPulseWidthAndPhaseShift(duty_cycle,percent_delay,pulse_width,phase_shift);
  setAllDeviceChannelsPulseWidth(device_address,pulse_width,phase_shift);
}

void PCA9685::setDeviceChannelPulseWidth(DeviceAddress device_address,
  Channel device_channel,
  Duration pulse_width,
  Duration phase_shift)
{
  Time on_time;
  Time off_time;
  pulseWidthAndPhaseShiftToOnTimeAndOffTime(pulse_width,phase_shift,on_time,off_time);
  setDeviceChannelOnAndOffTime(device_address,device_channel,on_time,off_time);
}

void PCA9685::setAllDeviceChannelsPulseWidth(DeviceAddress device_address,
  Duration pulse_width,
  Duration phase_shift)
{
  Time on_time;
  Time off_time;
  pulseWidthAndPhaseShiftToOnTimeAndOffTime(pulse_width,phase_shift,on_time,off_time);
  setAllDeviceChannelsOnAndOffTime(device_address,on_time,off_time);
}

void PCA9685::setDeviceChannelServoPulseDuration(DeviceAddress device_address,
  Channel device_channel,
  DurationMicroseconds pulse_duration_microseconds)
{
  Duration pulse_width;
  Duration phase_shift;
  servoPulseDurationToPulseWidthAndPhaseShift(pulse_duration_microseconds,pulse_width,phase_shift);
  setDeviceChannelPulseWidth(device_address,device_channel,pulse_width,phase_shift);
}

void PCA9685::setAllDeviceChannelsServoPulseDuration(DeviceAddress device_address,
  DurationMicroseconds pulse_duration_microseconds)
{
  Duration pulse_width;
  Duration phase_shift;
  servoPulseDurationToPulseWidthAndPhaseShift(pulse_duration_microseconds,pulse_width,phase_shift);
  setAllDeviceChannelsPulseWidth(device_address,pulse_width,phase_shift);
}

void PCA9685::setDeviceChannelOnAndOffTime(DeviceAddress device_address,
  Channel device_channel,
  Time on_time,
  Time off_time)
{
  if (device_channel >= getDeviceChannelCount())
  {
    return;
  }
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  uint32_t data = (off_time << BITS_PER_TWO_BYTES) | on_time;
  write(device_address,register_address,data);
}

void PCA9685::setAllDeviceChannelsOnAndOffTime(DeviceAddress device_address,
  Time on_time,
  Time off_time)
{
  uint8_t register_address = ALL_LED_ON_L_REGISTER_ADDRESS;
  uint32_t data = (off_time << BITS_PER_TWO_BYTES) | on_time;
  write(device_address,register_address,data);
}

void PCA9685::setDeviceChannelOnTime(DeviceAddress device_address,
  Channel device_channel,
  Time on_time)
{
  if (device_channel >= getDeviceChannelCount())
  {
    return;
  }
  uint8_t register_address = LED0_ON_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  write(device_address,register_address,on_time);
}

void PCA9685::setAllDeviceChannelsOnTime(DeviceAddress device_address,
  Time on_time)
{
  uint8_t register_address = ALL_LED_ON_L_REGISTER_ADDRESS;
  write(device_address,register_address,on_time);
}

void PCA9685::setDeviceChannelOffTime(DeviceAddress device_address,
  Channel device_channel,
  Time off_time)
{
  if (device_channel >= getDeviceChannelCount())
  {
    return;
  }
  uint8_t register_address = LED0_OFF_L_REGISTER_ADDRESS + LED_REGISTERS_SIZE * device_channel;
  write(device_address,register_address,off_time);
}

void PCA9685::setAllDeviceChannelsOffTime(DeviceAddress device_address,
  Time off_time)
{
  uint8_t register_address = ALL_LED_OFF_L_REGISTER_ADDRESS;
  write(device_address,register_address,off_time);
}

void PCA9685::setSingleDeviceOutputsInverted(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsInverted(device_index);
}

void PCA9685::setAllDevicesOutputsInverted()
{
  for (DeviceIndex device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsInverted(device_index);
  }
}

void PCA9685::setSingleDeviceOutputsNotInverted(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsNotInverted(device_index);
}

void PCA9685::setAllDevicesOutputsNotInverted()
{
  for (DeviceIndex device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsNotInverted(device_index);
  }
}

void PCA9685::setSingleDeviceOutputsToTotemPole(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsToTotemPole(device_index);
}

void PCA9685::setAllDevicesOutputsToTotemPole()
{
  for (DeviceIndex device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsToTotemPole(device_index);
  }
}

void PCA9685::setSingleDeviceOutputsToOpenDrain(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsToOpenDrain(device_index);
}

void PCA9685::setAllDevicesOutputsToOpenDrain()
{
  for (DeviceIndex device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsToOpenDrain(device_index);
  }
}

void PCA9685::setSingleDeviceOutputsLowWhenDisabled(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsLowWhenDisabled(device_index);
}

void PCA9685::setAllDevicesOutputsLowWhenDisabled()
{
  for (DeviceIndex device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsLowWhenDisabled(device_index);
  }
}

void PCA9685::setSingleDeviceOutputsHighWhenDisabled(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsHighWhenDisabled(device_index);
}

void PCA9685::setAllDevicesOutputsHighWhenDisabled()
{
  for (DeviceIndex device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsHighWhenDisabled(device_index);
  }
}

void PCA9685::setSingleDeviceOutputsHighImpedanceWhenDisabled(DeviceAddress device_address)
{
  int device_index = deviceAddressToDeviceIndex(device_address);
  if (device_index < 0)
  {
    return;
  }
  setOutputsHighImpedanceWhenDisabled(device_index);
}

void PCA9685::setAllDevicesOutputsHighImpedanceWhenDisabled()
{
  for (DeviceIndex device_index=0; device_index<device_count_; ++device_index)
  {
    setOutputsHighImpedanceWhenDisabled(device_index);
  }
}

// private

int PCA9685::deviceAddressToDeviceIndex(DeviceAddress device_address)
{
  int device_index = DEVICE_INDEX_NONE;
  if (device_address == DEVICE_ADDRESS_ALL)
  {
    device_index = DEVICE_INDEX_ALL;
  }
  else if (device_address == DEVICE_ADDRESS_GROUP0)
  {
    device_index = DEVICE_INDEX_GROUP0;
  }
  else if (device_address == DEVICE_ADDRESS_GROUP1)
  {
    device_index = DEVICE_INDEX_GROUP1;
  }
  else if (device_address == DEVICE_ADDRESS_GROUP2)
  {
    device_index = DEVICE_INDEX_GROUP2;
  }
  else
  {
    for (uint8_t index=0; index<device_count_; ++index)
    {
      if (device_address == device_addresses_[index])
      {
        device_index = index;
        break;
      }
    }
  }
  return device_index;
}

PCA9685::Mode1Register PCA9685::readMode1Register(DeviceIndex device_index)
{
  Mode1Register mode1_register;
  read(device_index,MODE1_REGISTER_ADDRESS,mode1_register.data);
  return mode1_register;
}

PCA9685::Mode2Register PCA9685::readMode2Register(DeviceIndex device_index)
{
  Mode2Register mode2_register;
  read(device_index,MODE2_REGISTER_ADDRESS,mode2_register.data);
  return mode2_register;
}

void PCA9685::sleep(DeviceIndex device_index)
{
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sleep = SLEEP;
  write(device_addresses_[device_index],MODE1_REGISTER_ADDRESS,mode1_register.data);
}

void PCA9685::wake(DeviceIndex device_index)
{
  Mode1Register mode1_register = readMode1Register(device_index);
  mode1_register.fields.sleep = WAKE;
  mode1_register.fields.ai = AUTO_INCREMENT_ENABLED;
  write(device_addresses_[device_index],MODE1_REGISTER_ADDRESS,mode1_register.data);
  if (mode1_register.fields.restart == RESTART_ENABLED)
  {
    delay(1);
    mode1_register.fields.restart = RESTART_CLEAR;
    write(device_addresses_[device_index],MODE1_REGISTER_ADDRESS,mode1_register.data);
  }
}

void PCA9685::wakeAll()
{
  for (DeviceIndex device_index=0; device_index<device_count_; ++device_index)
  {
    wake(device_index);
  }
}

void PCA9685::setAllDevicesToExternalClock()
{
  //go to sleep
  for (DeviceIndex device_index=0; device_index<device_count_; ++device_index)
  {
    Mode1Register mode1_register = readMode1Register(device_index);
    mode1_register.fields.sleep = SLEEP;
    write(device_addresses_[device_index],MODE1_REGISTER_ADDRESS,mode1_register.data);
  }
  delay(10);

  //use prescale to get minimum period
  for (DeviceIndex device_index=0; device_index<device_count_; ++device_index)
  {
    write(device_addresses_[device_index],PRE_SCALE_REGISTER_ADDRESS, PRE_SCALE_MIN);
  }
  delay(10);

  //set clock to external and wake up
  for (DeviceIndex device_index=0; device_index<device_count_; ++device_index)
  {
    Mode1Register mode1_register = readMode1Register(device_index);
    mode1_register.fields.extclk = USE_EXTERNAL_CLOCK;
    mode1_register.fields.sleep = WAKE;
    write(device_addresses_[device_index],MODE1_REGISTER_ADDRESS,mode1_register.data);
  }
}

void PCA9685::setPrescale(DeviceIndex device_index,
  uint8_t prescale)
{
  sleep(device_index);
  write(device_addresses_[device_index],PRE_SCALE_REGISTER_ADDRESS,prescale);
  wake(device_index);
}

void PCA9685::getPrescale(DeviceIndex device_index,
  uint8_t & prescale)
{
  read(device_index,PRE_SCALE_REGISTER_ADDRESS,prescale);
}

uint8_t PCA9685::frequencyToPrescale(Frequency frequency)
{
  DurationMicroseconds period_us = MICROSECONDS_PER_SECOND / frequency;
  period_us = constrain(period_us,PWM_PERIOD_MIN_US,PWM_PERIOD_MAX_US);
  uint8_t prescale = map(period_us,PWM_PERIOD_MIN_US,PWM_PERIOD_MAX_US,PRE_SCALE_MIN,PRE_SCALE_MAX);
  return prescale;
}

PCA9685::Frequency PCA9685::prescaleToFrequency(uint8_t prescale)
{
  Frequency frequency = 0;
  DurationMicroseconds period_us = map(prescale,PRE_SCALE_MIN,PRE_SCALE_MAX,PWM_PERIOD_MIN_US,PWM_PERIOD_MAX_US);
  if (period_us > 0)
  {
    frequency = round((double)MICROSECONDS_PER_SECOND / (double)period_us);
  }
  return frequency;
}

uint8_t PCA9685::channelToDeviceIndex(Channel channel)
{
  return channel / CHANNELS_PER_DEVICE;
}

PCA9685::Channel PCA9685::channelToDeviceChannel(Channel channel)
{
  return channel % CHANNELS_PER_DEVICE;
}

void PCA9685::dutyCycleAndPercentDelayToPulseWidthAndPhaseShift(Percent duty_cycle,
  Percent percent_delay,
  Duration & pulse_width,
  Duration & phase_shift)
{
  pulse_width = round(((double)TIME_MAX * duty_cycle) / (double)PERCENT_MAX);
  phase_shift = round(((double)TIME_MAX * percent_delay) / (double)PERCENT_MAX);
}

void PCA9685::pulseWidthAndPhaseShiftToDutyCycleAndPercentDelay(Duration pulse_width,
  Duration phase_shift,
  Percent & duty_cycle,
  Percent & percent_delay)
{
  duty_cycle = (double)(pulse_width * PERCENT_MAX) / (double)TIME_MAX;
  percent_delay = (double)(phase_shift * PERCENT_MAX) / (double)TIME_MAX;
}

void PCA9685::pulseWidthAndPhaseShiftToOnTimeAndOffTime(Duration pulse_width,
  Duration phase_shift,
  Time & on_time,
  Time & off_time)
{
  if (pulse_width == TIME_MIN)
  {
    on_time = TIME_MIN;
    off_time = TIME_MAX;
    return;
  }
  if (pulse_width >= TIME_MAX)
  {
    on_time = TIME_MAX;
    off_time = TIME_MIN;
    return;
  }
  on_time = phase_shift % TIME_MAX;
  off_time = (on_time + pulse_width) % TIME_MAX;
}

void PCA9685::onTimeAndOffTimeToPulseWidthAndPhaseShift(Time on_time,
  Time off_time,
  Duration & pulse_width,
  Duration & phase_shift)
{
  if (on_time == TIME_MAX)
  {
    pulse_width = TIME_MAX;
    phase_shift = TIME_MIN;
    return;
  }
  if (off_time == TIME_MAX)
  {
    pulse_width = TIME_MIN;
    phase_shift = TIME_MIN;
    return;
  }
  if (on_time > off_time)
  {
    pulse_width = TIME_MAX - (on_time - off_time);
    phase_shift = on_time;
    return;
  }
  pulse_width = off_time - on_time;
  phase_shift = on_time;
}

void PCA9685::servoPulseDurationToPulseWidthAndPhaseShift(DurationMicroseconds pulse_duration_microseconds,
  Duration & pulse_width,
  Duration & phase_shift)
{
  phase_shift = 0;
  pulse_width = (pulse_duration_microseconds * TIME_MAX) / SERVO_PERIOD_MICROSECONDS;
}

void PCA9685::pulseWidthAndPhaseShiftToServoPulseDuration(Duration pulse_width,
  Duration phase_shift,
  DurationMicroseconds & pulse_duration_microseconds)
{
  DurationMicroseconds period_us = SERVO_PERIOD_MICROSECONDS;
  pulse_duration_microseconds = (pulse_width * period_us) / TIME_MAX;
}

void PCA9685::setOutputsInverted(DeviceIndex device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.invrt = OUTPUTS_INVERTED;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsNotInverted(DeviceIndex device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.invrt = OUTPUTS_NOT_INVERTED;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsToTotemPole(DeviceIndex device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.outdrv = OUTPUTS_TOTEM_POLE;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsToOpenDrain(DeviceIndex device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.outdrv = OUTPUTS_OPEN_DRAIN;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsLowWhenDisabled(DeviceIndex device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.outne = OUTPUTS_LOW_WHEN_DISABLED;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsHighWhenDisabled(DeviceIndex device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.outne = OUTPUTS_HIGH_WHEN_DISABLED;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

void PCA9685::setOutputsHighImpedanceWhenDisabled(DeviceIndex device_index)
{
  Mode2Register mode2_register = readMode2Register(device_index);
  mode2_register.fields.outne = OUTPUTS_HIGH_IMPEDANCE_WHEN_DISABLED;
  write(device_addresses_[device_index],MODE2_REGISTER_ADDRESS,mode2_register.data);
}

template<typename T>
void PCA9685::write(DeviceAddress device_address,
  uint8_t register_address,
  T data)
{
  int byte_count = sizeof(data);
  wire_ptr_->beginTransmission(device_address);
  wire_ptr_->write(register_address);
  uint8_t write_byte;
  for (int byte_n=0; byte_n<byte_count; ++byte_n)
  {
    write_byte = (data >> (BITS_PER_BYTE * byte_n)) & BYTE_MAX;
    wire_ptr_->write(write_byte);
  }
  wire_ptr_->endTransmission();
}

template<typename T>
void PCA9685::read(DeviceIndex device_index,
  uint8_t register_address,
  T & data)
{
  int byte_count = sizeof(data);
  int device_address = device_addresses_[device_index];
  wire_ptr_->beginTransmission(device_address);
  wire_ptr_->write(register_address);
  wire_ptr_->endTransmission();

  wire_ptr_->requestFrom(device_address,byte_count);
  data = 0;
  for (int byte_n=0; byte_n<byte_count; ++byte_n)
  {
    data |= (wire_ptr_->read()) << (BITS_PER_BYTE * byte_n);
  }
}