#include "Alfredo_NoU3_I2S.h"

#include <Arduino.h>
#include "freertos/ringbuf.h"
#include <wiring_private.h>
#include "freertos/semphr.h"

namespace esp_i2s {
  #include "driver/i2s.h" // ESP specific i2s driver
}

#define _I2S_EVENT_QUEUE_LENGTH 1

//DBC = Real_DMA_Buff_size / (#CH * bytes per sample) = RDBS/2
#define DMA_BUFFER_COUNT 2 // BUFFER COUNT must be between 2 and 128
#define DMA_BUF_LEN 64

#define SAMPLE_RATE 512000
#define BYTES_PER_SAMPLE 2

#define I2S_DEVICE 0
#define I2S_CLOCK_GENERATOR 0 // does nothing for ESP

class I2SClass : public Stream
{
public:
  // The device index and pins must map to the "COM" pads in Table 6-1 of the datasheet
  I2SClass(uint8_t deviceIndex, uint8_t clockGenerator);

  // Init in MASTER mode: the SCK and FS pins are driven as outputs using the sample rate
  int begin();
  int setAllPins(int sckPin, int fsPin, int sdPin, int outSdPin, int inSdPin);
  void end();

  // from Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  // from Print
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buffer, size_t size);

  virtual int availableForWrite();

  int read(void* buffer, size_t size);

  size_t write_nonblocking(const void *buffer, size_t size);

private:

  int _enableTransmitter();
  void _onTransferComplete();

  int _createCallbackTask();

  static void onDmaTransferComplete(void*);
  void _uninstallDriver();
  int  _applyPinSetting();

private:
  typedef enum {
    I2S_STATE_IDLE,
    I2S_STATE_TRANSMITTER,
    I2S_STATE_RECEIVER,
    I2S_STATE_DUPLEX
  } i2s_state_t;

  int _deviceIndex;
  int _sdPin;
  int _inSdPin;
  int _outSdPin;
  int _sckPin;
  int _fsPin;

  i2s_state_t _state;
  int _bitsPerSample;
  uint32_t _sampleRate;

  uint16_t _buffer_byte_size;

  bool _driverInstalled; // Is IDF I2S driver installed?
  bool _initialized; // Is everything initialized (callback task, I2S driver, ring buffers)?
  TaskHandle_t _callbackTaskHandle;
  QueueHandle_t _i2sEventQueue;
  SemaphoreHandle_t _i2s_general_mutex;
  RingbufHandle_t _input_ring_buffer;
  RingbufHandle_t _output_ring_buffer;
  int _i2s_dma_buffer_size;
  bool _driveClock;
  uint32_t _peek_buff;
  bool _peek_buff_valid;

  void _tx_done_routine(uint8_t* prev_item);

  uint16_t _nesting_counter;
  void _take_if_not_holding();
  void _give_if_top_call();
  void _fix_and_write(void *output, size_t size, size_t *bytes_written = NULL, size_t *actual_bytes_written = NULL);
};

extern I2SClass I2S;

//--------------------------------------------------------------------------------------------------------------------------------//

I2SClass::I2SClass(uint8_t deviceIndex, uint8_t clockGenerator) :
  _deviceIndex(deviceIndex),
  _sdPin(-1),             // shared data pin
  _sckPin(-1),           // clock pin
  _fsPin(-1),             // frame (word) select pin

  _state(I2S_STATE_IDLE),
  _bitsPerSample(0),
  _sampleRate(0),

  _buffer_byte_size(0),

  _driverInstalled(false),
  _initialized(false),
  _callbackTaskHandle(NULL),
  _i2sEventQueue(NULL),
  _i2s_general_mutex(NULL),
  _input_ring_buffer(NULL),
  _output_ring_buffer(NULL),
  _i2s_dma_buffer_size(DMA_BUF_LEN), // Number of frames in each DMA buffer. Frame size = number of channels * Bytes per sample; Must be between 8 and 1024
  _driveClock(true),
  _peek_buff(0),
  _peek_buff_valid(false),
  _nesting_counter(0)
{
  _i2s_general_mutex = xSemaphoreCreateMutex();
  if(_i2s_general_mutex == NULL){
    log_e("I2S could not create internal mutex!");
  }
}

int I2SClass::_createCallbackTask(){
  int stack_size = 20000;
  if(_callbackTaskHandle != NULL){
    log_e("Callback task already exists!");
    return 0; // ERR
  }

  xTaskCreate(
    onDmaTransferComplete,   // Function to implement the task
    "onDmaTransferComplete", // Name of the task
    stack_size,              // Stack size in words
    NULL,                    // Task input parameter
    2,                       // Priority of the task
    &_callbackTaskHandle     // Task handle.
    );
  if(_callbackTaskHandle == NULL){
    log_e("Could not create callback task");
    return 0; // ERR
  }
  return 1; // OK
}


// Core function
int I2SClass::begin(){

  _take_if_not_holding();

  _state = I2S_STATE_DUPLEX;

  _driveClock = true;
  _sampleRate = (uint32_t) SAMPLE_RATE;
  _bitsPerSample = BYTES_PER_SAMPLE * 8;

  esp_i2s::i2s_config_t i2s_config = {
    .mode = (esp_i2s::i2s_mode_t)(esp_i2s::I2S_MODE_TX | esp_i2s::I2S_MODE_MASTER),
    .sample_rate = _sampleRate,
    .bits_per_sample = (esp_i2s::i2s_bits_per_sample_t)_bitsPerSample,
    .channel_format = esp_i2s::I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (esp_i2s::i2s_comm_format_t)(esp_i2s::I2S_COMM_FORMAT_STAND_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = DMA_BUFFER_COUNT,
    .dma_buf_len = _i2s_dma_buffer_size,
    .use_apll = true
  };

  esp_i2s::i2s_driver_install((esp_i2s::i2s_port_t) _deviceIndex, &i2s_config, _I2S_EVENT_QUEUE_LENGTH, &_i2sEventQueue);
  esp_i2s::i2s_set_clk((esp_i2s::i2s_port_t) _deviceIndex, _sampleRate, (esp_i2s::i2s_bits_per_sample_t)_bitsPerSample, esp_i2s::I2S_CHANNEL_MONO);

  _driverInstalled = true;
  _applyPinSetting();

  _buffer_byte_size = _i2s_dma_buffer_size * (_bitsPerSample / 8) * DMA_BUFFER_COUNT * 2;
  _input_ring_buffer  = xRingbufferCreate(_buffer_byte_size, RINGBUF_TYPE_BYTEBUF);
  _output_ring_buffer = xRingbufferCreate(_buffer_byte_size, RINGBUF_TYPE_BYTEBUF);

  _createCallbackTask();

  _initialized = true;

  _give_if_top_call();

  return 1; // OK
}

int I2SClass::_applyPinSetting(){
  if(_driverInstalled){

    esp_i2s::i2s_pin_config_t pin_config = {
      .bck_io_num = _sckPin,
      .ws_io_num = _fsPin,
      .data_out_num = _outSdPin,
      .data_in_num = _inSdPin
    };

    i2s_set_pin((esp_i2s::i2s_port_t) _deviceIndex, &pin_config);

  } // if(_driverInstalled)
  return 1; // OK
}

int I2SClass::setAllPins(int sckPin, int fsPin, int sdPin, int outSdPin, int inSdPin){
  _take_if_not_holding();

  if(sckPin >= 0) _sckPin = sckPin;
  else            _sckPin = -1;

  if(fsPin >= 0) _fsPin = fsPin;
  else           _fsPin = -1;

  if(sdPin >= 0) _sdPin = sdPin;
  else           _sdPin = -1;

  if(outSdPin >= 0) _outSdPin = outSdPin;
  else              _outSdPin = -1;

  if(inSdPin >= 0) _inSdPin = inSdPin;
  else             _inSdPin = -1;

  int ret = _applyPinSetting();
  _give_if_top_call();
  return ret;
}


void I2SClass::_uninstallDriver(){
  if(_driverInstalled){
    esp_i2s::i2s_driver_uninstall((esp_i2s::i2s_port_t) _deviceIndex);

    if(_state != I2S_STATE_DUPLEX){
      _state = I2S_STATE_IDLE;
    }
    _driverInstalled = false;
  } // if(_driverInstalled)
}

void I2SClass::end(){
  _take_if_not_holding();
  if(xTaskGetCurrentTaskHandle() != _callbackTaskHandle){
    if(_callbackTaskHandle){
      vTaskDelete(_callbackTaskHandle);
      _callbackTaskHandle = NULL; // prevent secondary termination to non-existing task
    }
    _uninstallDriver();
    if(_input_ring_buffer != NULL){
      vRingbufferDelete(_input_ring_buffer);
      _input_ring_buffer = NULL;
    }
    if(_output_ring_buffer != NULL){
      vRingbufferDelete(_output_ring_buffer);
      _output_ring_buffer = NULL;
    }
    _initialized = false;
  }else{
    log_w("WARNING: ending I2SClass from callback task not permitted, but attempted!");
  }
  _give_if_top_call();
}

// Bytes available to read
int I2SClass::available(){
  _take_if_not_holding();
  int ret = 0;
  if(_input_ring_buffer != NULL){
    ret = _buffer_byte_size - (int)xRingbufferGetCurFreeSize(_input_ring_buffer);
  }
  _give_if_top_call();
  return ret;
}


int I2SClass::read(){

}

int I2SClass::read(void* buffer, size_t size){

}

size_t I2SClass::write(uint8_t data){

}


size_t I2SClass::write(const uint8_t *buffer, size_t size){

}

// non-blocking version of write
// In case there is not enough space in buffer to write requested size
// this function will try to flush the buffer and write requested data with 0 time-out
size_t I2SClass::write_nonblocking(const void *buffer, size_t size){
  _take_if_not_holding();
  if(_initialized){
    if(_state != I2S_STATE_TRANSMITTER && _state != I2S_STATE_DUPLEX){
      if(!_enableTransmitter()){
        _give_if_top_call();
        return 0; // There was an error switching to transmitter
      }
    }
    if(availableForWrite() < size){
      flush();
    }
    if(_output_ring_buffer != NULL){
      if(pdTRUE == xRingbufferSend(_output_ring_buffer, buffer, size, 0)){
        _give_if_top_call();
        return size;
      }else{
        log_w("I2S could not write all data into ring buffer!");
        _give_if_top_call();
        return 0;
      }
    }
  } // if(_initialized)
  return 0;
  _give_if_top_call(); // this should not be needed
}

/*
  Read 1 sample from internal buffer and return it.
  Repeated peeks will return the same sample until read is called.
*/
int I2SClass::peek(){
  _take_if_not_holding();
  int ret = 0;
  if(_initialized && _input_ring_buffer != NULL && !_peek_buff_valid){
    size_t item_size = 0;
    void *item = NULL;

    item = xRingbufferReceiveUpTo(_input_ring_buffer, &item_size, 0, _bitsPerSample/8); // fetch 1 sample
    if (item != NULL && item_size == _bitsPerSample/8){
      _peek_buff = *((int*)item);
      vRingbufferReturnItem(_input_ring_buffer, item);
      _peek_buff_valid = true;
    }

  } // if(_initialized)
  if(_peek_buff_valid){
    ret = _peek_buff;
  }
  _give_if_top_call();
  return ret;
}

void I2SClass::flush(){
  _take_if_not_holding();
  if(_initialized){
    const size_t single_dma_buf = _i2s_dma_buffer_size*(_bitsPerSample/8)*2;
    size_t item_size = 0;
    void *item = NULL;
    if(_output_ring_buffer != NULL){
      item = xRingbufferReceiveUpTo(_output_ring_buffer, &item_size, 0, single_dma_buf);
      if (item != NULL){
        _fix_and_write(item, item_size);
        vRingbufferReturnItem(_output_ring_buffer, item);
      }
    }
  } // if(_initialized)
  _give_if_top_call();
}

// Bytes available to write
int I2SClass::availableForWrite(){
  _take_if_not_holding();
  int ret = 0;
  if(_initialized){
    if(_output_ring_buffer != NULL){
      ret = (int)xRingbufferGetCurFreeSize(_output_ring_buffer);
    }
  } // if(_initialized)
  _give_if_top_call();
  return ret;
}

int I2SClass::_enableTransmitter(){
  if(_state != I2S_STATE_DUPLEX && _state != I2S_STATE_TRANSMITTER){
    _state = I2S_STATE_TRANSMITTER;
    return _applyPinSetting();
  }
  return 1; // Ok
}

void I2SClass::_tx_done_routine(uint8_t* prev_item){
  static bool prev_item_valid = false;
  const size_t single_dma_buf = _i2s_dma_buffer_size*(_bitsPerSample/8)*2; // *2 for stereo - it has double number of samples for 2 channels
  static size_t item_size = 0;
  static size_t prev_item_size = 0;
  static void *item = NULL;
  static int prev_item_offset = 0;
  static size_t bytes_written = 0;

  if(prev_item_valid){ // use item from previous round
    _fix_and_write(prev_item+prev_item_offset, prev_item_size, &bytes_written);
    if(prev_item_size == bytes_written){
      prev_item_valid = false;
    } // write size check
    prev_item_offset = bytes_written;
    prev_item_size -= bytes_written;
  } // prev_item_valid

  if(_output_ring_buffer != NULL && (_buffer_byte_size - xRingbufferGetCurFreeSize(_output_ring_buffer) >= single_dma_buf)){ // fill up the I2S DMA buffer
    bytes_written = 0;
    item_size = 0;
    if(_buffer_byte_size - xRingbufferGetCurFreeSize(_output_ring_buffer) >= _i2s_dma_buffer_size*(_bitsPerSample/8)){ // don't read from almost empty buffer
      item = xRingbufferReceiveUpTo(_output_ring_buffer, &item_size, pdMS_TO_TICKS(0), single_dma_buf);
      if (item != NULL){
        _fix_and_write(item, item_size, &bytes_written);
        if(item_size != bytes_written){ // save item that was not written correctly for later
          memcpy(prev_item, (void*)&((uint8_t*)item)[bytes_written], item_size-bytes_written);
          prev_item_size = item_size - bytes_written;
          prev_item_offset = 0;
          prev_item_valid = true;
        } // save item that was not written correctly for later
        vRingbufferReturnItem(_output_ring_buffer, item);
      } // Check received item
    } // don't read from almost empty buffer
  } // fill up the I2S DMA buffer
}

void I2SClass::_onTransferComplete(){
  uint8_t prev_item[_i2s_dma_buffer_size*4];
  esp_i2s::i2s_event_t i2s_event;

  while(true){
    xQueueReceive(_i2sEventQueue, &i2s_event, portMAX_DELAY);
    if(i2s_event.type == esp_i2s::I2S_EVENT_TX_DONE){
      _tx_done_routine(prev_item);
    }
  } // infinite loop
}

void I2SClass::onDmaTransferComplete(void*){
  I2S._onTransferComplete();
}

void I2SClass::_take_if_not_holding(){
  TaskHandle_t mutex_holder = xSemaphoreGetMutexHolder(_i2s_general_mutex);
  if(mutex_holder != NULL && mutex_holder == xTaskGetCurrentTaskHandle()){
    ++_nesting_counter;
    return; // we are already holding this mutex - no need to take it
  }

  // we are not holding the mutex - wait for it and take it
  if(xSemaphoreTake(_i2s_general_mutex, portMAX_DELAY) != pdTRUE ){
    log_e("I2S internal mutex take returned with error");
  }
  //_give_if_top_call(); // call after this function
}

void I2SClass::_give_if_top_call(){
  if(_nesting_counter){
    --_nesting_counter;
  }else{
    if(xSemaphoreGive(_i2s_general_mutex) != pdTRUE){
      log_e("I2S internal mutex give error");
    }
  }
}

// Prepares data and writes them to IDF i2s driver.
// This counters possible bug in ESP IDF I2S driver
// output - bytes to be sent
// size - number of bytes in original buffer
// bytes_written - number of bytes used from original buffer
// actual_bytes_written - number of bytes written by i2s_write after fix
void I2SClass::_fix_and_write(void *output, size_t size, size_t *bytes_written, size_t *actual_bytes_written){
  ulong src_ptr = 0;
  uint8_t* buff = NULL;
  size_t buff_size = size;
  switch(_bitsPerSample){
    case 8:
      buff_size = size *2;
      buff = (uint8_t*)calloc(buff_size, sizeof(uint8_t));
      if(buff == NULL){
        log_e("callock error");
        if(bytes_written != NULL){ *bytes_written = 0; }
        return;
      }
      for(int i = 0; i < buff_size ; i+=4){
        ((uint8_t*)buff)[i+3] = (uint16_t)((uint8_t*)output)[src_ptr++];
        ((uint8_t*)buff)[i+1] = (uint16_t)((uint8_t*)output)[src_ptr++];
      }
    break;
    case 16:
      buff = (uint8_t*)malloc(buff_size);
      if(buff == NULL){
        log_e("malloc error");
        if(bytes_written != NULL){ *bytes_written = 0; }
        return;
      }
      for(int i = 0; i < size/2; i += 2 ){
        ((uint16_t*)buff)[i]   = ((uint16_t*)output)[i+1]; // [1] <- [0]
        ((uint16_t*)buff)[i+1] = ((uint16_t*)output)[i]; // [0] <- [1]
      }
    break;
    case 24:
      buff = (uint8_t*)output;
      break;
    case 32:
      buff = (uint8_t*)output;
      break;
    default: ; // Do nothing
  } // switch

  size_t _bytes_written;
  esp_err_t ret = esp_i2s::i2s_write((esp_i2s::i2s_port_t) _deviceIndex, buff, buff_size, &_bytes_written, 0); // fixed
  if(ret != ESP_OK){
    log_e("Error: writing data to i2s - function returned with err code %d", ret);
  }
  if(ret == ESP_OK && buff_size != _bytes_written){
    log_w("Warning: writing data to i2s - written %d B instead of requested %d B", _bytes_written, buff_size);
  }
  // free if the buffer was actually allocated
  if(_bitsPerSample == 8 || _bitsPerSample == 16){
    free(buff);
  }
  if(bytes_written != NULL){
    *bytes_written = _bitsPerSample == 8 ? _bytes_written/2 : _bytes_written;
  }
  if(actual_bytes_written != NULL){
    *actual_bytes_written = _bytes_written;
  }
}

I2SClass I2S(I2S_DEVICE, I2S_CLOCK_GENERATOR); // default - half duplex

//--------------------------------------------------------------------------------------------------------------------------------//

#define MP1 (4-1)
#define MP2 (5-1)
#define MP3 (3-1)
#define MP4 (6-1)
#define MP5 (2-1)
#define MP6 (7-1)
#define MP7 (1-1)
#define MP8 (8-1)

int my_buffer_len = 128; // 128frames * 2bytes a frame
uint16_t *my_buffer;
uint8_t step = 0;

//values -128 to 128
//          MotorPort:   4    5  3  6  2   7  1 8
int16_t NoU_I2S_motorsPower[] = {0,0,0,0,0,0,0,0};

//bufferIndex is a value between 0 and 127
uint16_t generateFrame(uint16_t bufferIndex, int16_t* NoU_I2S_motorsPower){
  uint16_t frame = 0x0000;

  for (uint16_t motorIndex = 0; motorIndex < 8; motorIndex++) {
    int16_t motorPower = NoU_I2S_motorsPower[motorIndex];

    if ( (motorPower > 0) && (bufferIndex < motorPower)    )  frame |= 0b00000001 << (2*motorIndex);
    if ( (motorPower < 0) && (bufferIndex < motorPower*-1) )  frame |= 0b00000010 << (2*motorIndex);
  } 
  return frame;
}

void NoU_I2S_Begin() {
  my_buffer = (uint16_t*) malloc(my_buffer_len*2); //*2 cause there are 2 bytes in a 16bit frame

  //I2S.setAllPins(4,5,-1,6,-1); // you can change default pins; order of pins = (sckPin, fsPin (word), sdPin, outSdPin, inSdPin)
  //SRCLK, RCLK, -1. SER, -1
  I2S.setAllPins(12,13,-1,14,-1); // you can change default pins; order of pins = (SCK, CS, -1, MOSI, -1)
  I2S.begin();

  NoU_I2S_Update();
}

void NoU_I2S_Update(){
  for (uint16_t i = 0; i < my_buffer_len/2; i++) {
    my_buffer[2*i] = generateFrame(2*i+1, NoU_I2S_motorsPower);
    my_buffer[2*i+1] = generateFrame(2*i, NoU_I2S_motorsPower);
  } 
  I2S.write_nonblocking((void*)my_buffer, my_buffer_len*2);
}

void NoU_I2S_SetMotor(uint8_t motorPort, int16_t motorPower){
	if (motorPower > 128) motorPower = 128;
	if (motorPower < -128) motorPower = -128;
	
	if (motorPort == 1) {NoU_I2S_motorsPower[MP1] = motorPower; Serial.println(motorPower); }
	if (motorPort == 2) NoU_I2S_motorsPower[MP2] = motorPower;
	if (motorPort == 3) NoU_I2S_motorsPower[MP3] = motorPower;
	if (motorPort == 4) NoU_I2S_motorsPower[MP4] = motorPower;
	if (motorPort == 5) NoU_I2S_motorsPower[MP5] = motorPower;
	if (motorPort == 6) NoU_I2S_motorsPower[MP6] = motorPower;
	if (motorPort == 7) NoU_I2S_motorsPower[MP7] = motorPower;
	if (motorPort == 8) NoU_I2S_motorsPower[MP8] = motorPower;
}
