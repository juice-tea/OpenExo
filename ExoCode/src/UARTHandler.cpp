#include "UARTHandler.h"
#include "Utilities.h"
#include "Logger.h"
#include "ParseIni.h"

#define MAX_NUM_LEGS 2
#define MAX_NUM_JOINTS_PER_LEG 2 //Current PCB can only do 2 motors per side, if you have made a new PCB, update.
#define UART_DATA_TYPE short int //If type is changes you will need to comment/uncomment lines in pack_float and unpack_float
#define FIXED_POINT_FACTOR 100

// Nano -> Teensy sends raw floats; Teensy -> Nano stays fixed-point ints.
#if defined(ARDUINO_ARDUINO_NANO33BLE) || defined(ARDUINO_NANO_RP2040_CONNECT)
#define UART_PACK_FLOATS 1
#define UART_UNPACK_FLOATS 0
#else
#define UART_PACK_FLOATS 0
#define UART_UNPACK_FLOATS 1
#endif

//Set to 1 to enable debug prints
#define DEBUG_UART_HANDLER 0

typedef enum 
{
  COMMAND = 0,
  JOINT_ID = 1,
  DATA_START = 2
} UARTPackingIndex;

static const uint8_t UART_CMD_UPDATE_REAL_TIME_DATA = 0x12;
static const uint8_t UART_TYPED_RT_FLOAT_COUNT = 11;
static const uint8_t UART_TYPED_RT_PAYLOAD_LEN = 16;
static const uint8_t UART_CMD_UPDATE_CONFIG = 0x06;


UARTHandler::UARTHandler()
{
  //_rx_raw = CircularBuffer_<char>(_rx_raw_buffer, _k_bufferSize);
  MY_SERIAL.begin(UART_BAUD);
  MY_SERIAL.setTimeout(0);
}

UARTHandler* UARTHandler::get_instance()
{
    static UARTHandler* instance = new UARTHandler();
    return instance;
}

void UARTHandler::UART_msg(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *buffer)
{
    uint8_t _packed_len = _get_packed_length(msg_id, len, joint_id, buffer);

    #if DEBUG_UART_HANDLER
    logger::print("UARTHandler::UART_msg->Packing Bytes: "); logger::println(_packed_len);
    #endif

    uint8_t _byte_data[_packed_len] = {0};
    _pack(msg_id, len, joint_id, buffer, _byte_data);

    #if DEBUG_UART_HANDLER
   logger::println("UARTHandler::UART_msg->Packed data:");
   for (int i=0; i<_packed_len; i++)
   {
     logger::print(_byte_data[i]); logger::print(", ");
   }
   logger::println();
    #endif
    _send_packet(_byte_data, _packed_len);
    MY_SERIAL.flush();

    #if DEBUG_UART_HANDLER
   logger::println("UARTHandler::UART_msg->Flushed tx buffer");
    #endif
}

void UARTHandler::UART_msg(UART_msg_t msg)
{
    #if DEBUG_UART_HANDLER
        logger::print("UARTHandler::UART_msg->Sending Message");
        UART_msg_t_utils::print_msg(msg);
    #endif

    UART_msg(msg.command, msg.len, msg.joint_id, msg.data);
}

UART_msg_t UARTHandler::poll(float timeout_us)
{
    static UART_msg_t empty_msg = {0, 0, 0, 0};
    _timeout_us = timeout_us;
    
    uint32_t _available_bytes = check_for_data();
    if (!_available_bytes) {return empty_msg;}

    #if DEBUG_UART_HANDLER
        logger::print("UARTHandler::poll->Bytes Available: "); logger::println(_available_bytes);
    #endif

    uint8_t _msg_buffer[MAX_RX_LEN];
    int _recv_len = _recv_packet(_msg_buffer, MAX_RX_LEN);
    
    if (_recv_len > 0)
    {
      //Add the partial data to the message
      if (_partial_packet_len) 
      {
        //This occurs if there was a timeout during _recv_packet and we have a complete message
        #if DEBUG_UART_HANDLER
            logger::println("UARTHandler::poll->_recv_len > 0 && _partial_packet_len");
        #endif

        //Shift _msg_buffer _partial_packet_len bytes to fit the previous partial packet using memmove
        memmove(_msg_buffer + _partial_packet_len, _msg_buffer, _recv_len);
        
        //Copy the partial packet to the beginning of the buffer
        memcpy(_msg_buffer, _partial_packet, _partial_packet_len);

        _recv_len += _partial_packet_len;

       _reset_partial_packet();
     }

      uint8_t validated_len = (uint8_t)_recv_len;
    #if UART_USE_CRC
      if (!_verify_crc(_msg_buffer, validated_len))
      {
        return empty_msg;
      }
    #endif

      UART_msg_t msg = _unpack(_msg_buffer, validated_len);

      #if DEBUG_UART_HANDLER
          logger::print("UARTHandler::poll->Got Message: ");
          UART_msg_t_utils::print_msg(msg);
      #endif

      return msg;
    }

    if (_recv_len < 0)
    {
      //This only occurs if there was a timeout during the previous _recv_packet and an end flag before any new data, in this case the partial packet is the full message
      #if DEBUG_UART_HANDLER
        logger::println("UARTHandler::poll->_recv_len < 0");
      #endif

      //Append the _partial packet to the full message
      memcpy(_msg_buffer, _partial_packet, _partial_packet_len);
       
      #if DEBUG_UART_HANDLER
        logger::println("UARTHandler::poll->_msg_buffer after copyting _packed_data: ");
          for (int i=0; i<(_partial_packet_len); i++)
          {
            logger::print(_msg_buffer[i]); logger::print(", ");
          }
          logger::println();
      #endif

      uint8_t validated_len = _partial_packet_len;
    #if UART_USE_CRC
      if (!_verify_crc(_msg_buffer, validated_len))
      {
        _reset_partial_packet();
        return empty_msg;
      }
    #endif

      UART_msg_t msg = _unpack(_msg_buffer, validated_len);

      _reset_partial_packet();

      #if DEBUG_UART_HANDLER
          logger::print("UARTHandler::poll->Got Message: ");
          UART_msg_t_utils::print_msg(msg);
      #endif

      return msg;
     }
    return empty_msg;
}

inline uint8_t UARTHandler::check_for_data()
{
    return MY_SERIAL.available();
}

void UARTHandler::_pack(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *data, uint8_t *data_to_pack)
{
    //Pack metadata
    data_to_pack[COMMAND] = msg_id;
    data_to_pack[JOINT_ID] = joint_id;

#if UART_USE_TYPED_RT_PACKET
  if (_should_use_typed_rt_packet(msg_id, len))
  {
    _pack_typed_rt_payload(data, data_to_pack + DATA_START);
    return;
  }
#endif

    if (_should_use_compact_config_packet(msg_id, len))
    {
      _pack_compact_config_payload(data, len, data_to_pack + DATA_START);
      return;
    }
    
    //Pack payload with platform-specific encoding.
#if UART_PACK_FLOATS
    uint8_t _num_bytes = sizeof(float);
#else
    uint8_t _num_bytes = sizeof(UART_DATA_TYPE);
    uint8_t buf[_num_bytes];
#endif
    for (int i=0; i<len; i++)
    {
        uint8_t _offset = (DATA_START) + _num_bytes*i;
#if UART_PACK_FLOATS
        memcpy((data_to_pack + _offset), (uint8_t*)&data[i], _num_bytes);
#else
        utils::float_to_short_fixed_point_bytes(data[i], buf, FIXED_POINT_FACTOR);
        memcpy((data_to_pack + _offset), buf, _num_bytes);
#endif
    }
}

UART_msg_t UARTHandler::_unpack(uint8_t* data, uint8_t len)
{
    UART_msg_t msg;

#if UART_USE_TYPED_RT_PACKET
  if (_unpack_typed_rt_payload(data, len, msg))
  {
    return msg;
  }
#endif

    if (_unpack_compact_config_payload(data, len, msg))
    {
      return msg;
    }

    msg.command = data[COMMAND];
    msg.joint_id = data[JOINT_ID];
    float _total_len = len*sizeof(uint8_t);
    float _meta_len = sizeof(msg.command)+sizeof(msg.joint_id);
#if UART_UNPACK_FLOATS
    float _bytes_per = sizeof(float);
    msg.len = (uint8_t)((_total_len - _meta_len) / _bytes_per);
#else
    float _bytes_per = sizeof(UART_DATA_TYPE);
    msg.len = (uint8_t)((_total_len - _meta_len) / _bytes_per);
#endif

    //Fill msg.data, converting the payload to floats as needed.
    for (int i=0; i<msg.len; i++)
    {
        uint8_t _data_offset = (DATA_START) + (i * (uint8_t)_bytes_per);
#if UART_UNPACK_FLOATS
        float tmp = 0;
        memcpy(&tmp, (uint8_t*)data + _data_offset, sizeof(float));
        msg.data[i] = tmp;
#else
        float tmp = 0;
        utils::short_fixed_point_bytes_to_float((uint8_t*)data+_data_offset, &tmp, FIXED_POINT_FACTOR);
        msg.data[i] = tmp;
#endif
    }

    return msg;
}

uint8_t UARTHandler::_get_packed_length(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *data)
{
  (void)joint_id;
  (void)data;

#if UART_USE_TYPED_RT_PACKET
  if (_should_use_typed_rt_packet(msg_id, len))
  {
    return DATA_START + UART_TYPED_RT_PAYLOAD_LEN;
  }
#endif

    if (_should_use_compact_config_packet(msg_id, len))
    {
      return DATA_START + len;
    }

    uint8_t _val = 0;
#if UART_PACK_FLOATS
    _val += (float)len * sizeof(float);
#else
    //We are converting from float to short int, we must multiply by the size difference
    _val += (float)len * sizeof(UART_DATA_TYPE);
#endif
    _val += sizeof(msg_id);
    _val += sizeof(joint_id); 
    return _val;
}


void UARTHandler::_send_char(uint8_t val)
{
  #if DEBUG_UART_HANDLER
      logger::print("UARTHandler::_send_char->Sending: 0x");
      logger::println(val);
  #endif

  MY_SERIAL.write(val);
}

uint8_t UARTHandler::_recv_char(void)
{  
  uint8_t _data = MY_SERIAL.read();

  #if DEBUG_UART_HANDLER
    logger::print("UARTHandler::_recv_char->Read: "); logger::println(_data);
  #endif

  return _data;
}

/* SEND_PACKET: sends a packet of length "len", starting at location "p". */
void UARTHandler::_send_packet(uint8_t* p, uint8_t len)
{
  uint8_t* payload_start = p;
  const uint8_t payload_len = len;

  /* Send an initial END character to flush out any data that may have accumulated in the receiver due to line noise */
  _send_char(END);

  /* For each byte in the packet, send the appropriate character sequence */
  while (len--) {
    switch (*p) {
      /* If it's the same code as an END character, we send a special two character code so as not to make the receiver think we sent an END */
      case END:
        _send_char(ESC);
        _send_char(ESC_END);
        break;

      /* If it's the same code as an ESC character, we send a special two character code so as not to make the receiver think we sent an ESC */
      case ESC:
        _send_char(ESC);
        _send_char(ESC_ESC);
        break;

      /* Otherwise, we just send the character */
      default:
        //logger::print("UARTHandler::_send_packet->Sending: 0x"); logger::println(*p);
        _send_char(*p);
    }

    p++;
  }

#if UART_USE_CRC
  uint8_t crc = _crc8(payload_start, payload_len);

  switch (crc)
  {
    case END:
      _send_char(ESC);
      _send_char(ESC_END);
      break;
    case ESC:
      _send_char(ESC);
      _send_char(ESC_ESC);
      break;
    default:
      _send_char(crc);
      break;
  }
#endif

  /* Tell the receiver that we're done sending the packet */
  _send_char(END);
}

/* RECV_PACKET: receives a packet into the buffer located at "p".
           If more than len bytes are received, the packet will
           be truncated.
           Returns the number of bytes stored in the buffer.
*/
int UARTHandler::_recv_packet(uint8_t *p, uint8_t len)
{
  uint8_t c;
  int received = 0;
  int bytes_left;
  
  _time_left(1);
  while (_time_left())
  {
    if (!check_for_data()) 
    {
      continue;
    }

    c = _recv_char();

    #if DEBUG_UART_HANDLER
        logger::print("UARTHandler::_recv_packet->Got char: ");
        logger::println(c);
    #endif

    //Handle bytestuffing if necessary
    switch (c) 
    {

      //If it's an END character then we're done with the packet
      case END:
        #if DEBUG_UART_HANDLER
            logger::println("UARTHandler::_recv_packet->END CASE");
        #endif

        if (received)
        {
            #if DEBUG_UART_HANDLER
                logger::print("UARTHandler::_recv_packet->Returning: ");
                logger::println(received);
            #endif

            return received;
        }
        else if (_partial_packet_len)
        {
            #if DEBUG_UART_HANDLER
                logger::print("UARTHandler::_recv_packet->Returning because of _partial_packet: ");
                logger::println(_partial_packet_len);
            #endif

            return -1;
        }
        else
        {   
            break;
        }

      /* If it's the same code as an ESC character, wait and get another character and then figure out what to store in the packet based on that. */
      case ESC:
        c = _recv_char();

        #if DEBUG_UART_HANDLER
            logger::println("UARTHandler::_recv_packet->ESC CASE");
            logger::print("UARTHandler::_recv_packet->ESC Char: ");
            logger::println(c);
        #endif

        /* If "c" is not one of these two, then we have a protocol violation.  The best bet seems to be to leave the byte alone and just stuff it into the packet */
        switch (c) 
        {
          case ESC_END:
              #if DEBUG_UART_HANDLER
                logger::println("UARTHandler::_recv_packet->ESC_END");
              #endif
            c = END;
            break;
          case ESC_ESC:
              #if DEBUG_UART_HANDLER
                logger::println("UARTHandler::_recv_packet->ESC_ESC");
              #endif

            c = ESC;
            break;
        }

      default:
        #if DEBUG_UART_HANDLER
            logger::println("UARTHandler::_recv_packet->Default CASE");
        #endif

        if (received < len)
        {
          #if DEBUG_UART_HANDLER
            logger::println("UARTHandler::_recv_packet->Added to buffer");
          #endif

          p[received++] = c;
        }
    }
  }

 //There was a timeout before a full message was recieved, save the data to be reconstructed later
  int prior_packet_len = _partial_packet_len;
  _partial_packet_len += received;

  if (_partial_packet_len > MAX_RX_LEN)
  {
    _reset_partial_packet();
    return 0;
  }

  #if DEBUG_UART_HANDLER
      logger::println("UARTHandler::_recv_packet->Timeout!");
      logger::print("UARTHandler::_recv_packet->Saved Bytes: "); 
      logger::print("Prior Packet Length: "); 
      logger::print(prior_packet_len);
      logger::print("\t");
      logger::print("Received: ");
      logger::print(received);
      logger::print("\t");
      logger::print("Partial Packet Length: ");
      logger::println(_partial_packet_len);
  #endif

  for (int i=0; i<received; i++)
  {
    _partial_packet[i+prior_packet_len] = p[i];

    #if DEBUG_UART_HANDLER
        logger::print(_partial_packet[i]); logger::println(", ");
    #endif
  }

  #if DEBUG_UART_HANDLER
    logger::println();
  #endif

  return 0;
}


uint8_t UARTHandler::_time_left(uint8_t should_latch)
{
    static float _start_time;
    if (should_latch)
    {
      _start_time = micros();

      #if DEBUG_UART_HANDLER
          logger::print("UARTHandler::_time_left->Latching on ");
          logger::println(_start_time);
      #endif
    }
    float del_t = micros() - _start_time;

    return (del_t <= _timeout_us);
}

void UARTHandler::_reset_partial_packet()
{
  memset(_partial_packet, 0, _partial_packet_len);
  _partial_packet_len = 0;
}

bool UARTHandler::_should_use_typed_rt_packet(uint8_t msg_id, uint8_t len)
{
  return (msg_id == UART_CMD_UPDATE_REAL_TIME_DATA) && (len == UART_TYPED_RT_FLOAT_COUNT);
}

bool UARTHandler::_should_use_compact_config_packet(uint8_t msg_id, uint8_t len)
{
  return (msg_id == UART_CMD_UPDATE_CONFIG) && (len == (uint8_t)ini_config::number_of_keys);
}

uint8_t UARTHandler::_pack_typed_rt_payload(float *data, uint8_t *payload)
{
  auto to_i16_scaled_100 = [](float value) -> int16_t
  {
    float scaled = value * 100.0f;
    if (scaled > 32767.0f)
    {
      scaled = 32767.0f;
    }
    else if (scaled < -32768.0f)
    {
      scaled = -32768.0f;
    }
    return (int16_t)(scaled >= 0.0f ? (scaled + 0.5f) : (scaled - 0.5f));
  };

  auto clamp_u8 = [](float value, uint8_t max_val) -> uint8_t
  {
    if (value <= 0.0f)
    {
      return 0;
    }
    if (value >= (float)max_val)
    {
      return max_val;
    }
    return (uint8_t)(value + 0.5f);
  };

  auto to_bool_bit = [](float value) -> uint8_t
  {
    return value >= 0.5f ? 1 : 0;
  };

  int16_t left_setpoint = to_i16_scaled_100(data[0]);
  int16_t left_sensor = to_i16_scaled_100(data[1]);
  int16_t right_setpoint = to_i16_scaled_100(data[2]);
  int16_t right_sensor = to_i16_scaled_100(data[3]);

  payload[0] = (uint8_t)(left_setpoint & 0xFF);
  payload[1] = (uint8_t)((left_setpoint >> 8) & 0xFF);
  payload[2] = (uint8_t)(left_sensor & 0xFF);
  payload[3] = (uint8_t)((left_sensor >> 8) & 0xFF);

  payload[4] = clamp_u8(data[4], 255);

  uint8_t flags = 0;
  flags |= to_bool_bit(data[5]) << 0;
  flags |= to_bool_bit(data[7]) << 1;
  flags |= to_bool_bit(data[8]) << 2;
  payload[5] = flags;

  payload[6] = (uint8_t)(right_setpoint & 0xFF);
  payload[7] = (uint8_t)((right_setpoint >> 8) & 0xFF);
  payload[8] = (uint8_t)(right_sensor & 0xFF);
  payload[9] = (uint8_t)((right_sensor >> 8) & 0xFF);

  payload[10] = clamp_u8(data[6], 255);

  float time_ms_f = data[9] * 1000.0f;
  if (time_ms_f < 0.0f)
  {
    time_ms_f = 0.0f;
  }
  if (time_ms_f > 4294967295.0f)
  {
    time_ms_f = 4294967295.0f;
  }
  uint32_t time_ms = (uint32_t)(time_ms_f + 0.5f);

  payload[11] = (uint8_t)(time_ms & 0xFF);
  payload[12] = (uint8_t)((time_ms >> 8) & 0xFF);
  payload[13] = (uint8_t)((time_ms >> 16) & 0xFF);
  payload[14] = (uint8_t)((time_ms >> 24) & 0xFF);

  payload[15] = clamp_u8(data[10], 100);

  return UART_TYPED_RT_PAYLOAD_LEN;
}

uint8_t UARTHandler::_pack_compact_config_payload(float *data, uint8_t len, uint8_t *payload)
{
  for (uint8_t i = 0; i < len; i++)
  {
    float value_f = data[i];
    if (value_f < 0.0f)
    {
      value_f = 0.0f;
    }
    if (value_f > 255.0f)
    {
      value_f = 255.0f;
    }
    payload[i] = (uint8_t)(value_f + 0.5f);
  }

  return len;
}

bool UARTHandler::_unpack_typed_rt_payload(uint8_t *data, uint8_t len, UART_msg_t &msg)
{
  if (!_should_use_typed_rt_packet(data[COMMAND], UART_TYPED_RT_FLOAT_COUNT))
  {
    return false;
  }

  if (len != (DATA_START + UART_TYPED_RT_PAYLOAD_LEN))
  {
    return false;
  }

  uint8_t *payload = data + DATA_START;

  auto i16_to_float_scaled_100 = [](uint8_t lo, uint8_t hi) -> float
  {
    int16_t raw = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
    return ((float)raw) / 100.0f;
  };

  msg.command = data[COMMAND];
  msg.joint_id = data[JOINT_ID];
  msg.len = UART_TYPED_RT_FLOAT_COUNT;

  msg.data[0] = i16_to_float_scaled_100(payload[0], payload[1]);
  msg.data[1] = i16_to_float_scaled_100(payload[2], payload[3]);
  msg.data[2] = i16_to_float_scaled_100(payload[6], payload[7]);
  msg.data[3] = i16_to_float_scaled_100(payload[8], payload[9]);
  msg.data[4] = (float)payload[4];
  msg.data[5] = (float)((payload[5] >> 0) & 0x01);
  msg.data[6] = (float)payload[10];
  msg.data[7] = (float)((payload[5] >> 1) & 0x01);
  msg.data[8] = (float)((payload[5] >> 2) & 0x01);

  uint32_t time_ms =
      ((uint32_t)payload[11]) |
      ((uint32_t)payload[12] << 8) |
      ((uint32_t)payload[13] << 16) |
      ((uint32_t)payload[14] << 24);
  msg.data[9] = ((float)time_ms) / 1000.0f;
  msg.data[10] = (float)payload[15];

  return true;
}

bool UARTHandler::_unpack_compact_config_payload(uint8_t *data, uint8_t len, UART_msg_t &msg)
{
  if (!_should_use_compact_config_packet(data[COMMAND], (uint8_t)ini_config::number_of_keys))
  {
    return false;
  }

  const uint8_t expected_len = DATA_START + (uint8_t)ini_config::number_of_keys;
  if (len != expected_len)
  {
    return false;
  }

  msg.command = data[COMMAND];
  msg.joint_id = data[JOINT_ID];
  msg.len = (uint8_t)ini_config::number_of_keys;

  uint8_t *payload = data + DATA_START;
  for (uint8_t i = 0; i < msg.len; i++)
  {
    msg.data[i] = (float)payload[i];
  }

  return true;
}

uint8_t UARTHandler::_crc8(const uint8_t* data, uint8_t len)
{
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ 0x07;
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

bool UARTHandler::_verify_crc(uint8_t* data, uint8_t& len)
{
  if (len < 3)
  {
    return false;
  }

  const uint8_t payload_len = len - 1;
  const uint8_t received_crc = data[payload_len];
  const uint8_t computed_crc = _crc8(data, payload_len);

  if (received_crc != computed_crc)
  {
    return false;
  }

  len = payload_len;
  return true;
}
