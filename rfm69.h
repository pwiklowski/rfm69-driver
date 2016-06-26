#ifndef RFM69_H_
#define RFM69_H_


#include <inttypes.h>


#define RFM69_MAX_PAYLOAD		63 ///< Maximum bytes payload

typedef enum
{
  RFM69_MODE_SLEEP = 0,
  RFM69_MODE_STANDBY,
  RFM69_MODE_FS,
  RFM69_MODE_TX,
  RFM69_MODE_RX
} Rfm69Mode;

typedef enum
{
  RFM69_DATA_MODE_PACKET = 0,                 //!< Packet engine active
  RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC = 2,   //!< Continuous mode with clock recovery
  RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC = 3,//!< Continuous mode without clock recovery
} Rfm69DataMode;

class Rfm69
{
public:
  Rfm69();
  virtual ~Rfm69();

  void reset();
  bool init();
  void setFrequency(unsigned int frequency);
  void setFrequencyDeviation(unsigned int frequency);
  void setBitrate(unsigned int bitrate);
  void setMode(Rfm69Mode mode);
  void setPowerLevel(uint8_t power);
  int setPowerDBm(int8_t dBm);

  bool setConfig(const uint8_t config[][2], unsigned int length);
  int receive(uint8_t* data, unsigned int dataLength);
  void sleep();
  int getRSSI()
  {
    return _rssi;
  }

  void setOOKMode(bool enable);

  void setDataMode(Rfm69DataMode dataMode = RFM69_DATA_MODE_PACKET);

  /**
   * Enable/disable the automatic reading of the RSSI value during packet reception.
   *
   * Default is off (no reading).
   *
   * @param enable true or false
   */
  void setAutoReadRSSI(bool enable)
  {
    _autoReadRSSI = enable;
  }

  /**
   * Enable/disable the CSMA/CA (carrier sense) algorithm before sending a packet.
   *
   * @param enable true or false
   */
  void setCSMA(bool enable)
  {
    _csmaEnabled = enable;
  }



  bool setAESEncryption(const void* aesKey, unsigned int keyLength);
  int read(uint8_t* data, unsigned int dataLength, uint8_t* sequence);

  void waitForModeReady();

  bool isFifoNotEmpty();
  bool isPacketReady();
  bool isFifoThreshold();

  int readPacket(uint8_t* data, unsigned int dataLength);
  int sendPacket(uint8_t* data, uint16_t len);

  int receivePacket(uint8_t* buf, uint16_t maxSize);

  int send(uint8_t* data, unsigned int dataLength, uint8_t sequence);
  int sendWithAck(uint8_t* data, uint16_t len, uint8_t sequence);

  uint8_t readRegister(uint8_t reg);

  void writeRegister(uint8_t reg, uint8_t value);
private:
  void clearFIFO();


  void waitForPacketSent();

  int readRSSI();

  bool isChannelFree();

  Rfm69Mode m_mode;

  uint8_t m_powerLevel;
  int _rssi;
  bool _autoReadRSSI;
  bool _ookEnabled;
  Rfm69DataMode _dataMode;

  bool _csmaEnabled;
  uint8_t _rxBuffer[RFM69_MAX_PAYLOAD];
  unsigned int _rxBufferLength;


};

#endif
