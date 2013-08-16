#ifndef ASYNC_MBUS_H
#define ASYNC_MBUS_H

#include <Arduino.h>
#include <TimerOne.h>

int bits(const uint64_t msg);

class AsyncMBus {
public:
  typedef enum {
    NONE = 0,
    BUS_OFF = 1,
    SHORT_PULSE = 2,
    LONG_PULSE = 3,
    WICKED_PULSE = 4,
    EARLY_PULSE = 5,
    MSG_OVERFLOW = 6,
    RX_STATE_MISMATCH_LOW = 7,
    RX_STATE_MISMATCH_HIGH = 8,
    TX_STATE_MISMATCH_LOW = 9,
    TX_STATE_MISMATCH_HIGH = 10,
    TX_PIN_MISMATCH_HIGH = 11,
    TX_PIN_MISMATCH_LOW = 12,
    TX_OVERRIDE = 13,
    PARITY = 14,
    PONG_DROPPED = 15, 
    MESSAGE_LENGTH = 16
  } Fault;
  typedef void (*t_onMessageCb)(const uint64_t msg, int len);
  typedef void (*t_onTxCb)(const uint64_t msg, bool success);
  typedef void (*t_onFaultCb)(const Fault fault, int arg);
  
  AsyncMBus(int rxPin, int txPin, t_onMessageCb onMessageCb, t_onTxCb onTxCb = 0, t_onFaultCb onFaultCb = 0);
  void setup();
  void loop();
  void pinChangeISR();
  void timerISR();
  bool tx(uint64_t msg);
  bool sendPlayingTrack(uint8_t Track,uint16_t Time);
  bool sendChangedCD(uint8_t CD,uint8_t Track);
  bool sendCDStatus(uint8_t CD);

  
private:
  typedef enum {
    RX_IDLE,
    RX_BEGIN,
    RX_WAIT,
    
    TX_IDLE,
    TX_ZERO,
    TX_ONE,
    TX_END
  } State;
  volatile State state;
  volatile int lastLevel;
  volatile int32_t ticks;
  volatile uint64_t msg;
  volatile int len;
  volatile bool ack;
  int rxPin, txPin;
  
  t_onMessageCb _messageCb;
  t_onTxCb _txCb;
  t_onFaultCb _faultCb;

  uint8_t parity(uint64_t m);  
  bool parityOk();
  void onMessage(uint64_t msg, int len);
  void onTx(bool success);
  void onFault(const Fault fault, int arg);
  
  void fatal(const char *msg);
  
  void goIdle() {
    state = RX_IDLE;
    msg = 0;
    len = 0;
    ticks = -1;
  }
};

#define AMBUS_ERROR Serial.print
#define AMBUS_DEBUG Serial.print
#define AMBUS_TIMER_USEC  600
#define AMBUS_TICKS_ZERO  1
#define AMBUS_TICKS_ONE   3
#define AMBUS_TICKS_TOTAL 5
#define AMBUS_TICKS_SLOPE 1
#define AMBUS_MAX_MSG_LEN 64

#define AMBUS_HELLO       0x19
#define AMBUS_HELLO_OK    0x810
#define AMBUS_PING        0x18
#define AMBUS_PONG        0x98
#define	AMBUS_PLAY        0x11101
#define AMBUS_WAIT        0x9F00000
#define AMBUS_OFF         0x11142
#define AMBUS_CHANGING    0x9B910100001
#define AMBUS_FWD         0x11104
#define AMBUS_RWD         0x11108

#endif // ASYNC_MBUS_H
