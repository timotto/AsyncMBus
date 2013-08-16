#include "AsyncMbus.h"

#define TX_OFF LOW
#define TX_ON  HIGH

static AsyncMBus *_aMbus = 0;
static void _asyncMBusTimerISR();
static void _asyncMBusPinChangeISR();
static inline int bits(const uint32_t msg);

AsyncMBus::AsyncMBus(int rxPin, int txPin, t_onMessageCb onMessageCb, t_onTxCb onTxCb, t_onFaultCb onFaultCb) 
  : lastLevel(-1), rxPin(rxPin), txPin(txPin), _messageCb(onMessageCb), _txCb(onTxCb), _faultCb(onFaultCb)
   {
  if (_aMbus)
    // it already is a class but uses static callback methods for timer and ISR. how do I change that
    // to be able to run multiple instances of AsyncMBus?
    fatal("unfortunately there can be only one MBus instance. Hack the source to make more of them");
    
    
  _aMbus = this;

  goIdle();
  
  pinMode(rxPin, INPUT); 
  digitalWrite(rxPin, LOW); // no internal pull-up
  pinMode(txPin, OUTPUT); 
  digitalWrite(txPin, TX_OFF); // initial state is shut up
}

void AsyncMBus::setup() {
// different on Due, probably Teensy 3.0 and others
  int intr;
#ifdef __AVR__
  switch(rxPin) {
    case 2:
      intr = 1;
      break;
    case 3:
      intr = 0;
      break;
    default:
      fatal("unsupported RX pin, choose 2 or 3 or change the code");
      intr = 0; // to silence warning
      break;
  }
#elif __arm__
    intr = rxPin;
#else
  fatal("unsupported architecture");
  intr = 0; // to silence warning
#endif
  Timer1.initialize(AMBUS_TIMER_USEC);

  attachInterrupt(intr, _asyncMBusPinChangeISR, CHANGE);
  Timer1.attachInterrupt(_asyncMBusTimerISR);
}

void AsyncMBus::loop() {
  switch(state) {
    case RX_BEGIN:
      // RX pin is still ON
      if (ticks == -1) {
        // but all time is used up. bus wired to VCC or pull-up too damn high or timing error
        goIdle();
        onFault(BUS_OFF, 1);
      }
      break;
    case RX_WAIT:
      if (ticks == -1) {
        // end of message. There's user code in onMessage, so this does not happen in timer ISR
		uint64_t rxMsg = msg;
		int rxLen = len;
        goIdle();
        onMessage(rxMsg, rxLen);
      }
    default:
      return;
  }
}

bool AsyncMBus::tx(uint64_t msg) {
  if (state != RX_IDLE)
    return false;
  
  // TODO 64 bit safe?
  len = 0;
  uint64_t x = msg;
  while (x > 0) {
    x /= 8;
    len += 4;
  }
  
  this->msg = msg;
  this->len = len + 4;
  this->msg <<= 4;
  this->msg |= parity(this->msg);
  state = TX_IDLE;
  
  Serial.print("[TX] ");
  if (msg > 0xffffffffull) {
    Serial.print((uint32_t)(msg >> 32), HEX);
  }
  Serial.println((uint32_t)(msg), HEX);
  return true;
}

volatile int lastLevel = -1;
void AsyncMBus::pinChangeISR() {
  int level = digitalRead(rxPin);
  if (level==lastLevel)return;
  lastLevel=level;
//  if(level)
//    Serial.println("{hi}");
//  else
//    Serial.println("{lo}");
    
  switch(level) {
    case HIGH:
      switch(state) {
        case TX_ONE:
        case TX_ZERO:
//          Serial.println("{ack}");
          ack = true;
          // this is expected. Nobody expectes the American inquisition.
          break;

        // pulse started during TX: We're overwritten, stop sending
        // TODO retry
        case TX_END:
        case TX_IDLE:
//          Serial.println("{txerr}");
		  onTx(false);
          goIdle();
          onFault(TX_OVERRIDE, 0);
          break;
          // no break
          
        case RX_WAIT:
//          Serial.println("{wait}");
//          if (ticks > AMBUS_TICKS_SLOPE)
//            onFault(EARLY_PULSE, ticks);
          state = RX_BEGIN;
          ticks = AMBUS_TICKS_TOTAL + (2 * AMBUS_TICKS_SLOPE);
          break;
          
        case RX_IDLE:
//          Serial.println("{idle}");
          state = RX_BEGIN;
          ticks = AMBUS_TICKS_TOTAL + (2 * AMBUS_TICKS_SLOPE);
          msg = 0;
          len = 0;
          break;
          
        default:
          onFault(RX_STATE_MISMATCH_HIGH, state);
          goIdle();
          break;
      }
      return;
      
    case LOW:
      switch(state) {
        case TX_END:
        case TX_IDLE:
          // expected
          ack = true;
//          Serial.println("{ackL}");
          break;
          
        // pulse stopped during TX: We're overwritten, stop sending
        // TODO retry
        case TX_ONE:
        case TX_ZERO:
          digitalWrite(txPin, TX_OFF);
		  onTx(false);
          goIdle();
          onFault(TX_OVERRIDE, 1);
          break;
          
        case RX_BEGIN:
        {
//          Serial.println("{rx1}");
          // pin went low after high. this is a pulse!
          // ticks contains 'ticks left'
          // tocks the ticks that happened
          int tocks = AMBUS_TICKS_TOTAL + ( 2 * AMBUS_TICKS_SLOPE ) - ticks;
          if ( tocks < (AMBUS_TICKS_ZERO - AMBUS_TICKS_SLOPE)) {
            onFault(SHORT_PULSE, tocks);
//            Serial.println("{rxF1}");
          } else if (tocks <= (AMBUS_TICKS_ZERO + AMBUS_TICKS_SLOPE)) {
            // received 0
            msg <<= 1;
            len++;
//            Serial.println("{rx_0}");
          } else if (tocks < (AMBUS_TICKS_ONE - AMBUS_TICKS_SLOPE)) {
            // between 6(8) and 18(16) * 100 usecs
            onFault(WICKED_PULSE, tocks);
//            Serial.println("{rxF2}");
          } else if (tocks <= (AMBUS_TICKS_ONE + AMBUS_TICKS_SLOPE)) {
            // received 1
            msg = (msg << 1) | 1;
            len++;
//            Serial.println("{rx_1}");
          } else {
            onFault(LONG_PULSE, tocks);
//            Serial.println("{rxF3}");
          }
          state = RX_WAIT;
          if (len > AMBUS_MAX_MSG_LEN) {
            goIdle();
            onFault(MSG_OVERFLOW, len);
          }
          break;
        }
        
        default:
          onFault(RX_STATE_MISMATCH_LOW, state);
          goIdle();
          break;
          
      }
      break;
  }
}

void AsyncMBus::timerISR() {
  if (ticks != -1)
    ticks--;
  
  switch(state) {
    case TX_END:
      if (ticks > 0)
        break;
        
//      Serial.println("{iEND}");
      if (!ack) {
        onFault(TX_PIN_MISMATCH_LOW, 0);
        // TODO canel? retry? slope?
      }

      if (len == 0) {
	    onTx(true);
        goIdle(); // was the last bit, ready to receive again
        break;
      } else 
        state = TX_IDLE; // there are bits left
        // and no break
    case TX_IDLE:
//      Serial.println("{iIDLE}");
      // tx was requested, this->msg contains the message, len the length.
      state = ((msg >> --len) & 1)?TX_ONE:TX_ZERO;
      ticks = AMBUS_TICKS_ZERO;
      ack = false;
	  //Serial.print(">");Serial.print(state);Serial.print(" ");
      digitalWrite(txPin, TX_ON);
      break;

    case TX_ZERO:
    case TX_ONE:
      if (ticks > 0)
        break;
        
//      Serial.println("{i0/1}");
      if (!ack) {
		onTx(false);
        goIdle();
        onFault(BUS_OFF, TX_PIN_MISMATCH_HIGH);
      } else {
        if (state == TX_ZERO)
          ticks = AMBUS_TICKS_TOTAL - AMBUS_TICKS_ZERO; // (300 us total = 240us)
        else // TX_ONE
          ticks = AMBUS_TICKS_TOTAL - AMBUS_TICKS_ONE; // (300 us total = 240us)
        state = TX_END; // wait for the end of the pulse duration 
      }
      digitalWrite(txPin, LOW);
      break;
      
    default:
      break;
  }
}

uint8_t AsyncMBus::parity(uint64_t m) {
  uint8_t p = 0;
  
  for(int i=4;i<64;i+=4)
    p ^= (m >> i ) & 0xf;

  return (1+p)%0x10;
}

bool AsyncMBus::parityOk() {
  return parity(msg) == (msg & 0xf);
}

void AsyncMBus::onMessage(uint64_t msg, int len) {
  
  if (len < 4) {
    onFault(MESSAGE_LENGTH, len);
    return;
  }
  
  uint8_t pa = parity(msg);
  if (pa != (msg&0xf)){
    onFault(PARITY, pa<<4 | (msg&0xf));
//    return;
  }
  
  msg >>= 4;
  len -= 4;
  
  if(!_messageCb)
    return;
  
  _messageCb(msg, len);
}

void AsyncMBus::onTx(bool success) {
	if(_txCb)
		_txCb(msg, success);
}

void AsyncMBus::onFault(const Fault fault, int arg) {
  if (_faultCb)
    _faultCb(fault, arg);
}

void AsyncMBus::fatal(const char *msg) {
  Serial.println(msg);
  while(1);
}

static void _asyncMBusPinChangeISR() {
  _aMbus->pinChangeISR();
}

static void _asyncMBusTimerISR() {
  _aMbus->timerISR();
}

static inline int bits(const uint32_t msg) {
  for(int b=4;b<32;b+=4)
    if (msg < ((uint32_t)1<<b))
      return b;

  return 32;
}

int bits(const uint64_t msg) {
  if (msg > 0xffffffffull) {
    return 32 + bits((const uint32_t)(msg >> 32));
  } else {
    return bits((const uint32_t)msg);
  }
}


