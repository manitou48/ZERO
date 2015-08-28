// ZERO demo of IR remote xmit recv
//  version 2 use analogWrite() to set clock,timer,pin
//   Sony remote tests
//  TC5 50us timer and ISR and use any pin
//  pin 12 TCC0 ch3 PORTA 19 PIO_TIMER_ALT pwm(khz)  need PWM out pin
// variant.cpp  PORTA group 0
// 48m/38k  1263 < 16 bits
// can run both xmit/recv at once
// transmit with IR LED, recvr GP1UX311QS
// ref https://github.com/z3t0/Arduino-IRremote

#include "IR_remote.h"

#define RECVPIN 7
#define PWMPIN 12
#define TCCx TCC0
#define TCCchannel 3

#define TCx TC5
#define TCx_Handler TC5_Handler
#define TCx_IRQn TC5_IRQn
#define TCchannel 0
#define TCGCLK GCM_TC4_TC5

#define syncTCC while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
#define syncTC  while (TCx->COUNT16.STATUS.bit.SYNCBUSY)

int rawbuf[RAWBUF], rawlen;
uint8_t rcvstate;
int results_decode_type; // NEC, SONY, RC5, UNKNOWN
unsigned long results_value;
int results_bits; // Number of bits in decoded value



void pwm_init(int khz) {
	uint32_t cc;

	analogWrite(PWMPIN,0);  // init clock,timer,pin, and zero
	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
	syncTCC;
	// set period and duty cycle
	cc = F_CPU/(khz*1000) - 1;
	TCCx->CC[TCCchannel].reg = cc/2;  // duty 50%
	syncTCC;
	TCCx->PER.reg = cc;               // period
	syncTCC;
}

void enable_pwm() {
	TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
	syncTCC;
}

void disable_pwm() {
	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
	syncTCC;
}


void tick_init() {
	// 50us ticks  20khz from 48mhz clock
	uint16_t cc;

	cc = F_CPU/(1000000/USECPERTICK) - 1;   // 20khz
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(TCGCLK));
	while (GCLK->STATUS.bit.SYNCBUSY);

	// reset
	TCx->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
	syncTC;
	TCx->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while (TCx->COUNT16.CTRLA.bit.SWRST);

	TCx->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
	TCx->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	TCx->COUNT16.CC[TCchannel].reg = cc;
	syncTC;

	NVIC_EnableIRQ(TCx_IRQn);
	TCx->COUNT16.INTENSET.bit.MC0 = 1;  // enable interrupt
	TCx->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;  // enable
	syncTC;
}


unsigned long myticks;

void TCx_Handler() {
	uint8_t irdata = (uint8_t)digitalRead(RECVPIN);
	myticks++;
	TCx->COUNT16.INTFLAG.bit.MC0 = 1;  // clear interrupt
  if (rawlen >= RAWBUF) {
    // Buffer overflow
    rcvstate = STATE_STOP;
  }
  switch(rcvstate) {
  case STATE_IDLE: // In the middle of a gap
    if (irdata == MARK) {
      if (myticks < GAP_TICKS) {
        // Not big enough to be a gap.
        myticks = 0;
      }
      else {
        // gap just ended, record duration and start recording transmission
        rawlen = 0;
        rawbuf[rawlen++] = myticks;
        myticks = 0;
        rcvstate = STATE_MARK;
      }
     }
    break;
  case STATE_MARK: // timing MARK
    if (irdata == SPACE) {   // MARK ended, record time
      rawbuf[rawlen++] = myticks;
      myticks = 0;
      rcvstate = STATE_SPACE;
    }
    break;
  case STATE_SPACE: // timing SPACE
    if (irdata == MARK) { // SPACE just ended, record it
      rawbuf[rawlen++] = myticks;
      myticks = 0;
      rcvstate = STATE_MARK;
    }
    else { // SPACE
      if (myticks > GAP_TICKS) {
        // big SPACE, indicates gap between codes
        // Mark current code as ready for processing
        // Switch to STOP
        // Don't reset timer; keep counting space width
        rcvstate = STATE_STOP;
      }
    }
    break;
  case STATE_STOP: // waiting, measuring gap
    if (irdata == MARK) { // reset gap timer
      myticks = 0;
    }
    break;
  }

}

void enableIROut(int khz) {
	pwm_init(khz);
}

void enableIRIn() {
	tick_init();
	rcvstate = STATE_IDLE;
	rawlen = 0;
	pinMode(RECVPIN, INPUT);
}

void irrecv_resume() {
    rcvstate = STATE_IDLE;
    rawlen = 0;
}


void mark(int time) {
	//TIMER_ENABLE_PWM;
	enable_pwm();
	if (time > 0) delayMicroseconds(time);
}

void space(int time){
	//TIMER_DISABLE_PWM;
	disable_pwm();
	if (time > 0) delayMicroseconds(time);
}

long decodeSony() {
  long data = 0;
  if (rawlen < 2 * SONY_BITS + 2) {
    return IRERR;
  }
  int offset = 1; // Skip first space
  // Initial mark
  if (!MATCH_MARK(rawbuf[offset], SONY_HDR_MARK)) {
    return IRERR;
  }
  offset++;

  while (offset + 1 < rawlen) {
    if (!MATCH_SPACE(rawbuf[offset], SONY_HDR_SPACE)) {
      break;
    }
    offset++;
    if (MATCH_MARK(rawbuf[offset], SONY_ONE_MARK)) {
      data = (data << 1) | 1;
    }
    else if (MATCH_MARK(rawbuf[offset], SONY_ZERO_MARK)) {
      data <<= 1;
    }
    else {
      return IRERR;
    }
    offset++;
  }

  // Success
  results_bits = (offset - 1) / 2;
  if (results_bits < 12) {
    results_bits = 0;
    return IRERR;
  }
  results_value = data;
  results_decode_type = SONY;
  return DECODED;
}

void sendSony(unsigned long data, int nbits) {
  enableIROut(40);
  mark(SONY_HDR_MARK);
  space(SONY_HDR_SPACE);
  data = data << (32 - nbits);
  for (int i = 0; i < nbits; i++) {
    if (data & TOPBIT) {
      mark(SONY_ONE_MARK);
      space(SONY_HDR_SPACE);
    }
    else {
      mark(SONY_ZERO_MARK);
      space(SONY_HDR_SPACE);
    }
    data <<= 1;
  }
}

void setup() {
	pinMode(13,OUTPUT);
	Serial.begin(9600);
	enableIRIn();
}

void loop() {
  long sonycmd[] = {0xA9A,0x91A,0x61A}; // power 0 7
  long cnt;

  Serial.println(" xmit");
  digitalWrite(13,HIGH);
  sendSony(sonycmd[0],SONY_BITS);
  digitalWrite(13,LOW);
  delay(6);   // let gap time grow

  if (rcvstate == STATE_STOP) {
    if (decodeSony() ) {
        char str[128];
        sprintf(str,"sony decoded. value %0x  %d bits",results_value, results_bits);
        Serial.println(str);
    }
    Serial.print("rawlen "); Serial.println(rawlen);
    for (int i=0; i < rawlen; i++) {
        if (i%2) Serial.print(" ");
        Serial.println(rawbuf[i]*USECPERTICK);
        }
    irrecv_resume();
  }

  delay(2000);
}

