// driftosc
//  check drift on various ZERO clock source
// after reset and startup() GEN clock 0, 1 48mhz, 2 ULPOSC32k, 3 OSC8M
//   other OSC32k
//  use free running TC4 clock, could use ISR/compare and tick like Tone.cpp
// 32-bit counter requires even/odd pair
//  hostdrift -f 32768   8388608  50331648
// https://github.com/manitou48/crystals
#define LED_PIN 13

void clkinit() {
#if 0
	// optionally create a new CLK4 with clock source either OSC32K or XOSC32K
	//XOSC32K is started by startup(), start OSC32K
   uint32_t calib = (*((uint32_t *) SYSCTRL_FUSES_OSC32K_ADDR) & SYSCTRL_FUSES_OSC32K_Msk) >> SYSCTRL_FUSES_OSC32K_Pos;
	SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_CALIB(calib) | SYSCTRL_OSC32K_STARTUP( 0x6u ) |
	 SYSCTRL_OSC32K_EN32K | SYSCTRL_OSC32K_ENABLE;
	while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC32KRDY) == 0 );
	GCLK->GENDIV.reg = GCLK_GENDIV_ID( 4) ;
	   // select  SRC OSC32K or XOSC32K below
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 4 ) | GCLK_GENCTRL_SRC_OSC32K | 
	  GCLK_GENCTRL_GENEN ;
	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );  // spin sync
#endif

	// Enable GCLK for TC4 and TC5  GCLK0 2 3 ready
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
	while (GCLK->STATUS.bit.SYNCBUSY);

	TC4->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
	while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
	TC4->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
	while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
	while (TC4->COUNT32.CTRLA.bit.SWRST);

	TC4->COUNT32.CTRLA.reg |= TC_CTRLA_MODE_COUNT32;
	while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
	TC4->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
}

void setup() {
  Serial.begin(9600);
  clkinit();
}

void display() {
  uint32_t t;
  static uint32_t prev=0;

  t = TC4->COUNT32.COUNT.reg;
  Serial.print(t); Serial.print(" ");
  Serial.println(t-prev);
  prev=t;
  delay(1000);
}

void logger(){
    static long cnt=0;
    uint32_t t;

    while(Serial.available() < 4);   // wait for host request
    t = TC4->COUNT32.COUNT.reg;
    Serial.read();
    Serial.read();
    Serial.read();
    Serial.read();
    Serial.write((uint8_t *)&t,4);
    cnt++;
    digitalWrite(LED_PIN, cnt & 1);
}

void loop() {
	 display();
//logger();
}

