// rng.ino   random numbers, dueling clocks entropy
// based on arduino/teensy Entropy lib
// https://sites.google.com/site/astudyofentropy/project-definition/timer-jitter-entropy-sources/entropy-library
// 48mhz system clock  GEN clock  2 ULPOSC32k, 3 OSC8M
//   other OSC32k optionally setup GCLK4
//  systick is only 24 bits
// connect TC5 to GCLK 2
// 1/32k  30us  prescale 16  488us
//  could also use WDT or RTC as timer with oscillator clocks

#define LED_PIN 13

// entropy collection
 const uint8_t gWDT_buffer_SIZE=32;
 const uint8_t WDT_POOL_SIZE=8;
 uint8_t gWDT_buffer[gWDT_buffer_SIZE];
 uint8_t gWDT_buffer_position;
 uint8_t gWDT_loop_counter;
 volatile uint8_t gWDT_pool_start;
 volatile uint8_t gWDT_pool_end;
 volatile uint8_t gWDT_pool_count;
 volatile uint32_t gWDT_entropy_pool[WDT_POOL_SIZE];

void collect() {
  //  clock tick
  gWDT_buffer[gWDT_buffer_position] = SysTick->VAL;
  gWDT_buffer_position++; 
  if (gWDT_buffer_position >= gWDT_buffer_SIZE)
  {
    gWDT_pool_end = (gWDT_pool_start + gWDT_pool_count) % WDT_POOL_SIZE;
    // The following code is an implementation of Jenkin's one at a time hash
    // This hash function has had preliminary testing to verify that it
    // produces reasonably uniform random results when using WDT jitter
    // on a variety of Arduino platforms
    for(gWDT_loop_counter = 0; gWDT_loop_counter < gWDT_buffer_SIZE; ++gWDT_loop_counter)
      {
    gWDT_entropy_pool[gWDT_pool_end] += gWDT_buffer[gWDT_loop_counter];
    gWDT_entropy_pool[gWDT_pool_end] += (gWDT_entropy_pool[gWDT_pool_end] << 10);
    gWDT_entropy_pool[gWDT_pool_end] ^= (gWDT_entropy_pool[gWDT_pool_end] >> 6);
      }
    gWDT_entropy_pool[gWDT_pool_end] += (gWDT_entropy_pool[gWDT_pool_end] << 3);
    gWDT_entropy_pool[gWDT_pool_end] ^= (gWDT_entropy_pool[gWDT_pool_end] >> 11);
    gWDT_entropy_pool[gWDT_pool_end] += (gWDT_entropy_pool[gWDT_pool_end] << 15);
    gWDT_entropy_pool[gWDT_pool_end] = gWDT_entropy_pool[gWDT_pool_end];
    gWDT_buffer_position = 0; // Start collecting the next 32 bytes of Timer 1 counts
    if (gWDT_pool_count == WDT_POOL_SIZE) // The entropy pool is full
      gWDT_pool_start = (gWDT_pool_start + 1) % WDT_POOL_SIZE;
    else // Add another unsigned long (32 bits) to the entropy pool
      ++gWDT_pool_count;
  }
}

uint32_t random() {
    uint32_t retVal;

    while (gWDT_pool_count < 1) ; // spin
    
    __disable_irq();  // crtical section
    retVal = gWDT_entropy_pool[gWDT_pool_start];
    gWDT_pool_start = (gWDT_pool_start + 1) % WDT_POOL_SIZE;
    --gWDT_pool_count;
    __enable_irq();
    return(retVal);
}

void TC5_Handler()
{
	collect();    // add a bit
	TC5->COUNT16.INTFLAG.bit.MC0 = 1;  // clear int flag
}

void clkinit() {
#if 0
    // optionally create a new CLK4 with clock source  OSC32K (NOT XOSC32K)
    SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_STARTUP( 0x6u ) |
     SYSCTRL_OSC32K_EN32K | SYSCTRL_OSC32K_ENABLE;
    while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC32KRDY) == 0 );
    GCLK->GENDIV.reg = GCLK_GENDIV_ID( 4) ;
       // select  SRC OSC32K or XOSC32K below
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 4 ) | GCLK_GENCTRL_SRC_OSC32K |
      GCLK_GENCTRL_GENEN ;
    while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );  // spin sync
#endif

	// Enable GCLK for TC5  GCLK2 or GCLK4
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
	while (GCLK->STATUS.bit.SYNCBUSY);

	TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);
	TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);
	while (TC5->COUNT16.CTRLA.bit.SWRST);

	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1 |
	  TC_CTRLA_WAVEGEN_MFRQ;   // MFRQ sends counter back to 0 after CC match
	TC5->COUNT16.CC[0].reg = 3; // duration in ticks
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

	NVIC_EnableIRQ(TC5_IRQn);
	TC5->COUNT16.INTENSET.bit.MC0 = 1;  // enable interrupt
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    // enable timer
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);
}

void setup() {
  	Serial.begin(9600);
  	pinMode(LED_PIN,OUTPUT);
  	clkinit();
    gWDT_buffer_position=0;
    gWDT_pool_start = 0;
    gWDT_pool_end = 0;
    gWDT_pool_count = 0;
}

#define REPS 50
void display() {
    uint32_t r,t;
    int i;
    float bps;

    t=micros();
    for (i=0;i<REPS;i++)r=random();
    t= micros() -t;
    bps = REPS*32.e6/t;
	Serial.print(bps,3); Serial.print(" bps  0x");
    Serial.println(r,HEX);
    delay(3000);
}

void logger(){
    uint32_t rng;

    while(!Serial.available());   // wait for host request
    Serial.read();
	while(1) {
    	rng = random(); 
    	Serial.write((uint8_t *)&rng,4);
    	digitalWrite(LED_PIN, rng & 1);  // random flash
	}
}

void loop() {
	 display();
	//logger();
}

