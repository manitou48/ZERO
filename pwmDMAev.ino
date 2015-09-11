//  pwmDMAev
//http://asf.atmel.com/docs/3.16.0/samd21/html/asfdoc_sam0_tcc_dma_use_case.html
//  TCC to generate PWM,  OVF triggers DMA to change duty cycle
//   rising triggers EVENT to trigger TCC capture with DMA to array

// use pin 12 for PWM TCC0 ch3     input capture ch1 event
#define PWMPIN 12
#define TCCx TCC0
#define TCCchannel 3
#define DMA_PWM_TRIGGER TCC0_DMAC_ID_OVF
#define DMA_CAPTURE_TRIGGER TCC0_DMAC_ID_MC_1
#define syncTCC while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)

#define TCC_CAPTURE_CHANNEL      1
#define EVENT_GENERATOR      EVSYS_ID_GEN_TCC0_MCX_3
#define EVENT_USER           EVSYS_ID_USER_TCC0_MC_1

#define PERIOD 0x1000
#define DUTY (3*PERIOD/4)

void pwm_init() {
	uint32_t cc;

	analogWrite(PWMPIN,0);  // init clock,timer,pin, and zero
	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // disable
	syncTCC;
	// set period and duty cycle
	TCCx->PER.reg = PERIOD;
	syncTCC;
	TCCx->CC[TCCchannel].reg = DUTY;   // initial duty cycle
	syncTCC;
	//  enable events on TCCx
	TCCx->EVCTRL.reg = TCC_EVCTRL_MCEI1 | TCC_EVCTRL_MCEO3;  
	TCCx->CTRLA.reg |= TCC_CTRLA_CPTEN1 | TCC_CTRLA_ENABLE ;
	syncTCC;
}

// event  generator: PWM rising, user: TCCx channel 1  
#define ECHNL 3
#define EGCLK GCLK_CLKCTRL_ID_EVSYS_3

void event_init() {
	// enable event device and clock, reset, setup USER, setup generator
	PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | 
	  GCLK_CLKCTRL_GEN_GCLK0 | EGCLK);
	while (GCLK->STATUS.bit.SYNCBUSY);

	EVSYS->CTRL.reg = EVSYS_CTRL_SWRST;   // reset
//	EVSYS->CTRL.reg =  EVSYS_CTRL_GCLKREQ;  // for SWEVT
	EVSYS->USER.reg = (uint16_t) (EVSYS_USER_USER(EVENT_USER) |
	  EVSYS_USER_CHANNEL(ECHNL+1));  // evchnl   (n+1)
	EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_RISING_EDGE | 
	  EVSYS_CHANNEL_EVGEN(EVENT_GENERATOR) |
	  EVSYS_CHANNEL_CHANNEL(ECHNL); // evchnl , synchronous
}


typedef struct {
    uint16_t btctrl;
    uint16_t btcnt;
    uint32_t srcaddr;
    uint32_t dstaddr;
    uint32_t descaddr;
} dmacdescriptor ;
volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor __attribute__ ((aligned (16)));

static uint32_t chnl0 = 0, chnl1 = 1;  // DMA channels

#define HWORDS 3
uint16_t dutys[HWORDS] = { PERIOD/4, PERIOD/2, 3*PERIOD/4};
volatile uint16_t captures[HWORDS] = {1,2,3};

void dma_duty_init() {
	// trigger on TCC OVF, update TCC duty, circular
    uint32_t temp_CHCTRLB_reg;

    // probably on by default
    PM->AHBMASK.reg |= PM_AHBMASK_DMAC ;
    PM->APBBMASK.reg |= PM_APBBMASK_DMAC ;

    DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
    DMAC->WRBADDR.reg = (uint32_t)wrb;
    DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);

    DMAC->CHID.reg = DMAC_CHID_ID(chnl0);
    DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
    DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
    DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << chnl0));
    temp_CHCTRLB_reg = DMAC_CHCTRLB_LVL(0) |
      DMAC_CHCTRLB_TRIGSRC(DMA_PWM_TRIGGER) | DMAC_CHCTRLB_TRIGACT_BEAT;
    DMAC->CHCTRLB.reg = temp_CHCTRLB_reg;
    descriptor.descaddr = (uint32_t) &descriptor_section[chnl0];   // circular
    descriptor.srcaddr = (uint32_t)dutys + HWORDS*2;
    descriptor.dstaddr = (uint32_t)&TCCx->CC[TCCchannel].reg;
    descriptor.btcnt =  HWORDS;
    descriptor.btctrl =  DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID;
    memcpy(&descriptor_section[chnl0],&descriptor, sizeof(dmacdescriptor));

    // start channel
    DMAC->CHID.reg = DMAC_CHID_ID(chnl0);
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void dma_capture_init() {
	// assumes dma_duty_init has been run
    uint32_t temp_CHCTRLB_reg;

    DMAC->CHID.reg = DMAC_CHID_ID(chnl1);
    DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
    DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
    DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << chnl1));
    temp_CHCTRLB_reg = DMAC_CHCTRLB_LVL(0) |
      DMAC_CHCTRLB_TRIGSRC(DMA_CAPTURE_TRIGGER) | DMAC_CHCTRLB_TRIGACT_BEAT;
    DMAC->CHCTRLB.reg = temp_CHCTRLB_reg;
    descriptor.descaddr = (uint32_t) &descriptor_section[chnl1];   // circular
    descriptor.dstaddr = (uint32_t)captures + HWORDS*2;
    descriptor.srcaddr = (uint32_t)&TCCx->CC[TCC_CAPTURE_CHANNEL].reg; 
    descriptor.btcnt =  HWORDS;
    descriptor.btctrl =  DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID;
    memcpy(&descriptor_section[chnl1],&descriptor, sizeof(dmacdescriptor));

    // start channel
    DMAC->CHID.reg = DMAC_CHID_ID(chnl1);
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void setup() {
	Serial.begin(9600);
	dma_duty_init();   // do me first
	dma_capture_init();
	event_init();
	pwm_init();   // startup PWM
}

void loop() {
	Serial.print(captures[0]); Serial.print(" ");
	Serial.print(captures[1]); Serial.print(" ");
	Serial.println(captures[2]);
	delay(2000);
}
