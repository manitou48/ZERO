//  dacdma
//  TCC timer   OVF triggers DMA to write to DAC (A0)


// use pin 12 for PWM TCC0 ch3 
#define PWMPIN 12
#define TCCx TCC0
#define TCCchannel 3

#define syncTCC while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)

// TCC freq = 48mhz/period
#define PERIOD 0x1000
#define DUTY (PERIOD/2)

void timer_init() {
	uint32_t cc;

	analogWrite(PWMPIN,0);  // init clock,timer,pin, and zero TODO just timer
	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // disable
	syncTCC;
	// set period and duty cycle
	TCCx->PER.reg = PERIOD;
	syncTCC;
	TCCx->CC[TCCchannel].reg = DUTY;   // initial duty cycle
	syncTCC;

	TCCx->CTRLA.reg |=  TCC_CTRLA_ENABLE ;  // start timer
	syncTCC;
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

static uint32_t chnl0 = 0;  // DMA channel
#define DMA_TRIGGER TCC0_DMAC_ID_OVF

#define HWORDS 128
uint16_t data[HWORDS];


void dma_init() {
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
      DMAC_CHCTRLB_TRIGSRC(DMA_TRIGGER) | DMAC_CHCTRLB_TRIGACT_BEAT;
    DMAC->CHCTRLB.reg = temp_CHCTRLB_reg;
    descriptor.descaddr = (uint32_t) &descriptor_section[chnl0];   // circular
    descriptor.srcaddr = (uint32_t)data + HWORDS*2;
    descriptor.dstaddr = (uint32_t)&DAC->DATA.reg;
    descriptor.btcnt =  HWORDS;
    descriptor.btctrl =  DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID;
    memcpy(&descriptor_section[chnl0],&descriptor, sizeof(dmacdescriptor));

    // start channel
    DMAC->CHID.reg = DMAC_CHID_ID(chnl0);
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}


void setup() {
	int i;
	float phase = 3.14159 * 2./HWORDS;

	Serial.begin(9600);
//	for (i=0;i<HWORDS/2;i++) data[i]=data[HWORDS-1-i] = 2*1024*i/HWORDS;  // sawtooth
	for (i=0;i<HWORDS;i++) data[i]= sinf(i*phase) * 500.0f + 512.0f;

	analogWriteResolution(10);
	analogWrite(A0,0);   // DAC init setup DAC pin and zero it
//	while(1) {for (i=0;i<HWORDS;i++){ analogWrite(A0,data[i]); delay(1);}} // scope 50ms
	dma_init();   // do me first
	timer_init();   // startup timer
}

void loop() {
	// could jumper DAC to A1 and sample it
 Serial.println(analogRead(A1));
}
