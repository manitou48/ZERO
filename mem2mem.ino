// DMA memory to memory   ZERO
// ch 18   beat burst block
//  xdk sam0/drivers/dma/dma.c
// packages/arduino/tools/CMSIS/4.0.0-atmel/Device/ATMEL/samd21/include/component/dmac.h

#define PRREG(x) Serial.print(#x" 0x"); Serial.println(x,HEX)


#define BYTES 1024
#define WORDS BYTES/sizeof(int)
char src[BYTES] __attribute__ ((aligned (8)));
char dst[BYTES] __attribute__ ((aligned (8)));
int *srcw = (int *) src, *dstw = (int *)dst;

void prmbs(char *lbl,unsigned long us,int bits) {
    float mbs = (float)bits/us;
    Serial.print(mbs,2); Serial.print(" mbs  ");
    Serial.print(us); Serial.print(" us   ");
    Serial.println(lbl);
}

// DMA   12 channels
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

static uint32_t chnl = 0; // DMA channel
volatile uint32_t dmadone;

void DMAC_Handler() {
	// interrupts DMAC_CHINTENCLR_TERR DMAC_CHINTENCLR_TCMPL DMAC_CHINTENCLR_SUSP
	uint8_t active_channel;

	// disable irqs ?
	active_channel =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk; // get channel number
	DMAC->CHID.reg = DMAC_CHID_ID(active_channel);
	dmadone = DMAC->CHINTFLAG.reg;
	DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear
	DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR;
	DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_SUSP;
}

void dma_init() {
	// probably on by default
	PM->AHBMASK.reg |= PM_AHBMASK_DMAC ;
	PM->APBBMASK.reg |= PM_APBBMASK_DMAC ;
	NVIC_EnableIRQ( DMAC_IRQn ) ;

	DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
	DMAC->WRBADDR.reg = (uint32_t)wrb;
	DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
	DMAC->CHID.reg = DMAC_CHID_ID(chnl); 
	DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
	DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
}

void memcpy8(void *dst, const void *src, size_t n) {
	DMAC->CHID.reg = DMAC_CHID_ID(chnl); // channel
	DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK ; // enable all 3 interrupts
	dmadone = 0;
	descriptor.descaddr = 0;
	descriptor.dstaddr = (uint32_t)dst + n;
	descriptor.srcaddr = (uint32_t)src + n;
	descriptor.btcnt =  n/4;
	descriptor.btctrl =  DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID;
	memcpy(&descriptor_section[chnl],&descriptor, sizeof(dmacdescriptor));
	DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
	DMAC->SWTRIGCTRL.reg |= (1 << chnl);  // trigger channel
	while(!dmadone);  // await DMA done isr
}

void setup() {
	Serial.begin(9600);
 	dma_init();
}

void loop() {
    int i, errs=0;
    unsigned long t1;
    for (i=0;i<BYTES;i++) src[i] =  i;
    memset(dst,0,BYTES);
    t1 = micros();
	memcpy(dst,src,BYTES);
    t1 = micros() -t1;
    prmbs("memcpy",t1,BYTES*8);
    memset(dst,0,BYTES);
    t1 = micros();
	memcpy8(dst,src,BYTES);
    t1 = micros() -t1;
    prmbs("dma",t1,BYTES*8);
    for (i=0;i<BYTES;i++) if (dst[i] !=  i%256)errs++;
    Serial.print("errs "); Serial.println(errs);
	delay(3000);
}
