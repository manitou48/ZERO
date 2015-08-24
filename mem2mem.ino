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
struct dmacdescriptor {
	uint16_t btctrl;
	uint16_t btcnt;
	uint32_t srcaddr;
	uint32_t dstaddr;
	uint32_t descaddr;
} ;
struct dmacdescriptor wrb[12] __attribute__ ((aligned (16)));
struct dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));
struct dmacdescriptor descriptor __attribute__ ((aligned (16)));

void DMAC_Handler() {
	// interrupts DMAC_CHINTENCLR_TERR DMAC_CHINTENCLR_TCMPL DMAC_CHINTENCLR_SUSP
	// disable irqs ?
	DMAC->CHID.reg = DMAC_CHID_ID(0);
	DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear
}

void dma_init() {
	// probably on by default
	PM->AHBMASK.reg |= PM_AHBMASK_DMAC ;
	PM->APBBMASK.reg |= PM_APBBMASK_DMAC ;
	// NVIC_EnableIRQ( DMAC_IRQn ) ;

	wrb[0].descaddr = (uint32_t)&descriptor;  // or not, memcpy below?
	DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
	DMAC->WRBADDR.reg = (uint32_t)wrb;
	DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
	DMAC->CHID.reg = DMAC_CHID_ID(0); // channel 0
	DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
	DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
}

void memcpy32(void *dst, const void *src, size_t n) {
	DMAC->CHID.reg = DMAC_CHID_ID(0); // channel 0
	// DMAC->CHINTENSET.reg = (DMAC_CHINTENSET_MASK & 2); // bits err0 complete1 suspend2
	descriptor.descaddr = 0;
	descriptor.dstaddr = (uint32_t)dst + n;
	descriptor.srcaddr = (uint32_t)src + n;
	descriptor.btcnt =  n;
	descriptor.btctrl =  DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID;
	memcpy(&descriptor_section[0],&descriptor, sizeof(DmacDescriptor));
	DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
	DMAC->SWTRIGCTRL.reg |= (1 << 0);  // trigger channel 0
	while (! (DMAC->CHINTFLAG.reg & DMAC_CHINTENCLR_TCMPL)); // spin wait
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
	memcpy32(dst,src,BYTES);
    t1 = micros() -t1;
    prmbs("dma",t1,BYTES*8);
    for (i=0;i<BYTES;i++) if (dst[i] !=  i%256)errs++;
    Serial.print("errs "); Serial.println(errs);
	delay(3000);

}
