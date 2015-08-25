// DMA memory to memory   ZERO
// ch 18   beat burst block
//  xdk sam0/drivers/dma/dma.c
// packages/arduino/tools/CMSIS/4.0.0-atmel/Device/ATMEL/samd21/include/component/dmac.h
// http://asf.atmel.com/docs/3.16.0/samd21/html/asfdoc_sam0_sercom_spi_dma_use_case.html
//  assume normal SPI setup, then we take over with DMA

#include <SPI.h>
#define SPISPEED 4000000

const int SPISS = 10;

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

static uint32_t chnltx = 0, chnlrx = 1; // DMA channels
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
	DMAC->CHID.reg = DMAC_CHID_ID(chnltx); 
	DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
	DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
}

Sercom *sercomtx = ((Sercom   *)0x42001800U);  // SERCOM4

void spi_write(void *data,  size_t n) {
	// transmit src increment, not dest
	// triggers SERCOM4 TX 0x0a   RX 0x09 table 18-8
	uint32_t temp_CHCTRLB_reg;

	DMAC->CHID.reg = DMAC_CHID_ID(chnltx); // channel
	DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << chnltx));
	temp_CHCTRLB_reg = DMAC_CHCTRLB_LVL(0) | 
	DMAC_CHCTRLB_TRIGSRC(SERCOM4_DMAC_ID_TX) | DMAC_CHCTRLB_TRIGACT_BEAT;
	DMAC->CHCTRLB.reg = temp_CHCTRLB_reg;
	DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK ; // enable all 3 interrupts
	dmadone = 0;
	descriptor.descaddr = 0;
	//descriptor.dstaddr = (uint32_t) &sercomtx->SPI.DATA.bit.DATA;  // TODO
	descriptor.dstaddr = (uint32_t) (0x42001800U + 0x28);
	descriptor.srcaddr = (uint32_t)data + n;
	descriptor.btcnt =  n;
	descriptor.btctrl =   DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID;
	memcpy(&descriptor_section[chnltx],&descriptor, sizeof(dmacdescriptor));
	DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
	DMAC->SWTRIGCTRL.reg |= (1 << chnltx);  // trigger channel
	while(!dmadone);  // await DMA done isr
}

void setup() {
	Serial.begin(9600); delay(1000);
	pinMode (SPISS, OUTPUT);
	digitalWrite(SPISS,HIGH);
	SPI.begin();
	SPI.beginTransaction(SPISettings(SPISPEED, MSBFIRST, SPI_MODE0));
	Serial.print("SPI clk "); Serial.println(SPISPEED);
 	dma_init();
}

void loop() {
    int i, errs=0;
    unsigned long t1;
digitalWrite(SPISS,LOW);
    for (i=0;i<BYTES;i++) src[i] =  i;
 digitalWrite(SPISS,HIGH);
  delayMicroseconds(15);
#if 1
    t1 = micros();
	digitalWrite(SPISS,LOW);
	spi_write(src,BYTES);
	digitalWrite(SPISS,HIGH);
    t1 = micros() -t1;
    prmbs("SPI write",t1,BYTES*8);
#else
    t1 = micros();
	digitalWrite(SPISS,LOW);
	for(i=0;i<BYTES;i++) SPI.transfer(src[i]);
	digitalWrite(SPISS,HIGH);
    t1 = micros() -t1;
    prmbs("SPI transfer",t1,BYTES*8);
#endif
	delay(3000);
}
