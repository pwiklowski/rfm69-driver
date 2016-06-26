#include "rfm69hal.h"
#include <unistd.h>
#include <sys/time.h>

#define STM32

#ifdef STM32
#include "systimer.h"
#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#endif


uint64_t base;

uint64_t get_current_ms(){
#ifdef STM32
	return mstimer_get();
#endif

#ifdef RPI
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
    return milliseconds;
#endif
}



int rfm69hal_init(){
#ifdef STM32
    base = get_current_ms();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	// configure SPI
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);

#endif

#ifdef RPI
    return wiringPiSPISetup(SPI_DEVICE, SPI_SPEED);
#endif

}


void rfm69hal_delay_ms(uint32_t ms){
#ifdef RPI
	usleep(ms*1000);
#endif

#ifdef STM32
	delay_ms(ms);
#endif


}

uint32_t rfm69hal_get_timer_ms(){
    return get_current_ms()-base;
}

void rfm69hal_enable(bool enable){

	if (enable)
	    GPIOA->BRR = GPIO_Pin_4;
	else
		GPIOA->BSRR = GPIO_Pin_4;
}

uint8_t rfm69hal_transfer(uint8_t* bytes, uint16_t size){
	for(uint16_t i=0; i<size; i++){
		while ((SPI1->SR & SPI_I2S_FLAG_TXE) == RESET);
		SPI1->DR = bytes[i];
		while ((SPI1->SR & SPI_I2S_FLAG_RXNE) == RESET);
		bytes[i] = SPI1->DR;
	}

#ifdef RPI
    wiringPiSPIDataRW(SPI_DEVICE, bytes,size);
#endif
}
