#include "rfm69hal.h"
#include <unistd.h>
#include <sys/time.h>


#ifdef STM32F10X_MD
#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include "systimer.h"
#endif

#ifdef STM32F030
#include "stm32f0xx.h"
#include "systimer.h"
#include "stm32f0xx_spi.h"
#endif


#define SPI1_DRB (*((uint8_t *)&SPI1->DR))
uint64_t base;

uint64_t get_current_ms(){
#if defined(STM32F10X_MD) || defined(STM32F030)
	return mstimer_get();
#endif

#ifdef RPI
    struct timeval te; 
    gettimeofday(&te, NULL);
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000;
    return milliseconds;
#endif
}



int rfm69hal_init(){



#if defined(STM32F10X_MD) || defined(STM32F030)
    base = get_current_ms();

#ifdef STM32F10X_MD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPAEN, ENABLE);
#endif

	RCC_APB2PeriphClockCmd(RCC_APB2ENR_SPI1EN, ENABLE);

#ifdef STM32F030

	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOAEN, ENABLE);

	GPIO_InitTypeDef gpioStructure;

	gpioStructure.GPIO_Pin = GPIO_Pin_6 ;
	gpioStructure.GPIO_Mode = GPIO_Mode_AF;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	gpioStructure.GPIO_OType= GPIO_OType_OD;
	gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpioStructure);

	gpioStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	gpioStructure.GPIO_Mode = GPIO_Mode_AF;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	gpioStructure.GPIO_OType= GPIO_OType_PP;
	gpioStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpioStructure);

	gpioStructure.GPIO_Pin = GPIO_Pin_4;
	gpioStructure.GPIO_Mode = GPIO_Mode_OUT;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	gpioStructure.GPIO_OType= GPIO_OType_PP;
	gpioStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &gpioStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);



#endif

	// configure SPI
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);

#ifdef STM32F030
	SPI_RxFIFOThresholdConfig (SPI1, SPI_RxFIFOThreshold_QF);
#endif

#endif

#ifdef RPI
    return wiringPiSPISetup(SPI_DEVICE, SPI_SPEED);
#endif

}


void rfm69hal_delay_ms(uint32_t ms){
#ifdef RPI
	usleep(ms*1000);
#endif

#if defined(STM32F10X_MD) || defined(STM32F030)
	delay_ms(ms);
#endif


}

uint32_t rfm69hal_get_timer_ms(){
    return get_current_ms()-base;
}

void rfm69hal_enable(bool enable){
#if defined(STM32F10X_MD) || defined(STM32F030)
    if (enable)
	    GPIOA->BRR = GPIO_Pin_4;
	else
        GPIOA->BSRR = GPIO_Pin_4;
#endif
}

uint8_t rfm69hal_transfer(uint8_t* bytes, uint16_t size){
#if defined(STM32F10X_MD) || defined(STM32F030)
    for(uint16_t i=0; i<size; i++){
		while ((SPI1->SR & SPI_I2S_FLAG_TXE) == RESET);
#ifdef STM32F10X_MD
		SPI1->DR = bytes[i];
#endif
#ifdef STM32F030
		SPI1_DRB = bytes[i];
#endif
		while ((SPI1->SR & SPI_I2S_FLAG_RXNE) == RESET);
#ifdef STM32F10X_MD
		bytes[i] = SPI1_DRB;
#endif
#ifdef STM32F030
		bytes[i] = SPI1->DR;
#endif
    }
#endif

#ifdef RPI
    wiringPiSPIDataRW(SPI_DEVICE, bytes,size);
#endif
}
