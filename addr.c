#include <stdio.h>
#include "stm32f2xx.h"

// print
//#define dump(name, str) printf("0x%08x\t%s\t%s\t%ld\n", name##_BASE, #name, #str, sizeof(str))

// c
//ETH_TypeDef * ETH = ((ETH_TypeDef *) ETH_BASE)
//#define dump(name, str) printf("%s* %s = (%s*) 0x%08x; //%ld\n", #str, #name, #str, name##_BASE,sizeof(str))

// idapython
//#define dump(name, str) printf("AddSeg(0x%08x, 0x%08x, 0x0, 1, 1, 0); s = getseg(0x%08x); set_segm_name(s, '%s')\n", name##_BASE, name##_BASE+(unsigned int)sizeof(str), name##_BASE, #name);

// idc
#define dump(name, str) printf("add_segm_ex(0x%08x, 0x%08x, 0, 1, 0, 0, 0); set_segm_name(0x%08x, \"%s\"); apply_type(0x%08x, get_tinfo(\"%s\"), 1);set_name(0x%08x, \"%s\", 2);\n", name##_BASE, name##_BASE+(unsigned int)sizeof(str), name##_BASE, #name, name##_BASE, #str, name##_BASE, #name);

int main(void) {
  //printf("from idautils import *\nfrom idc import *\nfrom idaapi import *\n\nSEGMENT_ARCH = 32\n");
  printf("#include <idc.idc>\nstatic main(){\n");

  dump(SCS, InterruptType_Type);
  dump(ITM, ITM_Type);
  dump(CoreDebug, CoreDebug_Type);
  dump(SysTick, SysTick_Type);
  dump(NVIC, NVIC_Type);
  dump(SCB, SCB_Type);
  dump(MPU, MPU_Type);

  dump(TIM2, TIM_TypeDef);
  dump(TIM3, TIM_TypeDef);
  dump(TIM4, TIM_TypeDef);
  dump(TIM5, TIM_TypeDef);
  dump(TIM6, TIM_TypeDef);
  dump(TIM7, TIM_TypeDef);
  dump(TIM12, TIM_TypeDef);
  dump(TIM13, TIM_TypeDef);
  dump(TIM14, TIM_TypeDef);
  dump(RTC, RTC_TypeDef);
  dump(WWDG, WWDG_TypeDef);
  dump(IWDG, IWDG_TypeDef);
  dump(SPI2, SPI_TypeDef);
  dump(SPI3, SPI_TypeDef);
  dump(USART2, USART_TypeDef);
  dump(USART3, USART_TypeDef);
  dump(UART4, USART_TypeDef);
  dump(UART5, USART_TypeDef);
  dump(I2C1, I2C_TypeDef);
  dump(I2C2, I2C_TypeDef);
  dump(I2C3, I2C_TypeDef);
  dump(CAN1, CAN_TypeDef);
  dump(CAN2, CAN_TypeDef);
  dump(PWR, PWR_TypeDef);
  dump(DAC, DAC_TypeDef);
  dump(TIM1, TIM_TypeDef);
  dump(TIM8, TIM_TypeDef);
  dump(USART1, USART_TypeDef);
  dump(USART6, USART_TypeDef);
  dump(ADC1, ADC_TypeDef);
  dump(ADC2, ADC_TypeDef);
  dump(ADC3, ADC_TypeDef);
  dump(ADC, ADC_Common_TypeDef);
  dump(SDIO, SDIO_TypeDef);
  dump(SPI1, SPI_TypeDef);
  dump(SYSCFG, SYSCFG_TypeDef);
  dump(EXTI, EXTI_TypeDef);
  dump(TIM9, TIM_TypeDef);
  dump(TIM10, TIM_TypeDef);
  dump(TIM11, TIM_TypeDef);
  dump(GPIOA, GPIO_TypeDef);
  dump(GPIOB, GPIO_TypeDef);
  dump(GPIOC, GPIO_TypeDef);
  dump(GPIOD, GPIO_TypeDef);
  dump(GPIOE, GPIO_TypeDef);
  dump(GPIOF, GPIO_TypeDef);
  dump(GPIOG, GPIO_TypeDef);
  dump(GPIOH, GPIO_TypeDef);
  dump(GPIOI, GPIO_TypeDef);
  dump(CRC, CRC_TypeDef);
  dump(RCC, RCC_TypeDef);
  dump(FLASH_R, FLASH_TypeDef);
  dump(DMA1, DMA_TypeDef);
  dump(DMA1_Stream0, DMA_Stream_TypeDef);
  dump(DMA1_Stream1, DMA_Stream_TypeDef);
  dump(DMA1_Stream2, DMA_Stream_TypeDef);
  dump(DMA1_Stream3, DMA_Stream_TypeDef);
  dump(DMA1_Stream4, DMA_Stream_TypeDef);
  dump(DMA1_Stream5, DMA_Stream_TypeDef);
  dump(DMA1_Stream6, DMA_Stream_TypeDef);
  dump(DMA1_Stream7, DMA_Stream_TypeDef);
  dump(DMA2, DMA_TypeDef);
  dump(DMA2_Stream0, DMA_Stream_TypeDef);
  dump(DMA2_Stream1, DMA_Stream_TypeDef);
  dump(DMA2_Stream2, DMA_Stream_TypeDef);
  dump(DMA2_Stream3, DMA_Stream_TypeDef);
  dump(DMA2_Stream4, DMA_Stream_TypeDef);
  dump(DMA2_Stream5, DMA_Stream_TypeDef);
  dump(DMA2_Stream6, DMA_Stream_TypeDef);
  dump(DMA2_Stream7, DMA_Stream_TypeDef);
  dump(ETH, ETH_TypeDef);
  dump(DCMI, DCMI_TypeDef);
  dump(CRYP, CRYP_TypeDef);
  dump(HASH, HASH_TypeDef);
  dump(RNG, RNG_TypeDef);
  dump(FSMC_Bank1_R, FSMC_Bank1_TypeDef);
  dump(FSMC_Bank1E_R, FSMC_Bank1E_TypeDef);
  dump(FSMC_Bank2_R, FSMC_Bank2_TypeDef);
  dump(FSMC_Bank3_R, FSMC_Bank3_TypeDef);
  dump(FSMC_Bank4_R, FSMC_Bank4_TypeDef);
  dump(DBGMCU, DBGMCU_TypeDef);

  printf("}\n");

  return 0;
}

