/******************************************************************************
* 
* Interrupt Handler for STM32F334
* (c) Jürgen Liegner 2015
* 
*******************************************************************************/  

#ifndef IRQ_STM32F103_H
  #define IRQ_STM32F103_H

#ifdef __cplusplus
extern "C" {
#endif

  #ifndef STARTUP_CPP
    #define WEAK
    #define ALIAS(a)
  #endif  
 
  WEAK void NMI_Handler(void);
  WEAK void HardFault_Handler(void);
  WEAK void MemManage_Handler(void);
  WEAK void BusFault_Handler(void);
  WEAK void UsageFault_Handler(void);
  WEAK void SVC_Handler(void);
  WEAK void DebugMon_Handler(void);
  WEAK void PendSV_Handler(void);
  WEAK void SysTick_Handler(void);
  WEAK void IntDefaultHandler(void);

  void WWDG_IRQHandler(void) ALIAS(IntDefaultHandler);
  void PVD_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TAMPER_IRQHandler(void) ALIAS(IntDefaultHandler);
  void RTC_IRQHandler(void) ALIAS(IntDefaultHandler);
  void FLASH_IRQHandler(void) ALIAS(IntDefaultHandler);
  void RCC_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI0_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI3_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI4_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Channel1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Channel2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Channel3_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Channel4_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Channel5_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Channel6_IRQHandler(void) ALIAS(IntDefaultHandler);
  void DMA1_Channel7_IRQHandler(void) ALIAS(IntDefaultHandler);
  void ADC1_2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void USB_HP_CAN1_TX_IRQHandler(void) ALIAS(IntDefaultHandler);
  void USB_LP_CAN1_RX0_IRQHandler(void) ALIAS(IntDefaultHandler);
  void CAN1_RX1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void CAN1_SCE_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI9_5_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM1_BRK_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM1_UP_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM1_TRG_COM_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM1_CC_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM3_IRQHandler(void) ALIAS(IntDefaultHandler);
  void TIM4_IRQHandler(void) ALIAS(IntDefaultHandler);
  void I2C1_EV_IRQHandler(void) ALIAS(IntDefaultHandler);
  void I2C1_ER_IRQHandler(void) ALIAS(IntDefaultHandler);
  void I2C2_EV_IRQHandler(void) ALIAS(IntDefaultHandler);
  void I2C2_ER_IRQHandler(void) ALIAS(IntDefaultHandler);
  void SPI1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void SPI2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void USART1_IRQHandler(void) ALIAS(IntDefaultHandler);
  void USART2_IRQHandler(void) ALIAS(IntDefaultHandler);
  void USART3_IRQHandler(void) ALIAS(IntDefaultHandler);
  void EXTI15_10_IRQHandler(void) ALIAS(IntDefaultHandler);
  void RTC_Alarm_IRQHandler(void) ALIAS(IntDefaultHandler);
  void USBWakeUp_IRQHandler(void) ALIAS(IntDefaultHandler);

#ifdef __cplusplus
  }
#endif
  
#endif

