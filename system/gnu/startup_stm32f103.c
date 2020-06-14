  extern void __libc_init_array(void);
  extern int   main(void);
  extern void  SystemInit (void);
  extern void  StackTop(void);

  #define WEAK __attribute__ ((weak))
  #define ALIAS(f) __attribute__ ((weak, alias (#f)))
  #define STARTUP_CPP       
  
  #include <irq_stm32f103.h>
  
#ifndef STARTUP_FROM_RESET     
  // Crossworks Besonderheit   
  void reset_wait(void) ALIAS(IntDefaultHandler);
#endif  
  void reset_handler(void);

extern void (* const vectors[])(void);
__attribute__ ((section(".vectors")))
void (* const vectors[])(void) = 
{
  &StackTop,                              
#ifdef STARTUP_FROM_RESET     // Crossworks besonderheit   
  reset_handler,                          
#else 
  reset_wait,
#endif 
  NMI_Handler,                // The NMI handler
  HardFault_Handler,          // The hard fault handler
  MemManage_Handler,          // 
  BusFault_Handler,           //
  UsageFault_Handler,         //
  0, 
  0,                          // Reserved
  0,                          // Reserved
  0,                          // Reserved
  SVC_Handler,                // SVCall handler
  DebugMon_Handler,           // Reserved
  0,                          // Reserved
  PendSV_Handler,             // The PendSV handler
  SysTick_Handler,            // The SysTick handler
  WWDG_IRQHandler,
  PVD_IRQHandler,
  TAMPER_IRQHandler,
  RTC_IRQHandler,
  FLASH_IRQHandler,
  RCC_IRQHandler,
  EXTI0_IRQHandler,
  EXTI1_IRQHandler,
  EXTI2_IRQHandler,
  EXTI3_IRQHandler,
  EXTI4_IRQHandler,
  DMA1_Channel1_IRQHandler,
  DMA1_Channel2_IRQHandler,
  DMA1_Channel3_IRQHandler,
  DMA1_Channel4_IRQHandler,
  DMA1_Channel5_IRQHandler,
  DMA1_Channel6_IRQHandler,
  DMA1_Channel7_IRQHandler,
  ADC1_2_IRQHandler,
  USB_HP_CAN1_TX_IRQHandler,
  USB_LP_CAN1_RX0_IRQHandler,
  CAN1_RX1_IRQHandler,
  CAN1_SCE_IRQHandler,
  EXTI9_5_IRQHandler,
  TIM1_BRK_IRQHandler,
  TIM1_UP_IRQHandler,
  TIM1_TRG_COM_IRQHandler,
  TIM1_CC_IRQHandler,
  TIM2_IRQHandler,
  TIM3_IRQHandler,
  TIM4_IRQHandler,
  I2C1_EV_IRQHandler,
  I2C1_ER_IRQHandler,
  I2C2_EV_IRQHandler,
  I2C2_ER_IRQHandler,
  SPI1_IRQHandler,
  SPI2_IRQHandler,
  USART1_IRQHandler,
  USART2_IRQHandler,
  USART3_IRQHandler,
  EXTI15_10_IRQHandler,
  RTC_Alarm_IRQHandler,
  USBWakeUp_IRQHandler,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  
  0,
  0,
  0,
  0, //TIM6_DAC_IRQHandler,
  0, //TIM7_IRQHandler,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  (void*)0xF108F85F
};
  
extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;

void reset_handler(void)
{
  unsigned char *ps, *pd;
  // data segment kopieren
  ps = (unsigned char *)&_etext;
  for (pd=(unsigned char *)&_data; pd < (unsigned char *)&_edata; )
    {
    *pd++ = *ps++;
    }

  // bss mit 0 initialisieren
  for (pd = (unsigned char *)&_bss; pd < (unsigned char *)&_ebss; pd++)
    *pd = 0;

  // oscillator und pll einstellen  
  // SystemInit();

  // c++ lib initialisieren
  __libc_init_array();
  main();
  while (1) 
    ;
}

void NMI_Handler(void)
{
  while(1)
    {
    }
}

void HardFault_Handler(void)
{
  while(1)
    {
    }
}

void BusFault_Handler(void)
{
  while(1)
    {
    }
}

void UsageFault_Handler(void)
{
  while(1)
    {
    }
}

void DebugMon_Handler(void)
{
  while(1)
    {
    }
}

void MemManage_Handler(void)
{
  while(1)
    {
    }
}

void SVC_Handler(void)
{
  while(1)
    {
    }
}

void PendSV_Handler(void)
{
  while(1)
    {
    }
}

void SysTick_Handler(void)
{
  while(1)
    {
    }
}

void IntDefaultHandler(void)
{
  while(1)
    {
    }
}
