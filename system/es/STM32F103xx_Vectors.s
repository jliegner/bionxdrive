/*****************************************************************************
 *                   SEGGER Microcontroller GmbH & Co. KG                    *
 *            Solutions for real time microcontroller applications           *
 *****************************************************************************
 *                                                                           *
 *               (c) 2017 SEGGER Microcontroller GmbH & Co. KG               *
 *                                                                           *
 *           Internet: www.segger.com   Support: support@segger.com          *
 *                                                                           *
 *****************************************************************************/

/*****************************************************************************
 *                         Preprocessor Definitions                          *
 *                         ------------------------                          *
 * VECTORS_IN_RAM                                                            *
 *                                                                           *
 *   If defined, an area of RAM will large enough to store the vector table  *
 *   will be reserved.                                                       *
 *                                                                           *
 *****************************************************************************/

  .syntax unified
  .code 16

  .section .init, "ax"
  .align 0

/*****************************************************************************
 * Default Exception Handlers                                                *
 *****************************************************************************/

  .thumb_func
  .weak NMI_Handler
NMI_Handler:
  b .

  .thumb_func
  .weak HardFault_Handler
HardFault_Handler:
  b .

  .thumb_func
  .weak SVC_Handler
SVC_Handler:
  b .

  .thumb_func
  .weak PendSV_Handler
PendSV_Handler:
  b .

  .thumb_func
  .weak SysTick_Handler
SysTick_Handler:
  b .

  .thumb_func
Dummy_Handler:
  b .

#if defined(__OPTIMIZATION_SMALL)

  .weak WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler,Dummy_Handler

  .weak PVD_IRQHandler
  .thumb_set PVD_IRQHandler,Dummy_Handler

  .weak TAMPER_IRQHandler
  .thumb_set TAMPER_IRQHandler,Dummy_Handler

  .weak RTC_IRQHandler
  .thumb_set RTC_IRQHandler,Dummy_Handler

  .weak FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler,Dummy_Handler

  .weak RCC_IRQHandler
  .thumb_set RCC_IRQHandler,Dummy_Handler

  .weak EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler,Dummy_Handler

  .weak EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler,Dummy_Handler

  .weak EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler,Dummy_Handler

  .weak EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler,Dummy_Handler

  .weak EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler,Dummy_Handler

  .weak DMA1_Channel1_IRQHandler
  .thumb_set DMA1_Channel1_IRQHandler,Dummy_Handler

  .weak DMA1_Channel2_IRQHandler
  .thumb_set DMA1_Channel2_IRQHandler,Dummy_Handler

  .weak DMA1_Channel3_IRQHandler
  .thumb_set DMA1_Channel3_IRQHandler,Dummy_Handler

  .weak DMA1_Channel4_IRQHandler
  .thumb_set DMA1_Channel4_IRQHandler,Dummy_Handler

  .weak DMA1_Channel5_IRQHandler
  .thumb_set DMA1_Channel5_IRQHandler,Dummy_Handler

  .weak DMA1_Channel6_IRQHandler
  .thumb_set DMA1_Channel6_IRQHandler,Dummy_Handler

  .weak DMA1_Channel7_IRQHandler
  .thumb_set DMA1_Channel7_IRQHandler,Dummy_Handler

  .weak ADC1_2_IRQHandler
  .thumb_set ADC1_2_IRQHandler,Dummy_Handler

  .weak USB_HP_CAN1_TX_IRQHandler
  .thumb_set USB_HP_CAN1_TX_IRQHandler,Dummy_Handler

  .weak USB_LP_CAN1_RX0_IRQHandler
  .thumb_set USB_LP_CAN1_RX0_IRQHandler,Dummy_Handler

  .weak CAN1_RX1_IRQHandler
  .thumb_set CAN1_RX1_IRQHandler,Dummy_Handler

  .weak CAN1_SCE_IRQHandler
  .thumb_set CAN1_SCE_IRQHandler,Dummy_Handler

  .weak EXTI9_5_IRQHandler
  .thumb_set EXTI9_5_IRQHandler,Dummy_Handler

  .weak TIM1_BRK_IRQHandler
  .thumb_set TIM1_BRK_IRQHandler,Dummy_Handler

  .weak TIM1_UP_IRQHandler
  .thumb_set TIM1_UP_IRQHandler,Dummy_Handler

  .weak TIM1_TRG_COM_IRQHandler
  .thumb_set TIM1_TRG_COM_IRQHandler,Dummy_Handler

  .weak TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler,Dummy_Handler

  .weak TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,Dummy_Handler

  .weak TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler,Dummy_Handler

  .weak I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler,Dummy_Handler

  .weak I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler,Dummy_Handler

  .weak I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler,Dummy_Handler

  .weak I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler,Dummy_Handler

  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Dummy_Handler

  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Dummy_Handler

  .weak USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Dummy_Handler

  .weak USART3_IRQHandler
  .thumb_set USART3_IRQHandler,Dummy_Handler

  .weak EXTI15_10_IRQHandler
  .thumb_set EXTI15_10_IRQHandler,Dummy_Handler

  .weak RTCAlarm_IRQHandler
  .thumb_set RTCAlarm_IRQHandler,Dummy_Handler

  .weak USBWakeUp_IRQHandler
  .thumb_set USBWakeUp_IRQHandler,Dummy_Handler

#else

  .thumb_func
  .weak WWDG_IRQHandler
WWDG_IRQHandler:
  b .

  .thumb_func
  .weak PVD_IRQHandler
PVD_IRQHandler:
  b .

  .thumb_func
  .weak TAMPER_IRQHandler
TAMPER_IRQHandler:
  b .

  .thumb_func
  .weak RTC_IRQHandler
RTC_IRQHandler:
  b .

  .thumb_func
  .weak FLASH_IRQHandler
FLASH_IRQHandler:
  b .

  .thumb_func
  .weak RCC_IRQHandler
RCC_IRQHandler:
  b .

  .thumb_func
  .weak EXTI0_IRQHandler
EXTI0_IRQHandler:
  b .

  .thumb_func
  .weak EXTI1_IRQHandler
EXTI1_IRQHandler:
  b .

  .thumb_func
  .weak EXTI2_IRQHandler
EXTI2_IRQHandler:
  b .

  .thumb_func
  .weak EXTI3_IRQHandler
EXTI3_IRQHandler:
  b .

  .thumb_func
  .weak EXTI4_IRQHandler
EXTI4_IRQHandler:
  b .

  .thumb_func
  .weak DMA1_Channel1_IRQHandler
DMA1_Channel1_IRQHandler:
  b .

  .thumb_func
  .weak DMA1_Channel2_IRQHandler
DMA1_Channel2_IRQHandler:
  b .

  .thumb_func
  .weak DMA1_Channel3_IRQHandler
DMA1_Channel3_IRQHandler:
  b .

  .thumb_func
  .weak DMA1_Channel4_IRQHandler
DMA1_Channel4_IRQHandler:
  b .

  .thumb_func
  .weak DMA1_Channel5_IRQHandler
DMA1_Channel5_IRQHandler:
  b .

  .thumb_func
  .weak DMA1_Channel6_IRQHandler
DMA1_Channel6_IRQHandler:
  b .

  .thumb_func
  .weak DMA1_Channel7_IRQHandler
DMA1_Channel7_IRQHandler:
  b .

  .thumb_func
  .weak ADC1_2_IRQHandler
ADC1_2_IRQHandler:
  b .

  .thumb_func
  .weak USB_HP_CAN1_TX_IRQHandler
USB_HP_CAN1_TX_IRQHandler:
  b .

  .thumb_func
  .weak USB_LP_CAN1_RX0_IRQHandler
USB_LP_CAN1_RX0_IRQHandler:
  b .

  .thumb_func
  .weak CAN1_RX1_IRQHandler
CAN1_RX1_IRQHandler:
  b .

  .thumb_func
  .weak CAN1_SCE_IRQHandler
CAN1_SCE_IRQHandler:
  b .

  .thumb_func
  .weak EXTI9_5_IRQHandler
EXTI9_5_IRQHandler:
  b .

  .thumb_func
  .weak TIM1_BRK_IRQHandler
TIM1_BRK_IRQHandler:
  b .

  .thumb_func
  .weak TIM1_UP_IRQHandler
TIM1_UP_IRQHandler:
  b .

  .thumb_func
  .weak TIM1_TRG_COM_IRQHandler
TIM1_TRG_COM_IRQHandler:
  b .

  .thumb_func
  .weak TIM1_CC_IRQHandler
TIM1_CC_IRQHandler:
  b .

  .thumb_func
  .weak TIM2_IRQHandler
TIM2_IRQHandler:
  b .

  .thumb_func
  .weak TIM3_IRQHandler
TIM3_IRQHandler:
  b .

  .thumb_func
  .weak TIM4_IRQHandler
TIM4_IRQHandler:
  b .

  .thumb_func
  .weak I2C1_EV_IRQHandler
I2C1_EV_IRQHandler:
  b .

  .thumb_func
  .weak I2C1_ER_IRQHandler
I2C1_ER_IRQHandler:
  b .

  .thumb_func
  .weak I2C2_EV_IRQHandler
I2C2_EV_IRQHandler:
  b .

  .thumb_func
  .weak I2C2_ER_IRQHandler
I2C2_ER_IRQHandler:
  b .

  .thumb_func
  .weak SPI1_IRQHandler
SPI1_IRQHandler:
  b .

  .thumb_func
  .weak SPI2_IRQHandler
SPI2_IRQHandler:
  b .

  .thumb_func
  .weak USART1_IRQHandler
USART1_IRQHandler:
  b .

  .thumb_func
  .weak USART2_IRQHandler
USART2_IRQHandler:
  b .

  .thumb_func
  .weak USART3_IRQHandler
USART3_IRQHandler:
  b .

  .thumb_func
  .weak EXTI15_10_IRQHandler
EXTI15_10_IRQHandler:
  b .

  .thumb_func
  .weak RTCAlarm_IRQHandler
RTCAlarm_IRQHandler:
  b .

  .thumb_func
  .weak USB_FS_WKUP_IRQHandler
USB_FS_WKUP_IRQHandler:
  b .


#endif

/*****************************************************************************
 * Vector Table                                                              *
 *****************************************************************************/

  .section .vectors, "ax"
  .align 0
  .global _vectors
  .extern __stack_end__
  .extern Reset_Handler

_vectors:
  .word __stack_end__
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word SVC_Handler
  .word 0 /* Reserved */
  .word 0 /* Reserved */
  .word PendSV_Handler
  .word SysTick_Handler
  .word WWDG_IRQHandler
  .word PVD_IRQHandler
  .word TAMPER_IRQHandler
  .word RTC_IRQHandler
  .word FLASH_IRQHandler
  .word RCC_IRQHandler
  .word EXTI0_IRQHandler
  .word EXTI1_IRQHandler
  .word EXTI2_IRQHandler
  .word EXTI3_IRQHandler
  .word EXTI4_IRQHandler
  .word DMA1_Channel1_IRQHandler
  .word DMA1_Channel2_IRQHandler
  .word DMA1_Channel3_IRQHandler
  .word DMA1_Channel4_IRQHandler
  .word DMA1_Channel5_IRQHandler
  .word DMA1_Channel6_IRQHandler
  .word DMA1_Channel7_IRQHandler
  .word ADC1_2_IRQHandler
  .word USB_HP_CAN1_TX_IRQHandler
  .word USB_LP_CAN1_RX0_IRQHandler
  .word CAN1_RX1_IRQHandler
  .word CAN1_SCE_IRQHandler
  .word EXTI9_5_IRQHandler
  .word TIM1_BRK_IRQHandler
  .word TIM1_UP_IRQHandler
  .word TIM1_TRG_COM_IRQHandler
  .word TIM1_CC_IRQHandler
  .word TIM2_IRQHandler
  .word TIM3_IRQHandler
  .word TIM4_IRQHandler
  .word I2C1_EV_IRQHandler
  .word I2C1_ER_IRQHandler
  .word I2C2_EV_IRQHandler
  .word I2C2_ER_IRQHandler
  .word SPI1_IRQHandler
  .word SPI2_IRQHandler
  .word USART1_IRQHandler
  .word USART2_IRQHandler
  .word USART3_IRQHandler
  .word EXTI15_10_IRQHandler
  .word RTCAlarm_IRQHandler
  .word USB_FS_WKUP_IRQHandler
_vectors_end:

#ifdef VECTORS_IN_RAM
  .section .vectors_ram, "ax"
  .align 0
  .global _vectors_ram

_vectors_ram:
  .space _vectors_end - _vectors, 0
#endif
