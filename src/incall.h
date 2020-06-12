#ifndef INCALL_H
  #define INCALL_H

#include <stm32f10x.h>
#include <irq_stm32f103.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

#define debug_printf printf

//---------------------------------------------------------------------------
// Delay ueber DWT-CycleCounter 
// SystemCoreClock wurde als constexpr definiert damit 
// die Berechnungen im Compiler und nicht zur laufzeit erfolgen
//---------------------------------------------------------------------------
constexpr uint32_t SystemCoreClock=72000000;

#define DWT_CTRL   (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t*)0xE0001004)
#define SCB_DEMCR  (*(volatile uint32_t*)0xE000EDFC)

#pragma GCC push_options
#pragma GCC optimize ("O3")
__attribute__((always_inline)) 
inline void delay_dwt(const uint32_t ticks) 
{
  uint32_t start = DWT_CYCCNT;
  while((DWT_CYCCNT - start) < ticks)
    ;
}

#define delay_us(us) delay_dwt((SystemCoreClock/1000000)*(us))
#define delay_ms(ms) delay_dwt((SystemCoreClock/1000)*(ms))

inline void InitDwtDelay()
{
  // enable Cycle Counter in DWT for delay_us
  SCB_DEMCR |= 0x01000000;
  DWT_CTRL |= 0x01;
  DWT_CYCCNT=0;
}

//---------------------------------------------------------------------------
// simple Helperclasse zum Irq-Sperren ohne dass man sich um das entsperren
// kümmern muss da es im Destruktor automatisch passiert
//---------------------------------------------------------------------------

class IrqLocker
{
  IRQn_Type irqnr;
public:
     IrqLocker(IRQn_Type IrqNr) { irqnr=IrqNr; NVIC_DisableIRQ(irqnr); }
    ~IrqLocker()                { NVIC_EnableIRQ(irqnr);               }
};

//---------------------------------------------------------------------------

#pragma GCC pop_options

// extern uint32_t SystemCoreClock;
#define SYSTICKFREQ      1000
extern volatile uint32_t TimeTick;

#include <gpio.h>
#include <can.h>

#endif
