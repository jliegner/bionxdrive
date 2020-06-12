/* ====================================================================
 *
 * Copyright (c) 2020 Juergen Liegner  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * 3. Neither the name of the author(s) nor the names of any contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR(S) AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR(S) OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * ====================================================================*/

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

#define debug_printf(...) 
#ifdef DEBUG
   #undef debug_printf
   #include "debugio.h"
#endif   

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
