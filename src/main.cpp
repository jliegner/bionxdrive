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

#include "incall.h"

//-------------------------------------------------------------------
// LED und CAN Pins werden schon vor main() im Constructor initialisiert
//-------------------------------------------------------------------

// onboard Led
GpioPin<PC, 13, GPIO_OUT_2MHZ, GPIO_OUT_NORMAL>      aLed(GPIO_SET);

// CAN
GpioPin<PB,  9, GPIO_OUT_50MHZ, GPIO_OUT_ALTNORMAL>  aCanTx; 
GpioPin<PB,  8, GPIO_IN, GPIO_IN_FLOAT>              aCanRx;

GpioPin<PA,  0, GPIO_IN, GPIO_IN_ANALOG>             aAdc_A0;
GpioPin<PA,  1, GPIO_IN, GPIO_IN_PULLUPDOWN>         aSwitch(GPIO_SET);

//---------------------------------------------------------------------------
// 
//---------------------------------------------------------------------------
void ClockConfig()
{
  // Resetvalue laut Datenblatt
  RCC->CR=0x81;
  
  // warten bis HSI an
  while ((RCC->CR & RCC_CR_HSIRDY)==0)  
    ;

  // externen 8 MHz Oszillator HSE einschalten 
  RCC->CR |= RCC_CR_HSEON;       

  while ((RCC->CR & RCC_CR_HSERDY)==0)  
    ;

  // Prefetchbuffer und 2 Waitstates
  FLASH->ACR=FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

  RCC->CFGR=  RCC_CFGR_HPRE_DIV1    // 
            | RCC_CFGR_PPRE1_DIV2   // ABP1 36MHz
            | RCC_CFGR_PPRE2_DIV1   // ABP2 72MHz
            | RCC_CFGR_ADCPRE_DIV8  // ADC prescaler
            | RCC_CFGR_PLLMULL9     // PLL * 9
            | RCC_CFGR_PLLSRC_HSE;  // HSE fuer PLL

  // PLL einschalten
  RCC->CR |= RCC_CR_PLLON;
  while ((RCC->CR & RCC_CR_PLLRDY)==0)
    ;

  // PLL als Taktquelle
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & 0b1100) != RCC_CFGR_SWS_PLL)
    ;
}

volatile uint32_t TimeTick;
volatile uint32_t Tick50ms;
volatile bool     bTick50ms;

void SysTick_Handler(void)
{
  uCanTimeTick++;
  TimeTick++;
  Tick50ms++;
  
  if (Tick50ms>=50)
    {
    bTick50ms=1;
    Tick50ms=0;
    }
}


void InitAdc()
{
  // Enable clock for ADC
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  
  // Adc On and software start trigger
  ADC1->CR2|=ADC_CR2_ADON | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;
  // 41us SampleTime
  ADC1->SMPR2|=ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_2;
  delay_ms(20);
  
  // ADC kalibrieren
  ADC1->CR2|=ADC_CR2_CAL;
  while ((ADC1->CR2 & ADC_CR2_CAL)!=0)
    ;
}

uint16_t ReadAdc(int channel, int cnt=1)
{
  uint32_t adc=0;
  // 1 Kanal convert
  ADC1->SQR1=0; 
  ADC1->SQR3=channel&0x1f;

  for (int n=0; n<cnt; n++)
    {
    ADC1->SR&=~ADC_SR_EOC;
    ADC1->CR2|=ADC_CR2_ADON;
    ADC1->CR2|=ADC_CR2_SWSTART;
    while ((ADC1->SR & ADC_SR_EOC)==0)
      ;
    adc+=ADC1->DR & 0xfff;
    }
  return adc/cnt;
}

bool WriteBionxReg(int canid, uint16_t addr, uint16_t value)
{
  CanMsg aMsg;
  aMsg.id=canid;
  aMsg.dlc=4;
  aMsg.data[0]=0;
  aMsg.data[1]=addr;
  aMsg.data[2]=(value>>8) & 0xff;
  aMsg.data[3]=value & 0xff;
  return aCan.Write(aMsg, 10000);
}

// Request with 10ms Timeout
bool ReadBionxReg(int canid, uint16_t addr, uint16_t &ret )
{
  CanMsg aMsg;
  aMsg.id=canid;
  aMsg.dlc=2;
  aMsg.data[0]=0;
  aMsg.data[1]=addr;
  aCan.Write(aMsg);
  
  for (int n=0; n<100; n++)
    {
    // Alle Requests an den Motor werden mit der id 0x008 beantwortet un haben immer eine 
    // Länge von 4 Byte, das erste Byte ist immer 0 
    if (aCan.GetNextMsg(aMsg) && aMsg.id==0x08 && aMsg.dlc==4 && aMsg.data[0]==0 && aMsg.data[1]==addr)
      {
      ret=aMsg.data[2]<<8 | aMsg.data[3];
      return true;
      }
    delay_us(100); 
    }
  return false;       
}

uint32_t uPotiAdc=0;
int16_t  iRealLevel;
uint8_t  uSwitch=0;

#define BXID_MOTOR            0x20
#define BXR_MOTOR_LEVEL       0x09
#define BXR_MOTOR_SWVERS      0x20  


int main(void)
{
  aLed.Set();
  ClockConfig();
  InitDwtDelay();
  InitAdc();

  SysTick_Config(SystemCoreClock/SYSTICKFREQ);
  
  aCan.Init(125000, false); // Init, not start
  aCan.SetFilter32(1, 0x00, 0x00, CF_MASKMODE_FIFO1_STD);
  aCan.Start();

  delay_ms(50);

  // Mit 0x20 der Softwarestand des Motors abfragen (0x66 bei mir)
  uint16_t SwVers=0;
  while (!ReadBionxReg(BXID_MOTOR, BXR_MOTOR_SWVERS, SwVers))
    {
    debug_printf("can not read sw-version from motor!\n");
    delay_ms(200);
    aLed.Toggle();
    }
  aLed.Clr();
  debug_printf("found motor with sw-vers: %i\n", (uint32_t)SwVers);

  bool b50mSec=false;
  while (1)
    {
    // Uebernahme der Flags aus dem SystickHdl
    b50mSec=bTick50ms; 
    bTick50ms=false;

    // Poti-Wert einlesen udn filtern
    uint32_t adc=ReadAdc(0,8);
    static uint32_t adcfilt=adc;
    adcfilt-=adcfilt/32;
    adcfilt+=adc;
    adc=adcfilt/32;    
    uPotiAdc=adc;

    uSwitch<<=1;
    if (aSwitch.In())
      uSwitch|=0x01;
 
    if (b50mSec)
      {      
      int16_t  level=0; 
      // Willkuerliche Festlegung der max und min Werte damit 
      // mir das Rad nicht umfaellt
      #define MAX_LEVEL 20
      #define MAX_REKU  30
      if (uPotiAdc>2300)
        {
        // Unterstuetzung von 0-MAX_LEVEL
        level=((uPotiAdc-2300)*MAX_LEVEL)/(4096-2300);
        }
      if (uPotiAdc<1800)
        {
        // Bremsen/Reku von 0 - -MAX_REKU
        level=((1800-uPotiAdc)*MAX_REKU)/1800;
        level*=-1;
        }

      if (uSwitch==0xff)
        {
        iRealLevel=0;
        }
      if (uSwitch==0)
        {
        // kurze Rampe fahren   
        if (iRealLevel<level)
          iRealLevel++; 
        if (iRealLevel>level)
          iRealLevel--; 
        if (level==0)
          iRealLevel=0;
        }
      
      // Msg 2 mal hintereinander zum Motor senden. Wird jedenfalls 
      // bei BionX so gemacht. 
      WriteBionxReg(BXID_MOTOR, BXR_MOTOR_LEVEL, iRealLevel);
      WriteBionxReg(BXID_MOTOR, BXR_MOTOR_LEVEL, iRealLevel);
      }
    }
    
  return 0;
}  

