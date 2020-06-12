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
volatile bool     bTick50ms2;

void SysTick_Handler(void)
{
  uCanTimeTick++;
  TimeTick++;
  Tick50ms++;
  
  // alle Message die nicht im 50ms Raster gesendet werden
  // werden zum Zeitpungt 50ms aber 25ms versetzt gesendet
  if (Tick50ms==25)
    bTick50ms2=1;   
    
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

// Diese Messages werden bei einen realen Rad ganz am Anfang 
// einmal gesendet
const uint8_t PreDriveMsg[][4]=  {{ 0x00, 0x02, 0x00, 0x00 },  // unklar
                                  { 0x00, 0x41, 0x00, 0x00 },  // unklar 
                                  { 0x00, 0x42, 0x00, 0x01 },  // direction ?
                                  { 0x00, 0xA5, 0x00, 0xAA },  // unlock
                                  { 0x00, 0x7A, 0x00, 0x82 },  //
                                  { 0x00, 0x7B, 0x00, 0x38 },  // 7a und 7b Sets maximum current drawn by motor
                                  { 0x00, 0x7C, 0x00, 0x1F },  //
                                  { 0x00, 0x7D, 0x00, 0x40 }}; // 7c und 7d Set maximum current regenerated by motor.


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
    if (aCan.GetNextMsg(aMsg) && aMsg.id==0x08 && aMsg.dlc==4 && aMsg.data[0]==0 && aMsg.data[1]==addr)
      {
      ret=aMsg.data[2]<<8 | aMsg.data[3];
      return true;
      }
    delay_us(100);
    }
  return false;       
}

uint32_t potiadc=0;
int16_t  level=0;

#define BXID_MOTOR            0x20
#define BXR_MOTOR_LEVEL       0x09
#define BXR_MOTOR_WALKLEVEL   0x0A
#define BXR_MOTOR_SPEED       0x11
#define BXR_MOTOR_POWERMETER  0x14
#define BXR_MOTOR_GAUGEVALUE  0x21
#define BXR_MOTOR_VPOWER_HI   0x70  // 300ms
#define BXR_MOTOR_V12V_HI     0x72  // 300ms
#define BXR_MOTOR_SWVERS      0x20  
#define BXR_MOTOR_GAUGETYP    0x6c
#define BXR_MOTOR_TEMP        0x16
#define BXR_MOTOR_CONDITIONS  0x92 // alle 1000ms
#define BXR_MOTOR_STATUS      0x47 // alle 1000ms

uint16_t uGaugeValue;
uint16_t uSpeed;
int16_t  iPowerMeter;
uint16_t uVPower;
uint16_t uV12V;
int16_t  iMotorTemp;
uint16_t uMotorConditions;
uint16_t uMotorStatus;
int16_t  iRealLevel;

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

  // Alle PreDrive-Messages versenden
  for (size_t n=0; n<sizeof(PreDriveMsg)/4; n++)
     {
     CanMsg aCanMsg;
     aCanMsg.id=BXID_MOTOR;
     aCanMsg.dlc=4;
     memcpy(aCanMsg.data, &PreDriveMsg[n][0], 4);   
     aCan.Write(aCanMsg);
     }
  
  delay_ms(50);
  // Danach wird mit 0x20 der Softwarestand des Motors abgefragt (0x66 bei mir)
  uint16_t SwVers=0;
  while (!ReadBionxReg(BXID_MOTOR, BXR_MOTOR_SWVERS, SwVers))
    {
    debug_printf("can not read sw-version from motor!\n");
    delay_ms(200);
    break;
    }
  debug_printf("found motor with sw-vers: %x\n", (uint32_t)SwVers);

  // dann mit 0x6c der Gauge-Typ (bei mir 0x00)
  uint16_t GaugeTyp=0;
  
  bool b=ReadBionxReg(BXID_MOTOR, BXR_MOTOR_GAUGETYP, GaugeTyp);
  debug_printf("read gaugetyp rc=%i typ=%i\n", (int)b, (int)GaugeTyp);

  
  bool b50mSec=false;
  bool b50mSec2=false;
  uint32_t u50mSecCnt=0;
  // zum Verschachteln der Can-Messages in TimeSlots 
  // so soll immer genügend Platz zwischen den Nachrichten bleiben
  #define TIMESLOT(n) (b50mSec2 ? (u50mSecCnt%(n)) : -1)
  while (1)
    {
    // Übernahme der Flags aus dem SystickHdl
    b50mSec=bTick50ms; 
    bTick50ms=false;
    b50mSec2=bTick50ms2; 
    bTick50ms2=false;
    if (b50mSec2)
      u50mSecCnt++;

    // Poti-Wert einlesen udn filtern
    uint32_t adc=ReadAdc(0,8);
    static uint32_t adcfilt=adc;
    adcfilt-=adcfilt/32;
    adcfilt+=adc;
    adc=adcfilt/32;    
    potiadc=adc;


    if (TIMESLOT(10)==0) // 10*50ms=500ms
      {
      debug_printf("adc: %i\n", (int)potiadc);
      }
   
    uint16_t value=0;
    
    // Speed alle 100ms lesen
    if (TIMESLOT(2)==1) 
      {
      if (ReadBionxReg(BXID_MOTOR, BXR_MOTOR_SPEED, value))
        {
        uSpeed=value;
        }
      }

    // PowerMeter alle 100ms
    if (TIMESLOT(2)==0) 
      {
      if (ReadBionxReg(BXID_MOTOR, BXR_MOTOR_POWERMETER, value))
        {
        iPowerMeter=(int16_t)value;
        }
      }

    // Spannung 36V(48V) alle 300ms
    if (TIMESLOT(6)==1) 
      {
      if (ReadBionxReg(BXID_MOTOR, BXR_MOTOR_VPOWER_HI, value))
        {
        uVPower=(int16_t)value;
        }
      }

    // Spannung 12V alle 300ms
    if (TIMESLOT(6)==2) 
      {
      if (ReadBionxReg(BXID_MOTOR, BXR_MOTOR_V12V_HI, value))
        {
        uV12V=(int16_t)value;
        }
      }

    // Motortemperatur alle 1000ms
    if (TIMESLOT(20)==1) 
      {
      if (ReadBionxReg(BXID_MOTOR, BXR_MOTOR_TEMP, value))
        {
        iMotorTemp=(int16_t)value;
        }
      }

    // Motor Condition Flags alle 1000ms
    if (TIMESLOT(20)==2) 
      {
      if (ReadBionxReg(BXID_MOTOR, BXR_MOTOR_CONDITIONS, value))
        {
        uMotorConditions=(int16_t)value;
        }
      }
    // Motor Condition Flags alle 1000ms
    if (TIMESLOT(20)==3) 
      {
      if (ReadBionxReg(BXID_MOTOR, BXR_MOTOR_STATUS, value))
        {
        uMotorStatus=(int16_t)value;
        }
      }

    if (b50mSec)
      {
      level=0;
      #define MAX_LEVEL 20
      #define MAX_REKU  30
      if (potiadc>2300)
        {
        // Unterstützung von 0-MAX_LEVEL
        level=((potiadc-2300)*MAX_LEVEL)/(4096-2300);
        }
      if (potiadc<1800)
        {
        // Unterstützung von 0 - -MAX_REKU
        level=((1800-potiadc)*MAX_REKU)/1800;
        level*=-1;
        }

      if (aSwitch.In())
        {
        iRealLevel=0;
        }
      else
        {
        if (iRealLevel<level)
          iRealLevel++; 
        if (iRealLevel>level)
          iRealLevel--; 
        if (level==0)
          iRealLevel=0;
        }

      WriteBionxReg(BXID_MOTOR, BXR_MOTOR_LEVEL, iRealLevel);
      WriteBionxReg(BXID_MOTOR, BXR_MOTOR_LEVEL, iRealLevel);
      WriteBionxReg(BXID_MOTOR, BXR_MOTOR_WALKLEVEL, 0);
      ReadBionxReg(BXID_MOTOR, BXR_MOTOR_GAUGEVALUE, uGaugeValue);
      }
    }
    
  return 0;
}  

