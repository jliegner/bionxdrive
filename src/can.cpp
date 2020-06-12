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

#include <incall.h>

void Can::Disable()
{
  RCC->APB1RSTR |=  RCC_APB1RSTR_CAN1RST;           // reset CAN
  NVIC_DisableIRQ(CAN1_RX1_IRQn);
}

// passt fuer einen SystemCoreClock von 72MHz (STM32F103) 
// der ClockIn fur den CAN Teil ist 36MHz
// nicht fuer 48MHz wie bei STM32F042 verwenden
volatile uint32_t uCanTimeTick=0; 
void Can::Init(uint32_t baudrate, bool bStart, bool bSilent)
{
  uCanTimeTick=0;

  // der Takt kommt von APB1 mit max 36MHz bei 72MHz ist da ein Vorteiler
  // von 2 drin
  uint32_t brp = SystemCoreClock/2;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;  
  RCC->APB1ENR  |= RCC_APB1ENR_CAN1EN;           // enable clock for CAN

                                                 // Note: MCBSTM32 uses PB8 and PB9 for CAN
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;            // enable clock for Alternate Function
  AFIO->MAPR   &= 0xFFFF9FFF;                    // reset CAN remap
  AFIO->MAPR   |= 0x00004000;                    //   set CAN remap, use PB8, PB9

  CAN1->MCR = (CAN_MCR_NART | CAN_MCR_INRQ | CAN_MCR_ABOM); // init mode, disable auto. retransmission, busoff autorecovery
                                                  
  CAN1->IER = (CAN_IER_FMPIE0 | CAN_IER_FMPIE1); // | CAN_IER_TMEIE); // FIFO 0/1 msg pending, Transmit mbx empty
  brp  = (brp / 18) / baudrate;                  // baudrate 
                                                                          
  // set BTR register so that sample point is at about 72% bit time from bit start 
  // TSEG1 = 12, TSEG2 = 5, SJW = 4 => 1 CAN bit = 18 TQ, sample at 72%    
  CAN1->BTR &= ~(((        0x03) << 24) | ((        0x07) << 20) | ((         0x0F) << 16) | (          0x1FF)); 
  CAN1->BTR |=  ((((4-1) & 0x03) << 24) | (((5-1) & 0x07) << 20) | (((12-1) & 0x0F) << 16) | ((brp-1) & 0x1FF));
  if (bSilent)
    CAN1->BTR|=CAN_BTR_SILM;
  
  if (bStart)
    Start();
}

void Can::Start()
{
  uCanTimeTick=0;
  CAN1->MCR &= ~CAN_MCR_INRQ;        // normal operating mode, reset INRQ
  while (CAN1->MSR & CAN_MCR_INRQ)
    ;

  NVIC_SetPriority(CAN1_RX1_IRQn, 2);
  NVIC_EnableIRQ(CAN1_RX1_IRQn);
}

bool Can::Write(CanMsg &rMsg, int timeout_us)
{
  CAN_TxMailBox_TypeDef *pM=0;

  while (1)
    {
    if ((CAN1->TSR & CAN_TSR_TME0) != 0)
      pM=&CAN1->sTxMailBox[0];
    else if ((CAN1->TSR & CAN_TSR_TME1) != 0)
      pM=&CAN1->sTxMailBox[1];
    else if ((CAN1->TSR & CAN_TSR_TME2) != 0)
      pM=&CAN1->sTxMailBox[2];

    if (pM)
      break;
    if (timeout_us>0)
      { delay_us(1); timeout_us--; }
    else
      return false;
    }

  pM->TIR=0;
  // Setup identifier information
  if (!rMsg.ide)  
    pM->TIR = (uint32_t)(rMsg.id << 21); // Standard ID
  else
    pM->TIR |= (uint32_t)(rMsg.id <<  3) | CAN_TI0R_IDE; // Extended ID
    
  // Setup data bytes
  pM->TDLR = (((uint32_t)rMsg.data[3] << 24) | 
              ((uint32_t)rMsg.data[2] << 16) |
              ((uint32_t)rMsg.data[1] <<  8) | 
              ((uint32_t)rMsg.data[0])        );

  pM->TDHR = (((uint32_t)rMsg.data[7] << 24) | 
              ((uint32_t)rMsg.data[6] << 16) |
              ((uint32_t)rMsg.data[5] <<  8) |
              ((uint32_t)rMsg.data[4])        );
  // Setup length
  pM->TDTR =  (rMsg.dlc & CAN_TDT0R_DLC);

  // transmit 
  if (!rMsg.rtr)  
    pM->TIR |= CAN_TI0R_TXRQ; // DATA FRAME
  else 
    pM->TIR |= CAN_TI0R_RTR; // REMOTE FRAME
  return true;
}

void Can::SetFilter32(int idx, uint32_t id1, uint32_t id2, eCanFilterMode eMode)  
{
  uint32_t  CAN_msgId1 = 0;
  uint32_t  CAN_msgId2 = 0;
  
  if (idx<0 || idx>13)
    return;
  
  // std or ext message
  if ((eMode & 0b100) == 0)  
    { // Standard ID
    CAN_msgId1  |= (uint32_t)(id1 << 21);
    CAN_msgId2  |= (uint32_t)(id2 << 21);
    }  
  else  
    {  // Extended ID
    CAN_msgId1  |= (uint32_t)(id1 <<  3) | CAN_TI0R_IDE;
    CAN_msgId2  |= (uint32_t)(id2 <<  3) | CAN_TI0R_IDE;
    }

  CAN1->FMR  |=  CAN_FMR_FINIT;                    // set Initialisation mode for filter banks
  CAN1->FA1R &=  ~(uint32_t)(1 << idx);            // deactivate filter

                                                   // initialize filter   
  CAN1->FS1R |= (uint32_t)(1 << idx);              // set 32-bit scale configuration
  // list or mask Mode ?
  if ((eMode & 0b010) == 0)  
    CAN1->FM1R |= (uint32_t)(1 << idx);            // set 2 32-bit identifier list mode
  else
    CAN1->FM1R &= ~((uint32_t)(1 << idx));         // set 2 32-bit identifier mask mode

  CAN1->sFilterRegister[idx].FR1 = CAN_msgId1;     //  32-bit identifier
  CAN1->sFilterRegister[idx].FR2 = CAN_msgId2;     //  32-bit identifier
    													   
  if ((eMode & 0b001) == 0)  
    CAN1->FFA1R &= ~(uint32_t)(1 << idx);  // assign filter to FIFO 0
  else
    CAN1->FFA1R |= (uint32_t)(1 << idx);   // assign filter to FIFO 1

  CAN1->FA1R  |=  (uint32_t)(1 << idx);  // activate filter
  CAN1->FMR &= ~CAN_FMR_FINIT;           // reset Initialisation mode for filter banks
}

void Can::SetFilter16(int idx, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, eCanFilterMode eMode)  
{
  uint32_t  CAN_msgId1 = 0;
  uint32_t  CAN_msgId2 = 0;
  
  if (idx<0 || idx>13)
    return;
                                                  // Setup identifier information
  // std or ext message
  if ((eMode & 0b100) == 0)  
    { // Standard ID
    CAN_msgId1=(uint32_t)(id1 << 5) | (id2<<21);
    CAN_msgId2=(uint32_t)(id3 << 5) | (id4<<21);
    }  
  else  
    { // Extended ID
    uint32_t r1=(uint32_t)((id1>>13) & 0xffffffe0) | ((id1>>15) & 0x1f) | (1<<3);
    uint32_t r2=(uint32_t)((id2>>13) & 0xffffffe0) | ((id2>>15) & 0x1f) | (1<<3);
    CAN_msgId1=(r2 & 0xffff)<<16 | (r1 & 0xffff);

    r1=(uint32_t)((id3>>13) & 0xffffffe0) | ((id3>>15) & 0x1f) | (1<<3);
    r2=(uint32_t)((id4>>13) & 0xffffffe0) | ((id4>>15) & 0x1f) | (1<<3);
    CAN_msgId2=(r2 & 0xffff)<<16 | (r1 & 0xffff);
    }

  CAN1->FMR  |=  CAN_FMR_FINIT;            // set Initialisation mode for filter banks
  CAN1->FA1R &=  ~(uint32_t)(1 << idx);    // deactivate filter

                                           // initialize filter   
  CAN1->FS1R &= ~(uint32_t)(1 << idx);     // set 16-bit scale configuration
  // list or mask Mode ?
  if ((eMode & 0b010) == 0)  
    CAN1->FM1R |= (uint32_t)(1 << idx);            
  else
    CAN1->FM1R &= ~((uint32_t)(1 << idx));         

  CAN1->sFilterRegister[idx].FR1 = CAN_msgId1;     
  CAN1->sFilterRegister[idx].FR2 = CAN_msgId2;     
    													   
  if ((eMode & 0b001) == 0)  
    CAN1->FFA1R &= ~(uint32_t)(1 << idx);  // assign filter to FIFO 0
  else
    CAN1->FFA1R |= (uint32_t)(1 << idx);   // assign filter to FIFO 1
  CAN1->FA1R  |=  (uint32_t)(1 << idx);    // activate filter

  CAN1->FMR &= ~CAN_FMR_FINIT;             // reset Initialisation mode for filter banks
}

void Can::Read(CanMsg &rMsg, bool bFifo1)  
{
  rMsg.timestamp=uCanTimeTick;
  rMsg.bfifo1=bFifo1;
  CAN_FIFOMailBox_TypeDef *pM=bFifo1 ? &CAN1->sFIFOMailBox[1] : &CAN1->sFIFOMailBox[0];
  
  // Read identifier information
  if ((pM->RIR & CAN_TI0R_IDE) == 0) 
    { 
    // Standard ID
    rMsg.ide=false;
    rMsg.id=(uint32_t)0x000007FF & (pM->RIR >> 21);
    }  
  else  
    {                                          
    // Extended ID
    rMsg.ide=true;
    rMsg.id=(uint32_t)(pM->RIR >> 3);
    }

  // Read type information
  if ((pM->RIR & CAN_TI0R_RTR) == 0) 
    rMsg.rtr=false;  // DATA FRAME
  else  
    rMsg.rtr=true;  // REMOTE FRAME
  
  // Read length (number of received bytes)
  rMsg.dlc=(unsigned char)0x0000000F & pM->RDTR;
  rMsg.fim=(unsigned char)0x000000FF & pM->RDTR>>9; //?
  // Read data bytes
  rMsg.data[0] = (uint32_t)0x000000FF & (pM->RDLR);
  rMsg.data[1] = (uint32_t)0x000000FF & (pM->RDLR >> 8);
  rMsg.data[2] = (uint32_t)0x000000FF & (pM->RDLR >> 16);
  rMsg.data[3] = (uint32_t)0x000000FF & (pM->RDLR >> 24);

  rMsg.data[4] = (uint32_t)0x000000FF & (pM->RDHR);
  rMsg.data[5] = (uint32_t)0x000000FF & (pM->RDHR >> 8);
  rMsg.data[6] = (uint32_t)0x000000FF & (pM->RDHR >> 16);
  rMsg.data[7] = (uint32_t)0x000000FF & (pM->RDHR >> 24);

  // Release FIFO 0 output mailbox
  if (bFifo1)
    CAN1->RF1R |= CAN_RF1R_RFOM1;                    
  else
    CAN1->RF0R |= CAN_RF0R_RFOM0;  
}

__attribute__ ((weak))
void Can::RxCallback(CanMsg &rMsg)
{
}

bool Can::GetNextMsg(CanMsg &rMsg)
{
  IrqLocker aIrqLck(CAN1_RX1_IRQn);
  return aRxBuf.GetNext(&rMsg);
}

void Can::RxIrqHdl()
{
  CanMsg aRxMsg;
  Read(aRxMsg, true);    // read the message
  RxCallback(aRxMsg);
  aRxBuf.Store(aRxMsg);
}

Can aCan;

void CAN1_RX1_IRQHandler (void) 
{
  CanMsg aRxMsg;
  if ((CAN1->RF1R & CAN_RF1R_FMP1) != 0) 
    {		                // message pending ?
    aCan.RxIrqHdl();
    }
}

