#ifndef GPIO_H
  #define GPIO_H

#pragma GCC push_options
#pragma GCC optimize ("O3")
  
#ifdef __cplusplus
extern "C" {

enum eGpioInOutMode
{
  GPIO_IN_ANALOG        = 0x00,
  GPIO_IN_FLOAT         = 0x01,
  GPIO_IN_PULLUPDOWN    = 0x02,

  GPIO_OUT_NORMAL       = 0x00,
  GPIO_OUT_OPENDRAIN    = 0x01,
  GPIO_OUT_ALTNORMAL    = 0x02,
  GPIO_OUT_ALTOPENDRAIN = 0x03
};

enum eGpioMode
{
  GPIO_IN               = 0x00,
  GPIO_OUT_10MHZ        = 0x01,
  GPIO_OUT_2MHZ         = 0x02,
  GPIO_OUT_50MHZ        = 0x03
};

enum eGpioPo
{
  PA = GPIOA_BASE,
  PB = GPIOB_BASE,
  PC = GPIOC_BASE,
  PD = GPIOD_BASE,
};  

enum eGpioState
{
  GPIO_INIT,
  GPIO_SET,
  GPIO_CLR,
  GPIO_UNINIT
};  


#else


#define GPIO_IN_ANALOG        0x00
#define GPIO_IN_FLOAT         0x01
#define GPIO_IN_PULLUPDOWN    0x02

#define GPIO_OUT_NORMAL       0x00
#define GPIO_OUT_OPENDRAIN    0x01
#define GPIO_OUT_ALTNORMAL    0x02
#define GPIO_OUT_ALTOPENDRAIN 0x03

#define GPIO_IN               0x00
#define GPIO_OUT_10MHZ        0x01
#define GPIO_OUT_2MHZ         0x02
#define GPIO_OUT_50MHZ        0x03

#define PA GPIOA
#define PB GPIOB
#define PC GPIOC
#define PD GPIOD

#endif


void GPIOSetPinMode( GPIO_TypeDef *port, uint8_t bitPosi, uint8_t dirmode, uint8_t mode);
void GPIOSetDir( GPIO_TypeDef *port, uint32_t bitPosi, uint32_t dir );

#define GPIO_SET(a,b)     (((GPIO_TypeDef *)(a))->BSRR  =  (1<<b)) 
#define GPIO_CLR(a,b)     (((GPIO_TypeDef *)(a))->BRR   =  (1<<b)) 
#define GPIO_IS_SET(a,b)  ((((GPIO_TypeDef *)(a))->IDR & (1<<b)) !=0)

static __INLINE void GPIOSet( GPIO_TypeDef * port, uint32_t bitPosi)
{
  GPIO_SET(port,bitPosi);
}

static __INLINE void GPIOClr( GPIO_TypeDef * port, uint32_t bitPosi)
{
  GPIO_CLR(port,bitPosi);
}

static __INLINE int GPIOIsSet( GPIO_TypeDef * port, uint32_t bitPosi)
{
  return GPIO_IS_SET(port,bitPosi);
}

#ifdef __cplusplus
}

template<eGpioPo ePort, uint32_t bitPosi, eGpioMode mode=GPIO_IN, eGpioInOutMode inoutmode=GPIO_IN_ANALOG>
class GpioPin 
{
  // inline GPIO_TypeDef *GetPort() { return (GPIO_TypeDef *)ePort; }
  constexpr GPIO_TypeDef * GetPort() { return (reinterpret_cast<GPIO_TypeDef*>(ePort)); }   
public:
  inline  GpioPin(eGpioState eState=GPIO_INIT)    
    { 
    switch (eState)
      {
      case GPIO_INIT: { Init(); return; }
      case GPIO_SET:  { Init(); Set(); return; }
      case GPIO_CLR:  { Init(); Clr(); return; }
      default:        { return; }
      }
    }
  
  inline void Init()            
    {
    uint32_t m=(inoutmode&0x03)<<2;
    m|=(mode&0x03);
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; 
    switch (ePort)
      {
      case PA: RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  break;
      case PB: RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  break;
      case PC: RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;  break;
      case PD: RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;  break;
      }

    if (bitPosi >=8)
      {
      uint32_t r=GetPort()->CRH;
      r&= ~(0b1111<<(((bitPosi-8)&0x7)*4));
      r|= ((m & 0b1111)<<(((bitPosi-8)&0x7)*4));
      GetPort()->CRH=r;
      }
    else
      {
      uint32_t r=GetPort()->CRL;
      r&= ~(0b1111<<((bitPosi&0x7)*4));
      r|= ((m & 0b1111)<<((bitPosi&0x7)*4));
      GetPort()->CRL=r;
      }
    }
   
  inline void Set()             { GetPort()->BSRR  = (1<<bitPosi); }
  inline void Clr()             { GetPort()->BRR   = (1<<bitPosi); }
  inline void Toggle()          { (((GetPort()->ODR & (1<<bitPosi))) !=0) ? Clr() : Set(); }
  inline void SetValue(bool v)  { v ? Set() : Clr();   } 
  inline bool IsSet()           { return (GetPort()->ODR & (1<<bitPosi)) !=0; }
  inline bool In()              { return (GetPort()->IDR & (1<<bitPosi)) !=0; }
};

#endif
#pragma GCC pop_options
#endif
