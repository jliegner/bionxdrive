#ifndef _can_h_
#define _can_h_


//----------------------------------------------------------------------------
// Ringbuffer implementation as template
//----------------------------------------------------------------------------

template<class T, uint32_t rbsize>
class RingBuffer
{
  volatile uint32_t cnt;
  volatile uint32_t overruncnt;
  volatile uint32_t rbin;
  volatile uint32_t rbout;
  T        aBuf[rbsize];

public:
        RingBuffer() { rbin=0; rbout=0; overruncnt=0; cnt=0; }

  // store one Element to ringbuffer 
  // no overload detection
  inline void Store(const T &u)
    {
    if ((rbin+1)%rbsize==rbout)
      overruncnt++; 
    aBuf[rbin]=u;
    rbin=(rbin+1)%rbsize;
    cnt++;
    }

  // read next element from ringbuffer
  inline bool GetNext(T *pu)
    {
    //IrqLocker aLck(CEC_CAN_IRQn);
    if (rbin==rbout)
      return false;
    *pu=aBuf[rbout];
    rbout=(rbout+1)%rbsize;
    cnt--;
    return true;
    }
  
  inline bool IsEmpty() { return (rbin==rbout); }
  inline void Reset()   { rbout=rbin;           } 
};


enum eCanFilterMode
{
  CF_LISTMODE_FIFO0_STD=0x0,
  CF_LISTMODE_FIFO1_STD=0x1,
  CF_MASKMODE_FIFO0_STD=0x2,
  CF_MASKMODE_FIFO1_STD=0x3,
  CF_LISTMODE_FIFO0_EXT=0x4,
  CF_LISTMODE_FIFO1_EXT=0x5,
  CF_MASKMODE_FIFO0_EXT=0x6,
  CF_MASKMODE_FIFO1_EXT=0x7
};

class CanMsg
{
public:
  uint32_t id=0;
  bool     ide=0;
  bool     rtr=0;
  bool     bfifo1=0;
  uint8_t  fim=0;     // filter Match
  uint8_t  dlc=0;
  uint8_t  data[8]={0,0,0,0,0,0,0,0};
  uint32_t timestamp=0;

           CanMsg() {};

  bool     Send();
  void     SetUInt16(int idx, uint16_t u);
  void     SetUInt32(int idx, uint32_t u);

  uint16_t GetUInt16(int idx);
  uint32_t GetUInt32(int idx);
};

class Can
{
public:
  void RxIrqHdl();
  RingBuffer<CanMsg, 128>  aRxBuf;
public:
       Can() {}
  
  void Disable();
  void Init(uint32_t baudrate=100000, bool bStart=true, bool bSilent=false);
  void Start();  
  bool Write(CanMsg &rMsg, int timeout_us=0);
  void Read(CanMsg &rMsg, bool bFifo1);
  void SetFilter32(int idx, uint32_t id1, uint32_t id2, eCanFilterMode eMode); 
  void SetFilter16(int idx, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4, eCanFilterMode);  
  void RxCallback(CanMsg &rMsg);
  bool GetNextMsg(CanMsg &rMsg);
};

extern Can aCan;
extern volatile uint32_t uCanTimeTick;

//---------------------------------------------------------------------------

#endif // _CAN_H_


