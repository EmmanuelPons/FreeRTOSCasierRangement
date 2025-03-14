    typedef unsigned int size_t;    
    typedef unsigned short wchar_t;  
typedef struct div_t { int quot, rem; } div_t;
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
typedef struct lldiv_t { long long quot, rem; } lldiv_t;
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);
extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long long atoll(const char *  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long long strtoll(const char * __restrict  ,
                                  char ** __restrict  , int  )
                          __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) unsigned long long strtoull(const char * __restrict  ,
                                            char ** __restrict  , int  )
                                   __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int rand(void);
extern __declspec(__nothrow) void srand(unsigned int  );
struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
extern __declspec(__nothrow) void free(void *  );
extern __declspec(__nothrow) void *malloc(size_t  );
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int  system(const char *  );
extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
extern __declspec(__nothrow) __attribute__((const)) int abs(int  );
extern __declspec(__nothrow) __attribute__((const)) div_t div(int  , int  );
extern __declspec(__nothrow) __attribute__((const)) long int labs(long int  );
extern __declspec(__nothrow) __attribute__((const)) ldiv_t ldiv(long int  , long int  );
extern __declspec(__nothrow) __attribute__((const)) long long llabs(long long  );
extern __declspec(__nothrow) __attribute__((const)) lldiv_t lldiv(long long  , long long  );
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);
extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);
    typedef unsigned int size_t;    
extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));
extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));
extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) char *strerror(int  );
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
  typedef signed int ptrdiff_t;
    typedef unsigned int size_t;    
      typedef unsigned short wchar_t;  
  typedef long double max_align_t;
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;
typedef enum IRQn
{
  Reset_IRQn                    = -15,       
  NonMaskableInt_IRQn           = -14,       
  HardFault_IRQn                = -13,       
  MemoryManagement_IRQn         = -12,       
  BusFault_IRQn                 = -11,       
  UsageFault_IRQn               = -10,       
  SVCall_IRQn                   = -5,        
  DebugMonitor_IRQn             = -4,        
  PendSV_IRQn                   = -2,        
  SysTick_IRQn                  = -1,        
  WDT_IRQn                      = 0,         
  TIMER0_IRQn                   = 1,         
  TIMER1_IRQn                   = 2,         
  TIMER2_IRQn                   = 3,         
  TIMER3_IRQn                   = 4,         
  UART0_IRQn                    = 5,         
  UART1_IRQn                    = 6,         
  UART2_IRQn                    = 7,         
  UART3_IRQn                    = 8,         
  PWM1_IRQn                     = 9,         
  I2C0_IRQn                     = 10,        
  I2C1_IRQn                     = 11,        
  I2C2_IRQn                     = 12,        
  SPI_IRQn                      = 13,        
  SSP0_IRQn                     = 14,        
  SSP1_IRQn                     = 15,        
  PLL0_IRQn                     = 16,        
  RTC_IRQn                      = 17,        
  EINT0_IRQn                    = 18,        
  EINT1_IRQn                    = 19,        
  EINT2_IRQn                    = 20,        
  EINT3_IRQn                    = 21,        
  ADC_IRQn                      = 22,        
  BOD_IRQn                      = 23,        
  USB_IRQn                      = 24,        
  CAN_IRQn                      = 25,        
  DMA_IRQn                      = 26,        
  I2S_IRQn                      = 27,        
  ENET_IRQn                     = 28,        
  RIT_IRQn                      = 29,        
  MCPWM_IRQn                    = 30,        
  QEI_IRQn                      = 31,        
  PLL1_IRQn                     = 32,        
  USBActivity_IRQn              = 33,        
  CANActivity_IRQn              = 34,        
} IRQn_Type;
__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}
__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}
typedef union
{
  struct
  {
    uint32_t _reserved0:27;               
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:15;               
    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;
typedef struct
{
  volatile uint32_t ISER[8];                  
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];                  
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];                  
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];                  
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];                  
       uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];                  
       uint32_t RESERVED5[644];
  volatile  uint32_t STIR;                     
}  NVIC_Type;
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
  volatile uint32_t VTOR;                     
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
  volatile uint8_t  SHP[12];                  
  volatile uint32_t SHCSR;                    
  volatile uint32_t CFSR;                     
  volatile uint32_t HFSR;                     
  volatile uint32_t DFSR;                     
  volatile uint32_t MMFAR;                    
  volatile uint32_t BFAR;                     
  volatile uint32_t AFSR;                     
  volatile const  uint32_t PFR[2];                   
  volatile const  uint32_t DFR;                      
  volatile const  uint32_t ADR;                      
  volatile const  uint32_t MMFR[4];                  
  volatile const  uint32_t ISAR[5];                  
       uint32_t RESERVED0[5];
  volatile uint32_t CPACR;                    
} SCB_Type;
typedef struct
{
       uint32_t RESERVED0[1];
  volatile const  uint32_t ICTR;                     
       uint32_t RESERVED1[1];
} SCnSCB_Type;
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                   
    volatile  uint16_t   u16;                  
    volatile  uint32_t   u32;                  
  }  PORT [32];                           
       uint32_t RESERVED0[864];
  volatile uint32_t TER;                      
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;                      
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;                      
       uint32_t RESERVED3[29];
  volatile  uint32_t IWR;                      
  volatile const  uint32_t IRR;                      
  volatile uint32_t IMCR;                     
       uint32_t RESERVED4[43];
  volatile  uint32_t LAR;                      
  volatile const  uint32_t LSR;                      
       uint32_t RESERVED5[6];
  volatile const  uint32_t PID4;                     
  volatile const  uint32_t PID5;                     
  volatile const  uint32_t PID6;                     
  volatile const  uint32_t PID7;                     
  volatile const  uint32_t PID0;                     
  volatile const  uint32_t PID1;                     
  volatile const  uint32_t PID2;                     
  volatile const  uint32_t PID3;                     
  volatile const  uint32_t CID0;                     
  volatile const  uint32_t CID1;                     
  volatile const  uint32_t CID2;                     
  volatile const  uint32_t CID3;                     
} ITM_Type;
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t CYCCNT;                   
  volatile uint32_t CPICNT;                   
  volatile uint32_t EXCCNT;                   
  volatile uint32_t SLEEPCNT;                 
  volatile uint32_t LSUCNT;                   
  volatile uint32_t FOLDCNT;                  
  volatile const  uint32_t PCSR;                     
  volatile uint32_t COMP0;                    
  volatile uint32_t MASK0;                    
  volatile uint32_t FUNCTION0;                
       uint32_t RESERVED0[1];
  volatile uint32_t COMP1;                    
  volatile uint32_t MASK1;                    
  volatile uint32_t FUNCTION1;                
       uint32_t RESERVED1[1];
  volatile uint32_t COMP2;                    
  volatile uint32_t MASK2;                    
  volatile uint32_t FUNCTION2;                
       uint32_t RESERVED2[1];
  volatile uint32_t COMP3;                    
  volatile uint32_t MASK3;                    
  volatile uint32_t FUNCTION3;                
} DWT_Type;
typedef struct
{
  volatile uint32_t SSPSR;                    
  volatile uint32_t CSPSR;                    
       uint32_t RESERVED0[2];
  volatile uint32_t ACPR;                     
       uint32_t RESERVED1[55];
  volatile uint32_t SPPR;                     
       uint32_t RESERVED2[131];
  volatile const  uint32_t FFSR;                     
  volatile uint32_t FFCR;                     
  volatile const  uint32_t FSCR;                     
       uint32_t RESERVED3[759];
  volatile const  uint32_t TRIGGER;                  
  volatile const  uint32_t FIFO0;                    
  volatile const  uint32_t ITATBCTR2;                
       uint32_t RESERVED4[1];
  volatile const  uint32_t ITATBCTR0;                
  volatile const  uint32_t FIFO1;                    
  volatile uint32_t ITCTRL;                   
       uint32_t RESERVED5[39];
  volatile uint32_t CLAIMSET;                 
  volatile uint32_t CLAIMCLR;                 
       uint32_t RESERVED7[8];
  volatile const  uint32_t DEVID;                    
  volatile const  uint32_t DEVTYPE;                  
} TPI_Type;
typedef struct
{
  volatile const  uint32_t TYPE;                     
  volatile uint32_t CTRL;                     
  volatile uint32_t RNR;                      
  volatile uint32_t RBAR;                     
  volatile uint32_t RASR;                     
  volatile uint32_t RBAR_A1;                  
  volatile uint32_t RASR_A1;                  
  volatile uint32_t RBAR_A2;                  
  volatile uint32_t RASR_A2;                  
  volatile uint32_t RBAR_A3;                  
  volatile uint32_t RASR_A3;                  
} MPU_Type;
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07);                
  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((0xFFFFUL << 16) | (7UL << 8));              
  reg_value  =  (reg_value                                 |
                ((uint32_t)0x5FA << 16) |
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) >> 8);    
}
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 5)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)] = ((priority << (8 - 5)) & 0xff);    }         
}
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{
  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 5)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)]           >> (8 - 5)));  }  
}
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;
  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 5) ? 5 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 5) < 7) ? 0 : PriorityGroupTmp - 7 + 5;
  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;
  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 5) ? 5 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 5) < 7) ? 0 : PriorityGroupTmp - 7 + 5;
  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) |
                 (1UL << 2));                    
  __dsb(0xF);                                                      
  while(1);                                                     
}
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0))  return (1);       
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = ticks - 1;                                   
  NVIC_SetPriority (SysTick_IRQn, (1<<5) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}
extern volatile int32_t ITM_RxBuffer;                     
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL << 0))                  &&       
      (((ITM_Type *) (0xE0000000UL) )->TER & (1UL << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000UL) )->PORT[0].u8 = (uint8_t) ch;
  }
  return (ch);
}
static __inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;                            
  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }
  return (ch);
}
static __inline int32_t ITM_CheckChar (void) {
  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}
extern uint32_t SystemCoreClock;
extern void SystemInit (void);
extern void SystemCoreClockUpdate (void);
typedef struct
{
  volatile uint32_t FLASHCFG;                
       uint32_t RESERVED0[31];
  volatile uint32_t PLL0CON;                 
  volatile uint32_t PLL0CFG;
  volatile const  uint32_t PLL0STAT;
  volatile  uint32_t PLL0FEED;
       uint32_t RESERVED1[4];
  volatile uint32_t PLL1CON;
  volatile uint32_t PLL1CFG;
  volatile const  uint32_t PLL1STAT;
  volatile  uint32_t PLL1FEED;
       uint32_t RESERVED2[4];
  volatile uint32_t PCON;
  volatile uint32_t PCONP;
       uint32_t RESERVED3[15];
  volatile uint32_t CCLKCFG;
  volatile uint32_t USBCLKCFG;
  volatile uint32_t CLKSRCSEL;
  volatile uint32_t	CANSLEEPCLR;
  volatile uint32_t	CANWAKEFLAGS;
       uint32_t RESERVED4[10];
  volatile uint32_t EXTINT;                  
       uint32_t RESERVED5;
  volatile uint32_t EXTMODE;
  volatile uint32_t EXTPOLAR;
       uint32_t RESERVED6[12];
  volatile uint32_t RSID;                    
       uint32_t RESERVED7[7];
  volatile uint32_t SCS;                     
  volatile uint32_t IRCTRIM;                 
  volatile uint32_t PCLKSEL0;
  volatile uint32_t PCLKSEL1;
       uint32_t RESERVED8[4];
  volatile uint32_t USBIntSt;                
  volatile uint32_t DMAREQSEL;
  volatile uint32_t CLKOUTCFG;               
 } LPC_SC_TypeDef;
typedef struct
{
  volatile uint32_t PINSEL0;
  volatile uint32_t PINSEL1;
  volatile uint32_t PINSEL2;
  volatile uint32_t PINSEL3;
  volatile uint32_t PINSEL4;
  volatile uint32_t PINSEL5;
  volatile uint32_t PINSEL6;
  volatile uint32_t PINSEL7;
  volatile uint32_t PINSEL8;
  volatile uint32_t PINSEL9;
  volatile uint32_t PINSEL10;
       uint32_t RESERVED0[5];
  volatile uint32_t PINMODE0;
  volatile uint32_t PINMODE1;
  volatile uint32_t PINMODE2;
  volatile uint32_t PINMODE3;
  volatile uint32_t PINMODE4;
  volatile uint32_t PINMODE5;
  volatile uint32_t PINMODE6;
  volatile uint32_t PINMODE7;
  volatile uint32_t PINMODE8;
  volatile uint32_t PINMODE9;
  volatile uint32_t PINMODE_OD0;
  volatile uint32_t PINMODE_OD1;
  volatile uint32_t PINMODE_OD2;
  volatile uint32_t PINMODE_OD3;
  volatile uint32_t PINMODE_OD4;
  volatile uint32_t I2CPADCFG;
} LPC_PINCON_TypeDef;
typedef struct
{
  union {
    volatile uint32_t FIODIR;
    struct {
      volatile uint16_t FIODIRL;
      volatile uint16_t FIODIRH;
    };
    struct {
      volatile uint8_t  FIODIR0;
      volatile uint8_t  FIODIR1;
      volatile uint8_t  FIODIR2;
      volatile uint8_t  FIODIR3;
    };
  };
  uint32_t RESERVED0[3];
  union {
    volatile uint32_t FIOMASK;
    struct {
      volatile uint16_t FIOMASKL;
      volatile uint16_t FIOMASKH;
    };
    struct {
      volatile uint8_t  FIOMASK0;
      volatile uint8_t  FIOMASK1;
      volatile uint8_t  FIOMASK2;
      volatile uint8_t  FIOMASK3;
    };
  };
  union {
    volatile uint32_t FIOPIN;
    struct {
      volatile uint16_t FIOPINL;
      volatile uint16_t FIOPINH;
    };
    struct {
      volatile uint8_t  FIOPIN0;
      volatile uint8_t  FIOPIN1;
      volatile uint8_t  FIOPIN2;
      volatile uint8_t  FIOPIN3;
    };
  };
  union {
    volatile uint32_t FIOSET;
    struct {
      volatile uint16_t FIOSETL;
      volatile uint16_t FIOSETH;
    };
    struct {
      volatile uint8_t  FIOSET0;
      volatile uint8_t  FIOSET1;
      volatile uint8_t  FIOSET2;
      volatile uint8_t  FIOSET3;
    };
  };
  union {
    volatile  uint32_t FIOCLR;
    struct {
      volatile  uint16_t FIOCLRL;
      volatile  uint16_t FIOCLRH;
    };
    struct {
      volatile  uint8_t  FIOCLR0;
      volatile  uint8_t  FIOCLR1;
      volatile  uint8_t  FIOCLR2;
      volatile  uint8_t  FIOCLR3;
    };
  };
} LPC_GPIO_TypeDef;
typedef struct
{
  volatile const  uint32_t IntStatus;
  volatile const  uint32_t IO0IntStatR;
  volatile const  uint32_t IO0IntStatF;
  volatile  uint32_t IO0IntClr;
  volatile uint32_t IO0IntEnR;
  volatile uint32_t IO0IntEnF;
       uint32_t RESERVED0[3];
  volatile const  uint32_t IO2IntStatR;
  volatile const  uint32_t IO2IntStatF;
  volatile  uint32_t IO2IntClr;
  volatile uint32_t IO2IntEnR;
  volatile uint32_t IO2IntEnF;
} LPC_GPIOINT_TypeDef;
typedef struct
{
  volatile uint32_t IR;
  volatile uint32_t TCR;
  volatile uint32_t TC;
  volatile uint32_t PR;
  volatile uint32_t PC;
  volatile uint32_t MCR;
  volatile uint32_t MR0;
  volatile uint32_t MR1;
  volatile uint32_t MR2;
  volatile uint32_t MR3;
  volatile uint32_t CCR;
  volatile const  uint32_t CR0;
  volatile const  uint32_t CR1;
       uint32_t RESERVED0[2];
  volatile uint32_t EMR;
       uint32_t RESERVED1[12];
  volatile uint32_t CTCR;
} LPC_TIM_TypeDef;
typedef struct
{
  volatile uint32_t IR;
  volatile uint32_t TCR;
  volatile uint32_t TC;
  volatile uint32_t PR;
  volatile uint32_t PC;
  volatile uint32_t MCR;
  volatile uint32_t MR0;
  volatile uint32_t MR1;
  volatile uint32_t MR2;
  volatile uint32_t MR3;
  volatile uint32_t CCR;
  volatile const  uint32_t CR0;
  volatile const  uint32_t CR1;
  volatile const  uint32_t CR2;
  volatile const  uint32_t CR3;
       uint32_t RESERVED0;
  volatile uint32_t MR4;
  volatile uint32_t MR5;
  volatile uint32_t MR6;
  volatile uint32_t PCR;
  volatile uint32_t LER;
       uint32_t RESERVED1[7];
  volatile uint32_t CTCR;
} LPC_PWM_TypeDef;
typedef struct
{
  union {
  volatile const  uint8_t  RBR;
  volatile  uint8_t  THR;
  volatile uint8_t  DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t  DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const  uint32_t IIR;
  volatile  uint8_t  FCR;
  };
  volatile uint8_t  LCR;
       uint8_t  RESERVED1[7];
  volatile const  uint8_t  LSR;
       uint8_t  RESERVED2[7];
  volatile uint8_t  SCR;
       uint8_t  RESERVED3[3];
  volatile uint32_t ACR;
  volatile uint8_t  ICR;
       uint8_t  RESERVED4[3];
  volatile uint8_t  FDR;
       uint8_t  RESERVED5[7];
  volatile uint8_t  TER;
} LPC_UART_TypeDef;
typedef struct
{
  union {
  volatile const  uint8_t  RBR;
  volatile  uint8_t  THR;
  volatile uint8_t  DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t  DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const  uint32_t IIR;
  volatile  uint8_t  FCR;
  };
  volatile uint8_t  LCR;
       uint8_t  RESERVED1[3];
  volatile uint8_t  MCR;
       uint8_t  RESERVED2[3];
  volatile const  uint8_t  LSR;
       uint8_t  RESERVED3[3];
  volatile const  uint8_t  MSR;
       uint8_t  RESERVED4[3];
  volatile uint8_t  SCR;
       uint8_t  RESERVED5[3];
  volatile uint32_t ACR;
       uint32_t RESERVED6;
  volatile uint32_t FDR;
       uint32_t RESERVED7;
  volatile uint8_t  TER;
       uint8_t  RESERVED8[27];
  volatile uint8_t  RS485CTRL;
       uint8_t  RESERVED9[3];
  volatile uint8_t  ADRMATCH;
       uint8_t  RESERVED10[3];
  volatile uint8_t  RS485DLY;
} LPC_UART1_TypeDef;
typedef struct
{
  volatile uint32_t SPCR;
  volatile const  uint32_t SPSR;
  volatile uint32_t SPDR;
  volatile uint32_t SPCCR;
       uint32_t RESERVED0[3];
  volatile uint32_t SPINT;
} LPC_SPI_TypeDef;
typedef struct
{
  volatile uint32_t CR0;
  volatile uint32_t CR1;
  volatile uint32_t DR;
  volatile const  uint32_t SR;
  volatile uint32_t CPSR;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
} LPC_SSP_TypeDef;
typedef struct
{
  volatile uint32_t I2CONSET;
  volatile const  uint32_t I2STAT;
  volatile uint32_t I2DAT;
  volatile uint32_t I2ADR0;
  volatile uint32_t I2SCLH;
  volatile uint32_t I2SCLL;
  volatile  uint32_t I2CONCLR;
  volatile uint32_t MMCTRL;
  volatile uint32_t I2ADR1;
  volatile uint32_t I2ADR2;
  volatile uint32_t I2ADR3;
  volatile const  uint32_t I2DATA_BUFFER;
  volatile uint32_t I2MASK0;
  volatile uint32_t I2MASK1;
  volatile uint32_t I2MASK2;
  volatile uint32_t I2MASK3;
} LPC_I2C_TypeDef;
typedef struct
{
  volatile uint32_t I2SDAO;
  volatile uint32_t I2SDAI;
  volatile  uint32_t I2STXFIFO;
  volatile const  uint32_t I2SRXFIFO;
  volatile const  uint32_t I2SSTATE;
  volatile uint32_t I2SDMA1;
  volatile uint32_t I2SDMA2;
  volatile uint32_t I2SIRQ;
  volatile uint32_t I2STXRATE;
  volatile uint32_t I2SRXRATE;
  volatile uint32_t I2STXBITRATE;
  volatile uint32_t I2SRXBITRATE;
  volatile uint32_t I2STXMODE;
  volatile uint32_t I2SRXMODE;
} LPC_I2S_TypeDef;
typedef struct
{
  volatile uint32_t RICOMPVAL;
  volatile uint32_t RIMASK;
  volatile uint8_t  RICTRL;
       uint8_t  RESERVED0[3];
  volatile uint32_t RICOUNTER;
} LPC_RIT_TypeDef;
typedef struct
{
  volatile uint8_t  ILR;
       uint8_t  RESERVED0[7];
  volatile uint8_t  CCR;
       uint8_t  RESERVED1[3];
  volatile uint8_t  CIIR;
       uint8_t  RESERVED2[3];
  volatile uint8_t  AMR;
       uint8_t  RESERVED3[3];
  volatile const  uint32_t CTIME0;
  volatile const  uint32_t CTIME1;
  volatile const  uint32_t CTIME2;
  volatile uint8_t  SEC;
       uint8_t  RESERVED4[3];
  volatile uint8_t  MIN;
       uint8_t  RESERVED5[3];
  volatile uint8_t  HOUR;
       uint8_t  RESERVED6[3];
  volatile uint8_t  DOM;
       uint8_t  RESERVED7[3];
  volatile uint8_t  DOW;
       uint8_t  RESERVED8[3];
  volatile uint16_t DOY;
       uint16_t RESERVED9;
  volatile uint8_t  MONTH;
       uint8_t  RESERVED10[3];
  volatile uint16_t YEAR;
       uint16_t RESERVED11;
  volatile uint32_t CALIBRATION;
  volatile uint32_t GPREG0;
  volatile uint32_t GPREG1;
  volatile uint32_t GPREG2;
  volatile uint32_t GPREG3;
  volatile uint32_t GPREG4;
  volatile uint8_t  RTC_AUXEN;
       uint8_t  RESERVED12[3];
  volatile uint8_t  RTC_AUX;
       uint8_t  RESERVED13[3];
  volatile uint8_t  ALSEC;
       uint8_t  RESERVED14[3];
  volatile uint8_t  ALMIN;
       uint8_t  RESERVED15[3];
  volatile uint8_t  ALHOUR;
       uint8_t  RESERVED16[3];
  volatile uint8_t  ALDOM;
       uint8_t  RESERVED17[3];
  volatile uint8_t  ALDOW;
       uint8_t  RESERVED18[3];
  volatile uint16_t ALDOY;
       uint16_t RESERVED19;
  volatile uint8_t  ALMON;
       uint8_t  RESERVED20[3];
  volatile uint16_t ALYEAR;
       uint16_t RESERVED21;
} LPC_RTC_TypeDef;
typedef struct
{
  volatile uint8_t  WDMOD;
       uint8_t  RESERVED0[3];
  volatile uint32_t WDTC;
  volatile  uint8_t  WDFEED;
       uint8_t  RESERVED1[3];
  volatile const  uint32_t WDTV;
  volatile uint32_t WDCLKSEL;
} LPC_WDT_TypeDef;
typedef struct
{
  volatile uint32_t ADCR;
  volatile uint32_t ADGDR;
       uint32_t RESERVED0;
  volatile uint32_t ADINTEN;
  volatile const  uint32_t ADDR0;
  volatile const  uint32_t ADDR1;
  volatile const  uint32_t ADDR2;
  volatile const  uint32_t ADDR3;
  volatile const  uint32_t ADDR4;
  volatile const  uint32_t ADDR5;
  volatile const  uint32_t ADDR6;
  volatile const  uint32_t ADDR7;
  volatile const  uint32_t ADSTAT;
  volatile uint32_t ADTRM;
} LPC_ADC_TypeDef;
typedef struct
{
  volatile uint32_t DACR;
  volatile uint32_t DACCTRL;
  volatile uint16_t DACCNTVAL;
} LPC_DAC_TypeDef;
typedef struct
{
  volatile const  uint32_t MCCON;
  volatile  uint32_t MCCON_SET;
  volatile  uint32_t MCCON_CLR;
  volatile const  uint32_t MCCAPCON;
  volatile  uint32_t MCCAPCON_SET;
  volatile  uint32_t MCCAPCON_CLR;
  volatile uint32_t MCTIM0;
  volatile uint32_t MCTIM1;
  volatile uint32_t MCTIM2;
  volatile uint32_t MCPER0;
  volatile uint32_t MCPER1;
  volatile uint32_t MCPER2;
  volatile uint32_t MCPW0;
  volatile uint32_t MCPW1;
  volatile uint32_t MCPW2;
  volatile uint32_t MCDEADTIME;
  volatile uint32_t MCCCP;
  volatile uint32_t MCCR0;
  volatile uint32_t MCCR1;
  volatile uint32_t MCCR2;
  volatile const  uint32_t MCINTEN;
  volatile  uint32_t MCINTEN_SET;
  volatile  uint32_t MCINTEN_CLR;
  volatile const  uint32_t MCCNTCON;
  volatile  uint32_t MCCNTCON_SET;
  volatile  uint32_t MCCNTCON_CLR;
  volatile const  uint32_t MCINTFLAG;
  volatile  uint32_t MCINTFLAG_SET;
  volatile  uint32_t MCINTFLAG_CLR;
  volatile  uint32_t MCCAP_CLR;
} LPC_MCPWM_TypeDef;
typedef struct
{
  volatile  uint32_t QEICON;
  volatile const  uint32_t QEISTAT;
  volatile uint32_t QEICONF;
  volatile const  uint32_t QEIPOS;
  volatile uint32_t QEIMAXPOS;
  volatile uint32_t CMPOS0;
  volatile uint32_t CMPOS1;
  volatile uint32_t CMPOS2;
  volatile const  uint32_t INXCNT;
  volatile uint32_t INXCMP;
  volatile uint32_t QEILOAD;
  volatile const  uint32_t QEITIME;
  volatile const  uint32_t QEIVEL;
  volatile const  uint32_t QEICAP;
  volatile uint32_t VELCOMP;
  volatile uint32_t FILTER;
       uint32_t RESERVED0[998];
  volatile  uint32_t QEIIEC;
  volatile  uint32_t QEIIES;
  volatile const  uint32_t QEIINTSTAT;
  volatile const  uint32_t QEIIE;
  volatile  uint32_t QEICLR;
  volatile  uint32_t QEISET;
} LPC_QEI_TypeDef;
typedef struct
{
  volatile uint32_t mask[512];               
} LPC_CANAF_RAM_TypeDef;
typedef struct                           
{
  volatile uint32_t AFMR;
  volatile uint32_t SFF_sa;
  volatile uint32_t SFF_GRP_sa;
  volatile uint32_t EFF_sa;
  volatile uint32_t EFF_GRP_sa;
  volatile uint32_t ENDofTable;
  volatile const  uint32_t LUTerrAd;
  volatile const  uint32_t LUTerr;
  volatile uint32_t FCANIE;
  volatile uint32_t FCANIC0;
  volatile uint32_t FCANIC1;
} LPC_CANAF_TypeDef;
typedef struct                           
{
  volatile const  uint32_t CANTxSR;
  volatile const  uint32_t CANRxSR;
  volatile const  uint32_t CANMSR;
} LPC_CANCR_TypeDef;
typedef struct                           
{
  volatile uint32_t MOD;
  volatile  uint32_t CMR;
  volatile uint32_t GSR;
  volatile const  uint32_t ICR;
  volatile uint32_t IER;
  volatile uint32_t BTR;
  volatile uint32_t EWL;
  volatile const  uint32_t SR;
  volatile uint32_t RFS;
  volatile uint32_t RID;
  volatile uint32_t RDA;
  volatile uint32_t RDB;
  volatile uint32_t TFI1;
  volatile uint32_t TID1;
  volatile uint32_t TDA1;
  volatile uint32_t TDB1;
  volatile uint32_t TFI2;
  volatile uint32_t TID2;
  volatile uint32_t TDA2;
  volatile uint32_t TDB2;
  volatile uint32_t TFI3;
  volatile uint32_t TID3;
  volatile uint32_t TDA3;
  volatile uint32_t TDB3;
} LPC_CAN_TypeDef;
typedef struct                           
{
  volatile const  uint32_t DMACIntStat;
  volatile const  uint32_t DMACIntTCStat;
  volatile  uint32_t DMACIntTCClear;
  volatile const  uint32_t DMACIntErrStat;
  volatile  uint32_t DMACIntErrClr;
  volatile const  uint32_t DMACRawIntTCStat;
  volatile const  uint32_t DMACRawIntErrStat;
  volatile const  uint32_t DMACEnbldChns;
  volatile uint32_t DMACSoftBReq;
  volatile uint32_t DMACSoftSReq;
  volatile uint32_t DMACSoftLBReq;
  volatile uint32_t DMACSoftLSReq;
  volatile uint32_t DMACConfig;
  volatile uint32_t DMACSync;
} LPC_GPDMA_TypeDef;
typedef struct                           
{
  volatile uint32_t DMACCSrcAddr;
  volatile uint32_t DMACCDestAddr;
  volatile uint32_t DMACCLLI;
  volatile uint32_t DMACCControl;
  volatile uint32_t DMACCConfig;
} LPC_GPDMACH_TypeDef;
typedef struct
{
  volatile const  uint32_t HcRevision;              
  volatile uint32_t HcControl;
  volatile uint32_t HcCommandStatus;
  volatile uint32_t HcInterruptStatus;
  volatile uint32_t HcInterruptEnable;
  volatile uint32_t HcInterruptDisable;
  volatile uint32_t HcHCCA;
  volatile const  uint32_t HcPeriodCurrentED;
  volatile uint32_t HcControlHeadED;
  volatile uint32_t HcControlCurrentED;
  volatile uint32_t HcBulkHeadED;
  volatile uint32_t HcBulkCurrentED;
  volatile const  uint32_t HcDoneHead;
  volatile uint32_t HcFmInterval;
  volatile const  uint32_t HcFmRemaining;
  volatile const  uint32_t HcFmNumber;
  volatile uint32_t HcPeriodicStart;
  volatile uint32_t HcLSTreshold;
  volatile uint32_t HcRhDescriptorA;
  volatile uint32_t HcRhDescriptorB;
  volatile uint32_t HcRhStatus;
  volatile uint32_t HcRhPortStatus1;
  volatile uint32_t HcRhPortStatus2;
       uint32_t RESERVED0[40];
  volatile const  uint32_t Module_ID;
  volatile const  uint32_t OTGIntSt;                
  volatile uint32_t OTGIntEn;
  volatile  uint32_t OTGIntSet;
  volatile  uint32_t OTGIntClr;
  volatile uint32_t OTGStCtrl;
  volatile uint32_t OTGTmr;
       uint32_t RESERVED1[58];
  volatile const  uint32_t USBDevIntSt;             
  volatile uint32_t USBDevIntEn;
  volatile  uint32_t USBDevIntClr;
  volatile  uint32_t USBDevIntSet;
  volatile  uint32_t USBCmdCode;              
  volatile const  uint32_t USBCmdData;
  volatile const  uint32_t USBRxData;               
  volatile  uint32_t USBTxData;
  volatile const  uint32_t USBRxPLen;
  volatile  uint32_t USBTxPLen;
  volatile uint32_t USBCtrl;
  volatile  uint32_t USBDevIntPri;
  volatile const  uint32_t USBEpIntSt;              
  volatile uint32_t USBEpIntEn;
  volatile  uint32_t USBEpIntClr;
  volatile  uint32_t USBEpIntSet;
  volatile  uint32_t USBEpIntPri;
  volatile uint32_t USBReEp;                 
  volatile  uint32_t USBEpInd;
  volatile uint32_t USBMaxPSize;
  volatile const  uint32_t USBDMARSt;               
  volatile  uint32_t USBDMARClr;
  volatile  uint32_t USBDMARSet;
       uint32_t RESERVED2[9];
  volatile uint32_t USBUDCAH;
  volatile const  uint32_t USBEpDMASt;
  volatile  uint32_t USBEpDMAEn;
  volatile  uint32_t USBEpDMADis;
  volatile const  uint32_t USBDMAIntSt;
  volatile uint32_t USBDMAIntEn;
       uint32_t RESERVED3[2];
  volatile const  uint32_t USBEoTIntSt;
  volatile  uint32_t USBEoTIntClr;
  volatile  uint32_t USBEoTIntSet;
  volatile const  uint32_t USBNDDRIntSt;
  volatile  uint32_t USBNDDRIntClr;
  volatile  uint32_t USBNDDRIntSet;
  volatile const  uint32_t USBSysErrIntSt;
  volatile  uint32_t USBSysErrIntClr;
  volatile  uint32_t USBSysErrIntSet;
       uint32_t RESERVED4[15];
  union {
  volatile const  uint32_t I2C_RX;                  
  volatile  uint32_t I2C_TX;
  };
  volatile const  uint32_t I2C_STS;
  volatile uint32_t I2C_CTL;
  volatile uint32_t I2C_CLKHI;
  volatile  uint32_t I2C_CLKLO;
       uint32_t RESERVED5[824];
  union {
  volatile uint32_t USBClkCtrl;              
  volatile uint32_t OTGClkCtrl;
  };
  union {
  volatile const  uint32_t USBClkSt;
  volatile const  uint32_t OTGClkSt;
  };
} LPC_USB_TypeDef;
typedef struct
{
  volatile uint32_t MAC1;                    
  volatile uint32_t MAC2;
  volatile uint32_t IPGT;
  volatile uint32_t IPGR;
  volatile uint32_t CLRT;
  volatile uint32_t MAXF;
  volatile uint32_t SUPP;
  volatile uint32_t TEST;
  volatile uint32_t MCFG;
  volatile uint32_t MCMD;
  volatile uint32_t MADR;
  volatile  uint32_t MWTD;
  volatile const  uint32_t MRDD;
  volatile const  uint32_t MIND;
       uint32_t RESERVED0[2];
  volatile uint32_t SA0;
  volatile uint32_t SA1;
  volatile uint32_t SA2;
       uint32_t RESERVED1[45];
  volatile uint32_t Command;                 
  volatile const  uint32_t Status;
  volatile uint32_t RxDescriptor;
  volatile uint32_t RxStatus;
  volatile uint32_t RxDescriptorNumber;
  volatile const  uint32_t RxProduceIndex;
  volatile uint32_t RxConsumeIndex;
  volatile uint32_t TxDescriptor;
  volatile uint32_t TxStatus;
  volatile uint32_t TxDescriptorNumber;
  volatile uint32_t TxProduceIndex;
  volatile const  uint32_t TxConsumeIndex;
       uint32_t RESERVED2[10];
  volatile const  uint32_t TSV0;
  volatile const  uint32_t TSV1;
  volatile const  uint32_t RSV;
       uint32_t RESERVED3[3];
  volatile uint32_t FlowControlCounter;
  volatile const  uint32_t FlowControlStatus;
       uint32_t RESERVED4[34];
  volatile uint32_t RxFilterCtrl;            
  volatile uint32_t RxFilterWoLStatus;
  volatile uint32_t RxFilterWoLClear;
       uint32_t RESERVED5;
  volatile uint32_t HashFilterL;
  volatile uint32_t HashFilterH;
       uint32_t RESERVED6[882];
  volatile const  uint32_t IntStatus;               
  volatile uint32_t IntEnable;
  volatile  uint32_t IntClear;
  volatile  uint32_t IntSet;
       uint32_t RESERVED7;
  volatile uint32_t PowerDown;
       uint32_t RESERVED8;
  volatile uint32_t Module_ID;
} LPC_EMAC_TypeDef;
typedef void (*TaskFunction_t)( void * );
typedef uint32_t StackType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;
	typedef uint32_t TickType_t;
extern void vPortEnterCritical( void );
extern void vPortExitCritical( void );
	extern void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );
static __forceinline void vPortSetBASEPRI( uint32_t ulBASEPRI )
{
	__asm
	{
		msr basepri, ulBASEPRI
	}
}
static __forceinline void vPortRaiseBASEPRI( void )
{
uint32_t ulNewBASEPRI = ( 5 << (8 - 5) );
	__asm
	{
		msr basepri, ulNewBASEPRI
		dsb
		isb
	}
}
static __forceinline void vPortClearBASEPRIFromISR( void )
{
	__asm
	{
		msr basepri, #0
	}
}
static __forceinline uint32_t ulPortRaiseBASEPRI( void )
{
uint32_t ulReturn, ulNewBASEPRI = ( 5 << (8 - 5) );
	__asm
	{
		mrs ulReturn, basepri
		msr basepri, ulNewBASEPRI
		dsb
		isb
	}
	return ulReturn;
}
static __forceinline BaseType_t xPortIsInsideInterrupt( void )
{
uint32_t ulCurrentInterrupt;
BaseType_t xReturn;
	__asm
	{
		mrs ulCurrentInterrupt, ipsr
	}
	if( ulCurrentInterrupt == 0 )
	{
		xReturn = ( ( BaseType_t ) 0 );
	}
	else
	{
		xReturn = ( ( BaseType_t ) 1 );
	}
	return xReturn;
}
		StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters ) ;
typedef struct HeapRegion
{
	uint8_t *pucStartAddress;
	size_t xSizeInBytes;
} HeapRegion_t;
void vPortDefineHeapRegions( const HeapRegion_t * const pxHeapRegions ) ;
void *pvPortMalloc( size_t xSize ) ;
void vPortFree( void *pv ) ;
void vPortInitialiseBlocks( void ) ;
size_t xPortGetFreeHeapSize( void ) ;
size_t xPortGetMinimumEverFreeHeapSize( void ) ;
BaseType_t xPortStartScheduler( void ) ;
void vPortEndScheduler( void ) ;
struct xSTATIC_LIST_ITEM
{
	TickType_t xDummy2;
	void *pvDummy3[ 4 ];
};
typedef struct xSTATIC_LIST_ITEM StaticListItem_t;
struct xSTATIC_MINI_LIST_ITEM
{
	TickType_t xDummy2;
	void *pvDummy3[ 2 ];
};
typedef struct xSTATIC_MINI_LIST_ITEM StaticMiniListItem_t;
typedef struct xSTATIC_LIST
{
	UBaseType_t uxDummy2;
	void *pvDummy3;
	StaticMiniListItem_t xDummy4;
} StaticList_t;
typedef struct xSTATIC_TCB
{
	void				*pxDummy1;
	StaticListItem_t	xDummy3[ 2 ];
	UBaseType_t			uxDummy5;
	void				*pxDummy6;
	uint8_t				ucDummy7[ ( 8 ) ];
		uint32_t 		ulDummy18;
		uint8_t 		ucDummy19;
} StaticTask_t;
typedef struct xSTATIC_QUEUE
{
	void *pvDummy1[ 3 ];
	union
	{
		void *pvDummy2;
		UBaseType_t uxDummy2;
	} u;
	StaticList_t xDummy3[ 2 ];
	UBaseType_t uxDummy4[ 3 ];
	uint8_t ucDummy5[ 2 ];
} StaticQueue_t;
typedef StaticQueue_t StaticSemaphore_t;
typedef struct xSTATIC_EVENT_GROUP
{
	TickType_t xDummy1;
	StaticList_t xDummy2;
} StaticEventGroup_t;
typedef struct xSTATIC_TIMER
{
	void				*pvDummy1;
	StaticListItem_t	xDummy2;
	TickType_t			xDummy3;
	void 				*pvDummy5;
	TaskFunction_t		pvDummy6;
	uint8_t 			ucDummy8;
} StaticTimer_t;
typedef struct xSTATIC_STREAM_BUFFER
{
	size_t uxDummy1[ 4 ];
	void * pvDummy2[ 3 ];
	uint8_t ucDummy3;
} StaticStreamBuffer_t;
typedef StaticStreamBuffer_t StaticMessageBuffer_t;
struct xLIST;
struct xLIST_ITEM
{
	 TickType_t xItemValue;			 
	struct xLIST_ITEM *  pxNext;		 
	struct xLIST_ITEM *  pxPrevious;	 
	void * pvOwner;										 
	struct xLIST *  pvContainer;		 
};
typedef struct xLIST_ITEM ListItem_t;					 
struct xMINI_LIST_ITEM
{
	 TickType_t xItemValue;
	struct xLIST_ITEM *  pxNext;
	struct xLIST_ITEM *  pxPrevious;
};
typedef struct xMINI_LIST_ITEM MiniListItem_t;
typedef struct xLIST
{
	volatile UBaseType_t uxNumberOfItems;
	ListItem_t *  pxIndex;			 
	MiniListItem_t xListEnd;							 
} List_t;
void vListInitialise( List_t * const pxList ) ;
void vListInitialiseItem( ListItem_t * const pxItem ) ;
void vListInsert( List_t * const pxList, ListItem_t * const pxNewListItem ) ;
void vListInsertEnd( List_t * const pxList, ListItem_t * const pxNewListItem ) ;
UBaseType_t uxListRemove( ListItem_t * const pxItemToRemove ) ;
struct tskTaskControlBlock;  
typedef struct tskTaskControlBlock* TaskHandle_t;
typedef BaseType_t (*TaskHookFunction_t)( void * );
typedef enum
{
	eRunning = 0,	 
	eReady,			 
	eBlocked,		 
	eSuspended,		 
	eDeleted,		 
	eInvalid		 
} eTaskState;
typedef enum
{
	eNoAction = 0,				 
	eSetBits,					 
	eIncrement,					 
	eSetValueWithOverwrite,		 
	eSetValueWithoutOverwrite	 
} eNotifyAction;
typedef struct xTIME_OUT
{
	BaseType_t xOverflowCount;
	TickType_t xTimeOnEntering;
} TimeOut_t;
typedef struct xMEMORY_REGION
{
	void *pvBaseAddress;
	uint32_t ulLengthInBytes;
	uint32_t ulParameters;
} MemoryRegion_t;
typedef struct xTASK_PARAMETERS
{
	TaskFunction_t pvTaskCode;
	const char * const pcName;	 
	uint16_t usStackDepth;
	void *pvParameters;
	UBaseType_t uxPriority;
	StackType_t *puxStackBuffer;
	MemoryRegion_t xRegions[ 1 ];
} TaskParameters_t;
typedef struct xTASK_STATUS
{
	TaskHandle_t xHandle;			 
	const char *pcTaskName;			   
	UBaseType_t xTaskNumber;		 
	eTaskState eCurrentState;		 
	UBaseType_t uxCurrentPriority;	 
	UBaseType_t uxBasePriority;		 
	uint32_t ulRunTimeCounter;		 
	StackType_t *pxStackBase;		 
	uint16_t usStackHighWaterMark;	 
} TaskStatus_t;
typedef enum
{
	eAbortSleep = 0,		 
	eStandardSleep,			 
	eNoTasksWaitingTimeout	 
} eSleepModeStatus;
	BaseType_t xTaskCreate(	TaskFunction_t pxTaskCode,
							const char * const pcName,	 
							const uint16_t usStackDepth,
							void * const pvParameters,
							UBaseType_t uxPriority,
							TaskHandle_t * const pxCreatedTask ) ;
void vTaskAllocateMPURegions( TaskHandle_t xTask, const MemoryRegion_t * const pxRegions ) ;
void vTaskDelete( TaskHandle_t xTaskToDelete ) ;
void vTaskDelay( const TickType_t xTicksToDelay ) ;
void vTaskDelayUntil( TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement ) ;
BaseType_t xTaskAbortDelay( TaskHandle_t xTask ) ;
UBaseType_t uxTaskPriorityGet( const TaskHandle_t xTask ) ;
UBaseType_t uxTaskPriorityGetFromISR( const TaskHandle_t xTask ) ;
eTaskState eTaskGetState( TaskHandle_t xTask ) ;
void vTaskGetInfo( TaskHandle_t xTask, TaskStatus_t *pxTaskStatus, BaseType_t xGetFreeStackSpace, eTaskState eState ) ;
void vTaskPrioritySet( TaskHandle_t xTask, UBaseType_t uxNewPriority ) ;
void vTaskSuspend( TaskHandle_t xTaskToSuspend ) ;
void vTaskResume( TaskHandle_t xTaskToResume ) ;
BaseType_t xTaskResumeFromISR( TaskHandle_t xTaskToResume ) ;
void vTaskStartScheduler( void ) ;
void vTaskEndScheduler( void ) ;
void vTaskSuspendAll( void ) ;
BaseType_t xTaskResumeAll( void ) ;
TickType_t xTaskGetTickCount( void ) ;
TickType_t xTaskGetTickCountFromISR( void ) ;
UBaseType_t uxTaskGetNumberOfTasks( void ) ;
char *pcTaskGetName( TaskHandle_t xTaskToQuery ) ;  
TaskHandle_t xTaskGetHandle( const char *pcNameToQuery ) ;  
UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask ) ;
uint16_t uxTaskGetStackHighWaterMark2( TaskHandle_t xTask ) ;
BaseType_t xTaskCallApplicationTaskHook( TaskHandle_t xTask, void *pvParameter ) ;
TaskHandle_t xTaskGetIdleTaskHandle( void ) ;
UBaseType_t uxTaskGetSystemState( TaskStatus_t * const pxTaskStatusArray, const UBaseType_t uxArraySize, uint32_t * const pulTotalRunTime ) ;
void vTaskList( char * pcWriteBuffer ) ;  
void vTaskGetRunTimeStats( char *pcWriteBuffer ) ;  
TickType_t xTaskGetIdleRunTimeCounter( void ) ;
BaseType_t xTaskGenericNotify( TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, uint32_t *pulPreviousNotificationValue ) ;
BaseType_t xTaskGenericNotifyFromISR( TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, uint32_t *pulPreviousNotificationValue, BaseType_t *pxHigherPriorityTaskWoken ) ;
BaseType_t xTaskNotifyWait( uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait ) ;
void vTaskNotifyGiveFromISR( TaskHandle_t xTaskToNotify, BaseType_t *pxHigherPriorityTaskWoken ) ;
uint32_t ulTaskNotifyTake( BaseType_t xClearCountOnExit, TickType_t xTicksToWait ) ;
BaseType_t xTaskNotifyStateClear( TaskHandle_t xTask );
BaseType_t xTaskIncrementTick( void ) ;
void vTaskPlaceOnEventList( List_t * const pxEventList, const TickType_t xTicksToWait ) ;
void vTaskPlaceOnUnorderedEventList( List_t * pxEventList, const TickType_t xItemValue, const TickType_t xTicksToWait ) ;
void vTaskPlaceOnEventListRestricted( List_t * const pxEventList, TickType_t xTicksToWait, const BaseType_t xWaitIndefinitely ) ;
BaseType_t xTaskRemoveFromEventList( const List_t * const pxEventList ) ;
void vTaskRemoveFromUnorderedEventList( ListItem_t * pxEventListItem, const TickType_t xItemValue ) ;
 void vTaskSwitchContext( void ) ;
TickType_t uxTaskResetEventItemValue( void ) ;
TaskHandle_t xTaskGetCurrentTaskHandle( void ) ;
void vTaskSetTimeOutState( TimeOut_t * const pxTimeOut ) ;
BaseType_t xTaskCheckForTimeOut( TimeOut_t * const pxTimeOut, TickType_t * const pxTicksToWait ) ;
void vTaskMissedYield( void ) ;
BaseType_t xTaskGetSchedulerState( void ) ;
BaseType_t xTaskPriorityInherit( TaskHandle_t const pxMutexHolder ) ;
BaseType_t xTaskPriorityDisinherit( TaskHandle_t const pxMutexHolder ) ;
void vTaskPriorityDisinheritAfterTimeout( TaskHandle_t const pxMutexHolder, UBaseType_t uxHighestPriorityWaitingTask ) ;
UBaseType_t uxTaskGetTaskNumber( TaskHandle_t xTask ) ;
void vTaskSetTaskNumber( TaskHandle_t xTask, const UBaseType_t uxHandle ) ;
void vTaskStepTick( const TickType_t xTicksToJump ) ;
eSleepModeStatus eTaskConfirmSleepModeStatus( void ) ;
TaskHandle_t pvTaskIncrementMutexHeldCount( void ) ;
void vTaskInternalSetTimeOutState( TimeOut_t * const pxTimeOut ) ;
struct tmrTimerControl;  
typedef struct tmrTimerControl * TimerHandle_t;
typedef void (*TimerCallbackFunction_t)( TimerHandle_t xTimer );
typedef void (*PendedFunction_t)( void *, uint32_t );
	TimerHandle_t xTimerCreate(	const char * const pcTimerName,			 
								const TickType_t xTimerPeriodInTicks,
								const UBaseType_t uxAutoReload,
								void * const pvTimerID,
								TimerCallbackFunction_t pxCallbackFunction ) ;
void *pvTimerGetTimerID( const TimerHandle_t xTimer ) ;
void vTimerSetTimerID( TimerHandle_t xTimer, void *pvNewID ) ;
BaseType_t xTimerIsTimerActive( TimerHandle_t xTimer ) ;
TaskHandle_t xTimerGetTimerDaemonTaskHandle( void ) ;
BaseType_t xTimerPendFunctionCallFromISR( PendedFunction_t xFunctionToPend, void *pvParameter1, uint32_t ulParameter2, BaseType_t *pxHigherPriorityTaskWoken ) ;
BaseType_t xTimerPendFunctionCall( PendedFunction_t xFunctionToPend, void *pvParameter1, uint32_t ulParameter2, TickType_t xTicksToWait ) ;
const char * pcTimerGetName( TimerHandle_t xTimer ) ;  
void vTimerSetReloadMode( TimerHandle_t xTimer, const UBaseType_t uxAutoReload ) ;
TickType_t xTimerGetPeriod( TimerHandle_t xTimer ) ;
TickType_t xTimerGetExpiryTime( TimerHandle_t xTimer ) ;
BaseType_t xTimerCreateTimerTask( void ) ;
BaseType_t xTimerGenericCommand( TimerHandle_t xTimer, const BaseType_t xCommandID, const TickType_t xOptionalValue, BaseType_t * const pxHigherPriorityTaskWoken, const TickType_t xTicksToWait ) ;
typedef struct tskTaskControlBlock 			 
{
	volatile StackType_t	*pxTopOfStack;	 
	ListItem_t			xStateListItem;	 
	ListItem_t			xEventListItem;		 
	UBaseType_t			uxPriority;			 
	StackType_t			*pxStack;			 
	char				pcTaskName[ ( 8 ) ];   
		volatile uint32_t ulNotifiedValue;
		volatile uint8_t ucNotifyState;
} tskTCB;
typedef tskTCB TCB_t;
 TCB_t * volatile pxCurrentTCB = 0;
 static List_t pxReadyTasksLists[ ( 4 ) ]; 
 static List_t xDelayedTaskList1;						 
 static List_t xDelayedTaskList2;						 
 static List_t * volatile pxDelayedTaskList;				 
 static List_t * volatile pxOverflowDelayedTaskList;		 
 static List_t xPendingReadyList;						 
	 static List_t xTasksWaitingTermination;				 
	 static volatile UBaseType_t uxDeletedTasksWaitingCleanUp = ( UBaseType_t ) 0U;
	 static List_t xSuspendedTaskList;					 
 static volatile UBaseType_t uxCurrentNumberOfTasks 	= ( UBaseType_t ) 0U;
 static volatile TickType_t xTickCount 				= ( TickType_t ) 0;
 static volatile UBaseType_t uxTopReadyPriority 		= ( ( UBaseType_t ) 0U );
 static volatile BaseType_t xSchedulerRunning 		= ( ( BaseType_t ) 0 );
 static volatile UBaseType_t uxPendedTicks 			= ( UBaseType_t ) 0U;
 static volatile BaseType_t xYieldPending 			= ( ( BaseType_t ) 0 );
 static volatile BaseType_t xNumOfOverflows 			= ( BaseType_t ) 0;
 static UBaseType_t uxTaskNumber 					= ( UBaseType_t ) 0U;
 static volatile TickType_t xNextTaskUnblockTime		= ( TickType_t ) 0U;  
 static TaskHandle_t xIdleTaskHandle					= 0;			 
 static volatile UBaseType_t uxSchedulerSuspended	= ( UBaseType_t ) ( ( BaseType_t ) 0 );
	static BaseType_t prvTaskIsTaskSuspended( const TaskHandle_t xTask ) ;
static void prvInitialiseTaskLists( void ) ;
static void prvIdleTask( void *pvParameters );
	static void prvDeleteTCB( TCB_t *pxTCB ) ;
static void prvCheckTasksWaitingTermination( void ) ;
static void prvAddCurrentTaskToDelayedList( TickType_t xTicksToWait, const BaseType_t xCanBlockIndefinitely ) ;
static void prvResetNextTaskUnblockTime( void );
static void prvInitialiseNewTask( 	TaskFunction_t pxTaskCode,
									const char * const pcName, 		 
									const uint32_t ulStackDepth,
									void * const pvParameters,
									UBaseType_t uxPriority,
									TaskHandle_t * const pxCreatedTask,
									TCB_t *pxNewTCB,
									const MemoryRegion_t * const xRegions ) ;
static void prvAddNewTaskToReadyList( TCB_t *pxNewTCB ) ;
	BaseType_t xTaskCreate(	TaskFunction_t pxTaskCode,
							const char * const pcName,		 
							const uint16_t usStackDepth,
							void * const pvParameters,
							UBaseType_t uxPriority,
							TaskHandle_t * const pxCreatedTask )
	{
	TCB_t *pxNewTCB;
	BaseType_t xReturn;
		{
		StackType_t *pxStack;
			pxStack = pvPortMalloc( ( ( ( size_t ) usStackDepth ) * sizeof( StackType_t ) ) );  
			if( pxStack != 0 )
			{
				pxNewTCB = ( TCB_t * ) pvPortMalloc( sizeof( TCB_t ) );  
				if( pxNewTCB != 0 )
				{
					pxNewTCB->pxStack = pxStack;
				}
				else
				{
					vPortFree( pxStack );
				}
			}
			else
			{
				pxNewTCB = 0;
			}
		}
		if( pxNewTCB != 0 )
		{
			prvInitialiseNewTask( pxTaskCode, pcName, ( uint32_t ) usStackDepth, pvParameters, uxPriority, pxCreatedTask, pxNewTCB, 0 );
			prvAddNewTaskToReadyList( pxNewTCB );
			xReturn = ( ( ( BaseType_t ) 1 ) );
		}
		else
		{
			xReturn = ( -1 );
		}
		return xReturn;
	}
static void prvInitialiseNewTask( 	TaskFunction_t pxTaskCode,
									const char * const pcName,		 
									const uint32_t ulStackDepth,
									void * const pvParameters,
									UBaseType_t uxPriority,
									TaskHandle_t * const pxCreatedTask,
									TCB_t *pxNewTCB,
									const MemoryRegion_t * const xRegions )
{
StackType_t *pxTopOfStack;
UBaseType_t x;
	{
		pxTopOfStack = &( pxNewTCB->pxStack[ ulStackDepth - ( uint32_t ) 1 ] );
		pxTopOfStack = ( StackType_t * ) ( ( ( uint32_t ) pxTopOfStack ) & ( ~( ( uint32_t ) ( 0x0007 ) ) ) );  
		;
	}
	if( pcName != 0 )
	{
		for( x = ( UBaseType_t ) 0; x < ( UBaseType_t ) ( 8 ); x++ )
		{
			pxNewTCB->pcTaskName[ x ] = pcName[ x ];
			if( pcName[ x ] == ( char ) 0x00 )
			{
				break;
			}
			else
			{
				;
			}
		}
		pxNewTCB->pcTaskName[ ( 8 ) - 1 ] = '\0';
	}
	else
	{
		pxNewTCB->pcTaskName[ 0 ] = 0x00;
	}
	if( uxPriority >= ( UBaseType_t ) ( 4 ) )
	{
		uxPriority = ( UBaseType_t ) ( 4 ) - ( UBaseType_t ) 1U;
	}
	else
	{
		;
	}
	pxNewTCB->uxPriority = uxPriority;
	vListInitialiseItem( &( pxNewTCB->xStateListItem ) );
	vListInitialiseItem( &( pxNewTCB->xEventListItem ) );
	( ( &( pxNewTCB->xStateListItem ) )->pvOwner = ( void * ) ( pxNewTCB ) );
	( ( &( pxNewTCB->xEventListItem ) )->xItemValue = ( ( TickType_t ) ( 4 ) - ( TickType_t ) uxPriority ) );  
	( ( &( pxNewTCB->xEventListItem ) )->pvOwner = ( void * ) ( pxNewTCB ) );
	{
		( void ) xRegions;
	}
	{
		pxNewTCB->ulNotifiedValue = 0;
		pxNewTCB->ucNotifyState = ( ( uint8_t ) 0 );
	}
	{
		{
			pxNewTCB->pxTopOfStack = pxPortInitialiseStack( pxTopOfStack, pxTaskCode, pvParameters );
		}
	}
	if( pxCreatedTask != 0 )
	{
		*pxCreatedTask = ( TaskHandle_t ) pxNewTCB;
	}
	else
	{
		;
	}
}
static void prvAddNewTaskToReadyList( TCB_t *pxNewTCB )
{
	vPortEnterCritical();
	{
		uxCurrentNumberOfTasks++;
		if( pxCurrentTCB == 0 )
		{
			pxCurrentTCB = pxNewTCB;
			if( uxCurrentNumberOfTasks == ( UBaseType_t ) 1 )
			{
				prvInitialiseTaskLists();
			}
			else
			{
				;
			}
		}
		else
		{
			if( xSchedulerRunning == ( ( BaseType_t ) 0 ) )
			{
				if( pxCurrentTCB->uxPriority <= pxNewTCB->uxPriority )
				{
					pxCurrentTCB = pxNewTCB;
				}
				else
				{
					;
				}
			}
			else
			{
				;
			}
		}
		uxTaskNumber++;
		;
		; ( uxTopReadyPriority ) |= ( 1UL << ( ( pxNewTCB )->uxPriority ) ); vListInsertEnd( &( pxReadyTasksLists[ ( pxNewTCB )->uxPriority ] ), &( ( pxNewTCB )->xStateListItem ) ); ;
		( void ) pxNewTCB;
	}
	vPortExitCritical();
	if( xSchedulerRunning != ( ( BaseType_t ) 0 ) )
	{
		if( pxCurrentTCB->uxPriority < pxNewTCB->uxPriority )
		{
			{ ( * ( ( volatile uint32_t * ) 0xe000ed04 ) ) = ( 1UL << 28UL ); __dsb( ( 15 ) ); __isb( ( 15 ) ); };
		}
		else
		{
			;
		}
	}
	else
	{
		;
	}
}
	void vTaskDelete( TaskHandle_t xTaskToDelete )
	{
	TCB_t *pxTCB;
		vPortEnterCritical();
		{
			pxTCB = ( ( ( xTaskToDelete ) == 0 ) ? pxCurrentTCB : ( xTaskToDelete ) );
			if( uxListRemove( &( pxTCB->xStateListItem ) ) == ( UBaseType_t ) 0 )
			{
				{ if( ( ( &( pxReadyTasksLists[ ( pxTCB->uxPriority ) ] ) )->uxNumberOfItems ) == ( UBaseType_t ) 0 ) { ( ( uxTopReadyPriority ) ) &= ~( 1UL << ( ( pxTCB->uxPriority ) ) ); } };
			}
			else
			{
				;
			}
			if( ( ( &( pxTCB->xEventListItem ) )->pvContainer ) != 0 )
			{
				( void ) uxListRemove( &( pxTCB->xEventListItem ) );
			}
			else
			{
				;
			}
			uxTaskNumber++;
			if( pxTCB == pxCurrentTCB )
			{
				vListInsertEnd( &xTasksWaitingTermination, &( pxTCB->xStateListItem ) );
				++uxDeletedTasksWaitingCleanUp;
				;
			}
			else
			{
				--uxCurrentNumberOfTasks;
				prvDeleteTCB( pxTCB );
				prvResetNextTaskUnblockTime();
			}
			;
		}
		vPortExitCritical();
		if( xSchedulerRunning != ( ( BaseType_t ) 0 ) )
		{
			if( pxTCB == pxCurrentTCB )
			{
				;
				{ ( * ( ( volatile uint32_t * ) 0xe000ed04 ) ) = ( 1UL << 28UL ); __dsb( ( 15 ) ); __isb( ( 15 ) ); };
			}
			else
			{
				;
			}
		}
	}
	void vTaskDelayUntil( TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement )
	{
	TickType_t xTimeToWake;
	BaseType_t xAlreadyYielded, xShouldDelay = ( ( BaseType_t ) 0 );
		;
		;
		;
		vTaskSuspendAll();
		{
			const TickType_t xConstTickCount = xTickCount;
			xTimeToWake = *pxPreviousWakeTime + xTimeIncrement;
			if( xConstTickCount < *pxPreviousWakeTime )
			{
				if( ( xTimeToWake < *pxPreviousWakeTime ) && ( xTimeToWake > xConstTickCount ) )
				{
					xShouldDelay = ( ( BaseType_t ) 1 );
				}
				else
				{
					;
				}
			}
			else
			{
				if( ( xTimeToWake < *pxPreviousWakeTime ) || ( xTimeToWake > xConstTickCount ) )
				{
					xShouldDelay = ( ( BaseType_t ) 1 );
				}
				else
				{
					;
				}
			}
			*pxPreviousWakeTime = xTimeToWake;
			if( xShouldDelay != ( ( BaseType_t ) 0 ) )
			{
				;
				prvAddCurrentTaskToDelayedList( xTimeToWake - xConstTickCount, ( ( BaseType_t ) 0 ) );
			}
			else
			{
				;
			}
		}
		xAlreadyYielded = xTaskResumeAll();
		if( xAlreadyYielded == ( ( BaseType_t ) 0 ) )
		{
			{ ( * ( ( volatile uint32_t * ) 0xe000ed04 ) ) = ( 1UL << 28UL ); __dsb( ( 15 ) ); __isb( ( 15 ) ); };
		}
		else
		{
			;
		}
	}
	void vTaskDelay( const TickType_t xTicksToDelay )
	{
	BaseType_t xAlreadyYielded = ( ( BaseType_t ) 0 );
		if( xTicksToDelay > ( TickType_t ) 0U )
		{
			;
			vTaskSuspendAll();
			{
				;
				prvAddCurrentTaskToDelayedList( xTicksToDelay, ( ( BaseType_t ) 0 ) );
			}
			xAlreadyYielded = xTaskResumeAll();
		}
		else
		{
			;
		}
		if( xAlreadyYielded == ( ( BaseType_t ) 0 ) )
		{
			{ ( * ( ( volatile uint32_t * ) 0xe000ed04 ) ) = ( 1UL << 28UL ); __dsb( ( 15 ) ); __isb( ( 15 ) ); };
		}
		else
		{
			;
		}
	}
	UBaseType_t uxTaskPriorityGet( const TaskHandle_t xTask )
	{
	TCB_t const *pxTCB;
	UBaseType_t uxReturn;
		vPortEnterCritical();
		{
			pxTCB = ( ( ( xTask ) == 0 ) ? pxCurrentTCB : ( xTask ) );
			uxReturn = pxTCB->uxPriority;
		}
		vPortExitCritical();
		return uxReturn;
	}
	UBaseType_t uxTaskPriorityGetFromISR( const TaskHandle_t xTask )
	{
	TCB_t const *pxTCB;
	UBaseType_t uxReturn, uxSavedInterruptState;
		;
		uxSavedInterruptState = ulPortRaiseBASEPRI();
		{
			pxTCB = ( ( ( xTask ) == 0 ) ? pxCurrentTCB : ( xTask ) );
			uxReturn = pxTCB->uxPriority;
		}
		vPortSetBASEPRI(uxSavedInterruptState);
		return uxReturn;
	}
	void vTaskPrioritySet( TaskHandle_t xTask, UBaseType_t uxNewPriority )
	{
	TCB_t *pxTCB;
	UBaseType_t uxCurrentBasePriority, uxPriorityUsedOnEntry;
	BaseType_t xYieldRequired = ( ( BaseType_t ) 0 );
		;
		if( uxNewPriority >= ( UBaseType_t ) ( 4 ) )
		{
			uxNewPriority = ( UBaseType_t ) ( 4 ) - ( UBaseType_t ) 1U;
		}
		else
		{
			;
		}
		vPortEnterCritical();
		{
			pxTCB = ( ( ( xTask ) == 0 ) ? pxCurrentTCB : ( xTask ) );
			;
			{
				uxCurrentBasePriority = pxTCB->uxPriority;
			}
			if( uxCurrentBasePriority != uxNewPriority )
			{
				if( uxNewPriority > uxCurrentBasePriority )
				{
					if( pxTCB != pxCurrentTCB )
					{
						if( uxNewPriority >= pxCurrentTCB->uxPriority )
						{
							xYieldRequired = ( ( BaseType_t ) 1 );
						}
						else
						{
							;
						}
					}
					else
					{
					}
				}
				else if( pxTCB == pxCurrentTCB )
				{
					xYieldRequired = ( ( BaseType_t ) 1 );
				}
				else
				{
				}
				uxPriorityUsedOnEntry = pxTCB->uxPriority;
				{
					pxTCB->uxPriority = uxNewPriority;
				}
				if( ( ( ( &( pxTCB->xEventListItem ) )->xItemValue ) & 0x80000000UL ) == 0UL )
				{
					( ( &( pxTCB->xEventListItem ) )->xItemValue = ( ( ( TickType_t ) ( 4 ) - ( TickType_t ) uxNewPriority ) ) );  
				}
				else
				{
					;
				}
				if( ( ( ( &( pxTCB->xStateListItem ) )->pvContainer == ( &( pxReadyTasksLists[ uxPriorityUsedOnEntry ] ) ) ) ? ( ( ( BaseType_t ) 1 ) ) : ( ( ( BaseType_t ) 0 ) ) ) != ( ( BaseType_t ) 0 ) )
				{
					if( uxListRemove( &( pxTCB->xStateListItem ) ) == ( UBaseType_t ) 0 )
					{
						( uxTopReadyPriority ) &= ~( 1UL << ( uxPriorityUsedOnEntry ) );
					}
					else
					{
						;
					}
					; ( uxTopReadyPriority ) |= ( 1UL << ( ( pxTCB )->uxPriority ) ); vListInsertEnd( &( pxReadyTasksLists[ ( pxTCB )->uxPriority ] ), &( ( pxTCB )->xStateListItem ) ); ;
				}
				else
				{
					;
				}
				if( xYieldRequired != ( ( BaseType_t ) 0 ) )
				{
					{ ( * ( ( volatile uint32_t * ) 0xe000ed04 ) ) = ( 1UL << 28UL ); __dsb( ( 15 ) ); __isb( ( 15 ) ); };
				}
				else
				{
					;
				}
				( void ) uxPriorityUsedOnEntry;
			}
		}
		vPortExitCritical();
	}
	void vTaskSuspend( TaskHandle_t xTaskToSuspend )
	{
	TCB_t *pxTCB;
		vPortEnterCritical();
		{
			pxTCB = ( ( ( xTaskToSuspend ) == 0 ) ? pxCurrentTCB : ( xTaskToSuspend ) );
			;
			if( uxListRemove( &( pxTCB->xStateListItem ) ) == ( UBaseType_t ) 0 )
			{
				{ if( ( ( &( pxReadyTasksLists[ ( pxTCB->uxPriority ) ] ) )->uxNumberOfItems ) == ( UBaseType_t ) 0 ) { ( ( uxTopReadyPriority ) ) &= ~( 1UL << ( ( pxTCB->uxPriority ) ) ); } };
			}
			else
			{
				;
			}
			if( ( ( &( pxTCB->xEventListItem ) )->pvContainer ) != 0 )
			{
				( void ) uxListRemove( &( pxTCB->xEventListItem ) );
			}
			else
			{
				;
			}
			vListInsertEnd( &xSuspendedTaskList, &( pxTCB->xStateListItem ) );
			{
				if( pxTCB->ucNotifyState == ( ( uint8_t ) 1 ) )
				{
					pxTCB->ucNotifyState = ( ( uint8_t ) 0 );
				}
			}
		}
		vPortExitCritical();
		if( xSchedulerRunning != ( ( BaseType_t ) 0 ) )
		{
			vPortEnterCritical();
			{
				prvResetNextTaskUnblockTime();
			}
			vPortExitCritical();
		}
		else
		{
			;
		}
		if( pxTCB == pxCurrentTCB )
		{
			if( xSchedulerRunning != ( ( BaseType_t ) 0 ) )
			{
				;
				{ ( * ( ( volatile uint32_t * ) 0xe000ed04 ) ) = ( 1UL << 28UL ); __dsb( ( 15 ) ); __isb( ( 15 ) ); };
			}
			else
			{
				if( ( ( &xSuspendedTaskList )->uxNumberOfItems ) == uxCurrentNumberOfTasks )  
				{
					pxCurrentTCB = 0;
				}
				else
				{
					vTaskSwitchContext();
				}
			}
		}
		else
		{
			;
		}
	}
	static BaseType_t prvTaskIsTaskSuspended( const TaskHandle_t xTask )
	{
	BaseType_t xReturn = ( ( BaseType_t ) 0 );
	const TCB_t * const pxTCB = xTask;
		;
		if( ( ( ( &( pxTCB->xStateListItem ) )->pvContainer == ( &xSuspendedTaskList ) ) ? ( ( ( BaseType_t ) 1 ) ) : ( ( ( BaseType_t ) 0 ) ) ) != ( ( BaseType_t ) 0 ) )
		{
			if( ( ( ( &( pxTCB->xEventListItem ) )->pvContainer == ( &xPendingReadyList ) ) ? ( ( ( BaseType_t ) 1 ) ) : ( ( ( BaseType_t ) 0 ) ) ) == ( ( BaseType_t ) 0 ) )
			{
				if( ( ( ( &( pxTCB->xEventListItem ) )->pvContainer == ( 0 ) ) ? ( ( ( BaseType_t ) 1 ) ) : ( ( ( BaseType_t ) 0 ) ) ) != ( ( BaseType_t ) 0 ) )  
				{
					xReturn = ( ( BaseType_t ) 1 );
				}
				else
				{
					;
				}
			}
			else
			{
				;
			}
		}
		else
		{
			;
		}
		return xReturn;
	}  
	void vTaskResume( TaskHandle_t xTaskToResume )
	{
	TCB_t * const pxTCB = xTaskToResume;
		;
		if( ( pxTCB != pxCurrentTCB ) && ( pxTCB != 0 ) )
		{
			vPortEnterCritical();
			{
				if( prvTaskIsTaskSuspended( pxTCB ) != ( ( BaseType_t ) 0 ) )
				{
					;
					( void ) uxListRemove(  &( pxTCB->xStateListItem ) );
					; ( uxTopReadyPriority ) |= ( 1UL << ( ( pxTCB )->uxPriority ) ); vListInsertEnd( &( pxReadyTasksLists[ ( pxTCB )->uxPriority ] ), &( ( pxTCB )->xStateListItem ) ); ;
					if( pxTCB->uxPriority >= pxCurrentTCB->uxPriority )
					{
						{ ( * ( ( volatile uint32_t * ) 0xe000ed04 ) ) = ( 1UL << 28UL ); __dsb( ( 15 ) ); __isb( ( 15 ) ); };
					}
					else
					{
						;
					}
				}
				else
				{
					;
				}
			}
			vPortExitCritical();
		}
		else
		{
			;
		}
	}
	BaseType_t xTaskResumeFromISR( TaskHandle_t xTaskToResume )
	{
	BaseType_t xYieldRequired = ( ( BaseType_t ) 0 );
	TCB_t * const pxTCB = xTaskToResume;
	UBaseType_t uxSavedInterruptStatus;
		;
		;
		uxSavedInterruptStatus = ulPortRaiseBASEPRI();
		{
			if( prvTaskIsTaskSuspended( pxTCB ) != ( ( BaseType_t ) 0 ) )
			{
				;
				if( uxSchedulerSuspended == ( UBaseType_t ) ( ( BaseType_t ) 0 ) )
				{
					if( pxTCB->uxPriority >= pxCurrentTCB->uxPriority )
					{
						xYieldRequired = ( ( BaseType_t ) 1 );
					}
					else
					{
						;
					}
					( void ) uxListRemove( &( pxTCB->xStateListItem ) );
					; ( uxTopReadyPriority ) |= ( 1UL << ( ( pxTCB )->uxPriority ) ); vListInsertEnd( &( pxReadyTasksLists[ ( pxTCB )->uxPriority ] ), &( ( pxTCB )->xStateListItem ) ); ;
				}
				else
				{
					vListInsertEnd( &( xPendingReadyList ), &( pxTCB->xEventListItem ) );
				}
			}
			else
			{
				;
			}
		}
		vPortSetBASEPRI(uxSavedInterruptStatus);
		return xYieldRequired;
	}
void vTaskStartScheduler( void )
{
BaseType_t xReturn;
	{
		xReturn = xTaskCreate(	prvIdleTask,
								"IDLE",
								( ( unsigned short ) 90 ),
								( void * ) 0,
								( ( UBaseType_t ) 0x00 ),  
								&xIdleTaskHandle );  
	}
	if( xReturn == ( ( ( BaseType_t ) 1 ) ) )
	{
		vPortRaiseBASEPRI();
		xNextTaskUnblockTime = ( TickType_t ) 0xffffffffUL;
		xSchedulerRunning = ( ( BaseType_t ) 1 );
		xTickCount = ( TickType_t ) 0;
		;
		;
		if( xPortStartScheduler() != ( ( BaseType_t ) 0 ) )
		{
		}
		else
		{
		}
	}
	else
	{
		;
	}
	( void ) xIdleTaskHandle;
}
void vTaskEndScheduler( void )
{
	vPortRaiseBASEPRI();
	xSchedulerRunning = ( ( BaseType_t ) 0 );
	vPortEndScheduler();
}
void vTaskSuspendAll( void )
{
	++uxSchedulerSuspended;
	;
}
BaseType_t xTaskResumeAll( void )
{
TCB_t *pxTCB = 0;
BaseType_t xAlreadyYielded = ( ( BaseType_t ) 0 );
	;
	vPortEnterCritical();
	{
		--uxSchedulerSuspended;
		if( uxSchedulerSuspended == ( UBaseType_t ) ( ( BaseType_t ) 0 ) )
		{
			if( uxCurrentNumberOfTasks > ( UBaseType_t ) 0U )
			{
				while( ( ( ( &xPendingReadyList )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) == ( ( BaseType_t ) 0 ) )
				{
					pxTCB = ( (&( ( ( &xPendingReadyList ) )->xListEnd ))->pxNext->pvOwner );  
					( void ) uxListRemove( &( pxTCB->xEventListItem ) );
					( void ) uxListRemove( &( pxTCB->xStateListItem ) );
					; ( uxTopReadyPriority ) |= ( 1UL << ( ( pxTCB )->uxPriority ) ); vListInsertEnd( &( pxReadyTasksLists[ ( pxTCB )->uxPriority ] ), &( ( pxTCB )->xStateListItem ) ); ;
					if( pxTCB->uxPriority >= pxCurrentTCB->uxPriority )
					{
						xYieldPending = ( ( BaseType_t ) 1 );
					}
					else
					{
						;
					}
				}
				if( pxTCB != 0 )
				{
					prvResetNextTaskUnblockTime();
				}
				{
					UBaseType_t uxPendedCounts = uxPendedTicks;  
					if( uxPendedCounts > ( UBaseType_t ) 0U )
					{
						do
						{
							if( xTaskIncrementTick() != ( ( BaseType_t ) 0 ) )
							{
								xYieldPending = ( ( BaseType_t ) 1 );
							}
							else
							{
								;
							}
							--uxPendedCounts;
						} while( uxPendedCounts > ( UBaseType_t ) 0U );
						uxPendedTicks = 0;
					}
					else
					{
						;
					}
				}
				if( xYieldPending != ( ( BaseType_t ) 0 ) )
				{
					{
						xAlreadyYielded = ( ( BaseType_t ) 1 );
					}
					{ ( * ( ( volatile uint32_t * ) 0xe000ed04 ) ) = ( 1UL << 28UL ); __dsb( ( 15 ) ); __isb( ( 15 ) ); };
				}
				else
				{
					;
				}
			}
		}
		else
		{
			;
		}
	}
	vPortExitCritical();
	return xAlreadyYielded;
}
TickType_t xTaskGetTickCount( void )
{
TickType_t xTicks;
	;
	{
		xTicks = xTickCount;
	}
	;
	return xTicks;
}
TickType_t xTaskGetTickCountFromISR( void )
{
TickType_t xReturn;
UBaseType_t uxSavedInterruptStatus;
	;
	uxSavedInterruptStatus = 0;
	{
		xReturn = xTickCount;
	}
	( void ) uxSavedInterruptStatus;
	return xReturn;
}
UBaseType_t uxTaskGetNumberOfTasks( void )
{
	return uxCurrentNumberOfTasks;
}
char *pcTaskGetName( TaskHandle_t xTaskToQuery )  
{
TCB_t *pxTCB;
	pxTCB = ( ( ( xTaskToQuery ) == 0 ) ? pxCurrentTCB : ( xTaskToQuery ) );
	;
	return &( pxTCB->pcTaskName[ 0 ] );
}
BaseType_t xTaskIncrementTick( void )
{
TCB_t * pxTCB;
TickType_t xItemValue;
BaseType_t xSwitchRequired = ( ( BaseType_t ) 0 );
	;
	if( uxSchedulerSuspended == ( UBaseType_t ) ( ( BaseType_t ) 0 ) )
	{
		const TickType_t xConstTickCount = xTickCount + ( TickType_t ) 1;
		xTickCount = xConstTickCount;
		if( xConstTickCount == ( TickType_t ) 0U )  
		{
			{ List_t *pxTemp; ; pxTemp = pxDelayedTaskList; pxDelayedTaskList = pxOverflowDelayedTaskList; pxOverflowDelayedTaskList = pxTemp; xNumOfOverflows++; prvResetNextTaskUnblockTime(); };
		}
		else
		{
			;
		}
		if( xConstTickCount >= xNextTaskUnblockTime )
		{
			for( ;; )
			{
				if( ( ( ( pxDelayedTaskList )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) != ( ( BaseType_t ) 0 ) )
				{
					xNextTaskUnblockTime = ( TickType_t ) 0xffffffffUL;  
					break;
				}
				else
				{
					pxTCB = ( (&( ( pxDelayedTaskList )->xListEnd ))->pxNext->pvOwner );  
					xItemValue = ( ( &( pxTCB->xStateListItem ) )->xItemValue );
					if( xConstTickCount < xItemValue )
					{
						xNextTaskUnblockTime = xItemValue;
						break;  
					}
					else
					{
						;
					}
					( void ) uxListRemove( &( pxTCB->xStateListItem ) );
					if( ( ( &( pxTCB->xEventListItem ) )->pvContainer ) != 0 )
					{
						( void ) uxListRemove( &( pxTCB->xEventListItem ) );
					}
					else
					{
						;
					}
					; ( uxTopReadyPriority ) |= ( 1UL << ( ( pxTCB )->uxPriority ) ); vListInsertEnd( &( pxReadyTasksLists[ ( pxTCB )->uxPriority ] ), &( ( pxTCB )->xStateListItem ) ); ;
					{
						if( pxTCB->uxPriority >= pxCurrentTCB->uxPriority )
						{
							xSwitchRequired = ( ( BaseType_t ) 1 );
						}
						else
						{
							;
						}
					}
				}
			}
		}
		{
			if( ( ( &( pxReadyTasksLists[ pxCurrentTCB->uxPriority ] ) )->uxNumberOfItems ) > ( UBaseType_t ) 1 )
			{
				xSwitchRequired = ( ( BaseType_t ) 1 );
			}
			else
			{
				;
			}
		}
	}
	else
	{
		++uxPendedTicks;
	}
	{
		if( xYieldPending != ( ( BaseType_t ) 0 ) )
		{
			xSwitchRequired = ( ( BaseType_t ) 1 );
		}
		else
		{
			;
		}
	}
	return xSwitchRequired;
}
void vTaskSwitchContext( void )
{
	if( uxSchedulerSuspended != ( UBaseType_t ) ( ( BaseType_t ) 0 ) )
	{
		xYieldPending = ( ( BaseType_t ) 1 );
	}
	else
	{
		xYieldPending = ( ( BaseType_t ) 0 );
		;
		;
		{ UBaseType_t uxTopPriority; uxTopPriority = ( 31UL - ( uint32_t ) __clz( ( uxTopReadyPriority ) ) ); ; { List_t * const pxConstList = ( &( pxReadyTasksLists[ uxTopPriority ] ) ); ( pxConstList )->pxIndex = ( pxConstList )->pxIndex->pxNext; if( ( void * ) ( pxConstList )->pxIndex == ( void * ) &( ( pxConstList )->xListEnd ) ) { ( pxConstList )->pxIndex = ( pxConstList )->pxIndex->pxNext; } ( pxCurrentTCB ) = ( pxConstList )->pxIndex->pvOwner; }; };  
		;
	}
}
void vTaskPlaceOnEventList( List_t * const pxEventList, const TickType_t xTicksToWait )
{
	;
	vListInsert( pxEventList, &( pxCurrentTCB->xEventListItem ) );
	prvAddCurrentTaskToDelayedList( xTicksToWait, ( ( BaseType_t ) 1 ) );
}
void vTaskPlaceOnUnorderedEventList( List_t * pxEventList, const TickType_t xItemValue, const TickType_t xTicksToWait )
{
	;
	;
	( ( &( pxCurrentTCB->xEventListItem ) )->xItemValue = ( xItemValue | 0x80000000UL ) );
	vListInsertEnd( pxEventList, &( pxCurrentTCB->xEventListItem ) );
	prvAddCurrentTaskToDelayedList( xTicksToWait, ( ( BaseType_t ) 1 ) );
}
BaseType_t xTaskRemoveFromEventList( const List_t * const pxEventList )
{
TCB_t *pxUnblockedTCB;
BaseType_t xReturn;
	pxUnblockedTCB = ( (&( ( pxEventList )->xListEnd ))->pxNext->pvOwner );  
	;
	( void ) uxListRemove( &( pxUnblockedTCB->xEventListItem ) );
	if( uxSchedulerSuspended == ( UBaseType_t ) ( ( BaseType_t ) 0 ) )
	{
		( void ) uxListRemove( &( pxUnblockedTCB->xStateListItem ) );
		; ( uxTopReadyPriority ) |= ( 1UL << ( ( pxUnblockedTCB )->uxPriority ) ); vListInsertEnd( &( pxReadyTasksLists[ ( pxUnblockedTCB )->uxPriority ] ), &( ( pxUnblockedTCB )->xStateListItem ) ); ;
	}
	else
	{
		vListInsertEnd( &( xPendingReadyList ), &( pxUnblockedTCB->xEventListItem ) );
	}
	if( pxUnblockedTCB->uxPriority > pxCurrentTCB->uxPriority )
	{
		xReturn = ( ( BaseType_t ) 1 );
		xYieldPending = ( ( BaseType_t ) 1 );
	}
	else
	{
		xReturn = ( ( BaseType_t ) 0 );
	}
	return xReturn;
}
void vTaskRemoveFromUnorderedEventList( ListItem_t * pxEventListItem, const TickType_t xItemValue )
{
TCB_t *pxUnblockedTCB;
	;
	( ( pxEventListItem )->xItemValue = ( xItemValue | 0x80000000UL ) );
	pxUnblockedTCB = ( ( pxEventListItem )->pvOwner );  
	;
	( void ) uxListRemove( pxEventListItem );
	( void ) uxListRemove( &( pxUnblockedTCB->xStateListItem ) );
	; ( uxTopReadyPriority ) |= ( 1UL << ( ( pxUnblockedTCB )->uxPriority ) ); vListInsertEnd( &( pxReadyTasksLists[ ( pxUnblockedTCB )->uxPriority ] ), &( ( pxUnblockedTCB )->xStateListItem ) ); ;
	if( pxUnblockedTCB->uxPriority > pxCurrentTCB->uxPriority )
	{
		xYieldPending = ( ( BaseType_t ) 1 );
	}
}
void vTaskSetTimeOutState( TimeOut_t * const pxTimeOut )
{
	;
	vPortEnterCritical();
	{
		pxTimeOut->xOverflowCount = xNumOfOverflows;
		pxTimeOut->xTimeOnEntering = xTickCount;
	}
	vPortExitCritical();
}
void vTaskInternalSetTimeOutState( TimeOut_t * const pxTimeOut )
{
	pxTimeOut->xOverflowCount = xNumOfOverflows;
	pxTimeOut->xTimeOnEntering = xTickCount;
}
BaseType_t xTaskCheckForTimeOut( TimeOut_t * const pxTimeOut, TickType_t * const pxTicksToWait )
{
BaseType_t xReturn;
	;
	;
	vPortEnterCritical();
	{
		const TickType_t xConstTickCount = xTickCount;
		const TickType_t xElapsedTime = xConstTickCount - pxTimeOut->xTimeOnEntering;
			if( *pxTicksToWait == ( TickType_t ) 0xffffffffUL )
			{
				xReturn = ( ( BaseType_t ) 0 );
			}
			else
		if( ( xNumOfOverflows != pxTimeOut->xOverflowCount ) && ( xConstTickCount >= pxTimeOut->xTimeOnEntering ) )  
		{
			xReturn = ( ( BaseType_t ) 1 );
		}
		else if( xElapsedTime < *pxTicksToWait )  
		{
			*pxTicksToWait -= xElapsedTime;
			vTaskInternalSetTimeOutState( pxTimeOut );
			xReturn = ( ( BaseType_t ) 0 );
		}
		else
		{
			*pxTicksToWait = 0;
			xReturn = ( ( BaseType_t ) 1 );
		}
	}
	vPortExitCritical();
	return xReturn;
}
void vTaskMissedYield( void )
{
	xYieldPending = ( ( BaseType_t ) 1 );
}
static void prvIdleTask( void *pvParameters )
{
	( void ) pvParameters;
	;
	for( ;; )
	{
		prvCheckTasksWaitingTermination();
		{
			if( ( ( &( pxReadyTasksLists[ ( ( UBaseType_t ) 0U ) ] ) )->uxNumberOfItems ) > ( UBaseType_t ) 1 )
			{
				{ ( * ( ( volatile uint32_t * ) 0xe000ed04 ) ) = ( 1UL << 28UL ); __dsb( ( 15 ) ); __isb( ( 15 ) ); };
			}
			else
			{
				;
			}
		}
	}
}
static void prvInitialiseTaskLists( void )
{
UBaseType_t uxPriority;
	for( uxPriority = ( UBaseType_t ) 0U; uxPriority < ( UBaseType_t ) ( 4 ); uxPriority++ )
	{
		vListInitialise( &( pxReadyTasksLists[ uxPriority ] ) );
	}
	vListInitialise( &xDelayedTaskList1 );
	vListInitialise( &xDelayedTaskList2 );
	vListInitialise( &xPendingReadyList );
	{
		vListInitialise( &xTasksWaitingTermination );
	}
	{
		vListInitialise( &xSuspendedTaskList );
	}
	pxDelayedTaskList = &xDelayedTaskList1;
	pxOverflowDelayedTaskList = &xDelayedTaskList2;
}
static void prvCheckTasksWaitingTermination( void )
{
	{
		TCB_t *pxTCB;
		while( uxDeletedTasksWaitingCleanUp > ( UBaseType_t ) 0U )
		{
			vPortEnterCritical();
			{
				pxTCB = ( (&( ( ( &xTasksWaitingTermination ) )->xListEnd ))->pxNext->pvOwner );  
				( void ) uxListRemove( &( pxTCB->xStateListItem ) );
				--uxCurrentNumberOfTasks;
				--uxDeletedTasksWaitingCleanUp;
			}
			vPortExitCritical();
			prvDeleteTCB( pxTCB );
		}
	}
}
	static void prvDeleteTCB( TCB_t *pxTCB )
	{
		( void ) pxTCB;
		{
			vPortFree( pxTCB->pxStack );
			vPortFree( pxTCB );
		}
	}
static void prvResetNextTaskUnblockTime( void )
{
TCB_t *pxTCB;
	if( ( ( ( pxDelayedTaskList )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) != ( ( BaseType_t ) 0 ) )
	{
		xNextTaskUnblockTime = ( TickType_t ) 0xffffffffUL;
	}
	else
	{
		( pxTCB ) = ( (&( ( pxDelayedTaskList )->xListEnd ))->pxNext->pvOwner );  
		xNextTaskUnblockTime = ( ( &( ( pxTCB )->xStateListItem ) )->xItemValue );
	}
}
TickType_t uxTaskResetEventItemValue( void )
{
TickType_t uxReturn;
	uxReturn = ( ( &( pxCurrentTCB->xEventListItem ) )->xItemValue );
	( ( &( pxCurrentTCB->xEventListItem ) )->xItemValue = ( ( ( TickType_t ) ( 4 ) - ( TickType_t ) pxCurrentTCB->uxPriority ) ) );  
	return uxReturn;
}
	uint32_t ulTaskNotifyTake( BaseType_t xClearCountOnExit, TickType_t xTicksToWait )
	{
	uint32_t ulReturn;
		vPortEnterCritical();
		{
			if( pxCurrentTCB->ulNotifiedValue == 0UL )
			{
				pxCurrentTCB->ucNotifyState = ( ( uint8_t ) 1 );
				if( xTicksToWait > ( TickType_t ) 0 )
				{
					prvAddCurrentTaskToDelayedList( xTicksToWait, ( ( BaseType_t ) 1 ) );
					;
					{ ( * ( ( volatile uint32_t * ) 0xe000ed04 ) ) = ( 1UL << 28UL ); __dsb( ( 15 ) ); __isb( ( 15 ) ); };
				}
				else
				{
					;
				}
			}
			else
			{
				;
			}
		}
		vPortExitCritical();
		vPortEnterCritical();
		{
			;
			ulReturn = pxCurrentTCB->ulNotifiedValue;
			if( ulReturn != 0UL )
			{
				if( xClearCountOnExit != ( ( BaseType_t ) 0 ) )
				{
					pxCurrentTCB->ulNotifiedValue = 0UL;
				}
				else
				{
					pxCurrentTCB->ulNotifiedValue = ulReturn - ( uint32_t ) 1;
				}
			}
			else
			{
				;
			}
			pxCurrentTCB->ucNotifyState = ( ( uint8_t ) 0 );
		}
		vPortExitCritical();
		return ulReturn;
	}
	BaseType_t xTaskNotifyWait( uint32_t ulBitsToClearOnEntry, uint32_t ulBitsToClearOnExit, uint32_t *pulNotificationValue, TickType_t xTicksToWait )
	{
	BaseType_t xReturn;
		vPortEnterCritical();
		{
			if( pxCurrentTCB->ucNotifyState != ( ( uint8_t ) 2 ) )
			{
				pxCurrentTCB->ulNotifiedValue &= ~ulBitsToClearOnEntry;
				pxCurrentTCB->ucNotifyState = ( ( uint8_t ) 1 );
				if( xTicksToWait > ( TickType_t ) 0 )
				{
					prvAddCurrentTaskToDelayedList( xTicksToWait, ( ( BaseType_t ) 1 ) );
					;
					{ ( * ( ( volatile uint32_t * ) 0xe000ed04 ) ) = ( 1UL << 28UL ); __dsb( ( 15 ) ); __isb( ( 15 ) ); };
				}
				else
				{
					;
				}
			}
			else
			{
				;
			}
		}
		vPortExitCritical();
		vPortEnterCritical();
		{
			;
			if( pulNotificationValue != 0 )
			{
				*pulNotificationValue = pxCurrentTCB->ulNotifiedValue;
			}
			if( pxCurrentTCB->ucNotifyState != ( ( uint8_t ) 2 ) )
			{
				xReturn = ( ( BaseType_t ) 0 );
			}
			else
			{
				pxCurrentTCB->ulNotifiedValue &= ~ulBitsToClearOnExit;
				xReturn = ( ( BaseType_t ) 1 );
			}
			pxCurrentTCB->ucNotifyState = ( ( uint8_t ) 0 );
		}
		vPortExitCritical();
		return xReturn;
	}
	BaseType_t xTaskGenericNotify( TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, uint32_t *pulPreviousNotificationValue )
	{
	TCB_t * pxTCB;
	BaseType_t xReturn = ( ( ( BaseType_t ) 1 ) );
	uint8_t ucOriginalNotifyState;
		;
		pxTCB = xTaskToNotify;
		vPortEnterCritical();
		{
			if( pulPreviousNotificationValue != 0 )
			{
				*pulPreviousNotificationValue = pxTCB->ulNotifiedValue;
			}
			ucOriginalNotifyState = pxTCB->ucNotifyState;
			pxTCB->ucNotifyState = ( ( uint8_t ) 2 );
			switch( eAction )
			{
				case eSetBits	:
					pxTCB->ulNotifiedValue |= ulValue;
					break;
				case eIncrement	:
					( pxTCB->ulNotifiedValue )++;
					break;
				case eSetValueWithOverwrite	:
					pxTCB->ulNotifiedValue = ulValue;
					break;
				case eSetValueWithoutOverwrite :
					if( ucOriginalNotifyState != ( ( uint8_t ) 2 ) )
					{
						pxTCB->ulNotifiedValue = ulValue;
					}
					else
					{
						xReturn = ( ( ( BaseType_t ) 0 ) );
					}
					break;
				case eNoAction:
					break;
				default:
					;
					break;
			}
			;
			if( ucOriginalNotifyState == ( ( uint8_t ) 1 ) )
			{
				( void ) uxListRemove( &( pxTCB->xStateListItem ) );
				; ( uxTopReadyPriority ) |= ( 1UL << ( ( pxTCB )->uxPriority ) ); vListInsertEnd( &( pxReadyTasksLists[ ( pxTCB )->uxPriority ] ), &( ( pxTCB )->xStateListItem ) ); ;
				;
				if( pxTCB->uxPriority > pxCurrentTCB->uxPriority )
				{
					{ ( * ( ( volatile uint32_t * ) 0xe000ed04 ) ) = ( 1UL << 28UL ); __dsb( ( 15 ) ); __isb( ( 15 ) ); };
				}
				else
				{
					;
				}
			}
			else
			{
				;
			}
		}
		vPortExitCritical();
		return xReturn;
	}
	BaseType_t xTaskGenericNotifyFromISR( TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, uint32_t *pulPreviousNotificationValue, BaseType_t *pxHigherPriorityTaskWoken )
	{
	TCB_t * pxTCB;
	uint8_t ucOriginalNotifyState;
	BaseType_t xReturn = ( ( ( BaseType_t ) 1 ) );
	UBaseType_t uxSavedInterruptStatus;
		;
		;
		pxTCB = xTaskToNotify;
		uxSavedInterruptStatus = ulPortRaiseBASEPRI();
		{
			if( pulPreviousNotificationValue != 0 )
			{
				*pulPreviousNotificationValue = pxTCB->ulNotifiedValue;
			}
			ucOriginalNotifyState = pxTCB->ucNotifyState;
			pxTCB->ucNotifyState = ( ( uint8_t ) 2 );
			switch( eAction )
			{
				case eSetBits	:
					pxTCB->ulNotifiedValue |= ulValue;
					break;
				case eIncrement	:
					( pxTCB->ulNotifiedValue )++;
					break;
				case eSetValueWithOverwrite	:
					pxTCB->ulNotifiedValue = ulValue;
					break;
				case eSetValueWithoutOverwrite :
					if( ucOriginalNotifyState != ( ( uint8_t ) 2 ) )
					{
						pxTCB->ulNotifiedValue = ulValue;
					}
					else
					{
						xReturn = ( ( ( BaseType_t ) 0 ) );
					}
					break;
				case eNoAction :
					break;
				default:
					;
					break;
			}
			;
			if( ucOriginalNotifyState == ( ( uint8_t ) 1 ) )
			{
				;
				if( uxSchedulerSuspended == ( UBaseType_t ) ( ( BaseType_t ) 0 ) )
				{
					( void ) uxListRemove( &( pxTCB->xStateListItem ) );
					; ( uxTopReadyPriority ) |= ( 1UL << ( ( pxTCB )->uxPriority ) ); vListInsertEnd( &( pxReadyTasksLists[ ( pxTCB )->uxPriority ] ), &( ( pxTCB )->xStateListItem ) ); ;
				}
				else
				{
					vListInsertEnd( &( xPendingReadyList ), &( pxTCB->xEventListItem ) );
				}
				if( pxTCB->uxPriority > pxCurrentTCB->uxPriority )
				{
					if( pxHigherPriorityTaskWoken != 0 )
					{
						*pxHigherPriorityTaskWoken = ( ( BaseType_t ) 1 );
					}
					xYieldPending = ( ( BaseType_t ) 1 );
				}
				else
				{
					;
				}
			}
		}
		vPortSetBASEPRI(uxSavedInterruptStatus);
		return xReturn;
	}
	void vTaskNotifyGiveFromISR( TaskHandle_t xTaskToNotify, BaseType_t *pxHigherPriorityTaskWoken )
	{
	TCB_t * pxTCB;
	uint8_t ucOriginalNotifyState;
	UBaseType_t uxSavedInterruptStatus;
		;
		;
		pxTCB = xTaskToNotify;
		uxSavedInterruptStatus = ulPortRaiseBASEPRI();
		{
			ucOriginalNotifyState = pxTCB->ucNotifyState;
			pxTCB->ucNotifyState = ( ( uint8_t ) 2 );
			( pxTCB->ulNotifiedValue )++;
			;
			if( ucOriginalNotifyState == ( ( uint8_t ) 1 ) )
			{
				;
				if( uxSchedulerSuspended == ( UBaseType_t ) ( ( BaseType_t ) 0 ) )
				{
					( void ) uxListRemove( &( pxTCB->xStateListItem ) );
					; ( uxTopReadyPriority ) |= ( 1UL << ( ( pxTCB )->uxPriority ) ); vListInsertEnd( &( pxReadyTasksLists[ ( pxTCB )->uxPriority ] ), &( ( pxTCB )->xStateListItem ) ); ;
				}
				else
				{
					vListInsertEnd( &( xPendingReadyList ), &( pxTCB->xEventListItem ) );
				}
				if( pxTCB->uxPriority > pxCurrentTCB->uxPriority )
				{
					if( pxHigherPriorityTaskWoken != 0 )
					{
						*pxHigherPriorityTaskWoken = ( ( BaseType_t ) 1 );
					}
					xYieldPending = ( ( BaseType_t ) 1 );
				}
				else
				{
					;
				}
			}
		}
		vPortSetBASEPRI(uxSavedInterruptStatus);
	}
	BaseType_t xTaskNotifyStateClear( TaskHandle_t xTask )
	{
	TCB_t *pxTCB;
	BaseType_t xReturn;
		pxTCB = ( ( ( xTask ) == 0 ) ? pxCurrentTCB : ( xTask ) );
		vPortEnterCritical();
		{
			if( pxTCB->ucNotifyState == ( ( uint8_t ) 2 ) )
			{
				pxTCB->ucNotifyState = ( ( uint8_t ) 0 );
				xReturn = ( ( ( BaseType_t ) 1 ) );
			}
			else
			{
				xReturn = ( ( ( BaseType_t ) 0 ) );
			}
		}
		vPortExitCritical();
		return xReturn;
	}
static void prvAddCurrentTaskToDelayedList( TickType_t xTicksToWait, const BaseType_t xCanBlockIndefinitely )
{
TickType_t xTimeToWake;
const TickType_t xConstTickCount = xTickCount;
	if( uxListRemove( &( pxCurrentTCB->xStateListItem ) ) == ( UBaseType_t ) 0 )
	{
		( uxTopReadyPriority ) &= ~( 1UL << ( pxCurrentTCB->uxPriority ) );  
	}
	else
	{
		;
	}
	{
		if( ( xTicksToWait == ( TickType_t ) 0xffffffffUL ) && ( xCanBlockIndefinitely != ( ( BaseType_t ) 0 ) ) )
		{
			vListInsertEnd( &xSuspendedTaskList, &( pxCurrentTCB->xStateListItem ) );
		}
		else
		{
			xTimeToWake = xConstTickCount + xTicksToWait;
			( ( &( pxCurrentTCB->xStateListItem ) )->xItemValue = ( xTimeToWake ) );
			if( xTimeToWake < xConstTickCount )
			{
				vListInsert( pxOverflowDelayedTaskList, &( pxCurrentTCB->xStateListItem ) );
			}
			else
			{
				vListInsert( pxDelayedTaskList, &( pxCurrentTCB->xStateListItem ) );
				if( xTimeToWake < xNextTaskUnblockTime )
				{
					xNextTaskUnblockTime = xTimeToWake;
				}
				else
				{
					;
				}
			}
		}
	}
}
