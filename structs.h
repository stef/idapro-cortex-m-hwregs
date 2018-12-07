typedef struct
{
  unsigned int ISER[8];                      /*!< Offset: 0x000  Interrupt Set Enable Register           */
  unsigned int RESERVED0[24];
  unsigned int ICER[8];                      /*!< Offset: 0x080  Interrupt Clear Enable Register         */
  unsigned int RSERVED1[24];
  unsigned int ISPR[8];                      /*!< Offset: 0x100  Interrupt Set Pending Register          */
  unsigned int RESERVED2[24];
  unsigned int ICPR[8];                      /*!< Offset: 0x180  Interrupt Clear Pending Register        */
  unsigned int RESERVED3[24];
  unsigned int IABR[8];                      /*!< Offset: 0x200  Interrupt Active bit Register           */
  unsigned int RESERVED4[56];
  unsigned char  IP[240];                      /*!< Offset: 0x300  Interrupt Priority Register (8Bit wide) */
  unsigned int RESERVED5[644];
  unsigned int STIR;                         /*!< Offset: 0xE00  Software Trigger Interrupt Register     */
}  NVIC_Type;

typedef struct
{
  unsigned int CPUID;                        /*!< Offset: 0x00  CPU ID Base Register                                  */
  unsigned int ICSR;                         /*!< Offset: 0x04  Interrupt Control State Register                      */
  unsigned int VTOR;                         /*!< Offset: 0x08  Vector Table Offset Register                          */
  unsigned int AIRCR;                        /*!< Offset: 0x0C  Application Interrupt / Reset Control Register        */
  unsigned int SCR;                          /*!< Offset: 0x10  System Control Register                               */
  unsigned int CCR;                          /*!< Offset: 0x14  Configuration Control Register                        */
  unsigned char  SHP[12];                      /*!< Offset: 0x18  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  unsigned int SHCSR;                        /*!< Offset: 0x24  System Handler Control and State Register             */
  unsigned int CFSR;                         /*!< Offset: 0x28  Configurable Fault Status Register                    */
  unsigned int HFSR;                         /*!< Offset: 0x2C  Hard Fault Status Register                            */
  unsigned int DFSR;                         /*!< Offset: 0x30  Debug Fault Status Register                           */
  unsigned int MMFAR;                        /*!< Offset: 0x34  Mem Manage Address Register                           */
  unsigned int BFAR;                         /*!< Offset: 0x38  Bus Fault Address Register                            */
  unsigned int AFSR;                         /*!< Offset: 0x3C  Auxiliary Fault Status Register                       */
  unsigned int PFR[2];                       /*!< Offset: 0x40  Processor Feature Register                            */
  unsigned int DFR;                          /*!< Offset: 0x48  Debug Feature Register                                */
  unsigned int ADR;                          /*!< Offset: 0x4C  Auxiliary Feature Register                            */
  unsigned int MMFR[4];                      /*!< Offset: 0x50  Memory Model Feature Register                         */
  unsigned int ISAR[5];                      /*!< Offset: 0x60  ISA Feature Register                                  */
} SCB_Type;

typedef struct
{
  unsigned int CTRL;                         /*!< Offset: 0x00  SysTick Control and Status Register */
  unsigned int LOAD;                         /*!< Offset: 0x04  SysTick Reload Value Register       */
  unsigned int VAL;                          /*!< Offset: 0x08  SysTick Current Value Register      */
  unsigned int CALIB;                        /*!< Offset: 0x0C  SysTick Calibration Register        */
} SysTick_Type;

typedef struct
{
  union
  {
    unsigned char    u8;                       /*!< Offset:       ITM Stimulus Port 8-bit                   */
    unsigned short   u16;                      /*!< Offset:       ITM Stimulus Port 16-bit                  */
    unsigned int   u32;                      /*!< Offset:       ITM Stimulus Port 32-bit                  */
  }  PORT [32];                               /*!< Offset: 0x00  ITM Stimulus Port Registers               */
  unsigned int RESERVED0[864];
  unsigned int TER;                          /*!< Offset:       ITM Trace Enable Register                 */
  unsigned int RESERVED1[15];
  unsigned int TPR;                          /*!< Offset:       ITM Trace Privilege Register              */
  unsigned int RESERVED2[15];
  unsigned int TCR;                          /*!< Offset:       ITM Trace Control Register                */
  unsigned int RESERVED3[29];
  unsigned int IWR;                          /*!< Offset:       ITM Integration Write Register            */
  unsigned int IRR;                          /*!< Offset:       ITM Integration Read Register             */
  unsigned int IMCR;                         /*!< Offset:       ITM Integration Mode Control Register     */
  unsigned int RESERVED4[43];
  unsigned int LAR;                          /*!< Offset:       ITM Lock Access Register                  */
  unsigned int LSR;                          /*!< Offset:       ITM Lock Status Register                  */
  unsigned int RESERVED5[6];
  unsigned int PID4;                         /*!< Offset:       ITM Peripheral Identification Register #4 */
  unsigned int PID5;                         /*!< Offset:       ITM Peripheral Identification Register #5 */
  unsigned int PID6;                         /*!< Offset:       ITM Peripheral Identification Register #6 */
  unsigned int PID7;                         /*!< Offset:       ITM Peripheral Identification Register #7 */
  unsigned int PID0;                         /*!< Offset:       ITM Peripheral Identification Register #0 */
  unsigned int PID1;                         /*!< Offset:       ITM Peripheral Identification Register #1 */
  unsigned int PID2;                         /*!< Offset:       ITM Peripheral Identification Register #2 */
  unsigned int PID3;                         /*!< Offset:       ITM Peripheral Identification Register #3 */
  unsigned int CID0;                         /*!< Offset:       ITM Component  Identification Register #0 */
  unsigned int CID1;                         /*!< Offset:       ITM Component  Identification Register #1 */
  unsigned int CID2;                         /*!< Offset:       ITM Component  Identification Register #2 */
  unsigned int CID3;                         /*!< Offset:       ITM Component  Identification Register #3 */
} ITM_Type;

typedef struct
{
  unsigned int RESERVED0;
  unsigned int ICTR;                         /*!< Offset: 0x04  Interrupt Control Type Register */
#if ((defined __CM3_REV) && (__CM3_REV >= 0x200))
  unsigned int ACTLR;                        /*!< Offset: 0x08  Auxiliary Control Register      */
#else
  unsigned int RESERVED1;
#endif
} InterruptType_Type;

typedef struct
{
  unsigned int TYPE;                         /*!< Offset: 0x00  MPU Type Register                              */
  unsigned int CTRL;                         /*!< Offset: 0x04  MPU Control Register                           */
  unsigned int RNR;                          /*!< Offset: 0x08  MPU Region RNRber Register                     */
  unsigned int RBAR;                         /*!< Offset: 0x0C  MPU Region Base Address Register               */
  unsigned int RASR;                         /*!< Offset: 0x10  MPU Region Attribute and Size Register         */
  unsigned int RBAR_A1;                      /*!< Offset: 0x14  MPU Alias 1 Region Base Address Register       */
  unsigned int RASR_A1;                      /*!< Offset: 0x18  MPU Alias 1 Region Attribute and Size Register */
  unsigned int RBAR_A2;                      /*!< Offset: 0x1C  MPU Alias 2 Region Base Address Register       */
  unsigned int RASR_A2;                      /*!< Offset: 0x20  MPU Alias 2 Region Attribute and Size Register */
  unsigned int RBAR_A3;                      /*!< Offset: 0x24  MPU Alias 3 Region Base Address Register       */
  unsigned int RASR_A3;                      /*!< Offset: 0x28  MPU Alias 3 Region Attribute and Size Register */
} MPU_Type;

typedef struct
{
  unsigned int DHCSR;                        /*!< Offset: 0x00  Debug Halting Control and Status Register    */
  unsigned int DCRSR;                        /*!< Offset: 0x04  Debug Core Register Selector Register        */
  unsigned int DCRDR;                        /*!< Offset: 0x08  Debug Core Register Data Register            */
  unsigned int DEMCR;                        /*!< Offset: 0x0C  Debug Exception and Monitor Control Register */
} CoreDebug_Type;

typedef struct
{
  unsigned int SR;     /*!< ADC status register,                         Address offset: 0x00 */
  unsigned int CR1;    /*!< ADC control register 1,                      Address offset: 0x04 */
  unsigned int CR2;    /*!< ADC control register 2,                      Address offset: 0x08 */
  unsigned int SMPR1;  /*!< ADC sample time register 1,                  Address offset: 0x0C */
  unsigned int SMPR2;  /*!< ADC sample time register 2,                  Address offset: 0x10 */
  unsigned int JOFR1;  /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
  unsigned int JOFR2;  /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
  unsigned int JOFR3;  /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
  unsigned int JOFR4;  /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
  unsigned int HTR;    /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
  unsigned int LTR;    /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
  unsigned int SQR1;   /*!< ADC regular sequence register 1,             Address offset: 0x2C */
  unsigned int SQR2;   /*!< ADC regular sequence register 2,             Address offset: 0x30 */
  unsigned int SQR3;   /*!< ADC regular sequence register 3,             Address offset: 0x34 */
  unsigned int JSQR;   /*!< ADC injected sequence register,              Address offset: 0x38*/
  unsigned int JDR1;   /*!< ADC injected data register 1,                Address offset: 0x3C */
  unsigned int JDR2;   /*!< ADC injected data register 2,                Address offset: 0x40 */
  unsigned int JDR3;   /*!< ADC injected data register 3,                Address offset: 0x44 */
  unsigned int JDR4;   /*!< ADC injected data register 4,                Address offset: 0x48 */
  unsigned int DR;     /*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_TypeDef;

typedef struct
{
  unsigned int CSR;    /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
  unsigned int CCR;    /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
  unsigned int CDR;    /*!< ADC common regular data register for dual
                     AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;

typedef struct
{
  unsigned int TIR;  /*!< CAN TX mailbox identifier register */
  unsigned int TDTR; /*!< CAN mailbox data length control and time stamp register */
  unsigned int TDLR; /*!< CAN mailbox data low register */
  unsigned int TDHR; /*!< CAN mailbox data high register */
} CAN_TxMailBox_TypeDef;

/**
 * @brief Controller Area Network FIFOMailBox
 */

typedef struct
{
  unsigned int RIR;  /*!< CAN receive FIFO mailbox identifier register */
  unsigned int RDTR; /*!< CAN receive FIFO mailbox data length control and time stamp register */
  unsigned int RDLR; /*!< CAN receive FIFO mailbox data low register */
  unsigned int RDHR; /*!< CAN receive FIFO mailbox data high register */
} CAN_FIFOMailBox_TypeDef;

/**
 * @brief Controller Area Network FilterRegister
 */

typedef struct
{
  unsigned int FR1; /*!< CAN Filter bank register 1 */
  unsigned int FR2; /*!< CAN Filter bank register 1 */
} CAN_FilterRegister_TypeDef;

/**
 * @brief Controller Area Network
 */

typedef struct
{
  unsigned int              MCR;                 /*!< CAN master control register,         Address offset: 0x00          */
  unsigned int              MSR;                 /*!< CAN master status register,          Address offset: 0x04          */
  unsigned int              TSR;                 /*!< CAN transmit status register,        Address offset: 0x08          */
  unsigned int              RF0R;                /*!< CAN receive FIFO 0 register,         Address offset: 0x0C          */
  unsigned int              RF1R;                /*!< CAN receive FIFO 1 register,         Address offset: 0x10          */
  unsigned int              IER;                 /*!< CAN interrupt enable register,       Address offset: 0x14          */
  unsigned int              ESR;                 /*!< CAN error status register,           Address offset: 0x18          */
  unsigned int              BTR;                 /*!< CAN bit timing register,             Address offset: 0x1C          */
  unsigned int                   RESERVED0[88];       /*!< Reserved, 0x020 - 0x17F                                            */
  CAN_TxMailBox_TypeDef      sTxMailBox[3];       /*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];     /*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
  unsigned int                   RESERVED1[12];       /*!< Reserved, 0x1D0 - 0x1FF                                            */
  unsigned int              FMR;                 /*!< CAN filter master register,          Address offset: 0x200         */
  unsigned int              FM1R;                /*!< CAN filter mode register,            Address offset: 0x204         */
  unsigned int                   RESERVED2;           /*!< Reserved, 0x208                                                    */
  unsigned int              FS1R;                /*!< CAN filter scale register,           Address offset: 0x20C         */
  unsigned int                   RESERVED3;           /*!< Reserved, 0x210                                                    */
  unsigned int              FFA1R;               /*!< CAN filter FIFO assignment register, Address offset: 0x214         */
  unsigned int                   RESERVED4;           /*!< Reserved, 0x218                                                    */
  unsigned int              FA1R;                /*!< CAN filter activation register,      Address offset: 0x21C         */
  unsigned int                   RESERVED5[8];        /*!< Reserved, 0x220-0x23F                                              */
  CAN_FilterRegister_TypeDef sFilterRegister[28]; /*!< CAN Filter Register,                 Address offset: 0x240-0x31C   */
} CAN_TypeDef;

typedef struct
{
unsigned int DR;         /*!< CRC Data register,             Address offset: 0x00 */
unsigned char  IDR;        /*!< CRC Independent data register, Address offset: 0x04 */
unsigned char       RESERVED0;  /*!< Reserved, 0x05                                      */
unsigned short      RESERVED1;  /*!< Reserved, 0x06                                      */
unsigned int CR;         /*!< CRC Control register,          Address offset: 0x08 */
} CRC_TypeDef;

typedef struct
{
  unsigned int CR;       /*!< DAC control register,                                    Address offset: 0x00 */
  unsigned int SWTRIGR;  /*!< DAC software trigger register,                           Address offset: 0x04 */
  unsigned int DHR12R1;  /*!< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08 */
  unsigned int DHR12L1;  /*!< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C */
  unsigned int DHR8R1;   /*!< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10 */
  unsigned int DHR12R2;  /*!< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14 */
  unsigned int DHR12L2;  /*!< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18 */
  unsigned int DHR8R2;   /*!< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C */
  unsigned int DHR12RD;  /*!< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20 */
  unsigned int DHR12LD;  /*!< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24 */
  unsigned int DHR8RD;   /*!< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28 */
  unsigned int DOR1;     /*!< DAC channel1 data output register,                       Address offset: 0x2C */
  unsigned int DOR2;     /*!< DAC channel2 data output register,                       Address offset: 0x30 */
  unsigned int SR;       /*!< DAC status register,                                     Address offset: 0x34 */
} DAC_TypeDef;

typedef struct
{
  unsigned int IDCODE;  /*!< MCU device ID code,               Address offset: 0x00 */
  unsigned int CR;      /*!< Debug MCU configuration register, Address offset: 0x04 */
  unsigned int APB1FZ;  /*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
  unsigned int APB2FZ;  /*!< Debug MCU APB2 freeze register,   Address offset: 0x0C */
}DBGMCU_TypeDef;

typedef struct
{
  unsigned int CR;       /*!< DCMI control register 1,                       Address offset: 0x00 */
  unsigned int SR;       /*!< DCMI status register,                          Address offset: 0x04 */
  unsigned int RISR;     /*!< DCMI raw interrupt status register,            Address offset: 0x08 */
  unsigned int IER;      /*!< DCMI interrupt enable register,                Address offset: 0x0C */
  unsigned int MISR;     /*!< DCMI masked interrupt status register,         Address offset: 0x10 */
  unsigned int ICR;      /*!< DCMI interrupt clear register,                 Address offset: 0x14 */
  unsigned int ESCR;     /*!< DCMI embedded synchronization code register,   Address offset: 0x18 */
  unsigned int ESUR;     /*!< DCMI embedded synchronization unmask register, Address offset: 0x1C */
  unsigned int CWSTRTR;  /*!< DCMI crop window start,                        Address offset: 0x20 */
  unsigned int CWSIZER;  /*!< DCMI crop window size,                         Address offset: 0x24 */
  unsigned int DR;       /*!< DCMI data register,                            Address offset: 0x28 */
} DCMI_TypeDef;

typedef struct
{
  unsigned int CR;     /*!< DMA stream x configuration register      */
  unsigned int NDTR;   /*!< DMA stream x number of data register     */
  unsigned int PAR;    /*!< DMA stream x peripheral address register */
  unsigned int M0AR;   /*!< DMA stream x memory 0 address register   */
  unsigned int M1AR;   /*!< DMA stream x memory 1 address register   */
  unsigned int FCR;    /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;

typedef struct
{
  unsigned int LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
  unsigned int HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
  unsigned int LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
  unsigned int HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;

typedef struct
{
  unsigned int MACCR;
  unsigned int MACFFR;
  unsigned int MACHTHR;
  unsigned int MACHTLR;
  unsigned int MACMIIAR;
  unsigned int MACMIIDR;
  unsigned int MACFCR;
  unsigned int MACVLANTR;             /*    8 */
  unsigned int      RESERVED0[2];
  unsigned int MACRWUFFR;             /*   11 */
  unsigned int MACPMTCSR;
  unsigned int      RESERVED1[2];
  unsigned int MACSR;                 /*   15 */
  unsigned int MACIMR;
  unsigned int MACA0HR;
  unsigned int MACA0LR;
  unsigned int MACA1HR;
  unsigned int MACA1LR;
  unsigned int MACA2HR;
  unsigned int MACA2LR;
  unsigned int MACA3HR;
  unsigned int MACA3LR;               /*   24 */
  unsigned int      RESERVED2[40];
  unsigned int MMCCR;                 /*   65 */
  unsigned int MMCRIR;
  unsigned int MMCTIR;
  unsigned int MMCRIMR;
  unsigned int MMCTIMR;               /*   69 */
  unsigned int      RESERVED3[14];
  unsigned int MMCTGFSCCR;            /*   84 */
  unsigned int MMCTGFMSCCR;
  unsigned int      RESERVED4[5];
  unsigned int MMCTGFCR;
  unsigned int      RESERVED5[10];
  unsigned int MMCRFCECR;
  unsigned int MMCRFAECR;
  unsigned int      RESERVED6[10];
  unsigned int MMCRGUFCR;
  unsigned int      RESERVED7[334];
  unsigned int PTPTSCR;
  unsigned int PTPSSIR;
  unsigned int PTPTSHR;
  unsigned int PTPTSLR;
  unsigned int PTPTSHUR;
  unsigned int PTPTSLUR;
  unsigned int PTPTSAR;
  unsigned int PTPTTHR;
  unsigned int PTPTTLR;
  unsigned int RESERVED8;
  unsigned int PTPTSSR;  /* added for STM32F2xx */
  unsigned int      RESERVED9[565];
  unsigned int DMABMR;
  unsigned int DMATPDR;
  unsigned int DMARPDR;
  unsigned int DMARDLAR;
  unsigned int DMATDLAR;
  unsigned int DMASR;
  unsigned int DMAOMR;
  unsigned int DMAIER;
  unsigned int DMAMFBOCR;
  unsigned int DMARSWTR;  /* added for STM32F2xx */
  unsigned int      RESERVED10[8];
  unsigned int DMACHTDR;
  unsigned int DMACHRDR;
  unsigned int DMACHTBAR;
  unsigned int DMACHRBAR;
} ETH_TypeDef;

typedef struct
{
  unsigned int IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  unsigned int EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  unsigned int RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  unsigned int FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  unsigned int SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  unsigned int PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;

typedef struct
{
  unsigned int ACR;      /*!< FLASH access control register, Address offset: 0x00 */
  unsigned int KEYR;     /*!< FLASH key register,            Address offset: 0x04 */
  unsigned int OPTKEYR;  /*!< FLASH option key register,     Address offset: 0x08 */
  unsigned int SR;       /*!< FLASH status register,         Address offset: 0x0C */
  unsigned int CR;       /*!< FLASH control register,        Address offset: 0x10 */
  unsigned int OPTCR;    /*!< FLASH option control register, Address offset: 0x14 */
} FLASH_TypeDef;

typedef struct
{
  unsigned int BTCR[8];    /*!< NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C */
} FSMC_Bank1_TypeDef;

typedef struct
{
  unsigned int BWTR[7];    /*!< NOR/PSRAM write timing registers, Address offset: 0x104-0x11C */
} FSMC_Bank1E_TypeDef;

typedef struct
{
  unsigned int PCR2;       /*!< NAND Flash control register 2,                       Address offset: 0x60 */
  unsigned int SR2;        /*!< NAND Flash FIFO status and interrupt register 2,     Address offset: 0x64 */
  unsigned int PMEM2;      /*!< NAND Flash Common memory space timing register 2,    Address offset: 0x68 */
  unsigned int PATT2;      /*!< NAND Flash Attribute memory space timing register 2, Address offset: 0x6C */
  unsigned int      RESERVED0;  /*!< Reserved, 0x70                                                            */
  unsigned int ECCR2;      /*!< NAND Flash ECC result registers 2,                   Address offset: 0x74 */
} FSMC_Bank2_TypeDef;

typedef struct
{
  unsigned int PCR3;       /*!< NAND Flash control register 3,                       Address offset: 0x80 */
  unsigned int SR3;        /*!< NAND Flash FIFO status and interrupt register 3,     Address offset: 0x84 */
  unsigned int PMEM3;      /*!< NAND Flash Common memory space timing register 3,    Address offset: 0x88 */
  unsigned int PATT3;      /*!< NAND Flash Attribute memory space timing register 3, Address offset: 0x8C */
  unsigned int      RESERVED0;  /*!< Reserved, 0x90                                                            */
  unsigned int ECCR3;      /*!< NAND Flash ECC result registers 3,                   Address offset: 0x94 */
} FSMC_Bank3_TypeDef;

typedef struct
{
  unsigned int PCR4;       /*!< PC Card  control register 4,                       Address offset: 0xA0 */
  unsigned int SR4;        /*!< PC Card  FIFO status and interrupt register 4,     Address offset: 0xA4 */
  unsigned int PMEM4;      /*!< PC Card  Common memory space timing register 4,    Address offset: 0xA8 */
  unsigned int PATT4;      /*!< PC Card  Attribute memory space timing register 4, Address offset: 0xAC */
  unsigned int PIO4;       /*!< PC Card  I/O space timing register 4,              Address offset: 0xB0 */
} FSMC_Bank4_TypeDef;

typedef struct
{
  unsigned int MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  unsigned int OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  unsigned int OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  unsigned int PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  unsigned int IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  unsigned int ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  unsigned short BSRRL;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
  unsigned short BSRRH;    /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
  unsigned int LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  unsigned int AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x24-0x28 */
} GPIO_TypeDef;

typedef struct
{
  unsigned int MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  unsigned int PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  unsigned int EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  unsigned int      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  unsigned int CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;

typedef struct
{
  unsigned short CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
  unsigned short      RESERVED0;  /*!< Reserved, 0x02                                   */
  unsigned short CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
  unsigned short      RESERVED1;  /*!< Reserved, 0x06                                   */
  unsigned short OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
  unsigned short      RESERVED2;  /*!< Reserved, 0x0A                                   */
  unsigned short OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
  unsigned short      RESERVED3;  /*!< Reserved, 0x0E                                   */
  unsigned short DR;         /*!< I2C Data register,          Address offset: 0x10 */
  unsigned short      RESERVED4;  /*!< Reserved, 0x12                                   */
  unsigned short SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
  unsigned short      RESERVED5;  /*!< Reserved, 0x16                                   */
  unsigned short SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
  unsigned short      RESERVED6;  /*!< Reserved, 0x1A                                   */
  unsigned short CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
  unsigned short      RESERVED7;  /*!< Reserved, 0x1E                                   */
  unsigned short TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
  unsigned short      RESERVED8;  /*!< Reserved, 0x22                                   */
} I2C_TypeDef;

typedef struct
{
  unsigned int KR;   /*!< IWDG Key register,       Address offset: 0x00 */
  unsigned int PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
  unsigned int RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
  unsigned int SR;   /*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_TypeDef;

typedef struct
{
  unsigned int CR;   /*!< PWR power control register,        Address offset: 0x00 */
  unsigned int CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;

typedef struct
{
  unsigned int CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  unsigned int PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  unsigned int CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  unsigned int CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  unsigned int AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  unsigned int AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  unsigned int AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  unsigned int      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  unsigned int APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  unsigned int APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  unsigned int      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  unsigned int AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  unsigned int AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  unsigned int AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  unsigned int      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  unsigned int APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  unsigned int APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  unsigned int      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  unsigned int AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  unsigned int AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  unsigned int AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  unsigned int      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  unsigned int APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  unsigned int APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  unsigned int      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  unsigned int BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  unsigned int CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  unsigned int      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  unsigned int SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  unsigned int PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_TypeDef;

typedef struct
{
  unsigned int TR;      /*!< RTC time register,                                        Address offset: 0x00 */
  unsigned int DR;      /*!< RTC date register,                                        Address offset: 0x04 */
  unsigned int CR;      /*!< RTC control register,                                     Address offset: 0x08 */
  unsigned int ISR;     /*!< RTC initialization and status register,                   Address offset: 0x0C */
  unsigned int PRER;    /*!< RTC prescaler register,                                   Address offset: 0x10 */
  unsigned int WUTR;    /*!< RTC wakeup timer register,                                Address offset: 0x14 */
  unsigned int CALIBR;  /*!< RTC calibration register,                                 Address offset: 0x18 */
  unsigned int ALRMAR;  /*!< RTC alarm A register,                                     Address offset: 0x1C */
  unsigned int ALRMBR;  /*!< RTC alarm B register,                                     Address offset: 0x20 */
  unsigned int WPR;     /*!< RTC write protection register,                            Address offset: 0x24 */
  unsigned int RESERVED1;    /*!< Reserved, 0x28                                                                 */
  unsigned int RESERVED2;    /*!< Reserved, 0x2C                                                                 */
  unsigned int TSTR;    /*!< RTC time stamp time register,                             Address offset: 0x30 */
  unsigned int TSDR;    /*!< RTC time stamp date register,                             Address offset: 0x34 */
  unsigned int RESERVED3;    /*!< Reserved, 0x38                                                                 */
  unsigned int RESERVED4;    /*!< Reserved, 0x3C                                                                 */
  unsigned int TAFCR;   /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
  unsigned int RESERVED5;    /*!< Reserved, 0x44                                                                 */
  unsigned int RESERVED6;    /*!< Reserved, 0x48                                                                 */
  unsigned int RESERVED7;    /*!< Reserved, 0x4C                                                                 */
  unsigned int BKP0R;   /*!< RTC backup register 1,                                    Address offset: 0x50 */
  unsigned int BKP1R;   /*!< RTC backup register 1,                                    Address offset: 0x54 */
  unsigned int BKP2R;   /*!< RTC backup register 2,                                    Address offset: 0x58 */
  unsigned int BKP3R;   /*!< RTC backup register 3,                                    Address offset: 0x5C */
  unsigned int BKP4R;   /*!< RTC backup register 4,                                    Address offset: 0x60 */
  unsigned int BKP5R;   /*!< RTC backup register 5,                                    Address offset: 0x64 */
  unsigned int BKP6R;   /*!< RTC backup register 6,                                    Address offset: 0x68 */
  unsigned int BKP7R;   /*!< RTC backup register 7,                                    Address offset: 0x6C */
  unsigned int BKP8R;   /*!< RTC backup register 8,                                    Address offset: 0x70 */
  unsigned int BKP9R;   /*!< RTC backup register 9,                                    Address offset: 0x74 */
  unsigned int BKP10R;  /*!< RTC backup register 10,                                   Address offset: 0x78 */
  unsigned int BKP11R;  /*!< RTC backup register 11,                                   Address offset: 0x7C */
  unsigned int BKP12R;  /*!< RTC backup register 12,                                   Address offset: 0x80 */
  unsigned int BKP13R;  /*!< RTC backup register 13,                                   Address offset: 0x84 */
  unsigned int BKP14R;  /*!< RTC backup register 14,                                   Address offset: 0x88 */
  unsigned int BKP15R;  /*!< RTC backup register 15,                                   Address offset: 0x8C */
  unsigned int BKP16R;  /*!< RTC backup register 16,                                   Address offset: 0x90 */
  unsigned int BKP17R;  /*!< RTC backup register 17,                                   Address offset: 0x94 */
  unsigned int BKP18R;  /*!< RTC backup register 18,                                   Address offset: 0x98 */
  unsigned int BKP19R;  /*!< RTC backup register 19,                                   Address offset: 0x9C */
} RTC_TypeDef;

typedef struct
{
  unsigned int POWER;          /*!< SDIO power control register,    Address offset: 0x00 */
  unsigned int CLKCR;          /*!< SDI clock control register,     Address offset: 0x04 */
  unsigned int ARG;            /*!< SDIO argument register,         Address offset: 0x08 */
  unsigned int CMD;            /*!< SDIO command register,          Address offset: 0x0C */
  unsigned int  RESPCMD;        /*!< SDIO command response register, Address offset: 0x10 */
  unsigned int  RESP1;          /*!< SDIO response 1 register,       Address offset: 0x14 */
  unsigned int  RESP2;          /*!< SDIO response 2 register,       Address offset: 0x18 */
  unsigned int  RESP3;          /*!< SDIO response 3 register,       Address offset: 0x1C */
  unsigned int  RESP4;          /*!< SDIO response 4 register,       Address offset: 0x20 */
  unsigned int DTIMER;         /*!< SDIO data timer register,       Address offset: 0x24 */
  unsigned int DLEN;           /*!< SDIO data length register,      Address offset: 0x28 */
  unsigned int DCTRL;          /*!< SDIO data control register,     Address offset: 0x2C */
  unsigned int  DCOUNT;         /*!< SDIO data counter register,     Address offset: 0x30 */
  unsigned int  STA;            /*!< SDIO status register,           Address offset: 0x34 */
  unsigned int ICR;            /*!< SDIO interrupt clear register,  Address offset: 0x38 */
  unsigned int MASK;           /*!< SDIO mask register,             Address offset: 0x3C */
  unsigned int      RESERVED0[2];   /*!< Reserved, 0x40-0x44                                  */
  unsigned int  FIFOCNT;        /*!< SDIO FIFO counter register,     Address offset: 0x48 */
  unsigned int      RESERVED1[13];  /*!< Reserved, 0x4C-0x7C                                  */
  unsigned int FIFO;           /*!< SDIO data FIFO register,        Address offset: 0x80 */
} SDIO_TypeDef;

typedef struct
{
  unsigned short CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  unsigned short      RESERVED0;  /*!< Reserved, 0x02                                                           */
  unsigned short CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  unsigned short      RESERVED1;  /*!< Reserved, 0x06                                                           */
  unsigned short SR;         /*!< SPI status register,                                Address offset: 0x08 */
  unsigned short      RESERVED2;  /*!< Reserved, 0x0A                                                           */
  unsigned short DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  unsigned short      RESERVED3;  /*!< Reserved, 0x0E                                                           */
  unsigned short CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  unsigned short      RESERVED4;  /*!< Reserved, 0x12                                                           */
  unsigned short RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  unsigned short      RESERVED5;  /*!< Reserved, 0x16                                                           */
  unsigned short TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  unsigned short      RESERVED6;  /*!< Reserved, 0x1A                                                           */
  unsigned short I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  unsigned short      RESERVED7;  /*!< Reserved, 0x1E                                                           */
  unsigned short I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
  unsigned short      RESERVED8;  /*!< Reserved, 0x22                                                           */
} SPI_TypeDef;

typedef struct
{
  unsigned short CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  unsigned short      RESERVED0;   /*!< Reserved, 0x02                                            */
  unsigned short CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  unsigned short      RESERVED1;   /*!< Reserved, 0x06                                            */
  unsigned short SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  unsigned short      RESERVED2;   /*!< Reserved, 0x0A                                            */
  unsigned short DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  unsigned short      RESERVED3;   /*!< Reserved, 0x0E                                            */
  unsigned short SR;          /*!< TIM status register,                 Address offset: 0x10 */
  unsigned short      RESERVED4;   /*!< Reserved, 0x12                                            */
  unsigned short EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  unsigned short      RESERVED5;   /*!< Reserved, 0x16                                            */
  unsigned short CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  unsigned short      RESERVED6;   /*!< Reserved, 0x1A                                            */
  unsigned short CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  unsigned short      RESERVED7;   /*!< Reserved, 0x1E                                            */
  unsigned short CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  unsigned short      RESERVED8;   /*!< Reserved, 0x22                                            */
  unsigned int CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  unsigned short PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  unsigned short      RESERVED9;   /*!< Reserved, 0x2A                                            */
  unsigned int ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  unsigned short RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  unsigned short      RESERVED10;  /*!< Reserved, 0x32                                            */
  unsigned int CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  unsigned int CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  unsigned int CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  unsigned int CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  unsigned short BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  unsigned short      RESERVED11;  /*!< Reserved, 0x46                                            */
  unsigned short DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  unsigned short      RESERVED12;  /*!< Reserved, 0x4A                                            */
  unsigned short DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  unsigned short      RESERVED13;  /*!< Reserved, 0x4E                                            */
  unsigned short OR;          /*!< TIM option register,                 Address offset: 0x50 */
  unsigned short      RESERVED14;  /*!< Reserved, 0x52                                            */
} TIM_TypeDef;

typedef struct
{
  unsigned short SR;         /*!< USART Status register,                   Address offset: 0x00 */
  unsigned short      RESERVED0;  /*!< Reserved, 0x02                                                */
  unsigned short DR;         /*!< USART Data register,                     Address offset: 0x04 */
  unsigned short      RESERVED1;  /*!< Reserved, 0x06                                                */
  unsigned short BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
  unsigned short      RESERVED2;  /*!< Reserved, 0x0A                                                */
  unsigned short CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
  unsigned short      RESERVED3;  /*!< Reserved, 0x0E                                                */
  unsigned short CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
  unsigned short      RESERVED4;  /*!< Reserved, 0x12                                                */
  unsigned short CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
  unsigned short      RESERVED5;  /*!< Reserved, 0x16                                                */
  unsigned short GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
  unsigned short      RESERVED6;  /*!< Reserved, 0x1A                                                */
} USART_TypeDef;

typedef struct
{
  unsigned int CR;   /*!< WWDG Control register,       Address offset: 0x00 */
  unsigned int CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
  unsigned int SR;   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;

typedef struct
{
  unsigned int CR;     /*!< CRYP control register,                            Address offset: 0x00 */
  unsigned int SR;     /*!< CRYP status register,                             Address offset: 0x04 */
  unsigned int DR;     /*!< CRYP data input register,                         Address offset: 0x08 */
  unsigned int DOUT;   /*!< CRYP data output register,                        Address offset: 0x0C */
  unsigned int DMACR;  /*!< CRYP DMA control register,                        Address offset: 0x10 */
  unsigned int IMSCR;  /*!< CRYP interrupt mask set/clear register,           Address offset: 0x14 */
  unsigned int RISR;   /*!< CRYP raw interrupt status register,               Address offset: 0x18 */
  unsigned int MISR;   /*!< CRYP masked interrupt status register,            Address offset: 0x1C */
  unsigned int K0LR;   /*!< CRYP key left  register 0,                        Address offset: 0x20 */
  unsigned int K0RR;   /*!< CRYP key right register 0,                        Address offset: 0x24 */
  unsigned int K1LR;   /*!< CRYP key left  register 1,                        Address offset: 0x28 */
  unsigned int K1RR;   /*!< CRYP key right register 1,                        Address offset: 0x2C */
  unsigned int K2LR;   /*!< CRYP key left  register 2,                        Address offset: 0x30 */
  unsigned int K2RR;   /*!< CRYP key right register 2,                        Address offset: 0x34 */
  unsigned int K3LR;   /*!< CRYP key left  register 3,                        Address offset: 0x38 */
  unsigned int K3RR;   /*!< CRYP key right register 3,                        Address offset: 0x3C */
  unsigned int IV0LR;  /*!< CRYP initialization vector left-word  register 0, Address offset: 0x40 */
  unsigned int IV0RR;  /*!< CRYP initialization vector right-word register 0, Address offset: 0x44 */
  unsigned int IV1LR;  /*!< CRYP initialization vector left-word  register 1, Address offset: 0x48 */
  unsigned int IV1RR;  /*!< CRYP initialization vector right-word register 1, Address offset: 0x4C */
} CRYP_TypeDef;

typedef struct
{
  unsigned int CR;        /*!< HASH control register,          Address offset: 0x00        */
  unsigned int DIN;       /*!< HASH data input register,       Address offset: 0x04        */
  unsigned int STR;       /*!< HASH start register,            Address offset: 0x08        */
  unsigned int HR[5];     /*!< HASH digest registers,          Address offset: 0x0C-0x1C   */
  unsigned int IMR;       /*!< HASH interrupt enable register, Address offset: 0x20        */
  unsigned int SR;        /*!< HASH status register,           Address offset: 0x24        */
  unsigned int  RESERVED[52];  /*!< Reserved, 0x28-0xF4                                         */
  unsigned int CSR[51];   /*!< HASH context swap registers,    Address offset: 0x0F8-0x1C0 */
} HASH_TypeDef;

typedef struct
{
  unsigned int CR;  /*!< RNG control register, Address offset: 0x00 */
  unsigned int SR;  /*!< RNG status register,  Address offset: 0x04 */
  unsigned int DR;  /*!< RNG data register,    Address offset: 0x08 */
} RNG_TypeDef;

InterruptType_Type* SCS = (InterruptType_Type*) 0xe000e000; //12
ITM_Type* ITM = (ITM_Type*) 0xe0000000; //4096
CoreDebug_Type* CoreDebug = (CoreDebug_Type*) 0xe000edf0; //16
SysTick_Type* SysTick = (SysTick_Type*) 0xe000e010; //16
NVIC_Type* NVIC = (NVIC_Type*) 0xe000e100; //3588
SCB_Type* SCB = (SCB_Type*) 0xe000ed00; //116
MPU_Type* MPU = (MPU_Type*) 0xe000ed90; //44
TIM_TypeDef* TIM2 = (TIM_TypeDef*) 0x40000000; //84
TIM_TypeDef* TIM3 = (TIM_TypeDef*) 0x40000400; //84
TIM_TypeDef* TIM4 = (TIM_TypeDef*) 0x40000800; //84
TIM_TypeDef* TIM5 = (TIM_TypeDef*) 0x40000c00; //84
TIM_TypeDef* TIM6 = (TIM_TypeDef*) 0x40001000; //84
TIM_TypeDef* TIM7 = (TIM_TypeDef*) 0x40001400; //84
TIM_TypeDef* TIM12 = (TIM_TypeDef*) 0x40001800; //84
TIM_TypeDef* TIM13 = (TIM_TypeDef*) 0x40001c00; //84
TIM_TypeDef* TIM14 = (TIM_TypeDef*) 0x40002000; //84
RTC_TypeDef* RTC = (RTC_TypeDef*) 0x40002800; //160
WWDG_TypeDef* WWDG = (WWDG_TypeDef*) 0x40002c00; //12
IWDG_TypeDef* IWDG = (IWDG_TypeDef*) 0x40003000; //16
SPI_TypeDef* SPI2 = (SPI_TypeDef*) 0x40003800; //36
SPI_TypeDef* SPI3 = (SPI_TypeDef*) 0x40003c00; //36
USART_TypeDef* USART2 = (USART_TypeDef*) 0x40004400; //28
USART_TypeDef* USART3 = (USART_TypeDef*) 0x40004800; //28
USART_TypeDef* UART4 = (USART_TypeDef*) 0x40004c00; //28
USART_TypeDef* UART5 = (USART_TypeDef*) 0x40005000; //28
I2C_TypeDef* I2C1 = (I2C_TypeDef*) 0x40005400; //36
I2C_TypeDef* I2C2 = (I2C_TypeDef*) 0x40005800; //36
I2C_TypeDef* I2C3 = (I2C_TypeDef*) 0x40005c00; //36
CAN_TypeDef* CAN1 = (CAN_TypeDef*) 0x40006400; //800
CAN_TypeDef* CAN2 = (CAN_TypeDef*) 0x40006800; //800
PWR_TypeDef* PWR = (PWR_TypeDef*) 0x40007000; //8
DAC_TypeDef* DAC = (DAC_TypeDef*) 0x40007400; //56
TIM_TypeDef* TIM1 = (TIM_TypeDef*) 0x40010000; //84
TIM_TypeDef* TIM8 = (TIM_TypeDef*) 0x40010400; //84
USART_TypeDef* USART1 = (USART_TypeDef*) 0x40011000; //28
USART_TypeDef* USART6 = (USART_TypeDef*) 0x40011400; //28
ADC_TypeDef* ADC1 = (ADC_TypeDef*) 0x40012000; //80
ADC_TypeDef* ADC2 = (ADC_TypeDef*) 0x40012100; //80
ADC_TypeDef* ADC3 = (ADC_TypeDef*) 0x40012200; //80
ADC_Common_TypeDef* ADC = (ADC_Common_TypeDef*) 0x40012300; //12
SDIO_TypeDef* SDIO = (SDIO_TypeDef*) 0x40012c00; //132
SPI_TypeDef* SPI1 = (SPI_TypeDef*) 0x40013000; //36
SYSCFG_TypeDef* SYSCFG = (SYSCFG_TypeDef*) 0x40013800; //36
EXTI_TypeDef* EXTI = (EXTI_TypeDef*) 0x40013c00; //24
TIM_TypeDef* TIM9 = (TIM_TypeDef*) 0x40014000; //84
TIM_TypeDef* TIM10 = (TIM_TypeDef*) 0x40014400; //84
TIM_TypeDef* TIM11 = (TIM_TypeDef*) 0x40014800; //84
GPIO_TypeDef* GPIOA = (GPIO_TypeDef*) 0x40020000; //40
GPIO_TypeDef* GPIOB = (GPIO_TypeDef*) 0x40020400; //40
GPIO_TypeDef* GPIOC = (GPIO_TypeDef*) 0x40020800; //40
GPIO_TypeDef* GPIOD = (GPIO_TypeDef*) 0x40020c00; //40
GPIO_TypeDef* GPIOE = (GPIO_TypeDef*) 0x40021000; //40
GPIO_TypeDef* GPIOF = (GPIO_TypeDef*) 0x40021400; //40
GPIO_TypeDef* GPIOG = (GPIO_TypeDef*) 0x40021800; //40
GPIO_TypeDef* GPIOH = (GPIO_TypeDef*) 0x40021c00; //40
GPIO_TypeDef* GPIOI = (GPIO_TypeDef*) 0x40022000; //40
CRC_TypeDef* CRC = (CRC_TypeDef*) 0x40023000; //12
RCC_TypeDef* RCC = (RCC_TypeDef*) 0x40023800; //136
FLASH_TypeDef* FLASH_R = (FLASH_TypeDef*) 0x40023c00; //24
DMA_TypeDef* DMA1 = (DMA_TypeDef*) 0x40026000; //16
DMA_Stream_TypeDef* DMA1_Stream0 = (DMA_Stream_TypeDef*) 0x40026010; //24
DMA_Stream_TypeDef* DMA1_Stream1 = (DMA_Stream_TypeDef*) 0x40026028; //24
DMA_Stream_TypeDef* DMA1_Stream2 = (DMA_Stream_TypeDef*) 0x40026040; //24
DMA_Stream_TypeDef* DMA1_Stream3 = (DMA_Stream_TypeDef*) 0x40026058; //24
DMA_Stream_TypeDef* DMA1_Stream4 = (DMA_Stream_TypeDef*) 0x40026070; //24
DMA_Stream_TypeDef* DMA1_Stream5 = (DMA_Stream_TypeDef*) 0x40026088; //24
DMA_Stream_TypeDef* DMA1_Stream6 = (DMA_Stream_TypeDef*) 0x400260a0; //24
DMA_Stream_TypeDef* DMA1_Stream7 = (DMA_Stream_TypeDef*) 0x400260b8; //24
DMA_TypeDef* DMA2 = (DMA_TypeDef*) 0x40026400; //16
DMA_Stream_TypeDef* DMA2_Stream0 = (DMA_Stream_TypeDef*) 0x40026410; //24
DMA_Stream_TypeDef* DMA2_Stream1 = (DMA_Stream_TypeDef*) 0x40026428; //24
DMA_Stream_TypeDef* DMA2_Stream2 = (DMA_Stream_TypeDef*) 0x40026440; //24
DMA_Stream_TypeDef* DMA2_Stream3 = (DMA_Stream_TypeDef*) 0x40026458; //24
DMA_Stream_TypeDef* DMA2_Stream4 = (DMA_Stream_TypeDef*) 0x40026470; //24
DMA_Stream_TypeDef* DMA2_Stream5 = (DMA_Stream_TypeDef*) 0x40026488; //24
DMA_Stream_TypeDef* DMA2_Stream6 = (DMA_Stream_TypeDef*) 0x400264a0; //24
DMA_Stream_TypeDef* DMA2_Stream7 = (DMA_Stream_TypeDef*) 0x400264b8; //24
ETH_TypeDef* ETH = (ETH_TypeDef*) 0x40028000; //4184
DCMI_TypeDef* DCMI = (DCMI_TypeDef*) 0x50050000; //44
CRYP_TypeDef* CRYP = (CRYP_TypeDef*) 0x50060000; //80
HASH_TypeDef* HASH = (HASH_TypeDef*) 0x50060400; //452
RNG_TypeDef* RNG = (RNG_TypeDef*) 0x50060800; //12
FSMC_Bank1_TypeDef* FSMC_Bank1_R = (FSMC_Bank1_TypeDef*) 0xa0000000; //32
FSMC_Bank1E_TypeDef* FSMC_Bank1E_R = (FSMC_Bank1E_TypeDef*) 0xa0000104; //28
FSMC_Bank2_TypeDef* FSMC_Bank2_R = (FSMC_Bank2_TypeDef*) 0xa0000060; //24
FSMC_Bank3_TypeDef* FSMC_Bank3_R = (FSMC_Bank3_TypeDef*) 0xa0000080; //24
FSMC_Bank4_TypeDef* FSMC_Bank4_R = (FSMC_Bank4_TypeDef*) 0xa00000a0; //20
DBGMCU_TypeDef* DBGMCU = (DBGMCU_TypeDef*) 0xe0042000; //16
