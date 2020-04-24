//! Implementation of the STM32L4 Reset and Clock Control peripheral.
//!
//! Handles core clock control and peripheral clock gating and reset function

use kernel::common::registers::{register_bitfields, ReadOnly, ReadWrite};
use kernel::common::StaticRef;
use kernel::ClockInterface;

use crate::memory_map;

/// Reset and clock control
#[repr(C)]
struct RccRegisters {
    /// clock control register
    cr: ReadWrite<u32, CR::Register>,
    /// internal clock sources calibration register
    icscr: ReadWrite<u32, ICSCR::Register>,
    /// clock configuration register
    cfgr: ReadWrite<u32, CFGR::Register>,
    /// PLL configuration register
    pllcfgr: ReadWrite<u32, PLLCFGR::Register>,
    /// PLL SAI1 configuration register
    pllsai1cfgr: ReadWrite<u32, PLLSAI1CFGR::Register>,
    /// PLL SAI2 configuration register
    pllsai2cfgr: ReadWrite<u32, PLLSAI2CFGR::Register>,
    /// clock interrupt enable register
    cier: ReadWrite<u32, CIER::Register>,
    /// clock interrupt flag register
    cifr: ReadWrite<u32, CIFR::Register>,
    /// clock interrupt clear register
    cicr: ReadWrite<u32, CICR::Register>,
    /// Reserved
    _reserved0: ReadOnly<u32>,
    /// AHB1 peripheral reset register
    ahb1rstr: ReadWrite<u32, AHB1RSTR::Register>,
    /// AHB2 peripheral reset register
    ahb2rstr: ReadWrite<u32, AHB2RSTR::Register>,
    /// AHB3 peripheral reset register
    ahb3rstr: ReadWrite<u32, AHB3RSTR::Register>,
    /// Reserved
    _reserved1: ReadOnly<u32>,
    /// APB1 peripheral reset register 1
    apb1rstr1: ReadWrite<u32, APB1RSTR1::Register>,
    /// APB1 peripheral reset register 2
    apb1rstr2: ReadWrite<u32, APB1RSTR2::Register>,
    /// APB2 peripheral reset register
    apb2rstr: ReadWrite<u32, APB2RSTR::Register>,
    /// Reserved
    _reserved2: ReadOnly<u32>,
    /// AHB1 peripheral clock register
    ahb1enr: ReadWrite<u32, AHB1ENR::Register>,
    /// AHB2 peripheral clock enable register
    ahb2enr: ReadWrite<u32, AHB2ENR::Register>,
    /// AHB3 peripheral clock enable register
    ahb3enr: ReadWrite<u32, AHB3ENR::Register>,
    /// Reserved
    _reserved3: ReadOnly<u32>,
    /// APB1 peripheral clock enable register 1
    apb1enr1: ReadWrite<u32, APB1ENR1::Register>,
    /// APB1 peripheral clock enable register 2
    apb1enr2: ReadWrite<u32, APB1ENR2::Register>,
    /// APB2 peripheral clock enable register
    apb2enr: ReadWrite<u32, APB2ENR::Register>,
    /// Reserved
    _reserved4: ReadOnly<u32>,
    /// AHB1 peripheral clocks enable in sleep and stop modes register
    ahb1smenr: ReadWrite<u32, AHB1SMENR::Register>,
    /// AHB2 peripheral clock enable in sleep and stop modes register
    ahb2smenr: ReadWrite<u32, AHB2SMENR::Register>,
    /// AHB3 peripheral clock enable in sleep and stop modes register
    ahb3smenr: ReadWrite<u32, AHB3SMENR::Register>,
    /// Reserved
    _reserved5: ReadOnly<u32>,
    /// APB1 peripheral clock enable in sleep and stop modes register 1
    apb1smenr1: ReadWrite<u32, APB1SMENR1::Register>,
    /// APB1 peripheral clock enable in sleep and stop modes register 2
    apb1smenr2: ReadWrite<u32, APB1SMENR2::Register>,
    /// APB2 peripheral clock enabled in sleep and stop modes register
    apb2smenr: ReadWrite<u32, APB2SMENR::Register>,
    /// Reserved
    _reserved6: ReadOnly<u32>,
    /// peripherals independent clock configuration register
    ccipr: ReadWrite<u32, CCIPR::Register>,
    /// Reserved
    _reserved7: ReadOnly<u32>,
    /// Backup domain control register
    bdcr: ReadWrite<u32, BDCR::Register>,
    /// clock control & status register
    csr: ReadWrite<u32, CSR::Register>,
    /// Clock recovery RC register
    crrcr: ReadWrite<u32, CRRCR::Register>,
    /// peripherals independent clock configuration register
    ccipr2: ReadWrite<u32, CCIPR2::Register>,
}

register_bitfields![u32,
    CR [
        /// SAI2 clock ready flag
        PLLSAI2RDY OFFSET(29) NUMBITS(1) [],
        /// SAI2 PLL enable
        PLLSAI2ON OFFSET(28) NUMBITS(1) [],
        /// SAI1 clock ready flag
        PLLSAI1RDY OFFSET(27) NUMBITS(1) [],
        /// SAI1 PLL enable
        PLLSAI1ON OFFSET(26) NUMBITS(1) [],
        /// Main PLL (PLL) clock ready flag
        PLLRDY OFFSET(25) NUMBITS(1) [],
        /// Main PLL (PLL) enable
        PLLON OFFSET(24) NUMBITS(1) [],
        /// Clock security system enable
        CSSON OFFSET(19) NUMBITS(1) [],
        /// HSE clock bypass
        HSEBYP OFFSET(18) NUMBITS(1) [],
        /// HSE clock ready flag
        HSERDY OFFSET(17) NUMBITS(1) [],
        /// HSE clock enable
        HSEON OFFSET(16) NUMBITS(1) [],
        /// HSI16 Automatic Start from Stop
        HSIAFS OFFSET(11) NUMBITS(1) [],
        /// Internal High Speed oscillator (HSI16) clock ready flag
        HSIRDY OFFSET(10) NUMBITS(1) [],
        /// Internal High Speed oscillator (HSI16) clock enable for some IPs Kernel
        HSIKERON OFFSET(9) NUMBITS(1) [],
        /// nternal High Speed oscillator (HSI16) clock enable
        HSION OFFSET(8) NUMBITS(1) [],
        /// Internal Multi Speed oscillator (MSI) clock Range
        MSIRANGE OFFSET(4) NUMBITS(4) [
            MSI_100kHz = 0,
            MSI_200kHz = 1,
            MSI_400kHz = 2,
            MSI_800kHz = 3,
            MSI_1MHz = 4,
            MSI_2MHz = 5,
            MSI_4MHz = 6,
            MSI_8MHz = 7,
            MSI_16MHz = 8,
            MSI_24MHz = 9,
            MSI_32MHz = 10,
            MSI_48MHz = 11
        ],
        /// Internal Multi Speed oscillator (MSI) range selection
        MSIRGSEL OFFSET(3) NUMBITS(1) [
            RANGE_FROM_RCC_CSR = 0,
            RANGE_FROM_RCC_CR = 1
        ],
        /// Internal Multi Speed oscillator (MSI) PLL enable
        MSIPLLEN OFFSET(2) NUMBITS(1) [],
        /// Internal Multi Speed oscillator (MSI) clock ready flag
        MSIRDY OFFSET(1) NUMBITS(1) [],
        /// Internal Multi Speed oscillator (MSI) clock enable
        MSION OFFSET(0) NUMBITS(1) []
    ],
    ICSCR [
        /// HSITRIM configuration
        HSITRIM OFFSET(24) NUMBITS(7) [],
        /// HSICAL configuration
        HSICAL OFFSET(16) NUMBITS(8) [],
        /// MSITRIM configuration
        MSITRIM OFFSET(8) NUMBITS(8) [],
        /// MSICAL configuration
        MSICAL OFFSET(0) NUMBITS(8) []
    ],
    CFGR [
        /// MCO prescaler
        MCOPRE OFFSET(28) NUMBITS(3) [],
        /// Wake Up from stop and CSS backup clock selection
        MCOSEL OFFSET(24) NUMBITS(4) [],
        /// Clock output selection
        STOPWUCK OFFSET(15) NUMBITS(1) [],
        /// APB2 prescaler
        PPRE2 OFFSET(11) NUMBITS(3) [],
        /// APB1 prescaler
        PPRE1 OFFSET(8) NUMBITS(3) [],
        /// AHB prescaler
        HPRE OFFSET(4) NUMBITS(4) [],
        /// System clock Switch Status
        SWS OFFSET(2) NUMBITS(2) [
            MSI_SYSCLK = 0,
            HSI16_SYSCLK = 1,
            HSE_SYSCLK = 2,
            PLL_SYSCLK = 4
        ],
        /// System clock Switch
        SW OFFSET(0) NUMBITS(2) [
            MSI_SYSCLK = 0,
            HSI6_SYSCLK = 1,
            HSE_SYSCLK = 2,
            PLL_SYSCLK = 3
        ]
    ],
    PLLCFGR [
        /// Main PLL division factor for PLLSAI2CLK (only for STM32L49x/L4Ax devices)
        PLLPDIV OFFSET(27) NUMBITS(4) [],
        /// Main PLL division factor for PLLCLK (system clock)
        PLLR OFFSET(25) NUMBITS(2) [],
        /// Main PLL PLLCLK output enable
        PLLREN OFFSET(24) NUMBITS(1) [],
        /// Main PLL division factor for PLL48M1CLK (48 MHz clock).
        PLLQ OFFSET(21) NUMBITS(2) [],
        /// Main PLL PLL48M1CLK output enable
        PLLQEN OFFSET(20) NUMBITS(1) [],
        /// Main PLL division factor for PLLSAI3CLK (SAI1 and SAI2 clock).
        PLLP OFFSET(17) NUMBITS(1) [],
        /// Main PLL PLLSAI3CLK output enable
        PLLPEN OFFSET(16) NUMBITS(1) [],
        /// Main PLL (PLL) multiplication factor for VCO
        PLLN OFFSET(8) NUMBITS(7) [],
        /// Division factor for the main PLL and audio PLL (PLLSAI1 and PLLSAI2) input clock
        PLLM OFFSET(4) NUMBITS(3) [],
        /// Main PLL, PLLSAI1 and PLLSAI2 entry clock source
        PLLSRC OFFSET(0) NUMBITS(2) []
    ],
    PLLSAI1CFGR [
        /// PLLSAI1 division factor for PLLSAI1CLK (only on STM32L49x/L4Ax devices)
        PLLSAI1PDIV OFFSET(27) NUMBITS(5) [],
        /// PLLSAI1 division factor for PLLADC1CLK (ADC clock)
        PLLSAI1R OFFSET(25) NUMBITS(2) [],
        /// PLLSAI1 PLLADC1CLK output enable
        PLLSAI1REN OFFSET(24) NUMBITS(1) [],
        /// PLLSAI1 division factor for PLL48M2CLK (48 MHz clock)
        PLLSAI1Q OFFSET(21) NUMBITS(2) [],
        /// PLLSAI1 PLL48M2CLK output enable
        PLLSAI1QEN OFFSET(20) NUMBITS(1) [],
        /// PLLSAI1 division factor for PLLSAI1CLK (SAI1 or SAI2 clock).
        PLLSAI1P OFFSET(17) NUMBITS(1) [],
        /// PLLSAI1 PLLSAI1CLK output enable
        PLLSAI1PEN OFFSET(16) NUMBITS(1) [],
        /// PLLSAI1 multiplication factor for VCO
        PLLSAI1N OFFSET(8) NUMBITS(7) []
    ],
    PLLSAI2CFGR [
        /// PLLSAI2 division factor for PLLSAI2CLK (only on STM32L49x/L4Ax devices)
        PLLSAI2PDIV OFFSET(27) NUMBITS(5) [],
        /// PLLSAI2 division factor for PLLADC2CLK (ADC clock)
        PLLSAI2R OFFSET(25) NUMBITS(2) [],
        /// PLLSAI2 PLLADC2CLK output enable
        PLLSAI2REN OFFSET(24) NUMBITS(1) [],
        /// PLLSAI2 division factor for PLLSAI2CLK (SAI1 or SAI2 clock).
        PLLSAI2P OFFSET(17) NUMBITS(1) [],
        /// PLLSAI2 PLLSAI2CLK output enable
        PLLSAI2PEN OFFSET(16) NUMBITS(1) [],
        /// PLLSAI2 multiplication factor for VCO
        PLLSAI2N OFFSET(8) NUMBITS(7) []
    ],
    CIER [
        /// HSI48 ready interrupt enable (only on STM32L49x/L4Ax devices)
        HSI48RDYIE OFFSET(10) NUMBITS(1) [],
        /// LSE clock security system interrupt enable
        LSECSSIE OFFSET(9) NUMBITS(1) [],
        /// PLLSAI2 ready interrupt enable
        PLLSAI2RDYIE OFFSET(7) NUMBITS(1) [],
        /// PLLSAI1 ready interrupt enable
        PLLSAI1RDYIE OFFSET(6) NUMBITS(1) [],
        /// PLL ready interrupt enable
        PLLRDYIE OFFSET(5) NUMBITS(1) [],
        /// HSE ready interrupt enable
        HSERDYIE OFFSET(4) NUMBITS(1) [],
        /// HSI ready interrupt enable
        HSIRDYIE OFFSET(3) NUMBITS(1) [],
        /// MSI ready interrupt enable
        MSIRDYIE OFFSET(2) NUMBITS(1) [],
        /// LSE ready interrupt enable
        LSERDYIE OFFSET(1) NUMBITS(1) [],
        /// LSI ready interrupt enable
        LSIRDYIE OFFSET(0) NUMBITS(1) []
    ],
    CIFR [
        /// HSI48 ready interrupt flag (only on STM32L49x/L4Ax devices)
        HSI48RDYF OFFSET(10) NUMBITS(1) [],
        /// LSE clock security system interrupt flag
        LSECSSF OFFSET(9) NUMBITS(1) [],
        /// Clock security system interrupt flag
        CSSF OFFSET(8) NUMBITS(1) [],
        /// PLLSAI2 ready interrupt flag
        PLLSAI2RDYF OFFSET(7) NUMBITS(1) [],
        /// PLLSAI1 ready interrupt flag
        PLLSAI1RDYF OFFSET(6) NUMBITS(1) [],
        /// PLL ready interrupt flag
        PLLRDYF OFFSET(5) NUMBITS(1) [],
        /// HSE ready interrupt flag
        HSERDYF OFFSET(4) NUMBITS(1) [],
        /// HSI ready interrupt flag
        HSIRDYF OFFSET(3) NUMBITS(1) [],
        /// MSI ready interrupt flag
        MSIRDYF OFFSET(2) NUMBITS(1) [],
        /// LSE ready interrupt flag
        LSERDYF OFFSET(1) NUMBITS(1) [],
        /// LSI ready interrupt flag
        LSIRDYF OFFSET(0) NUMBITS(1) []
    ],
    CICR [
        /// HSI48 ready interrupt clear (only on STM32L49x/L4Ax devices)
        HSI48RDYC OFFSET(10) NUMBITS(1) [],
        /// LSE clock security system interrupt clear
        LSECSSC OFFSET(9) NUMBITS(1) [],
        /// Clock security system interrupt clear
        CSSC OFFSET(8) NUMBITS(1) [],
        /// PLLSAI2 ready interrupt clear
        PLLSAI2RDYC OFFSET(7) NUMBITS(1) [],
        /// PLLSAI1 ready interrupt clear
        PLLSAI1RDYC OFFSET(6) NUMBITS(1) [],
        /// PLL ready interrupt clear
        PLLRDYC OFFSET(5) NUMBITS(1) [],
        /// HSE ready interrupt clear
        HSERDYC OFFSET(4) NUMBITS(1) [],
        /// HSI ready interrupt clear
        HSIRDYC OFFSET(3) NUMBITS(1) [],
        /// MSI ready interrupt clear
        MSIRDYC OFFSET(2) NUMBITS(1) [],
        /// LSE ready interrupt clear
        LSERDYC OFFSET(1) NUMBITS(1) [],
        /// LSI ready interrupt clear
        LSIRDYC OFFSET(0) NUMBITS(1) []
    ],
    AHB1RSTR [
        /// DMA2DRST reset
        DMA2DRST OFFSET(17) NUMBITS(1) [],
        /// TSCRST reset
        TSCRST OFFSET(16) NUMBITS(1) [],
        /// CRCRST reset
        CRCRST OFFSET(12) NUMBITS(1) [],
        /// FLASHRST reset
        FLASHRST OFFSET(8) NUMBITS(1) [],
        /// DMA2RST reset
        DMA2RST OFFSET(1) NUMBITS(1) [],
        /// DMA1RST reset
        DMA1RST OFFSET(0) NUMBITS(1) []
    ],
    AHB2RSTR [
        /// Random number generator reset
        RNGRST OFFSET(18) NUMBITS(1) [],
        /// Hash reset (this bit is reserved for STM32L47x/L48x devices)
        HASHRST OFFSET(17) NUMBITS(1) [],
        /// AES hardware accelerator reset (this bit is reserved on STM32L47x/L49x devices).
        AESRST OFFSET(16) NUMBITS(1) [],
        /// Camera interface reset
        DCMIRST OFFSET(14) NUMBITS(1) [],
        /// ADC reset
        ADCRST OFFSET(13) NUMBITS(1) [],
        /// USB OTG FS module reset
        OTGFSRST OFFSET(12) NUMBITS(1) [],
        /// IO port I reset
        GPIOIRST OFFSET(8) NUMBITS(1) [],
        /// IO port H reset
        GPIOHRST OFFSET(7) NUMBITS(1) [],
        /// IO port G reset
        GPIOGRST OFFSET(6) NUMBITS(1) [],
        /// IO port F reset
        GPIOFRST OFFSET(5) NUMBITS(1) [],
        /// IO port E reset
        GPIOERST OFFSET(4) NUMBITS(1) [],
        /// IO port D reset
        GPIODRST OFFSET(3) NUMBITS(1) [],
        /// IO port C reset
        GPIOCRST OFFSET(2) NUMBITS(1) [],
        /// IO port B reset
        GPIOBRST OFFSET(1) NUMBITS(1) [],
        /// IO port A reset
        GPIOARST OFFSET(0) NUMBITS(1) []
    ],
    AHB3RSTR [
        /// QUADSPI module reset
        QSPIRST OFFSET(8) NUMBITS(1) [],
        /// Flexible memory controller module reset
        FMCRST OFFSET(0) NUMBITS(1) []
    ],
    APB1RSTR1 [
        /// LPTIM1 reset
        LPTIM1RST OFFSET(31) NUMBITS(1) [],
        /// OPAMP reset
        OPAMPRST OFFSET(30) NUMBITS(1) [],
        /// DAC reset
        DAC1RST OFFSET(29) NUMBITS(1) [],
        /// Power interface reset
        PWRRST OFFSET(28) NUMBITS(1) [],
        /// CAN2 reset
        CAN2RST OFFSET(26) NUMBITS(1) [],
        /// CAN1 reset
        CAN1RST OFFSET(25) NUMBITS(1) [],
        /// CRS reset (this bit is reserved for STM32L47x/L48x devices)
        CRSRST OFFSET(24) NUMBITS(1) [],
        /// I2C3 reset
        I2C3RST OFFSET(23) NUMBITS(1) [],
        /// I2C 2 reset
        I2C2RST OFFSET(22) NUMBITS(1) [],
        /// I2C 1 reset
        I2C1RST OFFSET(21) NUMBITS(1) [],
        /// USART 5 reset
        UART5RST OFFSET(20) NUMBITS(1) [],
        /// USART 4 reset
        UART4RST OFFSET(19) NUMBITS(1) [],
        /// USART 3 reset
        USART3RST OFFSET(18) NUMBITS(1) [],
        /// USART 2 reset
        USART2RST OFFSET(17) NUMBITS(1) [],
        /// SPI 3 reset
        SPI3RST OFFSET(15) NUMBITS(1) [],
        /// SPI 2 reset
        SPI2RST OFFSET(14) NUMBITS(1) [],
        /// LCD reset
        LCDRST OFFSET(9) NUMBITS(1) [],
        /// TIM7 reset
        TIM7RST OFFSET(5) NUMBITS(1) [],
        /// TIM6 reset
        TIM6RST OFFSET(4) NUMBITS(1) [],
        /// TIM5 reset
        TIM5RST OFFSET(3) NUMBITS(1) [],
        /// TIM4 reset
        TIM4RST OFFSET(2) NUMBITS(1) [],
        /// TIM3 reset
        TIM3RST OFFSET(1) NUMBITS(1) [],
        /// TIM2 reset
        TIM2RST OFFSET(0) NUMBITS(1) []
    ],
    APB1RSTR2 [
        /// Low-power TIMER 2 reset
        LPTIM2RST OFFSET(5) NUMBITS(1) [],
        /// Single wire protocol reset
        SWPMI1RST OFFSET(2) NUMBITS(1) [],
        /// I2C4 reset (this bit is reserved for STM32L47x/L48x devices)
        I2C4RST OFFSET(1) NUMBITS(1) [],
        /// Low-power UART 1 reset
        LPUART1RST OFFSET(0) NUMBITS(1) []
    ],
    APB2RSTR [
        /// Digital filters for sigma-delta modulators (DFSDM1) reset
        DFSDM1RST OFFSET(24) NUMBITS(1) [],
        /// Serial audio interface 2 (SAI2) reset
        SAI2RST OFFSET(22) NUMBITS(1) [],
        /// Serial audio interface 1 (SAI1) reset
        SAI1RST OFFSET(21) NUMBITS(1) [],
        /// TIM17 reset
        TIM17RST OFFSET(18) NUMBITS(1) [],
        /// TIM16 reset
        TIM16RST OFFSET(17) NUMBITS(1) [],
        /// TIM15 reset
        TIM15RST OFFSET(16) NUMBITS(1) [],
        /// USART1 reset
        USART1RST OFFSET(14) NUMBITS(1) [],
        /// TIM8 reset
        TIM8RST OFFSET(13) NUMBITS(1) [],
        /// SPI 1 reset
        SPI1RST OFFSET(12) NUMBITS(1) [],
        /// TIM1 reset
        TIM1RST OFFSET(11) NUMBITS(1) [],
        /// SDMMC reset
        SDMMC1RST OFFSET(10) NUMBITS(1) [],
        /// SYSCFG + COMP + VREFBUF reset
        SYSCFGRST OFFSET(0) NUMBITS(1) []
    ],
    AHB1ENR [
        /// DMA2DEN enable
        DMA2DEN OFFSET(17) NUMBITS(1) [],
        /// TSCEN enable
        TSCEN OFFSET(16) NUMBITS(1) [],
        /// CRCEN enable
        CRCEN OFFSET(12) NUMBITS(1) [],
        /// FLASHEN enable
        FLASHEN OFFSET(8) NUMBITS(1) [],
        /// DMA2EN enable
        DMA2EN OFFSET(1) NUMBITS(1) [],
        /// DMA1EN enable
        DMA1EN OFFSET(0) NUMBITS(1) []
    ],
    AHB2ENR [
        /// Random number generator enable
        RNGEN OFFSET(18) NUMBITS(1) [],
        /// Hash enable (this bit is reserved for STM32L47x/L48x devices)
        HASHEN OFFSET(17) NUMBITS(1) [],
        /// AES hardware accelerator enable (this bit is reserved on STM32L47x/L49x devices).
        AESEN OFFSET(16) NUMBITS(1) [],
        /// Camera interface enable
        DCMIEN OFFSET(14) NUMBITS(1) [],
        /// ADC enable
        ADCEN OFFSET(13) NUMBITS(1) [],
        /// USB OTG FS module enable
        OTGFSEN OFFSET(12) NUMBITS(1) [],
        /// IO port I enable
        GPIOIEN OFFSET(8) NUMBITS(1) [],
        /// IO port H enable
        GPIOHEN OFFSET(7) NUMBITS(1) [],
        /// IO port G enable
        GPIOGEN OFFSET(6) NUMBITS(1) [],
        /// IO port F enable
        GPIOFEN OFFSET(5) NUMBITS(1) [],
        /// IO port E enable
        GPIOEEN OFFSET(4) NUMBITS(1) [],
        /// IO port D enable
        GPIODEN OFFSET(3) NUMBITS(1) [],
        /// IO port C enable
        GPIOCEN OFFSET(2) NUMBITS(1) [],
        /// IO port B enable
        GPIOBEN OFFSET(1) NUMBITS(1) [],
        /// IO port A enable
        GPIOAEN OFFSET(0) NUMBITS(1) []
    ],
    AHB3ENR [
        /// QUADSPI module enable
        QSPIEN OFFSET(8) NUMBITS(1) [],
        /// Flexible memory controller module enable
        FMCEN OFFSET(0) NUMBITS(1) []
    ],
    APB1ENR1 [
        /// LPTIM1 enable
        LPTIM1EN OFFSET(31) NUMBITS(1) [],
        /// OPAMP enable
        OPAMPEN OFFSET(30) NUMBITS(1) [],
        /// DAC enable
        DAC1EN OFFSET(29) NUMBITS(1) [],
        /// Power interface enable
        PWREN OFFSET(28) NUMBITS(1) [],
        /// CAN2 enable
        CAN2EN OFFSET(26) NUMBITS(1) [],
        /// CAN1 enable
        CAN1EN OFFSET(25) NUMBITS(1) [],
        /// CRS enable (this bit is reserved for STM32L47x/L48x devices)
        CRSEN OFFSET(24) NUMBITS(1) [],
        /// I2C3 enable
        I2C3EN OFFSET(23) NUMBITS(1) [],
        /// I2C 2 enable
        I2C2EN OFFSET(22) NUMBITS(1) [],
        /// I2C 1 enable
        I2C1EN OFFSET(21) NUMBITS(1) [],
        /// UART 5 enable
        UART5EN OFFSET(20) NUMBITS(1) [],
        /// UART 4 enable
        UART4EN OFFSET(19) NUMBITS(1) [],
        /// USART 3 enable
        USART3EN OFFSET(18) NUMBITS(1) [],
        /// USART 2 enable
        USART2EN OFFSET(17) NUMBITS(1) [],
        /// SPI 3 enable
        SPI3EN OFFSET(15) NUMBITS(1) [],
        /// SPI 2 enable
        SPI2EN OFFSET(14) NUMBITS(1) [],
        /// Window watchdog clock enable
        WWDGEN OFFSET(11) NUMBITS(1) [],
        /// RTC APB clock enable (this bit is reserved for STM32L47x/L48x devices)
        RTCAPBEN OFFSET(10) NUMBITS(1) [],
        /// LCD enable
        LCDEN OFFSET(9) NUMBITS(1) [],
        /// TIM7 enable
        TIM7EN OFFSET(5) NUMBITS(1) [],
        /// TIM6 enable
        TIM6EN OFFSET(4) NUMBITS(1) [],
        /// TIM5 enable
        TIM5EN OFFSET(3) NUMBITS(1) [],
        /// TIM4 enable
        TIM4EN OFFSET(2) NUMBITS(1) [],
        /// TIM3 enable
        TIM3EN OFFSET(1) NUMBITS(1) [],
        /// TIM2 enable
        TIM2EN OFFSET(0) NUMBITS(1) []
    ],
    APB1ENR2 [
        /// Low-power TIMER 2 enable
        LPTIM2EN OFFSET(5) NUMBITS(1) [],
        /// Single wire protocol enable
        SWPMI1EN OFFSET(2) NUMBITS(1) [],
        /// I2C4 enable (this bit is reserved for STM32L47x/L48x devices)
        I2C4EN OFFSET(1) NUMBITS(1) [],
        /// Low-power UART 1 enable
        LPUART1EN OFFSET(0) NUMBITS(1) []
    ],
    APB2ENR [
        /// Digital filters for sigma-delta modulators (DFSDM1) enable
        DFSDM1EN OFFSET(24) NUMBITS(1) [],
        /// Serial audio interface 2 (SAI2) enable
        SAI2EN OFFSET(22) NUMBITS(1) [],
        /// Serial audio interface 1 (SAI1) enable
        SAI1EN OFFSET(21) NUMBITS(1) [],
        /// TIM17 enable
        TIM17EN OFFSET(18) NUMBITS(1) [],
        /// TIM16 enable
        TIM16EN OFFSET(17) NUMBITS(1) [],
        /// TIM15 enable
        TIM15EN OFFSET(16) NUMBITS(1) [],
        /// USART1 enable
        USART1EN OFFSET(14) NUMBITS(1) [],
        /// TIM8 enable
        TIM8EN OFFSET(13) NUMBITS(1) [],
        /// SPI 1 enable
        SPI1EN OFFSET(12) NUMBITS(1) [],
        /// TIM1 enable
        TIM1EN OFFSET(11) NUMBITS(1) [],
        /// SDMMC enable
        SDMMC1EN OFFSET(10) NUMBITS(1) [],
        /// Firewall clock enable
        FWEN OFFSET(7) NUMBITS(1) [],
        /// SYSCFG + COMP + VREFBUF enable
        SYSCFGEN OFFSET(0) NUMBITS(1) []
    ],
    AHB1SMENR [
        /// DMA2DSMEN enable during Sleep and Stop modes
        DMA2DSMEN OFFSET(17) NUMBITS(1) [],
        /// TSCSMEN enable during Sleep and Stop modes
        TSCSMEN OFFSET(16) NUMBITS(1) [],
        /// CRCSMEN enable during Sleep and Stop modes
        CRCSMEN OFFSET(12) NUMBITS(1) [],
        /// SRAM1 interface clocks enable during Sleep and Stop modes
        SRAM1SMEN OFFSET(9) NUMBITS(1) [],
        /// FLASHSMEN enable during Sleep and Stop modes
        FLASHSMEN OFFSET(8) NUMBITS(1) [],
        /// DMA2SMEN enable during Sleep and Stop modes
        DMA2SMEN OFFSET(1) NUMBITS(1) [],
        /// DMA1SMEN enable during Sleep and Stop modes
        DMA1SMEN OFFSET(0) NUMBITS(1) []
    ],
    AHB2SMENR [
        /// Random number generator enable during Sleep and Stop modes
        RNGSMEN OFFSET(18) NUMBITS(1) [],
        /// Hash enable during Sleep and Stop modes (this bit is reserved for STM32L47x/L48x devices)
        HASHSMEN OFFSET(17) NUMBITS(1) [],
        /// AES hardware accelerator enable during Sleep and Stop modes (this bit is reserved on STM32L47x/L49x devices).
        AESSMEN OFFSET(16) NUMBITS(1) [],
        /// Camera interface enable during Sleep and Stop modes
        DCMISMEN OFFSET(14) NUMBITS(1) [],
        /// ADC enable during Sleep and Stop modes
        ADCSMEN OFFSET(13) NUMBITS(1) [],
        /// USB OTG FS module enable during Sleep and Stop modes
        OTGFSSMEN OFFSET(12) NUMBITS(1) [],
        /// SRAM2 interface clocks enable during Sleep and Stop modes
        SRAM2SMEN OFFSET(9) NUMBITS(1) [],
        /// IO port I enable during Sleep and Stop modes
        GPIOISMEN OFFSET(8) NUMBITS(1) [],
        /// IO port H enable during Sleep and Stop modes
        GPIOHSMEN OFFSET(7) NUMBITS(1) [],
        /// IO port G enable during Sleep and Stop modes
        GPIOGSMEN OFFSET(6) NUMBITS(1) [],
        /// IO port F enable during Sleep and Stop modes
        GPIOFSMEN OFFSET(5) NUMBITS(1) [],
        /// IO port E enable during Sleep and Stop modes
        GPIOESMEN OFFSET(4) NUMBITS(1) [],
        /// IO port D enable during Sleep and Stop modes
        GPIODSMEN OFFSET(3) NUMBITS(1) [],
        /// IO port C enable during Sleep and Stop modes
        GPIOCSMEN OFFSET(2) NUMBITS(1) [],
        /// IO port B enable during Sleep and Stop modes
        GPIOBSMEN OFFSET(1) NUMBITS(1) [],
        /// IO port A enable during Sleep and Stop modes
        GPIOASMEN OFFSET(0) NUMBITS(1) []
    ],
    AHB3SMENR [
        /// QUADSPI module enable during Sleep and Stop modes
        QSPISMEN OFFSET(8) NUMBITS(1) [],
        /// Flexible memory controller module enable during Sleep and Stop modes
        FMCSMEN OFFSET(0) NUMBITS(1) []
    ],
    APB1SMENR1 [
        /// LPTIM1 enable during Sleep and Stop modes
        LPTIM1SMEN OFFSET(31) NUMBITS(1) [],
        /// OPAMP enable during Sleep and Stop modes
        OPAMPSMEN OFFSET(30) NUMBITS(1) [],
        /// DAC enable during Sleep and Stop modes
        DAC1SMEN OFFSET(29) NUMBITS(1) [],
        /// Power interface enable during Sleep and Stop modes
        PWRSMEN OFFSET(28) NUMBITS(1) [],
        /// CAN2 enable during Sleep and Stop modes
        CAN2SMEN OFFSET(26) NUMBITS(1) [],
        /// CAN1 enable during Sleep and Stop modes
        CAN1SMEN OFFSET(25) NUMBITS(1) [],
        /// CRS enable during Sleep and Stop modes (this bit is reserved for STM32L47x/L48x devices)
        CRSSSMEN OFFSET(24) NUMBITS(1) [],
        /// I2C3 enable during Sleep and Stop modes
        I2C3SMEN OFFSET(23) NUMBITS(1) [],
        /// I2C 2 enable during Sleep and Stop modes
        I2C2SMEN OFFSET(22) NUMBITS(1) [],
        /// I2C 1 enable during Sleep and Stop modes
        I2C1SMEN OFFSET(21) NUMBITS(1) [],
        /// USART 5 enable during Sleep and Stop modes
        UART5SMEN OFFSET(20) NUMBITS(1) [],
        /// USART 4 enable during Sleep and Stop modes
        UART4SMEN OFFSET(19) NUMBITS(1) [],
        /// USART 3 enable during Sleep and Stop modes
        USART3SMEN OFFSET(18) NUMBITS(1) [],
        /// USART 2 enable during Sleep and Stop modes
        USART2SMEN OFFSET(17) NUMBITS(1) [],
        /// SPI 3 enable during Sleep and Stop modes
        SPI3SMEN OFFSET(15) NUMBITS(1) [],
        /// SPI 2 enable during Sleep and Stop modes
        SPI2SMEN OFFSET(14) NUMBITS(1) [],
        /// Window watchdog clock enable during Sleep and Stop modes
        WWDGSMEN OFFSET(11) NUMBITS(1) [],
        /// RTC APB clock enable during Sleep and Stop modes (this bit is reserved for STM32L47x/L48x devices)
        RTCAPBSMEN OFFSET(10) NUMBITS(1) [],
        /// LCD enable during Sleep and Stop modes
        LCDSMEN OFFSET(9) NUMBITS(1) [],
        /// TIM7 enable during Sleep and Stop modes
        TIM7SMEN OFFSET(5) NUMBITS(1) [],
        /// TIM6 enable during Sleep and Stop modes
        TIM6SMEN OFFSET(4) NUMBITS(1) [],
        /// TIM5 enable during Sleep and Stop modes
        TIM5SMEN OFFSET(3) NUMBITS(1) [],
        /// TIM4 enable during Sleep and Stop modes
        TIM4SMEN OFFSET(2) NUMBITS(1) [],
        /// TIM3 enable during Sleep and Stop modes
        TIM3SMEN OFFSET(1) NUMBITS(1) [],
        /// TIM2 enable during Sleep and Stop modes
        TIM2SMEN OFFSET(0) NUMBITS(1) []
    ],
    APB1SMENR2 [
        /// Low-power TIMER 2 enable during Sleep and Stop modes
        LPTIM2SMEN OFFSET(5) NUMBITS(1) [],
        /// Single wire protocol enable during Sleep and Stop modes
        SWPMI1SMEN OFFSET(2) NUMBITS(1) [],
        /// I2C4 enable during Sleep and Stop modes (this bit is reserved for STM32L47x/L48x devices)
        I2C4SMEN OFFSET(1) NUMBITS(1) [],
        /// Low-power UART 1 enable during Sleep and Stop modes
        LPUART1SMEN OFFSET(0) NUMBITS(1) []
    ],
    APB2SMENR [
        /// Digital filters for sigma-delta modulators (DFSDM1) enable during Sleep and Stop modes
        DFSDM1SMEN OFFSET(24) NUMBITS(1) [],
        /// Serial audio interface 2 (SAI2) enable during Sleep and Stop modes
        SAI2SMEN OFFSET(22) NUMBITS(1) [],
        /// Serial audio interface 1 (SAI1) enable during Sleep and Stop modes
        SAI1SMEN OFFSET(21) NUMBITS(1) [],
        /// TIM17 enable during Sleep and Stop modes
        TIM17SMEN OFFSET(18) NUMBITS(1) [],
        /// TIM16 enable during Sleep and Stop modes
        TIM16SMEN OFFSET(17) NUMBITS(1) [],
        /// TIM15 enable during Sleep and Stop modes
        TIM15SMEN OFFSET(16) NUMBITS(1) [],
        /// USART1 enable during Sleep and Stop modes
        USART1SMEN OFFSET(14) NUMBITS(1) [],
        /// TIM8 enable during Sleep and Stop modes
        TIM8SMEN OFFSET(13) NUMBITS(1) [],
        /// SPI 1 enable during Sleep and Stop modes
        SPI1SMEN OFFSET(12) NUMBITS(1) [],
        /// TIM1 enable during Sleep and Stop modes
        TIM1SMEN OFFSET(11) NUMBITS(1) [],
        /// SDMMC enable during Sleep and Stop modes
        SDMMC1SMEN OFFSET(10) NUMBITS(1) [],
        /// SYSCFG + COMP + VREFBUF enable during Sleep and Stop modes
        SYSCFGSMEN OFFSET(0) NUMBITS(1) []
    ],
    CCIPR [
        /// DFSDM1 clock source selection
        DFSDM1SEL OFFSET(31) NUMBITS(1) [],
        /// SWPMI1 clock source selection
        SWPMI1SEL OFFSET(30) NUMBITS(1) [],
        /// ADCs clock source selection
        ADCSEL OFFSET(28) NUMBITS(2) [],
        /// 48MHz clock source selection
        CLK48SEL OFFSET(26) NUMBITS(2) [],
        /// SAI2 clock source selection
        SAI2SEL OFFSET(24) NUMBITS(2) [],
        /// SAI1 clock source selection
        SAI1SEL OFFSET(22) NUMBITS(2) [],
        /// Low power timer 2 clock source selection
        LPTIM2SEL OFFSET(20) NUMBITS(2) [],
        /// Low power timer 1 clock source selection
        LPTIM1SEL OFFSET(18) NUMBITS(2) [],
        /// I2C3 clock source selection
        I2C3SEL OFFSET(16) NUMBITS(2) [],
        /// I2C2 clock source selection
        I2C2SEL OFFSET(14) NUMBITS(2) [],
        /// I2C1 clock source selection
        I2C1SEL OFFSET(12) NUMBITS(2) [],
        /// LPUART1 clock source selection
        LPUART1SEL OFFSET(10) NUMBITS(2) [],
        /// UART5 clock source selection
        UART5SEL OFFSET(8) NUMBITS(2) [],
        /// UART4 clock source selection
        UART4SEL OFFSET(6) NUMBITS(2) [],
        /// USART3 clock source selection
        USART3SEL OFFSET(4) NUMBITS(2) [],
        /// USART2 clock source selection
        USART2SEL OFFSET(2) NUMBITS(2) [],
        /// USART1 clock source selection
        USART1SEL OFFSET(0) NUMBITS(2) []
    ],
    BDCR [
        /// Low speed clock output selection
        LSCOSEL OFFSET(25) NUMBITS(1) [],
        /// Low speed clock output enable
        LSCOEN OFFSET(24) NUMBITS(1) [],
        /// Backup domain software reset
        BDRST OFFSET(16) NUMBITS(1) [],
        /// RTC clock enable
        RTCEN OFFSET(15) NUMBITS(1) [],
        /// RTC clock source selection
        RTCSEL OFFSET(8) NUMBITS(2) [],
        /// CSS on LSE failure Detection
        LSECSSD OFFSET(6) NUMBITS(1) [],
        /// CSS on LSE enable
        LSECSSON OFFSET(5) NUMBITS(1) [],
        /// LSE oscillator drive capability
        LSEDRV OFFSET(3) NUMBITS(2) [],
        /// External low-speed oscillator bypass
        LSEBYP OFFSET(2) NUMBITS(1) [],
        /// External low-speed oscillator ready
        LSERDY OFFSET(1) NUMBITS(1) [],
        /// External low-speed oscillator enable
        LSEON OFFSET(0) NUMBITS(1) []
    ],
    CSR [
        /// Low-power reset flag
        LPWRRSTF OFFSET(31) NUMBITS(1) [],
        /// Window watchdog reset flag
        WWDGRSTF OFFSET(30) NUMBITS(1) [],
        /// Independent watchdog reset flag
        IWWGRSTF OFFSET(29) NUMBITS(1) [],
        /// Software reset flag
        SFTRSTF OFFSET(28) NUMBITS(1) [],
        /// BOR reset flag
        BORRSTF OFFSET(27) NUMBITS(1) [],
        /// PIN reset flag
        PINRSTF OFFSET(26) NUMBITS(1) [],
        /// Option byte loader reset flag
        OBLRSTF OFFSET(25) NUMBITS(1) [],
        /// Remove reset flag
        FWRSTF OFFSET(24) NUMBITS(1) [],
        /// Remove reset flag
        RMVF OFFSET(23) NUMBITS(1) [],
        /// MSI range after Standby mode
        MSISRANGE OFFSET(8) NUMBITS(4) [],
        /// low-speed oscillator ready
        LSIRDY OFFSET(1) NUMBITS(1) [],
        /// low-speed oscillator enable
        LSION OFFSET(0) NUMBITS(1) []
    ],
    CRRCR [
        /// HSI48 clock calibration
        HSI48CAL OFFSET(7) NUMBITS(9) [],
        /// HSI48 clock ready flag
        HSI48RDY OFFSET(1) NUMBITS(1) [],
        /// HSI48 clock enable
        HSI48ON OFFSET(0) NUMBITS(1) []
    ],
    CCIPR2 [
        /// I2C4 clock source selection. Register is present on L496/L4A6 devices only.
        I2C4SEL OFFSET(0) NUMBITS(2) []
    ]
];

const RCC_REGS: StaticRef<RccRegisters> =
    unsafe { StaticRef::new(memory_map::RCC_BASE as *const RccRegisters) };

pub struct Rcc {
    registers: StaticRef<RccRegisters>,
}

pub static mut RCC: Rcc = Rcc::new();

impl Rcc {
    const fn new() -> Rcc {
        Rcc {
            registers: RCC_REGS,
        }
    }

    pub fn init(&self) {
        self.registers.apb1enr1.modify(APB1ENR1::PWREN::SET);
        self.registers.apb2enr.modify(APB2ENR::SYSCFGEN::SET)
    }

    // SPI3 clock

    fn is_enabled_spi3_clock(&self) -> bool {
        self.registers.apb1enr1.is_set(APB1ENR1::SPI3EN)
    }

    fn enable_spi3_clock(&self) {
        self.registers.apb1enr1.modify(APB1ENR1::SPI3EN::SET)
    }

    fn disable_spi3_clock(&self) {
        self.registers.apb1enr1.modify(APB1ENR1::SPI3EN::CLEAR)
    }

    // TIM2 clock

    fn is_enabled_tim2_clock(&self) -> bool {
        self.registers.apb1enr1.is_set(APB1ENR1::TIM2EN)
    }

    fn enable_tim2_clock(&self) {
        self.registers.apb1enr1.modify(APB1ENR1::TIM2EN::SET)
    }

    fn disable_tim2_clock(&self) {
        self.registers.apb1enr1.modify(APB1ENR1::TIM2EN::CLEAR)
    }

    // SYSCFG clock

    fn is_enabled_syscfg_clock(&self) -> bool {
        self.registers.apb2enr.is_set(APB2ENR::SYSCFGEN)
    }

    fn enable_syscfg_clock(&self) {
        self.registers.apb2enr.modify(APB2ENR::SYSCFGEN::SET)
    }

    fn disable_syscfg_clock(&self) {
        self.registers.apb2enr.modify(APB2ENR::SYSCFGEN::CLEAR)
    }

    // DMA1 clock

    // fn is_enabled_dma1_clock(&self) -> bool {
    //     self.registers.ahb1enr.is_set(AHB1ENR::DMA1EN)
    // }

    // fn enable_dma1_clock(&self) {
    //     self.registers.ahb1enr.modify(AHB1ENR::DMA1EN::SET)
    // }

    // fn disable_dma1_clock(&self) {
    //     self.registers.ahb1enr.modify(AHB1ENR::DMA1EN::CLEAR)
    // }

    // GPIOI clock

    fn is_enabled_gpioi_clock(&self) -> bool {
        self.registers.ahb2enr.is_set(AHB2ENR::GPIOIEN)
    }

    fn enable_gpioi_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOIEN::SET)
    }

    fn disable_gpioi_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOIEN::CLEAR)
    }

    // GPIOH clock

    fn is_enabled_gpioh_clock(&self) -> bool {
        self.registers.ahb2enr.is_set(AHB2ENR::GPIOHEN)
    }

    fn enable_gpioh_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOHEN::SET)
    }

    fn disable_gpioh_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOHEN::CLEAR)
    }

    // GPIOG clock

    fn is_enabled_gpiog_clock(&self) -> bool {
        self.registers.ahb2enr.is_set(AHB2ENR::GPIOGEN)
    }

    fn enable_gpiog_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOGEN::SET)
    }

    fn disable_gpiog_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOGEN::CLEAR)
    }

    // GPIOF clock

    fn is_enabled_gpiof_clock(&self) -> bool {
        self.registers.ahb2enr.is_set(AHB2ENR::GPIOFEN)
    }

    fn enable_gpiof_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOFEN::SET)
    }

    fn disable_gpiof_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOFEN::CLEAR)
    }

    // GPIOE clock

    fn is_enabled_gpioe_clock(&self) -> bool {
        self.registers.ahb2enr.is_set(AHB2ENR::GPIOEEN)
    }

    fn enable_gpioe_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOEEN::SET)
    }

    fn disable_gpioe_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOEEN::CLEAR)
    }

    // GPIOD clock

    fn is_enabled_gpiod_clock(&self) -> bool {
        self.registers.ahb2enr.is_set(AHB2ENR::GPIODEN)
    }

    fn enable_gpiod_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIODEN::SET)
    }

    fn disable_gpiod_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIODEN::CLEAR)
    }

    // GPIOC clock

    fn is_enabled_gpioc_clock(&self) -> bool {
        self.registers.ahb2enr.is_set(AHB2ENR::GPIOCEN)
    }

    fn enable_gpioc_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOCEN::SET)
    }

    fn disable_gpioc_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOCEN::CLEAR)
    }

    // GPIOB clock

    fn is_enabled_gpiob_clock(&self) -> bool {
        self.registers.ahb2enr.is_set(AHB2ENR::GPIOBEN)
    }

    fn enable_gpiob_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOBEN::SET)
    }

    fn disable_gpiob_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOBEN::CLEAR)
    }

    // GPIOA clock

    fn is_enabled_gpioa_clock(&self) -> bool {
        self.registers.ahb2enr.is_set(AHB2ENR::GPIOAEN)
    }

    fn enable_gpioa_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOAEN::SET)
    }

    fn disable_gpioa_clock(&self) {
        self.registers.ahb2enr.modify(AHB2ENR::GPIOAEN::CLEAR)
    }

    // USART2 clock

    fn is_enabled_usart2_clock(&self) -> bool {
        self.registers.apb1enr1.is_set(APB1ENR1::USART2EN)
    }

    fn enable_usart2_clock(&self) {
        self.registers.apb1enr1.modify(APB1ENR1::USART2EN::SET)
    }

    fn disable_usart2_clock(&self) {
        self.registers.apb1enr1.modify(APB1ENR1::USART2EN::CLEAR)
    }

    // USART3 clock

    fn is_enabled_usart3_clock(&self) -> bool {
        self.registers.apb1enr1.is_set(APB1ENR1::USART3EN)
    }

    fn enable_usart3_clock(&self) {
        self.registers.apb1enr1.modify(APB1ENR1::USART3EN::SET)
    }

    fn disable_usart3_clock(&self) {
        self.registers.apb1enr1.modify(APB1ENR1::USART3EN::CLEAR)
    }
}

/// Clock sources for CPU
pub enum CPUClock {
    MSI,
    HSI,
    HSE,
    PLL,
}

/// Bus + Clock name for the peripherals
///
/// Not yet implemented clocks:
///
/// AHB2(HCLK2)
/// AHB3(HCLK3)
pub enum PeripheralClock {
    AHB1(HCLK1),
    AHB2(HCLK2),
    AHB3(HCLK3),
    APB1(PCLK1_1),
    APB2(PCLK2),
}

/// Peripherals clocked by HCLK1
pub enum HCLK1 {
    // hclk1
    DMA1,
    DMA2,
    FLASH,
    CRC,
    TSC,
    DMA2D,
}

/// Peripherals clocked by HCLK2
pub enum HCLK2 {
    GPIOA,
    GPIOB,
    GPIOC,
    GPIOD,
    GPIOE,
    GPIOF,
    GPIOG,
    GPIOH,
    GPIOI,
    OTGFSEN,
    ADC,
    DCMI,
    // AES, RESERVED FOR STM32L47x/L49x
    // HASH, RESERVED FOR STM32L47x/L48x
    RNG,
}

/// Peripherals clocked by HCLK3
pub enum HCLK3 {
    FMC,
    QSPIEN,
}

/// Peripherals clocked by PCLK1
pub enum PCLK1_1 {
    // pclk1_1
    TIM2,
    TIM3,
    TIM4,
    TIM5,
    TIM6,
    TIM7,
    LCD,
    // RTC, RESERVED FOR STM32L47x/L48x
    WWDG,
    SPI2,
    SPI3,
    USART2,
    USART3,
    UART4,
    UART5,
    I2C1,
    I2C2,
    I2C3,
    // CRS, RESERVED FOR STM32L47x/L48x
    CAN1,
    CAN2,
    PWR,
    DAC1,
    OPAMP,
    LPTIM1,
}

/// Peripherals clocked by PCLK1_2
pub enum PCLK1_2 {
    LPUART1,
    // I2C4, RESERVED FOR STM32L47x/L48x
    SWPMI1,
    LPTIM2,
}

/// Peripherals clocked by PCLK2
pub enum PCLK2 {
    SYSCFG,
    FW,
    SDMMC1,
    TIM1,
    SPI1,
    TIM8,
    USART1,
    TIM15,
    TIM16,
    TIM17,
    SAI1,
    SAI2,
    DFSD,
}

impl ClockInterface for PeripheralClock {
    fn is_enabled(&self) -> bool {
        match self {
            &PeripheralClock::AHB1(ref v) => match v {
                // TODO
                _ => false,
            },
            &PeripheralClock::AHB2(ref v) => match v {
                HCLK2::GPIOI => unsafe { RCC.is_enabled_gpioi_clock() },
                HCLK2::GPIOH => unsafe { RCC.is_enabled_gpioh_clock() },
                HCLK2::GPIOG => unsafe { RCC.is_enabled_gpiog_clock() },
                HCLK2::GPIOF => unsafe { RCC.is_enabled_gpiof_clock() },
                HCLK2::GPIOE => unsafe { RCC.is_enabled_gpioe_clock() },
                HCLK2::GPIOD => unsafe { RCC.is_enabled_gpiod_clock() },
                HCLK2::GPIOC => unsafe { RCC.is_enabled_gpioc_clock() },
                HCLK2::GPIOB => unsafe { RCC.is_enabled_gpiob_clock() },
                HCLK2::GPIOA => unsafe { RCC.is_enabled_gpioa_clock() },

                HCLK2::OTGFSEN => unsafe { RCC.registers.ahb2enr.is_set(AHB2ENR::OTGFSEN) },
                HCLK2::ADC => unsafe { RCC.registers.ahb2enr.is_set(AHB2ENR::ADCEN) },
                HCLK2::DCMI => unsafe { RCC.registers.ahb2enr.is_set(AHB2ENR::DCMIEN) },
                // HCLK2::AES => unsafe { }, RESERVED FOR STM32L47x/L49x
                // HCLK2::HASH => unsafe { }, RESERVED FOR STM32L47x/L48x
                HCLK2::RNG => unsafe { RCC.registers.ahb2enr.is_set(AHB2ENR::RNGEN) },
            },
            &PeripheralClock::AHB3(ref v) => match v {
                // TODO
                _ => false,
            },
            &PeripheralClock::APB1(ref v) => match v {
                // PCLK1_1::TIM2 => unsafe { RCC.is_enabled_tim2_clock() },
                // PCLK1_1::USART2 => unsafe { RCC.is_enabled_usart2_clock() },
                // PCLK1_1::USART3 => unsafe { RCC.is_enabled_usart3_clock() },
                // PCLK1_1::SPI3 => unsafe { RCC.is_enabled_spi3_clock() },
                // pclk1_1
                PCLK1_1::TIM2 => unsafe { RCC.is_enabled_tim2_clock() },
                PCLK1_1::TIM3 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::TIM3EN) },
                PCLK1_1::TIM4 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::TIM4EN) },
                PCLK1_1::TIM5 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::TIM5EN) },
                PCLK1_1::TIM6 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::TIM6EN) },
                PCLK1_1::TIM7 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::TIM7EN) },
                PCLK1_1::LCD => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::LCDEN) },
                // PCLK1_1::RTC => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::RTCEN) }, RESERVED FOR STM32L47x/L48x
                PCLK1_1::WWDG => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::WWDGEN) },
                PCLK1_1::SPI2 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::SPI2EN) },
                PCLK1_1::SPI3 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::SPI3EN) },
                PCLK1_1::USART2 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::USART2EN) },
                PCLK1_1::USART3 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::USART3EN) },
                PCLK1_1::UART4 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::UART4EN) },
                PCLK1_1::UART5 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::UART5EN) },
                PCLK1_1::I2C1 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::I2C1EN) },
                PCLK1_1::I2C2 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::I2C2EN) },
                PCLK1_1::I2C3 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::I2C3EN) },
                // PCLK1_1::CRS => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::CRSEN) }, RESERVED FOR STM32L47x/L48x
                PCLK1_1::CAN1 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::CAN1EN) },
                PCLK1_1::CAN2 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::CAN2EN) },
                PCLK1_1::PWR => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::PWREN) },
                PCLK1_1::DAC1 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::DAC1EN) },
                PCLK1_1::OPAMP => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::OPAMPEN) },
                PCLK1_1::LPTIM1 => unsafe { RCC.registers.apb1enr1.is_set(APB1ENR1::LPTIM1EN) },
            },
            &PeripheralClock::APB2(ref v) => match v {
                // PCLK2::SYSCFG => unsafe { RCC.is_enabled_syscfg_clock() },
                PCLK2::SYSCFG => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::SYSCFGEN) },
                PCLK2::FW => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::FWEN) },
                PCLK2::SDMMC1 => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::SDMMC1EN) },
                PCLK2::TIM1 => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::TIM1EN) },
                PCLK2::SPI1 => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::SPI1EN) },
                PCLK2::TIM8 => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::TIM8EN) },
                PCLK2::USART1 => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::USART1EN) },
                PCLK2::TIM15 => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::TIM15EN) },
                PCLK2::TIM16 => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::TIM16EN) },
                PCLK2::TIM17 => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::TIM17EN) },
                PCLK2::SAI1 => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::SAI1EN) },
                PCLK2::SAI2 => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::SAI2EN) },
                PCLK2::DFSD => unsafe { RCC.registers.apb2enr.is_set(APB2ENR::DFSDM1EN) },
            },
        }
    }

    fn enable(&self) {
        match self {
            &PeripheralClock::AHB1(ref v) => match v {
                // TODO
                _ => {}
            },
            &PeripheralClock::AHB2(ref v) => match v {
                HCLK2::GPIOI => unsafe {
                    RCC.registers.ahb2enr.modify(AHB2ENR::GPIOIEN::SET);
                },
                HCLK2::GPIOH => unsafe {
                    RCC.enable_gpioh_clock();
                },
                HCLK2::GPIOG => unsafe {
                    RCC.enable_gpiog_clock();
                },
                HCLK2::GPIOF => unsafe {
                    RCC.enable_gpiof_clock();
                },
                HCLK2::GPIOE => unsafe {
                    RCC.enable_gpioe_clock();
                },
                HCLK2::GPIOD => unsafe {
                    RCC.enable_gpiod_clock();
                },
                HCLK2::GPIOC => unsafe {
                    RCC.enable_gpioc_clock();
                },
                HCLK2::GPIOB => unsafe {
                    RCC.enable_gpiob_clock();
                },
                HCLK2::GPIOA => unsafe {
                    RCC.enable_gpioa_clock();
                },

                HCLK2::OTGFSEN => unsafe { RCC.registers.ahb2enr.modify(AHB2ENR::OTGFSEN::SET) },
                HCLK2::ADC => unsafe { RCC.registers.ahb2enr.modify(AHB2ENR::ADCEN::SET) },
                HCLK2::DCMI => unsafe { RCC.registers.ahb2enr.modify(AHB2ENR::DCMIEN::SET) },
                // HCLK2::AES => unsafe { }, RESERVED FOR STM32L47x/L49x
                // HCLK2::HASH => unsafe { }, RESERVED FOR STM32L47x/L48x
                HCLK2::RNG => unsafe { RCC.registers.ahb2enr.modify(AHB2ENR::RNGEN::SET) },
            },
            &PeripheralClock::AHB3(ref v) => match v {
                // TODO
                _ => {}
            },
            &PeripheralClock::APB1(ref v) => match v {
                // PCLK1_1::TIM2 => unsafe { RCC.is_enabled_tim2_clock() },
                // PCLK1_1::USART2 => unsafe { RCC.is_enabled_usart2_clock() },
                // PCLK1_1::USART3 => unsafe { RCC.is_enabled_usart3_clock() },
                // PCLK1_1::SPI3 => unsafe { RCC.is_enabled_spi3_clock() },
                // pclk1_1
                PCLK1_1::TIM2 => unsafe { RCC.enable_tim2_clock() },
                PCLK1_1::TIM3 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::TIM3EN::SET);
                },
                PCLK1_1::TIM4 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::TIM4EN::SET);
                },
                PCLK1_1::TIM5 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::TIM5EN::SET);
                },
                PCLK1_1::TIM6 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::TIM6EN::SET);
                },
                PCLK1_1::TIM7 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::TIM7EN::SET);
                },
                PCLK1_1::LCD => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::LCDEN::SET);
                },
                // PCLK1_1::RTC => unsafe { RCC.registers.apb1enr1.modify(APB1ENR1::RTCEN::SET); }, RESERVED FOR STM32L47x/L48x
                PCLK1_1::WWDG => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::WWDGEN::SET);
                },
                PCLK1_1::SPI2 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::SPI2EN::SET);
                },
                PCLK1_1::SPI3 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::SPI3EN::SET);
                },
                PCLK1_1::USART2 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::USART2EN::SET);
                },
                PCLK1_1::USART3 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::USART3EN::SET);
                },
                PCLK1_1::UART4 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::UART4EN::SET);
                },
                PCLK1_1::UART5 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::UART5EN::SET);
                },
                PCLK1_1::I2C1 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::I2C1EN::SET);
                },
                PCLK1_1::I2C2 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::I2C2EN::SET);
                },
                PCLK1_1::I2C3 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::I2C3EN::SET);
                },
                // PCLK1_1::CRS => unsafe { RCC.registers.apb1enr1.modify(APB1ENR1::CRSEN::SET); }, RESERVED FOR STM32L47x/L48x
                PCLK1_1::CAN1 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::CAN1EN::SET);
                },
                PCLK1_1::CAN2 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::CAN2EN::SET);
                },
                PCLK1_1::PWR => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::PWREN::SET);
                },
                PCLK1_1::DAC1 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::DAC1EN::SET);
                },
                PCLK1_1::OPAMP => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::OPAMPEN::SET);
                },
                PCLK1_1::LPTIM1 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::LPTIM1EN::SET);
                },
            },
            &PeripheralClock::APB2(ref v) => match v {
                // PCLK2::SYSCFG => unsafe { RCC.is_enabled_syscfg_clock() },
                PCLK2::SYSCFG => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::SYSCFGEN::SET);
                },
                PCLK2::FW => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::FWEN::SET);
                },
                PCLK2::SDMMC1 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::SDMMC1EN::SET);
                },
                PCLK2::TIM1 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::TIM1EN::SET);
                },
                PCLK2::SPI1 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::SPI1EN::SET);
                },
                PCLK2::TIM8 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::TIM8EN::SET);
                },
                PCLK2::USART1 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::USART1EN::SET);
                },
                PCLK2::TIM15 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::TIM15EN::SET);
                },
                PCLK2::TIM16 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::TIM16EN::SET);
                },
                PCLK2::TIM17 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::TIM17EN::SET);
                },
                PCLK2::SAI1 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::SAI1EN::SET);
                },
                PCLK2::SAI2 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::SAI2EN::SET);
                },
                PCLK2::DFSD => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::DFSDM1EN::SET);
                },
            },
        }
    }

    fn disable(&self) {
        match self {
            &PeripheralClock::AHB1(ref v) => match v {
                // TODO
                _ => {}
            },
            &PeripheralClock::AHB2(ref v) => match v {
                HCLK2::GPIOI => unsafe {
                    RCC.registers.ahb2enr.modify(AHB2ENR::GPIOIEN::SET);
                },
                HCLK2::GPIOH => unsafe {
                    RCC.enable_gpioh_clock();
                },
                HCLK2::GPIOG => unsafe {
                    RCC.enable_gpiog_clock();
                },
                HCLK2::GPIOF => unsafe {
                    RCC.enable_gpiof_clock();
                },
                HCLK2::GPIOE => unsafe {
                    RCC.enable_gpioe_clock();
                },
                HCLK2::GPIOD => unsafe {
                    RCC.enable_gpiod_clock();
                },
                HCLK2::GPIOC => unsafe {
                    RCC.enable_gpioc_clock();
                },
                HCLK2::GPIOB => unsafe {
                    RCC.enable_gpiob_clock();
                },
                HCLK2::GPIOA => unsafe {
                    RCC.enable_gpioa_clock();
                },

                HCLK2::OTGFSEN => unsafe { RCC.registers.ahb2enr.modify(AHB2ENR::OTGFSEN::SET) },
                HCLK2::ADC => unsafe { RCC.registers.ahb2enr.modify(AHB2ENR::ADCEN::SET) },
                HCLK2::DCMI => unsafe { RCC.registers.ahb2enr.modify(AHB2ENR::DCMIEN::SET) },
                // HCLK2::AES => {unsafe { }, RESERVED FOR STM32L47x/L49x
                // HCLK2::HASH => unsafe { }, RESERVED FOR STM32L47x/L48x
                HCLK2::RNG => unsafe { RCC.registers.ahb2enr.modify(AHB2ENR::RNGEN::SET) },
            },
            &PeripheralClock::AHB3(ref v) => match v {
                // TODO
                _ => {}
            },
            &PeripheralClock::APB1(ref v) => match v {
                // PCLK1_1::TIM2 => unsafe { RCC.is_enabled_tim2_clock() },
                // PCLK1_1::USART2 => unsafe { RCC.is_enabled_usart2_clock() },
                // PCLK1_1::USART3 => unsafe { RCC.is_enabled_usart3_clock() },
                // PCLK1_1::SPI3 => unsafe { RCC.is_enabled_spi3_clock() },
                // pclk1_1
                PCLK1_1::TIM2 => unsafe { RCC.disable_tim2_clock() },
                PCLK1_1::TIM3 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::TIM3EN::CLEAR);
                },
                PCLK1_1::TIM4 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::TIM4EN::CLEAR);
                },
                PCLK1_1::TIM5 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::TIM5EN::CLEAR);
                },
                PCLK1_1::TIM6 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::TIM6EN::CLEAR);
                },
                PCLK1_1::TIM7 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::TIM7EN::CLEAR);
                },
                PCLK1_1::LCD => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::LCDEN::CLEAR);
                },
                // PCLK1_1::RTC => unsafe { RCC.registers.apb1enr1.modify(APB1ENR1::RTCEN::CLEAR); }, RESERVED FOR STM32L47x/L48x
                PCLK1_1::WWDG => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::WWDGEN::CLEAR);
                },
                PCLK1_1::SPI2 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::SPI2EN::CLEAR);
                },
                PCLK1_1::SPI3 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::SPI3EN::CLEAR);
                },
                PCLK1_1::USART2 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::USART2EN::CLEAR);
                },
                PCLK1_1::USART3 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::USART3EN::CLEAR);
                },
                PCLK1_1::UART4 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::UART4EN::CLEAR);
                },
                PCLK1_1::UART5 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::UART5EN::CLEAR);
                },
                PCLK1_1::I2C1 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::I2C1EN::CLEAR);
                },
                PCLK1_1::I2C2 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::I2C2EN::CLEAR);
                },
                PCLK1_1::I2C3 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::I2C3EN::CLEAR);
                },
                // PCLK1_1::CRS => unsafe { RCC.registers.apb1enr1.modify(APB1ENR1::CRSEN::CLEAR); }, RESERVED FOR STM32L47x/L48x
                PCLK1_1::CAN1 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::CAN1EN::CLEAR);
                },
                PCLK1_1::CAN2 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::CAN2EN::CLEAR);
                },
                PCLK1_1::PWR => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::PWREN::CLEAR);
                },
                PCLK1_1::DAC1 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::DAC1EN::CLEAR);
                },
                PCLK1_1::OPAMP => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::OPAMPEN::CLEAR);
                },
                PCLK1_1::LPTIM1 => unsafe {
                    RCC.registers.apb1enr1.modify(APB1ENR1::LPTIM1EN::CLEAR);
                },
            },
            &PeripheralClock::APB2(ref v) => match v {
                // PCLK2::SYSCFG => unsafe { RCC.is_enabled_syscfg_clock() },
                PCLK2::SYSCFG => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::SYSCFGEN::CLEAR);
                },
                PCLK2::FW => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::FWEN::CLEAR);
                },
                PCLK2::SDMMC1 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::SDMMC1EN::CLEAR);
                },
                PCLK2::TIM1 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::TIM1EN::CLEAR);
                },
                PCLK2::SPI1 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::SPI1EN::CLEAR);
                },
                PCLK2::TIM8 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::TIM8EN::CLEAR);
                },
                PCLK2::USART1 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::USART1EN::CLEAR);
                },
                PCLK2::TIM15 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::TIM15EN::CLEAR);
                },
                PCLK2::TIM16 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::TIM16EN::CLEAR);
                },
                PCLK2::TIM17 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::TIM17EN::CLEAR);
                },
                PCLK2::SAI1 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::SAI1EN::CLEAR);
                },
                PCLK2::SAI2 => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::SAI2EN::CLEAR);
                },
                PCLK2::DFSD => unsafe {
                    RCC.registers.apb2enr.modify(APB2ENR::DFSDM1EN::CLEAR);
                },
            },
        }
    }
}
