#include "main.h"
#include "stm32f7xx_hal.h"

#include "dcmi.h"
#include "gpio.h"

#include "timers.h"

#define TMRD(x) (x << 0)
#define TXSR(x) (x << 4)
#define TRAS(x) (x << 8)
#define TRC(x)  (x << 12)
#define TWR(x)  (x << 16)
#define TRP(x)  (x << 20)
#define TRCD(x) (x << 24)




void SystemClock_Config         (void);

void dma_dcmi_init              (void);



void fmc_sdram_init             (void);




void usart_init                 (void);

 
uint8_t     g_frame_buffer      [200][1504]; //[200][1504];

uint32_t    g_frame_cnt         = 0;






unsigned int*   pData = 0xD0000000;
unsigned int    Data  = 0;



int main(void)
{
    HAL_Init();

    SystemClock_Config();        


    
    gpio_init      ();
    fmc_sdram_init  ();
    
    pData[0] = 0x12345678;
    Data     = pData[0];

    
    for( int i = 0;i < 100;i++ )
    {
        pData[ i ] = i;
        Data = pData[ i ];
    }
    
    
    
    
    
    while(1);
    
    
    
    
    dma_dcmi_init   ();
    dcmi_fcb_ex48ep_init();
    

    // for ov7725
    //tim4_ch4_pwm_init(10, 10);
    //TIM4->CR1 |= TIM_CR1_CEN;

    usart_init();
    
                


    //while( !(DCMI->RISR & DCMI_RISR_FRAME_RIS) );
    
    
    uint8_t     bData   = 0;
    uint32_t    dwOffset= 0;

    while (1)
    {
        while( !(USART6->ISR & USART_ISR_RXNE));        
        bData       = USART6->RDR;
        
        if( bData == 'C' )
        {
            for( int i = 0;i < 1504;i++ )
            {
                USART6->TDR = g_frame_buffer[dwOffset][ i ];
                while( !(USART6->ISR & USART_ISR_TC));
            }
            
            dwOffset++;
        }
        
        
        if( bData == 'R' )
            dwOffset = 0;
        
        
        
        
    }
}


void dma_dcmi_init(void)
{
    // DMA2 controller clock enable    
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Setup DMA2 Stream7 for DCMI CHANNEL1
    DMA2_Stream7->CR &= ~DMA_SxCR_EN; // Выключить DMA
    while (DMA2_Stream7->CR & DMA_SxCR_EN); // Убедиться что он выключен

    DMA2_Stream7->PAR = (uint32_t) & DCMI->DR; // Source
    DMA2_Stream7->M0AR = (uint32_t) & g_frame_buffer[g_frame_cnt]; // Destination

    DMA2_Stream7->CR |= DMA_CHANNEL_1;
    DMA2_Stream7->NDTR = 65535;
    DMA2_Stream7->CR |= DMA_NORMAL;  
    
    DMA2_Stream7->CR |= DMA_PRIORITY_LOW;
    DMA2_Stream7->CR |= DMA_PERIPH_TO_MEMORY; // Memory to peripherial
    DMA2_Stream7->CR |= DMA_PINC_DISABLE; // Peripheral increment mode disable
    DMA2_Stream7->CR |= DMA_MINC_ENABLE; // Memory increment mode enable        
    DMA2_Stream7->CR |= DMA_PBURST_INC4;
    //DMA2_Stream7->CR |= DMA_PBURST_INC16;
    
    
    DMA2_Stream7->CR |= DMA_PDATAALIGN_WORD;
    DMA2_Stream7->CR |= DMA_MDATAALIGN_WORD;

    //DMA2_Stream7->FCR |= DMA_SxFCR_DMDIS; // Disable direct mode

    DMA2_Stream7->CR |= DMA_SxCR_EN;
}



//Note:     The DMA controller and all DCMI configuration registers should be 
  //          programmed correctly before enabling this bit.




// SDRAM: IC42S16400
//
// - Programmable CAS latency (2 and 3)

void fmc_sdram_init(void)
{
    RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;
    
    
    // SDRAM clock configuration
    // These bits define the SDRAM clock period for both SDRAM banks and allow disabling the clock
    // before changing the frequency. In this case the SDRAM must be re-initialized.
    
    
    

    FMC_Bank5_6->SDCR[0]  =  0x00000000;
    FMC_Bank5_6->SDCR[0] |= ~FMC_SDCR1_RPIPE_0;                   // No HCLK clock cycle delay
    FMC_Bank5_6->SDCR[0] |=  FMC_SDCR1_RBURST;                  // Single read requests are always managed as bursts
    FMC_Bank5_6->SDCR[0] |=  FMC_SDCR1_SDCLK_1;                 // SDCLK period = 2 x HCLK  133MHz

    // Bank_2
    FMC_Bank5_6->SDCR[1]  =  0x00000000;
    FMC_Bank5_6->SDCR[1] |=  FMC_SDCR2_NB;                      // Number of internal banks     : 4
    FMC_Bank5_6->SDCR[1] |=  FMC_SDCR2_CAS_0 | FMC_SDCR2_CAS_1; // CAS                          : 3 cycles
    FMC_Bank5_6->SDCR[1] |=  FMC_SDCR2_MWID_0;                  // Memory data width            : 16bit        
    FMC_Bank5_6->SDCR[1] |=  FMC_SDCR2_NR_0;                    // Number of row address bit    : 12bit
    FMC_Bank5_6->SDCR[1] &= ~FMC_SDCR2_NC;                      // Number of column address bit :  8bit

    

    
    FMC_Bank5_6->SDTR[0] = 0;
    FMC_Bank5_6->SDTR[1] = 0;
        
    // Trcd - RAS to CAS delay               
    // 20ns for 133MHz(-7)
    // 20ns / 7,5ns ~= 2.6 -> 3    
    FMC_Bank5_6->SDTR[1] |=  (3 - 1) << FMC_SDTR2_TRCD_Pos;
    
    
    // Trp - time row precharge delay.
    // 20ns for 133MHz(-7)
    // 20ns / 7,5ns ~= 2.6 -> 3    
    FMC_Bank5_6->SDTR[0] |=  (3 - 1) << FMC_SDTR1_TRP_Pos;  // 0: 1 cycle
                                                            // 1: 2 cycle
    
    // Twr(Tdpl Data in to precharge) - Recovery delay between a Write and a Precharge
    // 15ns / 7,5    = 2
    FMC_Bank5_6->SDTR[1] |=  (2 - 1) << FMC_SDTR2_TWR_Pos;  // 0: 1 cycle
                                                            // 1: 2 cycle
            
    // Trc - time row cycle     delay
    // 67.5ns for 133MHz(-7)
    // 67.5 / 7,5ns  = 9 cycles    
    FMC_Bank5_6->SDTR[0] |=  (9 - 1) << FMC_SDTR1_TRC_Pos;
    
    
    // Tras - row active time
    // 45ns
    // 45ns / 7,5ns  = 6 cycles    
    FMC_Bank5_6->SDTR[1] |= (6 - 1) << FMC_SDTR2_TRAS_Pos;
    
    
    // Txsr - Exit Self-refresh delay
    // 7,5ns/ 7,5ns  = 1 cycle
    FMC_Bank5_6->SDTR[1] |= (1 - 1) << FMC_SDTR2_TXSR_Pos;
   
    // Load Mode Register to Active
    // 10ns / 7,5ns ~= 1.3 -> 2    
    FMC_Bank5_6->SDTR[1] |=  (2 - 1) << FMC_SDTR2_TMRD_Pos;


       


    // 3. Set MODE bits to ‘001’ and configure the Target Bank bits (CTB1 and/or CTB2) in the
    // FMC_SDCMR register to start delivering the clock to the memory (SDCKE is driven high).
    // Target bank 2    
    FMC_Bank5_6->SDCMR   |=  FMC_SDCMR_MODE_0 | FMC_SDCMR_CTB2;
    
    
    // 4. Wait during the prescribed delay period. Typical delay is around 100 μs (refer to the
    // SDRAM datasheet for the required delay after power-up).
    // IC42S16400: Maintain stable power, stable clock , and NOP input conditions for a minimum of 200us    
    for(int i = 0;i < 25000;i++)    
        for(int j = 0;j < 300;j++);
    
    
    
    // 5. Set MODE bits to ‘010’ and configure the Target Bank bits (CTB1 and/or CTB2) in the
    // FMC_SDCMR register to issue a “Precharge All” command.
    //FMC_Bank5_6->SDCMR   &= ~( FMC_SDCMR_MODE | FMC_SDCMR_CTB1 | FMC_SDCMR_CTB2);
    FMC_Bank5_6->SDCMR  |=  FMC_SDCMR_MODE_1 | FMC_SDCMR_CTB2;
    
    for(int i = 0;i < 25000;i++)    
        for(int j = 0;j < 300;j++);
    
    // 6. Set MODE bits to ‘011’, and configure the Target Bank bits (CTB1 and/or CTB2) as well
    // as the number of consecutive Auto-refresh commands (NRFS) in the FMC_SDCMR
    // register. Refer to the SDRAM datasheet for the number of Auto-refresh commands that
    // should be issued. Typical number is 8
    //FMC_Bank5_6->SDCMR   &= ~( FMC_SDCMR_MODE | FMC_SDCMR_CTB1 | FMC_SDCMR_CTB2 | FMC_SDCMR_NRFS);
    //FMC_Bank5_6->SDCMR   |=   FMC_SDCMR_MODE_0 | FMC_SDCMR_MODE_1 | FMC_SDCMR_CTB2 | FMC_SDCMR_NRFS_3;        
    FMC_Bank5_6->SDCMR      =(8 << 5) /* NRFS */ | FMC_SDCMR_CTB2 | (3); // Autorefresh
    
    for(int i = 0;i < 25000;i++)    
        for(int j = 0;j < 300;j++);
    
    
    
    // 7. Configure the MRD field set the MODE bits to '100',
    // and configure the Target Bank bits (CTB1 and/or CTB2) in the FMC_SDCMR register
    // to issue a "Load Mode Register"
            
    // - the CAS latency must be selected   : 3
    // - burst length must be selected to   : 1
        
    
    FMC_Bank5_6->SDCMR   = ((/*(4 << 7) |*/ (3 << 4) | 0) << 9) | FMC_SDCMR_CTB2 | 4;
    
    for(int i = 0;i < 25000;i++)    
        for(int j = 0;j < 300;j++);
    
           
    // 8. Program the refresh rate in the FMC_SDRTR register
    FMC_Bank5_6->SDRTR   = 2058;
    
    
    
    
    
    
}







void usart_init(void)
{
    // PCLK2: 108MHz
    
    RCC->APB2ENR|= RCC_APB2ENR_USART6EN;
    
    // Word length                      : 1 Start bit, 8 data bits, n stop bits
    USART6->CR1 &= ~USART_CR1_M;    
    USART6->BRR  =  938;
    
    USART6->CR2 &= ~USART_CR2_STOP; 
    USART6->CR1 |=  USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;        
}




void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;


    uint32_t tmpreg;
    (void) tmpreg;

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    tmpreg = RCC->APB1ENR & RCC_APB1ENR_PWREN; // Delay after an RCC peripheral clock enabling


    PWR->CR1 &= ~PWR_CR1_VOS;
    PWR->CR1 |= PWR_REGULATOR_VOLTAGE_SCALE1;
    tmpreg = PWR->CR1 & PWR_CR1_VOS; // Delay after an RCC peripheral clock enabling



    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 4;     // 8;
    RCC_OscInitStruct.PLL.PLLN = 133;   // 216;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        _Error_Handler(__FILE__, __LINE__);



    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
        _Error_Handler(__FILE__, __LINE__);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
        _Error_Handler(__FILE__, __LINE__);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void _Error_Handler(char * file, int line)
{
    while (1)
    {
    }
}

void HAL_MspInit(void)
{
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    /* BusFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    /* UsageFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    /* SVCall_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    /* DebugMonitor_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}