#include "main.h"
#include "stm32f7xx_hal.h"

#include "dcmi.h"
#include "timers.h"

#define TMRD(x) (x << 0)
#define TXSR(x) (x << 4)
#define TRAS(x) (x << 8)
#define TRC(x)  (x << 12)
#define TWR(x)  (x << 16)
#define TRP(x)  (x << 20)
#define TRCD(x) (x << 24)




void SystemClock_Config         (void);
void ports_init                 (void);

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


    
    ports_init      ();
    fmc_sdram_init  ();
    

    
    *pData = 0x1234;
    Data  = *pData;
    
    
    
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

 void ports_init(void)
{
    // GPIO: A B C D E F G H I
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;



    // Mode                 : Alternate function
    GPIOA->MODER    &= ~(GPIO_MODER_MODER4   | GPIO_MODER_MODER6  );
    GPIOA->MODER    |=  (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER6_1);

    // Alternate function   :
    GPIOA->AFR[ 0 ] &= ~0x000F0000; // PA4  - DCMI_HSYNC
    GPIOA->AFR[ 0 ] |=  0x000D0000;

    GPIOA->AFR[ 0 ] &= ~0x0F000000; // PA6  - DCMI_PIXCLK
    GPIOA->AFR[ 0 ] |=  0x0D000000;



    // Mode                 : Alternate function
    GPIOB->MODER    &= ~(GPIO_MODER_MODER7   | GPIO_MODER_MODER8   | GPIO_MODER_MODER9  );
    GPIOB->MODER    |=  (GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
    
    GPIOB->AFR[ 0 ] &= ~0xF0000000; // PB7  - DCMI_VSYNC
    GPIOB->AFR[ 0 ] |=  0xD0000000;

    GPIOB->AFR[ 1 ] &= ~0x0000000F; // PB8  - DCMI_D6
    GPIOB->AFR[ 1 ] |=  0x0000000D;

    GPIOB->AFR[ 1 ] &= ~0x000000F0; // PB9  - DCMI_D7
    GPIOB->AFR[ 1 ] |=  0x000000D0;



    // Mode                 : Alternate function
    GPIOC->MODER   &= ~(GPIO_MODER_MODER6   | GPIO_MODER_MODER7   | GPIO_MODER_MODER8   | GPIO_MODER_MODER9   | GPIO_MODER_MODER10   | GPIO_MODER_MODER12  );
    GPIOC->MODER   |=  (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER12_1);

    // Alternate function   :         
    
    GPIOC->AFR[ 0 ] &= ~0x0F000000; // PC6  - DCMI_D0
    GPIOC->AFR[ 0 ] |=  0x0D000000;

    GPIOC->AFR[ 0 ] &= ~0xF0000000; // PC7  - DCMI_D1
    GPIOC->AFR[ 0 ] |=  0xD0000000;

    GPIOC->AFR[ 1 ] &= ~0x0000000F; // PC8  - DCMI_D2
    GPIOC->AFR[ 1 ] |=  0x0000000D;

    GPIOC->AFR[ 1 ] &= ~0x000000F0; // PC9  - DCMI_D3
    GPIOC->AFR[ 1 ] |=  0x000000D0;

    GPIOC->AFR[ 1 ] &= ~0x00000F00; // PC10 - DCMI_D8
    GPIOC->AFR[ 1 ] |=  0x00000D00;

    GPIOC->AFR[ 1 ] &= ~0x000F0000; // PC12 - DCMI_D9
    GPIOC->AFR[ 1 ] |=  0x000D0000;



                
    // Alternate function   : 
    //
    //  - PD0:  FMC_D2  - VERY HIGH SPEED
    //  - PD1:  FMC_D3  - VERY HIGH SPEED
    //  - PD3:  DCMI_D5 - LOW SPEED
    
    //  - PD8:  FMC_D13 - VERY HIGH SPEED
    //  - PD9:  FMC_D14 - VERY HIGH SPEED
    //  - PD10: FMC_D15 - VERY HIGH SPEED
    //  - PD14: FMC_D0  - VERY HIGH SPEED
    //  - PD15: FMC_D1  - VERY HIGH SPEED
    GPIOD->MODER    =   GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER3_1;
    GPIOD->MODER   |=   GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
           
    GPIOD->AFR[ 0 ] =   0x0000D0CC;    
    GPIOD->AFR[ 1 ] =   0xCC000CCC;
    
    GPIOD->OSPEEDR  =   0xF03F000F;
                    

    
    
    // Alternate function   :
    //
    //  - PE7:  FMC_D4  - VERY HIGH SPEED
    //  - PE8:  FMC_D5  - VERY HIGH SPEED
    //  - PE9:  FMC_D6  - VERY HIGH SPEED
    //  - PE10: FMC_D7  - VERY HIGH SPEED
    //  - PE11: FMC_D8  - VERY HIGH SPEED
    //  - PE12: FMC_D9  - VERY HIGH SPEED
    //  - PE13: FMC_D10 - VERY HIGH SPEED
    //  - PE14: FMC_D11 - VERY HIGH SPEED
    //  - PE15: FMC_D12 - VERY HIGH SPEED
    
    GPIOE->MODER    = GPIO_MODER_MODER7_1;    
    GPIOE->MODER   |= GPIO_MODER_MODER8_1  | GPIO_MODER_MODER9_1  | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;    
    GPIOE->MODER   |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
                
    GPIOE->AFR[ 0 ] = 0xC0000000;
    GPIOE->AFR[ 1 ] = 0xCCCCCCCC;               
    
    GPIOE->OSPEEDR  = 0xFFFFC000;
    


    // Mode     : Alternate function    
    
    //  - PF0:  FMC_A0      - VERY HIGH SPEED
    //  - PF1:  FMC_A1      - VERY HIGH SPEED
    //  - PF2:  FMC_A2      - VERY HIGH SPEED
    //  - PF3:  FMC_A3      - VERY HIGH SPEED
    //  - PF4:  FMC_A4      - VERY HIGH SPEED
    //  - PF5:  FMC_A5      - VERY HIGH SPEED
    //  - PF11: FMC_SDNRAS  - VERY HIGH SPEED
    //  - PF12: FMC_A6      - VERY HIGH SPEED
    //  - PF13: FMC_A7      - VERY HIGH SPEED
    //  - PF14: FMC_A8      - VERY HIGH SPEED
    //  - PF15: FMC_A9      - VERY HIGH SPEED
    
    GPIOF->MODER   =  0x00000000;
    GPIOF->MODER  |=  GPIO_MODER_MODER0_1  | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1  | GPIO_MODER_MODER3_1;        
    GPIOF->MODER  |=  GPIO_MODER_MODER4_1  | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1;    
    GPIOF->MODER  |=  GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1| GPIO_MODER_MODER15_1;    

    GPIOF->AFR[ 0 ]=  0x00CCCCCC;
    GPIOF->AFR[ 1 ]=  0xCCCCC000;
    
    GPIOF->OSPEEDR =  0xFFC00FFF;
    
    
    
    
    
    
    // Alternate functions  : 
    //
    //  - PG0:  FMC_A10    
    //  - PG1:  FMC_A11
    //  - PG2:  FMC_A12             
    //  - PG8:  FMC_SDCLK        
    //  - PG15: FMC_SDNCAS
    GPIOG->MODER    = GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER15_1;

    GPIOG->AFR[ 0 ] = 0x00000CCC;    
    GPIOG->AFR[ 1 ] = 0xC000000C;

    // Speed: Very high
    GPIOG->OSPEEDR  = 0xC003003F;
            
    
    
    // Mode                 : Alternate function
    GPIOH->MODER    &= ~(GPIO_MODER_MODER5   | GPIO_MODER_MODER6   | GPIO_MODER_MODER7   | GPIO_MODER_MODER14);
    GPIOH->MODER    |=  (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER14_1);

    // Alternate function   : 
    GPIOH->AFR[ 0 ] &= ~0x00F00000; // PH5  - FMC_SDNWE
    GPIOH->AFR[ 0 ] |=  0x00C00000;
    
    // Alternate function   : 
    GPIOH->AFR[ 0 ] &= ~0x0F000000; // PH6  - FMC_SDNE1
    GPIOH->AFR[ 0 ] |=  0x0C000000;
    
        // Alternate function   : 
    GPIOH->AFR[ 0 ] &= ~0xF0000000; // PH7  - FMC_SDCKE1
    GPIOH->AFR[ 0 ] |=  0xC0000000;
    
    // Alternate function   :     
    GPIOH->AFR[ 1 ] &= ~0x0F000000; // PH14 - DCMI_D4
    GPIOH->AFR[ 1 ] |=  0x0D000000;
    
    GPIOH->OSPEEDR  = 0xFFFFFFFF;
    
    
/*    
    // Mode                 : Alternate function
    GPIOI->MODER    &= ~(GPIO_MODER_MODER4   | GPIO_MODER_MODER5   );
    GPIOI->MODER    |=  (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 );
    
    // Alternate function   : 
    GPIOI->AFR[ 0 ] &= ~0x000F0000; // PI4  - FMC_NBL2
    GPIOI->AFR[ 0 ] |=  0x000C0000;
    
    // Alternate function   : 
    GPIOI->AFR[ 0 ] &= ~0x00F00000; // PI5  - FMC_NBL3
    GPIOI->AFR[ 0 ] |=  0x00C00000;
*/
    
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
    FMC_Bank5_6->SDCR[0] |=  FMC_SDCR1_RPIPE_0;                 // One HCLK clock cycle delay
    FMC_Bank5_6->SDCR[0] |=  FMC_SDCR1_RBURST;                  // Single read requests are always managed as bursts
    FMC_Bank5_6->SDCR[0] |=  FMC_SDCR1_SDCLK_1;                 // SDCLK period = 2 x HCLK  108MHz

    // Bank_2
    FMC_Bank5_6->SDCR[1]  =  0x00000000;
    FMC_Bank5_6->SDCR[1] |=  FMC_SDCR2_NB;                      // Number of internal banks     : 4
    FMC_Bank5_6->SDCR[1] |=  FMC_SDCR2_CAS_1;                   // CAS                          : 2 cycles
    FMC_Bank5_6->SDCR[1] |=  FMC_SDCR2_MWID_0;                  // Memory data width            : 16bit        
    FMC_Bank5_6->SDCR[1] |=  FMC_SDCR2_NR_0;                    // Number of row address bit    : 12bit
    FMC_Bank5_6->SDCR[1] &= ~FMC_SDCR2_NC;                      // Number of column address bit :  8bit

    

    FMC_Bank5_6->SDTR[0] = 0;
    FMC_Bank5_6->SDTR[1] = 0;
    
    
    // Trcd - RAS to CAS delay                        
    // 20ns for 133MHz(-7)
    // 20ns / 10ns ~= 2
    FMC_Bank5_6->SDTR[1] &= ~FMC_SDTR2_TRCD;
    FMC_Bank5_6->SDTR[1] |=  (2 - 1) << FMC_SDTR2_TRCD_Pos;
    
    
    // Trp - time row precharge delay.
    // 20ns for 133MHz(-7)
    // 20ns / 10ns ~= 2
    FMC_Bank5_6->SDTR[0] &= ~FMC_SDTR1_TRP;
    FMC_Bank5_6->SDTR[0] |=  FMC_SDTR1_TRP_1;   // 0: 1 cycle
                                                // 1: 2 cycle
    
    // Twr - Recovery delay between a Write and a Precharge    
    FMC_Bank5_6->SDTR[1] &= ~FMC_SDTR2_TWR;
    FMC_Bank5_6->SDTR[1] |=  (2 - 1) << FMC_SDTR2_TWR_Pos;  // 0: 1 cycle
                                                            // 1: 2 cycle
            
    // Trc - time row cycle     delay
    // 67.5ns for 133MHz(-7)
    // 67.5 / 10ns  = ~7 cycles
    FMC_Bank5_6->SDTR[0] &= ~FMC_SDTR1_TRC;
    FMC_Bank5_6->SDTR[0] |=  (7 - 1) << FMC_SDTR1_TRC_Pos;
    
    
    // Tras - row active time
    // 45ns
    // 45ns / 10ns  = ~5 cycles
    FMC_Bank5_6->SDTR[1] &= ~FMC_SDTR2_TRAS;
    FMC_Bank5_6->SDTR[1] |= (5 - 1) << FMC_SDTR2_TRAS_Pos;
    
    
    // Txsr - Exit Self-refresh delay
    // 7ns  / 10ns
    FMC_Bank5_6->SDTR[1] &= ~FMC_SDTR2_TXSR;
    //FMC_Bank5_6->SDTR[1] |= 7 << FMC_SDTR2_TXSR_Pos;
   
    // Load Mode Register to Active
    // 10ns / 10ns = 1
    FMC_Bank5_6->SDTR[1] &= ~FMC_SDTR2_TMRD;
    //FMC_Bank5_6->SDTR[1] |=  2 << FMC_SDTR2_TMRD_Pos;


       


    // 3. Set MODE bits to ‘001’ and configure the Target Bank bits (CTB1 and/or CTB2) in the
    // FMC_SDCMR register to start delivering the clock to the memory (SDCKE is driven high).
    // Target bank 1
    //FMC_Bank5_6->SDCMR   &= ~( FMC_SDCMR_MODE | FMC_SDCMR_CTB1 | FMC_SDCMR_CTB2);
    FMC_Bank5_6->SDCMR   |=  FMC_SDCMR_MODE_0 | FMC_SDCMR_CTB2;
    
    
    // 4. Wait during the prescribed delay period. Typical delay is around 100 μs (refer to the
    // SDRAM datasheet for the required delay after power-up).
    // IC42S16400: Maintain stable power, stable clock , and NOP input conditions for a minimum of 200us    
    for(int i = 0;i < 25000;i++)    
        for(int j = 0;j < 300;j++);
    
    
    
    // 5. Set MODE bits to ‘010’ and configure the Target Bank bits (CTB1 and/or CTB2) in the
    // FMC_SDCMR register to issue a “Precharge All” command.
    //FMC_Bank5_6->SDCMR   &= ~( FMC_SDCMR_MODE | FMC_SDCMR_CTB1 | FMC_SDCMR_CTB2);
    FMC_Bank5_6->SDCMR   =  FMC_SDCMR_MODE_1 | FMC_SDCMR_CTB2;
    
    for(int i = 0;i < 25000;i++);
    
    // 6. Set MODE bits to ‘011’, and configure the Target Bank bits (CTB1 and/or CTB2) as well
    // as the number of consecutive Auto-refresh commands (NRFS) in the FMC_SDCMR
    // register. Refer to the SDRAM datasheet for the number of Auto-refresh commands that
    // should be issued. Typical number is 8
    //FMC_Bank5_6->SDCMR   &= ~( FMC_SDCMR_MODE | FMC_SDCMR_CTB1 | FMC_SDCMR_CTB2 | FMC_SDCMR_NRFS);
    //FMC_Bank5_6->SDCMR   |=   FMC_SDCMR_MODE_0 | FMC_SDCMR_MODE_1 | FMC_SDCMR_CTB2 | FMC_SDCMR_NRFS_3;        
    FMC_Bank5_6->SDCMR      =(8 << 5) /* NRFS */ | FMC_SDCMR_CTB2 | (3); // Autorefresh
    
    
    
    // 7. Configure the MRD field set the MODE bits to '100',
    // and configure the Target Bank bits (CTB1 and/or CTB2) in the FMC_SDCMR register
    // to issue a "Load Mode Register"
            
    // - the CAS latency must be selected   : 2
    // - burst length must be selected to   : 1
        
    
    FMC_Bank5_6->SDCMR   = ((2 << 4) << 9) | FMC_SDCMR_CTB2 | 4;
           
    // 8. Program the refresh rate in the FMC_SDRTR register
    FMC_Bank5_6->SDRTR   = 1667;
    
    
    
    
    
    
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
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 216;
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