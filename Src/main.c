#include "main.h"
#include "stm32f7xx_hal.h"

#include "dcmi.h"
#include "gpio.h"

#include "sdram.h"
#include "timers.h"



void SystemClock_Config         (void);

void dma_dcmi_init              (void);

void usart_init                 (void);

 
uint8_t     g_frame_buffer      [200][1504]; //[200][1504];

uint32_t    g_frame_cnt         = 0;






unsigned int*   pData = 0xD0000000;
unsigned int    Data  = 0;



int main(void)
{
    HAL_Init            ();
    SystemClock_Config  ();        


    // В таком порядке
    gpio_init           ();
    
    sdram_init          ();
    
    dma_dcmi_init       ();
    
    dcmi_fcb_ex48ep_init();    
    
    
/*  
    // SDRAM TEST
    pData[0] = 0x12345678;
    Data     = pData[0];
    
    for( int i = 0;i < 100;i++ )
    {
        pData[ i ] = i;
        Data = pData[ i ];
    }
*/
                
    while(1);
    
    
    
    
    
    
    

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
    RCC->AHB1ENR        |= RCC_AHB1ENR_DMA2EN;

    
    // Setup DMA2 Stream7 for DCMI CHANNEL1
    DMA2_Stream7->CR    &= ~DMA_SxCR_EN;    // Выключить DMA
    while (DMA2_Stream7->CR & DMA_SxCR_EN); // Убедиться что он выключен

    DMA2_Stream7->PAR    = (uint32_t) &DCMI->DR;                    // Source    
  //DMA2_Stream7->M0AR   = (uint32_t) &g_frame_buffer[g_frame_cnt];
    DMA2_Stream7->M0AR   = 0xD0000000;                              // Destination

    DMA2_Stream7->CR    |= DMA_CHANNEL_1;
    DMA2_Stream7->NDTR   = 65535;
    DMA2_Stream7->CR    |= DMA_NORMAL;  
    
    DMA2_Stream7->CR    |= DMA_PRIORITY_LOW;
    DMA2_Stream7->CR    |= DMA_PERIPH_TO_MEMORY;                       // Memory to peripherial
    DMA2_Stream7->CR    |= DMA_PINC_DISABLE;                           // Peripheral increment mode disable
    DMA2_Stream7->CR    |= DMA_MINC_ENABLE;                            // Memory increment mode enable        
    DMA2_Stream7->CR    |= DMA_PBURST_INC4; /* DMA_PBURST_INC16 */
    
    
    DMA2_Stream7->CR    |= DMA_PDATAALIGN_WORD;
    DMA2_Stream7->CR    |= DMA_MDATAALIGN_WORD;

    DMA2_Stream7->FCR   |= DMA_SxFCR_DMDIS;                           // Disable direct mode

    DMA2_Stream7->CR    |= DMA_SxCR_EN;
}



//Note:     The DMA controller and all DCMI configuration registers should be 
  //          programmed correctly before enabling this bit.




// SDRAM: IC42S16400
//
// - Programmable CAS latency (2 and 3)









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