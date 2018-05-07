#include "main.h"
#include "stm32f7xx_hal.h"

#include "timers.h"

void SystemClock_Config         (void);
void ports_init                 (void);

void dma_dcmi_init              (void);

void dcmi_ov7725_init           (void);
void dcmi_fcb_ex48ep_init       (void);

void fmc_sdram_init             (void);




void usart_init                 (void);

 
uint8_t     g_frame_buffer      [200][1504]; //[200][1504];

uint32_t    g_frame_cnt         = 0;


#define TMRD(x) (x << 0)
#define TXSR(x) (x << 4)
#define TRAS(x) (x << 8)
#define TRC(x)  (x << 12)
#define TWR(x)  (x << 16)
#define TRP(x)  (x << 20)
#define TRCD(x) (x << 24)


// К этой плате нужно подключать USB питание 


int main(void)
{
    HAL_Init();

    SystemClock_Config();        


    
    ports_init      ();
    fmc_sdram_init  ();
    
    unsigned int*   pData = 0xD0000000;
    unsigned int    Data  = 0;
    
    *pData = 1;
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




    // Mode                 : Alternate function
    GPIOD->MODER    &= ~(GPIO_MODER_MODER0   | GPIO_MODER_MODER1   | GPIO_MODER_MODER3   | GPIO_MODER_MODER8   | GPIO_MODER_MODER9   | GPIO_MODER_MODER10   | GPIO_MODER_MODER14   | GPIO_MODER_MODER15);
    GPIOD->MODER    |=  (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER3_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);
        

    // Alternate function   : 
    GPIOD->AFR[ 0 ] &= ~0x0000000F; // PD0  - FMC_D2
    GPIOD->AFR[ 0 ] |=  0x0000000C;
    
    // Alternate function   : 
    GPIOD->AFR[ 0 ] &= ~0x000000F0; // PD1  - FMC_D3
    GPIOD->AFR[ 0 ] |=  0x000000C0;
    
    // Alternate function   : 
    GPIOD->AFR[ 0 ] &= ~0x0000F000; // PD3  - DCMI_D5
    GPIOD->AFR[ 0 ] |=  0x0000D000;

    // Alternate function   : 
    GPIOD->AFR[ 1 ] &= ~0x0000000F; // PD8  - FMC_D13
    GPIOD->AFR[ 1 ] |=  0x0000000C;
    
    // Alternate function   : 
    GPIOD->AFR[ 1 ] &= ~0x000000F0; // PD9  - FMC_D14
    GPIOD->AFR[ 1 ] |=  0x000000C0;
    
    // Alternate function   : 
    GPIOD->AFR[ 1 ] &= ~0x00000F00; // PD10 - FMC_D15
    GPIOD->AFR[ 1 ] |=  0x00000C00;    
    
    // Alternate function   : 
    GPIOD->AFR[ 1 ] &= ~0x0F000000; // PD14 - FMC_D0
    GPIOD->AFR[ 1 ] |=  0x0C000000;
    
    // Alternate function   : 
    GPIOD->AFR[ 1 ] &= ~0xF0000000; // PD15 - FMC_D1
    GPIOD->AFR[ 1 ] |=  0xC0000000;
    

    
    
    // Mode                 : Alternate function
    GPIOE->MODER    &= ~(/*GPIO_MODER_MODER0   | GPIO_MODER_MODER1   |*/ GPIO_MODER_MODER7   |  GPIO_MODER_MODER8   | GPIO_MODER_MODER9  | GPIO_MODER_MODER10  | GPIO_MODER_MODER11   | GPIO_MODER_MODER12     );
    GPIOE->MODER    |=  (/*GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 |*/ GPIO_MODER_MODER7_1 |  GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1| GPIO_MODER_MODER10_1| GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1 );
    
    GPIOE->MODER    &= ~(GPIO_MODER_MODER13  | GPIO_MODER_MODER14  | GPIO_MODER_MODER15  );
    GPIOE->MODER    |=  (GPIO_MODER_MODER13_1| GPIO_MODER_MODER14_1| GPIO_MODER_MODER15_1);
/*
    // Alternate function   : 
    GPIOE->AFR[ 0 ] &= ~0x0000000F;     // PE0  - FMC_NBL0
    GPIOE->AFR[ 0 ] |=  0x0000000C;
    
    // Alternate function   : 
    GPIOE->AFR[ 0 ] &= ~0x000000F0;     // PE1  - FMC_NBL1
    GPIOE->AFR[ 0 ] |=  0x000000C0;
*/    
    // Alternate function   : 
    GPIOE->AFR[ 0 ] &= ~0xF0000000;     // PE7  - FMC_D4
    GPIOE->AFR[ 0 ] |=  0xC0000000;
        
    // Alternate function   : 
    GPIOE->AFR[ 1 ] &= ~0x0000000F;     // PE8  - FMC_D5
    GPIOE->AFR[ 1 ] |=  0x0000000C;
    
    // Alternate function   : 
    GPIOE->AFR[ 1 ] &= ~0x000000F0;     // PE9  - FMC_D6
    GPIOE->AFR[ 1 ] |=  0x000000C0;    
    
    // Alternate function   : 
    GPIOE->AFR[ 1 ] &= ~0x00000F00;     // PE10 - FMC_D7
    GPIOE->AFR[ 1 ] |=  0x00000C00;  
    
    // Alternate function   : 
    GPIOE->AFR[ 1 ] &= ~0x0000F000;     // PE11 - FMC_D8
    GPIOE->AFR[ 1 ] |=  0x0000C000;
    
    // Alternate function   : 
    GPIOE->AFR[ 1 ] &= ~0x000F0000;     // PE12 - FMC_D9
    GPIOE->AFR[ 1 ] |=  0x000C0000;
    
    // Alternate function   : 
    GPIOE->AFR[ 1 ] &= ~0x00F00000;     // PE13 - FMC_D10
    GPIOE->AFR[ 1 ] |=  0x00C00000;    
    
    // Alternate function   : 
    GPIOE->AFR[ 1 ] &= ~0x0F000000;     // PE14 - FMC_D11
    GPIOE->AFR[ 1 ] |=  0x0C000000;
    
    // Alternate function   : 
    GPIOE->AFR[ 1 ] &= ~0xF0000000;     // PE15 - FMC_D12
    GPIOE->AFR[ 1 ] |=  0xC0000000; 


    // Mode                 : Alternate function
    GPIOF->MODER   &= ~(GPIO_MODER_MODER0   | GPIO_MODER_MODER1   | GPIO_MODER_MODER2   | GPIO_MODER_MODER3   | GPIO_MODER_MODER4   | GPIO_MODER_MODER5   | GPIO_MODER_MODER11   | GPIO_MODER_MODER12   );
    GPIOF->MODER   |=  (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1 );
            
    GPIOF->MODER   &= ~(GPIO_MODER_MODER13  | GPIO_MODER_MODER14  | GPIO_MODER_MODER15  );
    GPIOF->MODER   |=  (GPIO_MODER_MODER13_1| GPIO_MODER_MODER14_1| GPIO_MODER_MODER15_1);                                    
            
    // Alternate function   : 
    GPIOF->AFR[ 0 ] &= ~0x0000000F; // PF0  - FMC_A0
    GPIOF->AFR[ 0 ] |=  0x0000000C;
    
    GPIOF->AFR[ 0 ] &= ~0x000000F0; // PF1  - FMC_A1
    GPIOF->AFR[ 0 ] |=  0x000000C0;
    
    GPIOF->AFR[ 0 ] &= ~0x00000F00; // PF2  - FMC_A2
    GPIOF->AFR[ 0 ] |=  0x00000C00;
    
    GPIOF->AFR[ 0 ] &= ~0x0000F000; // PF3  - FMC_A3
    GPIOF->AFR[ 0 ] |=  0x0000C000;        
    
    GPIOF->AFR[ 0 ] &= ~0x000F0000; // PF4  - FMC_A4
    GPIOF->AFR[ 0 ] |=  0x000C0000;
    
    GPIOF->AFR[ 0 ] &= ~0x00F00000; // PF5  - FMC_A5
    GPIOF->AFR[ 0 ] |=  0x00C00000;

    GPIOF->AFR[ 1 ] &= ~0x0000F000; // PF11 - FMC_SDNRAS
    GPIOF->AFR[ 1 ] |=  0x0000C000;
    
    GPIOF->AFR[ 1 ] &= ~0x000F0000; // PF12 - FMC_A6
    GPIOF->AFR[ 1 ] |=  0x000C0000;
    
    GPIOF->AFR[ 1 ] &= ~0x00F00000; // PF13 - FMC_A7
    GPIOF->AFR[ 1 ] |=  0x00C00000;    
    
    GPIOF->AFR[ 1 ] &= ~0x0F000000; // PF14 - FMC_A8
    GPIOF->AFR[ 1 ] |=  0x0C000000;
    
    GPIOF->AFR[ 1 ] &= ~0xF0000000; // PF15 - FMC_A9
    GPIOF->AFR[ 1 ] |=  0xC0000000;    
    
    
    
    
    
    
    
    
    
    // Mode                 : Alternate function
    GPIOG->MODER    &= ~(GPIO_MODER_MODER0   | GPIO_MODER_MODER1   | GPIO_MODER_MODER2   | GPIO_MODER_MODER8   | /*GPIO_MODER_MODER9   | GPIO_MODER_MODER14   |*/ GPIO_MODER_MODER15    );
    GPIOG->MODER    |=  (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER8_1 | /*GPIO_MODER_MODER9_1 | GPIO_MODER_MODER14_1 |*/ GPIO_MODER_MODER15_1  );
    
    // Alternate function   : 
    GPIOG->AFR[ 0 ] &= ~0x0000000F; // PG0  - FMC_A10
    GPIOG->AFR[ 0 ] |=  0x0000000C;
    
    // Alternate function   : 
    GPIOG->AFR[ 0 ] &= ~0x000000F0; // PG1  - FMC_A11
    GPIOG->AFR[ 0 ] |=  0x000000C0;
    
    // Alternate function   : 
    GPIOG->AFR[ 0 ] &= ~0x00000F00; // PG2  - FMC_A12
    GPIOG->AFR[ 0 ] |=  0x00000C00;
    
    // Alternate function   : 
    GPIOG->AFR[ 1 ] &= ~0x0000000F; // PG8  - FMC_SDCLK
    GPIOG->AFR[ 1 ] |=  0x0000000C;
    
    GPIOG->OSPEEDR  |=  0x00030000;
    
    
    
/*
    // Alternate function   : 
    GPIOG->AFR[ 1 ] &= ~0x000000F0; // PG9  - USART6_RX
    GPIOG->AFR[ 1 ] |=  0x00000080;
    
    // Alternate function   : 
    GPIOG->AFR[ 1 ] &= ~0x0F000000; // PG14 - USART6_TX
    GPIOG->AFR[ 1 ] |=  0x08000000;    
*/    
    // Alternate function   : 
    GPIOG->AFR[ 1 ] &= ~0xF0000000; // PG15 - FMC_SDNCAS
    GPIOG->AFR[ 1 ] |=  0xC0000000;  
    
    
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

void dcmi_ov7725_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;

    // Extended data mode               : 10bit data bus
    DCMI->CR &= ~(DCMI_CR_EDM_0 | DCMI_CR_EDM_1);
    DCMI->CR |= DCMI_CR_EDM_0;

    // Vertical synchronization polarity: indicates the level on the DCMI_VSYNC pin when the data are NOT VALID    
    DCMI->CR |= DCMI_CR_VSPOL; //High


    // Embedded synchronization select  : Hardware synchronization    
    DCMI->CR &= ~DCMI_CR_ESS;

    // Capture mode                     : Snapshot mode (single frame)
    DCMI->CR |= DCMI_CR_CM;

    // Pixel clock polarity             : Rising edge active
    DCMI->CR |= DCMI_CR_PCKPOL;

    //DCMI->ESUR |=  0x40101000;
    //DCMI->ESCR |=  0x40100040;

    // Capture                          : Enabled
    DCMI->CR |= DCMI_CR_CAPTURE;

    // Enable interface                 : On
    DCMI->CR |= DCMI_CR_ENABLE;


    //Note:     The DMA controller and all DCMI configuration registers should be 
    //          programmed correctly before enabling this bit.
}

void DCMI_IRQHandler(void)
{
    if( DCMI->RISR & DCMI_RISR_LINE_RIS )
    {        
        DCMI->ICR |= DCMI_ICR_LINE_ISC; 

        //g_frame_cnt++;
        //DMA2_Stream7->M0AR = (uint32_t) & g_frame_buffer[g_frame_cnt]; // Destination
        
        //DCMI->CR |= DCMI_CR_CAPTURE;


    }            
        
}

void dcmi_fcb_ex48ep_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;

    // Extended data mode               : 8bit data bus
    DCMI->CR &= ~(DCMI_CR_EDM_0 | DCMI_CR_EDM_1);    
    


    // Embedded synchronization select  : Embedded synchronization    
    DCMI->CR |= DCMI_CR_ESS;

    // Capture mode                     : Snapshot mode (single frame)
    DCMI->CR |= DCMI_CR_CM;

    // Pixel clock polarity             : Rising edge active
    DCMI->CR |= DCMI_CR_PCKPOL;

    // Mask
    DCMI->ESUR =  0xF0F0F0F0;
    
    // Code
    DCMI->ESCR =  0xFF908080;
    
    NVIC_EnableIRQ( DCMI_IRQn );
    DCMI->IER  =  DCMI_IER_LINE_IE; //DCMI_IER_LINE_IE;

    // Capture                          : Enabled
    DCMI->CR |= DCMI_CR_CAPTURE;

    // Enable interface                 : On
    DCMI->CR |= DCMI_CR_ENABLE;


    //Note:     The DMA controller and all DCMI configuration registers should be 
    //          programmed correctly before enabling this bit.
}



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
    FMC_Bank5_6->SDCR[0] |=  FMC_SDCR1_RBURST;                  // Single read requests are not managed as bursts
    FMC_Bank5_6->SDCR[0] |=  FMC_SDCR1_SDCLK_1;                 // SDCLK period = 2 x HCLK  108MHz  было вот так FMC_SDCR1_SDCLK_1

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
    // 20ns / 10ns ~= 2
    FMC_Bank5_6->SDTR[1] &= ~FMC_SDTR2_TRCD;
    FMC_Bank5_6->SDTR[1] |=  2 << FMC_SDTR2_TRCD_Pos;
    
    
    // Trp - time row precharge delay.
    // 20ns for 133MHz(-7)
    // 20ns / 10ns ~= 2
    FMC_Bank5_6->SDTR[0] &= ~FMC_SDTR1_TRP;
    FMC_Bank5_6->SDTR[0] |=  FMC_SDTR1_TRP_1;   // 0: 1 cycle
                                                // 1: 2 cycle
    
    // Twr - Recovery delay between a Write and a Precharge    
    FMC_Bank5_6->SDTR[1] &= ~FMC_SDTR2_TWR;
    FMC_Bank5_6->SDTR[1] |=  2 << FMC_SDTR2_TWR_Pos;   // 0: 1 cycle
                                                // 1: 2 cycle
            
    // Trc - time row cycle     delay
    // 67.5ns for 133MHz(-7)
    // 67.5 / 10ns  = ~7 cycles
    FMC_Bank5_6->SDTR[0] &= ~FMC_SDTR1_TRC;
    FMC_Bank5_6->SDTR[0] |=  (7) << FMC_SDTR1_TRC_Pos;
    
    
    // Tras - self refresh period
    // 64ns
    // 64ns / 10ns  = ~7 cycles
    FMC_Bank5_6->SDTR[1] &= ~FMC_SDTR2_TRAS;
    FMC_Bank5_6->SDTR[1] |= 4 << FMC_SDTR2_TRAS_Pos;
    
    
    // Txsr - Exit Self-refresh delay
    FMC_Bank5_6->SDTR[1] &= ~FMC_SDTR2_TXSR;
    FMC_Bank5_6->SDTR[1] |= 7 << FMC_SDTR2_TXSR_Pos;
   
    // Load Mode Register to Active
    FMC_Bank5_6->SDTR[1] &= ~FMC_SDTR2_TMRD;
    FMC_Bank5_6->SDTR[1] |=  2 << FMC_SDTR2_TMRD_Pos;

/*    
        FMC_Bank5_6->SDTR[0] = ((2-1)<<24)                  // 2 cycle TRCD (18.5-ns > 15-ns)
                         | ((2-1)<<20)                  // 2 cycle TRP (18.5-ns > 15-ns)
                         | ((2-1)<<16)                  // 2 cycle TWR
                         | ((7-1)<<12)                  // 7 cycle TRC (64.7-ns > 63-ns)
                         | ((5-1)<<8)                   // 5 cycle TRAS (46.2-ns > 42-ns)
                         | ((8-1)<<4)                   // 8 cycle TXSR (74-ns > 70-ns)
                         | ((2-1)<<0);                  // 2 cycle TMRD
*/   
    
    


    // 3. Set MODE bits to ‘001’ and configure the Target Bank bits (CTB1 and/or CTB2) in the
    // FMC_SDCMR register to start delivering the clock to the memory (SDCKE is driven high).
    // Target bank 1
    //FMC_Bank5_6->SDCMR   &= ~( FMC_SDCMR_MODE | FMC_SDCMR_CTB1 | FMC_SDCMR_CTB2);
    FMC_Bank5_6->SDCMR   |=  FMC_SDCMR_MODE_0 | FMC_SDCMR_CTB2;
    
    
    // 4. Wait during the prescribed delay period. Typical delay is around 100 μs (refer to the
    // SDRAM datasheet for the required delay after power-up).
    // IC42S16400: Maintain stable power, stable clock , and NOP input conditions for a minimum of 200us    
    for(int i = 0;i < 250;i++)    
        for(int j = 0;j < 300;j++);
    
    
    
    // 5. Set MODE bits to ‘010’ and configure the Target Bank bits (CTB1 and/or CTB2) in the
    // FMC_SDCMR register to issue a “Precharge All” command.
    //FMC_Bank5_6->SDCMR   &= ~( FMC_SDCMR_MODE | FMC_SDCMR_CTB1 | FMC_SDCMR_CTB2);
    FMC_Bank5_6->SDCMR   =  FMC_SDCMR_MODE_1 | FMC_SDCMR_CTB2;
    
    
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
            
    // - the CAS latency must be selected   : 3
    // - burst length must be selected to   : 1
        
    
    FMC_Bank5_6->SDCMR   = ((3 << 4) << 9) | FMC_SDCMR_CTB2 | 4;
           
    // 8. Program the refresh rate in the FMC_SDRTR register
    FMC_Bank5_6->SDRTR   = 2572;
    
    
    
    
    
    
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