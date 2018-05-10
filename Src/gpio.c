#include "stm32f7xx_hal.h"
#include "gpio.h"

void gpio_init(void)
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
    //  - PE0:  FMC_NBL0
    //  - PE1:  FMC_NBL1
    //  - PE7:  FMC_D4
    //  - PE8:  FMC_D5
    //  - PE9:  FMC_D6
    //  - PE10: FMC_D7
    //  - PE11: FMC_D8
    //  - PE12: FMC_D9
    //  - PE13: FMC_D10
    //  - PE14: FMC_D11
    //  - PE15: FMC_D12
    
    GPIOE->MODER    = GPIO_MODER_MODER0_1  | GPIO_MODER_MODER1_1  | GPIO_MODER_MODER7_1;    
    GPIOE->MODER   |= GPIO_MODER_MODER8_1  | GPIO_MODER_MODER9_1  | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;    
    GPIOE->MODER   |= GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
        
    
    GPIOE->AFR[ 0 ] = 0xC00000CC;
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
    //  - PG4:  FMC_A14     [BA0]
    //  - PG5:  FMC_A15     [BA1]    
    //  - PG8:  FMC_SDCLK        
    //  - PG15: FMC_SDNCAS
    GPIOG->MODER    = GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER15_1;

    GPIOG->AFR[ 0 ] = 0x00CC00CC;
    GPIOG->AFR[ 1 ] = 0xC000000C;

    // Speed: Very high
    GPIOG->OSPEEDR  = 0xC0030F0F;
            
    
    
    // Mode                 : Alternate function    
    GPIOH->MODER    = GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER14_1;

    // Alternate function   : 
    //  - PH5:  FMC_SDNWE        
    //  - PH6:  FMC_SDNE1
    //  - PH7:  FMC_SDCKE1
    //  - PH14: DCMI_D4
    GPIOH->AFR[ 0 ] = 0xCCC00000;                       
    GPIOH->AFR[ 1 ] = 0x0D000000;
    
    GPIOH->OSPEEDR  = 0x0000FC00;
    
    
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

