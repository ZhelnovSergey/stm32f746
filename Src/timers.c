#include "stm32f7xx_hal.h"




/*
uint32_t    dwCounter = 0;
void TIM1_CC_IRQHandler(void)
{
    //if (TIM1->SR & TIM_SR_CC1IF)
    //{
        if( dwCounter < sizeof(g_gpio_buffer) )
            g_gpio_buffer[ dwCounter++ ] = (uint8_t)GPIOF->IDR;       
    //}
}
*/

void tim1_ch2_input_capture_init(void)
{
    NVIC_EnableIRQ( TIM1_CC_IRQn );
    
    // In frequency: Unknown    
    RCC->APB2ENR    |=  RCC_APB2ENR_TIM1EN;

    TIM1->CCMR1     &= ~TIM_CCMR1_CC1S;
    TIM1->CCMR1     |=  TIM_CCMR1_CC1S_1;    
    
    // Filter
    //TIM1->CCMR1     &= ~TIM_CCMR1_IC1F;
    //TIM1->CCMR1     |=  6 << TIM_CCMR1_IC1F_Pos; // 13

    TIM1->CCER      &= ~TIM_CCER_CC1P;      // Rising 
    TIM1->CCER      &= ~TIM_CCER_CC1NP;
    
    TIM1->CCER      |=  TIM_CCER_CC1E;
    
    //TIM1->DIER      |=  TIM_DIER_CC1DE;
    TIM1->DIER      |= TIM_DIER_CC1IE;  
    
    TIM1->CR1       |=  TIM_CR1_CEN;
}


void tim4_ch4_pwm_init(uint16_t wFirstHalf, uint16_t wSecondHalf)
{
    // In frequency: Unknown    
    RCC->APB1ENR    |= RCC_APB1ENR_TIM4EN;

    TIM4->CCMR2     |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;     // PWM                          : [ mode1   ]
    TIM4->CCMR2     |= TIM_CCMR2_OC4PE;                         // Preload register on CCR4     : [ enable  ]        

    TIM4->CCER      |= TIM_CCER_CC4P;                           // OC4 active low (first half)  : [ enable  ]
    TIM4->CCER      |= TIM_CCER_CC4E;                           // Output                       : [ enable  ]

    TIM4->CR1       |= TIM_CR1_ARPE;                            // Auto-reload (ARR) preload    : [ enable  ]
    TIM4->PSC       = 0;                                        // Frequency                    : [ 266MHz  ]
    TIM4->ARR       = wFirstHalf + wSecondHalf;
    TIM4->CCR4      = wFirstHalf;
    
    TIM4->EGR       |= TIM_EGR_UG; // Обновить данные в регистрах
    TIM4->SR        &=~TIM_SR_UIF; // Сбросить бит наступления переполнения
}