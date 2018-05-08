#include "main.h"
#include "dcmi.h"


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
    
  