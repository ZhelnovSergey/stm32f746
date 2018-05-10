#include "stm32f7xx_hal.h"
#include "sdram.h"

void sdram_init(void)
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