;/**************************************************************************//**
; * @file     startup_NUC1262.s
; * @version  V3.00
; * @brief    NUC1262 Series Startup Source File for IAR Platform
; *
; * @copyright SPDX-License-Identifier: Apache-2.0
; * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
; ******************************************************************************/

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


    MODULE  ?cstartup

    ;; Forward declaration of sections.
    SECTION CSTACK:DATA:NOROOT(3) ;; 8 bytes alignment

    SECTION .intvec:CODE:NOROOT(2);; 4 bytes alignment

    EXTERN  SystemInit
    EXTERN  ProcessHardFault
    EXTERN  __iar_program_start
    PUBLIC  __vector_table
    PUBLIC  __Vectors

    DATA
__Vectors    
__vector_table
    DCD     sfe(CSTACK)
    DCD     Reset_Handler

    DCD     NMI_Handler
    DCD     HardFault_Handler
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     0
    DCD     SVC_Handler
    DCD     0
    DCD     0
    DCD     PendSV_Handler
    DCD     SysTick_Handler

    ; External Interrupts
    DCD     BOD_IRQHandler              ; Brownout low voltage detected interrupt                 
    DCD     WDT_IRQHandler              ; Watch Dog Timer interrupt                              
    DCD     EINT024_IRQHandler          ; External interrupt from INT0/2/4 pin               
    DCD     EINT135_IRQHandler          ; External interrupt from INT1/3/5 pin            
    DCD     GPAB_IRQHandler             ; External interrupt from PA/PB   
    DCD     GPCDF_IRQHandler            ; External interrupt from PC/PD/PF
    DCD     BPWM0_IRQHandler            ; BPWM0 interrupt                                 
    DCD     BPWM1_IRQHandler            ; BPWM1 interrupt                                 
    DCD     TMR0_IRQHandler             ; Timer 0 interrupt                                      
    DCD     TMR1_IRQHandler             ; Timer 1 interrupt                                      
    DCD     TMR2_IRQHandler             ; Timer 2 interrupt                                      
    DCD     TMR3_IRQHandler             ; Timer 3 interrupt                                      
    DCD     UART0_IRQHandler            ; UART0 interrupt                                        
    DCD     UART1_IRQHandler            ; UART1 interrupt                                        
    DCD     SPI0_IRQHandler             ; SPI0 interrupt                                         
    DCD     SPI1_IRQHandler             ; SPI1 interrupt                                         
    DCD     Default_Handler             ; Reserved                                         
    DCD     Default_Handler             ; Reserved                                         
    DCD     I2C0_IRQHandler             ; I2C0 interrupt                                         
    DCD     I2C1_IRQHandler             ; I2C1 interrupt                                        
    DCD     BPWM2_IRQHandler            ; BPWM2 interrupt        
    DCD     BPWM3_IRQHandler            ; BPWM3 interrupt        
    DCD     Default_Handler             ; Reserved
    DCD     USBD_IRQHandler             ; USBD interrupt
    DCD     Default_Handler             ; Reserved
    DCD     Default_Handler             ; Reserved
    DCD     PDMA_IRQHandler             ; PDMA interrupt
    DCD     Default_Handler             ; Reserved
    DCD     PWRWU_IRQHandler            ; Clock controller interrupt for chip wake up from power-down mode
    DCD     ADC_IRQHandler              ; ADC interrupt                                          
    DCD     CLKDIRC_IRQHandler          ; Clock fail detect and IRC TRIM interrupt
    DCD     Default_Handler              ; Reserved                              
    DCD     LLSI0_IRQHandler            ; LLSI0 interrupt
    DCD     LLSI1_IRQHandler            ; LLSI1 interrupt
    DCD     LLSI2_IRQHandler            ; LLSI2 interrupt
    DCD     LLSI3_IRQHandler            ; LLSI3 interrupt
    DCD     LLSI4_IRQHandler            ; LLSI4 interrupt
    DCD     LLSI5_IRQHandler            ; LLSI5 interrupt
    DCD     LLSI6_IRQHandler            ; LLSI6 interrupt
    DCD     LLSI7_IRQHandler            ; LLSI7 interrupt
    DCD     LLSI8_IRQHandler            ; LLSI8 interrupt
    DCD     LLSI9_IRQHandler            ; LLSI9 interrupt

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
    THUMB
    PUBWEAK Reset_Handler   
    SECTION .text:CODE:REORDER:NOROOT(2)   ; 4 bytes alignment
Reset_Handler
        
        LDR      R0, =SystemInit
        BLX      R0
        LDR      R0, =__iar_program_start
        BX       R0

    PUBWEAK HardFault_Handler
HardFault_Handler\

        MOV     R0, LR
        MRS     R1, MSP
        MRS     R2, PSP
        LDR     R3, =ProcessHardFault
        BLX     R3
        BX      R0

    PUBWEAK NMI_Handler       
    PUBWEAK SVC_Handler       
    PUBWEAK PendSV_Handler    
    PUBWEAK SysTick_Handler   
    PUBWEAK BOD_IRQHandler   
    PUBWEAK WDT_IRQHandler   
    PUBWEAK EINT024_IRQHandler 
    PUBWEAK EINT135_IRQHandler 
    PUBWEAK GPAB_IRQHandler  
    PUBWEAK GPCDF_IRQHandler
    PUBWEAK BPWM0_IRQHandler 
    PUBWEAK BPWM1_IRQHandler 
    PUBWEAK TMR0_IRQHandler 
    PUBWEAK TMR1_IRQHandler 
    PUBWEAK TMR2_IRQHandler 
    PUBWEAK TMR3_IRQHandler 
    PUBWEAK UART0_IRQHandler
    PUBWEAK UART1_IRQHandler
    PUBWEAK SPI0_IRQHandler
    PUBWEAK SPI1_IRQHandler
    PUBWEAK BPWM2_IRQHandler 
    PUBWEAK BPWM3_IRQHandler 
    PUBWEAK I2C0_IRQHandler
    PUBWEAK I2C1_IRQHandler
    PUBWEAK USBD_IRQHandler
    PUBWEAK PDMA_IRQHandler
    PUBWEAK PWRWU_IRQHandler  
    PUBWEAK ADC_IRQHandler
    PUBWEAK CLKDIRC_IRQHandler
    PUBWEAK LLSI0_IRQHandler
    PUBWEAK LLSI1_IRQHandler
    PUBWEAK LLSI2_IRQHandler
    PUBWEAK LLSI3_IRQHandler
    PUBWEAK LLSI4_IRQHandler
    PUBWEAK LLSI5_IRQHandler
    PUBWEAK LLSI6_IRQHandler
    PUBWEAK LLSI7_IRQHandler
    PUBWEAK LLSI8_IRQHandler
    PUBWEAK LLSI9_IRQHandler
    
    SECTION .text:CODE:REORDER:NOROOT(2)
    
;HardFault_Handler 
NMI_Handler       
SVC_Handler       
PendSV_Handler    
SysTick_Handler   
BOD_IRQHandler   
WDT_IRQHandler   
EINT024_IRQHandler 
EINT135_IRQHandler 
GPAB_IRQHandler  
GPCDF_IRQHandler
BPWM0_IRQHandler  
BPWM1_IRQHandler  
TMR0_IRQHandler  
TMR1_IRQHandler  
TMR2_IRQHandler  
TMR3_IRQHandler  
UART0_IRQHandler 
UART1_IRQHandler 
SPI0_IRQHandler  
SPI1_IRQHandler   
BPWM2_IRQHandler  
BPWM3_IRQHandler  
I2C0_IRQHandler
I2C1_IRQHandler
USBD_IRQHandler
PDMA_IRQHandler
PWRWU_IRQHandler
ADC_IRQHandler
CLKDIRC_IRQHandler
LLSI0_IRQHandler
LLSI1_IRQHandler
LLSI2_IRQHandler
LLSI3_IRQHandler
LLSI4_IRQHandler
LLSI5_IRQHandler
LLSI6_IRQHandler
LLSI7_IRQHandler
LLSI8_IRQHandler
LLSI9_IRQHandler
Default_Handler
    B Default_Handler         


;void SH_ICE(void)
    PUBLIC    SH_ICE
SH_ICE    
    CMP   R2,#0
    BEQ   SH_End
    STR   R0,[R2]   ; Save the return value to *pn32Out_R0

;void SH_End(void)
    PUBLIC    SH_End
SH_End
    MOVS   R0,#1    ; Set return value to 1
    BX     lr       ; Return


;int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
    PUBLIC    SH_DoCommand
SH_DoCommand
    BKPT   0xAB             ; This instruction will cause ICE trap or system HardFault
    B      SH_ICE
SH_HardFault                ; Captured by HardFault
    MOVS   R0,#0            ; Set return value to 0
    BX     lr               ; Return
    
    
    PUBLIC    __PC
__PC          
        MOV     r0, lr
        BLX     lr
            
    END

