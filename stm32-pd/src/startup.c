/*
 * startup.c — vektor interupsi dan reset handler untuk STM32F407VG.
 *
 * Ditulis dalam C, bukan assembly, supaya seluruh proyek bisa dibangun hanya
 * dengan arm-none-eabi-gcc tanpa CMSIS, HAL, atau berkas startup vendor.
 */

#include "types.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

int  main(void);
void DMA2_Stream0_IRQHandler(void);
void USART2_IRQHandler(void);

void Default_Handler(void)
{
    for (;;) { }
}

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;

    while (dst < &_edata) { *dst++ = *src++; }
    for (dst = &_sbss; dst < &_ebss; ) { *dst++ = 0; }

    main();
    for (;;) { }
}

#define WEAK_DEFAULT __attribute__((weak, alias("Default_Handler")))

void NMI_Handler(void)        WEAK_DEFAULT;
void HardFault_Handler(void)  WEAK_DEFAULT;
void MemManage_Handler(void)  WEAK_DEFAULT;
void BusFault_Handler(void)   WEAK_DEFAULT;
void UsageFault_Handler(void) WEAK_DEFAULT;
void SVC_Handler(void)        WEAK_DEFAULT;
void DebugMon_Handler(void)   WEAK_DEFAULT;
void PendSV_Handler(void)     WEAK_DEFAULT;
void SysTick_Handler(void)    WEAK_DEFAULT;

/* 16 vektor inti Cortex-M4 + 82 IRQ periferal F407.
 * Hanya dua IRQ yang dinyalakan firmware ini, jadi slot lain dibiarkan nol. */
__attribute__((section(".isr_vector"), used))
void (* const g_vectors[16 + 82])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,

    [16 + 38] = USART2_IRQHandler,
    [16 + 56] = DMA2_Stream0_IRQHandler,
};
