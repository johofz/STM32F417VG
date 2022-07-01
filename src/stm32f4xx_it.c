#include "stm32f4xx_it.h"

void Default_Handler(void)
{
	while (1);
}

void NMI_Handler(void)
{
 	Default_Handler();
}

void HardFault_Handler(void)
{
	Default_Handler();
}

void MemManage_Handler(void)
{
	Default_Handler();
}

void BusFault_Handler(void)
{
	Default_Handler();
}

void UsageFault_Handler(void)
{
	Default_Handler();
}

void SVC_Handler(void)
{

}

void DebugMon_Handler(void)
{

}

void PendSV_Handler(void)
{

}

void SysTick_Handler(void)
{

}