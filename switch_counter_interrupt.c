#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"


#define RED_MASK 0x02
#define BLUE_MASK 0x04

#define		SW2					0x01 // PF0, 2^0 = 1
#define 	SW1					0x10 // PF4, 2^4 = 16
//*****************************************************************************
//
//!
//! Design a counter. The counter is incremented by 1 when SW1 (PF4) or SW2 (PF0) 
//! is pressed.
//
//*****************************************************************************

// global variable visible in Watch window of debugger
// increments at least once per button press
volatile unsigned long count = 0;

void
PortFunctionInit(void) {

    volatile uint32_t ui32Loop;

    // Enable the clock of the GPIO port that is used for the on-board LED and switch.
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

    //
    // Do a dummy read to insert a few cycles after enabling the peripheral.
    //
    ui32Loop = SYSCTL_RCGC2_R;

    // Unlock GPIO Port F
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R |= 0x01; // allow changes to PF0

    //
    // Enable pin PF2 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Enable pin PF0 for GPIOInput
    //

    //
    //First open the lock and select the bits we want to modify in the GPIO commit register.
    //
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;

    //
    //Now modify the configuration of the pins that we unlocked.
    //
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);

    //
    // Enable pin PF4 for GPIOInput
    //
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

    //
    // Enable pin PF1 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    // Set the direction of PF4 (SW1) and PF0 (SW2) as input by clearing the bit
    GPIO_PORTF_DIR_R &= ~0x11;

    // Enable PF4, and PF0 for digital function.
    GPIO_PORTF_DEN_R |= 0x11;

    //Enable pull-up on PF4 and PF0
    GPIO_PORTF_PUR_R |= 0x11;

}


//Globally enable interrupts 
void IntGlobalEnable(void) {
    __asm("    cpsie   i\n");
}

//Globally disable interrupts 
void IntGlobalDisable(void) {
    __asm("    cpsid   i\n");
}

void
Interrupt_Init(void) {

    NVIC_EN0_R |= 0x40000000; // enable interrupt 30 in NVIC address (GPIOF) 
    NVIC_PRI7_R &= 0x00E00000; // configure GPIOF interrupt priority as 0 global enable address
	
    GPIO_PORTF_IM_R |= 0x11; // arm interrupt on PF0 and PF4
    GPIO_PORTF_IS_R &= ~0x11; // PF0 and PF4 are edge-sensitive
    GPIO_PORTF_IBE_R |= 0x11; // PF0 and PF4 both edges trigger 
    //GPIO_PORTF_IEV_R &= ~0x11;  	// PF0 and PF4 falling edge event
    IntGlobalEnable(); // globally enable interrupt (Step 2, calls function)
}


//interrupt handler
void GPIOPortF_Handler(void)
{
	//switch debounce
	NVIC_EN0_R &= ~0x40000000; 
	SysCtlDelay(53333);	// Delay for a while
	NVIC_EN0_R |= 0x40000000; 
	
	//SW1 has action
	if(GPIO_PORTF_RIS_R&0x10)
	{
		// acknowledge flag for PF4
		GPIO_PORTF_ICR_R |= 0x10; 
		
		//SW1 is pressed
		if(((GPIO_PORTF_DATA_R&0x10)==0x00) && (GPIO_PORTF_DATA_R&0x00)!=0x00) 
		{
			//counter imcremented by 1
			count++;
			count = count & 3;
		}
	}
	
	//SW2 has action
  if(GPIO_PORTF_RIS_R&0x01)
	{
		// acknowledge flag for PF0
		GPIO_PORTF_ICR_R |= 0x01; 
		
		if(((GPIO_PORTF_DATA_R&0x01)==0x00) && (GPIO_PORTF_DATA_R&0x10)!=0x00) 
		{
			//counter imcremented by 1
			count--;
			count = count & 3;
		}
	}

}


int main(void) {

    //initialize the GPIO ports
    PortFunctionInit();
    //configure the GPIOF interrupt
    Interrupt_Init();

    //
    // Loop forever.
    //
    while (1) {

        switch (count) {

            case 0:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
                break;
            case 1:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
                break;
            case 2:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_MASK);
                break;
            case 3:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_MASK);
                break;
        }

    }
}
