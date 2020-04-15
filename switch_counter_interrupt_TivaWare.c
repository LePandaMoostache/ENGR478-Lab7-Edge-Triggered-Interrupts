#include <stdint.h>
#include <stdbool.h>
#include "switch_counter_interrupt_TivaWare.h"
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
PortFunctionInit(void)
{
    //
    // Enable Peripheral Clocks 
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

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
		
		    //Enable pull-up on PF4 and PF0
    GPIO_PORTF_PUR_R |= 0x11;
}


void
Interrupt_Init(void)
{
  IntEnable(INT_GPIOF);  							// enable interrupt 30 in NVIC (GPIOF)
	IntPrioritySet(INT_GPIOF, 0x00); 		// configure GPIOF interrupt priority as 0
	GPIO_PORTF_IM_R |= 0x11;   		// arm interrupt on PF0 and PF4
	GPIO_PORTF_IS_R &= ~0x11;     // PF0 and PF4 are edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;   	// PF0 and PF4 not both edges trigger 
  GPIO_PORTF_IEV_R &= ~0x11;  	// PF0 and PF4 falling edge event
	IntMasterEnable();       		// globally enable interrupt
}

//interrupt handler
void GPIOPortF_Handler(void) {
	
    // Rotary Counter
    //SW1 is pressed
    if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)==0x00) {
        // acknowledge flag for PF4
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
        //counter incremented by 1
        count++;
        count = count & 3;
    }
    //SW2 is pressed
    if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0)==0x00) {
        // acknowledge flag for PF0
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
        //counter decremented by 1
        count--;
        count = count & 3;
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
