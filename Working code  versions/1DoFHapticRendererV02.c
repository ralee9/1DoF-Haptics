/******************************************************************************
Randy Lee (ral63@pitt.edu)
Avi Marcovici
Visualization and Image Analysis Laboratory
Department of Bioengineering
University of Pittsburgh

1 DoF Haptic Renderer -- ADuC7026 control software

Code version: v02

Description and notes:

Controlling a commercial speaker in force and position to create a haptic 
renderer. Virtual membranes of variable stiffness are created by combining
two functionalities: (1) a zero stiffness spring -- defies gravity by keeping
force on the speaker drum at zero; (2) an infinite stiffness spring -- virtual
wall to control both force and position. Infinite stiffness spring will also
enable position dependent calibration for force and current

-----------

Change Log:

v00 -- Initial version. Imported previous functions. SPI communication prototype
V01 -- Adding the counter and the flag to the main program.
V02 -- changing the "if" statment to "while" statment so the program wait until there is data on SPIRX to read it.


Last updated: 27 October 2015
******************************************************************************/

// Libraries ------------------------------------------------------------------
#include <stdio.h>
#include <aduc7026.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Global variables -----------------------------------------------------------

// Flags ----------------------------------------------------------------------
volatile signed int timer0_Flag = 0;
volatile signed int timer1_Flag = 0;
int flag = 0;
int counter = 0;
int message = 0;

// Function declarations ------------------------------------------------------
void General_IRQ_Handler(void);
void system_init(void);
void timer0_init(void);
void SPI_init(void);
void SPI_write(unsigned char data);
void ADCpoweron(int);

// Imported HHFM functions ----------------------------------------------------

// initialize 200 us period time keeping
void timer0_init(void);

// variable software delay
void delay_sw(int time);

// variable us delay using timer1
void delay_us(unsigned long time);

// variable ms delay using timer1
void delay_ms(unsigned long time);

// converts hex result from ADC conversion to decimal voltage
float ADCinput(int ADC_hex_result);

// converts desired decimal voltage to hex integer for DAC output
int outputDAC(float desired_voltage);

// wait function to limit activity to timer0 period; regulates 
// sampling/ actuation rate
void waitForRestOfPeriod(void);

// Read data from the Wixel and returns a integer
int SPI_read(void);


// Main program ---------------------------------------------------------------
int main(void)
{
    delay_sw(50000);	//Wait to avoid JTAG lock while prototyping
    
    /* Core clock and power state control */
    system_init();
    
    /* Power on ADC and DAC channels */
    ADCpoweron(20000);      // power on ADC
    
    // Enable timer0 and time1 interrupts for timing
    //IRQEN = 0x2004;             // enable timer0 & SPI interrupts
	  IRQEN = 0x04;								// enable timer0 interrupt
    IRQ = General_IRQ_Handler;  // specify interrupt handler
    
    //wait();
    
    SPI_init();         // set up SPI on GPIO P1.4 - 1.7
                        // SPI master mode, clock pulses on init xfer,
                        // transmit on write to TX register
                        // interrupt when TX empty
    
    // Main loop --------------------------------------------------------------
		while(1) {
      timer0_init();
				
				if (flag == 0)
				{ 
					SPI_write(0x37);
					
					//GP2DAT |= 0x10000;								  // turn on LED
					/*
					SPI_write(0x99);
					while((SPISTA & 0x1) == 0x1) {
						// wait while SPI is transmitting
					}
					*/
				}
			
				//delay_us(50);
				else if (flag == 1)	
				{
					SPI_write(0xAE);
					
					//GP2DAT ^= 0x10000;					// turn off LED
					/*
					SPI_write(0x11);
					while((SPISTA & 0x1) == 0x1) {
							// wait while SPI is transmitting
					}
					*/
				}
				
				if (flag == 0) flag = 1;
				else if (flag == 1) flag = 0;
				
			/*
      if (counter%1000 == 0)
			{
				GP2DAT ^= 0x10000;
			}
				
				if (flag == 0)
				{ 
					SPI_write(0x37);
					while((SPISTA & 0x1) == 0x1) {
							// wait while SPI is transmitting
					}
				}
			
				//delay_us(50);
				else if (flag == 1)	
				{
					SPI_write(0xAE);
					while((SPISTA & 0x1) == 0x1) {
            // wait while SPI transmitting
					}
				}
				
				counter = 0;

				if (flag == 0) flag = 1;
				else if (flag == 1) flag = 0;
			}
			*/
			//counter++;
			//message = SPI_read();         // reading data from the Wixel
		
				while (!((SPISTA & 0x8) == 0x8)) 
				{
					// wait while nothing in SPIRX 
				}
				
				message = SPI_read();         // reading data from the Wixel
				
				if (message == 0xFF && counter%1000 == 0)
				{ 
					//GP2DAT |= (1<<16);					  // turn on LED
					GP2DAT ^= 0x10000;
				}	
				
				if (message == 0x00)
				{
					GP2DAT ^= 0x20000;
					//GP2DAT = 0xFF000000;		            // turn off LED
				}	
				
			counter++;
			waitForRestOfPeriod(); 
    }
		
} // end of Main --------------------------------------------------------------



// Functions

int SPI_read (void)
{
	int message;
	message = SPIRX;
	
	return message;
}


void system_init(void)
{
  POWKEY1 = 0x01;					
 	POWCON = 0x00;					//Configures CPU Clock for 41.78MHz, CD=0
 	POWKEY2 = 0xF4;	

	PLLKEY1 = 0xAA;
	PLLCON = 0x01;
	PLLKEY2 = 0x55;
	
	REFCON = 0x01;					//Connect internal 2.5V reference on Vref pin
	DAC3CON = 0x12;					//Set DAC3 output to 0-V_ref(2.5V) range and turn DAC3 on

	
	GP2CON = 0x00;					//Set digital I/O port 2 as *GPIO* on pins P2.0-2.7
	GP2DAT = 0xFF000000;			//Set all pins on port 2 as *OUTPUT*, initialize LOW
	/*
    GP0CON = 0x00;					//Set digital I/O port 0 as GPIO on pins P0.0-0.7
	GP0DAT = 0x00;					//Set all I/O pins on port 0 as *INPUT*, initialize LOW
	
	GP2CON = 0x00;					//Set digital I/O port 2 as *GPIO* on pins P2.0-2.7
	GP2DAT = 0xFF000000;			//Set all pins on port 2 as *OUTPUT*, initialize LOW
	
	GP3CON = 0x00;					//Set digital I/O port 3 as GPIO on pins P3.0-3.7
	GP3DAT = 0x00;					//Set all I/O pins on port 3 as *INPUT*, initialize LOW
    */
}

/** ADCpoweron(): Power on ADC, software-timed wait **/
void ADCpoweron(int time)
{
	ADCCON = 0x20;		  //Power-on the ADC
	delay_sw(time);		  //Wait for ADC to be fully powered on
}

void SPI_init(void)
{
    // Set GPIO as SPI
    // P1.4 - 1.7 = SCLK, MISO, MOSI, CS
    GP1CON = 0x22220000;
    SPIDIV = 0x05;
    
    // SPI enabled, master mode, serial clock pulses at beginning of xfer, 
    // serial clock idles high, MSB transmitted first, transfer on write to
    // SPITX, interrupt when TX empty (SPI interrupt not enabled in IRQEN)
	  // overwrite SPIRX when new serial byte received
    SPICON = 0x14F;
}

/** timer0_init(): Configures timer0 for 200us period keeping **/
void timer0_init(void)
{
	// timer0 set to 200us period
	timer0_Flag = 0;				// Re-initialize flag
	IRQEN |= 0x04;					// Re-initialize timer IRQ
	T0LD = 0x20A6;					// 0x1053 = 100us period at 41.78MHz clock freq (in system_init() );
													//    0x20A6 for 200us; x30F9 for 300us
													//		0x829 for 50us; 0x414 for 25us; 0x20A for 12.5us
	T0CON = 0xC0;					// enable timer0, set in periodic mode with multiplier = 1
}

void General_IRQ_Handler(void)
{
  if ((IRQSIG & 0x04) == 0x4)		//timer0 IRQ clear routine
	{
		timer0_Flag = 1;
		T0CLRI = 0xFF;
		return ;
	}

	if ((IRQSIG & 0x8) == 0x8)		//timer1 IRQ clear for us/ms delay
	{
		timer1_Flag = 1;
		T1CLRI = 0xFF;
		return ;
	}
  
  /*  
  if ((IRQSIG & 0xD) == 0xD)      // SPI IRQ clear for transmit
  {
    //SPITX_flag = 1;
    return ;
  }
	*/
    
}

void SPI_write(unsigned char data)
{
    SPITX = data;
		while((SPISTA & 0x1) == 0x1) {
			// wait while SPI is transmitting
		}
    return ;
}

/** waitForRestOfPeriod(): Maintains 200us cycle using timer0 IRQ and global flags **/
void waitForRestOfPeriod(void)
{
	if (timer0_Flag == 1)
	{
		timer0_Flag = 0;	//Reset timer flag
		return;				//Output LED light to signal slower than 100kHz frequency
	}
	else
	{
		while(timer0_Flag == 0){}	//Wait until interrupt request
		timer0_Flag = 0;
		return;
	}
}

/** ADCinput(): Converts ADC conversion HEX to decimal voltage. **/
float ADCinput(int ADC_hex_result)
{
	float input_voltage;
	int ADC_hex_result_shifted;
	ADC_hex_result_shifted = ADC_hex_result >> 16;
	input_voltage = ((ADC_hex_result_shifted)*2.52)/(0xFFF);	  	//2.52V chosen experimentally with ADC connected to voltmeter
	return input_voltage; 
}

/** outputDAC(): Converts decimal voltage to HEX for DAC output. **/
int outputDAC(float desired_voltage)	
{
	int output_integer = floor((desired_voltage*0xFFF)/2.497);		//"unit conversion" from desired decimal voltage to hex code
																																// 2.4976 is voltage when 0xFFF applied to DAC

	if (output_integer <= 0xFFF && output_integer >= 0x200)
		output_integer = (output_integer - 0x2) << 16;				//Shift integer result 16 bits to left to conform to DACxDAT format
																													// maximum and minimum integer cap
																													// result subtracted from small integer as rough calibration to measured values
	else if (output_integer > 0xFFF)								
	{	
		output_integer = 0xFFF;
		output_integer = output_integer << 16;
	}
	else
	{
		output_integer = 0x200;
		output_integer = output_integer << 16;
	}	
	return output_integer;
}

/** delay_sw(): Software delay. time = 10 --> ~12us **/
void delay_sw(int time)
{
	while (time >=0)
	time --;
}

/** delay_us(): Microsecond delay on timer1. Enables/disables timer1 within function **/
void delay_us(unsigned long time)
{
	timer1_Flag = 0;
	T1CON = 0x0;
	
	IRQEN |= 0x08;					    //Enable timer1 IRQ; timer1 = bit 3
										
	T1LD = ((time * 42782) >> 10) - 1;	//T1LD = time (us) * freq (in MHz) (e.g. 500us * 41.78 MHz = 20890-1) subtract 1 since includes zero
	T1CON = 0xC0;						//Init timer1, 41.78MHz, down mode, periodic				
	while(timer1_Flag == 0){}			//Wait for timer flag flip
	timer1_Flag = 0;
	T1CON = 0x0;						//Disable timer1
	IRQCLR |= 0x08;
	return ;
}

/** delay_ms(): Millisecond delay on timer1 using delay_us(). **/
void delay_ms(unsigned long time)
{
	unsigned short i = 0;	
	//Wait for 1 ms each time through loop
	for(i = 0; i < time; i++)
	{
		delay_us(1000);
	}
}



// EOF ------------------------------------------------------------------------
