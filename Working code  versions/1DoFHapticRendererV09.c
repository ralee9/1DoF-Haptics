/******************************************************************************
Randy Lee (ral63@pitt.edu)
Avi Marcovici (avi.marcovici@gmail.com)
Visualization and Image Analysis Laboratory
Department of Bioengineering
University of Pittsburgh

1 DoF Haptic Renderer -- ADuC7026 control software

Code version: v09

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
V02 -- Changing the "if" statment to "while" statment so the program wait until there is data on SPIRX to read it.
V03 -- Fix the function outputDAC to be more accurate.
V04 -- Make a small fix on outputDAC function and adding the testmode function.
V05 -- Fix the function outputDAC and changed to work until 2.49V.
V06 -- Added force sensor input and speaker output. 
       Updated testmode() to cycle starting/ending @ 1.25V
       Added sensor average function to tare force sensor
V07 -- Added zero stiffness spring behavior
			 Added Datachange_testmsg() that gives HEX integers from A00 to AFF and can cycle
			 Added send_data() to split 3 massges of 12 bits to 5 masseges and to send them
V08 -- Added PID_zeroStiffness()- PID controler 
V09 -- Fix the function Datachange_testmsg() by added global int - counterTest_message 
			 Added delay to send_data() between the send message(SPI_write)
			 Make red LED turn on when read FF and green LED when read 00 from the Wixel

Last updated: 16 December 2015
******************************************************************************/

// Libraries ------------------------------------------------------------------
#include <stdio.h>
#include <aduc7026.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Global variables -----------------------------------------------------------
float voltageForce, voltageForce_avg, voltageSpeaker, forceSlope, intersectionPoint;
float epsilon = 0.04;
int counterTest_message = 0xA00;

struct Wixel_msg{					// struct of 3 messages 
		int Position;
		int Force;
		int Output;
	};
// structure of 3 following HEX numbers for testing the Wixel
// S_msg to be used for messages to be sent (ADC/DAC)
struct Wixel_msg test_messages, *T_msg, *S_msg; 

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
void testmode(int times);
float sensor_avg(void);
struct Wixel_msg *Datachange_testmsg(struct Wixel_msg *);
void send_data(struct Wixel_msg *);
float PID_zeroStiffness(void); 

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

//Reads sensor voltage from ADC channel #
float readSensors(int channel);	

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
	
	  //testmode(5);			// sinusoidal increase/decrease in speaker voltage

		
		T_msg = &test_messages;		// for updating test_msg struct using Datachange_testmsg() 
		S_msg = &test_messages;		// will point to new struct for ADC/DAC values that will be sent
	
		voltageForce_avg = sensor_avg();		// Calculates the average voltage that comes from the sensor on resting
		
		
    // Main loop --------------------------------------------------------------
		while(1) {
				timer0_init();
			
				GP2DAT ^= 0x10000; //Set the pin 2.0 *OUTPUT*, initialize Low.
													 //To know when waitForRestOfPeriod() stops.
			
			
				S_msg = Datachange_testmsg(T_msg); // cycle through 3 12 bit messages
			  send_data(S_msg);						// separates 3 12 bit messages into 5 8 bit messages
																		// calls SPI_write to send data
				
			
				voltageSpeaker = PID_zeroStiffness();
				DAC0DAT = outputDAC(voltageSpeaker);
			
			
			
			
			
				while (!((SPISTA & 0x8) == 0x8)) 
					{
						// wait while nothing in SPIRX 
					}
				
				message = SPI_read();         // reading data from the Wixel
				
				if (message == 0xFF)           //&& counter%1000 == 0 
				{ 
					GP0DAT |= 0x10000;               // turn on red LED
					GP0DAT &= 0xFF010000;						 // turn off green LED
				}
				
				if (message == 0x00)    
				{
					GP0DAT |= 0x20000;							// turn on green LED
					GP0DAT &= 0xFF020000;           // turn off red LED
				}	
				
				
			
			/*-------------------------------------------------------------------------------
			
				// Zero stiffness spring behavior //
				voltageForce = readSensors(0);		// Push --> voltage drops, Pull --> voltage increase
				
				forceSlope = 1.25 / ( 2 - voltageForce_avg );				// at voltageForce_avg -> zero force generated by speaker
																														// 2V is "lowest" voltage seen when pushing on force sensor
																														// empirically obtained
				intersectionPoint = 1.25 - forceSlope * voltageForce_avg;		// "y-intercept" of linear function for speaker actuation
				voltageSpeaker = forceSlope *  voltageForce + intersectionPoint;
				//DAC0DAT = outputDAC(voltageSpeaker);
				
				if (voltageForce - voltageForce_avg < -epsilon || voltageForce - voltageForce_avg > epsilon)
				{
					// epsilon is minimum voltage change on sensor before speaker moves -- "dead zone"
					//Push --> move speaker down by increasing voltage
					//Pull --> move speaker up by decreasing voltage
					DAC0DAT = outputDAC(voltageSpeaker);
				}
				else
				{
					// mute speaker
					DAC0DAT = outputDAC(1.25);
				}
				
				//DAC0DAT = outputDAC(1.25);
		
			*/	
    
			
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

float readSensors(int channel)
{
	float sensorVoltage;
	
	ADCCP = channel;
	ADCCON = 0x6A3;
	ADCCON &= ~(1 << 7);
	while (!ADCSTA){}
	sensorVoltage = ADCinput(ADCDAT);
	return sensorVoltage;
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
	DAC2CON = 0x12;					//Set DAC3 output to 0-V_ref(2.5V) range and turn DAC2 on
	DAC0CON = 0x12;					//Set DAC3 output to 0-V_ref(2.5V) range and turn DAC0 on
	
	
	GP2CON = 0x00;					//Set digital I/O port 2 as *GPIO* on pins P2.0-2.7
	GP2DAT = 0xFF000000;		//Set all pins on port 2 as *OUTPUT*, initialize LOW
	
  GP0CON = 0x00;					//Set digital I/O port 0 as GPIO on pins P0.0-0.7
	GP0DAT |= 0xFF000000;		//Set all I/O pins on port 0 as *INPUT*, initialize LOW
	
	/*
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
	GP2DAT |= 0x10000; //Set pin 2.0  *OUTPUT*, initialize High.
										 // To know when waitForRestOfPeriod() begins.
	
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
	    int output_integer = floor((desired_voltage * 0xFFF) / 2.497);        //"unit conversion" from desired decimal voltage to hex code
                                                                            // 2.4976 is voltage when 0xFFF applied to DAC
    if (desired_voltage < 2.49 && desired_voltage >= 0.036)
    {
        output_integer = output_integer << 16;                //Shift integer result 16 bits to left to conform to DACxDAT format
                                                                                                    // maximum and minimum integer cap
    }
    else if (desired_voltage >= 2.49)                                
    {    
        output_integer = 0xFFF;
        output_integer = output_integer << 16;
    }
    else if (desired_voltage < 0.036 && desired_voltage >= 0.0065) // Under 0.036V needed a function to be more accurate
    {
        desired_voltage = 1.2 * desired_voltage - 0.0072733;         // Linear function obtained by excel calibration for small voltages
        output_integer = floor((desired_voltage * 0xFFF) / 2.497);   
        output_integer = output_integer << 16;
    }    
    else if (desired_voltage < 0.0065)  // The minimum voltage we can get
    {
        output_integer = 0x00;
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
/** testmode(): cycle DAC output up and down n times  **/
void testmode(int times)
{
	int n;
	float voltage;
	
	for (n=1 ; n <= times ; ++n )  
	{
		for ( voltage=1.25 ; voltage <= 2.5 ; voltage=voltage+0.001 ) // Cycle up to 2.49v
		{
			DAC0DAT = outputDAC(voltage);
			delay_ms(2);
		}	
		
		delay_ms(50);
		
		for ( voltage=2.5 ; voltage >= 0.0065 ; voltage=voltage-0.001 )  // Cycle down to 0.0065V
		{
			DAC0DAT = outputDAC(voltage);
			delay_ms(2);
		}
		
		delay_ms(50);
		
		for ( voltage=0.0065 ; voltage <= 1.25 ; voltage=voltage+0.001 ) // Cycle up to 2.49v
		{
			DAC0DAT = outputDAC(voltage);
			delay_ms(2);
		}	
		DAC0DAT = outputDAC(1.25);
		
		delay_ms(50);
	}
	
	/* Another possibility :
	
	float voltage=0.0065,step=0.001;
	
	DAC2DAT = outputDAC(1.25);
	
	while ( times >= 0 )
	{
		
		delay_ms(2);
		voltage += step;
	
		if ( voltage >= 2.5 ) // Cycle down to 0.0065V	
		{
			DAC2DAT = outputDAC(2.5);
		 step = -0.001;
		 delay_ms(100);
		}			
		if ( voltage <= 0.0064) // Cycle up to 2.49V
		{
			DAC2DAT = outputDAC(0.0065);
			step = 0.001;
			--times;
			delay_ms(50);
		}	
		DAC2DAT = outputDAC(voltage);
	}
	*/
}

/** sensor_avg(): takes 10 sample average of ADC0 **/
float sensor_avg(void)
{
	int i;
	float avg_voltage, add_voltage=0;
	float voltage[10]; 
	for (i=0 ; i<10 ; ++i)
	{
		voltage[i]=readSensors(0);
		delay_ms(2);
		add_voltage = add_voltage + voltage[i];
	}
	avg_voltage = add_voltage / 10.0;
	
	return avg_voltage;
}

/** Datachange_testmsg (): gives HEX integer from A00 to AFF **/
struct Wixel_msg *Datachange_testmsg(struct Wixel_msg *T_msg)
{
	// Creates 3 test messages which cycle, to test Wixel data logging interface
	// Messages are patterned like: A00, B01, C02 --> AFF, B00, C01
	/*
	T_msg->Position = counterTest_message;
	T_msg->Force = counterTest_message + 0x101;
	T_msg->Output = counterTest_message + 0x202;
	*/
	
	if (counterTest_message == 0xB00)
	{
		counterTest_message = 0xA00; 
	}
	T_msg->Position = counterTest_message;
	if (counterTest_message == 0xAFE)
	{
		 T_msg->Force = counterTest_message + 0x101;
		 T_msg->Output = 0xC00;
	}
	else if (counterTest_message == 0xAFF)
	{
		 T_msg->Force = 0xB00;
		 T_msg->Output = 0xC01;
	}
	else
	{
		 T_msg->Force = counterTest_message + 0x101;
		 T_msg->Output = counterTest_message + 0x202;
	}
		counterTest_message += 1;
	
	return T_msg;
}	
/** send_data (): splits 3 massges of 12 bits to 5 masseges and send them **/
void send_data(struct Wixel_msg *S_msg)
{
	int msg1, msg2, msg3;
	char split_msg[5];
	
	msg1 = S_msg->Position;
	msg2 = S_msg->Force;
	msg3 = S_msg->Output;
	
	// split 3 messages into 5 using bit masking & shifting
	split_msg[0] = (msg1 & 0xFF0) >> 4;
	split_msg[1] = (msg1 & 0xF) << 4 | (msg2 & 0xF00) >> 8;
	split_msg[2] = (msg2 & 0xFF);
	split_msg[3] = (msg3 & 0xFF0) >> 4;
	split_msg[4] = (msg3 & 0xF);
	
	SPI_write(split_msg[0]);
	delay_us(4);
	SPI_write(split_msg[1]);
	delay_us(4);
	SPI_write(split_msg[2]);
	delay_us(4);
	SPI_write(split_msg[3]);
	delay_us(4);
	SPI_write(split_msg[4]);
	//delay_us(5);
	//SPI_write(0xFF);
	
}
/** PID_controler (): use PID to make the force we apply to be equal to the speaker's spring (Zero stiffness spring behavior) **/
float PID_zeroStiffness (void) 
{
	float output_voltage, errorSum, lastError = 0;
	float	iState, dGain , iGain , pGain; 
	float	iMax = 2.5, iMin = -2.5;
		
	voltageForce = readSensors(0);	// Push --> voltage drops, Pull --> voltage increase
	iGain = readSensors(1); // first knob - Integral gain
	dGain = readSensors(2); // second knob - Differential gain
	
	errorSum += voltageForce;
	iState = iGain*errorSum; // istate it's the value of the integral in the PID
	
	if(iState > iMax) // limiting the integral not get to infinity
	{
		iState = iMax;
	}
	else if (iState < iMin)
	{
		iState = iMin;
	}
	
	pGain = 1.25 / ( 2 - voltageForce_avg );				// at voltageForce_avg -> zero force generated by speaker
																									// 2V is "lowest" voltage seen when pushing on force sensor
																									// empirically obtained
	intersectionPoint = 1.25 - pGain * voltageForce_avg;		// "y-intercept" of linear function for speaker actuation
				
	output_voltage = (pGain*voltageForce) - iState + (dGain*(voltageForce - lastError)) + intersectionPoint; 
																									// the istate is negativ to not get to inf
																									// the proportional work like liniar function with an offset(intersectionPoint)
	
	/*
	if (voltageForce - voltageForce_avg < -epsilon || voltageForce - voltageForce_avg > epsilon)
	{
										// epsilon is minimum voltage change on sensor before speaker moves -- "dead zone"
										//Push --> move speaker down by increasing voltage
										//Pull --> move speaker up by decreasing voltage
		//DAC0DAT = outputDAC(output_voltage);
		output_voltage = (pGain*voltageForce) + iState + (dGain*(voltageForce - lastError)) + intersectionPoint;
	}
	else
	{
		// mute speaker
		output_voltage = 1.25;
	}
	
	*/
	lastError = voltageForce; // Updating the lastError
	
	return output_voltage;
}
// EOF ------------------------------------------------------------------------
