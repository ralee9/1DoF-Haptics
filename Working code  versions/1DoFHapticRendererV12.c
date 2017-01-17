/****************************************************************************
Randy Lee (ral63@pitt.edu)
Avi Marcovici (avi.marcovici@gmail.com)
Visualization and Image Analysis Laboratory
Department of Bioengineering
University of Pittsburgh

1 DoF Haptic Renderer MCU Control

Equipment:
- Analog Devices ADuC7026
- FaitalPRO 5FE120 speaker
- Pololu Wixel programmable USB module

Code version: v12

------------
Description and notes:

Controlling a commercial speaker in force and position to create a haptic 
renderer. Virtual membranes of variable stiffness are created by combining
two functionalities: (1) a zero stiffness spring -- defies gravity by keeping
force on the speaker drum at zero; (2) an infinite stiffness spring -- virtual
wall to control both force and position. Infinite stiffness spring will also
enable position dependent calibration for force and current

-----------
Pin and Port Assignments:

ADC0 -- Honeywell FS01 Force Sensor
ADC1 -- Front panel knob 1
ADC2 -- Front panel knob 2
ADC3 -- Front panel knob 3
ADC4 -- IR proximity (optical) sensor
ADC5 -- Speaker impedence circuit output
ADC6 -- Desired speaker position, from 10 turn pot

DAC0 -- Speaker output

GPIO P0.x -- Digital output
GPIO P1.x -- SPI communication
GPIO P2.x -- Digital output

-----------

Change Log:

v00 -- Init version. Imported previous functions. SPI communication prototype
v01 -- Adding the counter and the flag to the main program.
v02 -- Changing the "if" statment to "while" statment so the program wait 
        until there is data on SPIRX to read it.
v03 -- Updated the function outputDAC to be more accurate.
v04 -- Make a small fix on outputDAC function and adding the testmode function
v05 -- Fix the function outputDAC and changed to work until 2.49V.
v06 -- Added force sensor input and speaker output. 
       Updated testmode() to cycle starting/ending @ 1.25V
       Added sensor_avg() to tare force sensor on ADC0
v07 -- Added basic zero stiffness spring behavior
       Added Datachange_testmsg() that gives HEX integers from A00 to AFF and 
        can cycle
       Added send_data() to split/ send 3 msgs of 12 bits to 5 msgs
v08 -- Added PID_zeroStiffness()- PID controller 
v09 -- Fix the function Datachange_testmsg() by added global var
        (counterTest_message)
       Added delay to send_data() between the send message(SPI_write)
       Make red LED turn on when read FF and green LED when read 00 from the 
        Wixel
v10 -- Reorganization of functions and whitespace deletion
       Added dataUpdate_send() to modify current messages to 12 bit format
       Changed dataUpdate functions to return void
       Added PID_const struct to save gains and error globally
v11 -- Updated send_data to include a 0xAA start byte and 0000 end byte
       Increased delay between successive messages to 10us
       More whitespace deletion in prep for open source release
v12 -- Added ADC6 to control "desired" position of speaker using 10 turn pot
       Added virtual wall function, controlled with PID using position sensor
	   Updated sensor_avg() to take average of indicated channel
       Updated Wixel_msg struct to accept 8 channels of ADC/DAC data
       Added send_curr_data() function to split 8 channels of 1DoF data into 
        12 separate byte messages
       Added sensor_status[] and check_sensor_status() to read sensor channels
        if the sensor had not yet been read that loop
       Updated functions that deal with raw ADC/DAC conversion values to use
        long integers (results are at least 32 bits long)
       Formatting and code cleaning

Last updated: 9 March 2016

*****************************************************************************/

/* Libraries -------------------------------------------------------------- */
#include <aduc7026.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Defines ---------------------------------------------------------------- */
#define OBSOLETE 0
#define NOT_YET_IMPLEMENTED 0
#define TEMP_DISABLED 0

/* Global variables ------------------------------------------------------- */
/* 1DoF device data */
//float voltageForce, voltageForce_avg, voltageSpeaker;

float force_volts, force_volts_avg;
float opticalPos_volts, opticalPos_mm;
float inductPos_volts, inductPos_mm;
long int speaker_voltageHex;    /* raw speaker hex result is 32 bit in size */
float speaker_volts;
float desiredSpeaker_volts, lastDesiredSpeaker_volts;
float desiredSpeaker_mm, lastDesiredSpeaker_mm;

/* linear fit variables for zero stiffness spring */
float forceSlope, intersectionPoint;

/* counter for SPI comm testing */
int counterTest_message = 0xA00;

/* testing for 8 data comms */	
char loop_count = 0x00;

/* define struct of three 12 bit messages to experiment with SPI sending */
struct Wixel_test{
    int Comp_1;
	int Comp_2;
	int Comp_3;
};

/* 
 * define struct for actual messages to be sent through wixel.
 * members are integers beause they are saved in read_store_sensors() a
 * shifted format from ADC/DAC raw results
 */
struct Wixel_msg{
    int forceSensor;
    int opticalSensor;
    int inductanceSensor;
    int speakerOut;
    int knob1;
    int knob2;
    int knob3;
    int tenTurnPot;
};

/* array of status flags to see if sensor has been polled this loop */
int sensor_status[8];

/* define struct for PID constants */
struct PID_const{
    float pGain;
    float iGain;
    float dGain;
    float setPoint;
	float speakerBias;
    float lastError;
    float errorSum;
};

/* structure of 3 integers for testing the Wixel */
struct Wixel_test test_messages, *pTest_msg;

/* structure of 8 data pieces to send through Wixel */
struct Wixel_msg curr_data;
struct Wixel_msg *pCurr_data;

/* structure of PID constants */
struct PID_const PID_zeroStiff, *pPID_zero;
struct PID_const PID_wall, *pPID_wall;

/* Flags, counters, and misc ---------------------------------------------- */
volatile signed int timer0_Flag = 0;
volatile signed int timer1_Flag = 0;
int flag = 0;
int changeFlag = 0;
int counter = 0;        /* loop counter */
int i = 0;              /* global loop counter */
int message = 0;
float epsilon = 0.02;

/* Function declarations -------------------------------------------------- */
/* system and components init */
void General_IRQ_Handler(void);
void system_init(void);
void timer0_init(void);
void SPI_init(void);
void SPI_write(unsigned char data);
void ADCpoweron(int software_delay_time);
void testmode(int times);

/* data update and formatting functions for SPI comm prototyping */
void dataUpdate_testmsg(struct Wixel_test *);
void dataUpdate_testformat(struct Wixel_test *);
void send_test_data(struct Wixel_test *);

/* 1DoF device sensor data read/store/format for Wixel */
void initial_read_store_sensors(struct Wixel_msg *data);
float read_store_sensors(int channel, struct Wixel_msg *data);
void store_DAC_data(int rawDAC_hex, struct Wixel_msg *data);
void send_curr_data(struct Wixel_msg *data);
void dataUpdate_msg(struct Wixel_msg *data);
void check_sensor_status(void);

/* sensor reading and system behavior functions */
float PID_zeroStiffness(struct PID_const *);
float PID_virtualWall(struct PID_const *);
float speakerDACtoDisplacement(float volts);
float opticalADCtoDisplacement(float volts);
float sensor_avg(int channel);

/* Imported HHFM functions ------------------------------------------------ */
/* initialize main loop period time keeping */
void timer0_init(void);

/* variable software delay using decreasing for loop */
void delay_sw(int time);

/* variable us delay using timer1 */
void delay_us(unsigned long time);

/* variable ms delay using timer1 */
void delay_ms(unsigned long time);

/* converts raw hex result from ADC conversion to decimal voltage */
float ADCinput(int ADC_hex_result);

/* converts desired decimal voltage to hex integer for DAC output */
int outputDAC(float desired_voltage);

/* read voltage on desired channel (ADC#). returns decimal voltage */
float read_sensors(int channel);

/* wait function at end of main loop to limit loop duration using timer0 */
void waitForRestOfPeriod(void);

/* read data from the Wixel and returns the received integer */
int SPI_read(void);

/* scales potentiometer voltage by some fullscale value */
float pot_voltage(float voltage, float fullscaleValue);

/* Main program ----------------------------------------------------------- */
int main(void)
{
    /* software delay to avoid JTAG lock while prototyping */
    delay_sw(50000);
    
    /* Core clock and power state control */
    system_init();
    
    /* Power on ADC channels */
    ADCpoweron(20000);      // power on ADC
    
    /* Enable timer0 and timer1 interrupts for timing */
    //IRQEN = 0x2004;           /* enable timer0 & SPI interrupts */
	IRQEN = 0x04;				/* enable timer0 interrupt */
    IRQ = General_IRQ_Handler;  /* specify interrupt handler */
    
    //wait();
    
    /*  
     * Set up SPI on GPIO P1.4 - 1.7
     * SPI in master mode, clock pulses on xfer init
     * transmit on write to TX register
     * interrupt when TX empty
     */
    SPI_init();
	
    /* 
     * testmode() generates a sinusoidal increase/decrease in speaker voltage 
     */
    //testmode(5);
	
    /* Initialize struct pointers */
    /* for updating test_msg struct using Datachage_testmsg() */
	pTest_msg = &test_messages;

    /* ADC/DAC values that will be sent to Wixel */
	//pCurr_msg = &curr_messages;	
    pCurr_data = &curr_data;

    /* PID controller values for zero stiffness and virtual wall modes */
	pPID_zero = &PID_zeroStiff;
    pPID_wall = &PID_wall;
	
	/* initial read/store of all sensors */
	initial_read_store_sensors(pCurr_data);

    /* calculate and save the average voltage from the force sensor on ADC0 */
    force_volts_avg = sensor_avg(0);
	
    /* Main loop ---------------------------------------------------------- */
	while(1) {
        /* start timer0 to while loop duration */
		timer0_init();

        /* 
         * flip P2.0 high to measure duration of waitForRestOfPeriod()
         * P2.0 is init LOW, and flipped again in waitForRestOfPeriod
         */
        GP2DAT ^= 0x10000;

		
        /* reset sensor status */
        for (i=0; i<8; i++)
        {
            sensor_status[i] = 0;
        }
		
		#if TEMP_DISABLED
        /* 
         * SPI send and receive testing:
         * cycle through three 12 bit messages (ADC/DAC data) with the 
         * following format: [A01, B02, C03] -> [A02, B03, C04] -> 
         * ... -> [AFD, BFE, CFF]
         */
		dataUpdate_testmsg(pTest_msg);
		send_test_data(pTest_msg);

        /* Read data from Wixel */
        while (!((SPISTA & 0x8) == 0x8)){
            // wait while nothing in SPIRX
        }
        message = SPI_read();
        

        /* 
         * Zero stiffness spring: 
         * read current force on force sensor, reduce voltage on speaker to 
         * "run away* so force sensor is as close to avg preload as possible
         */
        //voltageSpeaker = PID_zeroStiffness(pPID_zero);
		//speaker_voltageHex = outputDAC(voltageSpeaker);
        //DAC0DAT = speaker_voltageHex;
        //store_DAC_data(speaker_voltageHex, pCurr_data);
		

        /* 
         * Virtual Wall:
         * read desired speaker position from 10 turn pot, PID control to
         * minimize position error
         */ 
        desiredSpeaker_volts = read_store_sensors(6, pCurr_data);
        //desiredSpeaker_volts = read_sensors(6);
        //DAC0DAT = outputDAC(desiredSpeaker_volts);
        if (abs(desiredSpeaker_volts - lastDesiredSpeaker_volts) > epsilon)
        {
            // render new position if there is a change in desired speaker voltage
            //DAC0DAT = outputDAC(desiredSpeaker_volts);
			//pPID_wall->speakerBias = desiredSpeaker_volts;
            
            /* update virtual wall reference with change in 10 turn pot */
            desiredSpeaker_mm = speakerDACtoDisplacement(desiredSpeaker_volts);
            pPID_wall->setPoint = desiredSpeaker_mm;

            // save current optical sensor reading as setPoint
            //opticalPos_volts = sensor_avg(4);
			//pPID_wall->setPoint = opticalADCtoDisplacement(opticalPos_volts);
            //pPID_wall->setPoint = opticalPos_volts;
			//pPID_wall->setPoint = desiredSpeaker_mm;
        }
        else
        {
            /* if no change in desired position render virtual wall */
            //speaker_volts = PID_virtualWall(pPID_wall);
            //DAC0DAT = outputDAC(speaker_volts);

            speaker_volts = PID_virtualWall(pPID_wall);
            speaker_voltageHex = outputDAC(speaker_volts);
            DAC0DAT = speaker_voltageHex;
            store_DAC_data(speaker_voltageHex, pCurr_data);
        }
        /*
        else
        {
            opticalPos_volts = readSensors(4);
        }
        
        if (abs(pPID_wall->setPoint - opticalPos_volts) > epsilon)
        {			
            speaker_volts = PID_virtualWall(pPID_wall);
            DAC0DAT = outputDAC(speaker_volts);
        }
        */
        lastDesiredSpeaker_volts = desiredSpeaker_volts;
		#endif

		/* mute speaker for testing */
		speaker_voltageHex = outputDAC(1.25);
		DAC0DAT = speaker_voltageHex;
		store_DAC_data(speaker_voltageHex, pCurr_data);
		
        /* check to see if any sensors have not yet been read */
        check_sensor_status();

        /* send data to Wixel over SPI. Data already formatted to 12-bit
         * [0 - FFF] range. Split eight 12-bit messages into twelve 8-bit
         * messages
         */
		//dataUpdate_msg(pCurr_data);
        send_curr_data(pCurr_data);

        /* Read data from Wixel */
        while (!((SPISTA & 0x8) == 0x8)){
            /* wait while nothing in SPIRX */
        }
        message = SPI_read();
        
        #if NOT_YET_IMPLEMENTED
        /*
         * Update mode of 1DoF depending on message received from Wixel
         */
        if (message = 0xAA)
        {
            speaker_volts = PID_zeroStiffness(pPID_zero);
		    DAC0DAT = outputDAC(speaker_volts);
        }
        else if (message = 0xBB)
        {
            speaker_volts = PID_virtualWall(pPID_wall);
            DAC0DAT = outputDAC(speaker_volts);
        }
        #endif
				
		if (message == 0xFF)            //&& counter%1000 == 0 
		{ 
			GP0DAT |= 0x10000;          // turn on red LED
			GP0DAT &= 0xFF010000;		// turn off green LED
		}
	
		if (message == 0x00)    
		{
			GP0DAT |= 0x20000;			// turn on green LED
			GP0DAT &= 0xFF020000;       // turn off red LED
		}	
	
		counter++;
		waitForRestOfPeriod(); 
    }

    return 0;
} /* end of Main loop ----------------------------------------------------- */


/* Functions -------------------------------------------------------------- */

/**  PID_virtualWall() --
 **     PID control of position to resist speaker movement by an extern. force
 **/
float PID_virtualWall(struct PID_const *PID)
{
    float iMax = 2.5;
    float iMin = -2.5;
    float output_voltage;
    float currOptical_volts, currOptical_mm;
    float pState, iState, dState;
    float pGain, iGain, dGain;
    float error = 0;
    
    /* 
     * Obtain PID gains from knobs 1-3. Only update if knob has changed by
     * more than epsilon 
     */
    pGain = read_store_sensors(1, pCurr_data);
	//pGain = potVoltage(pGain, 5.0);
    //if (abs(pGain - PID->pGain) >= epsilon) PID->pGain = pGain;
    //else pGain = PID->pGain;
    
    iGain = read_store_sensors(2, pCurr_data);
    //if (abs(iGain - PID->iGain) >= epsilon) PID->iGain = iGain;
    //else iGain = PID->iGain;
    
    dGain = read_store_sensors(3, pCurr_data);
    //if (abs(dGain - PID->dGain) >= epsilon) PID->dGain = dGain;
    //else dGain = PID->dGain;

    /* set dGain & iGain to zero for Ziegler Nichols tuning */
    iGain = 0.0;
    PID->iGain = 0.0;
    dGain =0.0;
    PID->dGain = 0.0;
    
    /* read current position and calculate error */
    currOptical_volts = read_store_sensors(4, pCurr_data);
    currOptical_mm = opticalADCtoDisplacement(currOptical_volts);
    
    /* error calculated in volts */
    //error = PID->setPoint - currOptical_volts;
    
    /* error calculated in mm */
    error = PID->setPoint - currOptical_mm;
    PID->errorSum += error;
    
    /* Integral term */
    iState = iGain * (PID->errorSum);
    if (iState > iMax) iState = iMax;
    else if (iState < iMin) iState = iMin;
    
    /* Derivative term */
    dState = dGain * (error - PID->lastError);
    
    /* Proportional term */
    pState = pGain * error;
    
    /* Bias PID controller with the speaker voltage midpoint */
	// increasing voltage on speaker pushes down
	// optical position sensor increases as speaker pushes down
    //output_voltage = pState + iState + dState + PID->speakerBias;
    output_voltage = pState + iState + dState + 1.25;
    
    /*
	if (abs(error) > epsilon)
	{
		output_voltage = pState + iState + dState + desiredSpeaker_volts;
	}
	else
    {
		output_voltage = desiredSpeaker_volts;
    }
    */
    
    PID->lastError = error;
    return output_voltage;
}


/** PID_zeroStiffness() --
 **     use PID to make the force we apply to be equal to the speaker's 
 **     spring (Zero stiffness spring behavior) 
 **/
float PID_zeroStiffness(struct PID_const *PID) 
{
    float iMax = 2.5;
    float iMin = -2.5;
    float output_voltage, pState, iState, dState;
    float pGain, iGain, dGain;
    float error = 0.0;
    
    /* push --> voltage drop; pull --> voltage increase */
	force_volts = read_store_sensors(0, pCurr_data);
    //opticalPos_volts = read_store_sensors(4, pCurr_data);
    
    /* read gains from knobs and only save if they are different from last loop */
    pGain = read_store_sensors(1, pCurr_data);  // first knob - proportional gain
    //pGain = potVoltage(pGain, 10.0);
    if (abs(pGain - PID->pGain) > epsilon) PID->pGain = pGain;
    else pGain = PID->pGain;
    
    iGain = read_store_sensors(2, pCurr_data);  // second knob - Integral gain
    if (abs(iGain - PID->iGain) > epsilon) PID->iGain = iGain;
    else iGain = PID->iGain;
    
	dGain = read_store_sensors(3, pCurr_data); // third knob - Differential gain
    if (abs(dGain - PID->dGain) > epsilon) PID->dGain = dGain;
    else dGain = PID->dGain;

    /* Position & Force based PID implementation 
    float sensorLoad = force_volts - force_volts_avg;
    if (abs(sensorLoad) > epsilon)
    {
        
    }
    */
    
    /* Force-based error implementation */
    // calculate deviation from set point ("error") and update error integral
	error = force_volts - force_volts_avg;
	PID->errorSum += error;
    
    /* Integral Term */
    iState = iGain*(PID->errorSum);
    
    /* limit the integral to not get to infinity */
	if (iState > iMax)
	{
		iState = iMax;
	}
	else if (iState < iMin)
	{
		iState = iMin;
	}	
	
    /* Differential Term */
    dState = dGain * (error - PID->lastError);
    
	/* Proportional Term */ 
	pState = pGain * error;
    
	/* PID Controller Equation */
    // pState is negative to "run away" from the force on the sensor
	if (abs(error) > epsilon)
	{
        output_voltage = -pState + iState + dState + 1.25;
	}
    else
    {
        output_voltage = 1.25;
    }
    
    #if OBSOLETE
    /* OBSOLETE Implementation */
    /* Linear function controller implementation (original) --
     * Set up a linear function centered around the average voltage on
     * the force sensor input. Slope is emprically obtained, and a 
     * "y-intercept" for the function is calculated. Voltage on the speaker
     * (y) is calculated from this linear function with the sensor input (x). 

     * m = (y1-y0)/(x1-x0) form
	 * pGain = 1.25 / (2.0 - voltageForce_avg);
	 * at voltageForce_avg -> zero force generated by speaker
     * 2V is "lowest" voltage seen when pushing on force sensor
	 * empirically obtained
	
     * "y-intercept" of linear function for speaker actuation
     * intersectionPoint = 1.25 - pGain * voltageForce_avg;	
	 * output_voltage = (pGain*voltageForce) - iState + dState + intersectPoint; 
	 * the istate is negative to not get to inf
	 * the proportional work like linear function with an offset(intersectPoint)
	 */
	if (force_volts - force_volts_avg < -epsilon || 
        force_volts - force_volts_avg > epsilon)
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
	#endif  

	PID->lastError = error;
	return output_voltage;
}

/** speakerDACtoDisplacement() -- 
 **     Calculate expected speaker displacement from a reference voltage
 **/
float speakerDACtoDisplacement(float speaker_volts)
{
    float speaker_mm = 0.0;
    
    speaker_mm = 2.9892*pow(speaker_volts, 2.0) - 13.329*speaker_volts + 12.714;
    
    return speaker_mm;
}

/** opticalADCtoDisplacement() --
 **     Calculate speaker displacement from output voltage 
 **/
float opticalADCtoDisplacement(float optical_volts)
{
    float optical_mm = 0.0;
    
    optical_mm = 3.1185*pow(optical_volts, 2.0) - 10.17*optical_volts + 5.8269;
    
    return optical_mm;
}

/** displacementToOpticalADC() --
 **     Calculate expected optical ADC voltage from speaker displacement 
 **/
float displacementToOpticalADC(float displacement_mm)
{
    float optical_ADC = 0.0;
    
    optical_ADC = 0.0178*pow(displacement_mm, 2.0) - 0.2165*displacement_mm + 0.7478;
    
    return optical_ADC;
}


/** dataUpdate_testmsg() --
 **     Loop HEX integer from A00 to CFF amongst members of Wixel_testmsg
 **/
void dataUpdate_testmsg(struct Wixel_test *test_msg)
{
	/* 
     * Creates 3 test msgs which cycle, to test Wixel data logging interface
	 * Messages are patterned like: A00, B01, C02 --> AFF, B00, C01
     * A global counter counterTest_message is used to keep track btw loops
     */
	
	if (counterTest_message == 0xB00)
	{
		counterTest_message = 0xA00; 
	}
	
    test_msg->Comp_1 = counterTest_message;
    
	if (counterTest_message == 0xAFE)
	{
		 test_msg->Comp_2 = counterTest_message + 0x101;
		 test_msg->Comp_3 = 0xC00;
	}
	else if (counterTest_message == 0xAFF)
	{
		 test_msg->Comp_2 = 0xB00;
		 test_msg->Comp_3 = 0xC01;
	}
	else
	{
		 test_msg->Comp_2 = counterTest_message + 0x101;
		 test_msg->Comp_3 = counterTest_message + 0x202;
	}
    
	counterTest_message += 1;

	return;
}

void dataUpdate_msg(struct Wixel_msg *data)
{
	/*
	 * Create test messages which follow similar format to dataUpdate_testmsg().
	 * Start with 800, 901, A02, B03, C04, D05, E06, F07 --> 
	 * 8FF, 900, A01,...
	 * the global counter counterTest_message is used to keep track btw loops
	 */
	
	data->forceSensor = 0x800 + loop_count;
	data->opticalSensor = 0x900 + (loop_count + 1);
	data->inductanceSensor = 0xA00 + (loop_count + 2);
	data->speakerOut = 0xB00 + (loop_count + 3);
	data->knob1 = 0xC00 + (loop_count + 4);
	data->knob2 = 0xD00 + (loop_count + 5);
	data->knob3 = 0xE00 + (loop_count + 6);
	data->tenTurnPot = 0xF00 + (loop_count + 7);
	
	loop_count++;
	
	return;
}


/** dataUpdate_testsend() --
 **     Modify message members to fit 12 bit format
 **/
void dataUpdate_testformat(struct Wixel_test *curr_msg)
{
    // Modify messages to fit 12 bit format
    curr_msg->Comp_1 = curr_msg->Comp_1 >> 16;
    curr_msg->Comp_2 = curr_msg->Comp_2 >> 16;
    curr_msg->Comp_3 = curr_msg->Comp_3 >> 16;

    return;
}

/** send_test_data() -- 
 **     Splits 3 massges of 12 bits to 5 masseges and send 
 **/
void send_test_data(struct Wixel_test *msg)
{
	int msg1, msg2, msg3;
	char split_msg[5];
	
	msg1 = msg->Comp_1;
	msg2 = msg->Comp_2;
	msg3 = msg->Comp_3;
	
	// split three 12 bit messages into 5 using bit masking & shifting
    // msg #5 is shifted left 4 bits so last byte is 0000
	split_msg[0] = (msg1 & 0xFF0) >> 4;
	split_msg[1] = (msg1 & 0xF) << 4 | (msg2 & 0xF00) >> 8;
	split_msg[2] = (msg2 & 0xFF);
	split_msg[3] = (msg3 & 0xFF0) >> 4;
	split_msg[4] = (msg3 & 0xF) << 4;
	
    /* send messages with delay between each to accomodate slower Wixel 
     * operating freq
     */
    SPI_write(0xAA);
    delay_us(10);
	SPI_write(split_msg[0]);
	delay_us(10);
	SPI_write(split_msg[1]);
	delay_us(10);
	SPI_write(split_msg[2]);
	delay_us(10);
	SPI_write(split_msg[3]);
	delay_us(10);
	SPI_write(split_msg[4]);
	//delay_us(5);
	//SPI_write(0xFF);	

    return ;
}

/** send_curr_data() --
 **     split ADC/DAC data into individual byte messages and send data via SPI
 **/      
void send_curr_data(struct Wixel_msg *data)
{
    int msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7 = 0;
    char split_msg[12];

    msg0 = data->forceSensor;
    msg1 = data->opticalSensor;
    msg2 = data->inductanceSensor;
    msg3 = data->speakerOut;
    msg4 = data->knob1;
    msg5 = data->knob2;
    msg6 = data->knob3;
    msg7 = data->tenTurnPot;

    /* split each 1.5 byte piece of data into one byte segments */
    split_msg[0] = (char)((msg0 & 0xFF0) >> 4);
    split_msg[1] = (char)((msg0 & 0xF) << 4 | (msg1 & 0xF00) >> 8);
    split_msg[2] = (char)(msg1 & 0xFF);
    split_msg[3] = (char)((msg2 & 0xFF0) >> 4);
    split_msg[4] = (char)((msg2 & 0xF) << 4 | (msg3 & 0xF00) >> 8);
    split_msg[5] = (char)(msg3 & 0xFF);
    split_msg[6] = (char)((msg4 & 0xFF0) >> 4);
    split_msg[7] = (char)((msg4 & 0xF) << 4 | (msg5 & 0xF00) >> 8);
    split_msg[8] = (char)(msg5 & 0xFF);
    split_msg[9] = (char)((msg6 & 0xFF0) >> 4);
    split_msg[10] = (char)((msg6 & 0xF) << 4 | (msg7 & 0xF00) >> 8);
    split_msg[11] = (char)(msg7 & 0xFF);

    /* send messages with 10us delay so Wixel can process accurately */	
	SPI_write(0xAA);
    for (i=0;i<12;i++)
    {
        delay_us(10);
        SPI_write(split_msg[i]);
    }
    delay_us(10);
    SPI_write(0xBB);

    return;
}

/** SPI_read() -- 
 **     Read from SPI receive register
 **/
int SPI_read(void)
{
	int message;
	message = SPIRX;
	return message;
}

/** read_sensors() --
 **     Initialize ADC transfer from specified channel and return decimal
 **     voltage
 **/
float read_sensors(int channel)
{
	float sensorVoltage;

	ADCCP = channel;
	ADCCON = 0x6A3;
	ADCCON &= ~(1 << 7);
	while (!ADCSTA){
        /* wait until conversion completes */
    }
	sensorVoltage = ADCinput(ADCDAT);
	return sensorVoltage;
}


/** read_store_sensors() --
 **     Initialize ADC transfer from specified channel and store results into 
 **     curr_data.
 **/
 float read_store_sensors(int channel, struct Wixel_msg *data)
 {
    /* -----------
    Pin and Port Assignments:

    ADC0 -- Honeywell FS01 Force Sensor
    ADC1 -- Front panel knob 1
    ADC2 -- Front panel knob 2
    ADC3 -- Front panel knob 3
    ADC4 -- IR proximity (optical) sensor
    ADC5 -- Speaker inductance circuit output
    ADC6 -- 10 turn pot

    DAC0 -- Speaker output
    */

    float sensorVoltage = 0.0;
    int voltageHex = 0;    /* raw ADC conversion result is 32 bits long */
    int voltage_int = 0;

    ADCCP = channel;
    ADCCON = 0x6A3;
    ADCCON &= ~(1 << 7);   /* clear bit 7 to stop additional conversions */
    while (!ADCSTA){
        /* wait until conversion completes */
    }
    voltageHex = ADCDAT;
    sensorVoltage = ADCinput(voltageHex);

    /* right shift ADC data 16 bits to obtain result on [0, FFF] range */
    voltage_int = voltageHex >> 16;

    /* store raw ADC results into curr_data and update sensor status array */
    if (channel == 0) 
    {
        data->forceSensor = voltage_int;
        sensor_status[0] = 1;
    }
    if (channel == 1) 
    {
        data->knob1 = voltage_int;
        sensor_status[4] = 1;
    }
    if (channel == 2) 
    {
        data->knob2 = voltage_int;
        sensor_status[5] = 1;
    }
    if (channel == 3) 
    {
        data->knob3 = voltage_int;
        sensor_status[6] = 1;
    }
    if (channel == 4) 
    {
        data->opticalSensor = voltage_int;
        sensor_status[1] = 1;
    }
    if (channel == 5) 
    {
        data->inductanceSensor = voltage_int;
        sensor_status[2] = 1;
    }
    if (channel == 6)
    {
        data->tenTurnPot = voltage_int;
        sensor_status[7] = 1;
    }

    return sensorVoltage;
 }

/** initial_read_store_sensors() --
 **     read/store sensor data so struct members contain some data, even if
 **     sensor is not read in the loop
 **/
void initial_read_store_sensors(struct Wixel_msg *data)
{
	read_store_sensors(0, data);
    read_store_sensors(1, data);
    read_store_sensors(2, data);
    read_store_sensors(3, data);
    read_store_sensors(4, data);
    read_store_sensors(5, data);
    read_store_sensors(6, data);
	return;
}

/** store_DAC_data() --
 **     store DAC data and flip sensor_status[] entry for speakerOut
 **/ 
void store_DAC_data(int rawDAC_hex, struct Wixel_msg *data)
 {
    int voltage_int = 0;

    voltage_int = (rawDAC_hex >> 16);
    // store speakerOut voltage and set sensor status
    data->speakerOut = voltage_int;
    sensor_status[3] = 1;

    return;
}

/** check_sensor_status() --
 **     check if sensor has been read this loop. If not, read/store result
 **/
 void check_sensor_status(void)
 {
    /* sensor_status array mirrors the order of Wixel_msg struct */
    for (i=0; i<8; i++)
    {
        if (!sensor_status[i])
        {
            /* read sensor if it hasn't yet been read */
            if (i == 0) read_store_sensors(0, pCurr_data);
            if (i == 1) read_store_sensors(4, pCurr_data);
            if (i == 2) read_store_sensors(5, pCurr_data);
            if (i == 3) {/*do nothing for speakerOut voltage */}
            if (i == 4) read_store_sensors(1, pCurr_data);
            if (i == 5) read_store_sensors(2, pCurr_data);
            if (i == 6) read_store_sensors(3, pCurr_data);
            if (i == 7) read_store_sensors(6, pCurr_data);
        }
		delay_us(25);
    }

    return; 
}

/** system_init() -- 
 **     Initialize ADuC7026 CPU, DAC, GPIOs 
 **/
void system_init(void)
{
    POWKEY1 = 0x01;					
 	POWCON = 0x00;        /* Configures CPU clock for 41.78 MHz, CD=0 */
 	POWKEY2 = 0xF4;	

	PLLKEY1 = 0xAA;
	PLLCON = 0x01;
	PLLKEY2 = 0x55;
	
    /* Connect internal 2.5V reference on Vref pin */
	REFCON = 0x01;
	
    /* Set DAC0 output to 0-V_ref(2.5V) range and turn DAC0 on */
    DAC0CON = 0x12;
	
	/* Set DAC0 output to 0-V_ref(2.5V) range and turn DAC0 on */
    DAC1CON = 0x12;

    /* Set DAC3 output to 0-V_ref(2.5V) range and turn DAC2 on */
    //DAC2CON = 0x12;

    /* Set DAC3 output to 0-V_ref(2.5V) range and turn DAC3 on */
    //DAC3CON = 0x12;   
	
    /* 
     * Set digital I/O port 0 as GPIO on pins P0.0-0.7 
     * Set all I/O pins on port 0 as *OUTPUT*, init LOW 
     */
    GP0CON = 0x00;		
	GP0DAT |= 0xFF000000;

    /* GPIO Port 1 is used for SPI, set in SPI_init() */

    /* 
     * Set I/O port 2 as GPIO on pins P2.0-2.7
     * Set all pins on port 2 as *OUTPUT*, init LOW 
     */
    GP2CON = 0x00;      
    GP2DAT = 0xFF000000;
	
	/*
     * Set digital I/O port 3 as GPIO on pins P3.0-3.7
     * Set all I/O pins on port 3 as *INPUT*, initialize LOW
	GP3CON = 0x00;
	GP3DAT = 0x00;
    */
}

/** ADCpoweron() --
 **     Power on ADC, software-timed wait 
 **/
void ADCpoweron(int time)
{
	ADCCON = 0x20;		  //Power-on the ADC
	delay_sw(time);		  //Wait for ADC to be fully powered on
}

/** SPI_init(): 
 **     Initialize ADuC7026 SPI system on GPIO Port 1 
 **/
void SPI_init(void)
{
    /* Set GPIO P1.x as SPI */
    /* P1.4 - 1.7 = SCLK, MISO, MOSI, CS, respectively */
    GP1CON = 0x22220000;
    SPIDIV = 0x05;
    
    /* 
     * SPI enabled, master mode, serial clock pulses at beginning of xfer, 
     * serial clock idles high, MSB transmitted first, transfer on write to
     * SPITX, interrupt when TX empty (SPI interrupt not enabled in IRQEN)
	 * overwrite SPIRX when new serial byte received
     */
    SPICON = 0x14F;
}

/** timer0_init() --
 **     Configures timer0 for while loop period keeping 
 **/
void timer0_init(void)
{
	// timer0 currently set to 750 us period
	timer0_Flag = 0;	// Re-initialize flag
	IRQEN |= 0x04;		// Re-initialize timer IRQ
	
	/*
	 * 0x1053 = 100us period at 41.78MHz clock freq (set in system_init());
	 * 0x20A6 for 200us; x30F9 for 300us; x519A for 500us; x7A67 for 750us;
	 * 0x829 for 50us; 0x414 for 25us; 0x20A for 12.5us
	 */
	T0LD = 0x7A67;
	
    /* enable timer0, set in periodic mode with multiplier = 1 */
    T0CON = 0xC0;
}

/** General_IRQ_Handler()
 **     specifies behavior when an IRQ condition is reached 
 **/
void General_IRQ_Handler(void)
{
    if ((IRQSIG & 0x04) == 0x4)
	{
        /* timer0 IRQ clear routine */
		timer0_Flag = 1;
		T0CLRI = 0xFF;
		return;
	}

	if ((IRQSIG & 0x8) == 0x8)
	{
        /* timer1 IRQ clear for us/ms delay */
		timer1_Flag = 1;
		T1CLRI = 0xFF;
		return;
	}
  
  /*  
  if ((IRQSIG & 0xD) == 0xD)      // SPI IRQ clear for transmit
  {
    //SPITX_flag = 1;
    return ;
  }
  */
    
}

/** pot_voltage() --
 **     Calculates voltage on a potentiometer. Returns decimal scaled 
 **     by fullscaleValue
 **/
float pot_voltage(float voltage, float fullscaleValue)
{
	float calculatedVoltage;
	calculatedVoltage = (voltage / (float) 2.52) * (fullscaleValue);
	return calculatedVoltage;
}

/** SPI_write() --
 **     Send a single 8 bit message over SPI on GP1
 **/
void SPI_write(unsigned char data)
{
    SPITX = data;
	while((SPISTA & 0x1) == 0x1) {
        /* wait while SPI is transmitting */
    }
    return;
}

/** waitForRestOfPeriod() --
 **     Maintains 200us cycle using timer0 IRQ and global flags 
 **/
void waitForRestOfPeriod(void)
{
	GP2DAT ^= 0x10000;  // Flip P2.0 low
                        // To know when waitForRestOfPeriod() begins.
	
	if (timer0_Flag == 1)
	{
		timer0_Flag = 0;	//Reset timer flag
		return;
	}
	else
	{
		while(timer0_Flag == 0){}	//Wait until interrupt request
		timer0_Flag = 0;
		return;
	}
}

/** ADCinput() --
 **     Converts ADC conversion HEX to decimal voltage. 
 **/
float ADCinput(int ADC_hex_result)
{
	float input_voltage;
	int ADC_hex_result_shifted;
    
    /* shift to right 16 bits to obtain base integer */
	ADC_hex_result_shifted = ADC_hex_result >> 16;

    /* "unit conversion" using 2.52V with ADC connected to voltmeter */
	input_voltage = ((ADC_hex_result_shifted)*2.52)/(0xFFF);

	return input_voltage; 
}

/** outputDAC() -- 
 **     Converts decimal voltage to HEX for DAC output. 
 **/
int outputDAC(float desired_voltage)	
{
    //"unit conversion" from desired decimal voltage to hex code
    // 2.4976 is voltage when 0xFFF applied to DAC
	int output_integer = floor((desired_voltage * 0xFFF) / 2.497);  
    
    /* majority of voltage range is [0.036, 2.49] */                                                        
    if (desired_voltage < 2.49 && desired_voltage >= 0.036)
    {
        
        /* Shift integer result 16 bits to left to conform to DACxDAT format */
        output_integer = output_integer << 16;
    }
    else if (desired_voltage >= 2.49)                                
    {    
        output_integer = 0xFFF;
        output_integer = output_integer << 16;
    }
    /* Voltages under 0.036V need a function to be more accurate */ 
    else if (desired_voltage < 0.036 && desired_voltage >= 0.0065)
    {
        /* Linear function obtained by excel calibration for small voltages */
        desired_voltage = 1.2 * desired_voltage - 0.0072733;
        output_integer = floor((desired_voltage * 0xFFF) / 2.497);   
        output_integer = output_integer << 16;
    }  
    /* The minimum voltage we can get */
    else if (desired_voltage < 0.0065)
    {
        output_integer = 0x00;
    }
    
    return output_integer;
}

/** delay_sw() --
 **     Software delay. time = 10 --> ~12us 
 **/
void delay_sw(int time)
{
	while (time >=0)
	time --;
}

/** delay_us() -- 
 **     Microsecond delay on timer1. Enables/disables timer1 within function 
 **/
void delay_us(unsigned long time)
{
	timer1_Flag = 0;
	T1CON = 0x0;
	
    /* enable timer1 IRQ */
	IRQEN |= 0x08;					    //Enable timer1 IRQ; timer1 = bit 3
	
    /*
     * T1LD = time (us) * clock freq (MHz)
     * (e.g. 500us * 41.78 MHz = 20890 - 1)
     * subtract 1 since timer load includes zero
     */								
	T1LD = ((time * 42782) >> 10) - 1;	//T1LD = time (us) * freq (in MHz) 
                                        //(e.g. 500us * 41.78 MHz = 20890-1) 
                                        // subtract 1 since includes zero
    
    /* initialize timer1, 41.78MHz, down mode, periodic */
	T1CON = 0xC0;					//Init timer1, 41.78MHz, down mode, periodic				
	while(timer1_Flag == 0){}		//Wait for timer flag flip
	timer1_Flag = 0;
	T1CON = 0x0;					//Disable timer1
	IRQCLR |= 0x08;
	return;
}

/** delay_ms() --
 **     Millisecond delay on timer1 using delay_us(). 
 **/
void delay_ms(unsigned long time)
{
	unsigned short i = 0;
    
	//Wait for 1 ms each time through loop
	for (i=0; i<time; i++)
	{
		delay_us(1000);
	}
}

/** testmode() -- 
 **     cycle DAC output up and down n times  
 **/
void testmode(int times)
{
	int n;
	float voltage;
	
	for (n=1; n<=times; ++n)  
	{
        // Cycle up to 2.49V
		for (voltage=1.25; voltage<=2.5; voltage=voltage+0.001)
		{
			DAC0DAT = outputDAC(voltage);
			delay_ms(2);
		}	
		
		delay_ms(50);
		
        // Cycle down to 0.0065V
		for (voltage=2.5; voltage>=0.0065; voltage=voltage-0.001)
		{
			DAC0DAT = outputDAC(voltage);
			delay_ms(2);
		}
		
		delay_ms(50);
		
        // Cycle up to 2.49V
		for (voltage=0.0065; voltage<=1.25; voltage=voltage+0.001)
		{
			DAC0DAT = outputDAC(voltage);
			delay_ms(2);
		}	
		
        // Return to midpoint
        DAC0DAT = outputDAC(1.25);
		
		delay_ms(50);
	}
	

	#if OBSOLETE
    /* ALTERNATE/OBSOLETE Implementation */
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
    #endif
}

/** sensor_avg() --
 **     takes 10 sample average of indicated channel 
 **/
float sensor_avg(int channel)
{
	int i;
	float avg_voltage, add_voltage = 0;
	float voltage[10];
    
	for (i=0; i<10; i++)
	{
		voltage[i] = read_store_sensors(channel, pCurr_data);
		add_voltage += voltage[i];
        delay_us(10);
	}
	avg_voltage = add_voltage / 10.0;

	return avg_voltage;
}

/* EOF ---------------------------------------------------------------------- */
