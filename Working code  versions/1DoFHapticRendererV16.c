/****************************************************************************
Randy Lee (ral63@pitt.edu)
Avi Marcovici (avi.marcovici@gmail.com)
Avin Khera (avinkhera@gmail.com)
Visualization and Image Analysis Laboratory
Department of Bioengineering
University of Pittsburgh

1 DoF Haptic Renderer MCU Control

Equipment:
- Analog Devices ADuC7026
- FaitalPRO 5FE120 speaker
- Pololu Wixel programmable USB module

Code version: v16

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
       Formatting and code cleaning
v13 -- Changed Virtual wall mode to resist movement @ rest position
       - Tyreus-Luyben PI & PID best - PID stiffer, PI least noisy
       Added triangle wave mode
       Added sine wave mode w/ variable amplitude and frequency
       Added zero stiffness spring
       - Z-N P control works
       Added membrane puncture
       Initialize PID constants before while loop begins
       Added slew rate limitation to virtual wall - limit max change in output
       - T-L PID best
       Added slew rate limitation to membrane puncture
v14 -- Decreased operating frequency to 1kHz (loop time = 1000 us)
       Increased delta_theta in sine wave generator to match 1 kHz op freq
       Cleaning in prep for release
       Added variable critical force to membrane puncture
       Added ratchet insertion/removal simulation
       SPIRX is now read after every transmission
       Incoming data is saved in a char array and parsed for control keys
       Added position dependent bias to mem puncture, ratchet
       Replaced ratchet simulation with manual control before HAPTICS 2016
v15 -- Updated virtual wall to be movable via manual knob control
       Changed limits of PID integral term from (+-)2.5V to (+-)1.25V
       Removed speaker bias term from virtual wall PID function
	   Updated virtual wall function to include ADC1 potentiometer reading for
	     displacing virtual wall
	   Updated set point to correspond to optical sensor position reading
v16 -- Updated SPI_parse_data() to separate control & GUI data inputs to ADuC
       Added g_virtual_knob1,2,3 global to store incoming data from GUI


Last updated: 23 May 2016
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
#define PI 3.14159265358979323846264338327950288

/* Global variables ------------------------------------------------------- */
/* 1DoF device data */
float force_volts, force_volts_avg;
float opticalPos_volts, opticalPos_mm;
float inductPos_volts, inductPos_mm;
int speaker_voltageHex;
float speaker_volts;
float desiredSpeaker_volts, lastDesiredSpeaker_volts;
float desiredSpeaker_mm, lastDesiredSpeaker_mm;
float rest_pos_volts, rest_pos_mm;

unsigned char g_speaker_mode = 0x99;           /* speaker mode */
int g_virtual_knob1, g_virtual_knob2, g_virtual_knob3 = 0;

/* each message received from Wixel/GUI is 1 byte in size */
unsigned char SPI_received_data[14];

/* linear fit variables for zero stiffness spring 
 * @TODO: Confirm -- OBSOLETE AS OF VXX (?)
 */
float forceSlope, intersectionPoint;

/* counter for SPI comm testing */
int counterTest_message = 0xA00;

/* testing for 8 data comms */  
char g_loop_count = 0x00;

/* define struct of three 12 bit messages to experiment with SPI sending */
struct Wixel_test{
    int Comp_1;
    int Comp_2;
    int Comp_3;
};

/* define struct for actual messages to be sent through wixel.
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
char sensor_status[8];

/* define struct for PID constants */
struct PID_const{
    float pGain;
    float iGain;
    float dGain;
    float K_u;
    float T_u;
    float setPoint;
    float lastError;
    float errorSum;
    float lastOutput;
    float speakerBias;
};

/* define struct for wave generators */
struct Wave_const{
    int curr_theta;
    int delta_theta;
    int theta_counter;
    float amplitude;
    float frequency;
};

/* structure of 3 integers for testing the Wixel */
struct Wixel_test test_messages, *pTest_msg;

/* structure of 8 data points to send through Wixel */
struct Wixel_msg curr_data, *pCurr_data;
struct Wixel_msg init_data, *pInit_data;
struct Wixel_msg test_data, *pTest_data;

/* structure of PID constants */
struct PID_const PID_zeroStiff, *pPID_zero;
struct PID_const PID_wall, *pPID_wall;
struct PID_const PID_membrane, *pPID_mem;
struct PID_const PID_ratchet, *pPID_ratchet;

/* structure of wave constants */
struct Wave_const Sine_const, *pSine_const;

/* Flags, counters, and misc ---------------------------------------------- */
/* timer flags are volatile because they are flipped by interrupt routines */
volatile signed int timer0_Flag = 0;
volatile signed int timer1_Flag = 0;
int flag = 0;
int changeFlag = 0;
int counter = 0;        /* loop counter */
int i = 0;              /* global loop counter */
//int message = 0;        /* SPI comm incoming message */

int g_membrane_punctured = 0;   /* flag for membrane puncture function */
int g_ratchet_punctured = 0;    /* flag for ratchet function */
//int g_ratchet_level = 1;    /* counter for ratchet level */
float epsilon = 0.02;

/* Function declarations -------------------------------------------------- */
/* system and components init */
void General_IRQ_Handler(void);
void system_init(void);
void timer0_init(void);
void SPI_init(void);
void SPI_write(unsigned char data);
unsigned char SPI_read(void);
void ADCpoweron(int software_delay_time);

/* data update and formatting functions for 3 channel SPI comm testing */
void dataUpdate_testmsg(struct Wixel_test *);
void dataUpdate_testformat(struct Wixel_test *);

/* data update and formatting functions for 8 channel SPI comms testing */
void dataUpdate_msg(struct Wixel_msg *data);
void send_test_data(struct Wixel_test *);
void parse_SPI_data(unsigned char *received_data);

/* 1DoF device sensor data read/store/format for Wixel */
void initial_read_store_sensors(struct Wixel_msg *data);
float read_store_sensors(int channel, struct Wixel_msg *data);
void store_DAC_data(int rawDAC_hex, struct Wixel_msg *data);
void send_curr_data(struct Wixel_msg *data, unsigned char *received_data);
void check_sensor_status(void);

/* system behavior functions */
void testmode(int times);
float PID_zeroStiffness(struct PID_const *);
float PID_virtualWall(struct PID_const *);
float membrane_puncture(struct PID_const *, struct PID_const *, 
                        struct PID_const *);
float ratchet_simulation(struct PID_const *, struct PID_const *,
                         struct PID_const *);
float sine_wave(struct Wave_const *);

/* sensor calibration/ manipulation functions */
float speakerDACtoDisplacement(float volts);
float opticalADCtoDisplacement(float volts);
float displacementToOpticalADC(float displacement_mm);
float displacementToSpeakerDAC(float displacement_mm);
float sensor_avg(int channel);


/* Imported HHFM functions ------------------------------------------------ */
/* timer settings and delay functions */
/* initialize main loop period time keeping */
void timer0_init(void);

/* wait function at end of main loop to limit loop duration using timer0 */
void waitForRestOfPeriod(void);

/* variable software delay using decreasing for-loop */
void delay_sw(int time);

/* variable us delay using timer1 */
void delay_us(unsigned long time);

/* variable ms delay using timer1 */
void delay_ms(unsigned long time);

/* sensor and output functions */
/* converts raw hex result from ADC conversion to decimal voltage */
float ADCinput(int ADC_hex_result);

/* converts desired decimal voltage to hex integer for DAC output */
int outputDAC(float desired_voltage);

/* read voltage on desired channel (ADC#). returns decimal voltage */
float read_sensors(int channel);

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
    
    /* Enable timer0 interrupt for computation-actuation loop timing */
    IRQEN = 0x04;               /* enable timer0 interrupt */
    IRQ = General_IRQ_Handler;  /* specify interrupt handler */
    
    /* Set up SPI on GPIO P1.4 - 1.7
     * SPI in master mode, clock pulses on xfer init
     * transmit on write to TX register
     * interrupt when TX empty
     */
    SPI_init();
    
    /* testmode() generates a triangular increase/decrease in speaker voltage 
     */
    //testmode(5);
    
    /* Initialize struct pointers */
    /* pTest_msg for updating test_msg struct using dataUpdate_testmsg() */
    pTest_msg = &test_messages;
    pTest_data = &test_data;

    /* ADC/DAC values that will be sent to Wixel or received from Wixel */  
    pCurr_data = &curr_data;
    
    /* initial sensor data */
    pInit_data = &init_data;

    /* PID controller values for zero stiffness and virtual wall modes */
    pPID_zero = &PID_zeroStiff;
    pPID_wall = &PID_wall;
    pPID_mem = &PID_membrane;
    pPID_ratchet = &PID_ratchet;

    /* wave constants */
    pSine_const = &Sine_const;
    pSine_const->theta_counter = 0; /* initialize theta_counter */

    /* initialize speaker output at 1.25 so sensors have a baseline */
    speaker_voltageHex = outputDAC(1.25);
    DAC0DAT = speaker_voltageHex;
    store_DAC_data(speaker_voltageHex, pCurr_data);
    
    /* second delay to allow sensors to stabilize after power on/ reset */
    delay_ms(1000);
    
    /* initial read/store of all sensors */
    initial_read_store_sensors(pInit_data);
    
    /* calculate and save average voltage from the optical sensor */
    rest_pos_volts = sensor_avg(4);
    rest_pos_mm = opticalADCtoDisplacement(rest_pos_volts);
    pInit_data->opticalSensor = rest_pos_volts;

    /* calculate and save the average voltage from the force sensor on ADC0 */
    force_volts_avg = sensor_avg(0);
    pInit_data->forceSensor = force_volts_avg;

    /* initialize K_u, T_u, and PID constants for virtual wall */
    #if OBSOLETE
    /*Tyreus Luyben PI control for virtual wall w/o slew rate limiting
     * k_p = K_u/3.2; k_i = k_p/(2.2*T_u)
     *  STABLE for out = pState - iState + dState+ 1.25
     *  STABLE for out = pState - iState + dState
     */
    pPID_wall->pGain = pPID_wall->K_u/3.2;
    pPID_wall->iGain = pPID_wall->pGain/(2.2*pPID_wall->T_u);
    pPID_wall->dGain = 0.0;
    #endif
    /*Tyreus Luyben PID control for virtual wall w/ slew rate limiting
     * k_p = K_u/2.2; k_i = k_p/(2.2*T_u); k_d = (k_p*T_u)/6.3
     *  STABLE w/ slew @ 25mV
     */
    pPID_wall->K_u = 1.861;
    pPID_wall->T_u = 0.01;
    pPID_wall->pGain = pPID_wall->K_u/2.2;
    pPID_wall->iGain = pPID_wall->pGain/(2.2*pPID_wall->T_u);
    pPID_wall->dGain = (pPID_wall->pGain*pPID_wall->T_u)/6.3;
    pPID_wall->setPoint = rest_pos_mm;
    pPID_wall->errorSum = 0.0;
    /*Added to old version, not necessary for PID control*/
    /*pPID_wall->speakerBias = 1.25;*/

    /* initialize K_u, T_u, and PID constants for zero stiffness */
    /* P control only for zero stiffness */
    pPID_zero->K_u = 2.870;
    pPID_zero->T_u = 0.002;
    pPID_zero->pGain = 0.50*pPID_zero->K_u;
    pPID_zero->iGain = 0.0;
    pPID_zero->dGain = 0.0;
    pPID_zero->setPoint = force_volts_avg;
    pPID_zero->errorSum = 0.0;
    
    /* Main loop ---------------------------------------------------------- */
    while(1) {
        /* start timer0 to time while loop duration
         * timer0 currently set for 1000 us --> 1 kHz operating frequency
         */
        timer0_init();

        /* flip P2.0 HIGH to measure duration of computation - actuation loop
         * P2.0 flipped LOW in waitForRestOfPeriod()
         */
        GP2DAT ^= 0x10000;

        /* reset sensor status array */
        for (i=0; i<8; i++)
        {
            sensor_status[i] = 0;
        }

        /* Update mode of 1DoF depending on message received from Wixel */
        if (g_speaker_mode == 0xAA)
        {
            /* sine wave */
            speaker_volts = sine_wave(pSine_const);
            speaker_voltageHex = outputDAC(speaker_volts);
            DAC0DAT = speaker_voltageHex;
            store_DAC_data(speaker_voltageHex, pCurr_data);
        }
        else if (g_speaker_mode == 0xBB)
        {
            /* virtual wall behavior */
            speaker_volts = PID_virtualWall(pPID_wall);
            speaker_voltageHex = outputDAC(speaker_volts);
            DAC0DAT = speaker_voltageHex;
            store_DAC_data(speaker_voltageHex, pCurr_data);
        }
        else if (g_speaker_mode == 0xCC)
        {
            /* zero stiffness behavior */
            speaker_volts = PID_zeroStiffness(pPID_zero);
            speaker_voltageHex = outputDAC(speaker_volts);
            DAC0DAT = speaker_voltageHex;
            store_DAC_data(speaker_voltageHex, pCurr_data);
        }
        else if (g_speaker_mode == 0xDD)
        {
            /* membrane puncture */
            speaker_volts = membrane_puncture(pPID_wall, pPID_zero, pPID_mem);
            speaker_voltageHex = outputDAC(speaker_volts);
            DAC0DAT = speaker_voltageHex;
            store_DAC_data(speaker_voltageHex, pCurr_data);
        }
        else if (g_speaker_mode == 0xEE)
        {
            #if TEMP_DISABLED
            speaker_volts = ratchet_simulation(pPID_wall, pPID_zero, 
                                               pPID_ratchet);
            speaker_voltageHex = outputDAC(speaker_volts);
            DAC0DAT = speaker_voltageHex;
            store_DAC_data(speaker_voltageHex, pCurr_data);
            #endif          

            /* control speaker voltage w/ knob 1 */
            speaker_volts = read_store_sensors(1, pCurr_data);
            speaker_voltageHex = outputDAC(speaker_volts);
            DAC0DAT = speaker_voltageHex;
            store_DAC_data(speaker_voltageHex, pCurr_data);
        }
        else
        {
            /* mute if incorrect/undefined mode received */
            speaker_voltageHex = outputDAC(1.25);
            DAC0DAT = speaker_voltageHex;
            store_DAC_data(speaker_voltageHex, pCurr_data);
        }

        /* check to see if any sensors have not yet been read */
        check_sensor_status();

        /* dataUpdate_msg() is used to test SPI comms w/ 8 12-bit messages */
        //dataUpdate_msg(pTest_data);
        //send_curr_data(pTest_data);

        /* send data to Wixel over SPI. Data already formatted to 12-bit
         * [0 - FFF] range. Split eight 12-bit messages into twelve 8-bit
         * messages. Save received data into an array to be parsed later
         */
        GP2DAT ^= 0x20000; /*flip P2.1 to examine time for each send/ read*/
        send_curr_data(pCurr_data, SPI_received_data);
        GP2DAT ^= 0x20000;

        #if OBSOLETE
        /* Read data from Wixel */
        while (!((SPISTA & 0x8) == 0x8)){
            /* wait while nothing in SPIRX */
        }
        message = SPI_read();
        
        /* only update mode if message falls within particular values */
        if (message == 0xAA || message == 0xBB || message == 0xCC ||
            message == 0xDD || message == 0xEE || message == 0x99)
        {
            g_speaker_mode = message;
        }
        #endif
        
        /* parse recieved SPI data for control keys and incoming GUI info
         * global variables are updated in this function
         */
        parse_SPI_data(SPI_received_data);
    
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
    float iMax = 1.25;
    float iMin = -1.25;
    float output_voltage;
    float currOptical_volts, currOptical_mm;
    float pState, iState, dState;
    float pGain, iGain, dGain;
    float error = 0.0;
    float slew_rate_limit = 0;
    
    /* Z-N tuning parameters */
    float K_u = 0.0;
    float T_u = 0.0;

    /* Obtain PID gains from knobs 1-3. Only update if knob has changed by
     * more than epsilon 
     */
    #if TEMP_DISABLED
    //pGain = read_store_sensors(1, pCurr_data);
    pGain = 1.0 + read_store_sensors(6, pCurr_data);
    //pGain = pot_voltage(pGain, 5.0);
    if (abs(pGain - PID->pGain) >= epsilon) PID->pGain = pGain;
    else pGain = PID->pGain;

    iGain = read_store_sensors(2, pCurr_data);
    if (abs(iGain - PID->iGain) >= epsilon) PID->iGain = iGain;
    else iGain = PID->iGain;
    
    dGain = read_store_sensors(3, pCurr_data);
    if (abs(dGain - PID->dGain) >= epsilon) PID->dGain = dGain;
    else dGain = PID->dGain;
    
    /* set dGain & iGain to zero for Ziegler Nichols tuning */
    iGain = 0.0;
    PID->iGain = 0.0;
    dGain = 0.0;
    PID->dGain = 0.0;
    #endif


    /* Speaker Setpoint Manual Control tuning. Adjusts setpoint over based
     * on user control of position via knob 1. Setpoint changes due to while
     * function's placement in while loop (in main)*/

    rest_pos_volts = read_store_sensors(1, pCurr_data);
    rest_pos_mm = pot_voltage(rest_pos_volts, 8.4);
    //rest_pos_mm = opticalADCtoDisplacement(rest_pos_volts);
    PID->setPoint = rest_pos_mm;


    
    /* Z-N tuning resulted in Ku = 1.861, Tu = 10ms = 0.010 s
     * PID: K_p = 0.60 Ku; K_i = 2K_p/T_u; K_d = 0.125*K_p*T_u
     * PI: K_p = 0.45K_u; K_i = 1.2*K_p/T_u
     * for the eqn: output = K_p*e + K_i*integral(e) + K_d*d/dt(e)
     */
    K_u = 1.861;    /* ultimate gain @ sustained oscillation */
    T_u = 0.01;     /* oscillation period */
     
    /* PID -- 
     *  UNSTABLE with slew rate limit @ 25 mV, 20ms period oscillation
     */
    //pGain = 0.60*K_u;
    //iGain = 2.0*pGain/T_u;
    //dGain = 0.125*pGain*T_u;
     
    /* PI -- 
     *  STABLE with slew rate limit @ 25mV
     */
    //pGain = K_u/2.2;  
    //iGain = 1.2*pGain/T_u;
    //dGain = 0.0;
    
    /* PD -- 
     *  UNSTABLE w/ slew @ 25mV, 25ms oscillation*/
    //pGain = 0.80*K_u;
    //iGain = 0.0;
    //dGain = pGain*T_u*0.125;

    /* P control only -- 
     *  UNSTABLE w/ slew @ 25mV, ~20ms oscillation
     */
    //pGain = 0.50*K_u;
    //iGain = 0.0;
    //dGain = 0.0;
    
    /* no overshoot Z-N 
     *  STABLE, close to maxed w/ slew @ 25mV
     */
    //pGain = 0.20*K_u;
    //iGain = 2.0*pGain/T_u;
    //dGain = pGain*T_u/3.0;
    
    /* Pessen integral
     *  UNSTABLE w/ slew @ 25mV, ~20ms oscillation
     */
    //pGain = 0.70*K_u;
    //iGain = (2.5*pGain)/T_u;
    //dGain = (3.0*pGain*T_u)/20.0;
    
    /*Tyreus Luyben PID
     * k_p = K_u/2.2; k_i = k_p/(2.2*T_u); k_d = (k_p*T_u)/6.3
     *  STABLE w/ slew @ 25mV
     */
    pGain = K_u/2.2;
    iGain = pGain/(2.2*T_u);
    dGain = (pGain*T_u)/6.3;
    
    /*tyreus Luyben PI
     * k_p = K_u/3.2; k_i = k_p/(2.2*T_u)
     *  STABLE w/ slew @ 25mV
     */
    //pGain = K_u/3.2;
    //iGain = pGain/(2.2*T_u);
    //dGain = 0.0;
    
    PID->pGain = pGain;
    PID->iGain = iGain;     
    PID->dGain = dGain; 
    
    /* read current position and calculate error */
    currOptical_volts = read_store_sensors(4, pCurr_data);
    currOptical_mm = opticalADCtoDisplacement(currOptical_volts);

    /* error calculated in mm. setPoint is set at system initialization */
    /* error is curr - setPoint becuase the control is negative,  
     *  i.e., to push up against external force, reduce speaker voltage
     */
    error = currOptical_mm - PID->setPoint;
    PID->errorSum += error;
    
    /* Proportional term */
    pState = pGain * error;

    /* Integral term */
    iState = iGain * (PID->errorSum);
    if (iState > iMax) iState = iMax;
    else if (iState < iMin) iState = iMin;
    
    /* Derivative term */
    dState = dGain * (error - PID->lastError);
    
    /* Bias PID controller with the speaker voltage midpoint
     * increasing voltage on speaker pushes down
     * optical position sensor increases as speaker pushes down
     */
        
    /* output eqn for when error = curr - set */

    /*set speaker bias equal to the reading off of the knob 1 setting*/
    output_voltage = pState - iState + dState;

    /* output eqn w/ speaker bias */
    /* noisier than naive implementation w/ 1.25 bias */
    //PID->speakerBias = displacementToSpeakerDAC(PID->setPoint);
    //output_voltage = pState - iState + dState + PID->speakerBias;
    
    /* limiter to block single loop impulses, checks diff between last
     * output and current output
     *      - noise acceptable @ 25mV limit, speaker can zero out in
     *        virtual wall
     */
    slew_rate_limit = 0.025;
    if (output_voltage - PID->lastOutput >= slew_rate_limit)
    {
        /* if new output > last by 100mV only increment by 100mV */
        output_voltage = PID->lastOutput + slew_rate_limit;
    }
    else if (output_voltage - PID->lastOutput <= -1.0 * slew_rate_limit)
    {
        /* if new output < last by more than 100mV, decrement by 100mV*/
        output_voltage = PID->lastOutput - slew_rate_limit;
    }
    
    PID->lastOutput = output_voltage;
    PID->lastError = error;
    return output_voltage;
}

/** PID_zeroStiffness() --
 **     PID control of force to keep force on sensor at its preload point
 **/
float PID_zeroStiffness(struct PID_const *PID) 
{
    float iMax = 2.5;
    float iMin = -2.5;
    float output_voltage, pState, iState, dState;
    float pGain, iGain, dGain;
    float error = 0.0;
    float curr_pos_volts, curr_pos;

    /* Z-N tuning parameters */
    float K_u = 0.0;
    float T_u = 0.0;
    
    #if TEMP_DISABLED
    /* read gains from knobs and only save if different from last loop */
    /* first knob -- proportional gain */
    pGain = 2.0 + read_store_sensors(6, pCurr_data);
    //pGain = pot_voltage(pGain, 10.0);
    if (abs(pGain - PID->pGain) > epsilon) PID->pGain = pGain;
    else pGain = PID->pGain;
    
    /* second knob -- integral gain */
    iGain = read_store_sensors(2, pCurr_data);
    if (abs(iGain - PID->iGain) > epsilon) PID->iGain = iGain;
    else iGain = PID->iGain;
    
    /* third knob -- differential gain */
    dGain = read_store_sensors(3, pCurr_data);
    if (abs(dGain - PID->dGain) > epsilon) PID->dGain = dGain;
    else dGain = PID->dGain;

    /* set dGain & iGain to zero for Ziegler Nichols tuning */
    iGain = 0.0;
    PID->iGain = 0.0;
    dGain = 0.0;
    PID->dGain = 0.0;
    #endif

    /* Z-N tuning resulted in Ku = 2.870, Tu = 2ms = 0.002 s
     * PID: K_p = 0.60 Ku; K_i = 2K_p/T_u; K_d = 0.125*K_p*T_u
     * PI: K_p = 0.45K_u; K_i = 1.2*K_p/T_u
     * for the eqn: output = K_p*e + K_i*integral(e) + K_d*d/dt(e)
     */
    K_u = 2.870;        /* ultimate gain @ sustained oscillation */
    T_u = 0.002;        /* oscillation period */
     
    /* PID -- 
     *   MAXED for out = pState - iState + dState + 1.25
     *   UNSTABLE for out = pState + iState + dState + 1.25
     */
    //pGain = 0.60*K_u;
    //iGain = 2.0*pGain/T_u;
    //dGain = 0.125*pGain*T_u;
     
    /* PI -- 
     *   if out = pState - iState + dState + 1.25
     *   if out = pState - iState + dState
     */
    //pGain = K_u / 2.2;    
    //iGain = 1.2*pGain/T_u;
    //dGain = 0.0;
    
    /* PD -- UNSTABLE */
    //pGain = 0.80*K_u;
    //iGain = 0.0;
    //dGain = pGain*T_u*0.125;

    /* P control only -- WORKS */
    pGain = 0.50*K_u;
    iGain = 0.0;
    dGain = 0.0;
    
    /* no overshoot Z-N 
     *   if out = pState - iState + dState
     *   if out = pState - iState + dState + 1.25
     */
    //pGain = 0.20*K_u;
    //iGain = 2.0*pGain/T_u;
    //dGain = pGain*T_u/3.0;
    
    /* Pessen integral
     *   for out = pState - iState + dState + 1.25
     *   for out = pState - iState + dState
     */
    //pGain = 0.70*K_u;
    //iGain = (2.5*pGain)/T_u;
    //dGain = (3.0*pGain*T_u)/20.0;
    
    /*Tyreus Luyben PID
     * k_p = K_u/2.2; k_i = k_p/(2.2*T_u); k_d = (k_p*T_u)/6.3
     *   for out = pState - iState + dState + 1.25
     *   for out = pState - iState + dState
     */
    //pGain = K_u/2.2;
    //iGain = pGain/(2.2*T_u);
    //dGain = (pGain*T_u)/6.3;
    
    /*tyreus Luyben PI
     * k_p = K_u/3.2; k_i = k_p/(2.2*T_u)
     *   MAXED for out = pState - iState + dState+ 1.25
     *   MAXED for out = pState + iState + dState + 1.25
     */
    //pGain = K_u/3.2;
    //iGain = pGain/(2.2*T_u);
    //dGain = 0.0;
    
    PID->pGain = pGain;
    PID->iGain = iGain;     
    PID->dGain = dGain;

    /* push --> voltage drop; pull --> voltage increase */
    force_volts = read_store_sensors(0, pCurr_data);
    curr_pos_volts = read_store_sensors(4, pCurr_data);
    curr_pos = opticalADCtoDisplacement(curr_pos_volts);
    
    /* Voltage-based error implementation */
    /* error calculated as setPoint - curr because control is positive,
     *   i.e., with push on sensor, increase speaker voltage to "run away"
     */
    error = PID->setPoint - force_volts;
    PID->errorSum += error;
    
    /* Integral Term */
    iState = iGain*(PID->errorSum);
    if (iState > iMax) iState = iMax;
    else if (iState < iMin) iState = iMin;
    
    /* Differential Term */
    dState = dGain * (error - PID->lastError);
    
    /* Proportional Term */ 
    pState = pGain * error;
    
    /* PID Controller Equation */
    output_voltage = pState + iState + dState + 1.25;
    
    /* PID Controller Equation w/ position dependent speaker bias */
    //PID->speakerBias = displacementToSpeakerDAC(curr_pos);
    //output_voltage = pState + iState + dState + PID->speakerBias;

    
    #if OBSOLETE
    /* OBSOLETE Implementation */
    /* Linear function controller implementation (original) --
     * Set up a linear function centered around the average voltage on
     * the force sensor input. Slope is emprically obtained, and a 
     * "y-intercept" for the function is calculated. Voltage on the speaker
     * (y) is calculated from this linear function with the sensor input (x). 
     * slope (pGain) in m = (y1-y0)/(x1-x0) form
     */
     pGain = 1.25 / (2.0 - force_volts_avg);
    
    /* at voltageForce_avg -> zero force generated by speaker
     * 2V is "lowest" voltage seen when pushing on force sensor
     * empirically obtained "y-intercept" of linear function for 
     * speaker actuation
     */
     intersectionPoint = 1.25 - pGain * voltageForce_avg;   
     output_voltage = (pGain*voltageForce) - iState + dState + intersectPoint; 
    
    /* the istate is negative to not get to inf
     * the proportional work like linear function with an offset(intersectPoint)
     */
    if (force_volts - force_volts_avg < -epsilon || 
        force_volts - force_volts_avg > epsilon)
    {
        /* epsilon is minimum voltage change on sensor before speaker moves 
         * -- "dead zone"
         * Push --> move speaker down by increasing voltage
         * Pull --> move speaker up by decreasing voltage
         * DAC0DAT = outputDAC(output_voltage);
         */
        output_voltage = (pGain*force_volts) + iState + 
                         (dGain*(error - PID->lastError)) + intersectionPoint;
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

/** membrane_puncture() --
 **     Simulate membrane puncture by rendering virtual wall until a certain
 **     increase or decrease in force on the force sensor. Then render
 **     a zero stiffness membrane.
 **/
float membrane_puncture(struct PID_const *PID_w, struct PID_const *PID_z,
                        struct PID_const *PID_m)
{
    float curr_pos, curr_pos_volts;
    float curr_force_volts;
    float applied_force;
    float spring_stiff, critical_force;

    /* membrane is 1mm thick, centered around init point */
    float membrane_top = PID_w->setPoint + 0.50;
    float membrane_bot = PID_w->setPoint - 0.50;
    float membrane_center = PID_w->setPoint;
    int in_membrane_flag = 0;
    
    float pState, iState, dState;
    float pGain, iGain, dGain;
    float error;
    float output_voltage;
    float slew_rate_limit;
    float iMax = 2.5;
    float iMin = -2.5;

    curr_pos_volts = read_store_sensors(4, pCurr_data);
    curr_pos = opticalADCtoDisplacement(curr_pos_volts);

    curr_force_volts = read_store_sensors(0, pCurr_data);
    applied_force = PID_z->setPoint - curr_force_volts;
    
    critical_force = pot_voltage(1, 1.50);
    
    if (curr_pos >= membrane_top || curr_pos <= membrane_bot)
    {
        /* above & below membrane, render zero stiffness */
        pGain = PID_z->pGain;
        iGain = PID_z->iGain;
        dGain = PID_z->dGain;

        /* outside membrane, user applied force is "error" */
        error = applied_force;

        /* "reset" error sum for membrane before/after puncture */
        PID_m->errorSum = 0.0;

        /* reset puncture flag */
        g_membrane_punctured = 0;
        in_membrane_flag = 0;
    }
    else if (curr_pos < membrane_top && curr_pos > membrane_bot)
    {   
        if (abs(applied_force) <= critical_force && g_membrane_punctured == 0)
        {
            in_membrane_flag = 1;

            /* use knob to adjust stiffness of spring. Stop at 0.90
             * because @ 1.0*K_u, speaker oscillates
             */
            //spring_stiff = pot_voltage(2, 0.95);
            
            /* render virtual spring using only proportional control */
            //pGain = PID_w->K_u * spring_stiff;
            pGain = PID_w->K_u * 0.50;  /* Z-N tuning for P control */
            iGain = 0.0;
            dGain = 0.0;

            /* inside membrane, user position w/r/t membrane is "error" */
            /* error is negative because virtual wall control is negative
             *   i.e. increasing voltage on speaker pushes down
             */
            if (curr_pos < membrane_top && curr_pos > membrane_center)
            {
                /* error < 0 so speaker pushes up as user pushes down */
                error = curr_pos - membrane_top;
            }
            else if (curr_pos > membrane_bot && curr_pos < membrane_center)
            {
                /* error > 0 so speaker pushes down as user pulls up */
                error = curr_pos - membrane_bot;
            }

            PID_m->errorSum += error;
        }
        else
        {
            /* if applied force greater than critical value, render zero
             * stiffness membrane and flip punture flag
             */
            pGain = PID_z->pGain;
            iGain = PID_z->iGain;
            dGain = PID_z->dGain;

            error = applied_force;
            PID_m->errorSum = 0.0;

            g_membrane_punctured = 1;
            in_membrane_flag = 0;
        }
    }

    /* save gains into membrane puncture struct */
    PID_m->pGain = pGain;
    PID_m->iGain = iGain;
    PID_m->dGain = dGain;

    /* Proportional Term */
    pState = pGain * error;

    /* Integral Term */
    iState = iGain * PID_m->errorSum;
    if (iState > iMax) iState = iMax;
    else if (iState < iMin) iState = iMin;

    /* Differential Term */
    dState = dGain * (error - PID_m->lastError);

    /* PID output equation */
    //output_voltage = pState + iState + dState + 1.25;
    
    /* PID output equation with position dependent speaker bias */
    if (in_membrane_flag)
    {
        output_voltage = pState + iState + dState + 1.25;
    }
    else
    {
        PID_m->speakerBias = displacementToSpeakerDAC(curr_pos);
        output_voltage = pState + iState + dState + PID_m->speakerBias;
    }
    
    /* limiter to block single loop impulses, checks diff between last
     * output and current output
     *      - noise acceptable @ 25mV limit, viscosity noticeable in
     *        zero stiffness regions
     *      - noise acceptable @ 50mV limit, viscosity reduced compared
     *        to limit @ 25mV
     */
    slew_rate_limit = 0.050;
    if (output_voltage - PID_m->lastOutput >= slew_rate_limit)
    {
        /* if new output > last by 50mV only increment by 50mV */
        output_voltage = PID_m->lastOutput + slew_rate_limit;
    }
    else if (output_voltage - PID_m->lastOutput <= -1.0 * slew_rate_limit)
    {
        /* if new output < last by more than 50mV, decrement by 50mV*/
        output_voltage = PID_m->lastOutput - slew_rate_limit;
    }

    PID_m->lastOutput = output_voltage;
    PID_m->lastError = error;
    return output_voltage;
}

/** ratchet_simulation() --
 **     Simulate the tactile feeling of inserting/ removing linear ratchet.
 **     Determine if current position is within a membrane, if so, set a 
 **     global flag and membrane gains. Else, set zero stiffness gains.
 **/
float ratchet_simulation(struct PID_const *PID_w, struct PID_const *PID_z,
                         struct PID_const *PID_r)
{
    float iMax = 2.5;
    float iMin = -2.5;
    float pGain, iGain, dGain;
    float pState, iState, dState;
    float output_voltage;
    float curr_pos, curr_pos_volts;
    float critical_force, applied_force, curr_force_volts;
    float slew_rate_limit;
    float error;

    /* membrane parameters in mm */
    float membrane_thickness = 0.50;
    float membrane_center[4] = {1.50, 3.50, 5.50, 7.50};
    float curr_center = 0.0;
    float curr_mem_top = 0.0;
    float curr_mem_bot = 0.0;
    int in_membrane = 0;

    curr_pos_volts = read_store_sensors(4, pCurr_data);
    curr_pos = opticalADCtoDisplacement(curr_pos_volts);

    curr_force_volts = read_store_sensors(0, pCurr_data);
    applied_force = PID_z->setPoint - curr_force_volts;

    //critical_force = pot_voltage(1, 0.50);
    critical_force = 0.50;

    /* determine if current position is within a membrane */
    for (i=0;i<4;i++)
    {
        if (curr_pos <= (membrane_center[i] + membrane_thickness/2.0) && 
            curr_pos >= (membrane_center[i] - membrane_thickness/2.0))
        {
            in_membrane = 1;

            curr_center = membrane_center[i];
            curr_mem_top = membrane_center[i] + membrane_thickness/2.0;
            curr_mem_bot = membrane_center[i] - membrane_thickness/2.0;
        }
    }

    /* set PID gains based on whether we are currently in a membrane */
    if (in_membrane)
    {
        if (abs(applied_force) < critical_force && g_ratchet_punctured == 0)
        {
            /* while in membrane, and force is less than critical, error
             * given by PID
             */
            pGain = PID_w->K_u *0.50;
            iGain = 0.0;
            dGain = 0.0;

            if (curr_pos > curr_center && curr_pos <= curr_mem_top)
            {
                error = curr_pos - curr_mem_top;
            }
            else if (curr_pos < curr_center && curr_pos >= curr_mem_bot)
            {
                error = curr_pos - curr_mem_bot;
            }
            PID_r->errorSum += error;
        }
        else
        {
            /* force has exceeded critical, render zero stiffness spring */
            g_ratchet_punctured = 1;

            pGain = PID_z->pGain;
            iGain = PID_z->iGain;
            dGain = PID_z->dGain;

            error = applied_force;
            PID_r->errorSum = 0.0;
        }

    }
    else
    {
        /* not inside any membrane, so set zero stiffness gains & reset 
         * pucnture flag 
         */
        pGain = PID_z->pGain;
        iGain = PID_z->iGain;
        dGain = PID_z->dGain;

        error = applied_force;
        PID_r->errorSum = 0.0;
        
        g_ratchet_punctured = 0;
    }

    PID_r->pGain = pGain;
    PID_r->iGain = iGain;
    PID_r->dGain = dGain;

    /* Proportional Term */
    pState = pGain * error;

    /* Integral Term */
    iState = iGain * PID_r->errorSum;
    if (iState > iMax) iState = iMax;
    else if (iState < iMin) iState = iMin;

    /* Differential Term */
    dState = dGain * (error - PID_r->lastError);

    /* PID output equation */
    //output_voltage = pState + iState + dState + 1.25;
    
    /* PID output equation with position dependent speaker bias */
    if (in_membrane)
    {
        output_voltage = pState + iState + dState + 1.25;
    }
    else
    {
        PID_r->speakerBias = displacementToSpeakerDAC(curr_pos);
        output_voltage = pState + iState + dState + PID_r->speakerBias;
    }

    /* limiter to block single loop impulses, checks diff between last
     * output and current output
     *      - noise acceptable @ 25mV limit, viscosity noticeable in
     *        zero stiffness regions
     *      - noise acceptable @ 50mV limit, viscosity reduced compared
     *        to limit @ 25mV
     */
    slew_rate_limit = 0.050;
    if (output_voltage - PID_r->lastOutput >= slew_rate_limit)
    {
        /* if new output > last by 50mV only increment by 50mV */
        output_voltage = PID_r->lastOutput + slew_rate_limit;
    }
    else if (output_voltage - PID_r->lastOutput <= -1.0 * slew_rate_limit)
    {
        /* if new output < last by more than 50mV, decrement by 50mV*/
        output_voltage = PID_r->lastOutput - slew_rate_limit;
    }

    PID_r->lastOutput = output_voltage;
    PID_r->lastError = error;
    return output_voltage;
}

/** sine_wave() --
 **     Generate sine wave at specified amplitude and frequency, determined
 **     by knobs 1 and 2
 **/
float sine_wave(struct Wave_const *constants)
{
    float amplitude = 0.0;
    float frequency = 0.0;
    float output = 0.0;
    float radians = 0.0;
    float voltage = 0.0;
    
    int curr_theta = 0;
    int delta_theta = 0;
    int fullscale = pow(2.0, 16);   /* fullscale number of bits */

    /* read knob 1 to get amplitude, centered around 1.25 */
    voltage = read_store_sensors(1, pCurr_data);
    amplitude = pot_voltage(voltage, 1.0);

    /* read knob 2 to get frequency & calculate the desired change in theta */
    voltage = read_store_sensors(2, pCurr_data);
    frequency = pot_voltage(voltage, 40);

    /* update current theta and convert to radians */
    /* delta_theta = fullscale / operating freq (hz), the change in theta to  
     * obtain a 1 Hz sine wave. For 16 bits @ 1333 Hz update, delta = 49
     */
    //delta_theta = 49 * frequency; /* del_theta for 1333 Hz, loop = 750us */
    delta_theta = 65 * frequency;   /* del_theta for 1000 Hz, loop = 1000us */
    constants->theta_counter += delta_theta;
    curr_theta = constants->theta_counter % fullscale;
    radians = curr_theta * (2.0 * PI) / fullscale;

    /* calculate output voltage, centered around midpoint */
    output = amplitude*sin(radians) + 1.25;

    return output;
}

/** speakerDACtoDisplacement() -- 
 **     Calculate expected speaker displacement from a reference voltage
 **/
float speakerDACtoDisplacement(float speaker_volts)
{
    float speaker_mm = 0.0;
    
    #if OBSOLETE
    /* calibration from 26 Feb 2016, displacement = 0 when DAC0 = 1.25 */
    speaker_mm = 2.9892*pow(speaker_volts, 2.0) - 13.329*speaker_volts + 
                 12.714;

    /* calibration from 17 March 2016, displacement = 0 when DAC0 = 2.5 */
    speaker_mm = 1.3413*pow(speaker_volts, 3.0) - 
                 3.9286*pow(speaker_volts, 2.0) - 1.96*speaker_volts + 
                 8.8222;
    #endif

    /* calibration from 28 March 2016, displacement = 0 when DAC0 = 2.5 */
    speaker_mm = 2.0994*pow(speaker_volts, 3.0) -
                 7.5992*pow(speaker_volts, 2.0) + 2.8832*speaker_volts +
                 8.0796;
    
    return speaker_mm;
}

/** opticalADCtoDisplacement() --
 **     Calculate speaker displacement from output voltage 
 **/
float opticalADCtoDisplacement(float optical_volts)
{
    float optical_mm = 0.0;
    
    #if OBSOLETE
    /* calibration from 26 Feb 2016, displacement = 0 when DAC0 = 1.25 */
    optical_mm = 3.1185*pow(optical_volts, 2.0) - 10.17*optical_volts + 
                 5.8269;
    
    /* calibration from 17 Mar 2016, displacement = 0 when DAC0 = 2.5 */
    optical_mm = 3.1825*pow(optical_volts, 2.0) - 10.32*optical_volts +
                 8.4069;
    #endif

    /* calibration from 28 Mar 2016, displacement = 0 when DAC0 = 2.5 */
    optical_mm = 2.1677*pow(optical_volts, 2.0) - 1.2489*optical_volts + 0.32;
    
    return optical_mm;
}

/** displacementToOpticalADC() --
 **     Calculate expected optical ADC voltage from speaker displacement 
 **/
float displacementToOpticalADC(float displacement_mm)
{
    float optical_ADC = 0.0;
    
    #if OBSOLETE
    /* calibration from 26 Feb 2016, displacement = 0 when DAC0 = 1.25 */
    optical_ADC = 0.0178*pow(displacement_mm, 2.0) - 0.2165*displacement_mm + 
                  0.7478;

    /* calibration from 17 Mar 2016, displacement = 0 when DAC0 = 2.5 */
    optical_ADC = 0.016*pow(displacement_mm, 2.0) - 0.3002*displacement_mm +
                  1.4241;
    #endif

    /* calibration from 28 Mar 2016, displacement = 0 when DAC0 = 2.5 */
    optical_ADC = -0.0194*pow(displacement_mm, 2.0) - 0.3633*displacement_mm +
                   0.4993;
    
    return optical_ADC;
}

/** displacementToSpeakerDAC() --
 **     Calculate the expected speaker bias for a given desired displacement
 **/
float displacementToSpeakerDAC(float displacement_mm)
{
    float speaker_DAC;

    /* calibration from 28 Mar 2016, displacement = 0 when DAC0 = 2.5 */
    speaker_DAC = -0.0161*pow(displacement_mm, 3.0) +
                   0.2130*pow(displacement_mm, 2.0) - 0.9592*displacement_mm +
                   2.6129;

    return speaker_DAC;
}

/** dataUpdate_msg() --
 **     Update and format test messages sent to Wixel
 **/
void dataUpdate_msg(struct Wixel_msg *data)
{
    /* Create test messages which follow similar format to 
     * dataUpdate_testmsg().
     * Start with [800, 901, A02, B03, C04, D05, E06, F07] --> 
     * [8FF, 900, A01,... etc]
     * the global counter g_loop_count is used to keep track btw loops
     */
    
    data->forceSensor = 0x800 + g_loop_count;
    data->opticalSensor = 0x900 + (g_loop_count + 1);
    data->inductanceSensor = 0xA00 + (g_loop_count + 2);
    data->speakerOut = 0xB00 + (g_loop_count + 3);
    data->knob1 = 0xC00 + (g_loop_count + 4);
    data->knob2 = 0xD00 + (g_loop_count + 5);
    data->knob3 = 0xE00 + (g_loop_count + 6);
    data->tenTurnPot = 0xF00 + (g_loop_count + 7);
    
    /* g_loop_count is char, so it returns to 00 after FF */
    g_loop_count++;
    
    return;
}

/** dataUpdate_testmsg() --
 **     Loop HEX integer from A00 to CFF amongst members of Wixel_testmsg
 **/
void dataUpdate_testmsg(struct Wixel_test *test_msg)
{
    /* Creates 3 test msgs which cycle, to test Wixel data logging interface
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

/** dataUpdate_testsend() --
 **     Modify message members to fit 12 bit format
 **/
void dataUpdate_testformat(struct Wixel_test *curr_msg)
{
    /*  Modify messages to fit 12 bit format */
    curr_msg->Comp_1 = curr_msg->Comp_1 >> 16;
    curr_msg->Comp_2 = curr_msg->Comp_2 >> 16;
    curr_msg->Comp_3 = curr_msg->Comp_3 >> 16;

    return;
}

/** send_test_data() -- 
 **     Splits 3 messages of 12 bits into 5 8-bit messages and send 
 **/
void send_test_data(struct Wixel_test *msg)
{
    int msg1, msg2, msg3;
    char split_msg[5];
    
    msg1 = msg->Comp_1;
    msg2 = msg->Comp_2;
    msg3 = msg->Comp_3;
    
    /* split three 12 bit messages into 5 using bit masking & shifting 
     * msg #5 is shifted left 4 bits so last nibble is 0000
     */
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
void send_curr_data(struct Wixel_msg *data, unsigned char *received_data)
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
    /* Read data from Wixel */
    *received_data = SPI_read();
    for (i=0;i<12;i++)
    {
        delay_us(10);
        SPI_write(split_msg[i]);
        
        /* Read data from Wixel */
        received_data[i + 1] = SPI_read();
    }
    delay_us(10);
    SPI_write(0xBB);
    /* read data from Wixel */
    received_data[13] = SPI_read();

    return;
}

/** parse_SPI_data() --
 **     Read stored SPI data from Wixel. Return the control key if contained
 **     in the received data
 **/
void parse_SPI_data(unsigned char *received_data)
{
    int message;
    unsigned short rec1, rec2, rec3;
    int loop_count, i = 0;
    int decoded_msg[8] = {0,0,0,0,0,0,0,0};

    /* received data is command = 1 byte, followed by 7 1.5 byte data, with 
     * 4bits left over. 8th decoded point should be 0x0BB
     */
	/* received data is organized as:
	 * rec_data[0] = ignored
	 * rec_data[1] = command mode
	 * rec_data[2:12] = data from GUI
	 * rec_data[13] = 0xBB
	 */
    for (i=0; i<4; i++)
    {
        /* extract three byte sections from rec_data to decode into two 1.5
         * byte data points, i.e. [A0],[1B],[02] --> [A01],[B02]
         */
        rec1 = *(received_data + i*3 + 2);
        rec2 = *(received_data + i*3 + 3);
        rec3 = *(received_data + i*3 + 4);

        /* mask extracted bytes to obtain 1.5 byte data */
        decoded_msg[i*2] = (rec1 << 4)  | ((rec2 & 0xF0) >> 4);
        decoded_msg[i*2+ 1] = ((rec2 & 0xF) << 8) | rec3;
    }
    
	/* speaker mode is first entry after 0xAA response */
    g_speaker_mode = *(received_data + 1);
	
    /* loop through all messages and assign globals depending on function
	 * decoded_msg starts with virtual knob 1 
	 */
    for (loop_count=0; loop_count<8; loop_count++)
    {
        message = decoded_msg[loop_count];
		
        if (loop_count == 0)
        {
            g_virtual_knob1 = message;
        }
        else if (loop_count == 1)
        {
            g_virtual_knob2 = message;
        }

        else if (loop_count == 2)
        {
            g_virtual_knob3 = message;
        }

        #if OBSOLETE
        if (message == 0xAA || message == 0xBB || message == 0xCC ||
            message == 0xDD || message == 0xEE || message == 0x99)
        {
            /* at most one control key in each batch of received data
             * so we can exit the function immediately after identifying it
             */
            g_speaker_mode = message;
        }
        #endif
    }
}

/** SPI_read() -- 
 **     Read from SPI receive register
 **/
unsigned char SPI_read(void)
{
    unsigned char message;
    while (!((SPISTA & 0x8) == 0x8)){
        /* wait while nothing in SPIRX */
    }
    message = SPIRX;
    return message;
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

/** read_sensors() --
 **     Initialize ADC transfer from specified channel and return decimal
 **     voltage
 **/
float read_sensors(int channel)
{
    float sensorVoltage;

    ADCCP = channel;
    ADCCON = 0x6A3;
    ADCCON &= ~(1 << 7);   /* clear bit 7 to stop additional conversions */
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
    int voltageHex = 0;    
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
        //data->knob1 = voltage_int;
        data->knob1 = g_virtual_knob1;
        sensor_status[4] = 1;
    }
    if (channel == 2) 
    {
        //data->knob2 = voltage_int;
        data->knob2 = g_virtual_knob2;
        sensor_status[5] = 1;
    }
    if (channel == 3) 
    {
        //data->knob3 = voltage_int;
        data->knob3 = g_virtual_knob3;
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
    /* store speakerOut voltage and set sensor status */
    data->speakerOut = voltage_int;
    sensor_status[3] = 1;

    return;
}

/** check_sensor_status() --
 **     check if sensor has been read this loop. If not, read/store result
 **/
 void check_sensor_status(void)
 {
    /* sensor_status array mirrors the order of Wixel_msg struct
     * [0 force_sensor, 1 optical_sensor, 2 inductance_sensor, 3 DAC_output,
     *  4 knob1, 5 knob2, 6 knob3, 7 10-turn knob]
     */
    for (i=0; i<8; i++)
    {
        if (!sensor_status[i])
        {
            /* read sensor if it hasn't yet been read */
            if (i == 0) read_store_sensors(0, pCurr_data);
            else if (i == 1) read_store_sensors(4, pCurr_data);
            else if (i == 2) read_store_sensors(5, pCurr_data);
            else if (i == 3) {/*do nothing for speakerOut voltage */}
            else if (i == 4) read_store_sensors(1, pCurr_data);
            else if (i == 5) read_store_sensors(2, pCurr_data);
            else if (i == 6) read_store_sensors(3, pCurr_data);
            else if (i == 7) read_store_sensors(6, pCurr_data);
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
    
    /* Set digital I/O port 0 as GPIO on pins P0.0-0.7 
     * Set all I/O pins on port 0 as *OUTPUT*, init LOW 
     */
    GP0CON = 0x00;      
    GP0DAT |= 0xFF000000;

    /* GPIO Port 1 is used for SPI, set in SPI_init() */

    /* Set I/O port 2 as GPIO on pins P2.0-2.7
     * Set all pins on port 2 as *OUTPUT*, init LOW 
     */
    GP2CON = 0x00;      
    GP2DAT = 0xFF000000;
    
    /* Set digital I/O port 3 as GPIO on pins P3.0-3.7
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
    ADCCON = 0x20;        /* Power-on the ADC */
    delay_sw(time);       /* Wait for ADC to be fully powered on */
}

/** SPI_init() --
 **     Initialize ADuC7026 SPI system on GPIO Port 1 
 **/
void SPI_init(void)
{
    /* Set GPIO P1.x as SPI */
    /* P1.4 - 1.7 = SCLK, MISO, MOSI, CS, respectively */
    GP1CON = 0x22220000;
    SPIDIV = 0x05;
    
    /* SPI enabled, master mode, serial clock pulses at beginning of xfer, 
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
    /* timer0 currently set to 1000 us period */
    timer0_Flag = 0;    /* Re-initialize flag */
    IRQEN |= 0x04;      /* Re-initialize timer IRQ */
    
    /* 0x1053 = 100us period at 41.78MHz clock freq (set in system_init());
     * 0x20A6 for 200us; x30F9 for 300us; x519A for 500us; x7A67 for 750us;
     * 0xA334 for 1000us; 0x14668 for 2000us; 
     * 0x829 for 50us; 0x414 for 25us; 0x20A for 12.5us
     */
    T0LD = 0xA334;
    
    /* enable timer0, set in periodic mode with multiplier = 1 */
    T0CON = 0xC0;
}

/** General_IRQ_Handler() --
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

    else if ((IRQSIG & 0x8) == 0x8)
    {
        /* timer1 IRQ clear for us/ms delay */
        timer1_Flag = 1;
        T1CLRI = 0xFF;
        return;
    }
  
    #if TEMP_DISABLED 
    if ((IRQSIG & 0xD) == 0xD)
    {
        /* SPI IRQ clear on transmit */
        //SPITX_flag = 1;
        return ;
    }
    #endif
    
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

/** waitForRestOfPeriod() --
 **     Maintains 200us cycle using timer0 IRQ and global flags 
 **/
void waitForRestOfPeriod(void)
{
    /* flip P2.0 low, to know when waitForRestOfPeriod() begins */
    GP2DAT ^= 0x10000;
    
    if (timer0_Flag == 1)
    {
        timer0_Flag = 0;    /* Reset timer flag */
        return;
    }
    else
    {
        while(timer0_Flag == 0){
            /* Wait until interrupt request */
        }
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
    /* "unit conversion" from desired decimal voltage to hex code
     *  2.4976 is voltage when 0xFFF applied to DAC
     */
    int output_integer = floor((desired_voltage * 0xFFF) / 2.497);  
    
    /* majority of voltage range is [0.036, 2.49] */                                                        
    if (desired_voltage < 2.49 && desired_voltage >= 0.036)
    {
        
        /* Shift integer result 16 bits to left to conform to DACxDAT format*/
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
    
    /* enable timer1 IRQ, timer1 = bit3 */
    IRQEN |= 0x08;
    
    /* T1LD = time (us) * clock freq (MHz)
     * (e.g. 500us * 41.78 MHz = 20890 - 1)
     * subtract 1 since timer load includes zero
     */                             
    T1LD = ((time * 42782) >> 10) - 1;
    
    /* initialize timer1, 41.78MHz, down mode, periodic */  
    T1CON = 0xC0;       
    while(timer1_Flag == 0){
        /* Wait for timer flag flip */
    }       
    timer1_Flag = 0;
    T1CON = 0x0;            /* Disable timer1 */
    IRQCLR |= 0x08;
    return;
}

/** delay_ms() --
 **     Millisecond delay on timer1 using delay_us(). 
 **/
void delay_ms(unsigned long time)
{
    unsigned short i = 0;
    
    /* Wait for 1 ms each time through loop */
    for (i=0; i<time; i++)
    {
        delay_us(1000);
    }
}

/** testmode() -- 
 **     Cycle DAC output up and down n times in a triangular wave  
 **/
void testmode(int times)
{
    int n;
    float voltage;
    
    for (n=1; n<=times; ++n)  
    {
        /* Cycle up to 2.49V */
        for (voltage=1.25; voltage<=2.5; voltage=voltage+0.001)
        {
            DAC0DAT = outputDAC(voltage);
            delay_ms(2);
        }   
        
        delay_ms(50);
        
        /* Cycle down to 0.0065V */
        for (voltage=2.5; voltage>=0.0065; voltage=voltage-0.001)
        {
            DAC0DAT = outputDAC(voltage);
            delay_ms(2);
        }
        
        delay_ms(50);
        
        /* Cycle up to 2.49V */
        for (voltage=0.0065; voltage<=1.25; voltage=voltage+0.001)
        {
            DAC0DAT = outputDAC(voltage);
            delay_ms(2);
        }   
        
        /* Return to midpoint */
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
 **     Takes 10 sample average of indicated channel 
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

/* EOF -------------------------------------------------------------------- */
