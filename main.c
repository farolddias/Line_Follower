/*******************************************************

Project : ME 445 Final Project
Version : V18
Date    : 12/16/21
Author  : Josh Kurkiewicz and Farold Dias
Company : University of Wisconsin - Madison
Comments: N/A

*******************************************************/
 
#include <pololu/3pi.h>
#include <avr/pgmspace.h>
#include <math.h>

unsigned int sensors[5]; // an array to hold sensor values
int error_past = 0;
long integral = 0;
int error;				//error in position from centerline
float Kp = 0.05;		//PID gains tuned for line-following at motor speed of 80
float Ki = 0.0001;
float Kd = 1.2;
int delta_error;			//Change in error
int manipulation;			//Closed-loop manipulation
const int motor_max = 80;	//Set max motor speed

int Kp_fixed;					//Fixed Point variables
int Ki_fixed;
int Kd_fixed;

int QI_Kp;
int QI_Ki;
int QI_Kd;

int QF_Kp;
int QF_Ki;
int QF_Kd;

int SWL = 8;

unsigned char S1 = 1 ;	//State variables
unsigned char S2;
unsigned char S3;
unsigned char S4;
unsigned char S5;
unsigned char S6;
unsigned char S7;
unsigned char S8;
unsigned char S9;
unsigned char S10;
unsigned char S11;
unsigned char S12;

unsigned char PBA;		//Push button variables
unsigned char PBB;
unsigned char PBC;
  
    
unsigned char CAL;		//State machine output variables
unsigned char CHOOSE;
unsigned char LEFT;
unsigned char RIGHT;
unsigned char STRAIGHT;
unsigned char FOLLOW;
unsigned char START;
 
//  The "PROGMEM" identifier causes the data to
// go into program space.
const char welcome_line1[] PROGMEM = " Pololu";
const char welcome_line2[] PROGMEM = "3\xf7 Robot";
const char demo_name_line1[] PROGMEM = "Line";
const char demo_name_line2[] PROGMEM = "Follow";
 
// A couple of simple tunes, stored in program space.
const char welcome[] PROGMEM = ">g32>>c32";
const char go[] PROGMEM = "L16 cdegreg4";
 
// Data for generating the characters used in load_custom_characters
// and display_readings. This display function is used in the Pololu demo code
const char levels[] PROGMEM = {
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111
};
 
// This function loads custom characters into the LCD. 
//This display function is used in the Pololu demo code
void load_custom_characters()
{
    lcd_load_custom_character(levels+0,0); // no offset, e.g. one bar
    lcd_load_custom_character(levels+1,1); // two bars
    lcd_load_custom_character(levels+2,2); // etc...
    lcd_load_custom_character(levels+3,3);
    lcd_load_custom_character(levels+4,4);
    lcd_load_custom_character(levels+5,5);
    lcd_load_custom_character(levels+6,6);
    clear(); // the LCD must be cleared for the characters to take effect
}
 
// This function displays the sensor readings using a bar graph. 
//This display function is used in the Pololu demo code
void display_readings(const unsigned int *calibrated_values)
{
    unsigned char i;
 
    for(i=0;i<5;i++) {
        // Initialize the array of characters that we will use for the
        // graph.  Using the space, an extra copy of the one-bar
        // character, and character 255 (a full black box), we get 10
        // characters in the array.
        const char display_characters[10] = {' ',0,0,1,2,3,4,5,6,255};
 
        // The variable c will have values from 0 to 9, since
        // calibrated values are in the range of 0 to 1000, and
        // 1000/101 is 9 with integer math.
        char c = display_characters[calibrated_values[i]/101];
 
        // Display the bar graph character.
        print_character(c);
    }
}
 
// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.
void initialize()
{
    unsigned int counter; // used as a calibration timer variable
    unsigned int sensors[5]; // an array to hold sensor values
 
    // This must be called at the beginning of 3pi code, to set up the
    // sensors. Use a value of 2000 for the timeout, which
    // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
    pololu_3pi_init(2000);
    load_custom_characters(); // load the custom characters
     
    // Play welcome music and display a message
    print_from_program_space(welcome_line1);
    lcd_goto_xy(0,1);
    print_from_program_space(welcome_line2);
    play_from_program_space(welcome);
    delay_ms(1000);
 
    clear();
    print_from_program_space(demo_name_line1);
    lcd_goto_xy(0,1);
    print_from_program_space(demo_name_line2);
    delay_ms(1000);
 
    // Display current battery voltage and Prompt to begin calibration
    while(!button_is_pressed(BUTTON_B))
    {
        int bat = read_battery_millivolts();
 
        clear();
        print_long(bat);
        print("mV");
        lcd_goto_xy(0,1);
        print("Press B");	
 
        delay_ms(100);
    }
	
	//Once button B has been pressed and released, begin calibration after short delay.
    wait_for_button_release(BUTTON_B);
    delay_ms(1000);
 
    // Auto-calibration: turn right and left while calibrating sensors.
    for(counter=0;counter<80;counter++)
    {
        if(counter < 20 || counter >= 60)
            set_motors(40,-40);
        else
            set_motors(-40,40);
 
        // This function records a set of sensor readings and records min and max reflectance values
		//Want to try calibrating robot path setup in a uniformly lit area.
        calibrate_line_sensors(IR_EMITTERS_ON);
 
        delay_ms(20);
    }
    set_motors(0,0);
 
    // Display calibrated values as a bar graph.
    while(!button_is_pressed(BUTTON_B))
    {
        // Read the sensor values and get the position measurement.
        unsigned int position = read_line(sensors,IR_EMITTERS_ON);
 
        // Display the position measurement, which will go from 0 to 4000 
        clear();
        print_long(position);
        lcd_goto_xy(0,1);
        display_readings(sensors);
 
        delay_ms(100);
    }
    wait_for_button_release(BUTTON_B);
 
    clear();
    print("Pizza");
    lcd_goto_xy(0,1);
    print("Time!");    
 
    // Play music and wait for it to finish before we start driving.
    play_from_program_space(go);
    while(is_playing());
}
 
// This is the main function, where the code starts.
int main()
{
    // set up the 3pi
    initialize();
	set_motors(0,0);
	
	//Calculating Fixed point values
	QI_Kp = floor(((log10(fabs(Kp)))/(log10(2))) +2);
	QI_Ki = floor(((log10(fabs(Ki)))/(log10(2))) +2);
	QI_Kd = floor(((log10(fabs(Kd)))/(log10(2))) +2);
	
	QF_Kp = SWL - QI_Kp;
	QF_Ki = SWL - QI_Ki;
	QF_Kd = SWL - QI_Kd;
	
	Kp_fixed = (int)(Kp*pow(2,QF_Kp));
	Ki_fixed = (int)(Ki*pow(2,QF_Ki));
	Kd_fixed = (int)(Kd*pow(2,QF_Kd));
 
    // This is the "main loop" - it will run forever.
    while(1)
    {
		
		//Push button statuses 
		PBA = button_is_pressed(BUTTON_A);
		PBB = button_is_pressed(BUTTON_B);
		PBC = button_is_pressed(BUTTON_C);
		
		//State Equations
		S1 = (S1||((S12&&!PBA&&!PBB&&!PBC)))&&!S2;
		S2 = (S2||(S1))&&!S3&&!S4&&!S5;
		S3 = (S3||(S2&&PBA))&&!S6;
		S4 = (S4||(S2&&PBB))&&!S7;
		S5 = (S5||(S2&&PBC))&&!S8;
		S6 = (S6||(S3&&(sensors[0]>200 && sensors[4]>200)))&&!S9;
		S7 = (S7||(S4&&(sensors[0]>200 && sensors[4]>200)))&&!S10;
		S8 = (S8||(S5&&(sensors[0]>200 && sensors[4]>200)))&&!S11;
		S9 = (S9||(S6&&(sensors[0]<100 && sensors[4]<100)))&&!S12;
		S10 = (S10||(S7&&(sensors[0]<100 && sensors[4]<100)))&&!S12;
		S11 = (S11||(S8&&(sensors[0]<100 && sensors[4]<100)))&&!S12;
		S12 = (S12||((S9&&sensors[0]<100&&sensors[1]<100&&sensors[2]<100&&sensors[3]<100&&sensors[4]<100)||
		(S10&&sensors[0]<100&&sensors[1]<100&&sensors[2]<100&&sensors[3]<100&&sensors[4]<100)||
		(S11&&sensors[0]<100&&sensors[1]<100&&sensors[2]<100&&sensors[3]<100&&sensors[4]<100)))&&!S1;
		
		//Outputs
		CHOOSE = S2 && !S1 && !S3 && !S4 && !S5 && !S6 && !S7 && !S8 && !S9 && !S10 && !S11 && !S12;
		LEFT = S6 && !S2 && !S3 && !S4 && !S5 && !S1 && !S7 && !S8 && !S9 && !S10 && !S11 && !S12;
		RIGHT = S8 && !S2 && !S3 && !S4 && !S5 && !S6 && !S7 && !S1 && !S9 && !S10 && !S11 && !S12;
		STRAIGHT = S7 && !S2 && !S3 && !S4 && !S5 && !S6 && !S1 && !S8 && !S9 && !S10 && !S11 && !S12;
		FOLLOW = (S3||S4||S5||S9||S10||S11) && !S1 && !S2 && !S6 && !S7 && !S8 && !S12; 
		
		
		//Allow user to choose desired path
		if (CHOOSE != 0x00){
			set_motors(0,0);
			clear();
			print("Choose");
			lcd_goto_xy(0,1);
			print("A,B,or C");
			
		}
		
		//Engage PID Line-Following Routine
		if (FOLLOW != 0x00){
			unsigned int position = read_line(sensors,IR_EMITTERS_ON);	//read current position with respect to centerline
			
			error = ((int)position) - 2000;	//Error from being perfectly centered
			delta_error = error - error_past;	//change in error
			integral += error;			//Accumulation of error
			error_past = error;			//Store previous error term
			
			//Floating Point Version
			/*manipulation = Kp*error + Ki*integral + Kd*delta_error;*/
			
			manipulation = ( (((long)Kp_fixed*error) << 9) + ((long)Ki_fixed*integral) + (((long)Kd_fixed*delta_error) << 14) ) >> 20;
			
			//Ensure manipulation does not drive motors faster than desired
			if (manipulation > motor_max){
				manipulation = motor_max;
			}
			if (manipulation < -motor_max){
				manipulation = -motor_max;
			}
			//Manipulate motor speed in designated wheel depending on position error feedback
			if (manipulation < 0){
				set_motors(motor_max + manipulation, motor_max);
			}
			else {
				set_motors(motor_max,motor_max - manipulation);
			}
		}
		//Sense Intersection and turn left until straight path detected
		if (LEFT != 0x00){
			read_line(sensors,IR_EMITTERS_ON);
			set_motors(0,60);
		}
		//Sense Intersection and continue straight until straight path detected
		if (RIGHT != 0x00){
			read_line(sensors,IR_EMITTERS_ON);
			set_motors(60,0);
		}
		//Sense Intersection and turn right until straight path detected
		if (STRAIGHT != 0x00){
			read_line(sensors,IR_EMITTERS_ON);
			set_motors(motor_max,motor_max);
		}
    }
}
