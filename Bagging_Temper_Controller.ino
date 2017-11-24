//#define DEBUG
//#define BUZZER

/*  Bagging and Tempering Controller

TFT 2.2'' connection diagram:
+--------------+-----------+
| Arduino pins | TFT pins  |
+--------------+-----------+
| VCC 3.3V     | VCC       | - TFT has an own LDO regulator, so 5v will work also.
| GND          | GND       |
| D5           | CS        | - 3.3v tolerant
| VCC 3.3V     | RESET     | - I tried to connect it to the RESET of Arduino. Also worked.
| D6           | D/C       | - 3.3v tolerant
| D11          | SDI(MOSI) | - 3.3v tolerant
| D13          | SCK       | - 3.3v tolerant
| VCC 3.3V     | LED       | - Specification said that 3.4v is ok there.
| D12          | SDO(MISO) | - 3.3v tolerant
+--------------+-----------+

BMP085/BMP180 pressure sensor connection diagram:
// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here


  Routines I need:
    menuPrintItem(Nr, Text)
    menuPrintItemValue(Nr, Text)
    menuPrintSelector(Nr)
    menuClearSelector(Nr)
    menuCnahgeItemValue()

    menuShowHelpLeft(Text)
    menuShowHelpRight(Text)
    
    byte rotaryEncRead(rotaryNr) / return the change in the ecoder: 127 - encoder is pressed, (-100/+100) change
*/

//#include <stdint.h>
//#include ".\TFTv2_Lite\TFTv2_Lite.h"
//#include ".\eRCaGuy_NewAnalogRead\eRCaGuy_NewAnalogRead.h"
#include "TFTv2_Lite.h"
#include "eRCaGuy_NewAnalogRead.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <EEPROMex.h>
//#include <PID_v1.h>
#include <avr/wdt.h>


// Voltage divider: 
// Vout=ADC*(Vref/1024)
// R2=(Vout * R1)/(Vin-Vout)


//Define Variables we'll be connecting to
//double PID_Setpoint, PID_Input, PID_Output;

//Specify the links and initial tuning parameters
//PID myPID(&PID_Input, &PID_Output, &PID_Setpoint,2,5,1, DIRECT);


// Where Menu starts
#define MENU_X        10
#define MENU_Y        30
#define MENU_WIDTH   300
#define MENU_SPACING  21

#define PT1000_PIN    A7
#define SSD1_PIN       7	// for temperature controlling
#define SSD2_PIN      A0	// for pressure controlling
#define BUZZER_PIN    A1
#define BUTTON0_PIN    4
#define BUTTON1_PIN    9
#define encoder0PinA   2	// Interrupt pin (coupled with capacitor to GND)
#define encoder0PinB   8
#define encoder1PinA   3	// Interrupt pin (coupled with capacitor to GND)
#define encoder1PinB  10

#define ENCODERS_BACKGROUND_COLOR 0x1147

#define EEPROM_ADDR_VIN  10 // address for storing (float)Vin value 
#define EEPROM_ADDR_GENESIS 100  // configuration for Genesis colors tempering
#define EEPROM_ADDR_WING    200 // configuration for Wing tempering
#define EEPROM_ADDR_BAGGING 300 // configuration for Wing Bagging
#define EEPROM_ADDR_LAST_PROCESS 400 // Last running process before Watchdog reset

// to identify encoder 
#define LEFT_ENCODER   0
#define RIGHT_ENCODER  1

// to identify, what to print on screen
#define TEMPERING_PROCESS 0
#define BAGGING_PROCESS   1

static boolean bmp_presence=LOW;  // 1 - present; 0 - not present
static boolean PT1000_presence=LOW; // 1 - present; 0 - not present

#define filterSamples   15             // filterSamples should  be an odd number, no smaller than 3
unsigned int PT1000_SmoothArray[filterSamples];   // array for holding raw sensor values for PT1000 sensor 
unsigned int BMP_SmoothArray[filterSamples];   // array for holding raw sensor values for Pressure sensor 
int PT1000rawADC; // variable updates every main loop iteration to get ADC readings for PT1000
float PT1000temperature = 0; // deg c variable that continuously updates from PT1000 sensor in main loop

unsigned long BMPreadmillis = 0;
float BMPtemperature=0;  // deg c variable that continuously updates from BMP sensor in main loop
int BMPpressure=0;  // hPa(mBar) variable that continuously updates from BMP sensor in main loop

unsigned long timing=0;

volatile char encoder0Pos = 0;
volatile byte encoder0Change = 0;
unsigned long encoder0millis = 0;

volatile char encoder1Pos = 0;
volatile byte encoder1Change = 0;
unsigned long encoder1millis = 0;

// menu related variables
byte menuTotalItems;
byte menuCurrentItemNr;
unsigned long MenuAutoSwitch_millis=0;	// counter for autoswitch back from settings screen to live data (in running process)
unsigned long LiveDataUpdate_millis=0;  // use this var to make update of the real data on the screen or menu not often than every 100 or 200 ms.



// VCC of Arduino. It is stored in EEPROM and can be adjusted. 
// The reason is to store it in EEPROM - to adjust it depending on ADC readings.
float Vin;  


Adafruit_BMP085 bmp;  // initialize bmp085 or bmp180

eRCaGuy_NewAnalogRead newadc;

//bool parallelProcess = LOW;	// If Bagging and Tempering running at the same time, then HIGH
//unsigned int backupProgramStage=0;  // backup variable programStage

unsigned int programStage=0;  // program flow (start menu/set params/work...)

char TempertmpString[30];  // Name of the tempering process is here

const char TemperNameGenesisPGM[] PROGMEM = "Temperature Control 1";
const char TemperNameWingPGM[] PROGMEM = "Temperature Control 2";
const char BaggingNameWingPGM[] PROGMEM = "Vacuum Bagging";

//Temper variables
unsigned long TemperTime_millis;  // var for counting Temper time (only pure tempering time)
unsigned long TemperTotalTime_millis;  // var for counting total time with heating and cooling time
unsigned long TemperIDLE_millis;  // var for counting IDLE time
unsigned long Temper_millis=0;
unsigned long TemperHyst_millis=0; // min time while heater not allowed to run (turn on or off)
byte Temper_tempsetstate;  // 1 - heater warmup, 2 - heater in stable, 0 - heater is cooling down
bool Temper_started = LOW; // variable for indicating the satus of the process
long Temper_totaltime; // Total time in seconds for tempering
long Temper_stage2time; // Rest of this time in seconds will be on another temperature
long Temper_heatertime; // Total time of heater working in seconds
byte Temper_temperature; // Max set temperature in C degress
byte Temper_stage2temperature; // Max set temperature in C degress on stage 2
float Temper_setcurtemp; // Curret set Temperature in C degress
float Temper_smoothchange;  // change in C degrees per minute
byte Temper_sensor;  // 0 - BMPxxx, 1 - PT1000
byte Temper_stages;  // determines do we have one stage (one temmperature), or two stages (for WING tempering)
// default task for multitaksing:
char TemperNameString[30] = "Temperature Control 2";  // Name of the tempering process is here
int TemperEEPROMaddr = EEPROM_ADDR_WING; // Address of the settings for selected temper process
float Temper_finishedvalue; // Here stored tmperature at proccess finish

//Bagging variables
unsigned long BaggingTime_millis;  // var for counting Bagging time
unsigned long BaggingIDLE_millis;  // var for counting IDLE time
unsigned long Bagging_millis=0;
unsigned long BaggingHyst_millis=0; // min time while pump not allowed to run
bool Bagging_started = LOW; // variable for indicating the satus of the process
long Bagging_totaltime; // Total time in seconds for bagging
long Bagging_pumptime; // Total time of pump working in seconds
int Bagging_pressure; // Set pressure in mbar
int Bagging_lastgoodpressure; // This variable helps to control that pump is really working
int Bagging_tolerance; // tolerance in witch boundaries to hold the pressure
byte Bagging_pumpresttime; // Some pumps need a rest time between runs (about 60 sec)
// default task for multitaksing:
char BaggingNameString[30] = "Vacuum Bagging";  // Name of the tempering process is here
int BaggingEEPROMaddr = EEPROM_ADDR_BAGGING; // Address of the settings for selected temper process
int Bagging_finishedvalue; // Here stored pressure at proccess finish


void setup()
{
    analogReference(INTERNAL);
    pinMode(SSD1_PIN, OUTPUT);
    digitalWrite(SSD1_PIN, LOW);
    pinMode(SSD2_PIN, OUTPUT);
    digitalWrite(SSD2_PIN, LOW);
    
    #ifdef BUZZER
	    pinMode(BUZZER_PIN, OUTPUT);
	    digitalWrite(BUZZER_PIN, LOW);
    #endif

    pinMode(encoder0PinA, INPUT_PULLUP); 
    pinMode(encoder0PinB, INPUT_PULLUP); 

    pinMode(encoder1PinA, INPUT_PULLUP); 
    pinMode(encoder1PinB, INPUT_PULLUP); 

    pinMode(BUTTON0_PIN, INPUT_PULLUP);
    pinMode(BUTTON1_PIN, INPUT_PULLUP);

	newadc.setBitsOfResolution(12);
  	newadc.setNumSamplesToAvg(1);
  	
    Vin = constrain(EEPROM.readFloat(EEPROM_ADDR_VIN),2,6); // if thre is a garbage then normalize it
    // of course, 2 volts or 6 volts is not normal. This is just extreme values. Later, in config we can adjust it


    attachInterrupt(0, doEncoder0, FALLING);  // encoder pin on interrupt 0 - pin 2
    attachInterrupt(1, doEncoder1, FALLING);  // encoder pin on interrupt 1 - pin 3
 
    Tft.TFTinit(); // init TFT library
    drawScreenTitle("Initialization...");
    clearTFTScreen(); // clears everything under the Title

    bmp_presence=getBMPpresense();
	if (bmp_presence) {
		Tft.Bcolor=DARK_GREEN;Tft.fillRectangle(50,90,220,48);
		Tft.Bcolor=BLACK;Tft.fillRectangle(55,95,210,38);
		Tft.setXY(60,100);Tft.Fcolor=WHITE;Tft.drawString_P(PSTR("BMP085/BMP180"));
		Tft.setXY(60,120);Tft.drawString_P(PSTR("sensor is detected!"));
		delay(1500);
	} else {
		printBMPXXXnotfound();
	}

    // initialize ADC readings from PT1000 (fill digitalsmooth array), and sma with pressure
    for(byte i=0;i<filterSamples*2;i++){
		PT1000rawADC = digitalSmooth(newadc.newAnalogRead(PT1000_PIN), PT1000_SmoothArray);
		BMPpressure = digitalSmooth(bmp.readPressure() / 100, BMP_SmoothArray);
    }
    PT1000_presence=getPT1000presense();
	
	// Resume Bagging process if it was interrupted by Watchdog (Don't yet know why Whatchdog kicks in)
	if(EEPROM.read(EEPROM_ADDR_LAST_PROCESS)==1){
		Bagging_started = HIGH;
		programStage = 400; // Start Bagging
	}
}

void loop() {
	WDT_Init();	// reset watchdog. Seems system alive.
	readSensors();  // read PT1000 and BMP temp and pressure.
	
	doTemperControl();	// will only run if Tempering process started
	doBaggingControl(); // will only run if Bagging process started
	
	if(programStage>=0 && programStage<=99) {
		doPresets();
	} else if (programStage>=100 && programStage<=199) {
		doTemperTFT(TemperNameString, TemperEEPROMaddr);
	//} else if (programStage>=200 && programStage<=299) {
	} else if (programStage>=300 && programStage<=399) {
		doConfig();
	} else if (programStage>=400 && programStage<=499) {
	    doBaggingTFT(BaggingNameString, BaggingEEPROMaddr);
	} else if (programStage>=500 && programStage<=599) {
		doHelp();
	}
}

//---------------- end of main Loop -----------------


// Clears TFT screen, but leaves Screen Title
void clearTFTScreen() {
  Tft.Bcolor=BLACK;
  Tft.fillRectangle(0,17,319,239);
}


const unsigned char Tempering_bitmap[30] PROGMEM= {
	0b00110000,0b00000000,
	0b01001011,0b10011000,
	0b01001011,0b10111100,
	0b01001011,0b10110000,
	0b01001000,0b00110000,
	0b01001000,0b00111100,
	0b01001000,0b00011000,
	0b01111000,0b00000000,
	0b01111000,0b00000000,
	0b01111000,0b00000000,
	0b01111000,0b00000000,
	0b01111000,0b00000000,
	0b11111100,0b00000000,
	0b11111100,0b00000000,
	0b01111000,0b00000000
};

const unsigned char Bagging_bitmap[30] PROGMEM= {
	0b00000011,0b11000000,
	0b00000100,0b00100000,
	0b00001000,0b00010000,
	0b00000000,0b00010000,
	0b00000000,0b00010000,
	0b00000000,0b10010000,
	0b00000010,0b11100000,
	0b00111110,0b11100000,
	0b01000111,0b11110000,
	0b10001111,0b10000000,
	0b10000001,0b11000001,
	0b10000011,0b11100001,
	0b10000000,0b10000001,
	0b01000000,0b01000010,
	0b00100000,0b00111100
};

void drawScreenTitle(char* title) {
  Tft.Bcolor=GRAY2;
    Tft.fillRectangle(0,0,319,16);
	Tft.X = 300;
	if(Temper_started){Tft.Fcolor=DARK_RED;Tft.drawBitmap(Tft.X, 1, Tempering_bitmap, 16, 15);Tft.X-= 20;}
	if(Bagging_started){Tft.Fcolor=CYAN;Tft.drawBitmap(Tft.X, 1, Bagging_bitmap, 16, 15);}
    Tft.Fcolor=WHITE;
    Tft.setXY(5,2);
    Tft.bold = FONT_BOLD;
    Tft.drawString(title);
    Tft.bold = FONT_NORMAL;Tft.Bcolor=BLACK;

}


void menuPrintItem(byte mnuNr, const char* mnuText) {
  Tft.Fcolor=WHITE;
  Tft.X = MENU_X + 3; // leave space for selector
  Tft.Y = MENU_Y + (mnuNr-1) * MENU_SPACING + 3; // also leave space for selector
   while(pgm_read_byte(mnuText)!='\0')
   {
       Tft.drawChar(pgm_read_byte(mnuText));
       *mnuText++;
   }
  //Tft.drawString(mnuText);
}

// function only sets coordinates. Value printed with regular Tft.draw... in main code
void menuPrintItemValue(byte mnuNr) {
  Tft.Fcolor=BLUE_SKY;
  Tft.X = MENU_X + MENU_WIDTH - 90; // 
  Tft.Y = MENU_Y + (mnuNr-1) * MENU_SPACING + 3; // also leave space for selector
  //Tft.drawString(mnuText);
  
}

void menuPrintSelector(byte mnuNr) {
  Tft.Fcolor=YELLOW;
  Tft.drawRectangle( MENU_X, MENU_Y + (mnuNr-1) * MENU_SPACING, MENU_WIDTH, 18); // 14 - height of the symbols
}

void menuClearSelector(byte mnuNr) {
  Tft.Fcolor=BLACK;
  Tft.drawRectangle( MENU_X, MENU_Y + (mnuNr-1) * MENU_SPACING, MENU_WIDTH, 18); // 14 - height of the symbols
}


// 16x13
const unsigned char RotateKnob_bitmap[26] PROGMEM= {
	0b00000111,0b11100000,
	0b00011111,0b11111000,
	0b00111100,0b00111100,
	0b00110000,0b00001100,
	0b01110000,0b00001110,
	0b01100000,0b00000110,
	0b01100000,0b00000110,
	0b01100100,0b00100110,
	0b01101100,0b00110110,
	0b01111100,0b00111110,
	0b00111100,0b00111100,
	0b01111100,0b00111110,
	0b11111100,0b00111111
};

// 7x13
const unsigned char PushKnob_bitmap[13] PROGMEM= {
	0b00111000,
	0b00111000,
	0b00111000,
	0b11111110,
	0b01111100,
	0b00111000,
	0b00010000,
	0b00000000,
	0b00000000,
	0b11111110,
	0b11111110,
	0b11000110,
	0b11000110
};
/*const unsigned char RotateKnob_bitmap[14] PROGMEM= {
	0b11111111,
	0b11001101,
	0b10000001,
	0b10111001,
	0b11110001,
	0b11111111,
	0b11100111,
	0b11100111,
	0b11111111,
	0b10001111,
	0b10011101,
	0b10000001,
	0b10110011,
	0b11111111
};

const unsigned char PushKnob_bitmap[14] PROGMEM= {
	0b11111111,
	0b11100111,
	0b11100111,
	0b11100111,
	0b10000001,
	0b11000011,
	0b11100111,
	0b11111111,
	0b11111111,
	0b10000001,
	0b10000001,
	0b10011001,
	0b10011001,
	0b11111111
};
*/

void menuShowEncoder(char* rotaryText, char* pushText, byte textAlign, byte encoder) {
	// draw a green or red Knob
	const int knobXcntr = (encoder==LEFT_ENCODER) ? 22 : 297;
	const byte knobYcntr = 200;
	// first, draw a background
	Tft.Bcolor=ENCODERS_BACKGROUND_COLOR;
	Tft.Fcolor=ENCODERS_BACKGROUND_COLOR;
	Tft.fillCircle(knobXcntr,knobYcntr,22);
	Tft.fillRectangle(knobXcntr-22,knobYcntr,knobXcntr+22,215);
	if(encoder==0){Tft.fillRectangle(0,215,160,239);}else{Tft.fillRectangle(160,215,319,239);}
	
	// Now print a text
	Tft.X=knobXcntr-textAlign;Tft.Y = knobYcntr + 22;
	//Tft.setXY(knobXcntr-textAlign,knobYcntr + 22);
	//if(rotaryText[0]!=' '){Tft.Fcolor=GRAY2;Tft.drawBitmap(Tft.X-1, Tft.Y, RotateKnob_bitmap, 8, 14);Tft.X+=11;Tft.Fcolor=WHITE;Tft.drawString(rotaryText);Tft.X+=5;}
	//if(pushText[0]!=' '){Tft.Fcolor=GRAY2;Tft.drawBitmap(Tft.X-1, Tft.Y, PushKnob_bitmap, 8, 14);Tft.X+=11;Tft.Fcolor=WHITE;Tft.drawString(pushText);}
	if(rotaryText[0]!=' '){Tft.Fcolor=GRAY2;Tft.drawBitmap(Tft.X-1, Tft.Y, RotateKnob_bitmap, 16, 13);Tft.X+=18;Tft.Fcolor=WHITE;Tft.drawString(rotaryText);Tft.X+=5;}
	if(pushText[0]!=' '){Tft.Fcolor=GRAY2;Tft.drawBitmap(Tft.X-1, Tft.Y, PushKnob_bitmap, 7, 13);Tft.X+=9;Tft.Fcolor=WHITE;Tft.drawString(pushText);}
	
	// At the end draw a knob
	Tft.Fcolor=GRAY1; Tft.fillCircle(knobXcntr,knobYcntr,17);
	Tft.Fcolor=(encoder==LEFT_ENCODER) ? DARK_GREEN : DARK_RED; Tft.fillCircle(knobXcntr,knobYcntr,15);
	Tft.Bcolor=(encoder==LEFT_ENCODER) ? GREEN : RED; Tft.fillRectangle(knobXcntr-2,knobYcntr-16,4,10);
	Tft.Fcolor=WHITE;Tft.Bcolor=BLACK;
}

// routine can return more than 1 steps per call...
// Encoder data read by Interrupts, so, if pause between calling this procedure is high, 
//   then return value can be greather than 1
// 127 means button is pressed
char rotaryEncRead(byte rotaryNr) {
  noInterrupts();
  char tmp = (rotaryNr==LEFT_ENCODER) ? encoder0Pos : encoder1Pos;
  if(rotaryNr==LEFT_ENCODER){encoder0Pos=0;}else{encoder1Pos=0;}  // reset encoders
  interrupts();
  if(!digitalRead((rotaryNr==LEFT_ENCODER) ? BUTTON0_PIN : BUTTON1_PIN)){tmp=127;}
  return tmp;
}

void doEncoder0() {
    // If interrupts come faster than 5ms, assume it's a bounce and ignore
    if (millis() - encoder0millis > 5) {
        if (!digitalRead(encoder0PinB))
            encoder0Pos++;
        else
            encoder0Pos--;
        }
    encoder0millis = millis();
  // here is code to process encode change...
  encoder0Change = 1;
}

void doEncoder1() {
    // If interrupts come faster than 5ms, assume it's a bounce and ignore
    if (millis() - encoder1millis > 5) {
        if (!digitalRead(encoder1PinB))
            encoder1Pos++;
        else
            encoder1Pos--;
        }
    encoder1millis = millis();
  // here is code to process encode change...
  encoder1Change = 1;
}


/* Gets temperature in C from resistance of PT1000 sensor
 PT1000 temperature sensor lookup table:
 from 0-170 deg table (in 5th steps).
 total is 34 numbers with 5 deg steps
 resistance 1000 is 0 deg
*/
const unsigned char PT1000_lookup[34] PROGMEM= {19,20,19,20,19,20,19,19,20,19,19,19,20,19,19,19,19,19,19,19,19,19,19,19,18,19,19,19,18,19,19,18,19,19};
float getPT1000temperature(float PTresistance) {
	float PTtableVal = 1000.0; // base is 0 deg
	byte n;
	byte tmppgm;
	float PTcoeff=0.2777; // Default coeff for out of range values (to get some temperature)
	for(n=0;n<34;n++) {
		tmppgm = pgm_read_byte(&PT1000_lookup[n]);
		if((PTtableVal+(float)tmppgm)>PTresistance){
			PTcoeff = 5.0/(float)tmppgm;
			break; // exit from for loop
		}
		PTtableVal+=(float)tmppgm;
	}
	// calculate temperature:
	return (PTresistance-PTtableVal)*PTcoeff+(float)n*5.0;
}

// Gets resistance in Ohms from ADC value
// R1 = 3270, Aref = 1.115V, Vin = 3.32V (stored in EEPROM and can be adjusted to get precise readings).
// R2(PT1000) will be calculated
// Vin is initialized at the beginning
float getPT1000resistance(int pADC) {
	#define Aref  1.115  // Internal volage reference
	#define R1  3270.0  // R1 value in divider
	// first of all lets calculate Vout from divider 
	float Vout = (float)pADC * ((float)Aref/newadc.getMaxPossibleReading());
	// now lets calculate resistance of PT1000
	return (Vout * (float)R1) / (Vin-Vout);
}

bool getBMPpresense() {
  return bmp.begin(BMP085_HIGHRES);	// 14 ms conversion time (default is 26 ms). Libray halts the program for this period of time.
}

bool getPT1000presense() {
  return (PT1000rawADC>4000) ? LOW : HIGH;
}

// read ADC of PT1000 and BMP temp and pressure.
void readSensors(){ 
  // read ADC value of PT1000
  PT1000rawADC = digitalSmooth(newadc.newAnalogRead(PT1000_PIN), PT1000_SmoothArray);
  PT1000temperature=getPT1000temperature(getPT1000resistance(PT1000rawADC));
  // read BMP data every 30ms
  if((BMPreadmillis+30) < millis() && bmp_presence) {
    BMPreadmillis=millis();
	BMPpressure = digitalSmooth(bmp.readPressure() / 100, BMP_SmoothArray); // convert Pa to hPa (mBar)
    BMPtemperature = bmp.readTemperature();
  }
}

// smooth algorytm for ADC reading
unsigned int digitalSmooth(unsigned int rawIn, unsigned int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  unsigned int j, k, temp, top, bottom;
  long total;
  static int i;
  static unsigned int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
  }

  return total / k;    // divide by number of samples
}


void doConfig() {
	wdt_disable();	// disable WDT until we in config. It is safe.
	drawScreenTitle("Config");
	clearTFTScreen(); // clears everything under the Title
	menuShowEncoder("Adjust","Store", 20, LEFT_ENCODER); // number indicates text move to the left (centering)
	menuShowEncoder(" ","Cancel", 40, RIGHT_ENCODER); // number indicates text move to the left (centering)
	Tft.setXY(5,30);Tft.bold=FONT_BOLD;Tft.drawString_P(PSTR("Adjust readings of PT1000."));Tft.bold=FONT_NORMAL;
	Tft.setXY(5,60);Tft.drawString_P(PSTR("Compare temperatures below"));
	Tft.setXY(5,80);Tft.drawString_P(PSTR("and adjust PT1000 readings as close"));
	Tft.setXY(5,100);Tft.drawString_P(PSTR("as possible to BMPxxx readings."));
	Tft.setXY(5,120);Tft.drawString_P(PSTR("Difference in 0.5 deg is ok."));
	
	Tft.setXY(80,155);Tft.drawString_P(PSTR("BMPxxx:"));
	Tft.setXY(80,175);Tft.drawString_P(PSTR("PT1000:"));
	Tft.Fcolor=DARK_GREEN;Tft.drawLine(75,195,215,195);Tft.Fcolor=WHITE;
	Tft.setXY(80,200);Tft.drawString_P(PSTR("difference:"));
	char encVal = 0;  // signed value - nothing is pressed
	while (rotaryEncRead(1) != 127) {
		readSensors();
		encVal = rotaryEncRead(0);  // read left encoder
		if(encVal!=0 && encVal!=127) {
			Vin+=(float)encVal / 100.0; // go down or up with voltage value
		}
		if(encVal==127){	// store is pressed
			drawScreenTitle("Config - DONE!");
			EEPROM.updateFloat(EEPROM_ADDR_VIN, Vin);
			delay(1500);
			//reboot();
			break;	// exit after save
		}
		Tft.Fcolor=YELLOW;
		Tft.setXY(175,155);Tft.drawFloat(BMPtemperature,1);Tft.drawString_P(PSTR("    "));
		Tft.setXY(175,175);Tft.drawFloat(PT1000temperature,1);Tft.drawString_P(PSTR("    "));
		float tmpf=abs(BMPtemperature-PT1000temperature);
		Tft.Fcolor = (tmpf<=0.5) ? GREEN : RED;Tft.setXY(175,200);Tft.drawFloat(tmpf,1);Tft.drawString("    ");

		Tft.Fcolor=GRAY1;Tft.Bcolor=GRAY2;Tft.setXY(250,2);Tft.drawString_P(PSTR("Vin: "));Tft.drawFloat(Vin,2);Tft.Bcolor=BLACK;
		#ifdef DEBUG
			Tft.X=130;Tft.drawString_P(PSTR("Ohm: "));Tft.drawFloat(getPT1000resistance(PT1000rawADC),2);Tft.drawString_P(PSTR("  "));
		#endif
	}
	programStage=0; // exit to presets menu
}


void doPresets() {
	switch (programStage) {
		case 0:
			menuTotalItems = 5;
		    menuCurrentItemNr = 1;
		
		    drawScreenTitle("Presets");
		    clearTFTScreen(); // clears everything under the Title
		    menuShowEncoder("Select","Choose", 20, LEFT_ENCODER);
		    menuPrintItem(1,PSTR("Temperature Control 1"));
		    menuPrintItem(2,PSTR("Temperature Control 2"));
		    menuPrintItem(3,PSTR("Vacuum Bagging"));
		    menuPrintItem(4,PSTR("Config"));
		    menuPrintItem(5,PSTR("Help"));
		    menuPrintSelector(menuCurrentItemNr);
		    programStage = 1;
		case 1:
		    // we should check for the left rotary encoder...
		    char encVal = 0;  // signed value - nothing is pressed
		    encVal = rotaryEncRead(0);  // read left encoder
		    if(encVal!=0 && encVal!=127) {
		        // go down or up in menu
		        goDownUpInMenu(encVal);
		    } else if(encVal==127) {	// Knob is pressed
				  // button pressed
				  clearTFTScreen(); // clears everything under the Title
				  switch (menuCurrentItemNr) {
				    case 1:
				      programStage=100;
				      strcpy_P(TemperNameString, TemperNameGenesisPGM);
				      TemperEEPROMaddr = EEPROM_ADDR_GENESIS;
				      break;
				    case 2:
				      programStage=100;
				      strcpy_P(TemperNameString, TemperNameWingPGM);
				      TemperEEPROMaddr = EEPROM_ADDR_WING;
				      break;
				    case 3:
				      programStage=400;	// Yes 400 is Bagging but it is 3rd line in menu
				      strcpy_P(BaggingNameString, BaggingNameWingPGM);
				      BaggingEEPROMaddr = EEPROM_ADDR_BAGGING;
				      break;
				    case 4:
				      programStage=300;	// Config is 300
				      break;
				    case 5:
				      programStage=500; // Help screens
				  } 
		    }
		    if(LiveDataUpdate_millis+1000<millis()){
				LiveDataUpdate_millis = millis();
				printAllSensorsValues();	// Print current sensor values
		    }
	}
	//wdt_disable();	// disable WDT until we in Presets menu. It is safe.
    // we should wait for the left rotary encoder...
    //menuNavigate();
/*
    char encVal = 0;  // signed value - nothing is pressed
    while (encVal != 127) {
      encVal = rotaryEncRead(0);  // read left encoder
      if(encVal!=0 && encVal!=127) {
        // go down or up in menu
        goDownUpInMenu(encVal);
      }
	  readSensors();	// just to update values in variables
    }
*/
}

// bitmaps for arrows
const unsigned char ArrowUp_bitmap[10] PROGMEM= {
  0b00011000,
  0b00111100,
  0b01111110,
  0b11111111,
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000,
};
const unsigned char ArrowDown_bitmap[10] PROGMEM= {
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000,
  0b11111111,
  0b01111110,
  0b00111100,
  0b00011000,
};

/************ Temper routine ***************/

void doTemperTFT(char* Temper_screentitle, int Temper_eeprom) {
	
	//doTemperControl();
	
    // Also determine what to show on the screen (menu or live data)
	switch(programStage) {
		case 100: // menu for configuring data
			{
			MenuAutoSwitch_millis = millis(); // start timer for auto switch from menu back to real data (if tempering)
			
			drawScreenTitle(Temper_screentitle);
			clearTFTScreen(); // clears everything under the Title
			
			if(Temper_eeprom==EEPROM_ADDR_WING){Temper_stages=2;}else{Temper_stages=1;}
			
			menuTotalItems = 1; // will be calculated
			menuCurrentItemNr = 1;
			
			menuPrintItem(menuTotalItems,PSTR("Total temper time (h:m:s):"));menuTotalItems++;
			menuPrintItem(menuTotalItems,PSTR("Temper temperature:"));menuTotalItems++;
			if(Temper_stages==2){
				menuPrintItem(menuTotalItems,PSTR("2nd stage duration (h:m:s):"));menuTotalItems++;
				menuPrintItem(menuTotalItems,PSTR("2nd stage temperature:"));menuTotalItems++;
			}
			menuPrintItem(menuTotalItems,PSTR("Temperature change speed:"));menuTotalItems++;
			menuPrintItem(menuTotalItems,PSTR("Temperature sensor:"));

			if(Temper_tempsetstate!=2){
				Tft.setXY(80,165); Tft.Fcolor=GRAY1;
				Tft.drawString_P(PSTR("Heater will warm up"));
				Tft.setXY(80,185); Tft.drawString_P(PSTR("in "));
			}
			menuDrawEncodersInProccess(Temper_started);
			// reset(initialize) variables if process not started
			if(!Temper_started){
				Temper_heatertime = 0;
				Temper_tempsetstate=1; // heater in heating up state
				// Other variables we should restore from EEPROM
				Temper_totaltime=menuIncDecTime(EEPROM.readLong(Temper_eeprom), 0);
				Temper_temperature = EEPROM.read(Temper_eeprom + 5);Temper_temperature = constrain(Temper_temperature, 20, 180);
				Temper_smoothchange=menuIncDecSmoothChange(EEPROM.readFloat(Temper_eeprom + 10), 0);
				Temper_sensor = EEPROM.read(Temper_eeprom + 15);Temper_sensor = constrain(Temper_sensor, 0, 1);
				if(Temper_stages == 2){
					Temper_stage2time=menuIncDecTime(EEPROM.readLong(Temper_eeprom + 25), 0); if(Temper_stage2time>Temper_totaltime){Temper_stage2time=Temper_totaltime;}
					Temper_stage2temperature = EEPROM.read(Temper_eeprom + 30);Temper_stage2temperature = constrain(Temper_stage2temperature, 20, 180);
				}
			}
			programStage=101;
			} // end of case 100:
		case 101: 
			{// show values in the menu in realtime (not often than once per 150ms)
			if(LiveDataUpdate_millis+150<millis()){
				LiveDataUpdate_millis = millis();
				//char tmpchar[6];
				menuPrintItemValue(1);printTime(Temper_totaltime);
				menuPrintItemValue(2);Tft.drawNumber(Temper_temperature);Tft.drawString_P(PSTR(" C  "));
				byte curmnuitm=3;
				if(Temper_stages==2){
					menuPrintItemValue(curmnuitm);printTime(Temper_stage2time);curmnuitm++;
					menuPrintItemValue(curmnuitm);Tft.drawNumber(Temper_stage2temperature);Tft.drawString_P(PSTR(" C  "));curmnuitm++;
				}
				menuPrintItemValue(curmnuitm);Tft.drawFloat(Temper_smoothchange,1);Tft.drawString_P(PSTR(" C/Min"));curmnuitm++;
				menuPrintItemValue(curmnuitm);if(Temper_sensor==0){Tft.drawString_P(PSTR("BMPxxx "));}else{Tft.drawString_P(PSTR("PT1000 "));} 
				menuPrintSelector(menuCurrentItemNr);
				// show "about time" for heater warming up
				if(!Temper_started){Temper_setcurtemp=(Temper_sensor==0) ? BMPtemperature : PT1000temperature;} // take correct base value
				if(Temper_tempsetstate!=2){
					Tft.setXY(97,185);Tft.Fcolor=GRAY1;
					int tmpMinutes = ((float)Temper_temperature-Temper_setcurtemp) / Temper_smoothchange;
					if(tmpMinutes<=60){
						Tft.drawNumber(tmpMinutes);	// only minutes
					}else{
						// hours and minutes
						Tft.drawNumber(tmpMinutes / 60);Tft.drawString_P(PSTR(" h, "));Tft.drawNumber(tmpMinutes-tmpMinutes / 60 * 60);
					}
					Tft.drawString_P(PSTR(" minutes.  "));
				}
			}
			// now check encoders (encoders read as often as possible)
			// check left rotary encoder...
			char encVal = rotaryEncRead(LEFT_ENCODER);  // read left encoder
			if(encVal==127) { // button is pressed
				if(Temper_started){
					programStage = (Bagging_started) ? 410 : 400;	// go to Bagging process (live screen or menu to start the process)!
				} else {
					programStage = (Bagging_started) ? 410 : 0;	// go to presets if nothing started
				}
			} else if(encVal!=0) {  // rotary encoder rotated
				// go down or up in menu
				goDownUpInMenu(encVal);
			}
			// check right rotary encoder...
			encVal = rotaryEncRead(RIGHT_ENCODER);  // read right encoder
			if(encVal==127) { // button is pressed
				// go to live data or if process is stoped then start the process.
				if(!Temper_started){
					// ********** start the process *************
					Temper_started = HIGH;
					TemperTotalTime_millis = millis();
					TemperTime_millis=0;  // reset counter for seconds (yes, it is actually seconds, not millis
					// and store current settings in EEPROM for the future. Only store when values changed "offline"
					EEPROM.updateLong(Temper_eeprom, Temper_totaltime);
					EEPROM.update(Temper_eeprom + 5, Temper_temperature);
					EEPROM.updateFloat(Temper_eeprom + 10, Temper_smoothchange);
					EEPROM.update(Temper_eeprom + 15, Temper_sensor);
					if(Temper_stages == 2){
						EEPROM.updateLong(Temper_eeprom + 25, Temper_stage2time);
						EEPROM.update(Temper_eeprom + 30, Temper_stage2temperature);
					}
				    #ifdef BUZZER
						Beep(100);
					#endif
				}
				drawScreenTitle(Temper_screentitle);	// update icons
				programStage=110; // go to live data screen
			} else if(encVal!=0) {  // rotary encoder rotated
				switch(menuCurrentItemNr){
					case 1:
						Temper_totaltime=menuIncDecTime(Temper_totaltime, encVal);
						//Temper_totaltime=constrain((Temper_totaltime + encVal * 60 * ((Temper_totaltime>=3600) ? 10 : 1)), 60, 356400); // min: 60 sec, max: 99 hours;
						break;
					case 2:
						Temper_temperature=constrain((Temper_temperature+encVal),20,180); // min: 20c, max: 180c
						break;
					case 3:
						if(Temper_stages==2){ // stage2 temper time cannot be lower than total temper time, because it is part of the total time
							Temper_stage2time=menuIncDecTime(Temper_stage2time, encVal);if(Temper_stage2time>Temper_totaltime){Temper_stage2time=Temper_totaltime;}
							//Temper_stage2time=constrain((Temper_stage2time + encVal * 60 * ((Temper_stage2time>=3600) ? 10 : 1)), 60, Temper_totaltime); // min: 60 sec, max: 99 hours;
						} else {
							Temper_smoothchange=menuIncDecSmoothChange(Temper_smoothchange, encVal);
							//Temper_smoothchange=constrain((Temper_smoothchange+(float)encVal/((Temper_smoothchange>=1.0) ? 2.0 : 10.0)),0.1,9.9); // min: 0.1c, max: 9.9c
						}
						break;
					case 4:
						if(Temper_stages==2){
							Temper_stage2temperature=constrain(Temper_stage2temperature+encVal,20,180); // min: 20c, max: 180c
						} else {
							Temper_sensor=constrain(Temper_sensor+encVal,0,1); // min: 0, max: 1
						}
						break;
					case 5: // only if temper stages = 2
						Temper_smoothchange=menuIncDecSmoothChange(Temper_smoothchange, encVal);
						//Temper_smoothchange=constrain((Temper_smoothchange+(float)encVal/((Temper_smoothchange>=1.0) ? 2.0 : 10.0)),0.1,9.9); // min: 0.1c, max: 9.9c
						break;
					case 6: // only if temper stages = 2
						Temper_sensor=constrain(Temper_sensor+encVal,0,1); // min: 0, max: 1
					break;
				}
				MenuAutoSwitch_millis=millis(); // reset timer, because we manipulate with menu
			}
			// If we in the process of tempering, then it is good to switch back to live data after some time (lets say 10s)
			if(Temper_started && MenuAutoSwitch_millis+10000<millis()){
				programStage=110; // go to live data screen
			}
			break;
			}	// End of Case 101:
		case 110: // live screen text
			drawScreenTitle(Temper_screentitle);
			printLiveScreenTextGeneral(TEMPERING_PROCESS);
			Tft.setXY(80,190);Tft.Fcolor=GRAY1;Tft.drawString_P(PSTR("Phase:"));
			programStage=111;
		case 111: //live screen values
			{
			if(LiveDataUpdate_millis+150<millis()){
				LiveDataUpdate_millis = millis();
				#define TEMPERATURECOLOR_WARMING     ROSE
				#define TEMPERATURECOLOR_TEMPER      BLUE_SKY
				#define TEMPERATURECOLOR_COOLING     YELLOW
				
				Tft.setXY(5,42);Tft.Fcolor=GREEN; 
				printBigTime(Temper_totaltime);
				Tft.setXY(230,42);Tft.Fcolor=WHITE; 
				printTime(Temper_heatertime);
				
				Tft.setXY(130,190);
				if(Temper_tempsetstate==1){
					Tft.Fcolor=TEMPERATURECOLOR_WARMING; 
					Tft.drawString_P(PSTR("Warming up  "));
				} else if(Temper_tempsetstate==0) {
					Tft.Fcolor=TEMPERATURECOLOR_COOLING;
					Tft.drawString_P(PSTR("Cooling down"));
				}else{
					Tft.Fcolor=TEMPERATURECOLOR_TEMPER;
					Tft.drawString_P(PSTR("Tempering   "));
				}
				Tft.setXY(40,127);
				byte curTemp=round((Temper_sensor==0) ? BMPtemperature : PT1000temperature);
				if(curTemp<100){Tft.fillRectangle(Tft.X,Tft.Y,24,40);Tft.X+=24+(24 / 4);} // wipe out 
				Tft.Y=127;
				Tft.drawNumberPGM(curTemp);
				Tft.X=127;Tft.drawString("C");
				
				Tft.setXY(175,127);
				if(Temper_tempsetstate==1){
					Tft.Fcolor=RED;
					Tft.drawBitmap(Tft.X, Tft.Y+30, ArrowUp_bitmap, 8, 10); // print arrow from bitmap
					Tft.fillRectangle(Tft.X, Tft.Y,8,10); // clear other arrow
					Tft.Fcolor=TEMPERATURECOLOR_WARMING;
				} else if(Temper_tempsetstate==0) {
					Tft.Fcolor=ORANGE;
					Tft.drawBitmap(Tft.X, Tft.Y, ArrowDown_bitmap, 8, 10); // print arrow from bitmap
					Tft.fillRectangle(Tft.X, Tft.Y+30,8,10);  // clear other arrow
					Tft.Fcolor=TEMPERATURECOLOR_COOLING;
				}else{
					Tft.Fcolor=BLUE;
					Tft.drawBitmap(Tft.X, Tft.Y, ArrowDown_bitmap, 8, 10); // print arrow from bitmap
					Tft.drawBitmap(Tft.X, Tft.Y+30, ArrowUp_bitmap, 8, 10); // print arrow from bitmap
					Tft.Fcolor=TEMPERATURECOLOR_TEMPER;
				}
				//Tft.Y=142;Tft.X=170;Tft.drawNumber((int)round(Temper_setcurtemp));Tft.drawString_P(PSTR(" C  "));
				Tft.setXY(170,142);Tft.drawFloat(Temper_setcurtemp,1);Tft.drawString_P(PSTR(" C  "));
			}  
			
			// while in live data we check left encoder to go back to menu changes or to switch process
			// Also we check right encoder press for stopping process
			char encVal = rotaryEncRead(LEFT_ENCODER);  // read left encoder
			if(encVal!=0 && encVal!=127) { // Knob is rotated 
				programStage=100; // go to menu.
			} else if(encVal==127) {	// Knob is pressed
				// switch proccess
				programStage = (Bagging_started) ? 410 : 400;	// go to Bagging process (live screen or menu to start the process)!
			}
			encVal = rotaryEncRead(RIGHT_ENCODER);  // read left encoder
			if(encVal==127) { // button is pressed - stop the process EMERGENCY!
				Temper_started = LOW;
				turnTemperHeaterOnOff(LOW);
				programStage=100; // go to menu.
			}
			break;
			}  // End of case 111:
		case 120: // IDLE screen
			{
			drawScreenTitle(Temper_screentitle);
			printSummaryText(TEMPERING_PROCESS);
			programStage = 121;
			} // End of case 120:
		case 121:
			{    
			printSummaryValues(TEMPERING_PROCESS);
			break;
			}  // End of the case 121:
		
	}	// Endof the switch()

} // End of the doTempering 

void doTemperControl() {
  if(Temper_started) { // do here control of the heater for Wing Tempering
    if(TemperHyst_millis+2000<millis()){ // do not switch heater often than once per 2 seconds
      float tmp_temp = (Temper_sensor==0) ? BMPtemperature : PT1000temperature;
      if(tmp_temp > Temper_setcurtemp && digitalRead(SSD1_PIN)) {
        turnTemperHeaterOnOff(LOW);
      } else if(tmp_temp < Temper_setcurtemp && !digitalRead(SSD1_PIN)){
        turnTemperHeaterOnOff(HIGH);
      }
    }    
    if(Temper_millis+1000<millis()) {  // count time
      Temper_millis=millis();
      if(Temper_tempsetstate==2){
        if(Temper_totaltime != 0){
          Temper_totaltime--;  // minus one second
          TemperTime_millis++;  // also count temper time for summary report at the end
          // Check here, do we need to use temperature from the stage2?
          if(Temper_totaltime<=Temper_stage2time && Temper_stages==2){	// only go here if we have 2 stages of heating
          	Temper_temperature=Temper_stage2temperature;
       	    #ifdef BUZZER
	          	if(Temper_totaltime==Temper_stage2time){	// beep only once per change
	          		Beep(90);
	          		delay(50);
	          		Beep(90);
	          	}
          	#endif
          }
        } else {
          // change to cool down process
          Temper_tempsetstate=0; // cool down heater smoothly
        }
        // if we changed tempering temperature, we should check do we need to switch to warming state
        if(Temper_setcurtemp<Temper_temperature && Temper_tempsetstate!=0){Temper_tempsetstate=1;}
        	else if(Temper_setcurtemp>Temper_temperature){Temper_setcurtemp = Temper_temperature;} // update preset temp to go down if needed
      }
      // Heater time
      if(digitalRead(SSD1_PIN)){Temper_heatertime++;}
    // set current temperature (smooth heater warm up, constant, or cool down)
    if(Temper_tempsetstate==1){ // heater is warming up
      Temper_setcurtemp += Temper_smoothchange / 60.0;  // degrees per menute. we add every second. So we divide by 60 seconds.
      if(Temper_setcurtemp>=Temper_temperature){Temper_setcurtemp=Temper_temperature; Temper_tempsetstate=2;}
    }else if(Temper_tempsetstate==0){ // heater is cooling down
        Temper_setcurtemp -= Temper_smoothchange / 60.0;  // degrees per menute. we sub every second. So we divide by 60 seconds.
        if(Temper_setcurtemp<30.0){  
          // ************ stop process totally ****************
          Temper_started = LOW; 
          turnTemperHeaterOnOff(LOW);
          TemperIDLE_millis = millis(); // start to count IDLE time
          TemperTotalTime_millis=millis()-TemperTotalTime_millis; // store here total time system running
          Temper_finishedvalue = (Temper_sensor==0) ? BMPtemperature : PT1000temperature; // store finishing value
          programStage=120; // go to summary screen
        } 
    }
    }
  }
}

void turnTemperHeaterOnOff(bool state) {
	digitalWrite(SSD1_PIN, state);
	TemperHyst_millis=millis();
}


// print timer in big numbers: HH:MM:SS
void printBigTime(unsigned long t) {
  long h = t / 3600;
  t = t % 3600;
  int m = t / 60;
  int s = t % 60;
  printBigNumber00(h);
  printBigTimeDots();
  printBigNumber00(m);
  printBigTimeDots();
  printBigNumber00(s);
}

void printBigNumber00(byte n) {
  if (n<10) {Tft.drawNumberPGM(0);}
  Tft.drawNumberPGM(n);
}

void printBigTimeDots() {
  Tft.fillCircle(Tft.X+3,Tft.Y+13,2);
  Tft.fillCircle(Tft.X+3,Tft.Y+27,2);
  Tft.X+=12;
}

void printTime(unsigned long t) {
  long h = t / 3600;
  t = t % 3600;
  int m = t / 60;
  int s = t % 60;
  printNumber00(h);
  Tft.drawChar(':');
  printNumber00(m);
  Tft.drawChar(':');
  printNumber00(s);
}

void printNumber00(byte n) {
  if (n<10) {Tft.drawChar('0');}
  Tft.drawNumber(n);
}

/**************** BAGGING routines *****************/

void doBaggingTFT(char* Bagging_screentitle, int Bagging_eeprom) {

	//doBaggingControl();

  // Also determine what to show on the screen (menu or live data)
	switch(programStage) {
    case 400: // menu for configuring data
    {
	    MenuAutoSwitch_millis = millis(); // start timer for auto switch from menu back to real data (if bagging)
	    
	    drawScreenTitle(Bagging_screentitle);
	    clearTFTScreen(); // clears everything under the Title

		if (!bmp_presence) {
		    printBMPXXXnotfound();
    		programStage = (Temper_started) ? 110 : 0; // Go back to tempering proccess if started, or to menu
    		break;
  		} else {
  			
		    menuTotalItems = 4;
		    menuCurrentItemNr = 1;
		    
		    menuPrintItem(1,PSTR("Total bagging time (h:m:s):"));
		    menuPrintItem(2,PSTR("Bagging pressure:"));
		    menuPrintItem(3,PSTR("Pressure tolerance (+/-):"));
		    menuPrintItem(4,PSTR("Pump rest time:"));
		
		    menuDrawEncodersInProccess(Bagging_started);
		    // reset(initialize) variables if process not started
		    if(!Bagging_started){
			    Bagging_pumptime = 0;
			    // Other variables we should restore from EEPROM
			    Bagging_totaltime=menuIncDecTime(EEPROM.readLong(Bagging_eeprom), 0);
			    Bagging_pressure = EEPROM.readInt(Bagging_eeprom + 5);Bagging_pressure = constrain(Bagging_pressure, 200, 1200);
			    Bagging_tolerance = EEPROM.readInt(Bagging_eeprom + 10);Bagging_tolerance = constrain(Bagging_tolerance, 1, 240);
			    Bagging_pumpresttime = EEPROM.read(Bagging_eeprom + 15);Bagging_pumpresttime = constrain(Bagging_pumpresttime, 1, 240);
		    }
	    	programStage=401;
  		}
	
    } // end of current case
    case 401: 
	    {// show values in the menu in realtime (not often than once per 150ms)
		if(LiveDataUpdate_millis+150<millis()){
			LiveDataUpdate_millis = millis();
			char tmpchar[6];
			menuPrintItemValue(1);printTime(Bagging_totaltime);
			menuPrintItemValue(2);Tft.drawNumber(Bagging_pressure);Tft.drawString_P(PSTR(" mbar   "));
			menuPrintItemValue(3);Tft.drawNumber(Bagging_tolerance);Tft.drawString_P(PSTR("   "));
			menuPrintItemValue(4);Tft.drawNumber(Bagging_pumpresttime);Tft.drawString_P(PSTR(" s  "));
			menuPrintSelector(menuCurrentItemNr);
			// now check encoders (encoders read as often as possible)
			// check left rotary encoder...
		}
		    char encVal = rotaryEncRead(LEFT_ENCODER);  // read left encoder
			if(encVal==127) { // button is pressed
				if(Bagging_started){
					programStage = (Temper_started) ? 110 : 100;	// go to Temper process (live screen or menu to start the process)!
				} else {
					programStage = (Temper_started) ? 110 : 0;	// go to presets if nothing started
				}
			} else if(encVal!=0) {  // rotary encoder rotated
				// go down or up in menu
				goDownUpInMenu(encVal);
			}
			// check right rotary encoder...
			encVal = rotaryEncRead(RIGHT_ENCODER);  // read right encoder
			if(encVal==127) { // button is pressed
				// go to live data or if process is stoped then start the process.
				// store config allways
				//if(!Bagging_started){
					// ********** start the process *************
					Bagging_started = HIGH;
					// Store to EEPROM state of Bagging process
					EEPROM.update(EEPROM_ADDR_LAST_PROCCESS, 1);
					BaggingHyst_millis=millis(); // reset hyst millis to rest a pump before start
					BaggingTime_millis=0;  // reset counter for seconds (yes, it is actually seconds, not millis
					// and store current settings in EEPROM for the future. Only store when values changed "offline"
					EEPROM.updateLong(Bagging_eeprom, Bagging_totaltime);
					EEPROM.updateInt(Bagging_eeprom + 5, Bagging_pressure);
					EEPROM.updateInt(Bagging_eeprom + 10, Bagging_tolerance);
					EEPROM.update(Bagging_eeprom + 15, Bagging_pumpresttime);
					#ifdef BUZZER
						Beep(100);
					#endif
				//}
				drawScreenTitle(Bagging_screentitle);	// update icons
				programStage = 410;
			} else if(encVal!=0) {  // rotary encoder rotated
				switch(menuCurrentItemNr){
					case 1:
						Bagging_totaltime=menuIncDecTime(Bagging_totaltime, encVal);
						break;
					case 2:
						Bagging_pressure=constrain((Bagging_pressure + encVal * 10), 200, 1200); // min: 200 mbar, max: 1200 mbar
						break;
					case 3:
						Bagging_tolerance=constrain((Bagging_tolerance + encVal * 2), 1, 240); // min: 0 mbar, max: 300 mbar
						break;
					case 4:
						Bagging_pumpresttime=constrain((Bagging_pumpresttime + encVal * 2), 1, 240); // min: 1 sec, max: 240 (3 min)
						break;
				}
				MenuAutoSwitch_millis=millis(); // reset timer, because we manipulate with menu
			}
			// If we in the process of bagging, then it is good to switch back to live data after some time (lets say 10s)
			if(Bagging_started && MenuAutoSwitch_millis+10000<millis()){
				programStage = 410; // go to live data screen
			}
			break;
		} // End of Case 401:
	case 410: // live screen text
		drawScreenTitle(Bagging_screentitle);
		printLiveScreenTextGeneral(BAGGING_PROCESS);
		// print set pressure and tolerance - will not change in live
		Tft.setXY(205,127);
		Tft.Fcolor=BLUE;
		Tft.drawBitmap(Tft.X, Tft.Y, ArrowDown_bitmap, 8, 10); // print arrow from bitmap
		Tft.drawBitmap(Tft.X, Tft.Y+30, ArrowUp_bitmap, 8, 10); // print arrow from bitmap
		Tft.Fcolor=BLUE_SKY;
		Tft.setXY(200,142);Tft.drawNumber(Bagging_pressure);
		Tft.drawString_P(PSTR(" +/-"));Tft.drawNumber(Bagging_tolerance);Tft.drawString_P(PSTR(" mB  "));
		
		programStage=411;
	case 411: //live screen values
		{
		if(LiveDataUpdate_millis+150<millis()){
			LiveDataUpdate_millis = millis();
			Tft.setXY(5,42);Tft.Fcolor=GREEN; 
			printBigTime(Bagging_totaltime);
			Tft.setXY(230,42);Tft.Fcolor=WHITE; 
			printTime(Bagging_pumptime);
			// print pump rest time
			Tft.setXY(230,62);Tft.Fcolor=DARK_GREEN;
			long tmpS=BaggingHyst_millis + (unsigned long)Bagging_pumpresttime * 1000 - millis();
			if(tmpS>=0 && !digitalRead(SSD2_PIN)){
				Tft.drawString_P(PSTR("Rest... ("));Tft.drawNumber(tmpS / 1000);Tft.drawString_P(PSTR(")  "));
			} else {
				Tft.fillRectangle(Tft.X,Tft.Y,310,20);	// clear resting message
			}
			// print bmp pressure
			Tft.Fcolor=BLUE_SKY;
			if(BMPpressure<200){
				Tft.Fcolor=RED;
				BMPpressure=199;
			}
			Tft.setXY(40,127);
			if(BMPpressure<1000){Tft.fillRectangle(Tft.X,Tft.Y,24,40);Tft.X+=24+(24 / 4);} // wipe out 
			Tft.Y=127;
			Tft.drawNumberPGM(BMPpressure);
			Tft.X=160;Tft.drawString_P(PSTR("mB"));
		}  
		
		// while in live data we check left encoder to go back to menu changes.
		// Also we check right encoder press for stopping process
		char encVal = rotaryEncRead(LEFT_ENCODER);  // read left encoder
		if(encVal!=0 && encVal!=127) { // Knob is rotated
			programStage=400; // go to menu to adjust values.
		} else if(encVal==127) {	// Knob is pressed
			programStage = (Temper_started) ? 110 : 100; // Go to Temper live screen or menu to start Temper
		}
		encVal = rotaryEncRead(RIGHT_ENCODER);  // read left encoder
		if(encVal==127) { // button is pressed - stop the process EMERGENCY!
			Bagging_started = LOW;
			// Clear running state for Bagging (no resume after Watchdog reset)
			EEPROM.update(EEPROM_ADDR_LAST_PROCCESS, 0);
			turnBaggingPumpOnOff(LOW);
			programStage=400; // go to menu.
		}
		break;
		}  // End of case 411:
	case 420: // IDLE screen
		{
		drawScreenTitle(Bagging_screentitle);
		printSummaryText(BAGGING_PROCESS);
		programStage = 421;
		} // End of case 120:
	case 421:
		{    
		printSummaryValues(BAGGING_PROCESS);
		break;
		}  // End of the case 121:
		
	} // End of switch(programStage)

} // end of doBagging procedure 

void doBaggingControl() {
	if(Bagging_started) { // do here control of the pump for Wing Bagging
		if(BMPpressure < (Bagging_pressure - Bagging_tolerance) && digitalRead(SSD2_PIN)) {
			if(BaggingHyst_millis+1000<millis()){ // do not switch Pump off quicker than once per 1 seconds
				turnBaggingPumpOnOff(LOW);
			}
		} else if(BMPpressure > (Bagging_pressure + Bagging_tolerance) && !digitalRead(SSD2_PIN)){
			if((BaggingHyst_millis+(unsigned long)Bagging_pumpresttime*1000)<millis()){ // do not switch Pump on if Pump is resting
				turnBaggingPumpOnOff(HIGH);
				Bagging_lastgoodpressure = BMPpressure; // store the current pressure and follow, will it drop during the time
			}
		}
		if(Bagging_millis+1000<millis()) {  // count time
			Bagging_millis=millis();
			if(Bagging_totaltime != 0){
				Bagging_totaltime--;  // minus one second
				BaggingTime_millis++;  // also count bagging time for summary report at the end
				if(digitalRead(SSD2_PIN)){Bagging_pumptime++;} // Pump time
			} else {
				// ************ stop process totally ****************
				Bagging_started = LOW; 
				// No resume after Watchdog reset
				EEPROM.update(EEPROM_ADDR_LAST_PROCCESS, 0);
				turnBaggingPumpOnOff(LOW);
				BaggingIDLE_millis = millis(); // start to count IDLE time
				Bagging_finishedvalue = BMPpressure; // Store finished value
				programStage=420; // go to summary screen
			}
		}
    	// check, does pump really working when on? (maybe rest time is not enough)
    	// When pump turned on then Bagging_lastgoodpressure = BMPpressure.
    	// After 5 seconds (if pump still on) pressure should be lower. if so, everything is ok
    	// But if pressure stays higher and pump is ON then something wrong. 
    	//   Lets reset pump more (switch off and increase a bit Bagging_pumpresttime).
    	if(Bagging_lastgoodpressure<BMPpressure && digitalRead(SSD2_PIN) && BaggingHyst_millis+5000<millis()) {
    		Bagging_pumpresttime += 5; if(Bagging_pumpresttime>240){Bagging_pumpresttime=240;}	// add 5 seconds more
			// turn off pump and let it rest
			turnBaggingPumpOnOff(LOW);
    	}
	} //End of if(Bagging_started)
}

void turnBaggingPumpOnOff(bool state) {
	digitalWrite(SSD2_PIN, state);
	BaggingHyst_millis=millis();
}

// 0 - Temper Summary
// 1 - Bagging Summary
void printLiveScreenTextGeneral(byte processNr) {
	clearTFTScreen();
	menuShowEncoder("Adjust","Switch task",20, LEFT_ENCODER);  // number indicates text move to the left (centering)
	menuShowEncoder(" ","Stop",25, RIGHT_ENCODER); // number indicates text move to the left (centering)
	Tft.setXY(0,25);Tft.Fcolor=GRAY1;
	if(processNr==0){Tft.drawString_P(PSTR("Temper"));}else{Tft.drawString_P(PSTR("Bagging"));}
	Tft.drawString_P(PSTR(" time left (h:m:s):"));
	Tft.setXY(225,25);
	if(processNr==0){Tft.drawString_P(PSTR("Heater"));}else{Tft.drawString_P(PSTR("Pump"));}
	Tft.drawString_P(PSTR(" time:"));
	Tft.setXY(0,110);
	if(processNr==0){Tft.drawString_P(PSTR("Temperature:"));}else{Tft.drawString_P(PSTR("Pressure:"));}
	Tft.setXY(5,150);Tft.drawString_P(PSTR("Cur:"));
}

// 0 - Temper Summary
// 1 - Bagging Summary
void printSummaryText(byte processNr) {
	#ifdef BUZZER
		for(byte i=0;i<5;i++){
			// beeps
			Beep(200);
			delay(100);
		}
	#endif
	clearTFTScreen();
	if(Temper_started || Bagging_started){
		menuShowEncoder(" ","Switch task",20, LEFT_ENCODER);  // number indicates text move to the left (centering)
	} else {
		menuShowEncoder("Menu"," ",16, LEFT_ENCODER);  // number indicates text move to the left (centering)
	}
	Tft.setXY(85,30);Tft.bold=FONT_BOLD;Tft.Fcolor=WHITE;
	Tft.drawString_P(PSTR("Process finished"));Tft.bold=FONT_NORMAL;
	Tft.Fcolor=BLUE_SKY;Tft.drawRectangle(80,25, 150,22);
	Tft.Fcolor=WHITE;Tft.setXY(120,55);Tft.drawString_P(PSTR("Summary:"));
	Tft.Fcolor=GREEN; Tft.drawLine(10,73,300,73);Tft.Fcolor=WHITE;
	Tft.setXY(10,83);Tft.drawString_P(PSTR("System IDLE time:"));
	Tft.setXY(10,102);Tft.drawString_P(PSTR("Process running time:"));
	Tft.setXY(10,121);
	if(processNr==TEMPERING_PROCESS){Tft.drawString_P(PSTR("Tempering"));}else{Tft.drawString_P(PSTR("Bagging"));}
	Tft.drawString_P(PSTR(" time:"));
	Tft.setXY(10,140);
	if(processNr==TEMPERING_PROCESS){Tft.drawString_P(PSTR("Heater"));}else{Tft.drawString_P(PSTR("Pump"));}
	Tft.drawString_P(PSTR(" working time:"));
	//Print finishing value (temperature or pressure) 
	Tft.setXY(10,159);
	Tft.drawString_P(PSTR("Finished at:"));Tft.setXY(230,159);
	Tft.Fcolor=CYAN;
	if(processNr==TEMPERING_PROCESS){
		Tft.drawFloat(Temper_finishedvalue,1);Tft.drawString_P(PSTR(" C"));
	}else{
		Tft.drawNumber(Bagging_finishedvalue);Tft.drawString_P(PSTR(" mBar"));	
	}
    
}

// 0 - Temper Summary
// 1 - Bagging Summary
void printSummaryValues(byte processNr) {
	if(LiveDataUpdate_millis+250<millis()){
		LiveDataUpdate_millis = millis();
		Tft.Fcolor=CYAN;
		Tft.setXY(230,83);printTime((millis() - ((processNr==TEMPERING_PROCESS) ? TemperIDLE_millis : BaggingIDLE_millis)) / 1000);
		Tft.setXY(230,102);printTime(((processNr==TEMPERING_PROCESS) ? TemperTotalTime_millis : BaggingTime_millis)/1000);
		Tft.setXY(230,121);printTime(((processNr==TEMPERING_PROCESS) ? TemperTime_millis : BaggingTime_millis)); // it is actually in seconds, not millis, so no /1000
		Tft.setXY(230,140);printTime(((processNr==TEMPERING_PROCESS) ? Temper_heatertime : Bagging_pumptime));
	
		printAllSensorsValues();	// Print current sensor values
	}
	char encVal = rotaryEncRead(LEFT_ENCODER);  // read left encoder
	if(encVal!=127 && encVal!=0) { // Knob is rotated
		if(!Temper_started && !Bagging_started){programStage=0;} // go to initial menu with presets if no running processes.
	} else if(encVal==127) {  // Knob is pressed
		if(Temper_started || Bagging_started){	//only react, if there is processes running
			if(processNr==TEMPERING_PROCESS) {
				programStage = 410;  // go to bagging live
			} else {
				programStage = 110;  // goto tempering live
			}
		}
	}
}

void menuDrawEncodersInProccess(bool prcStarted) {
    if(Temper_started || Bagging_started){
	    menuShowEncoder("Select","Switch task", 20, LEFT_ENCODER); // number indicates text move to the left (centering)
    } else {
	    menuShowEncoder("Select","Menu", 20, LEFT_ENCODER); // number indicates text move to the left (centering)
    }
    if(prcStarted){
    	menuShowEncoder("Change","Back",93, RIGHT_ENCODER); // number indicates text move to the left (centering)
    } else {
    	menuShowEncoder("Change","Start",97, RIGHT_ENCODER); // number indicates text move to the left (centering)
    }
}

void printBMPXXXnotfound() {
	Tft.Bcolor=RED;Tft.fillRectangle(50,90,220,48);
	Tft.Bcolor=BLACK;Tft.fillRectangle(55,95,210,38);
	Tft.setXY(60,100);Tft.Fcolor=WHITE;Tft.drawString_P(PSTR("BMP085/BMP180"));
	Tft.setXY(60,120);Tft.drawString_P(PSTR("sensor is NOT found!"));
	delay(3000);
}

void goDownUpInMenu(char eval) {
	menuClearSelector(menuCurrentItemNr);
	menuCurrentItemNr=constrain(menuCurrentItemNr+eval,1,menuTotalItems);
	menuPrintSelector(menuCurrentItemNr);
	MenuAutoSwitch_millis=millis(); // reset timer, because we manipulate with menu
}

unsigned long menuIncDecTime(unsigned long tm, char ch) {
	return constrain((tm + ch * 60 * ((tm>=3600) ? 10 : 1)), 60, 356400); // min: 60 sec, max: 99 hours;
}

float menuIncDecSmoothChange(float sc, char ch) {
	return constrain((sc+(float)ch/((sc>=1.0) ? 2.0 : 10.0)),0.1,9.9); // min: 0.1c, max: 9.9c
}

// Watchdog routines to prevent system from hangup with load switched ON.
void WDT_Init() {
	cli();
	wdt_reset();
	WDTCSR = (1<<WDCE)|(1<<WDE); //set up WDT interrupt
	WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP3); //Start watchdog timer with 4s prescaller
	sei();
}

ISR(WDT_vect) {
	// we assume that interrupts already off
	// turn off all loads
    digitalWrite(SSD1_PIN, LOW);
    digitalWrite(SSD2_PIN, LOW);
	wdt_disable();	// disable WDT
	// show alert message
	Tft.Bcolor=DARK_RED;Tft.fillRectangle(0,17,319,239);
	Tft.bold = FONT_BOLD;Tft.setXY(50,100);Tft.Fcolor=WHITE;Tft.drawString_P(PSTR("Watchdog: System halted!"));
	#ifdef BUZZER
		for(byte i=0;i<10;i++){
			// beeps
			Beep(800);
			delay(500);
		}	// infinite loop
	#endif
	delay(5*60000);	// wait 5 minutes
	// restart
	cli();
	wdt_reset();
	WDTCSR = (1<<WDCE)|(1<<WDE); //set up WDT interrupt
	WDTCSR = (0<<WDIE)|(1<<WDE)|(1<<WDP3); //Start watchdog timer with 4s prescaller
	sei();
	while(1){}	// reset after 4 seconds

}

#ifdef BUZZER
	// turn on beeper for interval ms.
	void Beep(unsigned int interval) {
		digitalWrite(BUZZER_PIN, HIGH);
		delay(interval);
		digitalWrite(BUZZER_PIN, LOW);
	}
#endif

// Print all sensors values on the screen in coordinates pX,pY
/* Format: 
Sensors:
BMP: 23.3C, 995mB
PT1000: 23.1c
*/
void printAllSensorsValues(){ //(byte pX, byte pY) {
	Tft.Fcolor=ENCODERS_BACKGROUND_COLOR;
	Tft.drawRectangle(161,182, 158,57);
	Tft.Fcolor=GRAY1;
	Tft.setXY(167,185); Tft.drawString_P(PSTR("Sensors:")); //Tft.bold = FONT_NORMAL;
	Tft.setXY(167,203); Tft.drawString_P(PSTR("BMP: "));
	if(bmp_presence){
		Tft.drawFloat(BMPtemperature,1);Tft.drawString_P(PSTR("C, "));
		Tft.drawNumber(BMPpressure);Tft.drawString_P(PSTR("Mb  "));
	}else{
		Tft.drawString_P(PSTR("no data"));
	}
	Tft.setXY(167,221); Tft.drawString_P(PSTR("PT1000: ")); Tft.drawFloat(PT1000temperature,1);Tft.drawString_P(PSTR("C  "));
}

/* Help screen in Menu.
 It describes suggested values for Bagging and Tempereing
 Format (proccess per screen):
 ----------------------------------------
 Tempering (Epoxy HP-E111L):
  Epoxy curing 9h at 40C.
  Epoxy tempering 5h at 60C + 6h at 80C.
 
 Tempering (Vinyl):
  10 minutes at 105C.
 ----------------------------------------
 Bagging:
  Normal pressure is about 1000 mBar.
  Start with 990 mBar.
  Adjust bag and go to 400-600 mBar.

*/
#define TOTAL_HELP_SCREENS 2
void doHelp() {
	switch(programStage) {
		case 500: // Show first help screen (Tempering)
		{
			drawScreenTitle("Help - Tempering");
			clearTFTScreen(); // clears everything under the Title
			
			Tft.bold = FONT_BOLD;Tft.Fcolor=WHITE;
			Tft.setXY(1,35); Tft.drawString_P(PSTR("Tempering (Epoxy HP-E111L):")); Tft.bold = FONT_NORMAL;
			Tft.setXY(10,55); Tft.drawString_P(PSTR("Epoxy curing 9h at 40C."));
			Tft.setXY(10,75); Tft.drawString_P(PSTR("Epoxy tempering 5h at 60C + 6h at 80C."));
			Tft.bold = FONT_BOLD; 
			Tft.setXY(1,100); Tft.drawString_P(PSTR("Tempering (Vinyl):")); Tft.bold = FONT_NORMAL;
			Tft.setXY(10,120); Tft.drawString_P(PSTR("10 minutes at 105C."));
			
			menuShowEncoder("Prev/Next","Menu", 20, LEFT_ENCODER); // number indicates text move to the left (centering)
			programStage = 501;
		}
		case 501: // check rotary encoders
		case 511: // check rotary encoders
		{
			char encVal = rotaryEncRead(LEFT_ENCODER);  // read left encoder
			if(encVal!=0 && encVal!=127) { // Knob is rotated
				// detect direction and then go to the next screen or prev.
				if(encVal>0) {
					programStage+=9; if(programStage>500+(TOTAL_HELP_SCREENS-1)*10){programStage=501+(TOTAL_HELP_SCREENS-1)*10;} // go to next help screen.
				} else {
					programStage-=11; if(programStage<500){programStage=501;} // go to prev help screen.
				}
			} else if(encVal==127) {	// Knob is pressed
				programStage = 0; // Go to Menu
			}
			break;
		}
		case 510: // Show second help screen (Bagging)
		{
			drawScreenTitle("Help - Bagging");
			clearTFTScreen(); // clears everything under the Title
			
			Tft.bold = FONT_BOLD;Tft.Fcolor=WHITE;
			Tft.setXY(1,35); Tft.drawString_P(PSTR("Bagging:")); Tft.bold = FONT_NORMAL;
			Tft.setXY(10,55); Tft.drawString_P(PSTR("Normal pressure is about 1000 mBar."));
			Tft.setXY(10,80); Tft.drawString_P(PSTR("Start with 990 mBar."));
			Tft.setXY(10,100); Tft.drawString_P(PSTR("Adjust bag and go to 400-600 mBar."));

			menuShowEncoder("Prev/Next","Menu", 20, LEFT_ENCODER); // number indicates text move to the left (centering)
			programStage = 511;
			break;
		}
	}
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/