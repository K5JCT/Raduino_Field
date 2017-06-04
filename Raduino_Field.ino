//Raduino Field - K5JCT


/**
 * This source file is under General Public License version 3.
 */


#include <EEPROM.h>
#include <Wire.h>
#include <si5351.h>
Si5351 si5351;



/** 
 * The Raduino board is the size of a standard 16x2 LCD panel. It has three connectors:
 * 
 * First, is an 8 pin connector that provides +5v, GND and six analog input pins that can also be 
 * configured to be used as digital input or output pins. These are referred to as A0,A1,A2,
 * A3,A6 and A7 pins. The A4 and A5 pins are missing from this connector as they are used to 
 * talk to the Si5351 over I2C protocol. 
 * 
 *  A0     A1   A2   A3    GND    +5V   A6   A7
 * BLACK BROWN RED ORANGE YELLOW GREEN BLUE VIOLET
 * 
 * Second is a 16 pin LCD connector. This connector is meant specifically for the standard 16x2
 * LCD display in 4 bit mode. The 4 bit mode requires 4 data lines and two control lines to work:
 * Lines used are : RESET, ENABLE, D4, D5, D6, D7 
 * We include the library and declare the configuration of the LCD panel too
 */

#include <LiquidCrystal.h>
LiquidCrystal lcd(8,9,10,11,12,13);
#include <LcdBarGraph.h>
LcdBarGraph lbg(&lcd, 10, 8, 2);  // -- creating bargraph instance, format is (&lcd, lcdNumCols, start X, start Y).
//So (&lcd, 16, 0, 1) would set the bargraph length to 16 columns and start the bargraph at column 0 on row 1.

char serial_in[32], c[30], b[30], printBuff[32];
int count = 0;
unsigned char serial_in_count = 0;

/**
 * We need to carefully pick assignment of pin for various purposes.
 * There are two sets of completely programmable pins on the Raduino.
 * First, on the top of the board, in line with the LCD connector is an 8-pin connector
 * that is largely meant for analog inputs and front-panel control. It has a regulated 5v output,
 * ground and six pins. Each of these six pins can be individually programmed 
 * either as an analog input, a digital input or a digital output. 
 * The pins are assigned as follows: 
 *        A0,   A1,  A2,  A3,   GND,   *5V,  A6,  A7 
 *      BLACK BROWN RED ORANGE YELLW  GREEN  BLUEVIOLET
 *      (while holding the board up so that back of the board faces you)
 *      
 * Though, this can be assigned anyway, for this application of the Arduino, we will make the following
 * assignment
 
 * A1 is Cal Button. (Has to switch from A2 on my board, because A2 is damaged.)
 * A3 is connected to a switch that can ground this line. This will be used to lock tuning.
 * A6 is to i s connected to a voltage divider and used as a voltmeter.
 * A7 is connected to a center pin of good quality 100K or 10K linear potentiometer with the two other ends connected to
 * ground and +5v lines available on the connector. This implments the tuning mechanism
 */

#define POWER_METER (A0)
#define CAL_BUTTON (A1)
#define LOCK (A3)
#define VOLT_IN (A6)
#define ANALOG_TUNING (A7)

/** 
 *  The second set of 16 pins on the bottom connector are have the three clock outputs and the digital lines to control the rig.
 *  This assignment is as follows :
 *    Pin   1   2    3    4    5    6    7    8    9    10   11   12   13   14   15   16
 *         +5V +5V CLK0  GND  GND  CLK1 GND  GND  CLK2  GND  D2   D3   D4   D5   D6   D7  
 *  These too are flexible with what you may do with them, for the Raduino, we use them to :
 *  - TX_RX line : Switches between Transmit and Receive after sensing the PTT or the morse keyer
 *  These are not used at the moment.
 */

// * 6 will connect to the PTT line. TX= positive voltage
#define PTT   (6)
#define TX_RX (7) // Just in case a TX output is needed.




/**
 *  The raduino has a number of timing parameters, all specified in milliseconds 

 */
 
/**
 * The Raduino supports two VFOs : A and B and receiver incremental tuning (RIT). 
 * we define a variables to hold the frequency of the two VFOs, RITs
 * the rit offset as well as status of the RIT
 * 
 * To use this facility, wire up push button on A3 line of the control connector //NOT USED IN THIS SKETCH//
 */
#define VFO_A 0
#define VFO_B 1

char vfoActive = VFO_A;
unsigned long vfoA=7100000L, vfoB=14200000L, ritA, ritB, sideTone=800;

/**
 * Raduino needs to keep track of current state of the transceiver. These are a few variables that do it
 */

char inTx = 0;
char inLOCK = 0;



/** Tuning Mechanism of the Raduino
 *  We use a linear pot that has two ends connected to +5 and the ground. the middle wiper
 *  is connected to ANALOG_TUNNING pin. Depending upon the position of the wiper, the
 *  reading can be anywhere from 0 to 1024.
 *  The tuning control works in steps of 50Hz each for every increment between 50 and 950.
 *  Hence the turning the pot fully from one end to the other will cover 50 x 900 = 45 KHz.
 *  At the two ends, that is, the tuning starts slowly stepping up or down in 10 KHz steps 
 *  To stop the scanning the pot is moved back from the edge. 
 *  To rapidly change from one band to another, you press the function button and then
 *  move the tuning pot. Now, instead of 50 Hz, the tuning is in steps of 50 KHz allowing you
 *  rapidly use it like a 'bandset' control.
 *  To implement this, we fix a 'base frequency' to which we add the offset that the pot 
 *  points to. We also store the previous position to know if we need to wake up and change
 *  the frequency.
 */

#define INIT_BFO_FREQ (1199800L)
unsigned long baseTune =  7150000L;
unsigned long bfo_freq = 11998000L;

int  old_knob = 0;


#define LOWEST_FREQ  (7000000l)
#define HIGHEST_FREQ (7300000l)

long frequency, stepSize=100000;


 #define MODE_NORMAL (0)
 #define MODE_CALIBRATE (1)
 char mode = MODE_NORMAL;

/**
 * Display Routines
 * These two display routines print a line of characters to the upper and lower lines of the 16x2 display
 */

void printLine1(char *c){
  if (strcmp(c, printBuff)){
    lcd.setCursor(0, 0);
    lcd.print(c);
    strcpy(printBuff, c);
    count++;
  }
}

void printLine2(char *c){
  lcd.setCursor(0, 1);
  lcd.print(c);
}

/**
 * Building upon the previous  two functions, 
 * update Display paints the lines as per current state of the radio
 * 
 
 */

void updateDisplay(){
    sprintf(b, "%8ld", frequency);      
    sprintf(c, "%s%.2s.%.3s.%.2s", vfoActive == VFO_A ? "" : "B" , b,  b+2, b+5);

      strcat(c, " LSB");
      
    if (inTx)
      strcat(c, " TX");  //Print TX on display during TX
    else if (inLOCK)
      strcat(c, " LK");
    else
      strcat(c, "   ");
          
    printLine1(c);
   
}

/**
 * To use calibration sets the accurate readout of the tuned frequency
 * To calibrate, follow these steps:
 * 1. Tune in a signal that is at a known frequency.
 * 2. Now, set the display to show the correct frequency, 
 *    the signal will no longer be tuned up properly
 * 3. Press the CAL_BUTTON line to the ground (pin A2 - red wire)
 * 4. tune in the signal until it sounds proper.
 * 5. Release CAL_BUTTON
 * In step 4, when we say 'sounds proper' then, for a CW signal/carrier it means zero-beat 
 * and for SSB it is the most natural sounding setting.
 * 
 * Calibration is an offset that is added to the VFO frequency by the Si5351 library.
 * We store it in the EEPROM's first four bytes and read it in setup() when the Radiuno is powered up
 */
void calibrate(){
    int32_t cal;

    // The tuning knob gives readings from 0 to 1000
    // Each step is taken as 10 Hz and the mid setting of the knob is taken as zero
    cal = (analogRead(ANALOG_TUNING) * 10)-5000;

    // if the button is released, we save the setting
    // and delay anything else by 5 seconds to debounce the CAL_BUTTON
    // Debounce : it is the rapid on/off signals that happen on a mechanical switch
    // when you change it's state
    if (digitalRead(CAL_BUTTON) == HIGH){
      mode = MODE_NORMAL;
      printLine1((char *)"Calibrated      ");

      //calculate the correction factor in parts-per-billion (offset in relation to the osc frequency)
      cal = (cal * -1000000000LL) / (bfo_freq - frequency) ;
      //apply the correction factor     
      si5351.set_correction(cal);
      //Write the 4 bytes of the correction factor into the eeprom memory.
      EEPROM.write(0, (cal & 0xFF));
      EEPROM.write(1, ((cal >> 8) & 0xFF));
      EEPROM.write(2, ((cal >> 16) & 0xFF));
      EEPROM.write(3, ((cal >> 24) & 0xFF));
      printLine2((char *)"Saved.    ");
      delay(5000);
    }
    else {
      // while the calibration is in progress (CAL_BUTTON is held down), keep tweaking the
      // frequency as read out by the knob, display the chnage in the second line
      si5351.set_freq((bfo_freq - frequency) * 100LL, SI5351_CLK2); 
      sprintf(c, "offset:%d ", cal);
      printLine2(c);
      //calculate the correction factor in ppb and apply it
      cal = (cal * -1000000000LL) / (bfo_freq - frequency) ;
      si5351.set_correction(cal);
    }  
}


/**
 * The setFrequency is a little tricky routine, it works differently for USB and LSB
 * 
 * The BITX BFO is permanently set to lower sideband, (that is, the crystal frequency 
 * is on the higher side slope of the crystal filter).
 * 
 * LSB: The VFO frequency is subtracted from the BFO. Suppose the BFO is set to exactly 12 MHz
 * and the VFO is at 5 MHz. The output will be at 12.000 - 5.000  = 7.000 MHz
 * 
 * USB: The BFO is subtracted from the VFO. Makes the LSB signal of the BITX come out as USB!!
 * Here is how it will work:
 * Consider that you want to transmit on 14.000 MHz and you have the BFO at 12.000 MHz. We set
 * the VFO to 26.000 MHz. Hence, 26.000 - 12.000 = 14.000 MHz. Now, consider you are whistling a tone
 * of 1 KHz. As the BITX BFO is set to produce LSB, the output from the crystal filter will be 11.999 MHz.
 * With the VFO still at 26.000, the 14 Mhz output will now be 26.000 - 11.999 = 14.001, hence, as the
 * frequencies of your voice go down at the IF, the RF frequencies will go up!
 * 
 * Thus, setting the VFO on either side of the BFO will flip between the USB and LSB signals.
 */

void setFrequency(unsigned long f){
  uint64_t osc_f;
  
 {    si5351.set_freq((bfo_freq - f) * 100ULL, SI5351_CLK2);  }

  frequency = f;
}

/**
 * The Checkt TX toggles the T/R line. The current BITX wiring up doesn't use this
 * but if you would like to make use of RIT, etc, You must wireup an NPN transistor to to the PTT line
 * as follows :
 * emitter to ground, 
 * base to TX_RX line through a 4.7K resistr(as defined at the top of this source file)
 * collecter to the PTT line
 * Now, connect the PTT to the control connector's PTT line (see the definitions on the top)
 
 */


 
//TX PTT Instructions////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void checkTX(){
  
  // * inTX     : true when the radio is in transmit mode 
  // We don't check for ptt when transmitting cw
  // as long as the cwTimeout is non-zero, we will continue to hold the
  // radio in transmit mode

    
  if (digitalRead(PTT) == 1 && inTx == 0){
    inTx = 1;
    digitalWrite(TX_RX, 1);
    updateDisplay();
  }
	
  if (digitalRead(PTT) == 0 && inTx == 1){
    inTx = 0;
    digitalWrite(TX_RX, 0);
    updateDisplay();
  }
}


// Tuning Lock Instructions///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void checkLOCK(){  
     
  if (digitalRead(LOCK) == 0 && inLOCK == 0){
    inLOCK = 1;
    updateDisplay();
  }
  
  if (digitalRead(LOCK) == 1 && inLOCK == 1){
    inLOCK = 0;
    updateDisplay();
  }
}




//POWER_METER Instructions///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void doPOWER_METER(){
// -- draw bar graph from the analog value reading of pin A0
  
  if (inTx) {  
    lbg.drawValue(analogRead(POWER_METER), 925);
  }

  else { 
    lcd.setCursor(8, 2);
    lcd.print("K5JCT   ");
    }
}



// Voltmeter instructions/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float vin=0.0;
float temp=0.0;
float r1=98700.0;                       // Set the value of resister R1
float r2=9890.0;                       // Set the value of resister R2


unsigned long interval=500; // the time we need to wait
unsigned long previousMillis=0; // millis() returns an unsigned long.

void doVOLT(){

int analog_val=analogRead(VOLT_IN);      // read the value of analog pin A6 and store it in the variable analog_val
  temp = (analog_val * 5.07)/1024.0;   
  vin = temp /(r2/(r1+r2));
  if (vin<0.1)  {
    vin=0.0;
  }


unsigned long currentMillis = millis(); // grab current time
 
 // check if "interval" time has passed (500 milliseconds)
 if ((unsigned long)(currentMillis - previousMillis) >= interval) {
  
  lcd.setCursor(0, 1);        // set the cursor to column 0 line 1   
  lcd.print(vin);             // print the voltage
  lcd.print("V");
 
   // save the "current" time
   previousMillis = millis(); }
}

 


/**
 * The Tuning mechansim of the Raduino works in a very innovative way. It uses a tuning potentiometer.
 * The tuning potentiometer that a voltage between 0 and 5 volts at ANALOG_TUNING pin of the control connector.
 * This is read as a value between 0 and 1000. Hence, the tuning pot gives you 1000 steps from one end to 
 * the other end of its rotation. Each step is 50 Hz, thus giving approximately 50 Khz of tuning range.
 * When the potentiometer is moved to either end of the range, the frequency starts automatically moving
 * up or down in 10 Khz increments
 */

void doTuning(){
 unsigned long newFreq;
 
 int knob = analogRead(ANALOG_TUNING)-10;
 unsigned long old_freq = frequency;

  // the knob is fully on the low end, move down by 5 Khz and wait for 200 msec
 if (knob < 10 && frequency > LOWEST_FREQ) {
      baseTune = baseTune - 1000l;
      frequency = baseTune;
      setFrequency(frequency);
      printLine2((char *)"<<<<<<<<<<      ");
      updateDisplay();
      
      delay(100);
  } 
    // the knob is full on the high end, move up by 5 Khz and wait for 200 msec
  else if (knob > 1010 && frequency < HIGHEST_FREQ) {
     baseTune = baseTune + 1000l; 
     frequency = baseTune + 5000l;
     setFrequency(frequency);
     printLine2((char *)"      >>>>>>>>>>");
     updateDisplay();
     delay(100);
  }
 // the tuning knob is at neither extremities, tune the signals as usual
  else if (knob != old_knob){
    static char dir_knob;
    if ( (knob>old_knob) && ((dir_knob==1) || ((knob-old_knob) >5)) ||
         (knob<old_knob) && ((dir_knob==0) || ((old_knob-knob) >5)) )   {
       if (knob>old_knob) {
            dir_knob=1;
            frequency = baseTune + (5l * (knob-5));
       } else {
            dir_knob=0;
            frequency = baseTune + (5l * knob);
       }
       old_knob = knob;
       setFrequency(frequency);
       printLine2((char *)"                ");
       updateDisplay();
    }
  }
}



/**
 * setup is called on boot up
 * It setups up the modes for various pins as inputs or outputs
 * initiliaizes the Si5351 and sets various variables to initial state
 * 
 * Just in case the LCD display doesn't work well, the debug log is dumped on the serial monitor
 * Choose Serial Monitor from Arduino IDE's Tools menu to see the Serial.print messages
 */
void setup()
{
  int32_t cal;
  
  lcd.begin(16, 2);
  printBuff[0] = 0;
  printLine1((char *)"Raduino Field"); 
  printLine2((char *)"K5JCT        "); 
    
  // Start serial and initialize the Si5351
  Serial.begin(9600);
  analogReference(DEFAULT);
  Serial.println("*Raduino booting up\nv1.02\n");

  //configure the LOCK button to use the external pull-up
  pinMode(LOCK, INPUT);
  digitalWrite(LOCK, HIGH);

  pinMode(PTT, INPUT);
  digitalWrite(PTT, HIGH);

  pinMode(CAL_BUTTON, INPUT);
  digitalWrite(CAL_BUTTON, HIGH);

  digitalWrite(TX_RX, 0);
  delay(1000);

  //fetch the correction factor from EEPROM
  EEPROM.get(0, cal);
  Serial.println("fetched correction factor from EEPROM:");
  Serial.println(cal);
  //initialize the SI5351 and apply the correction factor
  si5351.init(SI5351_CRYSTAL_LOAD_8PF,25000000l,cal);
  
  Serial.println("*Initiliazed Si5351\n");
  
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLB);
  Serial.println("*Fixed PLL\n");  
  //si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 1);
  Serial.println("*Output enabled PLL\n");
  si5351.set_freq(500000000l , SI5351_CLK2);   
  
  Serial.println("*Si5350 ON\n");       
  mode = MODE_NORMAL;
  delay(10);
}

void loop(){



   if (digitalRead(CAL_BUTTON) == LOW && mode == MODE_NORMAL){
    mode = MODE_CALIBRATE;    
    si5351.set_correction(0);
    printLine1((char *)"Calibrating: Set");
    printLine2((char *)"to zerobeat.    ");
    delay(2000);
    return;
  }
  else if (mode == MODE_CALIBRATE){
    calibrate();
    delay(50);
    return;
  }


  checkTX();
  checkLOCK();
  doPOWER_METER();
  doVOLT();

  
  if (!(digitalRead(LOCK) == 0 ))    
  doTuning(); 
  delay(50); 
  
}



