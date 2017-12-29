/*
	Tombarks - UFH controller
	For use with SSR controlled electric UFH
	
	Functional Requirements
	
	1. Get target temperature (C) over serial port
	2. Send status information to serial port
	3. PWM control of SSR to calculated duty
	4. Calculate SSR duty cycle using closed loop control algorithm to target temperature (C)
	5. Store length of time UFH element has been active since uC power on
 */
#include <Firmata.h> 
#include <Filters.h> 
#include <ArduinoJson.h>

// create a one pole (RC) lowpass filter
FilterOnePole subFloorTempFilter(LOWPASS, 0.0001);
FilterOnePole middleFloorTempFilter(LOWPASS, 0.0001);

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);

  //Set LED as output
  pinMode(LED_BUILTIN, OUTPUT);

  //Start serial to get target temp
  Serial.begin(9600);

  //Set initial value of filters to latest sample
  subFloorTempFilter.setToNewValue(getTemp(0));
  middleFloorTempFilter.setToNewValue(getTemp(1));

  //Setup the timer for the PWM control of the SSR.
  cli();

  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0; //initialize counter value to 0
  
  // set compare match register for 1hz increments
  OCR1A = 0; //must be <65536
  
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);

  sei();
}

int ufhActive = 0;
float targetTemp = 0.0;
volatile unsigned long onTimeCounter = 0;
unsigned long onTimeSeconds = 0;

float dutyCycle = 0;

volatile unsigned char disableTimerInterrupts = 0;

long sendJSONTimer = 0;
long sendJSONFreq = 30000; //30s

long readTempTimer = 0;
long readTempFreq = 100; //100ms

long getTargetTimer = 0;
long getTargetFreq = 100; //100ms

long computeOnTimeTimer = 0;
long computeOnTimeFreq = 10000; //10s

long computeDutyTimer = 0;
long computeDutyFreq = 60000; //60s

int incomingByte = 0;

// the loop function runs over and over again forever
void loop() {

  //Send JSON
  if (millis() - sendJSONTimer > sendJSONFreq) {
    sendJSONTimer = millis();
    sendJSON();
  }

  //Read Temperature
  if (millis() - readTempTimer > readTempFreq) {
    readTempTimer = millis();
    readTemp();
  }

  //get target temperature
  if (millis() - getTargetTimer > getTargetFreq) {
    getTargetTimer = millis();
    getTarget();
  }

  //Compute on time
  if (millis() - computeOnTimeTimer > computeOnTimeFreq) {
    computeOnTimeTimer = millis();
    computeOnTime();
  }

  //Compute duty
  if (millis() - computeDutyTimer > computeDutyFreq) {
    computeDutyTimer = millis();
    computeDuty();
    controlPWMTimer();
  }
}

//Compute duty
void computeDuty() {
  dutyCycle = (float) targetTemp;
}

//Set PWM timer
void controlPWMTimer() {
  if (dutyCycle > 0.0 && dutyCycle <= 100.0) {
    //Calculate the timer value from the duty cycle
    float timerOCR = 65535.0 / 100.0 * dutyCycle;

    //Enable timer
    cli();
    OCR1A = (unsigned short) timerOCR;
    TIMSK1 |= (1 << OCIE1A);
    TIMSK1 |= (1 << TOIE1);
    disableTimerInterrupts = 0;
    sei();
  } else disableTimerInterrupts = 1;
}

//Compute on time
void computeOnTime() {
  //grab local copy of volatile varible
  unsigned long onTimeCounterLocal;
  cli();
  onTimeCounterLocal = onTimeCounter;
  sei();

  //15625hz timer
  while (onTimeCounterLocal >= 15625) {
    onTimeCounterLocal -= 15625;
    onTimeSeconds++;
  }
}

//read target temperature from serial
void getTarget() {
  //Check for new target temp
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte >= 0 && incomingByte <= 40) targetTemp = incomingByte;
  }
}

//Read underfloor temperature
void readTemp() {
  subFloorTempFilter.input(getTemp(0));
  middleFloorTempFilter.input(getTemp(1));
}

//Send JSON over serial
void sendJSON() {

  //----------------JSON
  StaticJsonBuffer < 200 > jsonBuffer;
  JsonObject & root = jsonBuffer.createObject();
  root["kitchen_sub_floor"] = subFloorTempFilter.output();
  root["kitchen_mid_floor"] = middleFloorTempFilter.output();
  root["fiveVoltRail"] = 0;
  root["ufhElementOnTimeSecs"] = onTimeSeconds;
  root["ufh_element_active"] = dutyCycle;
  root["ufhTgt"] = targetTemp;

  root.printTo(Serial);
}

//--------------------------

//Timer compare vector
ISR(TIMER1_COMPA_vect) {
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(13, LOW);
  onTimeCounter += OCR1A;

  //Disable timer when duty is 0;
  if (disableTimerInterrupts == 1) {
    //disable timer compare interrupt
    TIMSK1 &= ~(1 << OCIE1A);
    TIMSK1 &= ~(1 << TOIE1);

    disableTimerInterrupts = 0;
  }
}

//Timer overflow vector
ISR(TIMER1_OVF_vect) {
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(13, HIGH);
}

//--------------------------
double getTemp(int analogueChannel) {
  // Inputs ADC Value from Thermistor and outputs Temperature in Celsius
  int RawADC = analogRead(analogueChannel);
  //RawADC = testTemp;
  long Resistance;
  double Temp;

  // Assuming a 10k Thermistor.  Calculation is actually: Resistance = (1024/ADC)
  Resistance = ((10240000 / RawADC) - 10000);

  /******************************************************************/
  /* Utilizes the Steinhart-Hart Thermistor Equation:				*/
  /*    Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}		*/
  /*    where A = 0.001129148, B = 0.000234125 and C = 8.76741E-08	*/
  /******************************************************************/
  Temp = log(Resistance);
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15; // Convert Kelvin to Celsius

  #ifdef DEBUG
  Serial.print("ADC: ");
  Serial.print(RawADC);
  Serial.print("/1024"); // Print out RAW ADC Number
  Serial.print(", Volts: ");
  Serial.print(((RawADC * 4.860) / 1024.0), 3); // 4.860 volts is what my USB Port outputs.
  Serial.print(", Resistance: ");
  Serial.print(Resistance);
  Serial.print("ohms, ");
  Serial.print("Temperature: ");
  Serial.print(Temp);
  Serial.print("\n");
  #endif
  // Uncomment this line for the function to return Fahrenheit instead.
  //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert to Fahrenheit

  return Temp; // Return the Temperature
}
