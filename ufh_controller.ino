/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */
#include <Firmata.h>
#include <Filters.h>
#include <ArduinoJson.h>

// create a one pole (RC) lowpass filter
FilterOnePole subFloorTempFilter( LOWPASS, 0.001);
FilterOnePole middleFloorTempFilter( LOWPASS, 0.001);
FilterOnePole fiveVoltRail( LOWPASS, 0.001);

boolean toggle1 = 0;

// the setup function runs once when you press reset or power the board
void setup() 
{
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(9600);

  subFloorTempFilter.setToNewValue(getTemp(0));
  middleFloorTempFilter.setToNewValue(getTemp(1));
  fiveVoltRail.setToNewValue(analogRead(2));
  
  cli();
  
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();
}  

int ufhActive = 0;
float targetTemp = 24.0;
long lastMillis = 0;
long lastMillis2 = 0;
unsigned long onTimeCounter = 0;
char outputString[200];
int incomingByte = 0;

// the loop function runs over and over again forever
void loop() {

  if(millis() - lastMillis2 >= 1000)
  {
    lastMillis2 = millis();
    
    //Check for new target temp
    if(Serial.available() > 0)
    {
      incomingByte = Serial.read();
      
      if(incomingByte >= 0 && incomingByte <= 40)targetTemp = incomingByte;
    }    
    
    if(middleFloorTempFilter.output() < targetTemp) 
    {
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
      ufhActive = 1;
    }
    else if(middleFloorTempFilter.output() > (targetTemp + 0.5))
    {
      digitalWrite(13,LOW);
      ufhActive = 0;
    }
    
    if(ufhActive == 1) onTimeCounter++;
  
    subFloorTempFilter.input(getTemp(0));
    middleFloorTempFilter.input(getTemp(1));
    fiveVoltRail.input(analogRead(2));
  
    //Serial.print("Filtered Subfloor Temperature: "); Serial.print(subFloorTempFilter.output()); Serial.print("\n");
    //Serial.print("Filtered Middle Floor Temperature: "); Serial.print(middleFloorTempFilter.output()); Serial.print("\n");
    if(millis()-lastMillis > 30000)
    {
      lastMillis = millis();
      //----------------JSON
      StaticJsonBuffer<200> jsonBuffer;
    
      JsonObject& root = jsonBuffer.createObject();
      root["kitchen_sub_floor"] = subFloorTempFilter.output();
      root["kitchen_mid_floor"] = middleFloorTempFilter.output();
      root["fiveVoltRail"] = fiveVoltRail.output();
      root["ufhElementOnTimeSecs"] = onTimeCounter;
      root["ufh_element_active"] = ufhActive;
      root["ufhTgt"] = targetTemp;
      
      root.printTo(Serial);
    }
  }
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
  //generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle1){
    digitalWrite(LED_BUILTIN,HIGH);
    toggle1 = 0;
  }
  else{
    digitalWrite(LED_BUILTIN,LOW);
    toggle1 = 1;
  }
}

//--------------------------
double getTemp(int analogueChannel) {
  // Inputs ADC Value from Thermistor and outputs Temperature in Celsius
  int RawADC = analogRead(analogueChannel);
//RawADC = testTemp;
  long Resistance;
  double Temp;

  // Assuming a 10k Thermistor.  Calculation is actually: Resistance = (1024/ADC)
  Resistance=((10240000/RawADC) - 10000);

  /******************************************************************/
  /* Utilizes the Steinhart-Hart Thermistor Equation:				*/
  /*    Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}		*/
  /*    where A = 0.001129148, B = 0.000234125 and C = 8.76741E-08	*/
  /******************************************************************/
  Temp = log(Resistance);
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convert Kelvin to Celsius

#ifdef DEBUG
  Serial.print("ADC: "); Serial.print(RawADC); Serial.print("/1024");  // Print out RAW ADC Number
  Serial.print(", Volts: "); Serial.print(((RawADC*4.860)/1024.0),3);   // 4.860 volts is what my USB Port outputs.
  Serial.print(", Resistance: "); Serial.print(Resistance); Serial.print("ohms, ");
  Serial.print("Temperature: "); Serial.print(Temp); Serial.print("\n");
#endif
  // Uncomment this line for the function to return Fahrenheit instead.
  //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert to Fahrenheit

  return Temp;  // Return the Temperature
}


