#include <PID.h>
#include <EEPROM.h>

const int HTRCycleTime = 8192;

const int SensorPin = A0;

const int HeaterPin = 9;

const double Kp = 0.1;
const double Ki = 0.01;
const double Kd = 0.01;
const double SetPoint = 8.0; // should be about 8 psi

// strings take up memory, so those that occur in multiple places should refer to a global string
const char PressureMessage[] PROGMEM = {"Measured pressure is "}; 
const char MilliSecondsMessage[] PROGMEM = {" ms."};
const char PSIGMessage[] PROGMEM = {" psig."};

const double PowerPeriodInMs = 60.0/1000.0; // 60 Hz in USA 

double slopeADCtoPSIG = 0.0366569;
double offsetADCtoPSIG = -3.75;
double pressure;
double power = 0.0;
bool heaterOn = false;
unsigned long windowStopTime = 0;
int loopDelay = 1;

PID HTRPID(&pressure, &power, SetPoint, Kp, Ki, Kd);

void SaveToEEPROM( int addrEEPROM = 0 )
{
  EEPROM.put(addrEEPROM,HTRPID.Kp);
  addrEEPROM+=sizeof(double);
  EEPROM.put(addrEEPROM,HTRPID.Ki);
  addrEEPROM+=sizeof(double);
  EEPROM.put(addrEEPROM,HTRPID.Kd);
  addrEEPROM+=sizeof(double);
  EEPROM.put(addrEEPROM,HTRPID.setPoint);
  addrEEPROM+=sizeof(double);
  EEPROM.put(addrEEPROM,HTRPID.outMin);
  addrEEPROM+=sizeof(double);
  EEPROM.put(addrEEPROM,HTRPID.outMax);
  addrEEPROM+=sizeof(double);
  EEPROM.put(addrEEPROM,slopeADCtoPSIG);
  addrEEPROM+=sizeof(double);
  EEPROM.put(addrEEPROM,offsetADCtoPSIG);
  addrEEPROM+=sizeof(double);
  EEPROM.put(addrEEPROM,HTRPID.sampleTime);
  addrEEPROM+=sizeof(long);
  EEPROM.put(addrEEPROM,loopDelay);
}

void LoadFromEEPROM( int addrEEPROM = 0 )
{
  EEPROM.get(addrEEPROM,HTRPID.Kp);
  addrEEPROM+=sizeof(double);
  EEPROM.get(addrEEPROM,HTRPID.Ki);
  addrEEPROM+=sizeof(double);
  EEPROM.get(addrEEPROM,HTRPID.Kd);
  addrEEPROM+=sizeof(double);
  EEPROM.get(addrEEPROM,HTRPID.setPoint);
  addrEEPROM+=sizeof(double);
  EEPROM.get(addrEEPROM,HTRPID.outMin);
  addrEEPROM+=sizeof(double);
  EEPROM.get(addrEEPROM,HTRPID.outMax);
  addrEEPROM+=sizeof(double);
  EEPROM.get(addrEEPROM,slopeADCtoPSIG);
  addrEEPROM+=sizeof(double);
  EEPROM.get(addrEEPROM,offsetADCtoPSIG);
  addrEEPROM+=sizeof(double);
  EEPROM.get(addrEEPROM,HTRPID.sampleTime);
  addrEEPROM+=sizeof(long);
  EEPROM.get(addrEEPROM,loopDelay);
}


void setup()
{
  analogReference(DEFAULT); // use the 5 V reference
  HTRPID.sampleTime = HTRCycleTime;
  pinMode(HeaterPin, OUTPUT);
  Serial.begin(9600);
  HTRPID.inAuto = true;
  pinMode(5, OUTPUT);//for LED
  pinMode(6, OUTPUT);//for LED
  // load settings from EEPROM
  long iTest;
  EEPROM.get(0,iTest);
  if (iTest!=-1) // if the EEPROM has never been written to then the stored values should be 255 in every byte, which should be -1 for a signed long int.
    LoadFromEEPROM();
}

void parseSerial()
{
  double fVal;
  int iVal;
  if (Serial.available()<2) return; // if there's not at least two bytes available there's no valid incomming command
  switch (Serial.read()) // read the first byte of the incomming data
  {
    case 'p' : // set Kp
      fVal = Serial.parseFloat();
      HTRPID.Kp = fVal;
      Serial.print(F("Set Kp to ")); // F() stores the string in program storage flash memory, leaving more dynamic memory available
      Serial.println(fVal);
    break;
    case 'i' : // set Ki
      fVal = Serial.parseFloat();
      HTRPID.Ki = fVal;
      Serial.print(F("Set Ki to "));
      Serial.println(fVal);
    break;
    case 'd' : // set Kd
      fVal = Serial.parseFloat();
      HTRPID.Kd = fVal;
      Serial.print(F("Set Kd to "));
      Serial.println(fVal);
    break;
    case 'a' : // toggle auto
      HTRPID.inAuto = !HTRPID.inAuto;
      Serial.print(F("Heater control in ")); // using this extra function call saves a whopping 6 bytes
      Serial.println(HTRPID.inAuto?F("auto."):F("manual."));
    break;
    case 's' : // change setpoint, this is in fraction of full scale output of ADC
      fVal = Serial.parseFloat();
      HTRPID.setPoint = fVal;
      Serial.print(F("Setpoint is set to "));
      Serial.println(fVal);
    break;
    case 'o' : // set output, this is fraction of time heater is on
      fVal = Serial.parseFloat();
      HTRPID.SetOutput(fVal);
      Serial.print(F("Set output to "));
      Serial.println(fVal);
    break;
    case 't' : // set iTerm
      fVal = Serial.parseFloat();
      HTRPID.iTerm = fVal;
      Serial.print(F("Set iTerm to "));
      Serial.println(fVal);
    break;
    case 'w' : // set cycle time
      iVal = Serial.parseInt();
      HTRPID.sampleTime = iVal;
      Serial.print(F("Set heater cycle time to "));
      Serial.print(iVal);
      Serial.println(MilliSecondsMessage);
    break;
    case '*' : // set loop delay
      iVal = Serial.parseInt();
      loopDelay = iVal;
      Serial.print(F("Set loop delay time to "));
      Serial.print(iVal);
      Serial.println(MilliSecondsMessage);
    break;
    case 'm' : // set max output
      fVal = Serial.parseFloat();
      HTRPID.outMax = fVal;
      Serial.print(F("Set maximum output to "));
      Serial.println(fVal);
    break;    
    case 'n' : // set minimum output
      fVal = Serial.parseFloat();
      HTRPID.outMin = fVal;
      Serial.print(F("Set minimum output to "));
      Serial.println(fVal);
    break;    
    case 'g' : // set slopeADCtoPSIG for sensor calibration
      fVal = Serial.parseFloat();
      slopeADCtoPSIG = fVal;
      Serial.print(F("Set slope of ADC to pressure(psig) conversion to "));
      Serial.println(fVal);
    break;    
    case 'z' : // set offsetADCtoPSIG for sensor calibration
      fVal = Serial.parseFloat();
      offsetADCtoPSIG = fVal;
      Serial.print(F("Set offset of ADC to pressure(psig) converstion to "));
      Serial.println(fVal);
    break;    
    case 'v' : // save settings
      Serial.println(F("Saving settings to EEPROM."));
      SaveToEEPROM();
    break;
    
    case '?' : // display settings
    Serial.println(F("Commands are:"));
    Serial.println(F("p # - set Kp"));
    Serial.println(F("i # - set Ki"));
    Serial.println(F("d # - set Kd"));
    Serial.println(F("a - toggle between auto and manual"));
    Serial.println(F("s # - change setpoint"));
    Serial.println(F("o # - set output power"));
    Serial.println(F("t # - set iTerm of PID controller"));
    Serial.println(F("w # - set heater cycle time in milliseconds"));
    Serial.println(F("m # - set maximum PID output"));
    Serial.println(F("n # - set minimum PID output"));
    Serial.println(F("g # - set ADC to pressure slope"));
    Serial.println(F("z # - set ADC to pressure zero offset"));
    Serial.println(F("v - save settings to EEPROM, will always be in auto after a reset"));    
    Serial.println(F("Note that the PID output is limited by adjusting the iTerm. While the heater power can only be between 0 (always off) to 1 (always on) larger limits on the PID loop ouput are useful for preventing changes to the iTerm when first determining control values."));
    Serial.print(F("Output is "));
    Serial.println(power);
    Serial.print(F("Setpoint is "));
    Serial.print(HTRPID.setPoint);
    Serial.println(PSIGMessage);
    Serial.print(F("Output range is "));
    Serial.print(HTRPID.outMin);
    Serial.print(F(" to "));
    Serial.print(HTRPID.outMax);
    Serial.println(PSIGMessage);
    Serial.print(F("Kp is "));
    Serial.println(HTRPID.Kp);
    Serial.print(F("Ki is "));
    Serial.println(HTRPID.Ki);
    Serial.print(F("Kd is "));
    Serial.println(HTRPID.Kd);
    Serial.print(F("ADC to pressure conversion equation is ADC * "));
    Serial.print(slopeADCtoPSIG);
    Serial.print(F(" + "));
    Serial.println(offsetADCtoPSIG);
    Serial.print(F("Control is in "));
    Serial.println(HTRPID.inAuto?F("auto"):F("manual"));
    Serial.print(F("PID pTerm is "));
    Serial.println(HTRPID.pTerm);
    Serial.print(F("PID iTerm is "));
    Serial.println(HTRPID.iTerm);
    Serial.print(F("PID dTerm is "));
    Serial.println(HTRPID.dTerm);
    Serial.print(F("Heater cycle time is "));
    Serial.print(HTRPID.sampleTime);
    Serial.println(MilliSecondsMessage);
    Serial.print(F("Loop delay is "));
    Serial.print(loopDelay);
    Serial.println(MilliSecondsMessage);
  }
  
}


void loop()
{
  digitalWrite(5, HIGH);//Turns on the Green LED 
  parseSerial();
  // read current pressure
  pressure = float(analogRead(SensorPin)) * slopeADCtoPSIG + offsetADCtoPSIG; // 10 bit ADC with 5V full scale, 0.5 V is 0 psig, 4.5 V is 30 psig
  // compute the PID setting with measured pressure
  // Heater power is controller with pulse width modulation. 
  if (HTRPID.Compute()) // If the PID recalculated heater power, this is also the start of the heater power window
  {
    // turn the heater on at the beginning of the window and determine when to turn the heater off
    windowStopTime = (power > PowerPeriodInMs / HTRPID.sampleTime) ? power * HTRPID.sampleTime : 0; // if it won't be at least one cycle, don't turn on the heater
    if (windowStopTime>HTRPID.sampleTime) windowStopTime = HTRPID.sampleTime;
    if (Serial)
    {
      Serial.print(PressureMessage);
      Serial.print(pressure);
      Serial.println(PSIGMessage);
      Serial.print(F("Turning heater on for "));
      Serial.print(windowStopTime);
      Serial.print(F(" out of "));
      Serial.print(HTRPID.sampleTime);
      Serial.println(MilliSecondsMessage);
    }
    heaterOn = (windowStopTime > 0);
    digitalWrite(HeaterPin, heaterOn);
    digitalWrite(6, heaterOn); //turn on the red LED  
    windowStopTime += HTRPID.lastTime; // The heater pulse uses the same timing variable as the PID calculation to ensure synchronization.
  }
  else // if (HTRPID.Compute())
  {
    // If the heater is on and it's time to turn it off, then do so
    unsigned long now = millis();
    if ((now > windowStopTime) && heaterOn) 
    {
      if (Serial)
      {
        Serial.print(PressureMessage);
        Serial.print(pressure);
        Serial.println(F(" turning heater off."));
      }
      heaterOn = false;
      digitalWrite(HeaterPin, LOW);
      digitalWrite(6, LOW); //turn off the Red LED  
    }
  }
  if (loopDelay) delay(loopDelay);
}
