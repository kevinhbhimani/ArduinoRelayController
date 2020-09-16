#include <PID.h>


const int HTRCycleTime = 8192;

const int SensorPin = A0;

const int HeaterPin = 9;

const double Kp = 9.0;
const double Ki = 0.1;
const double Kd = 1.0;
const double SetPoint = 0.3; // should be about 8 psi

double pressure;
double power = 0.0;
bool heaterOn = false;
unsigned long windowStopTime = 0;

PID HTRPID(&pressure, &power, SetPoint, Kp, Ki, Kd);

void setup()
{
  analogReference(INTERNAL); // use the 1.1 V internal reference
  HTRPID.sampleTime = HTRCycleTime;
  pinMode(HeaterPin, OUTPUT);
  Serial.begin(9600);
  HTRPID.inAuto = true;
}

void parseSerial()
{
  double val;
  if (Serial.available()<2) return; // if there's not at least two bytes available there's no valid incomming command
  switch (Serial.read()) // read the first byte of the incomming data
  {
    case 'p' : // set Kp
      val = Serial.parseFloat();
      HTRPID.Kp = val;
      Serial.print("Set Kp to ");
      Serial.println(val);
    break;
    case 'i' : // set Ki
      val = Serial.parseFloat();
      HTRPID.Ki = val;
      Serial.print("Set Ki to ");
      Serial.println(val);
    break;
    case 'd' : // set Kd
      val = Serial.parseFloat();
      HTRPID.Kd = val;
      Serial.print("Set Kd to ");
      Serial.println(val);
    break;
    case 'a' : // toggle auto
      HTRPID.inAuto = !HTRPID.inAuto;
      Serial.println(HTRPID.inAuto?"Heater control in auto.":"Heater control in manual.");
    break;
    case 's' : // change setpoint, this is in fraction of full scale output of ADC
      val = Serial.parseFloat();
      HTRPID.setPoint = val;
      Serial.print("Setpoint is set to ");
      Serial.println(val);
    break;
    case 'o' : // set output, this is fraction of time heater is on
      val = Serial.parseFloat();
      HTRPID.SetOutput(val);
      Serial.print("Set output to ");
      Serial.println(val);
    break;
    
  }
  
}

void loop()
{
  parseSerial();
  // read current pressure
  pressure = float(analogRead(SensorPin)) / 1024.0;
  // compute the PID setting with measured pressure
  // Heater power is controller with pulse width modulation. 
  if (HTRPID.Compute()) // If the PID recalculated heater power, this is also the start of the heater power window
  {
    // turn the heater on at the beginning of the window and determine when to turn the heater off
    windowStopTime = power * HTRCycleTime ;
    if (power > 16.6667 / HTRCycleTime) // only turn on if it will be on for at least one 60 Hz cycle
    {
      if (Serial)
      {
        Serial.print("Measured pressure is ");
        Serial.print(pressure);
        Serial.print(" turning heater on for ");
        Serial.print(windowStopTime);
        Serial.println(" out of 8192 ms.");
      }
      heaterOn = true;
      digitalWrite(HeaterPin, HIGH);
    }
    else
    {
      if (Serial)
      {
        Serial.print("Measured pressure is ");
        Serial.print(pressure);
        Serial.println(" not turning heater on.");
      }
      heaterOn = false;
      digitalWrite(HeaterPin, LOW);
    }
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
        Serial.print("Measured pressure is ");
        Serial.print(pressure);
        Serial.println(" turning heater off.");
      }
      heaterOn = false;
      digitalWrite(HeaterPin, LOW);
    }
  }
}
