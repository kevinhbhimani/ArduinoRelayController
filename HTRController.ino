#include <PID.h>


const int HTRCycleTime = 8192;

const int SensorPin = A0;

const int HeaterPin = 9;

const double Kp = 17.0;
const double Ki = 122.4;
const double Kd = 7.65;
const double SetPoint = 0.25; // should be about 8 psi

double pressure;
double power = 0.0;
bool heaterOn = false;
unsigned long windowStopTime = 0;

PID HTRPID(&pressure, &power, SetPoint, Kp, Ki, Kd);

void setup()
{
  analogReference(DEFAULT); // use the 5 V reference
  HTRPID.sampleTime = HTRCycleTime;
  pinMode(HeaterPin, OUTPUT);
  Serial.begin(9600);
  HTRPID.inAuto = true;
  pinMode(5, OUTPUT);//for LED
  pinMode(6, OUTPUT);//for LED
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
      //set iterm function
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
    case 't' : // set iTerm
      val = Serial.parseFloat();
      HTRPID.iTerm = val;
      Serial.print("Set iTerm to ");
      Serial.println(val);
    break;
    
    case '?' : // display settings
    Serial.print("Output is ");
    Serial.println(power);
    Serial.print("Setpoint is ");
    Serial.println(HTRPID.setPoint);
    Serial.print("Kp is ");
    Serial.println(HTRPID.Kp);
    Serial.print("Ki is ");
    Serial.println(HTRPID.Ki);
    Serial.print("Kd is ");
    Serial.println(HTRPID.Kd);
    Serial.print("Automatic control is ");
    Serial.println(HTRPID.inAuto);
    Serial.print("PID pTerm is ");
    Serial.println(HTRPID.pTerm);
    Serial.print("PID iTerm is ");
    Serial.println(HTRPID.iTerm);
    Serial.print("PID dTerm is ");
    Serial.println(HTRPID.dTerm);
  }
  
}

void loop()
{
  digitalWrite(5, HIGH);//Turns on the Green LED 
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
      digitalWrite(6, HIGH); //turn on the red LED  

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
      digitalWrite(6, LOW); //Turns off the Red LED
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
      digitalWrite(6, LOW); //turn off the Red LED  
      digitalWrite(5, HIGH); //turn on the Red LED  
    }
  }
}
