

/* six pin module - sender-sensor*/


#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_LSM303_U.h>
#include <Event.h>
#include <Timer.h>

Timer t;

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);



// PIN GROUP
int tAIn    = 0;  // Pin for Temperature Analog Reading
int lAIn    = 1;  // Pin for Photoresistor Analog Reading
int acPower = 4;  // Pin for Accelerometer-Magnetometer Power
int tPower  = 3;  // Pin for Temperature Power
int lPower  = 2;  // Pin for Photoresistor Power
int buzzPin = 22; // Pin for Buzz

int BTRX    = 10; // RX Pin
int BTTX    = 11; // TX Pin


// FLAGS FOR PERIODIC MODE
int repeatMag;
int repeatAccel;
int repeatTemp;

int repeatMagDelay = 0;
int repeatAccelDelay = 0;
int repeatTempDelay = 0;



// FLAGS FOR ALARM MODE

int alarmEnableTemp = 0;
int alarmOnDark = 0;
bool buzzOn = false;


// STORE USER-DEFINED TEMPERATURE THRESHOLD

float thresholdTemp = 0;

SoftwareSerial bluetooth(BTRX, BTTX); // RX, TX

static int rpNum = 0;

// Turn off all sensors
void turnOffAllSensors() {
  digitalWrite(acPower, LOW);
  digitalWrite(tPower, LOW);
  digitalWrite(lPower, LOW);
}



// Turn on all sensors
void turnOnAllSensors() {
  digitalWrite(acPower, HIGH);
  digitalWrite(tPower, HIGH);
  digitalWrite(lPower, HIGH);
}




// Turn off periodic mode
void turnOffPeriodicMode() {
  t.stop(repeatMag);
  t.stop(repeatAccel);
  t.stop(repeatTemp);

}



// Turn off alarm mode
void turnOffAlarmMode() {
  t.stop(alarmEnableTemp);
  t.stop(alarmOnDark);
  digitalWrite(buzzPin, LOW);
}







// Display temperature upon user-defined threshold
void displayTempOnThreshold() {
  //get temperature via analog read
  float currentTemp = thermoQuickVal();
  if (currentTemp > thresholdTemp) {
    displayTempDetails();
    digitalWrite(buzzPin, buzzOn = !buzzOn);
  } else {
    digitalWrite(buzzPin, LOW);
  }
}



// Check temperature
float thermoQuickVal() {
  float referenceVoltage = 5.0;                       // set reference voltage to 5.0 (for Arduino Mega 2560 only)
  int thermo = analogRead(tAIn);                     // read raw sensor data (note: units are not C or F yet)
  float volts = (thermo * referenceVoltage) / 256.0;  // TMP36-RefVoltage formula = reading*refVoltage / 8-bit res (aka 256.0)
  float thermoC = (volts - 0.5) * 10.0;               // Formula for Celcius Conversion, adapted-fixed for volts instead of mV
  // Source: learn.adafruit.com/tmp36-temperature-sensor/overview
  float thermoF = (thermoC * 9.0 / 5.0) + 32.0;       // Farenheit Conversion
  return thermoF;
}




// Check amount of light via photosensor and alarm accordingly
void photosense() {
  int lightResist = abs(analogRead(lAIn) - 1023);     // read the input pin, minus 1023, turn to abs value

  if (lightResist > 500) {                   // if dark
    bluetooth.println("Alarm on Dark!");
    digitalWrite(buzzPin, buzzOn = !buzzOn);
  } else {
    digitalWrite(buzzPin, LOW);
  }

}


// Main setup function
void setup() {
  bluetooth.begin(9600);
  bluetooth.println("Press 0# to start");
  delay(1000);
  setPins();
  accel.begin();
  mag.begin();
}





// Forever loop
void loop() { // run over and over

  mainMenu();

  t.update();
}



// Initial status and mode of pins
void setPins() {
  digitalWrite(acPower, HIGH);
  digitalWrite(tPower, HIGH);
  digitalWrite(lPower, HIGH);
  digitalWrite(buzzPin, LOW);
  pinMode(tAIn, INPUT); // Temp Sensor In
  pinMode(lAIn, INPUT); // Photo Sensor In
  pinMode(acPower, OUTPUT);
  pinMode(tPower, OUTPUT);
  pinMode(lPower, OUTPUT);
  pinMode(buzzPin, OUTPUT);
}


// Basic Commands for Main Menu
void menuMessage() {
  bluetooth.println("");
  bluetooth.println("MAIN MENU! Press a number then press #...");
  bluetooth.println("0: General Sensor Info");
  bluetooth.println("1: Turn On-Off a Sensors");
  bluetooth.println("2: On-Demand Report");
  bluetooth.println("3: Periodic Report Mode");
  bluetooth.println("4: Alarm Mode");
  bluetooth.println("5: Change Pin");
  bluetooth.println("6: Change Pin Mode and Status");
  bluetooth.println("7: Disarm Alarm");
  bluetooth.println("");
}


// Main Menu
void mainMenu() { // run over and over
  //RECEIVE




  if (bluetooth.available()) {
    //bluetooth.println("ACK test");

    int input = bluetooth.read();

    switch (input) {

      case 0: // General Sensor Information
        displaySensorDetails();
        bluetooth.println();
        break;

      case 1: // On-Off Switch for Sensors
        switchOnOffASensor();
        bluetooth.println();
        break;

      case 2: // On-Demand Mode
        onDemandMode();
        bluetooth.println();
        break;

      case 3: // Periodic Updates Mode
        periodicMode();
        bluetooth.println();
        break;

      case 4: // Alarm Threshold Mode
        alarmMode();
        bluetooth.println();
        break;

      case 5: // Set-Change Pin
        //sub-switch method goes here
        changePin();
        bluetooth.println();
        break;

      case 6: // Pin On-Off
        //sub-switch method goes here
        pinRequest();
        bluetooth.println();
        break;

      case 7: // Pin On-Off
        //sub-switch method goes here
        turnOffAlarmMode();
        bluetooth.println();
        break;

      default:
        bluetooth.println("Invalid Input");
        break;
    }
    menuMessage();
  }

  if (Serial.available()) {
    bluetooth.write(Serial.read());
  }


}




// Switch on and off a sensor
void switchOnOffASensor() {
  //Start afresh
  turnOffPeriodicMode();
  turnOffAlarmMode();

  // Prompt User for Decision
  bluetooth.println("Pick a sensor. 1=Accelerometer, 2=Temperature, or 3=Photocell.");
  while (bluetooth.available() == 0) {}  // Wait for User to Input Data
  int decision = bluetooth.read();    // Save User Decision

  if      (decision == 1)   {
    setAccelPower();
  }
  else if (decision == 2)   {
    setTempPower();
  }
  else if (decision == 3)   {
    setLightPower();
  }
  else            {
    bluetooth.println("Invalid input. ");
  }

  bluetooth.println("End of Power On-Off Method");
}








// Turn on or off the acceleromter sensor
void setAccelPower() {
  bluetooth.println("Accelerometer-Magnetometer");

  // Prompt User for choice of on or off
  bluetooth.println("Press 1 to Power ON, Press 2 to Power OFF");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int choice = bluetooth.read();  // Save User Decision

  // Verify choice is 1 or 2
  if (choice == 1) {
    digitalWrite(acPower, HIGH);
  } else if (choice == 2) {
    digitalWrite(acPower, LOW);
  } else {
    bluetooth.println("You entered an invalid choice for powering on-off the Accelerometer. Try again.");
  }
}






// Turn on or off the temperature sensor
void setTempPower() {
  bluetooth.println("Temperature Sensor");

  // Prompt User for choice of on or off
  bluetooth.println("Press 1 to Power ON, Press 2 to Power OFF");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int choice = bluetooth.read();  // Save User Decision

  // Verify choice is 1 or 2
  if (choice == 1) {
    digitalWrite(tPower, HIGH);
  } else if (choice == 2) {
    digitalWrite(tPower, LOW);
  } else {
    bluetooth.println("You entered an invalid choice for powering on-off the TMP36. Try again.");
  }
}





// Turn on or off the photoresistor
void setLightPower() {
  bluetooth.println("Photocell Sensor");

  // Prompt User for choice of on or off
  bluetooth.println("Press 1 to Power ON, Press 2 to Power OFF");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int choice = bluetooth.read();  // Save User Decision

  // Verify choice is 1 or 2
  if (choice == 1) {
    digitalWrite(lPower, HIGH);
  } else if (choice == 2) {
    digitalWrite(lPower, LOW);
  } else {
    bluetooth.println("You entered an invalid choice for powering on-off the photocell. Try again.");
  }
}




// On Demand Mode
void onDemandMode() {
  //Start afresh
  //turnOffPeriodicMode();
  //turnOffAlarmMode();

  //turnOffAllSensors();
  //turnOnAllSensors();

  // Prompt User for Decision
  bluetooth.println("Pick a sensor. 1=Accelerometer, 2=Compass, or 3=Temperature.");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int decision = bluetooth.read();    // Save User Decision

  if      (decision == 1)   {
    displayAccelDetails();
  }
  else if (decision == 2)   {
    displayMagDetails();
  }
  else if (decision == 3)   {
    displayTempDetails();
  }
  else            {
    bluetooth.println("Invalid input. ");
  }

  bluetooth.println("End of Power On-Demand Mode");
}





// Periodic Mode
void periodicMode() {
  //Start afresh
  //turnOffPeriodicMode();
  //turnOffAlarmMode();

  // Prompt User for Decision
  bluetooth.println("Pick a sensor. 1=Accelerometer, 2=Compass, or 3=Temperature.");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int decision = bluetooth.read();    // Save User Decision

  if      (decision == 1)   {
    setAccelDelay();
  }
  else if (decision == 2)   {
    setMagDelay();
  }
  else if (decision == 3)   {
    setTempDelay();
  }
  else            {
    bluetooth.println("Invalid input. ");
  }

  bluetooth.println("End of Periodic Method");
}




// Alarm Mode
void alarmMode() {
  //Start afresh
  //turnOffPeriodicMode();
  // Prompt User for Decision
  bluetooth.println("Pick a sensor. 1=Temperature, 2=Photocell.");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int decision = bluetooth.read();      // Save User Decision

  if    (decision == 1)     {
    setTempAlarm();
  }
  else if (decision == 2)   {
    setPhotoAlarm();
  }
  else                        {
    bluetooth.println("Invalid input. ");
  }

  bluetooth.println("End of Alarm Mode");

}



// Set Alarm Level for Temperature Sensor
void setTempAlarm() {
  bluetooth.println("Temperature Sensor");
  // Prompt User for Threshold value
  bluetooth.println("Enter a threshold value for temperature in Fahrenheit: (from 60-90F) ");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  thresholdTemp = bluetooth.read();   // Save User Decision

  if (thresholdTemp >= 60 && thresholdTemp <= 90) {
    alarmEnableTemp = t.every(500, displayTempOnThreshold);
  }
  else {
    bluetooth.println("Invalid input. Back to main menu.");
  }
}






// Set Alarm Level for Photoresistor
void setPhotoAlarm() {
  bluetooth.println("Photocell Sensor");
  // Prompt User for Threshold value
  bluetooth.println("Initiate alarm when bright or when dark?");
  bluetooth.println("Press 1 for alarm on dark");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int choice = bluetooth.read();    // Save User Decision

  // if the user chose bright (1), then when the light amount goes over 500 => alarm
  if (choice == 1) {
    bluetooth.println("Alarm When Dark");
    // set a certain global flag to true or 1 so that
    // a continuously checking function will catch alarm
    alarmOnDark = t.every(500, photosense);
  } else {
    bluetooth.println("Invalid input. Back to main menu.");
  }

}










// Set Delay for Periodic Accelerometer Sensor Reading Report
void setAccelDelay() {
  bluetooth.println("Accelerometer");

  // Prompt User for Sampling Rate or Delay
  bluetooth.println("Enter a value between 1-100   (Scale 1==10hz and 100==0.1hz ).");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  repeatAccelDelay = bluetooth.read();  // Save User Decision

  // Verify delay is equal or between 1-100
  if (repeatAccelDelay >= 1 && repeatAccelDelay <= 100) {
    unsigned long int ms = repeatAccelDelay * 100;
    repeatAccel = t.every(ms, displayAccelDetails);
  } else {
    bluetooth.println("You entered an invalid delay for Temperature. Try again.");
  }
}




// Set Delay for Periodic Magnetometer Sensor Reading Report
void setMagDelay() {
  bluetooth.println("Compass");

  // Prompt User for Sampling Rate or Delay
  bluetooth.println("Enter a value between 1-100   (Scale 1==10hz and 100==0.1hz ).");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  repeatMagDelay = bluetooth.read();    // Save User Decision

  // Verify delay is equal or between 1-100
  if (repeatMagDelay >= 1 && repeatMagDelay <= 100) {
    unsigned long int ms = repeatMagDelay * 100;
    repeatMag = t.every(ms, displayMagDetails);
  } else {
    bluetooth.println("You entered an invalid delay for Temperature. Try again.");
  }
}



// Set Delay for Periodic Temperature Sensor Reading Report
void setTempDelay() {
  bluetooth.println("Temperature");

  // Prompt User for Sampling Rate or Delay
  bluetooth.println("Enter a value between 1-100   (Scale 1==10hz and 100==0.1hz ).");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  repeatTempDelay = bluetooth.read();   // Save User Decision


  // Verify delay is equal or between 1-100
  if (repeatTempDelay >= 1 && repeatTempDelay <= 100) {
    unsigned long int ms = repeatTempDelay * 100;
    repeatTemp = t.every(ms, displayTempDetails);
  } else {
    bluetooth.println("You entered an invalid delay for Temperature. Try again.");
  }

}







// Change a pin of a sensor
void changePin() {
  //Start afresh
  turnOffPeriodicMode();

  // Prompt User for Decision
  bluetooth.println("Pick a sensor. 1=Accelerometer, 2=Temperature, or 3=Photocell.");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int decision = bluetooth.read();    // Save User Decision

  if      (decision == 1)   {
    setAccelMagPin();
  }
  else if (decision == 2)   {
    setTempPin();
  }
  else if (decision == 3)   {
    setPhotoPin();
  }
  else            {
    bluetooth.println("Invalid input. ");
  }

  bluetooth.println("End of Change Pin Method");
}




// Change Power Pin for Accelerometer-Magnetometer
void setAccelMagPin() {
  // Prompt User for Sampling Rate or Delay
  bluetooth.println("What should the new power pin for Accelerometer be? (Enter a non-zero integer)");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int newPin = bluetooth.read();      // Save User Decision

  // Verify newPin is non-negative
  if (newPin >= 0)  {
    acPower = newPin;
  }
  else        {
    bluetooth.println("You entered an invalid pin number");
  }
}




// Change Power or Analog Pin for Temperature Sensor
void setTempPin() {
  // Prompt User for Input
  bluetooth.println("For temperature sensor, change power pin or change analog pin? (Press 1 for Power Pin, 2 for Analog Pin)");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int pinVal = bluetooth.read();      // Save User Decision

  // Power Pin or Analog Pin
  if (pinVal == 1) {
    bluetooth.println("What should the new power pin be? (Enter a positive integer value)");
    while (bluetooth.available() == 0) {} // Wait for User to Input Data
    int newPin = bluetooth.read();      // Save User Decision

    // Verify newPin is positive
    if (newPin >= 0)  {
      tPower = newPin;
    }
    else        {
      bluetooth.println("You entered an invalid pin number");
    }
  }
  else if (pinVal == 2) {
    bluetooth.println("What should the new analog pin be? (Enter a positive integer value)");
    while (bluetooth.available() == 0) {} // Wait for User to Input Data
    int newPin = bluetooth.read();      // Save User Decision

    // Verify newPin is positive
    if (newPin >= 0)  {
      tAIn = newPin;
    }
    else        {
      bluetooth.println("You entered an invalid pin number");
    }
  } else {
    bluetooth.println("You entered an invalid pin number");
  }
}




// Change Power or Analog Pin for Photoresistor
void setPhotoPin() {
  // Prompt User for Input
  bluetooth.println("For photocell sensor, change power pin or change analog pin? (Press 1 for Power Pin, 2 for Analog Pin)");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int pinVal = bluetooth.read();      // Save User Decision

  // Power Pin or Analog Pin
  if (pinVal == 1) {
    bluetooth.println("What should the new power pin be? (Enter a positive integer value)");
    while (bluetooth.available() == 0) {} // Wait for User to Input Data
    int newPin = bluetooth.read();      // Save User Decision

    // Verify newPin is positive
    if (newPin >= 0)  {
      lPower = newPin;
    }
    else        {
      bluetooth.println("You entered an invalid pin number");
    }
  }
  else if (pinVal == 2) {
    bluetooth.println("What should the new analog pin be? (Enter a positive integer value)");
    while (bluetooth.available() == 0) {} // Wait for User to Input Data
    int newPin = bluetooth.read();      // Save User Decision

    // Verify newPin is positive
    if (newPin >= 0)  {
      lAIn = newPin;
    }
    else        {
      bluetooth.println("You entered an invalid pin number");
    }
  } else {
    bluetooth.println("You entered an invalid input");
  }
}




// Set Pin Mode and Status
void pinRequest() {
  bluetooth.println("Enter a pin number from 1 to 52: ");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int pinChoice = bluetooth.read();     // Save User Decision
  bluetooth.println("Press 1 for change mode (e.g. INPUT, OUTPUT), Press 2 for change status (e.g. LOW, HIGH)...");
  while (bluetooth.available() == 0) {} // Wait for User to Input Data
  int modeChoice = bluetooth.read();      // Save User Decision

  if (modeChoice == 1) {
    bluetooth.println("You want the pin is INPUT(1) or OUTPUT(0)? ");
    while (bluetooth.available() == 0) {} // Wait for User to Input Data
    int pinModeChoice = bluetooth.read();     // Save User Decision
    if (pinModeChoice == 1) {
      pinMode(pinChoice, INPUT);
    }
    if (pinModeChoice == 0) {
      pinMode(pinChoice, OUTPUT);
    }
  }
  if (modeChoice == 2) {
    bluetooth.println("Which status do you want for the pin? 1 for HIGH, 0 for LOW ");
    while (bluetooth.available() == 0) {} // Wait for User to Input Data
    int pinModeChoice = bluetooth.read();     // Save User Decision
    if (pinModeChoice == 1) {
      digitalWrite(pinChoice, HIGH);
    }
    if (pinModeChoice == 0) {
      digitalWrite(pinChoice, LOW);
    }
  }
}



// Display Magnetometer Readings
void displayMagDetails() {
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  bluetooth.println("-----------Magnetic Vector-----------");
  bluetooth.print(" X: "); bluetooth.print(event.magnetic.x); bluetooth.print(" ");
  bluetooth.print(" Y: "); bluetooth.print(event.magnetic.y); bluetooth.print(" ");
  bluetooth.print(" Z: "); bluetooth.print(event.magnetic.z); bluetooth.print(" "); bluetooth.println("uT");
  bluetooth.print("Report: ");
  bluetooth.println(rpNum);
  rpNum++;
}



// Display Accelerometer Readings
void displayAccelDetails() {
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  /* Display the results (acceleration is measured in m/s^2) */
  bluetooth.println("-------------Acceleration------------");
  bluetooth.print(" X: "); bluetooth.print(event.acceleration.x); bluetooth.print(" ");
  bluetooth.print(" Y: "); bluetooth.print(event.acceleration.y); bluetooth.print(" ");
  bluetooth.print(" Z: "); bluetooth.print(event.acceleration.z); bluetooth.print(" "); bluetooth.println("m/s^2");
  bluetooth.print("Report: ");
  bluetooth.println(rpNum);
  rpNum++;
}



// Display Temperature Sensor Readings
void displayTempDetails() {
  float referenceVoltage = 5.0;                       // set reference voltage to 5.0 (for Arduino Mega 2560 only)
  int temp = analogRead(tAIn);                     // read raw sensor data (note: units are not C or F yet)
  float volts = (temp * referenceVoltage) / 256.0;  // TMP36-RefVoltage formula = reading*refVoltage / 8-bit res (aka 256.0)
  float thermoC = (volts - 0.5) * 10.0;               // Formula for Celcius Conversion, adapted-fixed for volts instead of mV
  float thermoF = (thermoC * 9.0 / 5.0) + 32.0;       // Farenheit Conversion
  bluetooth.println("-------------Temperature-------------");
  bluetooth.print(" ");
  bluetooth.print(thermoF); bluetooth.println(" degree Fahrenheit"); bluetooth.print(" ");
  bluetooth.print(thermoC); bluetooth.println(" degree Celcius"); bluetooth.print(" ");
  bluetooth.print(thermoC + 273.15); bluetooth.println(" degree Kelvin"); bluetooth.print(" ");
  bluetooth.print((thermoC + 273.15) * 1.8); bluetooth.println(" degree Rankine");
  bluetooth.print("Report: ");
  bluetooth.println(rpNum);
  rpNum++;
}



// Display General Accelerometer Information
void displaySensorDetails(void) {
  turnOnAllSensors();
  turnOffPeriodicMode();
  turnOffAlarmMode();
  sensor_t sensor;
  accel.getSensor(&sensor);
  bluetooth.println("-------------------------------------");
  bluetooth.print  ("Sensor:       "); bluetooth.println(sensor.name);
  bluetooth.print  ("Driver Ver:   "); bluetooth.println(sensor.version);
  bluetooth.print  ("Unique ID:    "); bluetooth.println(sensor.sensor_id);
  bluetooth.print  ("Max Value:    "); bluetooth.print(sensor.max_value); bluetooth.println(" m/s^2");
  bluetooth.print  ("Min Value:    "); bluetooth.print(sensor.min_value); bluetooth.println(" m/s^2");
  bluetooth.print  ("Resolution:   "); bluetooth.print(sensor.resolution); bluetooth.println(" m/s^2");
  bluetooth.println("-------------------------------------");

}








