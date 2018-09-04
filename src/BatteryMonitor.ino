#include <Thread.h>
#include <ThreadController.h>
#include <StaticThreadController.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_RGBLCDShield.h>
#include <Adafruit_MCP23017.h>
#include <avr/sleep.h>


#define ON 0x1
#define OFF 0x0

/*
    7.0 = 2.176
    7.5 = 2.352
    8.0 = 2.494
    8.5 = 2.660
    9.0 = 2.811
    9.5 = 2.964
    10.0 = 3.111
    10.5 = 3.271
    11.0 = 3.419
    11.5 = 3.581
    12.0 = 3.728
    12.5 = 3.874
    13.0 = 4.044
    13.5 = 4.193
    14.0 =  4.352
*/

// TODO:


// Current Sense constants
const int analogOutPin = 13; // Analog output pin that the LED is attached to
const int batteryVoltageAI = A0;  // Analog input pin that the Battery Voltage is attached to
const int chargeCurrentSolarAI = A9; // Solar Panels
const int chargeCurrentAcAI = A3;  // AC Battery Charger
const int chargeCurrentCarAI = A2; // Car Alternator
const int drawCurrentAI = A1;  // battery draw current (out)
const float mvPerAmp = 0.066;
const float vMaxAt5v = 15.88;
const float SolarChargeCurrentOffset = 0.05;
const float DrawCurrentOffset = 0.03;
const float CarChargeCurrentOffset = 0.24;
const float AcChargeCurrentOffset = 0.00;
const int BatteryVoltageSenseRelayPin = 8; 

// Sleep Constants
const int wakePin = 2;                 // pin used for waking up

typedef enum
{
  INFO,
  WARN,
  ERR
} LogLevel;

char logFileName [13] = "";


RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();


File dataFile;

// ==================================================================================
//THREAD CONTROLLER GLOBALS
// ==================================================================================
// ThreadController that will controll all threads
ThreadController threadController = ThreadController();
Thread* lcdDisplayThread = new Thread();
Thread* measurementThread = new Thread();

// prototypes
float CalculateCurrent(int adcChannel, float offsetCurrent = 0.0);
void TakeMeasurements();
void displayHandler();
void wakeUpNow();
void sleepNow();


void setup()
{
  Serial.begin(9600);
  Serial.println("Hello!");

  //--------------------------------------------
  // Set up sleep mode
  //--------------------------------------------
  // set up wake pin
  pinMode(wakePin, INPUT);
  digitalWrite(wakePin, HIGH);
  // wakeUpNow when pin 2 gets LOW
  attachInterrupt(0, wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function

  // Set up I/O
  pinMode(BatteryVoltageSenseRelayPin, OUTPUT);
  digitalWrite(BatteryVoltageSenseRelayPin, LOW);

  //--------------------------------------------
  // Set up LCD
  //--------------------------------------------
  lcd.begin(16, 2);
  lcd.setBacklight(ON);
  lcd.print("Battery Monitor");

  //--------------------------------------------
  // Set up SD Card
  //--------------------------------------------
  if (!SD.begin(10, 11, 12, 13))
  {
    Serial.println("Card failed, or not present");
  }

  //--------------------------------------------
  // Set up Real Time Clock
  //--------------------------------------------
  if (! rtc.begin())
  {
    Serial.println("Couldn't find RTC");
  }
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  DateTime now = rtc.now();
  char buffer [13] = "";

  // Set the name of the log file.
  if (!rtc.isrunning())
  {
    Serial.println("RTC is NOT running!");
    sprintf(logFileName, "NoRTC.txt");
  }
  else
  {
    sprintf(buffer, "%02d%02d%02d%02d.txt", now.month(), now.day(), now.hour(), now.minute());
    sprintf(logFileName, "%s", buffer);
  }

  // Open up the file we're going to log to!
  dataFile = SD.open(logFileName, FILE_WRITE);
  if (! dataFile)
  {
    Serial.println("Error Opening");
    Serial.println(logFileName);
    // Wait forever since we cant write data
    while (1) ;
  }
  String dataString = "Battery Monitor";
  dataFile.println(dataString);
  dataFile.flush(); // if needed .. will flush after 512kb is written

  //
  // Set up threads
  measurementThread->onRun(MeasurementRunner);
  measurementThread->setInterval(5000);

  lcdDisplayThread->onRun(displayHandler);
  lcdDisplayThread->setInterval(53);

    threadController.add(lcdDisplayThread); // & to pass the pointer to it
    threadController.add(measurementThread); // & to pass the pointer to it
}


void loop()
{
  threadController.run();
}





// ==================================================================================
// CALCULATION METHODS
// ==================================================================================
volatile bool measurementsRunning = false;
const int NUM_MEASUREMENTS = 10;
float measurements[NUM_MEASUREMENTS];

typedef enum {
  BattVoltage,
  DrawCurrent,
  SolarCurrent,
  AcCurrent,
  CarCurrent,
  TotalChargingCurrent, 
  CurrentBalance,
  Power, 
  Energy, 
  SolarPower
  
} Calculation;

String measurementText[NUM_MEASUREMENTS] = 
{
    "Battery Volts", 
    "Draw Current",
    "Solar Current", 
    "AC Current",
    "Car Current",
    "Total Charge",
    "Current Bal",
    "Power Draw(W)",
    "Energy Draw(J)",
    "Solar Power(w)"
};

float CalculateCurrent(int adcChannel, float offset = 0.0)
{
  int adcValue = analogRead(adcChannel);
  float current = (((float)map(adcValue, 0, 1023, -1650, 1650)) / mvPerAmp) / 1000.0;
  //Serial.println(String(adcChannel, 1) + " " + String(current, 3));
  if (abs(current + offset) <= 0.18) // ignore noise/inaccurate readings 
    return 0.0;
  return current + offset;
}


void MeasurementRunner()
{
    TakeMeasurements();
    LogMeasurements();
}

void TakeMeasurements() // every 5 seconds
{
    const float MeasurementPeriodInSeconds = 5.3; // thread period + time it takes to get through this method
  measurementsRunning = true;
  // read the analog in value:
  digitalWrite(BatteryVoltageSenseRelayPin, HIGH);
  delay(200);
  int batteryVoltageAIValue =  analogRead(batteryVoltageAI);
  delay(100);
  digitalWrite(BatteryVoltageSenseRelayPin, LOW);
  
  measurements[BattVoltage] = (((float)map(batteryVoltageAIValue, 0, 1023, 0, vMaxAt5v * 100.0) / 100));
  measurements[SolarCurrent] = -CalculateCurrent(chargeCurrentSolarAI, SolarChargeCurrentOffset);
  measurements[AcCurrent]  = -CalculateCurrent(chargeCurrentAcAI, AcChargeCurrentOffset);
  measurements[DrawCurrent] = CalculateCurrent(drawCurrentAI, DrawCurrentOffset);
  measurements[CarCurrent] = -CalculateCurrent(chargeCurrentCarAI, CarChargeCurrentOffset);
  measurements[TotalChargingCurrent] = measurements[SolarCurrent] + measurements[AcCurrent] + measurements[CarCurrent]; 
  measurements[CurrentBalance] = measurements[TotalChargingCurrent] + measurements[DrawCurrent];
  measurements[Power] = measurements[DrawCurrent] * measurements[BattVoltage];
  measurements[Energy] = measurements[Power] * MeasurementPeriodInSeconds; // if the polling time + delays change, change this. 
  measurements[SolarPower] = measurements[SolarCurrent] * measurements[BattVoltage];
 
  measurementsRunning = false;
}

void LogMeasurements()
{
  String line;
  for (int ii = 0; ii < NUM_MEASUREMENTS ; ii++)
  {
    line += measurementText[ii] + " " + String(measurements[ii], 2) + " \t| ";
  }
  
  // Log to sd
  Log(line, "Loop()", INFO);
  Serial.println(line);
 
}

// ==================================================================================
// DISPLAY METHODS
// ==================================================================================
void displayHandler() // every 50ms
{
  if (measurementsRunning)
    return;
    
 static uint16_t calc = BattVoltage; 
 static uint16_t lastCalc;
 static float lastMeasurement;
    

    uint8_t buttons = lcd.readButtons();
  if (buttons)
  {
    if (buttons & BUTTON_UP)
    { 
    }
    if (buttons & BUTTON_DOWN)
    {   
        
    }
    if (buttons & BUTTON_LEFT)
    {
        calc--;
    }
    if (buttons & BUTTON_RIGHT)
    {
     calc++; 
    }
    if (buttons & BUTTON_SELECT)
    {
      Serial.println("SELECT");
    }
  }
        if (calc >= NUM_MEASUREMENTS)
            calc = 0; 
        else if (calc < 0)
            calc = NUM_MEASUREMENTS-1;

  if (measurements[calc] != lastMeasurement || calc != lastCalc)
  {
    lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(measurementText[calc]);
        lcd.setCursor(0,1);
        lcd.print(String(measurements[calc],2));
        
        lastMeasurement = measurements[calc];
        lastCalc = calc;
  }

}

// ==================================================================================
// LOGGING METHODS
// ==================================================================================

LogLevel AA;
char logLevelText[3][10] = {"INFO", "WARN", "ERROR"};

void Log (String text, String callingMethod, LogLevel logLevel)
{
  DateTime now = rtc.now();
  
  String separator = " - ";
  String line = logLevelText[logLevel] + separator + now.year() + "-" + now.month() + "-" + now.day() + separator + now.hour() + ":" + now.minute() + ":" + now.second() + separator + text; /// change this to represent format belew.

  /// sprintf(line, "[%s] - %04d-%02d-%02d-%02d:%02d:%02d:%02d - '%s' - %s \r\n", logLevel, now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), callingMethod, text);
  dataFile.println(line);
  dataFile.flush(); // if needed .. will flush after 512kb is written

  // if file size of the filename of the last saved file (date in name) is larger than 1Mb then create a new file.
  //File.size()

}

// ==================================================================================
// SLEEP MODE METHODS
// ==================================================================================

//int sleepStatus = 0;             // variable to store a request for sleep
//int count = 0;                   // counter
const long awakeTimeInSeconds = 2000000;
void SleepModeLoop()
{
  static long count = 0;

  // check if it should go to sleep because of time
  if (count >= awakeTimeInSeconds) {
    Serial.println("Timer: Entering Sleep mode");
    delay(100);     // this delay is needed, the sleep
    //function will provoke a Serial error otherwise!!
    count = 0;
    sleepNow();     // sleep function called here
  }


  // display information about the counter
  Serial.print("Awake for ");
  Serial.print(count);
  Serial.println("sec");
  count++;

  digitalWrite(BatteryVoltageSenseRelayPin, digitalRead(BatteryVoltageSenseRelayPin) ^ 0x1);
}

// working - drop pin 2 low and wakes from sleep
void sleepNow()         // here we put the arduino to sleep
{
  digitalWrite(BatteryVoltageSenseRelayPin, HIGH); // open relay to stop current sense resistor drain

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
  sleep_enable();          // enables the sleep bit in the mcucr register
  // so sleep is possible. just a safety pin
  attachInterrupt(0, wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
  // wakeUpNow when pin 2 gets LOW
  lcd.setBacklight(0);

  sleep_mode();            // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  sleep_disable();         // first thing after waking from sleep:
  // disable sleep...
  detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
  // wakeUpNow code will not be executed
  // during normal running time.

}

void wakeUpNow()        // here the interrupt is handled after wakeup
{
  //  digitalWrite(BatteryVoltageSenseRelayPin, LOW); // when we wake, close the voltage sense relay
  //do something on wakeup
}




