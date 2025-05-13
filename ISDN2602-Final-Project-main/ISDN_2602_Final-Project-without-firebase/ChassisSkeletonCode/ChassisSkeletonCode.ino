#include "Pinout.hpp"
#include "esp32-hal-ledc.h"


// Include Arduino FreeRTOS library
// #include "Arduino_FreeRTOS.h"

/*Include the FreeRTOS library*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

/*Pre-Set Data*/
volatile float LeftSpeed = 220.0f; // 0.0f - 1024.0f
volatile float RightSpeed = 220.0f; // 0.0f - 1024.0f

/*-------------------------------------------------------------------------------------------------------------------------*/
/*-------------RFID Reader Task-------------*/
/*--------Stack and Handle Settings---------*/

StackType_t uxRFIDTagReader[configMINIMAL_STACK_SIZE];
StaticTask_t xRFIDTagReaderTCB;
TaskHandle_t RFIDTagReaderTCB;

/*Creating the Class for RFID Reader*/
MFRC522 mfrc522(0x28, RFID_RST);

/*Function for getting the RFID Tag ID Number*/
String getTagUID(){
  String tagUID = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    tagUID += mfrc522.uid.uidByte[i] < 0x10 ? "0" : "";
    tagUID += String(mfrc522.uid.uidByte[i], HEX);
  }
  return tagUID;
};

/*Define a struct to store the RFID tag*/
struct RFIDTag {
  char uid[9];
};
RFIDTag tag;

/*Define a Storage to store the previous tagUID*/
String prevRFIDTag; 

/*User Task for RFID Tag Reader*/
void RFIDTagReader(void *pvPara)
{
    /*Setup for the task*/

    /*Do*/
    while(true){
    
    /*DONT CHANGE THE FOLLOWING 2 LINE OF CODE*/
    if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) 
    vTaskDelay(50);

    /*You may want to setup a check new UID to save procress power
      Comparing the PrevUID and CurrentUID...*/
  
    String currenttagUID = getTagUID();
    currenttagUID.toCharArray(tag.uid, sizeof(tag.uid));

    // if(currenttagUID != prevRFIDTag){
    Serial.print("RFID Tag: ");
    Serial.println(tag.uid);
    // }


    /*----------------------------------------------------*/
    // FOR DEBUG USAGE
    // Serial.print("RFID Tag: ");
    // Serial.println(tag.uid);
    /*----------------------------------------------------*/
    
    /*A delay must be added inside each User Task*/

    vTaskDelay(10);
      
    }
}

/*-------------------------------------------------------------------------------------------------------------------------*/
/*------------Line Tracking Task------------*/
/*--------Stack and Handle Settings---------*/

StackType_t uxLineTrackingTaskStack[configMINIMAL_STACK_SIZE];
StaticTask_t xLineTrackingTaskTCB;
TaskHandle_t LineTrackingTaskTCB;


/*User Task for Line Tracking*/
void LineTrackingTask(void *pvPara)
{
  /*Setup for the task*/

  /*DO*/
  while(true){

    LineTracking::FollowingLine(IR::Tracking(), LeftSpeed, RightSpeed);

    /*A delay must be added inside each User Task*/    
    vTaskDelay(10);
  }
}



/*-------------------------------------------------------------------------------------------------------------------------*/
/*------------FireBase Task------------*/
/*--------Stack and Handle Settings---------*/
StackType_t uxFireBaseTaskStack[configMINIMAL_STACK_SIZE];
StaticTask_t xFireBaseTaskTCB;
TaskHandle_t FireBaseTaskTCB;


void FireBaseTask(void *pvPara){
  /*Setup for the task*/

  while(true){


  /*A delay must be added inside each User Task*/ 
  vTaskDelay(10);
  }
}



/*DO NOT Chnage the code below*/
/*-------------------------------------------------------------------------------------------------------------------------*/
/*PID Control for Motor
  PID Function and Settings
  Creating PID_t for Mult. PID Setting 
  PID 1 for LeftWheel
  PID 2 for RightWheel */
struct PID_t
{
    /*Creating the parameters for PID*/
    volatile float Kp ;
    volatile float Ki ;
    volatile float Kd ;

    volatile float target_val;  // The target RPM
    float actual_val;           // Actual RPM Reading
    float err;                  // Error 
    float err_last;
    float integral;

    /*General PID Function*/
    float PID_realize(float temp_val)
    {
        this->err = this-> target_val - temp_val;

        this->integral += this->err;

        this->actual_val = this->Kp * this->err + this->Ki * this->integral + this->Kd * (this->err - this->err_last);

        this->err_last = this->err;

        return this->actual_val;
    }

}PID;

/*-------------------------------------------------------------------------------------------------------------------------*/
/*-------------RPM Measure Task-------------*/
/*--------Stack and Handle Settings---------*/
StackType_t uxcalculateRPMTaskStack[configMINIMAL_STACK_SIZE];
StaticTask_t xcalculateRPMTaskTCB;
TaskHandle_t calculateRPMTaskTCB;

/*Constants for Encoder
  Find out the encoder resolution by yourself */
const int encoderResolution =320; // Number of pulses per revolution
const unsigned long interval = 1000; // Time interval in milliseconds 1000ms

/*Encoder to RPM Function and Settings
  Creating RPMCounter_t for 2 Wheel Setting 
  */
typedef struct RPMCounter_t{

volatile int encoderPulses;
unsigned long previousMillis;
volatile float rpm;


  void RPMCounter(){
  unsigned long currentMillis = millis();

  // Check if the time interval has elapsed
  if (currentMillis - previousMillis >= interval) {
    // Calculate RPM
    float rotations = float(encoderPulses) / ((float) encoderResolution);
    float time = (currentMillis - previousMillis) / 1000.0f; // Convert to seconds
    float rpm = (rotations / time) * 60.0f;

    // Reset encoder pulse count and update previousMillis
    encoderPulses = 0;
    previousMillis = currentMillis;

    // Print RPM
    Serial.println(rpm);
    vTaskDelay(1000/ portTICK_PERIOD_MS);  // Delay for 1 second
  }

}
} RPM; 

/*Define 2 Sets of Variables using RPMCounter_t for 2 Wheel 
  Init the RPM related Variables before the task starts   */
  RPMCounter_t LeftMotor  = {0, 0, 0};
  RPMCounter_t RightMotor = {0, 0, 0};


// Interrupt Service Routine
// Create a struct to handle 2 motors encoder
struct Encoder_t{
  int pinAState;
  int pinBState;
  int Encoder_A;
  int Encoder_B;

}Encoder;

/*Init the Enocoder related Variables before the task starts*/
  Encoder_t EncoderLeft  = {0, 0, Motor_L_Encoder_A, Motor_L_Encoder_B};
  Encoder_t EncoderRight = {0, 0, Motor_R_Encoder_A, Motor_R_Encoder_B};

/*-------------------------------------------------------------------------------------------------------------------------*/
/*Interrupt Service Routine Function
  Since attachInterrupt() cannot using non Static function 
  Below are 2 IRAM_ATTR function for handle the interrupts for the encoder*/
void IRAM_ATTR handleLeftEncoderInterrupt() {
  //init the local variable
  int change = 0;

  // Read the current state of the encoder pins
  EncoderLeft.pinAState = digitalRead(EncoderLeft.Encoder_A);
  EncoderLeft.pinBState = digitalRead(EncoderLeft.Encoder_B);

  // Determine the direction of rotation based on the phase change
  if (EncoderLeft.pinAState != EncoderLeft.pinBState) {
    change = (EncoderLeft.pinAState == HIGH) ? 1 : 0;
  } else {
    change = (EncoderLeft.pinAState == HIGH) ? 0 : 1;
  }

  // Update the encoder count
  LeftMotor.encoderPulses += change;
}

void IRAM_ATTR handleRightEncoderInterrupt() {
  //init the local variable
  int change = 0;

  // Read the current state of the encoder pins
  EncoderRight.pinAState = digitalRead(EncoderRight.Encoder_A);
  EncoderRight.pinBState = digitalRead(EncoderRight.Encoder_B);

  // Determine the direction of rotation based on the phase change
  if (EncoderRight.pinAState != EncoderRight.pinBState) {
    change = (EncoderRight.pinAState == HIGH) ? 1 : 0;
  } else {
    change = (EncoderRight.pinAState == HIGH) ? 0 : 1;
  }

  // Update the encoder count
  RightMotor.encoderPulses += change;
}


/*-------------------------------------------------------------------------------------------------------------------------*/
/*-------------RPM Measure User Task-------------*/

void calculateRPMTask(void *pvPara) {
  /*Setup for the Task*/
  /*----------------------------------------------------*/
  /*Define 2 sets PID for 2 Motors*/
  RPMCounter_t TargetRPM;
  /*Change the PID Para. here
    LeftMotor PID*/
    PID_t pid1 = {10.0f   //Kp
                , 0.3f   //Ki
                , 0.0f}; //Kd

  /*RightMotor PID*/
    PID_t pid2 = {8.0f   //Kp
                , 0.5f   //Ki
                , 0.0f}; //Kd

  /*Set the initial Target RPM Here*/
  pid1.target_val = 150.0f;
  pid2.target_val = 150.0f;
/*----------------------------------------------------*/
  while (true) {
/*----------------------------------------------------*/
    /*FOR DEBUG USAGE*/
    // Serial.print("RPM Left: ");
    // LeftMotor.RPMCounter(); 

    // Serial.print("RPM Right: ");
    // RightMotor.RPMCounter();
/*----------------------------------------------------*/
    /*Setting the actual value to PID function*/
    pid1.actual_val = LeftMotor.rpm;
    pid2.actual_val = RightMotor.rpm;

    /*Compute the PID and Write the Result to Speed of the Wheel*/
    LeftSpeed  = Motor::RPMtoPWM(pid1.PID_realize(LeftMotor.rpm), LeftWheel);
    RightSpeed = Motor::RPMtoPWM(pid1.PID_realize(RightMotor.rpm), RightWheel);

/*----------------------------------------------------*/
    /*FOR DEBUG USAGE*/
    Serial.print("Speed Left: ");
    Serial.println(LeftSpeed);

    Serial.print("Speed Right: ");
    Serial.println(RightSpeed);
/*----------------------------------------------------*/
/*A delay must be added inside each User Task*/ 
    vTaskDelay(10);
  }
}




/*DO NOT CHANGE THE CODE BELOW*/
/*----------------------------------------------------*/
/*------------LED Blinking Task-------------
  --------Stack and Handle Settings---------
To ensure there is visualization that the program is running*/
StackType_t uxBlinkTaskStack[configMINIMAL_STACK_SIZE];
StaticTask_t xBlinkTaskTCB;
TaskHandle_t BlinkTaskTCB;



void Blink(void *pvPara)
{
  /*Setup for the task*/
  pinMode(LED1, OUTPUT);
  /*DO*/
  while(true){
  digitalWrite(LED1,HIGH);
  vTaskDelay(100);
  digitalWrite(LED1, LOW);
  vTaskDelay(200);
  }
}

/*----------------------------------------------------*/



void setup() {
// put your setup code here, to run once:
Serial.begin(115200);
Serial.println("---------Initializing...---------");
//Set up PWM Channel for Motor
Motor::Init();
Serial.println("Wheel Motors Initialized");

//Set up PWM Channel for Servo Motor
Servo::Init();
Serial.println("Servo Motor Initialized");

//Initialize IR Sensor
IR::Init();
Serial.println("IR Sensor Initialized");

//Initialize RFID Reader 
Wire.begin(RFID_SDA, RFID_SCL);
mfrc522.PCD_Init();

Serial.println("RFID Initialized");
//Initialize IMU

//Initialize the FireBase Connection


// Serial.println("FireBase Initialized");


// Init the PinMode for the Encoder Pins 
pinMode(Motor_L_Encoder_A, INPUT_PULLUP);
pinMode(Motor_L_Encoder_B, INPUT_PULLUP);

pinMode(Motor_R_Encoder_A, INPUT_PULLUP);
pinMode(Motor_R_Encoder_B, INPUT_PULLUP);

// Attach the interrupt service routine to the encoder pins
attachInterrupt(digitalPinToInterrupt(Motor_L_Encoder_A), handleLeftEncoderInterrupt, CHANGE);
attachInterrupt(digitalPinToInterrupt(Motor_R_Encoder_A), handleRightEncoderInterrupt, CHANGE);
Serial.println("Interrupt Pins Initialized");




Serial.println("---------Initialized---------");




/*FreeRTOS Task Pinned to core*/
/*Do not change the config of the core
  Events Run on Core:   Core 0 (For FreeRTOS)
  Arduino Runs on Core: Core 1 (As Default)

  Run Core Tasks Config:
  Core 0: local  task  (Control)
  Core 1: online task (Firebase)*/
/*xTaskCreatePinnedtoCore: pin the specific task to desired core (esp32 is a dual cores MCU)
  xTaskCreatePinnedToCore(  void((void *pvPara)), Text for the task, Stack (Min. is 1024), const para. , &TaskTCB, uxPriority, Core )*/
xTaskCreatePinnedToCore(FireBaseTask, "FireBase", 10000, NULL, 3 , &FireBaseTaskTCB, 0 );
xTaskCreatePinnedToCore(Blink, "Blink", 2048, NULL, 1 , &BlinkTaskTCB, 1 );
xTaskCreatePinnedToCore(RFIDTagReader, "RFIDReader", 2048, NULL, 2 , &RFIDTagReaderTCB, 1 );
xTaskCreatePinnedToCore(LineTrackingTask, "LineTracking", 10000, NULL, 2 , &LineTrackingTaskTCB, 1 );
xTaskCreatePinnedToCore(calculateRPMTask, "calculateRPM", 10000, NULL, 3 , &calculateRPMTaskTCB, 1 );

/*Adding a small delay for the setup()*/

vTaskDelay(10);
}


/*Nothing will run in loop()
  Please do not write any code inside the loop()*/
void loop() {

}



