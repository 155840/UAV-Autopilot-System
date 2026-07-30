#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
//#include <ESP32Servo.h>
#include <mcp2515.h>
#include <math.h>

//Sensors 
Adafruit_MPU6050 imu; //IMU for pitch/roll
Adafruit_BMP280 bmp; //Altitude measurement
#define SDA_PIN 8
#define SCL_PIN 9

//Servos
//Servo elevatorServo;
//Servo aileronLeft;
//Servo aileronRight;
//int elevatorPin = 15;
//int aileronPin = 14;
//#define ELEVATOR_PIN 15
//#define AILERON_LEFT_PIN 14
//#define AILERON_RIGHT_PIN 8

//Switches
#define SWITCH_ON1 11 //Attitude mode switch input
#define SWITCH_ON2 12 //Altitude mode switch input

//Flight Modes
#define MODE_ASSISTED 0
#define MODE_ALTITUDE 1
#define MODE_ATTITUDE 2  

//LEDs
//#define LED_ASSISTED 18 //A0
//#define LED_ATTITUDE 17 //A1
//#define LED_ALTITUDE 16 //A2

int flightMode = MODE_ASSISTED;
int previousFlightMode = -1; //Will detect mode change

//CAN
struct can_frame canMsg;
struct MCP2515 mcp2515(10); //CS pin for MCP2515
#define CAN_CMD_ID 0x039 //Id for sending servo commands
#define CAN_ACK_ID 0x100   //ID for recieving ACKs (not needed here)

//Calibration
float pitchOffset = 0.0; //Will level the IMU
float rollOffset  = 0.0;
const int CALIB_SAMPLES = 100;

//Flight variables
float pitch = 0.0;
float roll = 0.0;
float altitude = 0.0;
float altitudeSetpoint = 0.0; //Will be used in altitude hold

float Kp_alt = 1.5; //Gain for altitude control
float Kp_att = 1.5; //Pitch/Roll gain for attitude control

//Functions
//IMU had to be calibrated to remove any offsets
void calibrateIMU(int samples = CALIB_SAMPLES) {
    float pitchSum = 0;
    float rollSum  = 0;
    Serial.println("Calibrating IMU, keep board flat...");
    for (int i = 0; i < samples; i++) {
        sensors_event_t accel, gyro, temp;
        imu.getEvent(&accel, &gyro, &temp);
        //Computes pitch and roll angles from accelerometer
        pitchSum += atan2(accel.acceleration.x, accel.acceleration.z) * 180.0 / M_PI;
        rollSum += atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / M_PI;
        delay(10);
    }
    pitchOffset = pitchSum / samples;
    rollOffset  = rollSum / samples;
    Serial.print("Pitch Offset: "); Serial.println(pitchOffset, 2);
    Serial.print("Roll Offset: ");  Serial.println(rollOffset, 2);
    Serial.println("Calibration complete!\n");
}

void readSensors() {
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);
    pitch = atan2(accel.acceleration.x, accel.acceleration.z) * 180.0 / M_PI - pitchOffset;
    roll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / M_PI - rollOffset;
    altitude = bmp.readAltitude(1013.25); //Standard sea level pressure
}

//Flight mode
void updateFlightMode() {
    bool altitudeState = digitalRead(SWITCH_ON2) == LOW; //Will be active when LOW
    bool attitudeState = digitalRead(SWITCH_ON1) == LOW; 
    
    if (altitudeState) flightMode = MODE_ALTITUDE;
    else if (attitudeState) flightMode = MODE_ATTITUDE; 
    else flightMode = MODE_ASSISTED;

    //Serial.print("Pin11: ");
    //Serial.print(digitalRead(SWITCH_ON1));
    //Serial.print("  Pin12: ");
    //Serial.println(digitalRead(SWITCH_ON2));
    Serial.print("FlightMode: ");
    Serial.println(flightMode); //This will print 0,1,2
}

void handleModeChange() {
     if (flightMode != previousFlightMode) {
        //Only set the setpoint the very first time ever (setup or first ALTITUDE)
        static bool altitudeSetpointInitialized = false;
        if (flightMode == MODE_ALTITUDE && !altitudeSetpointInitialized) {
            altitudeSetpoint = altitude; //Initialize once
            altitudeSetpointInitialized = true;
        }
        previousFlightMode = flightMode;

        Serial.print("Flight Mode changed to: ");
        if(flightMode == MODE_ASSISTED) Serial.println("ASSISTED");
        else if(flightMode == MODE_ALTITUDE) Serial.println("ALTITUDE HOLD");
        else if(flightMode == MODE_ATTITUDE) Serial.println("ATTITUDE HOLD");
    }
}

//LED control
/*void setupLEDs() {
    pinMode(LED_ASSISTED, OUTPUT);
    pinMode(LED_ATTITUDE, OUTPUT);
    pinMode(LED_ALTITUDE, OUTPUT);
    //Start with all LEDs off
    digitalWrite(LED_ASSISTED, LOW);
    digitalWrite(LED_ATTITUDE, LOW);
    digitalWrite(LED_ALTITUDE, LOW);
}

void updateLEDs() {
    //Turn only the active mode LED on
    digitalWrite(LED_ASSISTED, flightMode == MODE_ASSISTED ? HIGH : LOW);
    digitalWrite(LED_ATTITUDE, flightMode == MODE_ATTITUDE ? HIGH : LOW);
    digitalWrite(LED_ALTITUDE, flightMode == MODE_ALTITUDE ? HIGH : LOW);
}*/

//Map a value to servo command range (-120 to 120)
int mapControl(float val, float minVal, float maxVal) {
    val = constrain(val, minVal, maxVal);
    return (int)( (val - minVal) / (maxVal - minVal) * 240.0 - 120.0 );
}

//Send servo commands over CAN 
void sendCANCommand(int elevatorCmd, int aileronCmd) {
    int leftAileron = aileronCmd;
    int rightAileron = -aileronCmd; //differential mirror

    leftAileron  = constrain(leftAileron, -120, 120);
    rightAileron = constrain(rightAileron, -120, 120);
    elevatorCmd  = constrain(elevatorCmd, -120, 120);

    canMsg.can_id  = CAN_CMD_ID;
    canMsg.can_dlc = 6; //3 values, 2 bytes

    //Elevator
    canMsg.data[0] = (elevatorCmd >> 8) & 0xFF;
    canMsg.data[1] = elevatorCmd & 0xFF;
    //Left Aileron
    canMsg.data[2] = (leftAileron >> 8) & 0xFF;
    canMsg.data[3] = leftAileron & 0xFF;
    //Right Aileron
    canMsg.data[4] = (rightAileron >> 8) & 0xFF;
    canMsg.data[5] = rightAileron & 0xFF;

    mcp2515.sendMessage(&canMsg);
}

/*void sendServoCommand(int elevatorCmd, int aileronCmd){
    int elevatorAngle = map(elevatorCmd, -120, 120, 0, 180);
    int aileronAngle = map(aileronCmd, -120, 120, 0, 180);

    elevatorAngle = constrain(elevatorAngle, 0, 180);
    aileronAngle = constrain(aileronAngle, 0, 180);

    elevatorServo.write(elevatorAngle);
    aileronLeft.write(aileronAngle);
    aileronRight.write(180 - aileronAngle);
}*/
//Control functions
void assistedControl(int &elevatorCmd, int &aileronCmd)
{
    elevatorCmd = 0;
    aileronCmd  = 0;
}

void altitudeControl(int &elevatorCmd, int &aileronCmd)
{
    float altitudeError = altitudeSetpoint - altitude;

    //Convert altitude error to servo command
    //1 meter - 50 units
    elevatorCmd = (int)(altitudeError * 100); //Gain (scale factor 100 works better)
    elevatorCmd = constrain(elevatorCmd, -120, 120);

    //Roll control
    aileronCmd  = (int)(roll * Kp_att * 2); // adjust gain for visibility
    aileronCmd  = constrain(aileronCmd, -120, 120);
}

void attitudeControl(int &elevatorCmd, int &aileronCmd)
{
    //Scale pitch and roll to -120 120 using the same mapping as altitude/roll
    elevatorCmd = mapControl(Kp_att * pitch, -45, 45); //Pitch - elevator
    aileronCmd  = mapControl(Kp_att * roll,  -45, 45); //Roll - ailerons
}

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10);
    
    //Initializes the switches
    pinMode(SWITCH_ON1, INPUT_PULLUP);
    pinMode(SWITCH_ON2, INPUT_PULLUP);

    //Initializes the LEDs
    //setupLEDs();

    //Initializes the MPU6050
    if (!imu.begin(0x68)) {
        Serial.println("MPU6050 not detected!");
        while(1);
    }
    imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 Ready");

    //Initializes the BMP280
    if (!bmp.begin()) {
        Serial.println("BMP280 not detected!");
        while(1);
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                 Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280 Ready");

    //Attaching servos
    //elevatorServo.attach(ELEVATOR_PIN);
    //aileronLeft.attach(AILERON_LEFT_PIN);
    //aileronRight.attach(AILERON_RIGHT_PIN);

    //In neutral/level position 
    //elevatorServo.write(90);
    //aileronLeft.write(90);
    //aileronRight.write(90);

    //Initializes the CAN
    SPI.begin();
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    Serial.println("CAN Initialized");

    calibrateIMU();
}

void loop() {
    readSensors();
    updateFlightMode();
    handleModeChange();
    //updateLEDs();

    int elevatorCmd = 0;
    int aileronCmd = 0;

switch (flightMode) {
    //Call the control functions depending on flight mode
    case MODE_ASSISTED:
        assistedControl(elevatorCmd, aileronCmd);
        break;

    case MODE_ALTITUDE:
        altitudeControl(elevatorCmd, aileronCmd);
        break;

    case MODE_ATTITUDE:
        attitudeControl(elevatorCmd, aileronCmd);
        break;
    }

    //Send command to servo over CAN
    sendCANCommand(elevatorCmd, aileronCmd);
    //sendServoCommand(elevatorCmd, aileronCmd);

    
    if (flightMode == MODE_ALTITUDE) {
        Serial.print("Altitude: "); Serial.println(altitude, 2);
        Serial.print("Altitude Setpoint: "); Serial.println(altitudeSetpoint, 2);
        Serial.print("Altitude Error: "); Serial.println(altitudeSetpoint - altitude, 4);
        Serial.print("Elevator Cmd: "); Serial.println(elevatorCmd);
        Serial.print("Aileron Cmd: "); Serial.println(aileronCmd);
    } 
    else if (flightMode == MODE_ATTITUDE) {
        Serial.print("Pitch: "); Serial.println(pitch, 2);
        Serial.print("Roll: "); Serial.println(roll, 2);
        Serial.print("Pitch Error: "); Serial.println(0 - pitch, 4);  
        Serial.print("Roll Error: "); Serial.println(0 - roll, 4);    
        Serial.print("Elevator Cmd: "); Serial.println(elevatorCmd);
        Serial.print("Aileron Cmd: "); Serial.println(aileronCmd);
    } 
    else if (flightMode == MODE_ASSISTED) {
        Serial.println("ASSISTED MODE - No automatic control");

    delay(20); //50Hz loop
    }
}