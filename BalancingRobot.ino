#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>
#include <ArduinoOTA.h>
#include <PID_v1.h>

#include "secrets.h"

const bool debugMode = false;

Adafruit_MotorShield AFMS(0x60); // Default address, no jumpers

Adafruit_StepperMotor *stepper1 = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *stepper2 = AFMS.getStepper(200, 2);

MPU6050 mpu;
uint8_t mode = 0;

const uint8_t MPU_INTERRUPT_PIN = 7;
const uint8_t LED_PIN = 13;
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

long interval = 20;        // time constant for timers
unsigned long count;

double var1;
double var2;

volatile bool imuDataReady = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    imuDataReady = true;
}

// Gyro PID

double Kp1 = 10.0;  
double Ki1 = 0.0;
double Kd1 = 0.0;

double Setpoint1 = 0;
double Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);    // PID Setup

/*
uint32_t nextGyroUpdate = 0;
const uint16_t gyroUpdateInterval = 100;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t gyroX, gyroY, gyroXRate, gyroYRate;
float gyroXAngle, gyroYAngle;

float accelX, accelY;
float currentXAngle, currentYAngle;
float prevXAngle, prevYAngle;
*/

// wrappers for the first motor!
void forwardstep1()
{
  stepper1->onestep(BACKWARD, DOUBLE);
}
void backwardstep1()
{
  stepper1->onestep(FORWARD, DOUBLE);
}

// wrappers for the second motor!
void forwardstep2()
{
  stepper2->onestep(FORWARD, DOUBLE);
}
void backwardstep2()
{
  stepper2->onestep(BACKWARD, DOUBLE);
}

AccelStepper aStepper1(forwardstep1, backwardstep1);
AccelStepper aStepper2(forwardstep2, backwardstep2);

void setMotorSpeed(double speed)
{
    aStepper1.setSpeed(speed);
    aStepper2.setSpeed(speed);
}

const uint8_t I2CAFMS_SDA = 33; //8;
const uint8_t I2CAFMS_SCL = 38; //9;
TwoWire I2CAFMS = TwoWire(1);

unsigned long wifiReconnectPreviousMillis = 0;
unsigned long wifiReconnectInterval = 30000;
WiFiServer telnetServer(23);
WiFiClient telnetClient; // one telnetClient should be able to telnet to this ESP32

char inputBuffer[20];
uint8_t inputBufferPosition = 0;

uint32_t currentMillis;
uint32_t nextIMUCheck;

uint32_t getMillis()
{
    return esp_timer_get_time() / 1000;
}

void connectToNetwork()
{
    WiFi.begin(ssid, password);
 
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
    }

    //if (WiFi.status() == WL_CONNECTED)
    //    infoLog->println("Connected to WiFi");
}

void readAngles()  {

    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));
     } 
     
     else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        imuDataReady = 0;

        count = count + 1;
    }
}

void setup()
{
    Serial.begin(9600);
    if (debugMode)
        while (!Serial);
    Serial.println("Robot test!");

    pinMode(LED_PIN, OUTPUT);
    pinMode(MPU_INTERRUPT_PIN, INPUT);


    connectToNetwork();
    if (WiFi.status() == WL_CONNECTED)
    {
        telnetServer.begin();
        telnetServer.setNoDelay(true);
        Serial.println("WiFi done!");
    }

    // ArduinoOTA.begin(WiFi.localIP(), "Arduino", "password", InternalStorage);
    ArduinoOTA.setHostname(deviceName);
    ArduinoOTA.begin();

    I2CAFMS.begin(I2CAFMS_SDA, I2CAFMS_SCL, (uint32_t) 400000);

    if (AFMS.begin(4000, &I2CAFMS)) // Start the shield
    {
        Serial.println("AFMS init success!");
    }
    else
    {
        Serial.println("AFMS init failed!");
    }

    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    //           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
    //OFFSETS     -331,    -519,    1310,      62,      45,     -33
    mpu.setXAccelOffset(-331);
    mpu.setYAccelOffset(-519);
    mpu.setZAccelOffset(1310);
    mpu.setXGyroOffset (62);
    mpu.setYGyroOffset (45);
    mpu.setZGyroOffset (-33);
    
    Serial.println("Gyro init!");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(MPU_INTERRUPT_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    PID1.SetMode(AUTOMATIC);              
    PID1.SetOutputLimits(-1000, 1000);
    PID1.SetSampleTime(10);

    aStepper1.setMaxSpeed(1000.0); // Only used by stepper.run() and not used by stepper.runSpeed()
    //aStepper1.setSpeed(600.0); // 1000 is too high
    //stepper1.setAcceleration(255.0);
    //stepper1.moveTo(1000);

    aStepper2.setMaxSpeed(1000.0);
    //aStepper2.setSpeed(600.0);
    //stepper2.setAcceleration(255.0);
    //stepper2.moveTo(1000);
    aStepper1.setSpeed(0);
    aStepper2.setSpeed(0);

    Serial.println("Setup done!");
}

void loop()
{
    currentMillis = getMillis();

    ArduinoOTA.handle();

    // if WiFi is down, try reconnecting
    if (WiFi.status() == WL_CONNECTED)
    {
        if (telnetServer.hasClient()) //check if there are any new telnetClients
        {
            telnetClient = telnetServer.available();
            //if (!telnetClient)
            //    Serial.println("available broken");
            //Serial.print("New telnetClient: ");
            //Serial.println(telnetClient.remoteIP());
            telnetClient.write("Welcome!!!\n");
        }

        if (telnetClient && telnetClient.connected()) //check telnetClients for data
        {
            while (telnetClient.available())
            {
                char buffer = telnetClient.read();
                
                if (buffer == '\n' && inputBufferPosition > 0)
                {
                    if (inputBuffer[0] == 'n')
                    {
                        mode++;
                        if (mode > 2)
                            mode = 0;

                        if (mode == 0)
                            telnetClient.write("Changing to Kp\r\n");
                        else if (mode == 1)
                            telnetClient.write("Changing to Ki\r\n");
                        else if (mode == 1)
                            telnetClient.write("Changing to SetPoint\r\n");
                    }
                    else
                    {
                        inputBuffer[inputBufferPosition] = '\0';
                        double value = atof(inputBuffer);
                        char outputBuffer[100];

                        if (mode == 0)
                        {
                            Kp1 = value;
                            PID1.SetTunings(Kp1, Ki1, Kd1);
                            snprintf(outputBuffer, 100, "Setting Kp to %.2f\r\n", Kp1);
                        }
                        else if (mode == 1)
                        {
                            Ki1 = value;
                            PID1.SetTunings(Kp1, Ki1, Kd1);
                            snprintf(outputBuffer, 100, "Setting Ki to %.2f\r\n", Ki1);
                        }
                        else if (mode == 2)
                        {
                            Setpoint1 = value;
                            snprintf(outputBuffer, 100, "Setting SetPoint to %.2f\r\n", Setpoint1);
                        }

                        
                        size_t bufferSize = strlen(outputBuffer);
                        telnetClient.write(outputBuffer, bufferSize);
                    }
                    inputBufferPosition = 0;
                }
                else
                {
                    if ((inputBufferPosition == 0 && (buffer == 'n' || buffer == '-' || buffer == '.')) || (buffer >= '0' && buffer <= '9'))
                    {
                        inputBuffer[inputBufferPosition] = buffer;
                        inputBufferPosition++;
                    }
                }
            }
        }
        else if (telnetClient)
        {
            telnetClient.stop();
        }
    }
    else if ((WiFi.status() != WL_CONNECTED) && (currentMillis - wifiReconnectPreviousMillis >= wifiReconnectInterval))
    {
        Serial.print(currentMillis);
        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();

        //if (WiFi.status() == WL_CONNECTED)
        //    infoLog->println("Reconnected to WiFi");

        wifiReconnectPreviousMillis = currentMillis;
    }

/*
    if (currentMillis > nextGyroUpdate)
    {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        accelY = (atan2(ay, az) * 180 / PI);
        accelX = (atan2(ax, az) * 180 / PI);
        gyroXRate = map(gx, -32768, 32767, -250, 250);
        gyroYRate = map(gy, -32768, 32767, -250, 250);
        gyroXAngle = gyroXAngle + (float)gyroXRate*gyroUpdateInterval/1000;
        gyroYAngle = gyroYAngle + (float)gyroYRate*gyroUpdateInterval/1000;

        currentXAngle = 0.9934 * (prevXAngle + gyroXAngle) + 0.0066 * (accelX);
        currentYAngle = 0.9934 * (prevYAngle + gyroYAngle) + 0.0066 * (accelY);
        prevXAngle = currentXAngle;
        prevYAngle = currentYAngle;

        if (debugMode)
        {
            //Serial.printf("%d,%d,%d,%d,%d,%d\r\n", ax, ay, az, gx, gy, gz);
            //Serial.printf("%.2f,%.2f,%.2f,%.2f\r\n", accelY, accelX, gyroYAngle, gyroXAngle);
            Serial.printf("%.2f,%.2f\r\n", prevXAngle, prevYAngle);
        }
        nextGyroUpdate = currentMillis + gyroUpdateInterval;
    }
*/

    if (currentMillis > nextIMUCheck)
    {
        if (imuDataReady == 1)
        {
            readAngles();
        

            var1 = ypr[1] * 180/M_PI;
            var2 = ypr[2] * 180/M_PI;
            
            // Serial.print(var1);
            // Serial.print(',');
            // Serial.println(var2);

            //Setpoint1 = 0;    // fine tune cetre point
            Input1 = var2;       // use IMU over serial as input
            PID1.Compute();     // compute PID output    
            //Output1 = constrain(Output1, -500, 500);
            if (var2 <= -45 || var2 >= 45)
                setMotorSpeed(0);
            else    
                setMotorSpeed(Output1);
            Serial.printf("%.2f,%.2f\r\n", Input1, Output1);

        
        }
        nextIMUCheck = currentMillis + 10;
    }

    aStepper1.runSpeed();
    aStepper2.runSpeed();
}