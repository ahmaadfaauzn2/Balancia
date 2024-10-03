// #include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include  <Fuzzy.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


float error = 0;
float error_sebelum = 0;
float derror = 0;
float pitch = 0;


// //PID
// double originalSetpoint = 175.8;
// double setpoint = originalSetpoint;
// double movingAngleOffset = 0.1;
// double input, output;
// int moveState=0; //0 = balance; 1 = back; 2 = forth
// double Kp = 50;
// double Kd = 1.4;
// double Ki = 60;
// PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.5;
//MOTOR CONTROLLER
int ENA = 3;
int IN1 = 4;
int IN2 = 5;
int IN3 = 6;
int IN4 = 7;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

// Initialize the Fuzzy Logic Controller
Fuzzy *fuzzy = new Fuzzy();

//timers
long time1Hz = 0;
long time5Hz = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}


void setup()
{

    // Define Fuzzy sets for input "error"
    FuzzySet *neg = new FuzzySet(-30, -30, -20, 0);
    FuzzySet *zero = new FuzzySet(-15, 0, 0, 15);
    FuzzySet *pos = new FuzzySet(0, 20, 30, 30);

    // Define Fuzzy sets for input "derror"
    FuzzySet *dneg = new FuzzySet(-30, -30, -20, 0);
    FuzzySet *dzero = new FuzzySet(-15, 0, 0, 15);
    FuzzySet *dpos = new FuzzySet(0, 20, 30, 30);


    // Define Fuzzy sets for output "pwm"
    FuzzySet *apelan = new FuzzySet(90, 90, 90, 90);
    FuzzySet *pelan = new FuzzySet(90, 90, 90, 90);
    FuzzySet *sedang = new FuzzySet(120, 120, 120, 120);
    FuzzySet *asedang = new FuzzySet(120, 120, 120, 120);
    FuzzySet *cepat = new FuzzySet(150, 150, 150, 150);

    
    //FuzzyInput1-error
  FuzzyInput *error = new FuzzyInput(1);
  error->addFuzzySet(neg);
  error->addFuzzySet(zero);
  error->addFuzzySet(pos);
  fuzzy->addFuzzyInput(error);

  //FuzzyInput2-derror
  FuzzyInput *derror = new FuzzyInput(2);
  derror->addFuzzySet(dneg);
  derror->addFuzzySet(dzero);
  derror->addFuzzySet(dpos);
  fuzzy->addFuzzyInput(derror);

  //FuzzyOutput-pwm
  FuzzyOutput *pwm = new FuzzyOutput(1);
  pwm->addFuzzySet(apelan);
  pwm->addFuzzySet(pelan);
  pwm->addFuzzySet(sedang);
  pwm->addFuzzySet(asedang);
  pwm->addFuzzySet(cepat);
  fuzzy->addFuzzyOutput(pwm);


   //Build Rule Base
  // 1 -- pwm = pelan IF er = neg && der = dzero OR er = zero && der = dneg (2 RULE BASE)
  FuzzyRuleAntecedent *errorNegAndderrorDzero = new FuzzyRuleAntecedent();
  errorNegAndderrorDzero->joinWithAND(neg, dzero);
  FuzzyRuleAntecedent *errorZeroAndderrorDneg = new FuzzyRuleAntecedent();
  errorZeroAndderrorDneg->joinWithAND(zero, dneg);

  FuzzyRuleAntecedent *iferrorNegAndderrorDzeroOrerrorZeroAndderrorDneg = new FuzzyRuleAntecedent();
  iferrorNegAndderrorDzeroOrerrorZeroAndderrorDneg->joinWithOR(errorNegAndderrorDzero, errorZeroAndderrorDneg);
  FuzzyRuleConsequent *thenpwmPelan = new FuzzyRuleConsequent();
  thenpwmPelan->addOutput(pelan);

  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, iferrorNegAndderrorDzeroOrerrorZeroAndderrorDneg, thenpwmPelan);
  fuzzy->addFuzzyRule(fuzzyRule1);

  // 2 -- pwm = apelan IF er = zero && der = dzero (1 RULE BASE)
  FuzzyRuleAntecedent *iferrorZeroAndderrorDzero = new FuzzyRuleAntecedent();
  iferrorZeroAndderrorDzero->joinWithAND(zero, dzero);
  FuzzyRuleConsequent *thenpwmApelan = new FuzzyRuleConsequent();
  thenpwmApelan->addOutput(apelan);

  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, iferrorZeroAndderrorDzero, thenpwmApelan);
  fuzzy->addFuzzyRule(fuzzyRule2);

  // 3 -- pwm = sedang IF er = neg && der = dpos OR er = zero && der = dpos  (2 RULE BASE)
  FuzzyRuleAntecedent *errorNegAndderrorDpos = new FuzzyRuleAntecedent();
  errorNegAndderrorDpos->joinWithAND(neg, dpos);
  FuzzyRuleAntecedent *errorZeroAndderrorDpos = new FuzzyRuleAntecedent();
  errorZeroAndderrorDpos->joinWithAND(zero, dpos);

  FuzzyRuleAntecedent *iferrorNegAndderrorDposOrerrorZeroAndderrorDpos = new FuzzyRuleAntecedent();
  iferrorNegAndderrorDposOrerrorZeroAndderrorDpos->joinWithOR(errorNegAndderrorDpos, errorZeroAndderrorDpos);
  FuzzyRuleConsequent *thenpwmSedang = new FuzzyRuleConsequent();
  thenpwmSedang->addOutput(sedang);

  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, iferrorNegAndderrorDposOrerrorZeroAndderrorDpos, thenpwmSedang);
  fuzzy->addFuzzyRule(fuzzyRule3);

  // 4 -- pwm = asedang IF er = pos && der = dneg OR er = pos && der = dzero (2 RULE BASE)
  FuzzyRuleAntecedent *errorPosAndderrorDneg = new FuzzyRuleAntecedent();
  errorPosAndderrorDneg->joinWithAND(pos, dneg);
  FuzzyRuleAntecedent *errorPosAndderrorDzero = new FuzzyRuleAntecedent();
  errorPosAndderrorDzero->joinWithAND(pos, dzero);

  FuzzyRuleAntecedent *iferrorPosAndderrorDnegOrerrorPosAndderrorDzero = new FuzzyRuleAntecedent();
  iferrorPosAndderrorDnegOrerrorPosAndderrorDzero->joinWithOR(errorPosAndderrorDneg, errorPosAndderrorDzero);
  FuzzyRuleConsequent *thenpwmAsedang = new FuzzyRuleConsequent();
  thenpwmAsedang->addOutput(asedang);

  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, iferrorPosAndderrorDnegOrerrorPosAndderrorDzero, thenpwmAsedang);
  fuzzy->addFuzzyRule(fuzzyRule4);

  // 5 -- pwm = cepat IF er = neg && der = dneg OR er = pos && der = dpos (2 RULE BASE)
  FuzzyRuleAntecedent *errorNegAndderrorDneg = new FuzzyRuleAntecedent();
  errorNegAndderrorDneg->joinWithAND(neg, dneg);
  FuzzyRuleAntecedent *errorPosAndderrorDpos = new FuzzyRuleAntecedent();
  errorPosAndderrorDpos->joinWithAND(pos, dpos);

  FuzzyRuleAntecedent *iferrorNegAndderrorDnegOrerrorPosAndderrorDpos = new FuzzyRuleAntecedent();
  iferrorNegAndderrorDnegOrerrorPosAndderrorDpos->joinWithOR(errorNegAndderrorDneg, errorPosAndderrorDpos);
  FuzzyRuleConsequent *thenpwmCepat = new FuzzyRuleConsequent();
  thenpwmCepat->addOutput(cepat);

  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, iferrorNegAndderrorDnegOrerrorPosAndderrorDpos, thenpwmCepat);
  fuzzy->addFuzzyRule(fuzzyRule5);


    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        // //setup PID
        
        // pid.SetMode(AUTOMATIC);
        // pid.SetSampleTime(10);
        // pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}


void loop() {
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // Idle loop waiting for MPU data
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // Process MPU6050 data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        pitch = ypr[1] * 180/M_PI;  // Calculate pitch angle in degrees

        // Compute fuzzy logic output
        double fuzzyInput = ypr[1] * 180/M_PI + 180;
        double fuzzyOutput = fuzzy->defuzzify(1);

        // Update error and derror
        error = 0 - pitch; // Example target pitch angle is 0 degrees
        derror = error - error_sebelum;
        error_sebelum = error;

        // Set inputs for fuzzy logic
        fuzzy->setInput(1, error);
        fuzzy->setInput(2, derror);

        // Fuzzy computation
        fuzzy->fuzzify();

        // Get output
        float motorOutput = fuzzy->defuzzify(1);

        // Serial prints for logging
        Serial.print("Error: ");
        Serial.print(error);
        Serial.print(", Derror: ");
        Serial.print(derror);
        Serial.print(", Gyroscope: ");
        Serial.print(pitch);
        Serial.print(", PWM: ");
        Serial.println(motorOutput);
        
        // Control motors with fuzzy output
        // if (motorOutput >= 0) {
        //     // Forward motion
        //     motorController.move(motorOutput, MIN_ABS_SPEED);
        // } else {
        //     // Reverse motion
        //     motorController.move(-motorOutput, MIN_ABS_SPEED);
        // }
    }
}




