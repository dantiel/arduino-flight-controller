void serialEventRun(){}

#define HAS_GYRO
// #define DOPRINTS

#include <Wire.h>
#include <DigitalServo.h>
#include <PPMReader.h>

#define M_PI 3.14159265359


// MPU register addresses
#define MPU_REG_SMPLRT_DIV               0x19
#define MPU_REG_CONFIG                   0x1A
#define MPU_REG_GYRO_CONFIG              0x1B
#define MPU_REG_ACCEL_CONFIG             0x1C
#define MPU_REG_INT_PIN_CFG              0x37
#define MPU_REG_INT_ENABLE               0x38
#define MPU_REG_USER_CTRL                0x6A
#define MPU_REG_SIGNAL_PATH_RESET        0x68
#define MPU_REG_PWR_MGMT_1               0x6B
#define MPU_REG_ACCEL_XOUT_H             0x3B
#define MPU_REG_GYRO_XOUT_H              0x43
#define MPU_REG_TEMP_OUT_H               0x41

// MPU6050 configuration parameters
// sample rate divider, to set the sample rate of the sensor
#define MPU_SAMPLE_RATE_DIV 0x07 // to generate the desired Sample Rate for MPU                              // external FSYNC pin sampling
#define MPU_EXT_SYNC 0
// digital low pass filter bandwidth
#define MPU_DLP_BW 0
// gyroscope full scale range
#define MPU_GYRO_FS_RANGE 0x18 // full scale range = ± 1000 °/s
// accelerometer full scale range
#define MPU_ACC_FS_RANGE 0x18 // full Scale Range = ± 16 °/s
// interrupt status bit clear by read operation
#define MPU_INT_STAT_CLEAR 0x10 // enable
// set FSYNC pin active logic level
#define MPU_FSYNC_LOGIC_LEVEL 0x80 // active low
// set aux I2C bus access for host
#define MPU_I2C_BYPASS 0x20 // enable
// enable interrupts
// #define MPU_INT_ENABLE 0x59 // enabled interrupts:  motion detection,
                            //                      FIFO overflow,
                            //                      I2C master,
                            //                      data ready
// clock selection
#define MPU_CLK_SEL 0 // internal 8MHz oscillator

// gyroscope scaling factor. This depends on MPU_GYRO_FS_RANGE
#define MPU_GYRO_SCALE_FACTOR            0.060975//0.060975
// accelerometer scaling factor. This depends on MPU_ACC_FS_RANGE
#define MPU_ACC_SCALE_FACTOR             1.5//0.488281

#define CALIBRATION_COUNT      200
#define MPU_I2C_ADDRESS        0x68
#define MPU_PWR_MGMT_1         0x6B   // R/W
#define GYRO_ADDRESS           0x68
#define GYRO_HIGH_BYTE_FIRST

// Set the order in which the gyro values are read
#define GYRO_AXIS_1    gyro_yaw
#define GYRO_AXIS_2    gyro_roll
#define GYRO_AXIS_3    gyro_pitch

// Set the order in which the accelerator values are read
#define ACCEL_AXIS_1   accel_yaw
#define ACCEL_AXIS_2   accel_pitch
#define ACCEL_AXIS_3   accel_roll

float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float gyro_pitch, gyro_roll, gyro_yaw;
float accel_roll_cal, accel_pitch_cal, accel_yaw_cal;
float accel_pitch, accel_roll, accel_yaw;

float complementary_roll, complementary_pitch;
float roll_input, pitch_input, yaw_input;

float mpu_pitch_gain = 1;
float mpu_roll_gain = 7.5;
float mpu_yaw_gain = 3.5;

unsigned long ct, dt, pt;

int forceMagnitudeApprox;
float pid_error_temp;
float accel_pitch_temp, accel_roll_temp;

float uptake = 0.3;
float oneMinusUptake = 1 - uptake;


// PID

float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, pid_output_yaw, pid_last_yaw_d_error;

float pid_p_gain_roll = 1.10; //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.00;//15; //Gain setting for the roll I-controller (0.0)
float pid_d_gain_roll = 0.000;//1; //Gain setting for the roll D-controller (15)
int   pid_max_roll = 650;    //Maximum output of the PID-controller (+/-)
float pid_p_gain_pitch = 1.4;//3.5;//Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.000;//Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 0.000;//Gain setting for the pitch D-controller.
int pid_max_pitch = 650;     //Maximum output of the PID-controller (+/-)
float pid_p_gain_yaw = 1.5;  //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02; //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.000;//1;  //Gain setting for the pitch D-controller.
int pid_max_yaw = 650;       //Maximum output of the PID-controller (+/-)
int pid_max_yaw_i = 100;     //Maximum value of I term of the PID-controller (+/-)


// RC

int receiver_interrupt_pin = 2;
int channel_amount = 6;
PPMReader ppm(receiver_interrupt_pin, channel_amount);

float gain = 1.0;

int gyro_mode = 0;
volatile int rc_throttle = 900;
volatile int rc_rudder = 1550;
volatile int rc_rudder_stabilized = rc_rudder;
volatile int rc_elevator = 1500;
volatile int rc_elevator_stabilized = rc_elevator;
volatile int rc_aileron = 1550;
volatile int rc_aileron_stabilized = rc_aileron;
volatile int rc_gyro_gain = 1000;
volatile int rc_gyro_mode = 1000;

// ppm startup values
int ppm1_default_value = rc_elevator;
int ppm2_default_value = rc_rudder;
int ppm3_default_value = rc_throttle;
int ppm4_default_value = rc_aileron;
int ppm5_default_value = rc_gyro_mode;
int ppm6_default_value = rc_gyro_gain;

volatile int ppm1_value = ppm1_default_value;
volatile int ppm2_value = ppm2_default_value;
volatile int ppm3_value = ppm3_default_value;
volatile int ppm4_value = ppm4_default_value;
volatile int ppm5_value = ppm5_default_value;
volatile int ppm6_value = ppm6_default_value;

int servo_left_pin = 4;
int servo_right_pin = 5;
int motor_pin = 6;
DigitalServo servo_left, servo_right, motor;




void setup() {

#ifdef DOPRINTS
    Serial.begin(19200);
#endif

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    //Configure servo pins as output.
    pinMode(servo_left_pin, OUTPUT);
    pinMode(servo_right_pin, OUTPUT);
    pinMode(motor_pin, OUTPUT);
    servo_left.attach(servo_left_pin);
    servo_right.attach(servo_right_pin);
    motor.attach(motor_pin);

    delay(50);

    servo_left.writeMicroseconds(ppm1_value);
    servo_right.writeMicroseconds(ppm2_value);
    motor.writeMicroseconds(ppm3_value);

#ifdef HAS_GYRO
    resetPositions();
    initMPU();
    calibrate();
#endif
}


void loop() {
    ppm1_value = ppm.rawChannelValue(1);
    ppm2_value = ppm.rawChannelValue(2);
    ppm3_value = ppm.rawChannelValue(3);
    ppm4_value = ppm.rawChannelValue(4);
    ppm5_value = ppm.rawChannelValue(5);
    ppm6_value = ppm.rawChannelValue(6);


    if (0 == ppm1_value) {
        rc_elevator = ppm1_default_value;
        rc_rudder = ppm2_default_value;
        rc_throttle = ppm3_default_value;
        rc_aileron = ppm4_default_value;
        rc_gyro_mode = ppm5_default_value;
        rc_gyro_gain = ppm6_default_value;
    }
    else {
        rc_elevator = ppm1_value;
        rc_rudder = ppm2_value;
        rc_throttle = ppm3_value;
        rc_aileron = ppm4_value;
        rc_gyro_mode = ppm5_value;
        rc_gyro_gain = ppm6_value;
    }
    // rc_aileron = ppm4_value;
    // rc_elevator_stabilizer_magnification = ppm.rawChannelValue(5);
    // rc_rudder_stabilizer_magnification = ppm.rawChannelValue(6);


#ifdef HAS_GYRO

    if (rc_throttle < 1020 && rc_aileron < 1020 && rc_elevator < 1020 && rc_rudder > 1980) {
        resetPositions();
        initMPU();
        calibrate();
    }

    ct = micros();
    dt = ct - pt;
    pt = ct;
    //
    if (rc_gyro_mode > 1800) {
        gyro_mode = 2;
    }
    else if (rc_gyro_mode > 1400) {
        gyro_mode = 1;
    }
    else {
        gyro_mode = 0;
    }

    gain = (rc_gyro_gain / 1500.0f) * 1 + 0.2;

#else
    gyro_mode = 0;
#endif

    if (gyro_mode > 0) {
        accelReadRaw();
        gyroReadRaw();

        applyCalibration();
        applyInversionAndScale();

#ifdef DOPRINTS
    Serial.print(accel_pitch);
    Serial.print(",\t");
    Serial.print(accel_roll);
    Serial.print(",\t");
    Serial.print(accel_yaw);
    Serial.print(",\t");
    Serial.print(gyro_pitch);
    Serial.print(",\t");
    Serial.print(gyro_roll);
    Serial.print(",\t");
    Serial.print(gyro_yaw);
    Serial.print(",\t||\t");
    Serial.print(complementary_pitch);
    Serial.print(",\t");
    Serial.print(complementary_roll);
    Serial.print(",\t");
    Serial.println();

#endif

        applyGain();

        if (gyro_mode == 1) {
            pitch_input = (pitch_input * oneMinusUptake) + (gyro_pitch * uptake);
            roll_input  = (roll_input * oneMinusUptake)  + (gyro_roll * uptake);
            yaw_input   = (yaw_input * oneMinusUptake)   + (gyro_yaw * uptake);

            pid_pitch_setpoint = rc_elevator - 1500;
            pid_roll_setpoint  = rc_rudder - 1500;
            pid_yaw_setpoint   = rc_rudder - 1500;

            calculatePid();

            rc_elevator_stabilized = -pid_output_pitch + 1500;
            rc_rudder_stabilized   = -.5*pid_output_roll - .5*pid_output_yaw + 1500;
            // rc_aileron_stabilized  = pid_output_yaw + 1500;
        }
        else if (gyro_mode == 2) {
            complementaryFilter(accel_pitch, accel_roll, accel_yaw,
                                gyro_pitch, gyro_roll, gyro_yaw,
                                &complementary_pitch, &complementary_roll);

            pitch_input = (pitch_input * oneMinusUptake) + (complementary_pitch * uptake);
            roll_input  = (roll_input * oneMinusUptake)  + (complementary_roll * uptake);
            yaw_input   = (yaw_input * oneMinusUptake)   + (gyro_yaw * uptake);

            pid_pitch_setpoint = rc_elevator - 1500;
            pid_roll_setpoint  = rc_rudder - 1500;
            pid_yaw_setpoint   = rc_rudder - 1500;

            calculatePid();

            rc_elevator_stabilized = -pid_output_pitch + 1500;
            rc_rudder_stabilized   = -.5 * pid_output_roll - .5 * pid_output_yaw + 1500;
            // rc_aileron_stabilized  = pid_output_yaw + 1500;
        }
        // rc_aileron_stabilized = pid_output_yaw + 1500;

    }
    else {
        rc_elevator_stabilized = rc_elevator;
        rc_rudder_stabilized = rc_rudder;
    }

#ifdef DOPRINTS
    // Serial.print(ppm1_value);
    // Serial.print(",\t");
    // Serial.print(ppm2_value);
    // Serial.print(",\t");
    // Serial.print(ppm3_value);
    // Serial.print(",\t");
    // Serial.print(ppm4_value);
    // Serial.print(",\t");
    // Serial.print(ppm5_value);
    // Serial.print(",\t");
    // Serial.print(ppm6_value);
    // Serial.print(",\t");

    // Serial.print(rc_elevator_stabilized);
    // Serial.print(",\t");
    // Serial.print(rc_rudder_stabilized);
    // Serial.print(",\t");
    // Serial.print(rc_throttle);
    // Serial.print(",\t");
    // Serial.print(rc_aileron_stabilized);
    // Serial.print("----\t");

    // Serial.print(0.5 * reverse(rc_elevator_stabilized) + 0.5 * (rc_rudder_stabilized));
    // Serial.print(",\t");
    // Serial.print(0.5 * rc_elevator_stabilized + 0.5 * (rc_rudder_stabilized));
    // Serial.print("----\t");

    // Serial.print(rc_throttle + ((- rc_aileron_stabilized) * 0.5));
    // Serial.print(",\t");
    // Serial.print(rc_throttle - ((- rc_aileron_stabilized) * 0.5));
    Serial.println();
#endif

    servo_left.writeMicroseconds(constrain(0.5 * reverse(rc_elevator_stabilized) + 0.5 * (rc_rudder_stabilized),870,1980));
    servo_right.writeMicroseconds(constrain(0.5 * rc_elevator_stabilized + 0.5 * (rc_rudder_stabilized),870,1980));
    motor.writeMicroseconds(constrain(rc_throttle,1000,2000));
}


int mpuReadInt() {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();

    return (highByte << 8) | lowByte;
}



void gyroReadRaw() {
    Wire.beginTransmission(GYRO_ADDRESS);//Start communication with the gyro
    Wire.write(MPU_REG_GYRO_XOUT_H);    //Start reading and auto increment with every read
    Wire.endTransmission();              //End the transmission
    Wire.requestFrom(GYRO_ADDRESS, 6);   //Request 6 bytes from the gyro

    while (Wire.available() < 6);        //Wait until the 6 bytes are received
    GYRO_AXIS_1 = mpuReadInt();
    GYRO_AXIS_2 = mpuReadInt();
    GYRO_AXIS_3 = mpuReadInt();
}


void accelReadRaw() {
    Wire.beginTransmission(GYRO_ADDRESS);//Start communication with the gyro
    Wire.write(MPU_REG_ACCEL_XOUT_H);    //Start reading and auto increment with every read
    Wire.endTransmission();              //End the transmission
    Wire.requestFrom(GYRO_ADDRESS, 6);   //Request 6 bytes from the gyro

    while (Wire.available() < 6);        //Wait until the 6 bytes are received
    ACCEL_AXIS_1 = mpuReadInt();
    ACCEL_AXIS_2 = mpuReadInt();
    ACCEL_AXIS_3 = mpuReadInt();
}


inline void applyCalibration() {
    accel_pitch -= accel_pitch_cal;
    accel_roll  -= accel_roll_cal;
    accel_yaw   -= accel_yaw_cal;
    gyro_pitch -= gyro_pitch_cal;
    gyro_roll  -= gyro_roll_cal;
    gyro_yaw   -= gyro_yaw_cal;
}


inline void applyInversionAndScale() {
    accel_pitch *= 1 * MPU_ACC_SCALE_FACTOR;
    accel_roll  *= -1 * MPU_ACC_SCALE_FACTOR;
    accel_yaw   *= -1 * MPU_ACC_SCALE_FACTOR;
    gyro_pitch  *= -1 * MPU_GYRO_SCALE_FACTOR;
    gyro_roll   *= 1 * MPU_GYRO_SCALE_FACTOR;
    gyro_yaw    *= 1 * MPU_GYRO_SCALE_FACTOR;
}


inline void applyGain() {
    accel_pitch *= gain * mpu_pitch_gain;
    accel_roll *= gain * mpu_roll_gain;
    accel_yaw *= gain * mpu_yaw_gain;
    gyro_pitch *= gain * mpu_pitch_gain;
    gyro_roll *= gain * mpu_roll_gain;
    gyro_yaw *= gain * mpu_yaw_gain;
}


void resetPositions() {
    servo_left.writeMicroseconds(1500);
    servo_right.writeMicroseconds(1500);
    motor.writeMicroseconds(900);
}


void initMPU() {

    // PWR_MGMT_1    -- DEVICE_RESET 1
    i2cWriteReg(GYRO_ADDRESS, MPU_REG_PWR_MGMT_1, 0x80);

    delayMicroseconds(5000);

    // PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    i2cWriteReg(GYRO_ADDRESS, MPU_REG_PWR_MGMT_1, 0x03);

    // CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    // i2cWriteReg(GYRO_ADDRESS, 0x1A, 0);
    // GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 1000 deg/sec
    // i2cWriteReg(GYRO_ADDRESS, MPU_REG_GYRO_CONFIG, MPU_GYRO_FS_RANGE);


    // set sampling rate
    i2cWriteReg(GYRO_ADDRESS, MPU_REG_SMPLRT_DIV, MPU_SAMPLE_RATE_DIV);
    // FSYNC and digital low pass filter settings
    // i2cWriteReg(GYRO_ADDRESS, MPU_REG_CONFIG, (MPU_EXT_SYNC | MPU_DLP_BW) );
    // set gyroscope full scale range
    i2cWriteReg(GYRO_ADDRESS, MPU_REG_GYRO_CONFIG, MPU_GYRO_FS_RANGE);
    // set accelerometer full scale range
    i2cWriteReg(GYRO_ADDRESS, MPU_REG_ACCEL_CONFIG, MPU_ACC_FS_RANGE);

    // MPU control functions
    // set interrupt clear option, FSYNC logic level, aux bus access
    // i2cWriteReg(MPU_REG_INT_PIN_CFG,
        // (MPU_INT_STAT_CLEAR | MPU_FSYNC_LOGIC_LEVEL | MPU_I2C_BYPASS));
    // enable interrupts
    // i2cWriteReg(MPU_REG_INT_ENABLE, MPU_INT_ENABLE);
    // configure MPU hardware FIFO


    // configure MPU hardware FIFO
    i2cWriteReg(GYRO_ADDRESS, MPU_REG_USER_CTRL, 0x40);
    // reset the analog and digital signal paths of all on-chip sensors
    i2cWriteReg(GYRO_ADDRESS, MPU_REG_SIGNAL_PATH_RESET, 0x07);
    // CLKSEL is a 3-bit unsigned value specifying // the clock source of the device
    i2cWriteReg(GYRO_ADDRESS, MPU_REG_PWR_MGMT_1, MPU_CLK_SEL);
    // Clear the 'sleep' bit to start the sensor.
    mpuWriteReg (MPU_PWR_MGMT_1, 0);
}


void calibrate() {
    gyro_pitch_cal = 0;
    gyro_roll_cal = 0;
    gyro_yaw_cal = 0;
    accel_pitch_cal = 0;
    accel_roll_cal = 0;
    accel_yaw_cal = 0;

#ifdef DOPRINTS
    Serial.println("Start Calibration");
#endif
    for (int i = 0; i < CALIBRATION_COUNT; i++) {
        gyroReadRaw();
        gyro_pitch_cal += gyro_pitch;
        gyro_roll_cal  += gyro_roll;
        gyro_yaw_cal   += gyro_yaw;
        accelReadRaw();
        accel_pitch_cal += accel_pitch;
        accel_roll_cal  += accel_roll;
        accel_yaw_cal   += accel_yaw;
        delayMicroseconds(3000);
    }

    gyro_pitch_cal /= CALIBRATION_COUNT;
    gyro_roll_cal  /= CALIBRATION_COUNT;
    gyro_yaw_cal   /= CALIBRATION_COUNT;
    accel_pitch_cal /= CALIBRATION_COUNT;
    accel_roll_cal  /= CALIBRATION_COUNT;
    accel_yaw_cal   /= CALIBRATION_COUNT;
#ifdef DOPRINTS
    // Serial.println("Calibration Done. Values:");
    // Serial.print("accel_pitch_cal="); Serial.println(accel_pitch_cal);
    // Serial.print("accel_roll_cal="); Serial.println(accel_roll_cal);
    // Serial.print("accel_yaw_cal="); Serial.println(accel_yaw_cal);
    // Serial.print("gyro_pitch_cal="); Serial.println(gyro_pitch_cal);
    // Serial.print("gyro_roll_cal="); Serial.println(gyro_roll_cal);
    // Serial.print("gyro_yaw_cal="); Serial.println(gyro_yaw_cal);
#endif
}


int reverse(int val) {
 return (val-1500) * -1 + 1500;
}


int mpuWrite(int start, const uint8_t *pData, int size) {
    int n, error;

    Wire.beginTransmission(MPU_I2C_ADDRESS);
    n = Wire.write(start);        // write the start address
    if (n != 1)
        return (-20);

    n = Wire.write(pData, size);  // write data bytes
    if (n != size)
        return (-21);

    error = Wire.endTransmission(true); // release the I2C-bus
    if (error != 0)
        return (error);

    return (0);         // return : no error
}


int mpuWriteReg(int reg, uint8_t data) {
    int error;

    error = mpuWrite(reg, &data, 1);

    return (error);
}


void i2cWriteReg(int address, byte reg, byte val) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}


void calculatePid() {
    pid_error_temp = pitch_input - pid_pitch_setpoint;
    //Pitch calculations
    pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
    pid_i_mem_pitch = constrain(pid_i_mem_pitch, -pid_max_pitch, pid_max_pitch);

    pid_output_pitch =  pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
    pid_output_pitch = constrain(pid_output_pitch, -pid_max_pitch, pid_max_pitch);

    pid_last_pitch_d_error = pid_error_temp;

#ifdef DOPRINTS
    // Serial.print(pitch_input);
    // Serial.print(",\t");
    // Serial.print(pid_pitch_setpoint);
    // Serial.print(",\t");
    // Serial.print(pid_output_pitch);
    // Serial.print(",\t||\t");
#endif

    //Roll calculations
    pid_error_temp = roll_input - pid_roll_setpoint;
    pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
    pid_i_mem_roll = constrain(pid_i_mem_roll, -pid_max_roll, pid_max_roll);

    pid_output_roll =  pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
    pid_output_roll = constrain(pid_output_roll, -pid_max_roll, pid_max_roll);

    pid_last_roll_d_error = pid_error_temp;

#ifdef DOPRINTS
    // Serial.print(roll_input);
    // Serial.print(",\t");
    // Serial.print(pid_yaw_setpoint);
    // Serial.print(",\t");
    // Serial.print(pid_output_roll);
    // Serial.print(",\t||\t");
    // Serial.println();
#endif

    //Yaw calculations
    pid_error_temp = yaw_input - pid_yaw_setpoint;
    pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
    pid_i_mem_yaw = constrain(pid_i_mem_yaw, -pid_max_yaw_i, pid_max_yaw_i);

    pid_output_yaw =  pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
    pid_output_yaw = constrain(pid_output_yaw, -pid_max_yaw, pid_max_yaw);

    pid_last_yaw_d_error = pid_error_temp;

#ifdef DOPRINTS
    // Serial.print(yaw_input);
    // Serial.print(",\t");
    // Serial.print(pid_yaw_setpoint);
    // Serial.print(",\t");
    // Serial.print(pid_output_yaw);
    // Serial.print(",\t");
    // Serial.println();
#endif
}


// complementary filter by https://www.pieter-jan.com/node/11

void complementaryFilter(short accel_pitch, short accel_roll, short accel_yaw,
                         short gyro_pitch, short gyro_roll, short gyro_yaw,
                         float *pitch, float *roll) {

    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float)gyro_pitch) * dt * 0.00001; // Angle around the X-axis
    *roll += ((float)gyro_roll) * dt * 0.00001;    // Angle around the Y-axis

    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    forceMagnitudeApprox = abs(accel_pitch) + abs(accel_roll) + abs(accel_yaw);
    // Serial.print(forceMagnitudeApprox);
    // Serial.print("==");

    if (forceMagnitudeApprox > 81.92 && forceMagnitudeApprox < 32768) {
        // Serial.print("XXX");
        // Turning around the X axis results in a vector on the Y-axis
        accel_pitch_temp = atan2f((float)accel_pitch, -(float)accel_yaw) * 180 / M_PI;
        *pitch = *pitch * 0.98 + accel_pitch_temp * 0.02;

        // Turning around the Y axis results in a vector on the X-axis
        accel_roll_temp = atan2f((float)accel_roll, (float)accel_yaw) * 180 / M_PI;
        *roll = *roll * 0.98 + accel_roll_temp * 0.02;
    }
}
