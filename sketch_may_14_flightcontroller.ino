void serialEventRun(){}

#define HAS_GYRO
// #define DOPRINTS

#include <Wire.h>
#include <DigitalServo.h>
#include <PPMReader.h>

#define M_PI 3.14159265359


// MPU register addresses
#define MPU_REG_SMPLRT_DIV               0x19
// #define MPU_REG_CONFIG                   0x1A
#define MPU_REG_GYRO_CONFIG              0x1B
#define MPU_REG_ACCEL_CONFIG             0x1C
// #define MPU_REG_INT_PIN_CFG              0x37
// #define MPU_REG_INT_ENABLE               0x38
#define MPU_REG_USER_CTRL                0x6A
#define MPU_REG_SIGNAL_PATH_RESET        0x68
#define MPU_REG_PWR_MGMT_1               0x6B
#define MPU_REG_ACCEL_XOUT_H             0x3B
#define MPU_REG_GYRO_XOUT_H              0x43
// #define MPU_REG_TEMP_OUT_H               0x41

// MPU6050 configuration parameters
// sample rate divider, to set the sample rate of the sensor
#define MPU_SAMPLE_RATE_DIV 0x07 // to generate the desired Sample Rate for MPU                              // external FSYNC pin sampling
// #define MPU_EXT_SYNC 0
// digital low pass filter bandwidth
// #define MPU_DLP_BW 0
// gyroscope full scale range
#define MPU_GYRO_FS_RANGE 0x18 // full scale range = ± 1000 °/s
// accelerometer full scale range
#define MPU_ACC_FS_RANGE 0x18 // full Scale Range = ± 16 °/s
// interrupt status bit clear by read operation
// #define MPU_INT_STAT_CLEAR 0x10 // enable
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

#define CALIBRATION_COUNT      500
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

// Gyro + accelerator values
float gyro_pitch, gyro_roll, gyro_yaw;
float accel_pitch, accel_roll, accel_yaw;
// Calibration values
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float accel_roll_cal, accel_pitch_cal, accel_yaw_cal;

// Combined values (gyro + accelerator) by complementary filter
float complementary_roll, complementary_pitch;

// Input values for PID-Controller
float roll_input, pitch_input, yaw_input;

#define MPU_PITCH_GAIN 1.0
#define MPU_ROLL_GAIN 7.5
#define MPU_YAW_GAIN 3.5

// derivative of time in microseconds
unsigned long ct, dt, pt;

// PID and complementary filter correction temporary values
int forceMagnitudeApprox;
float pid_error_temp;
float accel_pitch_temp, accel_roll_temp;

#define UPTAKE_ 0.3
#define ONE_MINUS_UPTAKE 0.7


// PID-Controller

float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, pid_output_yaw, pid_last_yaw_d_error;

#define PID_P_GAIN_ROLL 1.10  //Gain setting for the roll P-controller (1.3)
#define PID_I_GAIN_ROLL 0.00  //Gain setting for the roll I-controller (0.0)
#define PID_D_GAIN_ROLL 0.000 //1; //Gain setting for the roll D-controller (15)
#define PID_MAX_ROLL 650      //Maximum output of the PID-controller (+/-)
#define PID_P_GAIN_PITCH 1.4  //3.5;//Gain setting for the pitch P-controller.
#define PID_I_GAIN_PITCH 0.000//Gain setting for the pitch I-controller.
#define PID_D_GAIN_PITCH 0.000//Gain setting for the pitch D-controller.
#define PID_MAX_PITCH 650     //Maximum output of the PID-controller (+/-)
#define PID_P_GAIN_YAW 1.5    //Gain setting for the pitch P-controller. //4.0
#define PID_I_GAIN_YAW 0.02   //Gain setting for the pitch I-controller. //0.02
#define PID_D_GAIN_YAW 0.000  //1;  //Gain setting for the pitch D-controller.
#define PID_MAX_YAW 650       //Maximum output of the PID-controller (+/-)
#define PID_MAX_YAW_I 100     //Maximum value of Iterm of PID-controller(+/-)


// RC

#define RECEIVER_INTERRUPT_PIN 2
#define SERVO_LEFT_PIN 4
#define SERVO_RIGHT_PIN 5
#define MOTOR_PIN 6
#define HALL_SENSOR_PIN 12

#define CHANNEL_AMOUNT 6

PPMReader ppm(RECEIVER_INTERRUPT_PIN, CHANNEL_AMOUNT);

// stabilizer master gain
float gain = 1.0;

bool glide_mode = true;
int glide_mode_throttle = 1100;
unsigned long glide_requested_at = 0;
int hall_sensor_status = 0;

// ppm startup and failsafe values
#define PPM1_DEFAULT_VALUE 1500  // elevator
#define PPM2_DEFAULT_VALUE 1550  // rudder
#define PPM3_DEFAULT_VALUE 900   // throttle
#define PPM4_DEFAULT_VALUE 1550  // aileron   (not in use)
#define PPM5_DEFAULT_VALUE 1000  // gyro-mode (manual/stabilize/balance)
#define PPM6_DEFAULT_VALUE 1000  // gyro-gain (0.8 ... 2.2)

volatile int rc_throttle = PPM3_DEFAULT_VALUE;
volatile int rc_rudder   = PPM2_DEFAULT_VALUE;
volatile int rc_rudder_stabilized = rc_rudder;
volatile int rc_elevator = PPM1_DEFAULT_VALUE;
volatile int rc_elevator_stabilized = rc_elevator;
volatile int rc_aileron = PPM4_DEFAULT_VALUE;
volatile int rc_aileron_stabilized = rc_aileron;
volatile int rc_gyro_gain = PPM5_DEFAULT_VALUE;
volatile int rc_gyro_mode = PPM6_DEFAULT_VALUE;

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
    pinMode(SERVO_LEFT_PIN, OUTPUT);
    pinMode(SERVO_RIGHT_PIN, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(HALL_SENSOR_PIN, INPUT);

    servo_left.attach(SERVO_LEFT_PIN);
    servo_right.attach(SERVO_RIGHT_PIN);
    motor.attach(MOTOR_PIN);

    delay(50);

    servo_left.writeMicroseconds(PPM1_DEFAULT_VALUE);
    servo_right.writeMicroseconds(PPM2_DEFAULT_VALUE);
    motor.writeMicroseconds(PPM3_DEFAULT_VALUE);

#ifdef HAS_GYRO
    resetPositions();
    initMPU();
    calibrate();
#endif
}


void loop() {
    rc_elevator = ppm.rawChannelValue(1);
    rc_rudder = ppm.rawChannelValue(2);
    rc_throttle = ppm.rawChannelValue(3);
    rc_aileron = ppm.rawChannelValue(4);
    rc_gyro_mode = ppm.rawChannelValue(5);
    rc_gyro_gain = ppm.rawChannelValue(6);

    // arduino failsafe: reset all to default value if values not present
    if (0 == rc_elevator) {
        rc_elevator = PPM1_DEFAULT_VALUE;
        rc_rudder = PPM2_DEFAULT_VALUE;
        rc_throttle = PPM3_DEFAULT_VALUE;
        rc_aileron = PPM4_DEFAULT_VALUE;
        rc_gyro_mode = PPM5_DEFAULT_VALUE;
        rc_gyro_gain = PPM6_DEFAULT_VALUE;
    }


#ifdef HAS_GYRO
    /*
    ENTER CALIBRATION MODE:
    1. Hold model in equilibrium position unmoving
    2. Move left stick to left bottom, right stick to right bottom:
        |    _____        _____    |
        |   /     \      /     \   |
        |  |   .   |    |   .   |  |
        |  |  /    |    |    \  |  |
        |   \˚____/      \____˚/   |
        |                          |
    3. Continue to still hold model unmoving for about 2 seconds
    */
    if (rc_throttle < 1050 && rc_aileron < 1050 && rc_elevator < 1050 && rc_rudder > 1980) {
        resetPositions();
        initMPU();
        calibrate();
    }

    gain = (rc_gyro_gain / 1500.0f) * 1 + 0.2;
#else
    rc_gyro_mode = 1000;
#endif

    ct = micros();
    dt = ct - pt;
    pt = ct;


    /*
    ENTER GLIDE THROTTLE SETTING MODE:
    1. Hold model in air ensuring that wings can move freely
    2. Move left stick to right bottom, right stick to left bottom
    |    _____        _____    |
    |   /     \      /     \   |
    |  |   .   |    |   .   |  |
    |  |    \  |    |  /    |  |
    |   \____˚/      \˚____/   |
    |                          |
    3. Let go of right stick and move throttle to desired position,
        which is set after two exact seconds
    */
    if (rc_throttle < 1050 && rc_aileron < 1950 && rc_elevator < 1050 && rc_rudder > 1050) {
        delay(2000);
        glide_mode_throttle = ppm.rawChannelValue(3);
    }


    // CH5 > 1400 --> gyro/accelerator active
    if (rc_gyro_mode > 1400) {
        accelReadRaw();
        gyroReadRaw();

        applyCalibration();
        applyInversionAndScale();

#ifdef DOPRINTS
    // Serial.print(accel_pitch);
    // Serial.print(",\t");
    // Serial.print(accel_roll);
    // Serial.print(",\t");
    // Serial.print(accel_yaw);
    // Serial.print(",\t");
    // Serial.print(gyro_pitch);
    // Serial.print(",\t");
    // Serial.print(gyro_roll);
    // Serial.print(",\t");
    // Serial.print(gyro_yaw);
    // Serial.print(",\t||\t");
    // Serial.print(complementary_pitch);
    // Serial.print(",\t");
    // Serial.print(complementary_roll);
    // Serial.print(",\t");
    // Serial.println();

#endif

        applyGain();

        // CH5 < 1800 => stabilize-mode: gyroscope enable/accelerometer disable
        if (rc_gyro_mode < 1800) {
            pitch_input = pitch_input * ONE_MINUS_UPTAKE + gyro_pitch * UPTAKE_;
            roll_input  = roll_input * ONE_MINUS_UPTAKE  + gyro_roll * UPTAKE_;
            yaw_input   = yaw_input * ONE_MINUS_UPTAKE   + gyro_yaw * UPTAKE_;

            pid_pitch_setpoint = rc_elevator - 1500;
            pid_roll_setpoint  = rc_rudder - 1500;
            pid_yaw_setpoint   = rc_rudder - 1500;
        }
        // CH5 > 1800 => balance-mode: gyroscope enable/accelerometer enable
        else {
            complementaryFilter(accel_pitch, accel_roll, accel_yaw,
                                gyro_pitch, gyro_roll, gyro_yaw,
                                &complementary_pitch, &complementary_roll);

            pitch_input = pitch_input * ONE_MINUS_UPTAKE + complementary_pitch * UPTAKE_;
            roll_input  = roll_input * ONE_MINUS_UPTAKE  + complementary_roll * UPTAKE_;
            yaw_input   = yaw_input * ONE_MINUS_UPTAKE   + gyro_yaw * UPTAKE_;

            pid_pitch_setpoint = rc_elevator - 1500;
            pid_roll_setpoint  = rc_rudder - 1500;
            pid_yaw_setpoint   = rc_rudder - 1500;
        }

        calculatePid();

        rc_elevator_stabilized = -pid_output_pitch + 1500;
        rc_rudder_stabilized   = -.5 * pid_output_roll - .5 * pid_output_yaw + 1500;
        // rc_aileron_stabilized = pid_output_yaw + 1500;

    }
    // CH5 < 1400 --> manual mode: gyroscope disable/accelerometer disable
    else {
        rc_elevator_stabilized = rc_elevator;
        rc_rudder_stabilized = rc_rudder;
    }

#ifdef DOPRINTS
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
    // Serial.println();
#endif

    if (glide_mode_throttle > rc_throttle) {
        if (false == glide_mode) {
            glide_requested_at = ct;
            glide_mode = true;
        }
        if (0 == glide_requested_at) {
            rc_throttle = 950;
        } else if (1500000 > ct - glide_requested_at) {
            rc_throttle = glide_mode_throttle;
        }
    }
    else {
        glide_mode = false;
    }

    hall_sensor_status = digitalRead(HALL_SENSOR_PIN);

    if (LOW == hall_sensor_status && true == glide_mode) {
        glide_requested_at = 0;
    }

#ifdef DOPRINTS
    Serial.print(hallSensorStatus);
    Serial.print(", \t");
    Serial.print(glide_mode);
    Serial.print(", \t");
    Serial.print(rc_throttle);
    Serial.println();
#endif

    servo_left.writeMicroseconds(constrain(
        0.5 * reverse(rc_elevator_stabilized) + 0.5 * rc_rudder_stabilized,
        870, 1980));
    servo_right.writeMicroseconds(constrain(
        0.5 * rc_elevator_stabilized + 0.5 * rc_rudder_stabilized,
        870, 1980));
    motor.writeMicroseconds(constrain(rc_throttle, 1000, 2000));
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
    accel_pitch *= gain * MPU_PITCH_GAIN;
    accel_roll *= gain * MPU_ROLL_GAIN;
    accel_yaw *= gain * MPU_YAW_GAIN;
    gyro_pitch *= gain * MPU_PITCH_GAIN;
    gyro_roll *= gain * MPU_ROLL_GAIN;
    gyro_yaw *= gain * MPU_YAW_GAIN;
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
    pid_i_mem_pitch += PID_I_GAIN_PITCH * pid_error_temp;
    pid_i_mem_pitch = constrain(pid_i_mem_pitch, -PID_MAX_PITCH, PID_MAX_PITCH);

    pid_output_pitch =  PID_P_GAIN_PITCH * pid_error_temp + pid_i_mem_pitch + PID_D_GAIN_PITCH * (pid_error_temp - pid_last_pitch_d_error);
    pid_output_pitch = constrain(pid_output_pitch, -PID_MAX_PITCH, PID_MAX_PITCH);

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
    pid_i_mem_roll += PID_I_GAIN_ROLL * pid_error_temp;
    pid_i_mem_roll = constrain(pid_i_mem_roll, -PID_MAX_ROLL, PID_MAX_ROLL);

    pid_output_roll =  PID_P_GAIN_ROLL * pid_error_temp + pid_i_mem_roll + PID_D_GAIN_ROLL * (pid_error_temp - pid_last_roll_d_error);
    pid_output_roll = constrain(pid_output_roll, -PID_MAX_ROLL, PID_MAX_ROLL);

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
    pid_i_mem_yaw += PID_I_GAIN_YAW * pid_error_temp;
    pid_i_mem_yaw = constrain(pid_i_mem_yaw, -PID_MAX_YAW_I, PID_MAX_YAW_I);

    pid_output_yaw =  PID_P_GAIN_YAW * pid_error_temp + pid_i_mem_yaw + PID_D_GAIN_YAW * (pid_error_temp - pid_last_yaw_d_error);
    pid_output_yaw = constrain(pid_output_yaw, -PID_MAX_YAW, PID_MAX_YAW);

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
        *roll = *roll * 0.98 + accel_roll_temp * 0.02;
    }
}
