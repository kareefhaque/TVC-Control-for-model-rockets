#include <PWMServo.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

PWMServo ServoX;
PWMServo ServoY;

//variables for PID controller
float errorX; //measured accelerometer error in x direction
float errorX_i; //integral control error for x
float errorX_d; //derivative control error for x
float lasterrorX; //getting dx for derivative error for x
float PID_X; //total PID value for x (accel only)
float pwm_x; //servo write value for x
float errorY; //measured accelerometer error in y direction
float errorY_i; //integral control error for y
float errorY_d; //derivative control error for y
float lasterrorY; //getting dx for derrivative error for y
float PID_Y; //total PID value for y (accel only)
float pwm_y; //servo write value for y
float pidX_p; //proportional value multiplied by gain
float pidX_i; //integral value multiplied by gain
float pidX_d; //derivative value multiplied by gain
float pidY_p; //p-value for y
float pidY_i; //i-value for y
float pidY_d; //d-value for y

float servoCenter = 90; //resetting servos to basic position

float desiredValueX; //accel value we want for x
float desiredValueY; //accel value we want for y

//gain values (MODIFY FOR DIFFERENT ROCKETS AND ENGINES)
float kp = 0.6; // insert your p gain
float ki = 0.2; // insert your i gain
float kd = 0.1; // insert your d gain

//time values for integral and derivative
unsigned long counter;
unsigned long lastCounter;
float timeTaken;

void setup() {
  ServoX.attach(5); //attaching x-servo to PWM pin 5
  ServoY.attach(6); //attaching y-servo to PWM pin 6

  //resetting servos
  ServoX.write(servoCenter);
  ServoY.write(servoCenter);

  //initializing accelerometer
  Wire.begin();
  mpu.begin();
  Serial.begin(115200); 
  if (!mpu.begin()) {
  Serial.println("Failed to find MPU6050 chip");
  while (1) {
    delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
}

void loop() {
  desiredValueX = 0;
  desiredValueY = 0;
  pid();
}

//PID controller, my sweet baby boy who I barely understand
void pid() {
  lastCounter = millis();
  if (millis() >= counter + 10) {
  timeTaken = lastCounter - counter;
  counter = millis();
  Serial.println(timeTaken);
  //mpu update
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
 
  float y = a.acceleration.y;
  float x = a.acceleration.x;
  
  errorX = x - desiredValueX;
  errorX_i += errorX * timeTaken / 1000;
  errorX_d = (errorX - lasterrorX) / timeTaken / 1000;
  lasterrorX = errorX;
  pidX_p = kp * errorX;
  pidX_i = ki * errorX_i;
  pidX_d = kd * errorX_d;
  PID_X = pidX_p +  pidX_i + pidX_d;
  
  Serial.println("X: ");
  Serial.print(PID_X);
  
  errorY = y - desiredValueY;
  errorY_i += errorY * timeTaken / 1000; 
  errorY_d = (errorY - lasterrorY) / timeTaken / 1000;
  lasterrorY = errorY;
  pidY_p = kp * errorY;
  pidY_i = ki * errorY_i;
  pidY_d = kd * errorY_d;
  PID_Y = pidY_p +  pidY_i + pidY_d;
  
  Serial.println("Y: ");
  Serial.print(PID_Y);

  clampIntegrals();

  //pid clamps for x (cant move past these values or windup will happen)
  if (PID_X < -10) { //input the angle your tvc mount can move
  PID_X = -10;       //input the angle your tvc mount can move
  }
  if (PID_X > 10) { //input the angle your tvc mount can move
  PID_X = 10;       //input the angle your tvc mount can move
  }

  //pid clamps for y
  if (PID_Y < -10) { //input the angle your tvc mount can move
  PID_Y = -10;      //input the angle your tvc mount can move
  }
  if (PID_Y > 10) { //input the angle your tvc mount can move
  PID_Y = 10;      //input the angle your tvc mount can move
  }
  
  servoWrite();
  
  //Go to serial plotter and see the results
  Serial.print(pidY_p); 
  Serial.print(",");
  Serial.print(pidY_i);
  Serial.print(",");
  Serial.print(pidY_d);
  Serial.print(",");
  Serial.print(PID_Y);  
  Serial.print(",");
  Serial.println(x); 
}
}

void servoWrite(){
  pwm_x = map (PID_X, -10, 10, 0, 180); //translates accelerometer readings to servo steps, depends on angle mount can move
  
  pwm_y = map (PID_Y, -10, 10, 0, 180);  //translates accelerometer readings to servo steps, depends on angle mount can move
  
  ServoX.write(pwm_x);
  ServoY.write(pwm_y);
}

void clampIntegrals(){
  //integral clamps for x, prevents windup
  if (errorX_i > 10) { // if it goes past 10, it causes windup
  errorX_i = 10; // clamps it to 10
  }
  if (errorX_i < -10) { //if it goes past -10, it causes windup
  errorX_i = -10; // clamps it to -10
  }

  //integral clamps for y, prevents windup
  if (errorY_i < -10) { // if it goes past -10, it causes windup
  errorY_i = -10; // clamps it to 10
  }
  if (errorY_i > 10) { // if it goes past 10, it causes windup
  errorY_i = 10; // clamps it to 10
  } 
}
