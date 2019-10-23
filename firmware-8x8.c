/***************************************************************
   LIBRARIES
****************************************************************/
#include <Adafruit_PWMServoDriver.h>
#include <i2c_t3.h>
#include <Bounce.h>

/***************************************************************
    DEFINES
****************************************************************/
#define GRID_SIZE 8
#define NUMBER_OF_SERVOS GRID_SIZE * GRID_SIZE
#define CHANNELS_PER_CTR 16
#define SERVO_MIN 85
#define SERVO_MAX 437
#define SERVO_BOARDS_PER_BUS 4
#define u8 uint8_t
#define u16 uint16_t
#define MAX_ANGLE_DISPLACEMENT 40
#define NUM_TO_COORD(X) servo_max_cal[(int)floor(X / (NUMBER_OF_SERVOS / 2))][X % (NUMBER_OF_SERVOS / 2)]
#define COORD_TO_CHANNEL(X, Y) ((X * GRID_SIZE) + Y) % CHANNELS_PER_CTR
#define BUTTON 32
#define LAMBDA_HALF 4.29 // in mm 
#define INIT_ALPHA 0.02188776 // CONCAVITY FOR THE BOWL 
#define ANGLE_STEP_TO_MM 0.1589
#define DELTA 4.0 // in mm, the distance between the centre of the reflectors
#define WRITE_DELAY 400

/***************************************************************
   GLOBALS
****************************************************************/
float x[GRID_SIZE][GRID_SIZE];
float y[GRID_SIZE][GRID_SIZE];
float z[GRID_SIZE][GRID_SIZE];
u8 z_angle[GRID_SIZE][GRID_SIZE];
int button_pressed = LOW;
u16 servo_max_cal[GRID_SIZE][GRID_SIZE] = // put calibrated values here
{ {430, 437, 428, 432, 437, 425, 442, 432},
  {444, 432, 428, 425, 431, 420, 422, 422},
  {444, 435, 427, 440, 445, 445, 445, 445},
  {445, 445, 445, 445, 445, 445, 445, 445},
  {445, 445, 445, 445, 445, 445, 445, 445},
  {445, 445, 445, 445, 445, 445, 445, 445},
  {445, 445, 445, 445, 445, 445, 445, 445},
  {445, 445, 445, 445, 445, 445, 445, 445}
};
Bounce bouncer = Bounce( BUTTON, 5 );
Adafruit_PWMServoDriver servo_ctr_0 = Adafruit_PWMServoDriver(&Wire, 0x40);
Adafruit_PWMServoDriver servo_ctr_1 = Adafruit_PWMServoDriver(&Wire, 0x41);
Adafruit_PWMServoDriver servo_ctr_2 = Adafruit_PWMServoDriver(&Wire, 0x42);
Adafruit_PWMServoDriver servo_ctr_3 = Adafruit_PWMServoDriver(&Wire, 0x43);
Adafruit_PWMServoDriver* servo_ctr_bus[SERVO_BOARDS_PER_BUS] = {&servo_ctr_0, &servo_ctr_1, &servo_ctr_2, &servo_ctr_3};

/***************************************************************
    SETUP
****************************************************************/
void setup() {
  Serial.begin(115200);
  for (u8 i = 0; i < GRID_SIZE; i++ ) {
    for (u8 j = 0; j < GRID_SIZE; j++ ) {
      x[i][j] = -(DELTA / 2.0) - ((GRID_SIZE / 2.0) - 1) * DELTA + i * DELTA;
      y[i][j] = -(DELTA / 2.0) - ((GRID_SIZE / 2.0) - 1) * DELTA + j * DELTA;
    }
  }
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  reset_servos();
  delay(500);
}

/***************************************************************
   LOOP
****************************************************************/
void loop() {
  for (u8 i = 0; i < 5; i++ ) {
    Serial.print("i is: "); Serial.println(i);
    bowl(0.0, 0.0 + (2 * i), 0.0, INIT_ALPHA);
    write_array(z_angle, WRITE_DELAY);
  }
  
}

/***************************************************************
   FUNCTIONS 
****************************************************************/

void write_array(u8 input[GRID_SIZE][GRID_SIZE], u16 _delay) {
  for (u8 x = 0; x < GRID_SIZE; x++ ) {
    for (u8 y = 0; y < GRID_SIZE; y++ ) {
      move_motor(x, y, input[x][y]);
      Serial.print(degrees_to_duty_cycle(input[x][y], x, y)); Serial.print("\t");
    }
  }
  delay(_delay);
}

/**
   Enter number in degrees, outputs duty cycle
*/
int degrees_to_duty_cycle(u8 degree, u8 x, u8 y) {
  return map(degree, 0, 180, (long)SERVO_MIN, servo_max_cal[x][y]);
}

/**
  Move the motor
*/
void move_motor(u8 x, u8 y, u8 angle) {
  u8 i = (floor(((x * GRID_SIZE) + y) / CHANNELS_PER_CTR ));
  servo_ctr_bus[i]->setPWM(COORD_TO_CHANNEL(x, y), 0, degrees_to_duty_cycle(angle, x, y));
}

/**
   Reset the servos to the zeroth position
*/
void reset_servos(void) {
  for (u8 i = 0; i < SERVO_BOARDS_PER_BUS; i++ ) {
    servo_ctr_bus[i]->begin();
    delay(30);
  }
  Serial.println("Reset!");
}

/**
   Generate a bown shape based on the input parameters 
*/
void bowl(float x_off, float y_off, float z_off, float alpha) {
  for (u8 i = 0; i < GRID_SIZE; i++ ) {
    for (u8 j = 0; j < GRID_SIZE; j++ ) {
      z[i][j] = alpha * (sq(x[i][j] - x_off) + sq(y[i][j] - y_off)) + z_off;
      if (z[i][j] > LAMBDA_HALF)
        z[i][j] = LAMBDA_HALF;
      z_angle[i][j] = (u8)round(z[i][j] / ANGLE_STEP_TO_MM);
      Serial.print(z_angle[i][j]); Serial.print("\t");
    }
  }
}


