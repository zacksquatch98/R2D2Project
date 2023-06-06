#define LED_Y_PIN A0
#define LED_G_PIN A1
#define LED_R_PIN A2
#define LED_B_PIN A3

#define LCD_PIN 12
#define SERVO_PIN 10

#define LB_EN2_PIN 2 // LEFT BACK
#define LF_EN4_PIN 3 // LEFT FRONT
#define RB_EN3_PIN 4 // RIGHT BACK
#define RF_EN1_PIN 5 // RIGHT FRONT

// Head Motor pins
#define EN1_PIN 9
#define EN2_PIN 6
#define EN3_PIN 8
#define EN4_PIN 7

#define DEFAULT_SPEED 255
#define MAX_SPEED 255 // Should never exceed 255
#define INC 50

#define ON HIGH
#define OFF LOW

int WHEEL_SPEED = DEFAULT_SPEED;

// LED and LCD states
int LED_R;
int LED_B;
int LED_Y;
int LED_G;
int LCD;

// Motor states
int LEFT_FORWARD;
int LEFT_BACK;
int RIGHT_FORWARD;
int RIGHT_BACK;

/* Set all LED pins with LOW signal. */
void powerOffAllLEDs()
{
  digitalWrite(LED_Y_PIN, LOW);
  digitalWrite(LED_G_PIN, LOW);
  digitalWrite(LED_R_PIN, LOW);
  digitalWrite(LED_B_PIN, LOW);
}

/* Set the state of all LEDs to be OFF. */
void setAllLEDsOFF()
{
  LED_R = OFF;
  LED_B = OFF;
  LED_Y = OFF;
  LED_G = OFF;
}

/* Set all motor controller pins with LOW signal. */
void powerOffAllMotors()
{
  digitalWrite(LB_EN2_PIN, LOW);
  digitalWrite(LF_EN4_PIN, LOW);
  digitalWrite(RB_EN3_PIN, LOW);
  digitalWrite(RF_EN1_PIN, LOW);
}

/* Set the state of all wheel motors to be false. */
void setAllMotorsOff()
{
  LEFT_FORWARD = false;
  LEFT_BACK = false;
  RIGHT_FORWARD = false;
  RIGHT_BACK = false;
}

/* 
 *  Sets the state of the wheel motors. 
 *  dir: incoming instruction code
 */
void setMotors(char dir)
{
  setAllMotorsOff();
  switch (dir) {
    case 's':
      // stop
      setAllMotorsOff();
      break;
    case 'f':
      // go forward
      LEFT_FORWARD = true;
      RIGHT_FORWARD = true;
      break;
    case 'b':
      // go backward
      LEFT_BACK = true;
      RIGHT_BACK = true;
      break;
    case 'l':
      // turn left
      RIGHT_FORWARD = true;
      break;
    case 'r':
      // turn right
      LEFT_FORWARD = true;
      break;
    default:
      break;
  }
}

/* 
 *  Sets the wheel speed for the motors.
 *  dir: incoming instruction code
 */
void setWheelSpeed(char dir)
{
  switch(dir) {
    case 'u':
      // speed up
      WHEEL_SPEED += INC;
      if (WHEEL_SPEED > MAX_SPEED) {
        WHEEL_SPEED = MAX_SPEED;
      }
      break;
    case 'd':
      // speed down
      WHEEL_SPEED -= INC;
      if (WHEEL_SPEED < 0) {
        WHEEL_SPEED = 0;
      }
      break;
    case 'n':
      // normal speed
      WHEEL_SPEED = DEFAULT_SPEED;
      break;
  }

}

/* 
 *  Sets the state of the LEDs. 
 *  LED: instruction corresponding to the color
 */
void turnOnLED(char LED)
{
  setAllLEDsOFF();
  switch (LED) {
  case 'y':
    LED_Y = ON;
    break;
  case 'g':
    LED_G = ON;
    break;
  case 'r':
    LED_R = ON;
    break;
  case 'b':
    LED_B = ON;
    break;
  default:
    break;
  }
}

void setup() {
  Serial.begin(9600); //Sets the data rate in bits per second (baud) for serial data transmission
  
  // Set up motors as output
  pinMode(LB_EN2_PIN, OUTPUT);
  pinMode(LF_EN4_PIN, OUTPUT);
  pinMode(RB_EN3_PIN, OUTPUT);
  pinMode(RF_EN1_PIN, OUTPUT);

  // Set up LEDs and LCD as output
  pinMode(LED_Y_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(LCD_PIN, OUTPUT);

  // Power off all outputs and set their states to be off.
  digitalWrite(LCD_PIN, LOW);
  LCD = OFF;
  powerOffAllLEDs();
  setAllLEDsOFF();
  powerOffAllMotors();
  setAllMotorsOff();

}

void loop() {
  if (Serial.available() > 0) {
    // Read in instruction string.
    String instruction = Serial.readStringUntil('\n');
//    char instruction = Serial.read();

    // Based on first char of instruction set the appropriate actions
    switch (instruction[0]) {
      case 'm':
        // move wheels
        setMotors(instruction[1]);
        break;
      case 's':
        // speed of wheels
        setWheelSpeed(instruction[1]);
      case 'l':
        // leds
        turnOnLED(instruction[1]);
        break;
      case 'd':
        // lcd
        LCD = ON;
        break;
      case 'q':
        // quit - turn off everything
        setAllLEDsOFF();
        setAllMotorsOff();
        LCD = OFF;
    }
  }

  // After updating the states, write the values to the pins.
  analogWrite(LF_EN4_PIN, (LEFT_FORWARD * WHEEL_SPEED));
  analogWrite(LB_EN2_PIN, (LEFT_BACK * WHEEL_SPEED));
  analogWrite(RF_EN1_PIN, (RIGHT_FORWARD * WHEEL_SPEED));
  analogWrite(RB_EN3_PIN, (RIGHT_BACK * WHEEL_SPEED));

  digitalWrite(LED_R_PIN, LED_R);
  digitalWrite(LED_B_PIN, LED_B);
  digitalWrite(LCD_PIN, LCD);
}
