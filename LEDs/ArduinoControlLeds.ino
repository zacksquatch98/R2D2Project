#define LED_YELLOW_PIN A0
#define LED_GREEN_PIN A1
#define LED_RED_PIN A2
#define LED_BLUE_PIN A3

void powerOffAllLEDs()
{
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);

  powerOffAllLEDs();
}

void loop() {
  
  if (Serial.available() > 0) {
    int ledState = Serial.read() - '0';

    powerOffAllLEDs();
    
//    String data = Serial.readStringUntil('\n');
//    Serial.print("You sent me: ");
//    Serial.println(data);

    switch (ledState) {
      case 1:
        digitalWrite(LED_YELLOW_PIN, HIGH);
        break;
      case 2:
        digitalWrite(LED_GREEN_PIN, HIGH);
        break;
      case 3:
        digitalWrite(LED_RED_PIN, HIGH);
        break;
      case 4:
        digitalWrite(LED_BLUE_PIN, HIGH);
        break;
      default:
        break;
    }
  }
}
