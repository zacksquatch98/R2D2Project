int speedVal = 16;
int maxSpeed = 255; // Should never be higher than 255
int increment = 16;
int leftForward = 3;
int leftBackward = 5;
int rightForward = 6;
int rightBackward = 9;
int rForward = true;
int lForward = true;
int canRun = false;
char serialData = '0';

void setup() {
  Serial.begin(9600); //Sets the data rate in bits per second (baud) for serial data transmission
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
}

void loop() {
  if(Serial.available() > 0)  // Send data only when you receive data:
  {
    serialData = Serial.read();      //Read the incoming data and store it into variable data
    if(serialData == 's')
    {
      canRun = false;
      Serial.print("Stopped\n");
    }
    else if(serialData == 'g')
    {
      canRun = true;
      Serial.print("Going\n");
    }
    else if(serialData == 'f')
    {
      rForward = true;
      lForward = true;
      Serial.print("Forward set\n");
    }
    else if(serialData == 'b')
    {
      rForward = false;
      lForward = false;
      Serial.print("Backward set\n");
    }
    else if(serialData == 'l')
    {
      rForward = true;
      lForward = false;
      Serial.print("Turning Left\n");
    }
    else if(serialData == 'r')
    {
      rForward = false;
      lForward = true;
      Serial.print("Turning Right\n");
    }
    else if(serialData == 'u')
    {
      String outputMessage;
      int speedPercentage;
      speedVal += increment;

      if (speedVal > maxSpeed)
      {
        speedVal = maxSpeed;
      }
      
      speedPercentage = (speedVal * 100) / maxSpeed;
      outputMessage = String("Speed set to ") + speedPercentage + String("%\n");
      Serial.print(outputMessage);
    }
    else if(serialData == 'd')
    {      
      String outputMessage;
      int speedPercentage;
      speedVal -= increment;

      if (speedVal < 0)
      {
        speedVal = 0;
      }
      
      speedPercentage = (speedVal * 100) / maxSpeed;
      outputMessage = String("Speed set to ") + speedPercentage + String("%\n");
      Serial.print(outputMessage);
    }
  }
  
  analogWrite(leftBackward, (!lForward * speedVal * canRun));
  analogWrite(rightBackward, (!rForward * speedVal * canRun));
  analogWrite(leftForward, (lForward * speedVal * canRun));
  analogWrite(rightForward, (rForward * speedVal * canRun));
  delay(10);
}
