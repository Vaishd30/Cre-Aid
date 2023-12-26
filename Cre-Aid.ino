// Motor control pins
const int motorPwmPin = 5;                  // PWM pin for motor speed control
const int motorDir1Pin = 18;                 // H-Bridge input 1
const int motorDir2Pin = 19;                 // H-Bridge input 2
const int encoderChannelAPin = 2;            // Encoder channel A pin
const int encoderChannelBPin = 3;            // Encoder channel B pin

// PID controller variables
float pidProportionalGain = 1.0;             // PID gains
float pidIntegralGain = 0.1;
float pidDerivativeGain = 0.05;
float integralTerm = 0;
float derivativeTerm = 0;
float previousError = 0;
unsigned long previousTime = 0;

// Motor speed variables
int targetSpeed = 0;
volatile int encoderPulseCount = 0;

void setupMotor();
void processSerialCommand();
void runPIDControl();
int parseSpeed();
void setMotorDirection(bool forward);
void setMotorSpeed(int speed);
void stopMotor();
void handleEncoderInterrupt();
int calculateEncoderSpeed();

void setup() {
  setupMotor();
}

void loop() {
  processSerialCommand();
  runPIDControl();
}

void setupMotor() {
  pinMode(motorPwmPin, OUTPUT);
  pinMode(motorDir1Pin, OUTPUT);
  pinMode(motorDir2Pin, OUTPUT);
  pinMode(encoderChannelAPin, INPUT_PULLUP);
  pinMode(encoderChannelBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderChannelAPin), handleEncoderInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderChannelBPin), handleEncoderInterrupt, RISING);
  Serial.begin(9600);
  stopMotor();
}

void processSerialCommand() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case 'F':
        setMotorDirection(true);
        targetSpeed = parseSpeed();
        break;
      case 'B':
        setMotorDirection(false);
        targetSpeed = parseSpeed();
        break;
      case 'S':
        stopMotor();
        break;
      default:
        Serial.println("Invalid command");
    }
  }
}

void runPIDControl() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;
  int currentSpeed = calculateEncoderSpeed();
  float error = targetSpeed - currentSpeed;
  integralTerm += error * deltaTime;
  derivativeTerm = (error - previousError) / deltaTime;
  previousError = error;
  previousTime = currentTime;

  // PID output mapping and constraint
  int pwmValue = targetSpeed + pidProportionalGain * error + pidIntegralGain * integralTerm + pidDerivativeGain * derivativeTerm;
  pwmValue = constrain(pwmValue, 0, 255);
  setMotorSpeed(pwmValue);
}

int parseSpeed() {
  String speedString = Serial.readStringUntil('\n');
  return speedString.toInt();
}

void setMotorDirection(bool forward) {
  digitalWrite(motorDir1Pin, forward ? HIGH : LOW);
  digitalWrite(motorDir2Pin, forward ? LOW : HIGH);
}

void setMotorSpeed(int speed) {
  analogWrite(motorPwmPin, speed);
}

void stopMotor() {
  setMotorSpeed(0);
}

void handleEncoderInterrupt() {
  if (digitalRead(encoderChannelBPin) == HIGH) {
    encoderPulseCount++;
  }
 else {
    encoderPulseCount--;
  }
}

int calculateEncoderSpeed() {
  int currentCount = encoderPulseCount;
  int countsPerSecond = static_cast<int>(currentCount / (millis() - previousTime) * 1000.0);
  return countsPerSecond;
}

