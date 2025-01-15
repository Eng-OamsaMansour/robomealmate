
#include <Encoder.h>

// Motor Controllers Pins -->

// Motor A Controller -->
const int RPWM_A = 4;
const int LPWM_A = 5;
const int R_EN_A = 26;
const int L_EN_A = 27;

// Motor B Controller -->
const int RPWM_B = 6;
const int LPWM_B = 7;
const int R_EN_B = 28;
const int L_EN_B = 29;

// Motor C Controller -->
const int RPWM_C = 8;
const int LPWM_C = 9;
const int R_EN_C = 30;
const int L_EN_C = 31;

// Motor D Controller -->
const int RPWM_D = 10;
const int LPWM_D = 11;
const int R_EN_D = 32;
const int L_EN_D = 33;

// Encoders Pins -->

// Motor A Encoder -->
Encoder motorAEncoder(2, 22);

// Motor B Encoder -->
Encoder motorBEncoder(3, 23);

// Motor C Encoder
Encoder motorCEncoder(18, 24);

// Motor D Encoder
Encoder motorDEncoder(19, 25);


// Sensor Variables -->
long motorACount = 0;
long motorBCount = 0;
long motorCCount = 0;
long motorDCount = 0;
// serial baud -->
#define SERIAL_BAUD 9600 

void setup(){ 
    // Serial Communication -->
    Serial.begin(SERIAL_BAUD);
    Serial3.begin(SERIAL_BAUD);
    while (!Serial)
    {
    }
    Serial3.println("Arduino [INFO]:RoboMealMate Communication With the Brain Started.....");
    // Setting up controll pins
    // Motor A
    pinMode(RPWM_A, OUTPUT);
    pinMode(LPWM_A, OUTPUT);
    pinMode(R_EN_A, OUTPUT);
    pinMode(L_EN_A, OUTPUT);

    // Motor B
    pinMode(RPWM_B, OUTPUT);
    pinMode(LPWM_B, OUTPUT);
    pinMode(R_EN_B, OUTPUT);
    pinMode(L_EN_B, OUTPUT);

    // Motor C
    pinMode(RPWM_C, OUTPUT);
    pinMode(LPWM_C, OUTPUT);
    pinMode(R_EN_C, OUTPUT);
    pinMode(L_EN_C, OUTPUT);

    // Motor D
    pinMode(RPWM_D, OUTPUT);
    pinMode(LPWM_D, OUTPUT);
    pinMode(R_EN_D, OUTPUT);
    pinMode(L_EN_D, OUTPUT);

    // Enable Motors
    digitalWrite(R_EN_A, HIGH);
    digitalWrite(L_EN_A, HIGH);

    digitalWrite(R_EN_B, HIGH);
    digitalWrite(L_EN_B, HIGH);

    digitalWrite(R_EN_C, HIGH);
    digitalWrite(L_EN_C, HIGH);

    digitalWrite(R_EN_D, HIGH);
    digitalWrite(L_EN_D, HIGH);
}



void updateAllMotors(int speedA, int speedB, int speedC, int speedD) {
  setMotorOutput(RPWM_A, LPWM_A, speedA);
  setMotorOutput(RPWM_B, LPWM_B, speedB);
  setMotorOutput(RPWM_C, LPWM_C, speedC);
  setMotorOutput(RPWM_D, LPWM_D, speedD);
}

void setMotorOutput(int rpwmPin, int lpwmPin, int speedVal) {
  speedVal = constrain(speedVal, -255, 255);
  if (speedVal >= 0) {
    analogWrite(rpwmPin, speedVal);
    analogWrite(lpwmPin, 0);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, abs(speedVal));
  }
}

void handleIncomingCommands(HardwareSerial &serialPort, HardwareSerial &serialPort2) {
  while (serialPort.available() > 0) {

    String command = serialPort.readStringUntil('\r');

    serialPort2.println("Arduino[RECIVED]: Brain Sayes--> " + command);

    command.trim();

    if (command.length() == 0) return;

    char cmd = command.charAt(0);

    if(cmd == 'm'){
      command.remove(0, 2);
        int speedA, speedB, speedC, speedD;
        int parsed = sscanf(command.c_str(), "%d %d %d %d", &speedA, &speedB, &speedC, &speedD);
        if (parsed == 4) {
          updateAllMotors(speedA, speedB, speedC, speedD);
          serialPort2.print("Arduino[INFO]: Motor speeds set to: ");
          serialPort2.print(speedA); serialPort2.print(" ");
          serialPort2.print(speedB); serialPort2.print(" ");
          serialPort2.print(speedC); serialPort2.print(" ");
          serialPort2.println(speedD);
        } else {
          serialPort2.println("Arduino[WARN]: Invalid Command");
        }
    }else if(cmd == 'e'){
        motorACount = motorAEncoder.read();
        motorBCount = motorBEncoder.read();
        motorCCount = motorCEncoder.read();
        motorDCount = motorDEncoder.read();

        serialPort.print(": ");
        serialPort.print(motorACount); serialPort.print(" ");
        serialPort.print(motorBCount); serialPort.print(" ");
        serialPort.print(motorCCount); serialPort.print(" ");
        serialPort.println(motorDCount);
        serialPort2.print(": ");
        serialPort2.print(motorACount); serialPort2.print(" ");
        serialPort2.print(motorBCount); serialPort2.print(" ");
        serialPort2.print(motorCCount); serialPort2.print(" ");
        serialPort2.println(motorDCount);
    } else {
          serialPort2.println("Arduino[WARN]: Invalid Command");
      }
  }
}
void loop() {
  handleIncomingCommands(Serial,Serial3);
}