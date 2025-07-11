#include <AccelStepper.h>
#include "Adafruit_VL53L0X.h"
#include <WiFiS3.h>
#include <WiFiServer.h>

// Configuração do WiFi Access Point
char ssid[] = "Scanner3D";    
char pass[] = "scanner3d";       
int status = WL_IDLE_STATUS;
WiFiServer server(80);
WiFiClient client;
WiFiClient scanClient;  // Cliente dedicado para o scan

// Modos de operação
bool calibrationMode = false;
bool scanMode = false;
bool cancelRequested = false;
String fileType = "";

// Motores - configuração para AccelStepper
const int stepsPerRevolution = 2048;
const int baseMotor_pin1 = 2;
const int baseMotor_pin2 = 3;
const int baseMotor_pin3 = 12;
const int baseMotor_pin4 = 13;
const int zAxisMotor_pin1 = A0;
const int zAxisMotor_pin2 = A1;
const int zAxisMotor_pin3 = A2;
const int zAxisMotor_pin4 = A3;

AccelStepper baseMotor(AccelStepper::FULL4WIRE, baseMotor_pin1, baseMotor_pin2, baseMotor_pin3, baseMotor_pin4);
AccelStepper zAxisMotor(AccelStepper::FULL4WIRE, zAxisMotor_pin1, zAxisMotor_pin2, zAxisMotor_pin3, zAxisMotor_pin4);

// Sensor VL53L0X
Adafruit_VL53L0X lox;
const float sensorOffset = 10.1; //////////// real é 10.1

const int Z_LIMIT_SWITCH_PIN = 4;

// Parâmetros
bool scanRapido = false;
float currentZ = 0;
float userDefinedHeight = 26.0; 
const float layerHeight = 0.15;
const int stepsPerLayer = stepsPerRevolution; // 2048 steps = 1.5mm
const int baseStepIncrement = 2;
float currentAngle = 0;
const int measurementsPerPoint = 1;

const int POST_ROTATION_DELAY = 300;
const int POST_MEASUREMENT_DELAY = 100;
const int POST_LAYER_DELAY = 1000;
const int BETWEEN_READINGS_DELAY = 100;

volatile bool emergencyStop = false;

void setup() {
  Serial.begin(115200);

  baseMotor.setMaxSpeed(600);
  baseMotor.setSpeed(60);
  baseMotor.setAcceleration(100);
  
  zAxisMotor.setMaxSpeed(100);
  zAxisMotor.setSpeed(10);
  zAxisMotor.setAcceleration(50);

  pinMode(baseMotor_pin1, OUTPUT);
  pinMode(baseMotor_pin2, OUTPUT);
  pinMode(baseMotor_pin3, OUTPUT);
  pinMode(baseMotor_pin4, OUTPUT);
  pinMode(zAxisMotor_pin1, OUTPUT);
  pinMode(zAxisMotor_pin2, OUTPUT);
  pinMode(zAxisMotor_pin3, OUTPUT);
  pinMode(zAxisMotor_pin4, OUTPUT);
  pinMode(Z_LIMIT_SWITCH_PIN, INPUT_PULLUP);

  if (!lox.begin()) {
    Serial.println("ERRO: Sensor VL53L0X!");
  } else {
    lox.setMeasurementTimingBudgetMicroSeconds(200000); // 200 ms para alta precisão
    lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
  }

  status = WiFi.beginAP(ssid, pass);

  if (status != WL_AP_LISTENING) {
    Serial.println("Erro ao iniciar AP");
    while (true);
  }

  delay(1000);

  IPAddress ip = WiFi.localIP();
  Serial.println(ip);

  server.begin();
}

void loop() {
  if (!scanMode) {
    checkForCommands();
  }
  
  if (emergencyStop) {
    forceStopMotors();
    scanMode = false;
    emergencyStop = false;
    if (scanClient && scanClient.connected()) {
      scanClient.println("SCAN_CANCELADO");
      scanClient.stop();
    }
    return;
  }
  
  if (calibrationMode) {
    calibrate();
    calibrationMode = false;
    if (client && client.connected()) {
      client.println("Calibração concluída");
      client.stop();
    }
  }

  if (scanMode) {
    performScan();
    scanMode = false;
    calibrate();
  }

  delay(10);
}

void checkForCommands() {
  WiFiClient newClient = server.available();

  if (newClient) {
    if (newClient.available()) {
      String request = newClient.readStringUntil('\r');
      newClient.flush();

      Serial.print("Comando recebido: ");
      Serial.println(request);

      if (request == "calibrar") {
        client = newClient;
        calibrationMode = true;
      }
      else if (request.startsWith("iniciar_txt|")) {
        int separatorPos = request.indexOf('|');
        if (separatorPos != -1) {
          String heightStr = request.substring(separatorPos + 1);
          userDefinedHeight = heightStr.toFloat();
          
          scanClient = newClient;  
          scanMode = true;
          scanRapido = false;
          fileType = "txt";
          cancelRequested = false;
          emergencyStop = false;
        }
      }
      else if (request.startsWith("iniciar_stl|")) {
        int separatorPos = request.indexOf('|');
        if (separatorPos != -1) {
          String heightStr = request.substring(separatorPos + 1);
          userDefinedHeight = heightStr.toFloat();
          
          scanClient = newClient;
          scanMode = true;
          scanRapido = false;
          fileType = "stl";
          cancelRequested = false;
          emergencyStop = false;
        }
      }
      else if (request.startsWith("iniciar_txt_fast|")) {
        int separatorPos = request.indexOf('|');
        if (separatorPos != -1) {
          String heightStr = request.substring(separatorPos + 1);
          userDefinedHeight = heightStr.toFloat();
          
          scanClient = newClient;
          scanMode = true;
          scanRapido = true;
          fileType = "txt";
          cancelRequested = false;
          emergencyStop = false;
        }
      }
      else if (request.startsWith("iniciar_stl_fast|")) {
        int separatorPos = request.indexOf('|');
        if (separatorPos != -1) {
          String heightStr = request.substring(separatorPos + 1);
          userDefinedHeight = heightStr.toFloat();
          
          scanClient = newClient;
          scanMode = true;
          scanRapido = true;
          fileType = "stl";
          cancelRequested = false;
          emergencyStop = false;
        }
      }
      else if (request == "cancelar") {
        emergencyStop = true;
        if (newClient.connected()) {
          newClient.println("COMANDO_CANCELAR_RECEBIDO");
          newClient.stop();
        }
      }
      else {
        if (newClient.connected()) {
          newClient.println("ERRO:comando_invalido");
          newClient.stop();
        }
      }
    }
  }
}

void checkForCancelDuringScan() {
  WiFiClient cancelClient = server.available();
  if (cancelClient && cancelClient.available()) {
    String request = cancelClient.readStringUntil('\r');
    cancelClient.flush();
    
    if (request == "cancelar") {
      emergencyStop = true;
      if (cancelClient.connected()) {
        cancelClient.println("COMANDO_CANCELAR_RECEBIDO");
        cancelClient.stop();
      }
    } else {
      cancelClient.stop();
    }
  }
}

void forceStopMotors() {
  
  baseMotor.stop();  
  zAxisMotor.stop();
  
  digitalWrite(baseMotor_pin1, LOW);
  digitalWrite(baseMotor_pin2, LOW);
  digitalWrite(baseMotor_pin3, LOW);
  digitalWrite(baseMotor_pin4, LOW);
  
  digitalWrite(zAxisMotor_pin1, LOW);
  digitalWrite(zAxisMotor_pin2, LOW);
  digitalWrite(zAxisMotor_pin3, LOW);
  digitalWrite(zAxisMotor_pin4, LOW);

  baseMotor.disableOutputs();
  zAxisMotor.disableOutputs();
  
  Serial.println("Motores parados e desligados!");
}

void calibrate() {
  // Configuração
  pinMode(Z_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  int extraCalibrationSteps = 6000;
  zAxisMotor.enableOutputs();
  zAxisMotor.setMaxSpeed(650);
  zAxisMotor.setAcceleration(300);
  
  if (client && client.connected()) {
    client.println(F("CALIBRACAO_INICIADA"));
    client.flush();
  }
  
  if (digitalRead(Z_LIMIT_SWITCH_PIN) == LOW) {  
    while (digitalRead(Z_LIMIT_SWITCH_PIN) == LOW) {
      zAxisMotor.setSpeed(-500);  
      zAxisMotor.runSpeed();
    }
  }
  
  while (digitalRead(Z_LIMIT_SWITCH_PIN) == HIGH) {
    zAxisMotor.setSpeed(+600); 
    zAxisMotor.runSpeed();
  }
  zAxisMotor.stop();
  delay(200);  

  zAxisMotor.move(+extraCalibrationSteps); 
  while (zAxisMotor.distanceToGo() != 0) {
    zAxisMotor.run();
  }
  zAxisMotor.stop();
  
  zAxisMotor.setCurrentPosition(0);
  currentZ = 0.0;
  
  desligarMotores();
  if (client && client.connected()) {
    client.println(F("CALIBRACAO_CONCLUIDA"));
  }
}

void performScan() {
  currentZ = 0;
  cancelRequested = false;
  currentAngle = 0; 

  if (scanClient && scanClient.connected()) {
    scanClient.println("Starting Scan");
    scanClient.println("Altura máxima definida: " + String(userDefinedHeight) + " cm");
  }

  while (currentZ < userDefinedHeight && !emergencyStop) {
    checkForCancelDuringScan(); 
    if (emergencyStop) return;
    
    int passo = scanRapido ? baseStepIncrement * 20 : baseStepIncrement;
    int i = 0;

    while (i < stepsPerRevolution && !emergencyStop) {
      baseMotor.move(passo);
      
      while (baseMotor.distanceToGo() != 0 && !emergencyStop) {
        baseMotor.run();
        static int motorCounter = 0;
        if (++motorCounter >= 10) {  
          motorCounter = 0;
          checkForCancelDuringScan();
        }
      }
      
      if (emergencyStop) return;
      
      i += passo;
      delay(scanRapido ? POST_ROTATION_DELAY/2 : POST_ROTATION_DELAY);

      currentAngle += (360.0 * passo / stepsPerRevolution);
      if (currentAngle >= 360.0) currentAngle -= 360.0;

      checkForCancelDuringScan();
      if (emergencyStop) return;

      float distance = getDistance();
      
      checkForCancelDuringScan();
      if (emergencyStop) return;

      if (distance > 0) {
        String rawData = "RAW_DATA|" + String(currentAngle, 3) + "|" + 
                         String(distance, 3) + "|" + String(currentZ, 3) + 
                         "|" + String(sensorOffset, 3);
        
        Serial.println(rawData);
        
        if (scanClient && scanClient.connected()) {
          scanClient.println(rawData);
          scanClient.flush();  
        }
      }

      delay(scanRapido ? POST_MEASUREMENT_DELAY/2 : POST_MEASUREMENT_DELAY);
    }

    checkForCancelDuringScan();
    if (emergencyStop) return;

    float nextZ = currentZ + (scanRapido ? layerHeight * 2 : layerHeight);
    if (nextZ >= userDefinedHeight) break;
    
    moveZAxisUp();
    currentZ = nextZ;

    desligarMotores();
    delay(POST_LAYER_DELAY);
  }

  if (scanClient && scanClient.connected()) {
    scanClient.println("SCAN_COMPLETE");
    scanClient.println("ALTURA_FINAL: " + String(currentZ));
    scanClient.stop();
  }
}

float getDistance() {
  float sum = 0;
  int valid = 0;
  int totalReadings = 5;
  
  Serial.println("--- DEBUG SENSOR ---");
  Serial.print("Posição: Ângulo=");
  Serial.print(currentAngle);
  Serial.print("°, Z=");
  Serial.print(currentZ);
  Serial.println(" cm");
  
  for (int i = 0; i < totalReadings && !emergencyStop; i++) {
    if (i > 0) checkForCancelDuringScan();
    if (emergencyStop) return -1;
    
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    
    Serial.print("Leitura #");
    Serial.print(i+1);
    Serial.print(" Status=");
    Serial.print(measure.RangeStatus);
    
    if (measure.RangeStatus != 4) { 
      float cm = (measure.RangeMilliMeter) / 10.0;
      Serial.print(", Dist=");
      Serial.print(cm);
      Serial.println(" cm (válida)");
      
    
      if (cm > 0 && cm < 30) {
        sum += cm;
        valid++;
      }
    } else {
      Serial.println(" (inválida)");
    }
    
    delay(50); 
  }
  
  float finalDistance = -1;
  
  if (valid > 0) {
    finalDistance = sum / valid;
  }
  
  Serial.print("RESULTADO: ");
  Serial.print(valid);
  Serial.print("/");
  Serial.print(totalReadings);
  Serial.print(" válidas, média=");
  Serial.print(finalDistance);
  Serial.println("cm");
  
  return finalDistance;
}

void moveZAxisUp() {
  Serial.println("Subindo eixo Z...");
  
  int zSteps = scanRapido ? stepsPerLayer * 2 : stepsPerLayer;
  zAxisMotor.move(zSteps);
  
  while (zAxisMotor.distanceToGo() != 0 && !emergencyStop) {
    zAxisMotor.run();
    
    static int counter = 0;
    if (++counter >= 5) {
      counter = 0;
      checkForCancelDuringScan();
    }
  }
  
  Serial.println("Eixo Z em nova posição");
}

void desligarMotores() {
  baseMotor.disableOutputs();
  zAxisMotor.disableOutputs();
}


