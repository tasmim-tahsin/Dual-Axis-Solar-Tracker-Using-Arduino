# **Dual-Axis Solar Tracking System Using Arduino**  

## **Project Overview**  
This project is a **Dual-Axis Solar Tracking System** that optimizes solar energy capture by dynamically adjusting the position of a solar panel using **servo motors** and **light-dependent resistors (LDRs)**. The system also monitors battery charging status and displays real-time voltage and battery percentage on an **I2C LCD display**.

## **Features**  
✔️ **Dual-Axis Solar Tracking**: Uses LDRs to detect sunlight direction and adjust the panel accordingly.  
✔️ **Battery Monitoring System**: Displays real-time voltage and battery percentage.  
✔️ **LED Status Indicators**:  
   - Charging LED (ON when charging)  
   - Tracking LED (ON when panel is adjusting)  
   - Full-Charge LED (ON when battery is fully charged)  
✔️ **Buzzer Alert**: Signals when the battery is fully charged.  
✔️ **Efficient Power Management**: Ensures optimal use of solar energy.  

---

## **Hardware Requirements**  
- **Arduino Uno**  
- **4 x Light Dependent Resistors (LDRs)**  
- **2 x Servo Motors (for dual-axis control)**  
- **16x2 I2C LCD Display**  
- **3.7V Li-Ion Battery**  
- **Voltage Sensor**  
- **Buzzer**  
- **3 LEDs (Charging, Tracking, Full Charge)**  
- **Resistors and Connecting Wires**  

---

## **Software Requirements**  
- **Arduino IDE**  
- **Libraries Used:**  
  - `Servo.h` (for motor control)  
  - `Wire.h` (for I2C communication)  
  - `LiquidCrystal_I2C.h` (for LCD display)  

---

## **Circuit Diagram**  
![Image](https://github.com/user-attachments/assets/81a63f88-20b5-4a66-aadd-b6a73a637512)

---

## **Installation & Usage**  

1️⃣ **Install Arduino IDE** from [official website](https://www.arduino.cc/en/software).  
2️⃣ **Install Required Libraries** via Arduino Library Manager.  
3️⃣ **Connect the Components** as per the circuit diagram.  
4️⃣ **Upload the Code** to Arduino Uno.  
5️⃣ **Observe the LCD Display** and LED indicators for real-time tracking and battery status.  

---

## **Code**  
```cpp
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

Servo horizontal;
int servohori = 180; 
int servohoriLimitHigh = 175;
int servohoriLimitLow = 5;

Servo vertical;
int servovert = 45; 
int servovertLimitHigh = 120;
int servovertLimitLow = 1;

int ldrlt = A0;
int ldrrt = A3;
int ldrld = A1;
int ldrrd = A2;

const int voltageSensorPin = 8;

float adc_voltage = 0.0;
float in_voltage = 0.0;

float R1 = 30000.0;
float R2 = 7500.0; 

float ref_voltage = 5.0;

int adc_value = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);

const float minVoltage = 3.0;
const float maxVoltage = 4.2;

int batteryPercentage;

const int LED_Charging = 4;
const int LED_Tracking = 5;
const int LED_FullCharge = 6;
const int Buzzer = 7;

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.clear();

  pinMode(LED_Charging, OUTPUT);
  pinMode(LED_Tracking, OUTPUT);
  pinMode(LED_FullCharge, OUTPUT);
  pinMode(Buzzer, OUTPUT);

  horizontal.attach(9);
  vertical.attach(10);
  horizontal.write(180);
  vertical.write(45);
  delay(500);

  Serial.begin(9600);
}

void loop() {
  lcd.setCursor(0, 0);
  lcd.print("Solar Tracking");

  adc_value = analogRead(voltageSensorPin);
  adc_voltage  = (adc_value * ref_voltage) / 1024.0;
  in_voltage = adc_voltage*(R1+R2)/R2;
  batteryPercentage = map(in_voltage * 1000, minVoltage * 1000, maxVoltage * 1000, 0, 100);

  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(in_voltage, 1);
  lcd.print("V   ");
  lcd.print("B:");
  lcd.print(batteryPercentage);

  if (batteryPercentage > 100) {
    digitalWrite(LED_FullCharge, HIGH);
    digitalWrite(LED_Charging, LOW);
    digitalWrite(LED_Tracking, HIGH);
  } 
  else if (batteryPercentage < 100){
    digitalWrite(LED_Charging, HIGH);
    digitalWrite(LED_FullCharge, LOW);
    digitalWrite(LED_Tracking, HIGH);
    noTone(Buzzer);
  }
  else {
    digitalWrite(LED_Tracking, HIGH);
    digitalWrite(LED_Charging, LOW);
    digitalWrite(LED_FullCharge, LOW);
    noTone(Buzzer);
  }

  int lt = analogRead(ldrlt);
  int rt = analogRead(ldrrt);
  int ld = analogRead(ldrld);
  int rd = analogRead(ldrrd);

  int dtime = 1; 
  int tol = 90;

  int avt = (lt + rt) / 2;
  int avd = (ld + rd) / 2;
  int avl = (lt + ld) / 2;
  int avr = (rt + rd) / 2;

  int dvert = avt - avd;
  int dhoriz = avl - avr;

  if (abs(dvert) > tol) { 
    if (avt > avd) {
      servovert = ++servovert;
      if (servovert > servovertLimitHigh) servovert = servovertLimitHigh;
    } else {
      servovert = --servovert;
      if (servovert < servovertLimitLow) servovert = servovertLimitLow;
    }
    vertical.write(servovert);
  }

  if (abs(dhoriz) > tol) { 
    if (avl > avr) {
      servohori = --servohori;
      if (servohori < servohoriLimitLow) servohori = servohoriLimitLow;
    } else {
      servohori = ++servohori;
      if (servohori > servohoriLimitHigh) servohori = servohoriLimitHigh;
    }
    horizontal.write(servohori);
  }

  delay(dtime);
}
```

---

## **Project Demonstration**  

Google Drive Link for video demonstration: https://drive.google.com/drive/folders/1G3n3rvxma3F90vM00QJjmqxbYikTeg4V?usp=sharing
![Image](https://github.com/user-attachments/assets/56c0cc42-eea6-4761-9147-f0092b26a4b4)


---

## **Future Enhancements**  
🔹 **Improve Efficiency** with advanced MPPT algorithms.  
🔹 **Expand for Large-Scale Solar Panels**.  

---

## **Contributors**  
- **Md Tasmim Al Tahsin**  
- **Md Mansur Islam**
- **Tonima Islam Dristy**  
- **Intesar Ahmed Siyan**
- **Rumaiya Islam**

---

## **License**  
This project is licensed under the **MIT License** – feel free to use and modify it!  

---
