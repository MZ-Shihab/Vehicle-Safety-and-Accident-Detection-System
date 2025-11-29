# 🚗 Vehicle Accident Detection & Alert System

This project is an IoT-based safety system that detects vehicle accidents using **ESP32 Wroom**, **MPU6050**, **GPS (Neo-6M)**, and **SIM800L**. When a crash or abnormal tilt is detected, the system instantly sends an SMS alert with the real-time GPS location and also triggers optional buzzer and display notifications. It additionally includes RFID-based SOS triggering and speed monitoring using an IR sensor.

---

## ⭐ Key Features

- Real-time accident detection using MPU6050 shock & rotation values  
- Automatic SMS alert with live GPS coordinates  
- Emergency call triggering via SIM800L  
- RFID card–based manual SOS signal  
- IR sensor–based speed monitoring  
- Live status display on 0.96" OLED  
- Motor control and speed feedback using IRFZ44N MOSFET & Potentiometer
- Compact, low-power ESP32-based system suitable for vehicles  

---

## 🛠 Hardware Components Used

- ESP32-Wroom  
- MPU6050 Accelerometer & Gyroscope  
- Neo-6M GPS module  
- SIM800L GSM module  
- RFID RC522  
- IRFZ44N MOSFET, 10K Potentiometer  
- TCRT5000 IR Speed Sensor  
- 0.96" OLED Display  
- Buzzer  
- Motors, wheels  
- Li-ion battery  

---

## 📡 System Overview

The ESP32 constantly monitors acceleration, rotation, and speed parameters. When values cross a defined threshold, the system assumes a crash and:

1. Gets real-time GPS coordinates  
2. Sends an SMS alert (including location link)  
3. Optionally activates a warning buzzer  
4. Logs detection information  
5. Displays status on OLED  

The RFID card can also be used to immediately trigger a manual SOS alert. The IR sensor measures vehicle speed and sends overspeed alerts through GSM.

---

## 📦 Software & Libraries

- EasyEDA 
- Arduino IDE 
- Wire.h  
- Adafruit_GFX.h  
- Adafruit_SSD1306.h
- TinyGPSPlus.h 
- MPU6050.h
- MFRC522.h  
- SPI.h
- WiFi.h
- HTTPClient.h
- ArduinoOTA.h

---

## 📑 Documentation Included

- Flowchart  
- Circuit diagram  
- Components list  
- Prototype images  
- Test output screenshots  
- Project overview report / presentation  

---

## ▶️ How to Run the Project

1. Upload the ESP32 main code from the `code` folder into the ESP32.
2. Insert your **SIM card** inside SIM800L.
3. Ensure the GPS antenna has a clear outdoor view for accurate location.
4. Power the board with 7.4V battery or regulated 5V supply.
5. Open Serial Monitor for live debugging.
6. Test accident simulation by applying a controlled jerk or tilt.
7. Check SMS output from SIM800L on your phone & Telegram

---

## 📲 SMS Alert Format

Example SMS sent during accident:

Accident Detected!
Location: 23.750912, 90.390133
Google Maps: https://maps.google.com/?q=23.750912,90.390133

Time: 14:32:10


---

## 📘 Applications

- Personal vehicle safety  
- Delivery vehicle tracking  
- School/college transport monitoring  
- Motorcycle accident response  
- Fleet management system  
- Emergency alert systems  

---

## 🚀 Future Enhancements

- Add camera module for accident image capture  
- Cloud dashboard for real-time monitoring  
- Smartphone app integration  
- Data logging on SD card  
- Improved power optimization  
- Vibration filter algorithm for false detection reduction  

---

## 📝 License

This project is open-source under the **MIT License**.

---

## 👤 Author

**MD. MYMO ZAMAN SHIHAB**  
Student & Embedded Systems Developer  
Bangladesh  
