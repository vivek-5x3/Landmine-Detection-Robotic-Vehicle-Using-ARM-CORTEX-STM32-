// #include <Arduino.h>

// // Motor Driver (L293N) Pins
// #define IN1 PB0
// #define IN2 PB1
// #define IN3 PB10
// #define IN4 PB11

// // Metal Detector Sensor
// #define METAL_DETECTOR PC13

// // Buzzer
// #define BUZZER PA5

// // Bluetooth Module (HC-05) via USART1
// #define BT_TX PA9
// #define BT_RX PA10

// // GPS Module (Neo-6M) via USART2
// #define GPS_TX PA2
// #define GPS_RX PA3


// // Fix for Serial2
// HardwareSerial Serial2(USART2);

// bool metalDetected = false; // Flag to prevent multiple messages

// void setup()
// {
//   Serial1.begin(9600); // HC-05 Bluetooth
//   Serial2.begin(9600); // GPS Neo-6M

//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);
//   pinMode(IN3, OUTPUT);
//   pinMode(IN4, OUTPUT);
//   pinMode(BUZZER, OUTPUT);
//   pinMode(METAL_DETECTOR, INPUT);
// }


// void sendGPSData()
// {
//     Serial1.println("Reading GPS...");

//     String gpsData = "";
//     unsigned long startTime = millis();

//     while (millis() - startTime < 2000) // Wait for 2 seconds to receive GPS data
//     {
//         if (Serial2.available())
//         {
//             char c = Serial2.read();
//             gpsData += c;
//         }
//     }

//     if (gpsData.length() > 0)
//     {
//         Serial1.println("GPS Location: " + gpsData); // Send GPS data via Bluetooth
//     }
//     else
//     {
//         Serial1.println("GPS Data Not Available!");
//     }
// }
// void handleCommand(char command)
// {
//   switch (command)
//   {
//   case 'F': // Move Forward
//     digitalWrite(IN1, HIGH);
//     digitalWrite(IN2, LOW);
//     digitalWrite(IN3, HIGH);
//     digitalWrite(IN4, LOW);
//     break;
//   case 'B': // Move Backward
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, HIGH);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, HIGH);
//     break;
//   case 'R': // Turn right
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, HIGH);
//     digitalWrite(IN3, HIGH);
//     digitalWrite(IN4, LOW);
//     break;
//   case 'L': // Turn left
//     digitalWrite(IN1, HIGH);
//     digitalWrite(IN2, LOW);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, HIGH);
//     break;
//   case 'S': // Stop
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, LOW);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, LOW);
//     break;
//   case 'G': // Send GPS Coordinates
//     sendGPSData();
//     break;
//   default:
//     Serial1.println("Invalid Command");
//     break;
//   }
// }

// void loop()
// {
//   // Check for Bluetooth commands from mobile
//   if (Serial1.available())
//   {
//     char command = Serial1.read();
//     handleCommand(command);
//   }

//   // Check Metal Detector Sensor
//   if (digitalRead(METAL_DETECTOR) == HIGH && !metalDetected)
//   {
//     metalDetected = true;                      // Prevent multiple alerts
//     Serial1.println("ALERT: Metal Detected!"); // Send message to mobile app


//     sendGPSData(); // Send GPS coordinates when metal is detected
//   }

//   // Reset flag if metal is removed
//   if (digitalRead(METAL_DETECTOR) == LOW)
//   {
//     metalDetected = false;
//   }
// }



// #include <Arduino.h>

// // Motor Driver (L293N) Pins
// #define IN1 PB0
// #define IN2 PB1
// #define IN3 PB10
// #define IN4 PB11
// #define ENA PA6  // PWM for Motor A
// #define ENB PA7  // PWM for Motor B

// // Metal Detector Sensor
// #define METAL_DETECTOR PC13

// // Buzzer
// #define BUZZER PA5

// // Bluetooth Module (HC-05) via USART1
// #define BT_TX PA9
// #define BT_RX PA10

// // GPS Module (Neo-6M) via USART2
// #define GPS_TX PA2
// #define GPS_RX PA3

// // Fix for Serial2
// HardwareSerial Serial2(USART2);

// bool metalDetected = false; // Flag to prevent multiple messages
// int motorSpeed = 255; // Default full speed (0-255)

// void setup()
// {
//   Serial1.begin(9600); // HC-05 Bluetooth
//   Serial2.begin(9600); // GPS Neo-6M

//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);
//   pinMode(IN3, OUTPUT);
//   pinMode(IN4, OUTPUT);
//   pinMode(ENA, OUTPUT);
//   pinMode(ENB, OUTPUT);
//   pinMode(BUZZER, OUTPUT);
//   pinMode(METAL_DETECTOR, INPUT);

//   analogWrite(ENA, motorSpeed); // Set initial speed
//   analogWrite(ENB, motorSpeed);
// }

// void sendGPSData()
// {
//     Serial1.println("Reading GPS...");

//     String gpsData = "";
//     unsigned long startTime = millis();

//     while (millis() - startTime < 2000) // Wait for 2 seconds to receive GPS data
//     {
//         if (Serial2.available())
//         {
//             char c = Serial2.read();
//             gpsData += c;
//         }
//     }

//     if (gpsData.length() > 0)
//     {
//         Serial1.println("GPS Location: " + gpsData); // Send GPS data via Bluetooth
//     }
//     else
//     {
//         Serial1.println("GPS Data Not Available!");
//     }
// }

// void handleCommand(char command)
// {
//   switch (command)
//   {
//   case 'F': // Move Forward
//     digitalWrite(IN1, HIGH);
//     digitalWrite(IN2, LOW);
//     digitalWrite(IN3, HIGH);
//     digitalWrite(IN4, LOW);
//     break;
//   case 'B': // Move Backward
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, HIGH);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, HIGH);
//     break;
//   case 'R': // Turn right
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, HIGH);
//     digitalWrite(IN3, HIGH);
//     digitalWrite(IN4, LOW);
//     break;
//   case 'L': // Turn left
//     digitalWrite(IN1, HIGH);
//     digitalWrite(IN2, LOW);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, HIGH);
//     break;
//   case 'S': // Stop
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, LOW);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, LOW);
//     break;
//   case 'G': // Send GPS Coordinates
//     sendGPSData();
//     break;
//   case '1': // Set Speed to 50%
//     motorSpeed = 128;
//     analogWrite(ENA, motorSpeed);
//     analogWrite(ENB, motorSpeed);
//     Serial1.println("Speed: 50%");
//     break;
//   case '2': // Set Speed to 75%
//     motorSpeed = 192;
//     analogWrite(ENA, motorSpeed);
//     analogWrite(ENB, motorSpeed);
//     Serial1.println("Speed: 75%");
//     break;
//   case '3': // Set Speed to 100%
//     motorSpeed = 255;
//     analogWrite(ENA, motorSpeed);
//     analogWrite(ENB, motorSpeed);
//     Serial1.println("Speed: 100%");
//     break;
//   default:
//     Serial1.println("Invalid Command");
//     break;
//   }
// }

// void loop()
// {
//   // Check for Bluetooth commands from mobile
//   if (Serial1.available())
//   {
//     char command = Serial1.read();
//     handleCommand(command);
//   }

//   // Check Metal Detector Sensor
//   if (digitalRead(METAL_DETECTOR) == HIGH && !metalDetected)
//   {
//     metalDetected = true;                      // Prevent multiple alerts
//     Serial1.println("ALERT: Metal Detected!"); // Send message to mobile app

//     sendGPSData(); // Send GPS coordinates when metal is detected
//   }

//   // Reset flag if metal is removed
//   if (digitalRead(METAL_DETECTOR) == LOW)
//   {
//     metalDetected = false;
//   }
// }


#include <Arduino.h>

// Motor Driver (L293N) Pins
#define IN1 PB0
#define IN2 PB1
#define IN3 PB10
#define IN4 PB11
#define ENA PA6  // PWM for Motor A
#define ENB PA7  // PWM for Motor B

// Metal Detector Sensor
#define METAL_DETECTOR PC13

// Buzzer
#define BUZZER PA5

// Bluetooth Module (HC-05) via USART1
#define BT_TX PA9
#define BT_RX PA10

// GPS Module (Neo-6M) via USART2
#define GPS_TX PA2
#define GPS_RX PA3

// Fix for Serial2
HardwareSerial Serial2(USART2);

bool metalDetected = false; // Flag to prevent multiple messages
int motorSpeed = 128; // Default speed set to 50% (0-255)

void setup()
{
  Serial1.begin(9600); // HC-05 Bluetooth
  Serial2.begin(9600); // GPS Neo-6M

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(METAL_DETECTOR, INPUT);

  analogWrite(ENA, motorSpeed); // Set initial speed to 50%
  analogWrite(ENB, motorSpeed);
}

// void sendGPSData()
// {
//     Serial1.println("Reading GPS...");

//     String gpsData = "";
//     unsigned long startTime = millis();

//     while (millis() - startTime < 2000) // Wait for 2 seconds to receive GPS data
//     {
//         if (Serial2.available())
//         {
//             char c = Serial2.read();
//             gpsData += c;
//         }
//     }

//     if (gpsData.length() > 0)
//     {
//         Serial1.println("GPS Location: " + gpsData); // Send GPS data via Bluetooth
//     }
//     else
//     {
//         Serial1.println("GPS Data Not Available!");
//     }
// }

// Function to extract latitude and longitude from GPS data
void sendGPSData()
{
  Serial1.println("Reading GPS...");

  String gpsData = "";
  unsigned long startTime = millis();

  while (millis() - startTime < 2000) // Wait for 2 seconds to receive GPS data
  {
    if (Serial2.available())
    {
      char c = Serial2.read();
      gpsData += c;
    }
  }

  if (gpsData.length() > 0)
  {
    String latitude = "";
    String longitude = "";

    // Look for GPGGA or GPRMC sentences
    int gpggaIndex = gpsData.indexOf("$GPGGA");
    int gprmcIndex = gpsData.indexOf("$GPRMC");

    if (gpggaIndex != -1)
    {
      int start = gpggaIndex;
      int commaCount = 0;
      for (int i = start; i < gpsData.length(); i++)
      {
        if (gpsData[i] == ',')
        {
          commaCount++;
          if (commaCount == 2) // Latitude starts after 2nd comma
          {
            int latStart = i + 1;
            int latEnd = gpsData.indexOf(',', latStart);
            latitude = gpsData.substring(latStart, latEnd);

            int lonStart = latEnd + 2; // Skip 'N' or 'S' identifier
            int lonEnd = gpsData.indexOf(',', lonStart);
            longitude = gpsData.substring(lonStart, lonEnd);
            break;
          }
        }
      }
    }
    else if (gprmcIndex != -1)
    {
      int start = gprmcIndex;
      int commaCount = 0;
      for (int i = start; i < gpsData.length(); i++)
      {
        if (gpsData[i] == ',')
        {
          commaCount++;
          if (commaCount == 3) // Latitude starts after 3rd comma
          {
            int latStart = i + 1;
            int latEnd = gpsData.indexOf(',', latStart);
            latitude = gpsData.substring(latStart, latEnd);

            int lonStart = latEnd + 2; // Skip 'N' or 'S' identifier
            int lonEnd = gpsData.indexOf(',', lonStart);
            longitude = gpsData.substring(lonStart, lonEnd);
            break;
          }
        }
      }
    }

    if (latitude != "" && longitude != "")
    {
      Serial1.println("Latitude: " + latitude + ", Longitude: " + longitude);
    }
    else
    {
      Serial1.println("GPS Data Not Available!");
    }
  }
  else
  {
    Serial1.println("GPS Data Not Available!");
  }
}

void handleCommand(char command)
{
  switch (command)
  {
  case 'F': // Move Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    break;
  case 'B': // Move Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    break;
  case 'R': // Turn right
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    break;
  case 'L': // Turn left
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    break;
  case 'S': // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    break;
  case 'G': // Send GPS Coordinates
    sendGPSData();
    break;
  case '1': // Set Speed to 50%
    motorSpeed = 128;
    analogWrite(ENA, motorSpeed);
    analogWrite(ENB, motorSpeed);
    Serial1.println("Speed: 50%");
    break;
  case '2': // Set Speed to 75%
    motorSpeed = 192;
    analogWrite(ENA, motorSpeed);
    analogWrite(ENB, motorSpeed);
    Serial1.println("Speed: 75%");
    break;
  case '3': // Set Speed to 100%
    motorSpeed = 255;
    analogWrite(ENA, motorSpeed);
    analogWrite(ENB, motorSpeed);
    Serial1.println("Speed: 100%");
    break;
  default:
    Serial1.println("Invalid Command");
    break;
  }
}

void loop()
{
  // Check for Bluetooth commands from mobile
  if (Serial1.available())
  {
    char command = Serial1.read();
    handleCommand(command);
  }

  // Check Metal Detector Sensor
  if (digitalRead(METAL_DETECTOR) == HIGH && !metalDetected)
  {
    metalDetected = true;                      // Prevent multiple alerts
    Serial1.println("ALERT: Metal Detected!"); // Send message to mobile app

    sendGPSData(); // Send GPS coordinates when metal is detected
  }

  // Reset flag if metal is removed
  if (digitalRead(METAL_DETECTOR) == LOW)
  {
    metalDetected = false;
  }
}
