#include <Arduino.h>
#include "esp_deep_sleep.h"
#include "EEPROM.h"
#include <Keypad.h>
#include <SimpleTimer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

const byte NUMBER_OF_ROWS = 4;
const byte NUMBER_OF_COLUMNS = 4; 
//define the symbols on the buttons of the keypad
char hexaKeys[NUMBER_OF_ROWS][NUMBER_OF_COLUMNS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[NUMBER_OF_ROWS] = {GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26};
byte colPins[NUMBER_OF_COLUMNS] = {GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13};

const byte PASSWORD_LENGTH = 5;
char enteredValues[PASSWORD_LENGTH]; 
char password[PASSWORD_LENGTH]; 
byte enteredValuesCount = 0;

const int ONBOARD_LED = GPIO_NUM_2;
const int LED_RED = GPIO_NUM_19;
const int LED_GREEN = GPIO_NUM_18;
const int LED_BLUE = GPIO_NUM_5; 
const int CHANNEL_RED = 1;
const int CHANNEL_GREEN = 2;
const int CHANNEL_BLUE = 3;
const int MAX_BRIGHTNESS = 255;

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, NUMBER_OF_ROWS, NUMBER_OF_COLUMNS);
SimpleTimer timer;
int timerId;

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      ledcWrite(CHANNEL_BLUE, MAX_BRIGHTNESS);
      timer.disable(timerId);
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      ledcWrite(CHANNEL_BLUE, 0);
      timer.enable(timerId);
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0 && rxValue.length() == 4) {
        Serial.println("*********");
        Serial.print("New Password: ");

        strcpy(password, rxValue.c_str());
        Serial.println(password);      
        EEPROM.put(0, password);
        EEPROM.commit();
      }
    }
};

void timerCallback() {
  // Turn off all columns and pull all rows high
  byte i;
  for (i = 0; i < NUMBER_OF_COLUMNS; i++)
  {
    pinMode (colPins[i], OUTPUT);
    digitalWrite (colPins[i], LOW);
  } 
  for (i = 0; i < NUMBER_OF_ROWS; i++)
  {
    pinMode (rowPins[i], INPUT_PULLUP);
  }

  // Check to make sure no pins are currently being pressed
  for (i = 0; i < NUMBER_OF_ROWS; i++)
  {
    if (digitalRead (rowPins[i]) == LOW)
    {
      Serial.println("button pressed");
      return; 
    }
  }
  // Sleep quickly to account for any button debouncing
  delay (50);

  digitalWrite(ONBOARD_LED, LOW);

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_deep_sleep_enable_ext1_wakeup(BIT(GPIO_NUM_27) | BIT(GPIO_NUM_14) | BIT(GPIO_NUM_12)| BIT(GPIO_NUM_13), ESP_EXT1_WAKEUP_ANY_HIGH);

  esp_deep_sleep_start();
}

int addr = 0;
#define EEPROM_SIZE 64

void setup()
{
  Serial.begin(115200);

  timerId = timer.setInterval(20000, timerCallback);
  
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, HIGH);

  ledcAttachPin(LED_RED, CHANNEL_RED);
  ledcAttachPin(LED_GREEN, CHANNEL_GREEN);
  ledcAttachPin(LED_BLUE, CHANNEL_BLUE);
  
  ledcSetup(CHANNEL_RED, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(CHANNEL_GREEN, 12000, 8);
  ledcSetup(CHANNEL_BLUE, 12000, 8);

  BLEDevice::init("Keypad Security"); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("Failed to initialise EEPROM");
    ESP.restart();
  }
  
  //--if eeprom is empty add a default password
  if(byte(EEPROM.read(0)) == byte(255))
  {
    Serial.println("Storing default Password 1234 into eeprom");
    EEPROM.put(0, "1234");
    EEPROM.commit();
  }

  EEPROM.get(0, password);
  Serial.print("Current Password: ");
  Serial.println(password);
}

void clearEnteredValues()
{
  ledcWrite(CHANNEL_RED, 0);
  ledcWrite(CHANNEL_GREEN, 0);
  while(enteredValuesCount != 0)
  {
    enteredValuesCount--;
    enteredValues[enteredValuesCount] = 0;
  }
  if (deviceConnected) {
      Serial.println(enteredValues);
      pCharacteristic->setValue(enteredValues);
      pCharacteristic->notify();
    }
  return;
}

void loop()
{
  timer.run();

  char customKey = customKeypad.getKey();
  if (customKey)
  {
    timer.restartTimer(timerId);
    enteredValues[enteredValuesCount] = customKey;
    enteredValuesCount++;
    Serial.println(customKey);
    if (deviceConnected) {
      pCharacteristic->setValue(enteredValues);
      pCharacteristic->notify();
    }
  }
  if(enteredValuesCount == PASSWORD_LENGTH - 1)
  {
    if(!strcmp(enteredValues, password))
    {
      ledcWrite(CHANNEL_GREEN, MAX_BRIGHTNESS);
      delay(3000);
      clearEnteredValues();
    } 
    else
    {
      ledcWrite(CHANNEL_RED, MAX_BRIGHTNESS);
      delay(3000);
      clearEnteredValues();
    }
  }
}
