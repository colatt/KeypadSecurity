#include <Arduino.h>
#include "esp_deep_sleep.h"
#include <Keypad.h>
#include <SimpleTimer.h>

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
char password[PASSWORD_LENGTH] = "1234"; 
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

void setup()
{
  Serial.begin(115200);

  timerId = timer.setInterval(5000, timerCallback);
  
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, HIGH);

  ledcAttachPin(LED_RED, CHANNEL_RED);
  ledcAttachPin(LED_GREEN, CHANNEL_GREEN);
  ledcAttachPin(LED_BLUE, CHANNEL_BLUE);
  
  ledcSetup(CHANNEL_RED, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(CHANNEL_GREEN, 12000, 8);
  ledcSetup(CHANNEL_BLUE, 12000, 8);
}

void clearEnteredValues()
{
  ledcWrite(CHANNEL_RED, 0);
  ledcWrite(CHANNEL_GREEN, 0);
  while(enteredValuesCount !=0)
  {
    enteredValues[enteredValuesCount--] = 0;
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
