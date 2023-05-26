#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include "AiEsp32RotaryEncoder.h"

Adafruit_MCP23X17 mcp;

// The component is called 74HC595. 
// But variables cannot start with a number :)
#define U74HC_SI       19
#define U74HC_RCK      21
#define U74HC_SCK      22
#define OutputEnabled  23

// I2C for the MCP23017
#define SDA            32
#define SCL            33

// Rotary encoder
#define ROTARY_ENCODER_A_PIN 16
#define ROTARY_ENCODER_B_PIN 18
#define ROTARY_ENCODER_BUTTON_PIN 17
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

// State
bool RelayStatus[8];
bool PreviousButtonStatus[8];

void IRAM_ATTR readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
}

void setup()
{
  Serial.begin(115200);
  serialPrintln("Initializing");

  pinMode(OutputEnabled, OUTPUT);
  digitalWrite(OutputEnabled, LOW);

  // Shift register
  pinMode(U74HC_SCK, OUTPUT);
  pinMode(U74HC_RCK, OUTPUT);
  pinMode(U74HC_SI, OUTPUT);

  digitalWrite(U74HC_SCK, LOW);
  digitalWrite(U74HC_RCK, LOW);
  digitalWrite(U74HC_SI, LOW);

  // I2C expander
  Wire.setClock(20000);
  Wire.begin(SDA, SCL); // wake up I2C bus
  try {
    if (!mcp.begin_I2C()) {
      serialPrintln("Error. I2C not initialized");
      //while (1);
    }
  }catch (String error){
    Serial.println(error);
  }

  for (int i = 0; i < 16; i++ ) {
    mcp.pinMode(i, INPUT);
  }

  // Before activating the relays, let's give some time to the input pins to estabilize
  serialPrintln("Delaying relays activation");

  delay(3000);
  digitalWrite(OutputEnabled, HIGH);

  serialPrintln("Relays activated!");

  // Rotary encoder
  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  //set boundaries and if values should cycle or not. 
  bool circleValues = false;
  int minValue = 0;
  int maxValue = 100;
  rotaryEncoder.setBoundaries(0, 100, circleValues);
  rotaryEncoder.setAcceleration(0); // aceleration helps reach higher values faster by increasing the increment if you move the encoder a lot
}

void loop()
{
  int buttonValue;

  // I2C
  for(int i = 0; i < 8; i++)
  {
    buttonValue = !mcp.digitalRead(i);

    // This logic makes sure we change the state of the relay only once if the button is holded
    if(!buttonValue)
    {
      PreviousButtonStatus[i] = false;
    }
    else if(!PreviousButtonStatus[i])
    {
      // Button is now HIGH and previously was LOW, so change the relay state
      RelayStatus[i] = !RelayStatus[i];
      PreviousButtonStatus[i] = true;

      if (i == 5 || i == 6)
      {
        serialPrint("Switching relay '");
        Serial.print(i);
        Serial.print("'. new state: ");
        Serial.println(RelayStatus[i]);
      }
    }
    
    // Rotary encoder
    //dont print anything unless value changed
    if (rotaryEncoder.encoderChanged())
    {
      serialPrint("Value: ");
      Serial.println(rotaryEncoder.readEncoder());
    }
    if (rotaryEncoder.isEncoderButtonClicked())
    {
      serialPrintln("Encoder cliecked!");
    }
  }
  writeRelaysStatus();
  delay(10);
}

void writeRelaysStatus()
{
  // Latching
  digitalWrite(U74HC_RCK, LOW);

  // First 8 bits are for the relays, sencond time are for the button LED
  for(int j = 0; j < 2; j++)
  {
    for(int i = 0; i < 8; i++)
    {
      digitalWrite(U74HC_SCK, LOW);
      digitalWrite(U74HC_SI, RelayStatus[i]);
      digitalWrite(U74HC_SCK, HIGH);
    }
  }

  // Unlatching
  digitalWrite(U74HC_RCK, HIGH);
}

void serialPrint(String msg)
{
  Serial.print("v9:");
  Serial.print(msg);
}

void serialPrintln(String msg)
{
  serialPrint(msg);
  Serial.println(".");
}
