#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include "AiEsp32RotaryEncoder.h"
#include <FastLED.h>

Adafruit_MCP23X17 mcp;

#define NR_INPUTS  16
#define NR_OUTPUTS 24  

#define NR_RELAYS  8
#define NR_BUTTONS 10

// The component is called 74HC595. 
// But variables cannot start with a number :)
#define U74HC_SI       19
#define U74HC_RCK      21
#define U74HC_SCK      22
#define OutputEnabled  23

// I2C for the MCP23017
#define SDA            26
#define SCL            27

// Rotary encoder
#define ROTARY_ENCODER_A_PIN 25
#define ROTARY_ENCODER_B_PIN 33
#define ROTARY_ENCODER_BUTTON_PIN 32
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

void IRAM_ATTR readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
}

// LED
#define NUM_LEDS  800
#define LED_DATA_PIN 12

// GLobal State
bool OutputStatus[NR_OUTPUTS];
bool PreviousButtonStatus[NR_BUTTONS];
CRGB Leds[NUM_LEDS];
bool LedsAreOn;
int LedsAreStarting;
int gHue;

void setup()
{
  Serial.begin(115200);
  Serial.println("V9 - Initializing");

  pinMode(OutputEnabled, OUTPUT);
  digitalWrite(OutputEnabled, LOW);

  FastLED.addLeds<WS2812, LED_DATA_PIN, RGB>(Leds, NUM_LEDS);

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
      Serial.println("Error. I2C not initialized");
      //while (1);
    }
  }catch (String error){
    Serial.println(error);
  }

  for (int i = 0; i < NR_INPUTS; i++ ) {
    mcp.pinMode(i, INPUT);
  }

  // Before activating the relays, let's give some time to the input pins to estabilize
  Serial.println("Delaying relays activation");

  delay(3000);
  digitalWrite(OutputEnabled, HIGH);

  Serial.println("Relays activated!");

  // Rotary encoder
  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  //set boundaries and if values should cycle or not. 
  bool circleValues = false;
  int minValue = 0;
  int maxValue = 100;
  rotaryEncoder.setBoundaries(0, 25, circleValues);
  rotaryEncoder.setAcceleration(0); // aceleration helps reach higher values faster by increasing the increment if you move the encoder a lot
  
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } 
}

void serialPrintRelayChange(int relayIndex)
{
      Serial.print("Switching relay '");
      Serial.print(relayIndex);
      Serial.print("'. new state: ");
      Serial.print(OutputStatus[relayIndex]);
      Serial.print(" Outputs: ");
      for(int i = 0; i < NR_OUTPUTS; i++)
      {
        Serial.print(OutputStatus[i]);
      }
      Serial.println(".");
}

void ProcessButtons()
{
  // I2C
  // First buttons are for relay
  for(int i = 0; i < NR_BUTTONS; i++)
  {
    int buttonValue = !mcp.digitalRead(i);

    // This logic makes sure we change the state of the relay only once if the button is holded for some milliseconds (debouncing)
    if(!buttonValue)
    {
      PreviousButtonStatus[i] = false;
    }
    else if(!PreviousButtonStatus[i])
    {
      // Button is now HIGH and previously was LOW, so change the relay state
      if( i < NR_RELAYS)
      {
        // The button has a relay associated
        OutputStatus[i] = !OutputStatus[i];
      }
      OutputStatus[i + NR_RELAYS] = !OutputStatus[i + NR_RELAYS]; // Button light
      PreviousButtonStatus[i] = true;

      serialPrintRelayChange(i);
    }
  }
}

struct RotaryEncoderState
{
  bool changed;
  int value;
  bool clicked;
};

struct RotaryEncoderState ProcessRotaryEncoder()
{
  RotaryEncoderState encoderState = {rotaryEncoder.encoderChanged(), rotaryEncoder.readEncoder(), rotaryEncoder.isEncoderButtonClicked() };
  
  if (encoderState.changed)
  {
    Serial.print("Value: ");
    Serial.println(encoderState.value);
  }
  if (encoderState.clicked)
  {
    Serial.println("Encoder cliecked!");
  }  
  
  return encoderState;
}

void ProcessLeds(RotaryEncoderState encoderState)
{
  if(encoderState.clicked)
  {
    if(!LedsAreOn)
    {
      LedsAreOn = true;
      LedsAreStarting = 1;
      Serial.print("Turning on leds. Value: ");
      Serial.println(encoderState.value);
    }
    else
    {
      LedsAreOn = false;
      LedsAreStarting = 0;
      for(int j = 0; j < NUM_LEDS; j++)
      {
        Leds[j] = CRGB(0, 0, 0);
      }
    }
  }

  if(LedsAreStarting > 900)
  {
    // a colored dot sweeping back and forth, with fading trails
    // fadeToBlackBy( Leds, NUM_LEDS, 20);
    // int pos = beatsin16( 13, 0, NUM_LEDS-1 );
    // Leds[pos] += CHSV( gHue, 255, 192);
    for(int j = 0; j < NUM_LEDS; j++)
    {
      Leds[j] = CRGB(0, 0, encoderState.value * 10);
    }
  }
  else if(LedsAreOn)
  {
    // Leds
    for(int j = 0; j < NUM_LEDS / 10; j++)
    {
      for(int i = 0; i < 10; i++)
      {
        if(j % 3 == 0)
        {
          Leds[i+j*10] = CRGB(encoderState.value * 10, 0, 0);
        }
        if(j % 3 == 1)
        {
          Leds[i+j*10] = CRGB(0, encoderState.value * 10, 0);
        }
        if(j % 3 == 2)
        {
          Leds[i+j*10] = CRGB(0, 0, encoderState.value * 10);
        }
      }
    }
  }
  
  FastLED.show();
}

void loop()
{
  ProcessButtons();
  writeOutputStatus();

  RotaryEncoderState encoderState = ProcessRotaryEncoder();
  ProcessLeds(encoderState);

  delay(10);
}

void writeOutputStatus()
{
  // Latching
  digitalWrite(U74HC_RCK, LOW);

  for(int i = 0; i < NR_OUTPUTS; i++)
  {
    digitalWrite(U74HC_SCK, LOW);
    digitalWrite(U74HC_SI, OutputStatus[i]);
    digitalWrite(U74HC_SCK, HIGH);
  }

  // Unlatching
  digitalWrite(U74HC_RCK, HIGH);
}
