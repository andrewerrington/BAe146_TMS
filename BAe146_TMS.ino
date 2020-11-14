/*
    BAe 146 TMS model for flight simulator
    
    For James (Mach7) https://www.cockpitbuilders.com
    Copyright November 2020 A M Errington
    All rights reserved.
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

// Arduino code for custom-made BAe 147 TMS

// Requires three thumbwheels for Tref input (+/-, 0-9, 0-9)
// Three thumbwheels for TGT input (0-9 x 3)
// Two latching pushbuttons (TO and TGT)
// Two momentary pushbuttons (MCT and TEST)
// One LED indicator (TO)
// Three digit 7-segment display
// One potentiometer for display brightness (optional)

// In this version, the display is driven by a TM1637

// -----------------------------------------------------------------------------
//234567890123456789012345678901234567890123456789012345678901234567890123456789

// AKJ7 TM1637 LED driver library
#include <TM1637.h>

// I/O pins for inputs

// Thumbwheel/switch matrix
// Common lines for bits
int SW_D0 = 2;  // Shared with TO
int SW_D1 = 3;  // Shared with MCT
int SW_D2 = 4;  // Shared with TEST
int SW_D3 = 5;  // Shared with TGT

// Dimmer pot on A7
int POT_DIM = A7;


// I/O pins for outputs

// Grounding lines for thumbwheel matrix
int TREF_UNITS = 6;
int TREF_TENS = 7;
int TREF_SIGN = 8;  // This is a single switch on D0 only

int BUTTONS = 12;

int TGT_UNITS = A0;
int TGT_TENS = A1;
int TGT_HUNS = A2;

// TO indicator LED
int LED_TO = 9; // Using pin 9 for PWM brightness.

// LED module (Using A4/A5 means I2C is no longer available)
int TM1637_CLK = A4;
int TM1637_DIO = A5;

TM1637 tm(TM1637_CLK, TM1637_DIO);  // CLK, DIO

// Currently 10, 11, A3, A6 are free
// 10 and 11 are PWM capable
// A6 is analog input only
// Pin 13 is also available, but attached to built-in LED

// Variables
int TGT;                // TGT reading from thumbwheels, 0 to 999
int Tref;               // Tref reading from thumbwheels, -99 to +99
int PA = 0;             // Pressure altitude (ft x1000) (constant in this version)
int N1;                 // N1 output, percent x10

long MCT_start_time;    // Timer for MCT display to change

int pot_dim_value = 0;  // Reading from DIM potentiometer
uint8_t brightness = 0;
uint8_t last_brightness = 0;

bool BTN_TO;    // Latching pushbutton
bool BTN_MCT;   // Momentary pushbutton
bool BTN_TEST;  // Momentary pushbutton
bool BTN_TGT;   // Latching pushbutton

// Constants

int settle_ms = 10;     // Time for switch matrix to settle after column is driven

enum display_states {
  DISP_INIT, // Startup, display is blank
  DISP_IDLE, // system is idle, display is blank
  DISP_N1, // TO is pressed (it is latching) display N1
  DISP_MCT_INIT_WAIT_ENTER, // Wait for the MCT button to be released
  DISP_MCT_INIT, // MCT is pressed. Show initial value for 6 seconds.
  DISP_MCT_CONST_WAIT_ENTER, // Wait for the MCT button to be released
  DISP_MCT_CONST, // Show indefinite MCT value until MCT is pressed again
  DISP_MCT_CONST_WAIT_EXIT, // Wait for the MCT button to be released
  DISP_TEST, // Test button is pressed. Show test value until it is released.
  DISP_TGT // TGT is pressed (it is latching) display TGT
};

enum display_states display_state;

// Lookup table for N1
// 17 rows for PA (ft x 1000) from 15 to -1 -> 0 to 16. It's in this order
// because that's how the original table was published.
// 23 columns for Tref from -50 to +60 deg. C inclusive, in steps of 5 degrees -> 0 to 22
// Returned value is (N1 x 10%)-800. A value of zero means the input is out of range,
// otherwise add 800 to get the actual value of N1x10. We store N1x10 to avoid floating point.
const uint8_t N1_lookup[17][23] PROGMEM =
{
  {   0, 167, 167, 167, 167, 167, 163, 157, 151, 147, 143, 139, 135, 126, 117, 108, 100,  91,   0,   0,   0,   0,   0 },
  {   0, 167, 167, 167, 167, 167, 164, 158, 151, 148, 143, 140, 136, 128, 119, 110, 102,  93,   0,   0,   0,   0,   0 },
  {   0,   0, 167, 167, 167, 167, 165, 159, 152, 149, 144, 141, 137, 129, 120, 111, 103,  95,  84,   0,   0,   0,   0 },
  {   0,   0, 167, 167, 167, 167, 165, 160, 153, 149, 145, 141, 138, 130, 122, 113, 105,  96,  86,   0,   0,   0,   0 },
  {   0,   0, 167, 167, 167, 167, 166, 161, 153, 150, 146, 142, 139, 131, 123, 114, 106,  98,  87,  82,   0,   0,   0 },
  {   0,   0,   0, 167, 167, 167, 166, 161, 153, 151, 147, 143, 140, 133, 124, 115, 107,  99,  89,  83,   0,   0,   0 },
  {   0,   0,   0, 167, 167, 167, 166, 162, 154, 152, 147, 144, 140, 134, 125, 117, 108, 101,  91,  84,   0,   0,   0 },
  {   0,   0,   0, 155, 164, 167, 166, 162, 155, 152, 147, 144, 141, 135, 126, 118, 109, 102,  92,  85,  80,   0,   0 },
  {   0,   0,   0, 143, 152, 161, 167, 162, 155, 152, 147, 144, 141, 136, 128, 119, 110, 103,  93,  86,  81,   0,   0 },
  {   0,   0,   0, 130, 139, 148, 157, 162, 155, 152, 148, 144, 141, 136, 128, 120, 111, 104,  95,  86,  82,  77,   0 },
  {   0,   0, 108, 118, 127, 136, 146, 155, 155, 152, 148, 145, 142, 136, 129, 120, 112, 105,  96,  87,  83,  78,   0 },
  {   0,  86,  96, 105, 114, 123, 132, 141, 150, 152, 148, 145, 142, 137, 129, 121, 111, 106,  98,  88,  84,  79,   0 },
  {   0,  75,  84,  93, 103, 111, 121, 129, 138, 147, 148, 145, 142, 137, 129, 121, 112, 106,  98,  89,  85,  80,  76 },
  {  53,  62,  71,  80,  89,  98, 107, 116, 125, 133, 141, 145, 142, 137, 129, 121, 113, 107,  99,  89,  85,  80,  76 },
  {  40,  49,  58,  67,  75,  84,  93, 102, 111, 119, 127, 136, 142, 137, 129, 121, 114, 107,  99,  89,  85,  81,  77 },
  {  28,  36,  45,  54,  62,  71,  79,  88,  97, 105, 113, 121, 129, 137, 130, 122, 114, 107,  99,  89,  86,  81,  77 },
  {  17,  25,  33,  41,  49,  57,  66,  74,  82,  90,  99, 107, 115, 123, 130, 122, 114, 107,  99,  90,  86,  82,  78 }
};

void setup() {

  // Start serial port
  // We will use this for diagnostics and debugging
  Serial.begin(115200);

  Serial.println(F("BAe 146 TMS start"));

  // Configure I/O pins
  pinMode(SW_D0, INPUT_PULLUP);
  pinMode(SW_D1, INPUT_PULLUP);
  pinMode(SW_D2, INPUT_PULLUP);
  pinMode(SW_D3, INPUT_PULLUP);

  digitalWrite(TREF_UNITS, HIGH);
  pinMode(TREF_UNITS, OUTPUT);
  digitalWrite(TREF_TENS, HIGH);
  pinMode(TREF_TENS, OUTPUT);
  digitalWrite(TREF_SIGN, HIGH);
  pinMode(TREF_SIGN, OUTPUT);

  digitalWrite(TGT_UNITS, HIGH);
  pinMode(TGT_UNITS, OUTPUT);
  digitalWrite(TGT_TENS, HIGH);
  pinMode(TGT_TENS, OUTPUT);
  digitalWrite(TGT_HUNS, HIGH);
  pinMode(TGT_HUNS, OUTPUT);

  digitalWrite(BUTTONS, HIGH);
  pinMode(BUTTONS, OUTPUT);   // Matrix column for buttons

  digitalWrite(LED_TO, LOW);  // TO indicator, initially off
  pinMode(LED_TO, OUTPUT);

  // Set up LED display
  tm.begin();
  tm.setBrightness(4);
  last_brightness = 4;

  display_state = DISP_INIT;
  
 /* 
  // Temporary test to print N1 lookup table
  for (int i=15; i>=-1; i--) { // PA
    for (int j=-50; j<=60; j+=5) { // Tref
      N1 = calc_N1(i,j);
      if (N1 < 100) {
        Serial.print(" ");
      }
      if (N1 < 10) {
        Serial.print(" ");
      }
      Serial.print(N1);
      Serial.print(" ");
    }
    Serial.println("");
  }
  */
}


void loop() {

  // Check the pot value and write a new brightness value
  // to the module. Do nothing if we are within 10 units of
  // the transition points to avoid flickering due to noise
  pot_dim_value = analogRead(POT_DIM);
  brightness = (pot_dim_value/128)+1;
  if (brightness != last_brightness) {
    if (abs(pot_dim_value - (last_brightness*128)) > 10) {
      Serial.print(F("Brightness: "));
      Serial.println(brightness);
      tm.setBrightness(brightness); // fixme: is this needed?
      tm.changeBrightness(brightness);
      last_brightness = brightness;
    }
  }

  // Read the pushbutton states
  digitalWrite(BUTTONS, LOW);
  delay(settle_ms);
  BTN_TO = !digitalRead(SW_D0);
  BTN_MCT = !digitalRead(SW_D1);
  BTN_TEST = !digitalRead(SW_D2);
  BTN_TGT = !digitalRead(SW_D3);
  digitalWrite(BUTTONS, HIGH);

  switch (display_state) {
    case DISP_INIT:
      // Initial state for display is clear
      tm.clearScreen();
      display_state = DISP_IDLE;
      break;

    case DISP_IDLE:
      // Anything can happen from here. Make sure these tests are prioritised.

      if (BTN_TEST) {
        // The Test button is pressed.
        Serial.println(F("Test pressed"));
        // Display Test value on LEDs
        tm.setDp(0);
        tm.display("F14 ");
        display_state = DISP_TEST;
        break;
      }

      if (BTN_MCT) {
        // The MCT button is pressed.
        Serial.println(F("MCT pressed"));
        // Display MCT value on LEDs and start timer
        tm.setDp(0);
        tm.display(8570); // Last digit is not shown
        MCT_start_time = millis();
        display_state = DISP_MCT_INIT_WAIT_ENTER;
        break;
      }
      
      if (BTN_TO) {
        // The TO button is pressed.
        Serial.println(F("TO pressed"));
        digitalWrite(LED_TO, HIGH);  // TO indicator on
        display_state = DISP_N1;
        break;
      }

      if (BTN_TGT) {
        // The TGT button is pressed.
        Serial.println(F("TGT pressed"));
        display_state = DISP_TGT;
        break;
      }
      
      break;

    case DISP_TEST:
      // We are showing the test value.
      // When the test button is released, clear the display.
      if (!BTN_TEST) {
        Serial.println(F("Test released"));
        tm.display("    ");
        display_state = DISP_IDLE;
      }
      break;

    case DISP_MCT_INIT_WAIT_ENTER:
      // MCT was just pressed. We are showing the initial value.
      // Wait for MCT to be released so we can act on another press later.
      // Handle the timer here too, in case the button is stuck on
      // for a long time.
      if (!BTN_MCT) {
        // The MCT button is released.
        Serial.println(F("MCT released"));
        display_state = DISP_MCT_INIT;
      }

      if ((millis() - MCT_start_time) > 6000) {
        Serial.println(F("MCT init timer expired while pressed"));
        tm.setDp(2);
        tm.display(9670); // Last digit is not shown
        display_state = DISP_MCT_CONST_WAIT_ENTER;
      }
      break;
      
    case DISP_MCT_INIT:
      // We are showing the initial MCT value
      // Do nothing until the timer has expired, then show the
      // next MCT value. If MCT is pressed during this time,
      // clear the display and return to idle (without showing
      // the second value).
      if ((millis() - MCT_start_time) > 6000) {
        Serial.println(F("MCT init timer expired"));
        tm.setDp(2);
        tm.display(9670); // Last digit is not shown
        display_state = DISP_MCT_CONST;
      }

      if (BTN_MCT) {
        // The MCT button is pressed.
        Serial.println(F("MCT pressed during 6s timer"));
        tm.setDp(0);
        tm.display("    ");
        display_state = DISP_MCT_CONST_WAIT_EXIT;
      }
      break;

    case DISP_MCT_CONST_WAIT_ENTER:
      // We are showing the second (constant) MCT value.
      // Do nothing until the MCT button is released again.  
      if (!BTN_MCT) {
        // The MCT button is released.
        Serial.println(F("MCT released in MCT const mode"));
        display_state = DISP_MCT_CONST;
      }
      break;

    case DISP_MCT_CONST:
      // We are showing the second (constant) MCT value.
      // Do nothing until the MCT button is pressed again.  
      if (BTN_MCT) {
        // The MCT button is pressed.
        Serial.println(F("MCT pressed in MCT mode. Clearing display."));
        tm.setDp(0);
        tm.display("    ");
        display_state = DISP_MCT_CONST_WAIT_EXIT;
      }
      break;

    case DISP_MCT_CONST_WAIT_EXIT:
      // We have finished showing the second (constant) MCT value.
      // Do nothing until the MCT button is released.  
      if (!BTN_MCT) {
        // The MCT button is released.
        Serial.println(F("MCT released in MCT mode. Back to idle."));
        display_state = DISP_IDLE;
      }
      break;

    case DISP_N1:
      // TO is a latching button. Stay in this mode
      // and update TO on the LEDs until the button
      // is released
      // Read Tref from thumbwheels
      Tref = get_Tref();
      Serial.print(F("PA = "));
      Serial.println(PA);
      Serial.print(F("Tref = "));
      Serial.println(Tref);
      // Calculate N1 from PA and Tref (PA is currently a global constant).
      N1 = calc_N1(PA, Tref);
      Serial.print(F("N1 = "));
      Serial.println(N1);
      // If N1 is zero then either the input values were out of range,
      // or the lookup table did not have a result for that combination
      // of Tref and PA
      // Display N1 on LEDs
      if (N1 != 0) {
        // N1 is valid
        tm.setDp(2);
        tm.display(N1*10);  // Shift numbers left by one space
      }
      else
      {
        // N1 is not valid
        tm.setDp(0);
        tm.display("----"); // Show '---' on display
      }
  
      if (!BTN_TO) {
        // The TO button is released.
        Serial.println(F("TO released. Back to idle."));
        tm.setDp(0);
        tm.display("    ");
        digitalWrite(LED_TO, LOW);  // TO indicator off
        display_state = DISP_IDLE;
      }
      break;

    case DISP_TGT:
      // TGT is a latching button. Stay in this mode
      // and update TGT on the LEDs until the button
      // is released
      // Read TGT from thumbwheels
      TGT = get_TGT();
      Serial.print(F("TGT = "));
      Serial.println(TGT);
      // Display TGT on LEDs
      tm.setDp(0);
      tm.display(TGT*10);
      
      if (!BTN_TGT) {
        // The TGT button is released.
        Serial.println(F("TGT released. Back to idle."));
        tm.setDp(0);
        tm.display("    ");
        display_state = DISP_IDLE;
      }
      break;

    default:
      // We should never get here
      Serial.println(F("State machine fell through to default state. Ouch!"));
      display_state = DISP_INIT;
      break;
  }

  delay(20);  // This should quench any switch bounce

}


int get_Tref(){
  // Read thumbwheels and return signed value between -99 and +99
  int units;
  int tens;
  int result;

  // Turn Tref thumbwheel common pins off (should be off all the time)
  // by driving them high
  digitalWrite(TREF_UNITS, HIGH);
  digitalWrite(TREF_TENS, HIGH);
  digitalWrite(TREF_SIGN, HIGH);

  // Look at units thumbwheel, and invert the bits
  digitalWrite(TREF_UNITS, LOW);
  delay(settle_ms);
  units = ((digitalRead(SW_D3) << 3) | (digitalRead(SW_D2) << 2) | (digitalRead(SW_D1) << 1) | digitalRead(SW_D0)) ^ 0x0F;
  digitalWrite(TREF_UNITS, HIGH);

  // Look at tens thumbwheel
  digitalWrite(TREF_TENS, LOW);
  delay(settle_ms);
  tens = ((digitalRead(SW_D3) << 3) | (digitalRead(SW_D2) << 2) | (digitalRead(SW_D1) << 1) | digitalRead(SW_D0)) ^ 0x0F;
  digitalWrite(TREF_TENS, HIGH);

  // Accumulate result
  result = (tens * 10) + units;

  // Look at sign thumbwheel and negate result if necessary
  digitalWrite(TREF_SIGN, LOW);
  delay(settle_ms);
  if (!digitalRead(SW_D0)) {
    result *= -1;
  }
  digitalWrite(TREF_SIGN, HIGH);

  return result;
}


int get_TGT() {
  // Read thumbwheels and return value between 0 and 999
  int units;
  int tens;
  int hundreds;
  int result;

  // Turn TGT thumbwheel common pins off (should be off all the time)
  // by driving them high
  digitalWrite(TGT_UNITS, HIGH);
  digitalWrite(TGT_TENS, HIGH);
  digitalWrite(TGT_HUNS, HIGH);

  // Look at units thumbwheel, and invert the bits
  digitalWrite(TGT_UNITS, LOW);
  delay(settle_ms);
  units = ((digitalRead(SW_D3) << 3) | (digitalRead(SW_D2) << 2) | (digitalRead(SW_D1) << 1) | digitalRead(SW_D0)) ^ 0x0F;
  digitalWrite(TGT_UNITS, HIGH);

  // Look at tens thumbwheel
  digitalWrite(TGT_TENS, LOW);
  delay(settle_ms);
  tens = ((digitalRead(SW_D3) << 3) | (digitalRead(SW_D2) << 2) | (digitalRead(SW_D1) << 1) | digitalRead(SW_D0)) ^ 0x0F;
  digitalWrite(TGT_TENS, HIGH);

  // Look at hundreds thumbwheel
  digitalWrite(TGT_HUNS, LOW);
  delay(settle_ms);
  hundreds = ((digitalRead(SW_D3) << 3) | (digitalRead(SW_D2) << 2) | (digitalRead(SW_D1) << 1) | digitalRead(SW_D0)) ^ 0x0F;
  digitalWrite(TGT_HUNS, HIGH);

  // Accumulate result
  result = (hundreds*100) + (tens * 10) + units;
  return result;
}


int calc_N1(int PA, int Tref) {
  // Given PA (ft x 1000) and Tref (degrees C) calculate N1
  // Note that PA never changes in this version, but it is useful
  // to remember that it could be incorporated into calculations later.

  uint8_t i,j;  // Row/column index
  int N1;       // Result
  
  // Row index is calculated from PA, which is altitude in ft x 1000.
  // Rows are numbered from 0 to 16 for the range of PA 15 to -1
  // Column index is calculated from Tref
  // Columns are numbered from 0 to 23 for the range of Tref -50 to +60
  // in steps of 5 degrees.
  
  if ((PA < -1) || (PA > 15)) {
    Serial.print(F("PA out of range : "));
    Serial.println(PA);
    return 0;
  }
  
  if ((Tref < -50) || (Tref > 60)) {
    Serial.print(F("Tref out of range : "));
    Serial.println(Tref);
    return 0;
  }
  
  i = 16 - (PA + 1);
  j = (Tref + 50) / 5;
  
  N1 = pgm_read_byte_near(&N1_lookup[i][j]);
  
  if (N1 != 0) {
    // Add offset to restore N1
    N1 += 800;
  }
  
  return N1;
}
