// EtherKeyer Mini
// 
// Jason Milldrum, NT7S
// Etherkit LLC
//
// This work is licensed under CC BY-SA 4.0
//
// Last Revision: 20 March 2024
//
// A basic memory Morse Code keyer for use with paddles.
// Keyer speed is adjustable via potentiometer. Three message memories with dedicated playback
// buttons are provided. Message memory programming and other parameter settings are done via
// serial terminal access at 57600 baud.
//
// Libraries Used
// ==============
// Etherkit Morse (Library Manager)
// Arduino Timer (Library Manager)
//
// Serial commands
// ===============
// Serial terminal parameters: 19200 baud, send New Line only
// The format for interacting with EtherKeyer via the UART is very simple. In order to place EtherKeyer
// into UART mode, press and hold button 3 for at least one second. If your serial terminal
// is open when you do this, you'll get a greeting from EtherKeyer to let you know it is ready for commands.
//
// Query
// -----
// W?     Keyer speed
// 1?     Message memory 1
// 2?     Message memory 2
// 3?     Message memory 3
//
// Parameter Set
// -------------
// 1:<message>     Message memory 1
// 2:<message>     Message memory 2
// 3:<message>     Message memory 3
// X:              Exit UART Mode
// R:              Reverse Paddle Terminals
// N:              Normal Paddle Terminals
// A:              Set Iambic A Mode
// B:              Set Iambic B Mode
//
// Notes
// =====
// Maximum amount of RAM used should be no more than than 386 bytes in order for this to work

#include <Morse.h>
#include <arduino-timer.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

// #include <util/atomic.h> // For testing millis rollover

#define FIRMWARE_VERSION "20 Mar 2024"

// Pin defines
#define BUTTON_INPUT A0
#define SPEED_INPUT A1
#define PADDLE_RING PIN_PB0  // Typical config, ring == DAH
#define PADDLE_TIP PIN_PB1   // Typical config, tip == DIT
#define KEY_OUTPUT PIN_PB3
#define SIDETONE_OUTPUT PIN_PB4

// Defaults
#define DEFAULT_SIDETONE_FREQ 600
#define DEFAULT_KEYER_SPEED 20

// Limits
#define KEYER_SPEED_LOWER 2
#define KEYER_SPEED_UPPER 40
#define BUTTON_ADC_MARGIN 20
#define BUTTON_PRESS_SHORT 10
#define BUTTON_PRESS_LONG 1000
#define SLEEP_TIME 1000  // In milliseconds

// EEPROM Addresses
// Why not just use a struct for all of EEPROM? RAM is too scarce to work with the whole struct at once.
// So, we only load one param at a time. A bit cumbersome and not very foolproof, but we have our limits.
#define EEP_M1_ADDR 0  // Message memory 3 string, 40 chars plus terminator
#define EEP_M2_ADDR 41  // Message memory 3 string, 40 chars plus terminator
#define EEP_M3_ADDR 82  // Message memory 3 string, 40 chars plus terminator
#define EEP_PADDLE_REV 123  // Paddle reverse, 1 byte (uint8_t)
#define EEP_SIDETONE_FREQ 124  // Sidetone frequency, 2 bytes (uint16_t)
#define EEP_SPEED_LOWER 126  // Keyer speed lower limit, 1 byte (uint8_t)
#define EEP_SPEED_UPPER 127  // Keyer speed upper limit, 1 byte (uint8_t)
#define EEP_KEYER_MODE 128  // Keyer mode, 1 byte (uint8_t)

// Other constants
#define BUTTON_1_ADC 550
#define BUTTON_2_ADC 730
// #define BUTTON_3_ADC 937
#define BUTTON_3_ADC 840
// #define BUTTONS_1_AND_2_ADC 456
// #define BUTTONS_2_AND_3_ADC 689
// #define BUTTONS_1_AND_3_ADC 527
// #define BUTTONS_ALL 440

// Enumerations
enum class KeyerState {IDLE, DIT, DAH, DITIDLE, DAHIDLE, CHARSPACE, PLAYBACK, ANNUNCIATE, UART, TUNE};
enum class Button {NONE, S1, S2, S3, S1S2, S2S3, S1S3, HOLD};
enum class Iambic {A, B};

// Global variables
// uint16_t speed_pot_adc;
// uint16_t button_adc;
uint32_t keyer_speed = DEFAULT_KEYER_SPEED;
uint16_t sidetone_freq = DEFAULT_SIDETONE_FREQ;
bool paddle_ring_active = false;
bool paddle_tip_active = false;
bool key_down = false;
uint16_t dit_length;
// uint32_t button_press_time;
uint32_t sleep_timeout;
KeyerState prev_keyer_state = KeyerState::IDLE;
KeyerState curr_keyer_state = KeyerState::IDLE;
KeyerState next_keyer_state = KeyerState::IDLE;
// Button last_button = Button::NONE;
uint8_t paddle_reverse = 0;
// volatile bool led = true;
Iambic keyer_mode = Iambic::A;


// Object constructors
Timer<1, micros> morse_timer; // create a timer with 1 task and microsecond resolution
Timer<2, millis> state_expire_timer; // create a timer with 2 tasks and millisecond resolution
Morse morse(KEY_OUTPUT, keyer_speed);

ISR (PCINT0_vect)        // Interrupt service routine 
{
  // digitalWrite(SIDETONE_OUTPUT, led ? HIGH : LOW);
  // led = !led;
}

ISR (WDT_vect)
{
  //
}

// Timer Callbacks
bool process_keyer_sm(void *)
{
  uint16_t speed_pot_adc;
  uint16_t button_adc;
  static uint32_t button_press_time;
  static Button last_button = Button::NONE;

  if(curr_keyer_state == KeyerState::PLAYBACK)
  {
    morse.update();
  }

  // Update inputs and output states
  if (digitalRead(PADDLE_RING) == LOW)
  {
    reset_sleep_timer();
    if (curr_keyer_state == KeyerState::PLAYBACK || curr_keyer_state == KeyerState::TUNE)
    {
      morse.reset();
      digitalWrite(KEY_OUTPUT, LOW);
      curr_keyer_state = KeyerState::IDLE;
    }
    else
    {
      paddle_ring_active = true;
    }
  }
  else
  {
    paddle_ring_active = false;
  }

  if (digitalRead(PADDLE_TIP) == LOW)
  {
    reset_sleep_timer();
    if (curr_keyer_state == KeyerState::PLAYBACK || curr_keyer_state == KeyerState::TUNE)
    {
      morse.reset();
      digitalWrite(KEY_OUTPUT, LOW);
      curr_keyer_state = KeyerState::IDLE;
    }
    else
    {
      paddle_tip_active = true;
    }
  }
  else
  {
    paddle_tip_active = false;
  }

  speed_pot_adc = analogRead(SPEED_INPUT);
  button_adc = analogRead(BUTTON_INPUT);

  // Process speed pot
  keyer_speed = (speed_pot_adc * (KEYER_SPEED_UPPER - KEYER_SPEED_LOWER) / 1023) + KEYER_SPEED_LOWER;
  // setWPM();
  dit_length = (1200 / keyer_speed);
  morse.setWPM(keyer_speed);

  // Process buttons
  if(button_adc > BUTTON_1_ADC - BUTTON_ADC_MARGIN && button_adc < BUTTON_1_ADC + BUTTON_ADC_MARGIN)
  {
    // Button 1 pressed
    if (last_button == Button::NONE)
    {
      delay(BUTTON_PRESS_SHORT);  // Debounce via short delay
      button_adc = analogRead(BUTTON_INPUT);
      if(button_adc > BUTTON_1_ADC - BUTTON_ADC_MARGIN && button_adc < BUTTON_1_ADC + BUTTON_ADC_MARGIN)
      {
        button_press_time = millis();
        last_button = Button::S1;
      }
    }
    else if ((millis() > button_press_time + BUTTON_PRESS_LONG) && (last_button != Button::HOLD))  // Long press
    {
      reset_sleep_timer();
      curr_keyer_state = KeyerState::TUNE;
      digitalWrite(KEY_OUTPUT, HIGH);
      last_button = Button::HOLD;
    }
  }
  else if(button_adc > BUTTON_2_ADC - BUTTON_ADC_MARGIN && button_adc < BUTTON_2_ADC + BUTTON_ADC_MARGIN)
  {
    // Button 2 pressed
    if (last_button == Button::NONE)
    {
      delay(BUTTON_PRESS_SHORT);  // Debounce via short delay
      button_adc = analogRead(BUTTON_INPUT);
      if(button_adc > BUTTON_2_ADC - BUTTON_ADC_MARGIN && button_adc < BUTTON_2_ADC + BUTTON_ADC_MARGIN)
      {
        button_press_time = millis();
        last_button = Button::S2;
      }
    }
    else if ((millis() > button_press_time + BUTTON_PRESS_LONG) && (last_button != Button::HOLD))  // Long press
    {
      // reset_sleep_timer();
      // curr_keyer_state = KeyerState::ANNUNCIATE;
      // tone(SIDETONE_OUTPUT, 600);
      // morse.send("HI");
      // last_button = Button::HOLD;
    }
  }
  else if(button_adc > BUTTON_3_ADC - BUTTON_ADC_MARGIN && button_adc < BUTTON_3_ADC + BUTTON_ADC_MARGIN)
  {
    // Button 3 pressed
   if (last_button == Button::NONE)
    {
      delay(BUTTON_PRESS_SHORT);  // Debounce via short delay
      button_adc = analogRead(BUTTON_INPUT);
      if(button_adc > BUTTON_3_ADC - BUTTON_ADC_MARGIN && button_adc < BUTTON_3_ADC + BUTTON_ADC_MARGIN)
      {
        button_press_time = millis();
        last_button = Button::S3;
      }
    }
    else if ((millis() > button_press_time + BUTTON_PRESS_LONG) && (last_button != Button::HOLD))  // Long press
    {
      reset_sleep_timer();
      if (curr_keyer_state == KeyerState::UART)
      {
        // digitalWrite(SIDETONE_OUTPUT, HIGH);
        curr_keyer_state = KeyerState::IDLE;
        Serial.println(F("Exit UART"));
        Serial.end();
        // Toggle the pin modes for paddle input
        pinMode(PADDLE_RING, INPUT_PULLUP);
        pinMode(PADDLE_TIP, INPUT_PULLUP);
        GIMSK |= bit(PCIE);                    // Turn off pin change interrupts
        last_button = Button::HOLD;
      }
      else
      {
        // digitalWrite(SIDETONE_OUTPUT, LOW);
        GIMSK &= ~(bit(PCIE));                    // Turn off pin change interrupts
        // PCMSK |= bit(PCINT0) | bit(PCINT1);      // Interrupt on pins PB0 and PB1
        curr_keyer_state = KeyerState::UART;
        // Toggle the pin modes for UART service
        pinMode(PADDLE_RING, OUTPUT);
        pinMode(PADDLE_TIP, INPUT);
        Serial.begin(19200);
        Serial.println();
        Serial.println(F("EtherKeyer Mini"));
        Serial.print(F("Firmware "));
        Serial.println(FIRMWARE_VERSION);
        last_button = Button::HOLD;
      }
    }
  }
  // else if(button_adc > BUTTONS_1_AND_3_ADC - BUTTON_ADC_MARGIN && button_adc < BUTTONS_1_AND_3_ADC + BUTTON_ADC_MARGIN)
  // {
  //   // Button 3 pressed
  //  if (last_button == Button::NONE)
  //   {
  //     button_press_time = millis();
  //     last_button = Button::S1S3;
  //   }
  //   else if ((millis() > button_press_time + BUTTON_PRESS_LONG) && (last_button != Button::HOLD))  // Long press
  //   {
  //     reset_sleep_timer();
  //     if (curr_keyer_state == KeyerState::UART)
  //     {
  //       digitalWrite(SIDETONE_OUTPUT, HIGH);
  //       curr_keyer_state = KeyerState::IDLE;
  //       Serial.println("Ending UART Mode");
  //       Serial.end();
  //       // Toggle the pin modes for paddle input
  //       pinMode(PADDLE_RING, INPUT_PULLUP);
  //       pinMode(PADDLE_TIP, INPUT_PULLUP);
  //       last_button = Button::HOLD;
  //     }
  //     else
  //     {
  //       digitalWrite(SIDETONE_OUTPUT, LOW);
  //       curr_keyer_state = KeyerState::UART;
  //       // Toggle the pin modes for UART service
  //       pinMode(PADDLE_RING, OUTPUT);
  //       pinMode(PADDLE_TIP, INPUT);
  //       Serial.begin(57600);
  //       Serial.println();
  //       Serial.println("EtherKeyer Mini");
  //       Serial.print("Firmware ");
  //       Serial.println(FIRMWARE_VERSION);
  //       last_button = Button::HOLD;
  //     }
  //   }
  // }
  // else if(button_adc > BUTTONS_1_AND_2_ADC - BUTTON_ADC_MARGIN && button_adc < BUTTONS_1_AND_2_ADC + BUTTON_ADC_MARGIN)
  // {
  //   // Buttons S1 and S1 pressed
  //  if (last_button == Button::NONE)
  //   {
//     button_press_time = millis();
  //     last_button = Button::S1S2;
  //   }
  //   else if ((millis() > button_press_time + BUTTON_PRESS_LONG) && (last_button != Button::HOLD))  // Long press
  //   {
  //     reset_sleep_timer();
  //     last_button = Button::HOLD;
  //   }
  // }
  else if (button_adc > 1023 - BUTTON_ADC_MARGIN) // No button being pressed
  {
    if (last_button != Button::NONE)  // Check if this is a release
    {
      // reset_sleep_timer();
      if ((millis() >= (button_press_time + BUTTON_PRESS_SHORT)) && (millis() < (button_press_time + BUTTON_PRESS_LONG))) // Short press
      {
        // char out[41];
        switch (last_button)
        {
        case Button::S1:
          if (curr_keyer_state == KeyerState::PLAYBACK || curr_keyer_state == KeyerState::TUNE)  // Cancel playback if button pressed
          {
            morse.reset();
            digitalWrite(KEY_OUTPUT, LOW);
            curr_keyer_state = KeyerState::IDLE;
          }
          else
          {
            curr_keyer_state = KeyerState::PLAYBACK;
            send_message(EEP_M1_ADDR);
            // EEPROM.get(EEP_M1_ADDR, out);
            // morse.send(out);
          }
          break;
        case Button::S2:
          if (curr_keyer_state == KeyerState::PLAYBACK || curr_keyer_state == KeyerState::TUNE)  // Cancel playback if button pressed
          {
            morse.reset();
            digitalWrite(KEY_OUTPUT, LOW);
            curr_keyer_state = KeyerState::IDLE;
          }
          else
          {
            curr_keyer_state = KeyerState::PLAYBACK;
            send_message(EEP_M2_ADDR);
            // EEPROM.get(EEP_M2_ADDR, out);
            // morse.send(out);
          }
          break;
        case Button::S3:
          if (curr_keyer_state == KeyerState::PLAYBACK || curr_keyer_state == KeyerState::TUNE)  // Cancel playback if button pressed
          {
            morse.reset();
            digitalWrite(KEY_OUTPUT, LOW);
            curr_keyer_state = KeyerState::IDLE;
          }
          else
          {
            curr_keyer_state = KeyerState::PLAYBACK;
            send_message(EEP_M3_ADDR);
            // EEPROM.get(EEP_M3_ADDR, out);
            // morse.send(out);
          }
          break;
        case Button::S1S2:
          break;
        }
      }

      last_button = Button::NONE;
    }
  }

  // Process the keyer state machine
  switch (curr_keyer_state)
  {
    case KeyerState::IDLE:
      // Check paddle inputs
      if (paddle_tip_active)
      {
        if (paddle_reverse)
        {
          curr_keyer_state = KeyerState::DAH;
          // prev_keyer_state = KeyerState::DAH;
          state_expire_timer.in(dit_length * 3, ditdah_expire);
        }
        else
        {
          curr_keyer_state = KeyerState::DIT;
          // prev_keyer_state = KeyerState::DIT;
          state_expire_timer.in(dit_length, ditdah_expire);
        }
        prev_keyer_state = KeyerState::IDLE;
        next_keyer_state = KeyerState::IDLE;
        digitalWrite(KEY_OUTPUT, HIGH);
        // state_expire_timer.in(dit_length, ditdah_expire);
      }
      else if (paddle_ring_active)
      {
        if (paddle_reverse)
        {
          curr_keyer_state = KeyerState::DIT;
          // prev_keyer_state = KeyerState::DIT;
          state_expire_timer.in(dit_length, ditdah_expire);
        }
        else
        {
          curr_keyer_state = KeyerState::DAH;
          // prev_keyer_state = KeyerState::DAH;
          state_expire_timer.in(dit_length * 3, ditdah_expire);
        }
        prev_keyer_state = KeyerState::IDLE;
        next_keyer_state = KeyerState::IDLE;
        digitalWrite(KEY_OUTPUT, HIGH);
        // state_expire_timer.in(dit_length * 3, ditdah_expire);
      }
      break;
    case KeyerState::DIT:  // Where the squeeze keying happens
      // if (paddle_reverse)
      // {
      //   if (paddle_tip_active)
      //   {
      //     next_keyer_state = KeyerState::DAH;
      //     prev_keyer_state = KeyerState::DIT;
      //   }
      // }
      // else
      // {
        if (keyer_mode == Iambic::A)
        {
          if (paddle_reverse ? paddle_tip_active : paddle_ring_active)
          {
            prev_keyer_state = KeyerState::DIT;
            next_keyer_state = KeyerState::DAH;
          }
          else if (!paddle_ring_active && !paddle_tip_active)
          {
            next_keyer_state = KeyerState::IDLE;
          }
        }
        else if (keyer_mode == Iambic::B)
        {
          if (paddle_reverse ? paddle_tip_active : paddle_ring_active)
          {
            prev_keyer_state = KeyerState::DIT;
            next_keyer_state = KeyerState::DAH;
          }
          else if (!paddle_ring_active && !paddle_tip_active && next_keyer_state == KeyerState::DAH)
          {
            prev_keyer_state = KeyerState::IDLE;
            next_keyer_state = KeyerState::DAH;
          }
        }
        
      // }
      break;
    case KeyerState::DAH:  // Where the squeeze keying happens
      // if (paddle_reverse)
      // {
      //   if (paddle_ring_active)
      //   {
      //     next_keyer_state = KeyerState::DIT;
      //     prev_keyer_state = KeyerState::DAH;
      //   }
      // }
      // else
      // {
        if (keyer_mode == Iambic::A)
        {
          if (paddle_reverse ? paddle_ring_active : paddle_tip_active)
          {

            prev_keyer_state = KeyerState::DAH;
            next_keyer_state = KeyerState::DIT;
          }
          else if (!paddle_ring_active && !paddle_tip_active)
          {
            next_keyer_state = KeyerState::IDLE;
          }
        }
        else if (keyer_mode == Iambic::B)
        {
          if (paddle_reverse ? paddle_ring_active : paddle_tip_active)
          {
            prev_keyer_state = KeyerState::DAH;
            next_keyer_state = KeyerState::DIT;
          }
          else if (!paddle_ring_active && !paddle_tip_active && next_keyer_state == KeyerState::DIT)
          {
            prev_keyer_state = KeyerState::IDLE;
            next_keyer_state = KeyerState::DIT;
          }
        }
      // }
      break;
    case KeyerState::CHARSPACE:
      reset_sleep_timer();
      if (prev_keyer_state == KeyerState::DAH)
      {
        if (paddle_reverse ? paddle_ring_active : paddle_tip_active)
        {
          next_keyer_state = KeyerState::DIT;
        }
      }
      else if (prev_keyer_state == KeyerState::DIT)
      {
        if (paddle_reverse ? paddle_tip_active : paddle_ring_active)
        {
          next_keyer_state = KeyerState::DAH;
        }
      }
      break;
    case KeyerState::PLAYBACK:
      reset_sleep_timer();
      if (morse.busy == false)
      {
        noTone();
        curr_keyer_state = KeyerState::IDLE;
      }
      break;
    case KeyerState::ANNUNCIATE:
      reset_sleep_timer();
      // if (morse.busy == false)
      // {
      //   noTone();
      //   curr_keyer_state = KeyerState::IDLE;
      // }
      // if (morse.tx)
      // {
      //   tone(SIDETONE_OUTPUT, 600);
      // }
      // else
      // {
      //   noTone();
      // }
      break;
    case KeyerState::TUNE:

      break;
    case KeyerState::UART:
      // TODO: UART idle timer to go back to sleep
      reset_sleep_timer();
      if (Serial.available())
      {
        char buf[45];
        char out[41];
        int size = Serial.readBytesUntil('\n', buf, 44);
        buf[size] = '\0'; // Null terminate so it's a string

        if (buf[1] == '?')  // Get parameter
        {
          switch (toupper(buf[0]))
          {
            case '1':
            case '2':
            case '3':
              // print_message((buf[0] - 49) * EEP_M2_ADDR);
              EEPROM.get((buf[0] - 49) * EEP_M2_ADDR, out);
              Serial.println(out);
              break;
            case 'W':  // Get WPM
              Serial.println(keyer_speed);
              break;
          }
        }
        else if (buf[1] == ':')  // Set parameter
        {
          switch (toupper(buf[0]))
          {
            case 'X':  // Exit UART mode
              // digitalWrite(SIDETONE_OUTPUT, HIGH);
              curr_keyer_state = KeyerState::IDLE;
              Serial.println(F("Exit UART"));
              Serial.end();
              // Toggle the pin modes for paddle input
              pinMode(PADDLE_RING, INPUT_PULLUP);
              pinMode(PADDLE_TIP, INPUT_PULLUP);
              last_button = Button::HOLD;
              break;
            case 'R':
              paddle_reverse = 1;
              EEPROM.put(EEP_PADDLE_REV, 1);
              break;
            case 'N':
              paddle_reverse = 0;
              EEPROM.put(EEP_PADDLE_REV, 0);
              break;
            case 'A':
              // TODO: These use a lot of code for some reason
              keyer_mode = Iambic::A;
              EEPROM.put(EEP_KEYER_MODE, keyer_mode);
              break;
            case 'B':
              keyer_mode = Iambic::B;
              EEPROM.put(EEP_KEYER_MODE, keyer_mode);
              break;
            case '1':
            case '2':
            case '3':
              uint8_t addr = buf[0] - 49;
              // set_message((buf[0] - 49) * EEP_M2_ADDR, buf);
              memmove(buf, buf + 2, 40);
              strupr(buf);
              EEPROM.put(addr * EEP_M2_ADDR, buf);
              break;
          }
        }

        // Clear serial buffer
        memset(buf,0,strlen(buf));
      }
      break;
  }

  return true;
}

bool ditdah_expire(void *)
{
  digitalWrite(KEY_OUTPUT, LOW);
  curr_keyer_state = KeyerState::CHARSPACE;
  state_expire_timer.in(dit_length, charspace_expire);
  return false;
}

static bool charspace_expire(void *)
{
  // if (next_keyer_state == KeyerState::IDLE)
  // {
  //   if (paddle_ring_active)
  //   {
  //     if (paddle_reverse)
  //     {
  //       curr_keyer_state = KeyerState::DIT;
  //     }
  //     else
  //     {
  //       curr_keyer_state = KeyerState::DAH;
  //       // next_keyer_state = KeyerState::DAH;
  //     }
  //     // next_keyer_state = KeyerState::DAH;
  //   }
  //   else if (paddle_tip_active)
  //   {
  //     if (paddle_reverse)
  //     {
  //       curr_keyer_state = KeyerState::DAH;
  //     }
  //     else
  //     {
  //       curr_keyer_state = KeyerState::DIT;
  //       // next_keyer_state = KeyerState::DAH;
  //     }
  //     // next_keyer_state = KeyerState::DIT;
  //   }
  // }
  // prev_keyer_state = KeyerState::IDLE;
  if (next_keyer_state == KeyerState::DIT)
  {
    prev_keyer_state = KeyerState::IDLE;
    curr_keyer_state = KeyerState::DIT;
    next_keyer_state = KeyerState::IDLE;
    digitalWrite(KEY_OUTPUT, HIGH);
    state_expire_timer.in(dit_length, ditdah_expire);
  }
  else if (next_keyer_state == KeyerState::DAH)
  {
    prev_keyer_state = KeyerState::IDLE;
    curr_keyer_state = KeyerState::DAH;
    next_keyer_state = KeyerState::IDLE;
    digitalWrite(KEY_OUTPUT, HIGH);
    state_expire_timer.in(dit_length * 3, ditdah_expire);
  }
  else if (next_keyer_state == KeyerState::DITIDLE)
  {
    prev_keyer_state = KeyerState::IDLE;
    curr_keyer_state = KeyerState::DIT;
    next_keyer_state = KeyerState::IDLE;
    digitalWrite(KEY_OUTPUT, HIGH);
    state_expire_timer.in(dit_length, ditdah_expire);
  }
  else if (next_keyer_state == KeyerState::DAHIDLE)
  {
    prev_keyer_state = KeyerState::IDLE;
    curr_keyer_state = KeyerState::DAH;
    next_keyer_state = KeyerState::IDLE;
    digitalWrite(KEY_OUTPUT, HIGH);
    state_expire_timer.in(dit_length * 3, ditdah_expire);
  }
  else
  {
    digitalWrite(KEY_OUTPUT, LOW);
    curr_keyer_state = KeyerState::IDLE;
    next_keyer_state = KeyerState::IDLE;
  }

  // if (paddle_ring_active)
  // {
  //   digitalWrite(KEY_OUTPUT, HIGH);
  //   next_keyer_state = KeyerState::IDLE;
  //   if (paddle_reverse)
  //   {
  //     curr_keyer_state = KeyerState::DIT;
  //     state_expire_timer.in(dit_length, ditdah_expire);
  //   }
  //   else
  //   {
  //     curr_keyer_state = KeyerState::DAH;
  //     state_expire_timer.in(dit_length * 3, ditdah_expire);
  //   }
  //   // next_keyer_state = KeyerState::DAH;
  // }
  // else if (paddle_tip_active)
  // {
  //   digitalWrite(KEY_OUTPUT, HIGH);
  //   next_keyer_state = KeyerState::IDLE;
  //   if (paddle_reverse)
  //   {
  //     curr_keyer_state = KeyerState::DAH;
  //     state_expire_timer.in(dit_length * 3, ditdah_expire);
  //   }
  //   else
  //   {
  //     curr_keyer_state = KeyerState::DIT;
  //     state_expire_timer.in(dit_length, ditdah_expire);
  //   }
  //   // next_keyer_state = KeyerState::DIT;
  // }
  // else
  // {
  //   digitalWrite(KEY_OUTPUT, LOW);
  //   // PORTB &= ~(1 << PB3);
  //   curr_keyer_state = KeyerState::IDLE;
  //   next_keyer_state = KeyerState::IDLE;
  // }
  
  return false;
}

// Other Functions
void setWPM()
{
  // Dit length in milliseconds is 1200 ms / WPM
	// dit_length = (1200 / keyer_speed);
}

// void sleep()
// {
//   noInterrupts();
//   GIMSK |= (1 << PCIE);                    // Turn on pin change interrupts
//   // PCMSK |= (1 << PCINT0);                  // Pin change interrupt for PB0
//   // PCMSK |= (1 << PCINT1);                  // Pin change interrupt for PB1
//   PCMSK |= bit(PCINT0) | bit(PCINT1);
//   // digitalWrite(SIDETONE_OUTPUT, HIGH);
//   set_sleep_mode(SLEEP_MODE_PWR_DOWN);     // Set sleep mode
//   power_all_disable();                     // Turn off all peripherals
//   sleep_enable();                          // enables the sleep bit in the mcucr register so sleep is possible
//   ADCSRA &= (~(1 << ADEN));                // Disable ADC
//   interrupts();
//   sleep_mode();                            // Put controller to sleep
  
//   sleep_disable();                         // first thing after waking from sleep: disable sleep...
//   wdt_disable();
//   power_all_enable();
//   ADCSRA |= (1 << ADEN);;                  // Enable ADC
//   GIMSK &= (~(1 << PCIE));                 // Turn off pin change interrupts
//   // digitalWrite(SIDETONE_OUTPUT, LOW);
//   reset_sleep_timer();
// }

void reset_watchdog()
{
  // wdt_reset();
  // MCUSR = 0;
  // // Setup watchdog to fire every 125 ms
  // WDTCR = bit(WDCE) | bit(WDE) | bit(WDIF) | bit(WDIE) | bit(WDP0)| bit (WDP2);
}

void reset_sleep_timer()
{
  // Reset the sleep timer
  sleep_timeout = millis() + SLEEP_TIME;
}


void wdt_on()
{
  noInterrupts();
  WDTCR |= bit(WDCE) | bit(WDE);   // Enable the WD Change Bit
  WDTCR = bit(WDIE) | bit(WDP0);             // Enable WDT Interrupt and set timeout to 32 ms (for responsiveness)
  interrupts();
}

void wdt_off()
{
  noInterrupts();
  // MCUSR &= ~bit(WDRF);                 // Clear the WDT reset flag
  MCUSR = 0; 
  WDTCR |= (bit(WDCE) | bit(WDE));   // Enable the WD Change Bit
  WDTCR = 0x00;   
  interrupts();
}

// void print_message(uint8_t addr)
// {
//   char out[41];
//   EEPROM.get(addr, out);
//   Serial.println(out);
// }

void send_message(uint8_t addr)
{
  char out[41];
  EEPROM.get(addr, out);
  morse.send(out);
}

// void set_message(uint8_t addr, char* msg)
// {
//   memmove(msg, msg + 2, 40);
//   strupr(msg);
//   EEPROM.put(addr, msg);
// }

// bool playback_morse(void *)
// {
//   if(curr_keyer_state == KeyerState::PLAYBACK)
//   {
//     morse.update();
//   }

//   return true;
// }

// void setMillis(unsigned long ms)
// {
//     extern unsigned long millis_timer_millis;
//     ATOMIC_BLOCK (ATOMIC_RESTORESTATE) {
//         millis_timer_millis = ms;
//     }
// }

void setup()
{
  noInterrupts();
  // Set system clock to 4 MHz
  CLKPR = (1<<CLKPCE); // Prescaler enable
  CLKPR = (1<<CLKPS0); // Clock division factor 2 (0001), 4 MHz clock

  if(MCUSR & _BV(WDRF))  // If a reset was caused by WDT, then disable it so not stuck in loop
  {
    wdt_off();
  }

  // Set up sleep mode
  sleep_enable(); 
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);     // Set sleep mode

  GIMSK |= bit(PCIE);                    // Turn on pin change interrupts
  PCMSK |= bit(PCINT0) | bit(PCINT1);      // Interrupt on pins PB0 and PB1

  interrupts();

  // Load and check EEPROM params
  EEPROM.get(EEP_PADDLE_REV, paddle_reverse);
  EEPROM.get(EEP_KEYER_MODE, keyer_mode);

  // Set up pins
  pinMode(KEY_OUTPUT, OUTPUT);
  pinMode(SIDETONE_OUTPUT, OUTPUT);
  pinMode(PADDLE_RING, INPUT_PULLUP);
  pinMode(PADDLE_TIP, INPUT_PULLUP);
  digitalWrite(KEY_OUTPUT, LOW);
  digitalWrite(SIDETONE_OUTPUT, LOW);

  // Timer Setup
  morse_timer.every(1000, process_keyer_sm);

  // Set initial keyer speed
  // setWPM();
}

void loop()
{
  uint16_t button_adc;

  // Tick the various timers
  morse_timer.tick();
  state_expire_timer.tick();

  if (millis() > sleep_timeout)
  {
    power_all_disable();                     // Turn off all peripherals
    ADCSRA &= (~(1 << ADEN));                // Disable ADC
    wdt_on();
    sleep_mode();                            // Put controller to sleep

    wdt_off();
    power_all_enable();
    // ADCSRA |= bit(ADEN);

    button_adc = analogRead(BUTTON_INPUT);
    if (button_adc < 1020)
    {
      reset_sleep_timer();
    }
    else
    {
      sleep_timeout = 0;  // Put back to sleep immediately in loop
    }
  }
}
