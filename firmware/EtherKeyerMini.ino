// EtherKeyer Mini
// 
// Jason Milldrum, NT7S
// Etherkit LLC
//
// This work is licensed under CC BY-SA 4.0
//
// Last Revision: 11 May 2025
// For Rev C PCB
//
// A basic memory Morse Code keyer for use with paddles.
// Keyer speed is adjustable via potentiometer. Three message memories with dedicated playback
// buttons are provided. Message memory programming and other parameter settings are done via
// serial terminal access at 19200 baud.
//
// Libraries Used
// ==============
// Etherkit Morse (Library Manager)
// Arduino Timer (Library Manager)
//
// Pushbuttons
// ===========
//
// Short Press
// -----------
// Cancel any message playback by pressing any button or paddle.
//
// * 1 - Play Message Memory 1
// * 2 - Play Message Memory 2
// * 3 - Play Message Memory 3
//
// Long Press
// ----------
// * 1 - Tune Mode (press any key to exit)
// * 2 - Toggle Internal Sidetone
// * 3 - Enter/Exit UART Mode
//
// Serial commands
// ===============
// Serial terminal parameters: 19200 baud, send New Line only, no flow control.
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
// S:              Sidetone On
// O:              Sidetone Off
//
// Notes
// =====
// Maximum amount of RAM used should be no more than than 386 bytes in order for this to work

#include <stdint.h>
#include <Morse.h>
#include <arduino-timer.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define FIRMWARE_VERSION "11 May 2025"

// Pin defines
#define SPEED_INPUT PIN_PB5
#define PADDLE_RING PIN_PB2  // Typical config, ring == DAH
#define PADDLE_TIP PIN_PB3   // Typical config, tip == DIT
#define BUTTON_S1 PIN_PC0
#define BUTTON_S2 PIN_PC1
#define BUTTON_S3 PIN_PC2
#define KEY_OUTPUT PIN_PA6
#define SIDETONE_OUTPUT PIN_PA7

// Defaults
#define DEFAULT_SIDETONE_FREQ 600
#define DEFAULT_KEYER_SPEED 20

// Limits
#define KEYER_SPEED_LOWER 5
#define KEYER_SPEED_UPPER 40
#define BUTTON_PRESS_SHORT 10
#define BUTTON_PRESS_LONG 1000
#define SLEEP_TIME 3000  // In milliseconds, has to be long enough to not go to sleep during a long button press

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
#define EEP_SIDETONE_ON 129  // Sidetone active, 1 byte (uint8_t)

// Enumerations
enum class KeyerState {IDLE, DIT, DAH, DITIDLE, DAHIDLE, CHARSPACE, PLAYBACK, ANNUNCIATE, UART, TUNE};
enum class Button {NONE, S1, S2, S3, S1S2, S2S3, S1S3, HOLD};
enum class Iambic {A, B};

// Global variables
uint32_t keyer_speed = DEFAULT_KEYER_SPEED;
uint16_t sidetone_freq = DEFAULT_SIDETONE_FREQ;
bool paddle_ring_active = false;
bool paddle_tip_active = false;
bool key_down = false;
uint16_t dit_length;
uint32_t button_press_time;
uint32_t sleep_timeout;
KeyerState prev_keyer_state = KeyerState::IDLE;
KeyerState curr_keyer_state = KeyerState::IDLE;
KeyerState next_keyer_state = KeyerState::IDLE;
// Button last_button = Button::NONE;
bool paddle_reverse = false;
Iambic keyer_mode = Iambic::A;
bool sidetone_active = false;


// Object constructors
Timer<2, micros> morse_timer; // create a timer with 2 tasks and microsecond resolution
Timer<2, micros> memory_keyer_timer; // create a timer with 2 tasks and microsecond resolution
Timer<2, millis> state_expire_timer; // create a timer with 2 tasks and millisecond resolution
Morse morse(KEY_OUTPUT, keyer_speed);

ISR(PORTB_PORT_vect) {
  uint8_t flags = PORTB.INTFLAGS;
  PORTB.INTFLAGS = flags; //clear flags
  // if (flags & 0x02) {
  //   //
  // }
  // if (flags & 0x04) {
  //   //
  // }
  reset_sleep_timer();
}

ISR(PORTC_PORT_vect) {
  uint8_t flags = PORTC.INTFLAGS;
  PORTC.INTFLAGS = flags; //clear flags
  // if (flags & 0x02) {
  //   //
  // }
  // if (flags & 0x04) {
  //   //
  // }
  reset_sleep_timer();
}

// Timer Callbacks
// bool process_memory_keyer(void *) {
//   if(curr_keyer_state == KeyerState::PLAYBACK)
//   {
//     morse.update();
//   }

//   return true;
// }

bool process_keyer_sm(void *)
{
  uint16_t speed_pot_adc;
  // static uint32_t button_press_time;
  static Button last_button;

  if(curr_keyer_state == KeyerState::PLAYBACK)
  {
    morse.update();
  }

  // Process speed pot
  speed_pot_adc = analogRead(SPEED_INPUT);
  keyer_speed = (speed_pot_adc * (KEYER_SPEED_UPPER - KEYER_SPEED_LOWER) / 1023) + KEYER_SPEED_LOWER;
  dit_length = (1200 / keyer_speed);
  morse.setWPM(keyer_speed);

  // Process buttons
  Button cur_button = process_button();

  if (cur_button != Button::NONE) // Handle a button press
  {
    if (last_button == Button::NONE) // Short press
    {
      last_button = cur_button;
      button_press_time = millis();
    }
    else if (millis() > (button_press_time + BUTTON_PRESS_LONG) && (last_button != Button::HOLD))  // Long press
    {
      switch(cur_button)
      {
        case Button::S1:
          reset_sleep_timer();
          curr_keyer_state = KeyerState::TUNE;
          keyline_on();
          last_button = Button::HOLD;
          break;
        case Button::S2:
          if (sidetone_active)
          {
            set_sidetone(false);
            tone(SIDETONE_OUTPUT, 400);
            delay(200);
            noTone(SIDETONE_OUTPUT);
          }
          else
          {
            set_sidetone(true);
            tone(SIDETONE_OUTPUT, 1000);
            delay(200);
            noTone(SIDETONE_OUTPUT);
          }
          last_button = Button::HOLD;
          break;
        case Button::S3:
          reset_sleep_timer();
          if (curr_keyer_state == KeyerState::UART)
          {
            exit_uart();
          }
          else
          {
            curr_keyer_state = KeyerState::UART;
            // Toggle the pin modes for UART service
            pinMode(PADDLE_RING, OUTPUT);
            pinMode(PADDLE_TIP, INPUT);
            Serial.begin(19200);
            delay(100);
            Serial.println();
            Serial.println("EtherKeyer Mini");
            Serial.print("Firmware ");
            Serial.println(FIRMWARE_VERSION);
          }
          last_button = Button::HOLD;
          break;
      }
    }
  }
  else
  {
    if (last_button != Button::NONE)  // Check if this is a release
    {
      reset_sleep_timer();
      if ((millis() >= (button_press_time + BUTTON_PRESS_SHORT)) && (millis() < (button_press_time + BUTTON_PRESS_LONG))) // Short press
      {
        if (curr_keyer_state == KeyerState::PLAYBACK || curr_keyer_state == KeyerState::TUNE)  // Cancel playback if button pressed
        {
          morse.reset();
          // digitalWrite(KEY_OUTPUT, LOW);
          keyline_off();
          curr_keyer_state = KeyerState::IDLE;
        }
        else
        {
          curr_keyer_state = KeyerState::PLAYBACK;
          morse.setWPM(keyer_speed);
          send_message(((uint8_t)last_button - 1) * EEP_M2_ADDR);
        }
      }
      // button_press_time = UINT32_MAX;
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
        // digitalWrite(KEY_OUTPUT, HIGH);
        keyline_on();
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
        // digitalWrite(KEY_OUTPUT, HIGH);
        keyline_on();
        // state_expire_timer.in(dit_length * 3, ditdah_expire);
      }
      else
      {
        keyline_off();
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
        noTone(SIDETONE_OUTPUT);
        curr_keyer_state = KeyerState::IDLE;
      }
      if (sidetone_active)
      {
        if (morse.tx)
        {
          tone(SIDETONE_OUTPUT, DEFAULT_SIDETONE_FREQ);
        }
        else
        {
          noTone(SIDETONE_OUTPUT);
        }
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
      reset_sleep_timer();
      break;
    case KeyerState::UART:
      // TODO: UART idle timer to go back to sleep
      reset_sleep_timer();
      if (Serial.available())
      {
        char buf[41];
        char out[41];
        int size = Serial.readBytesUntil('\n', buf, 40);
        buf[size] = '\0'; // Null terminate so it's a string

        if (buf[1] == '?')  // Get parameter
        {
          switch (toupper(buf[0]))
          {
            case '1':
            case '2':
            case '3':
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
              exit_uart();
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
              keyer_mode = Iambic::A;
              EEPROM.put(EEP_KEYER_MODE, keyer_mode);
              break;
            case 'B':
              keyer_mode = Iambic::B;
              EEPROM.put(EEP_KEYER_MODE, keyer_mode);
              break;
            case 'S':
              // sidetone_active = true;
              // EEPROM.put(EEP_SIDETONE_ON, 1);
              set_sidetone(true);
              break;
            case 'O':
              // sidetone_active = false;
              // EEPROM.put(EEP_SIDETONE_ON, 0);
              set_sidetone(false);
              break;
            case '1':
            case '2':
            case '3':
              uint8_t addr = buf[0] - 49;
              // set_message((buf[0] - 49) * EEP_M2_ADDR, buf);
              memmove(buf, buf + 2, 40);
              strupr(buf);
              EEPROM.put(addr * EEP_M2_ADDR, buf);
              // Print it back
              EEPROM.get((addr) * EEP_M2_ADDR, out);
              Serial.print(addr + 1);
              Serial.print(":");
              Serial.println(out);
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
  // digitalWrite(KEY_OUTPUT, LOW);
  keyline_off();
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
    reset_sleep_timer();
    prev_keyer_state = KeyerState::IDLE;
    curr_keyer_state = KeyerState::DIT;
    next_keyer_state = KeyerState::IDLE;
    // digitalWrite(KEY_OUTPUT, HIGH);
    keyline_on();
    state_expire_timer.in(dit_length, ditdah_expire);
  }
  else if (next_keyer_state == KeyerState::DAH)
  {
    reset_sleep_timer();
    prev_keyer_state = KeyerState::IDLE;
    curr_keyer_state = KeyerState::DAH;
    next_keyer_state = KeyerState::IDLE;
    // digitalWrite(KEY_OUTPUT, HIGH);
    keyline_on();
    state_expire_timer.in(dit_length * 3, ditdah_expire);
  }
  else if (next_keyer_state == KeyerState::DITIDLE)
  {
    reset_sleep_timer();
    prev_keyer_state = KeyerState::IDLE;
    curr_keyer_state = KeyerState::DIT;
    next_keyer_state = KeyerState::IDLE;
    // digitalWrite(KEY_OUTPUT, HIGH);
    keyline_on();
    state_expire_timer.in(dit_length, ditdah_expire);
  }
  else if (next_keyer_state == KeyerState::DAHIDLE)
  {
    reset_sleep_timer();
    prev_keyer_state = KeyerState::IDLE;
    curr_keyer_state = KeyerState::DAH;
    next_keyer_state = KeyerState::IDLE;
    // digitalWrite(KEY_OUTPUT, HIGH);
    keyline_on();
    state_expire_timer.in(dit_length * 3, ditdah_expire);
  }
  else
  {
    // digitalWrite(KEY_OUTPUT, LOW);
    keyline_off();
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

void send_message(uint8_t addr)
{
  char out[41];
  EEPROM.get(addr, out);
  morse.send(out);
}

void keyline_on()
{
  digitalWriteFast(KEY_OUTPUT, HIGH);
  if (sidetone_active)
  {
    tone(SIDETONE_OUTPUT, DEFAULT_SIDETONE_FREQ);
  }
}

void keyline_off()
{
  digitalWriteFast(KEY_OUTPUT, LOW);
  if (sidetone_active)
  {
    noTone(SIDETONE_OUTPUT);
  }
}

void set_sidetone(uint8_t status)
{
  if (status)
  {
    sidetone_active = true;
    EEPROM.put(EEP_SIDETONE_ON, 1);
  }
  else
  {
    sidetone_active = false;
    EEPROM.put(EEP_SIDETONE_ON, 0);
  }
}

void exit_uart()
{
  curr_keyer_state = KeyerState::IDLE;
  Serial.println("Exit UART");
  Serial.end();
  // Toggle the pin modes for paddle input
  pinMode(PADDLE_RING, INPUT_PULLUP);
  pinMode(PADDLE_TIP, INPUT_PULLUP);
  // last_button = Button::HOLD;
}

Button process_button()
{
  if (digitalReadFast(BUTTON_S1) == LOW) {
    return Button::S1;
  }
  else if (digitalReadFast(BUTTON_S2) == LOW) {
    return Button::S2;
  }
  else if (digitalReadFast(BUTTON_S3) == LOW) {
    return Button::S3;
  }
  else
  {
    return Button::NONE;
  }
}

void setup()
{
  // Set up pins
  // For the purposes of saving power, we need to set all unused pins (not actively driven)
  // to be outputs or input w/ pullup
  pinMode(KEY_OUTPUT, OUTPUT);
  pinMode(SIDETONE_OUTPUT, OUTPUT);
  pinMode(PADDLE_RING, INPUT_PULLUP);
  pinMode(PADDLE_TIP, INPUT_PULLUP);
  pinMode(BUTTON_S1, INPUT_PULLUP);
  pinMode(BUTTON_S2, INPUT_PULLUP);
  pinMode(BUTTON_S3, INPUT_PULLUP);
  pinMode(SPEED_INPUT, INPUT);
  pinMode(PIN_PA1, OUTPUT);
  pinMode(PIN_PA2, OUTPUT);
  pinMode(PIN_PA3, OUTPUT);
  pinMode(PIN_PA4, OUTPUT);
  pinMode(PIN_PA5, OUTPUT);
  pinMode(PIN_PB0, OUTPUT);
  pinMode(PIN_PB1, OUTPUT);
  pinMode(PIN_PC3, OUTPUT);

  digitalWriteFast(KEY_OUTPUT, LOW);
  digitalWriteFast(SIDETONE_OUTPUT, LOW);

  // Set up pin interrupts
  PORTB.PIN2CTRL = 0b00001001; // Input pullup, interrupt on change;
  PORTB.PIN3CTRL = 0b00001001; // Input pullup, interrupt on change;
  PORTC.PIN0CTRL = 0b00001001; // Input pullup, interrupt on change;
  PORTC.PIN1CTRL = 0b00001001; // Input pullup, interrupt on change;
  PORTC.PIN2CTRL = 0b00001001; // Input pullup, interrupt on change;

  // Set up sleep mode
  sleep_enable(); 
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);     // Set sleep mode

  // Load and check EEPROM params
  // EEPROM.get(EEP_PADDLE_REV, paddle_reverse);
  // EEPROM.get(EEP_KEYER_MODE, keyer_mode);
  // EEPROM.get(EEP_SIDETONE_ON, sidetone_active);

  // Timer Setup
  morse_timer.every(1000, process_keyer_sm);
  // memory_keyer_timer.every(1000, process_memory_keyer);
  delay(100);  // Short delay here to stop the keyer from emitting a dit on power up
}

void loop()
{
  // Tick the various timers
  morse_timer.tick();
  // memory_keyer_timer.tick();
  state_expire_timer.tick();

  // Update inputs and output states
  if (digitalReadFast(PADDLE_RING) == LOW)
  {
    reset_sleep_timer();
    if (curr_keyer_state == KeyerState::PLAYBACK || curr_keyer_state == KeyerState::TUNE)
    {
      morse.reset();
      // digitalWrite(KEY_OUTPUT, LOW);
      keyline_off();
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

  if (digitalReadFast(PADDLE_TIP) == LOW)
  {
    reset_sleep_timer();
    if (curr_keyer_state == KeyerState::PLAYBACK || curr_keyer_state == KeyerState::TUNE)
    {
      morse.reset();
      // digitalWrite(KEY_OUTPUT, LOW);
      keyline_off();
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

  // Go to sleep if necessary
  if (millis() > sleep_timeout)
  {
    ADC0.CTRLA &= ~ADC_ENABLE_bm;
    sleep_cpu();
    ADC0.CTRLA |= ADC_ENABLE_bm;
  }
}
