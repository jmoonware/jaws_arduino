pin_size_t audioPin = D0; // D0 = PWM5A = GPI26
pin_size_t motorPin = D2;  // D2 = PWM6A = GPIO28
pin_size_t irqPin = D4; // program D9 = PWM2A = GPIO4 for IRQ
pin_size_t debugPin = D6; // jjust a pin to send signals for debugging

pin_size_t limitOpenPin = D7; // limit switch, open lid
pin_size_t limitClosePin = D8; // limit switch, close lid
pin_size_t motorControlPin = D9; // relay for switching motor on/off 
pin_size_t musicEnablePin = D10; // set high during open/speech/close states 

#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <elapsedMillis.h>
#include "jaws_include.h"

// extern uint8_t *audio_buffer;

// were used in hard-coded test pattern
//#define IRQ_TOP 6400
//#define MOTOR_TOP 10431
//#define MOTOR_DIV 255
//#define MOTOR_UPDATE 415

//#define AUDIO_SAMPLES 100000
//#define MOTOR_SAMPLES 241 // should be >= AUDIO_SAMPLES/MOTOR_UPDATE
//#define MOTOR_MIN 260
//#define MOTOR_MAX 1043
//uint8_t audio_buffer[AUDIO_SAMPLES];
//uint16_t motor_buffer[MOTOR_SAMPLES];

#define STOP_AFTER 1 // repeat speech cycle this many times in SPEECH_STATE

static uint32_t audio_buffer_position = 0;
static uint32_t motor_update_count = 0;
static uint32_t motor_buffer_position = 0;
static uint32_t stop_after = 0;

uint8_t audioSlice;
uint8_t irqSlice;
uint8_t motorSlice;

uint8_t openLimitState;
uint8_t closeLimitState;

// state machine
enum states {
 INITIAL_STATE,
 WAIT_STATE,
 HOMING_STATE,
 OPENING_STATE,
 SPEECH_STATE,
 CLOSING_STATE,
 DELAY_STATE
};

states machineState = HOMING_STATE;


void isr_audio() {

  if (pwm_get_irq_status_mask()&(1<<irqSlice)) {
    pwm_clear_irq(irqSlice);
//    digitalWrite(debugPin, HIGH); 
//    digitalWrite(debugPin, LOW);
  }

 // digitalWrite(irqPin, HIGH);



  // set audio level
  pwm_set_chan_level(audioSlice, 0, (uint16_t)audio_buffer[audio_buffer_position]);

  // increment or reset to beginning at end of buffer
  audio_buffer_position++;
  if (audio_buffer_position >= AUDIO_SAMPLES) {
    audio_buffer_position=0;
    motor_update_count=0;
    motor_buffer_position=0;
    ++stop_after;
    if (stop_after >= STOP_AFTER) {
      pwm_set_irq_enabled(irqSlice, false); 
      irq_set_enabled(PWM_IRQ_WRAP, false);
      pwm_set_enabled(irqSlice, false);
      return;
    }
  }

  // increment motor update count and update motor if necessary
  motor_update_count++;
  if (motor_update_count >= MOTOR_UPDATE) {
    motor_update_count=0;
    pwm_set_chan_level(motorSlice, 0, (uint16_t)motor_buffer[motor_buffer_position]);
    motor_buffer_position++;
    if (motor_buffer_position >= MOTOR_SAMPLES) motor_buffer_position=0;
  }

//  digitalWrite(irqPin, LOW);

}



void setup_isr() {
  // reset repeat counter
  stop_after=0;
  pwm_config irqConfig = pwm_get_default_config();
  pwm_config_set_wrap(&irqConfig, IRQ_TOP); // number of clock cycles to update audio values
  pwm_init(irqSlice, &irqConfig, true);
  pwm_set_chan_level(irqSlice, 0, 100); // just something to look at on scope
  irq_set_enabled(PWM_IRQ_WRAP, true);
  pwm_set_irq_enabled(irqSlice, true);
  pwm_clear_irq(irqSlice);
}

void setup() {


  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);

  digitalWrite(PIN_LED_R, HIGH); // HIGH turns off
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, HIGH);

  gpio_set_function(audioPin, GPIO_FUNC_PWM);
//  pinMode(irqPin, OUTPUT);
  pinMode(debugPin, OUTPUT);
  gpio_set_function(irqPin, GPIO_FUNC_PWM);
  gpio_set_function(motorPin, GPIO_FUNC_PWM);

// motor relay, limit switches
  pinMode(motorControlPin, OUTPUT);
  pinMode(limitOpenPin, INPUT);
  pinMode(limitClosePin, INPUT);
  pinMode(musicEnablePin, OUTPUT);

  // simple test pattern
//  for (uint32_t i=0; i < AUDIO_SAMPLES; ++i) {
//    audio_buffer[i]=(uint8_t)(200*(float)i/(float)AUDIO_SAMPLES);
//  }

  // full range sweep of motor
//  for (uint32_t i=0; i < MOTOR_SAMPLES; ++i) {
//    motor_buffer[i]=(uint16_t)(MOTOR_MIN + ((float)(MOTOR_MAX-MOTOR_MIN)/(float)MOTOR_SAMPLES)*i);
//  }

  // "slice" is a weird name for "Counter Number" - there are 8 16 bit counters (0-7), 
  // each having two channels (A=0,B=1) supporting two outputs with different CC values
  audioSlice = pwm_gpio_to_slice_num(audioPin);
  motorSlice = pwm_gpio_to_slice_num(motorPin);

  // audio counter (slice) - creates PWM audio signal
  pwm_config audioConfig = pwm_get_default_config();
  pwm_config_set_wrap(&audioConfig, AUDIO_TOP); // this many audio resolution values  


  // motor counter (slice) - creates PWM signal for motor
  pwm_config motorConfig = pwm_get_default_config();
  pwm_config_set_wrap(&motorConfig, MOTOR_TOP); // with prescaling, gets to 50 Hz  
  
  pwm_init(audioSlice, &audioConfig, true);
  pwm_set_chan_level(audioSlice, 0, 100);

  pwm_init(motorSlice, &motorConfig, true);
  pwm_set_chan_level(motorSlice, 0, 3000);
  pwm_set_clkdiv_int_frac(motorSlice, MOTOR_DIV, 0);

  irq_add_shared_handler(PWM_IRQ_WRAP, isr_audio,PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

  pwm_set_irq_enabled(audioSlice,false);
  pwm_clear_irq(audioSlice);
  pwm_set_irq_enabled(motorSlice,false);
  pwm_clear_irq(motorSlice);
  

  machineState = INITIAL_STATE; // initial state
  digitalWrite(musicEnablePin, LOW);
  digitalWrite(motorControlPin, LOW);

}

elapsedMillis delay_millis;
uint32_t delay_between_performances = 5000; // ms
uint32_t music_pulse_width = 500; // ms

void loop() {

  openLimitState = digitalRead(limitOpenPin);
  closeLimitState = digitalRead(limitClosePin);

  switch(machineState) {
    case INITIAL_STATE: // initially run until open limit switch is activated
      if (openLimitState == HIGH) {
        digitalWrite(PIN_LED_R, LOW);
        delay(50);
        digitalWrite(PIN_LED_R, HIGH);
        digitalWrite(PIN_LED_G, LOW);
        delay(50);
        digitalWrite(PIN_LED_G, HIGH);
        digitalWrite(PIN_LED_B, LOW);
        delay(50);
        digitalWrite(PIN_LED_B, HIGH);
        digitalWrite(motorControlPin, HIGH);
        digitalWrite(musicEnablePin, LOW);
        break;
      }
      else {
        machineState=WAIT_STATE;
        delay_millis=0;
        break;
      }

	case WAIT_STATE: // waiting for manual close
      if (closeLimitState == HIGH) {
        digitalWrite(PIN_LED_R, LOW);
        digitalWrite(PIN_LED_G, LOW);
        delay(50);
        digitalWrite(PIN_LED_R, HIGH);
        digitalWrite(PIN_LED_G, HIGH);
        delay(50);
        digitalWrite(motorControlPin, LOW);
        digitalWrite(musicEnablePin, LOW);
        break;
      }
      else {
        machineState=HOMING_STATE;
        delay_millis=0;
        break;
      }

    case HOMING_STATE: // run until closed limit switch is activated
      if (closeLimitState == HIGH) {
        digitalWrite(PIN_LED_R, LOW);
        digitalWrite(PIN_LED_G, LOW);
        delay(250);
        digitalWrite(PIN_LED_R, HIGH);
        digitalWrite(PIN_LED_G, HIGH);
        delay(250);
        digitalWrite(motorControlPin, HIGH);
        digitalWrite(musicEnablePin, LOW);
        break;
      }
      else {
        machineState=OPENING_STATE;
        delay_millis=0;
        break;
      }
    case OPENING_STATE: // run until open limit switch is activated
      if (openLimitState == HIGH) {
        if (delay_millis < music_pulse_width) { 
          digitalWrite(musicEnablePin, HIGH);
        }
        else {
          digitalWrite(musicEnablePin, LOW);
        }
        digitalWrite(PIN_LED_G, LOW);
        delay(50);
        digitalWrite(PIN_LED_G, HIGH);
        delay(50);
        digitalWrite(motorControlPin, HIGH);
        break;
      }
      else {
        digitalWrite(motorControlPin, LOW);
        machineState=SPEECH_STATE;

        setup_isr();
        break;
      }
    case SPEECH_STATE:
      if (irq_is_enabled(PWM_IRQ_WRAP)) {
        digitalWrite(PIN_LED_B, LOW);
        delay(50);
        digitalWrite(PIN_LED_B, HIGH);
        delay(50);
        break;
      }
      else {
        machineState=CLOSING_STATE;
        break;
      }
    case CLOSING_STATE:
      if (closeLimitState == HIGH) {
        digitalWrite(PIN_LED_G, LOW);
        delay(250);
        digitalWrite(PIN_LED_G, HIGH);
        delay(50);
        digitalWrite(motorControlPin, HIGH);
        break;
      }
      else {
        digitalWrite(motorControlPin, LOW);
        digitalWrite(musicEnablePin, LOW);
        machineState=DELAY_STATE;
        delay_millis = 0;
        break;
      }
    case DELAY_STATE:
      if (delay_millis < delay_between_performances) {
        digitalWrite(PIN_LED_B, LOW);
        delay(200);
        digitalWrite(PIN_LED_B, HIGH);
        delay(200);
        digitalWrite(PIN_LED_G, LOW);   
        delay(200);
        digitalWrite(PIN_LED_G, HIGH);
        delay(200);
        break;
      }
      else {
        machineState = OPENING_STATE; // start over!
        delay_millis=0;
        break;
      }
    default: // this is an error state - should not hit this ever
      digitalWrite(PIN_LED_R, LOW);
      delay(100);
      digitalWrite(PIN_LED_R, HIGH);
      delay(500);
      break;
  };

}
