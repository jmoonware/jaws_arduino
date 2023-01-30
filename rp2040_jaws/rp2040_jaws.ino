pin_size_t audioPin = D0; // D0 = PWM5A = GPI26
pin_size_t motorPin = D2;  // D2 = PWM6A = GPIO28
pin_size_t irqPin = D4; // program D9 = PWM2A = GPIO4 for IRQ
pin_size_t debugPin = D6; // jjust a pin to send signals for debugging


#include <hardware/pwm.h>
#include <hardware/irq.h>
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

#define STOP_AFTER 1 // turn off interrupts after this many iterations

static uint32_t audio_buffer_position = 0;
static uint32_t motor_update_count = 0;
static uint32_t motor_buffer_position = 0;
static uint32_t stop_after = 0;

uint8_t audioSlice;
uint8_t irqSlice;
uint8_t motorSlice;

void isr_audio() {

  if (pwm_get_irq_status_mask()&(1<<irqSlice)) {
    pwm_clear_irq(irqSlice);
//    digitalWrite(debugPin, HIGH); 
//    digitalWrite(debugPin, LOW);
  }

//  digitalWrite(irqPin, HIGH);

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

void setup() {

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);

  digitalWrite(PIN_LED_R, HIGH); // HIGH turns off
  digitalWrite(PIN_LED_B, HIGH);

  gpio_set_function(audioPin, GPIO_FUNC_PWM);
//  pinMode(irqPin, OUTPUT);
  pinMode(debugPin, OUTPUT);
  gpio_set_function(irqPin, GPIO_FUNC_PWM);
  gpio_set_function(motorPin, GPIO_FUNC_PWM);

  // simple test pattern
//  for (uint32_t i=0; i < AUDIO_SAMPLES; ++i) {
//    audio_buffer[i]=(uint8_t)(200*(float)i/(float)AUDIO_SAMPLES);
//  }

  // full range sweep of motor
//  for (uint32_t i=0; i < MOTOR_SAMPLES; ++i) {
//    motor_buffer[i]=(uint16_t)(MOTOR_MIN + ((float)(MOTOR_MAX-MOTOR_MIN)/(float)MOTOR_SAMPLES)*i);
//  }

  // "slice" is a weird name for "Counter Number" - there are 8 16 bit counters (0-7), each having two channels (A=0,B=1) supporting two outputs with different CC values
  audioSlice = pwm_gpio_to_slice_num(audioPin);
  irqSlice = pwm_gpio_to_slice_num(irqPin);
  motorSlice = pwm_gpio_to_slice_num(motorPin);

  // audio counter (slice) - creates PWM audio signal
  pwm_config audioConfig = pwm_get_default_config();
  pwm_config_set_wrap(&audioConfig, AUDIO_TOP); // this many audio resolution values  

  // irqCounter; just generates an interrupt on every audio sample
  pwm_config irqConfig = pwm_get_default_config();
  pwm_config_set_wrap(&irqConfig, IRQ_TOP); // number of clock cycles to update audio values  
  // the irq counter will trigger the isr routine

  irq_add_shared_handler(PWM_IRQ_WRAP, isr_audio,PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  // motor counter (slice) - creates PWM signal for motor
  pwm_config motorConfig = pwm_get_default_config();
  pwm_config_set_wrap(&motorConfig, MOTOR_TOP); // with prescaling, gets to 50 Hz  
  
  pwm_init(audioSlice, &audioConfig, true);
  pwm_set_chan_level(audioSlice, 0, 100);

  pwm_init(motorSlice, &motorConfig, true);
  pwm_set_chan_level(motorSlice, 0, 500);
  pwm_set_clkdiv_int_frac(motorSlice, MOTOR_DIV, 0);

  pwm_init(irqSlice, &irqConfig, true);
  pwm_set_chan_level(irqSlice, 0, 100); // just something to look at on scope

  pwm_set_irq_enabled(audioSlice,false);
  pwm_clear_irq(audioSlice);
  pwm_set_irq_enabled(motorSlice,false);
  pwm_clear_irq(motorSlice);
  pwm_set_irq_enabled(irqSlice, true); 

}

void loop() {

  digitalWrite(PIN_LED_G, HIGH);
  delay(250);
  digitalWrite(PIN_LED_G, LOW);
  delay(250);
  delay(10000);

}
