#include<avr/io.h>

volatile uint32_t timer0_millis = 0;
uint32_t last_tick = 0;



uint16_t start_ticks, end_ticks;

uint32_t total_ticks;



const uint8_t FIXED_ANGLE = 90;  

float smoothAngle = 90.0;      



float raw_distance = 0;

float Unfiltered_arr[5];

float filtered_arr[5];

int pointer = 0;

int N = 4;

float final_ultrasonic_reading = 0;



const float target_distance = 12.415;

const float kp = 1.2;  

const float ki = 0.5;

const float kd = 1.3;

float integral = 0;

float lastError = 0;

float lastTime = 0;


ISR(TIMER0_COMPA_vect) {

    timer0_millis++;

}



float ultrasonic();

float calculatePID(float distance);

float calculateAngle(float pid_output);

void setFixedAngle(uint8_t angle);

void insertionsort(float arr[], int n);

void init_millis();

uint32_t get_millis();


int main(){

  init_millis();



  DDRB |= (1 << PB1);//servo pin

  DDRD |= (1 << PD6);//trig

  DDRD &= ~(1 << PD5);//echo


  TCCR1A = (1 << COM1A1) | (1 << WGM11);

  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

  ICR1 = 39999;


  setFixedAngle(FIXED_ANGLE);

 

  for(int i = 0; i <= N ; i++) {

    Unfiltered_arr[i] = target_distance;

  }

  while(1){
    uint32_t current_tick = get_millis();

        if (current_tick - last_tick >= 40) {

             last_tick = current_tick;

              raw_distance = ultrasonic();
              Unfiltered_arr[pointer] = raw_distance;
              pointer = (pointer + 1) % 5;

              for (int i = 0; i <= N; i++) {

                filtered_arr[i] = Unfiltered_arr[i];

              }

              insertionsort(filtered_arr, 5);
              final_ultrasonic_reading = filtered_arr[2];
              float pid_val = calculatePID(final_ultrasonic_reading);
              float targetAngle = calculateAngle(pid_val);
              smoothAngle = (smoothAngle * 0.8) + (targetAngle * 0.2);
              setFixedAngle((uint8_t)smoothAngle);
        }
  }
return 0;

}

float ultrasonic() {
  PORTD &= ~(1 << PD6);

  for(volatile uint8_t i = 0; i < 6; i++) {}

  PORTD |= (1 << PD6);

  for(volatile uint8_t i = 0; i < 35; i++) {}

  PORTD &= ~(1 << PD6);
     uint32_t timeout = 60000;
    //wait for the echoo para mo state of highhhz

    while (!(PIND & (1 << PIND5))) {

        if (--timeout == 0) return 12.4;

    }


    start_ticks = TCNT1;

    //wait for the echoo para mo state of lowzz

    timeout = 60000;

    while (PIND & (1 << PIND5)) {

        if (--timeout == 0) break;

    }

    end_ticks = TCNT1;

    if (end_ticks >= start_ticks) {

        total_ticks = end_ticks - start_ticks;

    } else {

        total_ticks = (40000 - start_ticks) + end_ticks;

    }


  float reading = (total_ticks * 0.5 * 0.0343) / 2.0;



  reading = reading - 1.5;

 

  if(reading < 1.5 || reading > 26.50) {

    reading = target_distance;

  }


  return reading;

}


float calculatePID(float distance) {
    float dt_fixed = 0.04; 
    float error = target_distance - distance;
    
    // --- NEW: THE "STOP" LOGIC ---
    // Check if we are in your desired range (12.1 to 12.8)
    if (distance >= 9.5 && distance <= 11.8) {
        error = 0;       // Stop the Proportional and Derivative terms
        integral = 0;  
        //  dt_fixed = 0;   // WIPE the memory so the beam doesn't "creep"
    } else {
        // Only accumulate integral if we are OUTSIDE the target
        integral += error * dt_fixed;
    }
    // -----------------------------

    // Tighten Anti-windup (prevent the value from getting too huge)
    if(integral < -10) integral = -10;
    if(integral > 10)  integral = 10;
    
    float derivative = (error - lastError) / dt_fixed;
    lastError = error;
    
    return (kp * error) + (ki * integral) + (kd * derivative);
}


float calculateAngle(float pid_output) {

 float angle = 90.0 - pid_output;

if(angle < 30)  angle = 30;

if(angle > 150) angle = 150;
 
  return angle;

}


void setFixedAngle(uint8_t angle) {
  uint16_t ticks = (uint16_t)(( (uint32_t)(angle - 30) * 4200L / 120L ) + 800L);
  OCR1A = ticks;

}


void insertionsort(float arr[], int n) {

  for (int i = 1; i < n; i++) {

    float key = arr[i];

    int j = i - 1;

    while (j >= 0 && arr[j] > key) {

      arr[j + 1] = arr[j];

      j--;

    }

    arr[j + 1] = key;

  }

}

void init_millis() {

    // I Set CTC mode Clear Timer on Compare Match

    TCCR0A = (1 << WGM01);

    // Ang Set Prescaler to  64

    TCCR0B = (1 << CS01) | (1 << CS00);

    // Set compare match value for 1ms (250 ticks)

     OCR0A = 249;

    // compare

    TIMSK0 |= (1 << OCIE0A);

    sei();

}

uint32_t get_millis() {

    uint32_t millis_copy;
    uint8_t sreg = SREG;

    cli();

  
    millis_copy = timer0_millis;

    SREG = sreg;

    return millis_copy;}