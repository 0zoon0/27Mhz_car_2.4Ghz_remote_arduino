#include "iface_nrf24l01.h"

// for debugging
//#define ENABLE_SERIAL_COM

//SPI Comm pins with nRF24L01
#define MOSI_pin  11  
#define SCK_pin   13
#define CE_pin    10  
#define MISO_pin  12 
#define CS_pin    9 

#define LED_pin    13 // LED  - D13
#define HEADLIGHT1_pin A0
#define HEADLIGHT2_pin A1

//motor pins
#define MOTOR1_FW 8
#define MOTOR1_BW 7
#define MOTOR2_FW 6
#define MOTOR2_BW 5
#define MOTOR3_DW 4
#define MOTOR3_UP 3
#define MOTOR4 2

// SPI outputs
#define MOSI_on PORTB |= _BV(3)  
#define MOSI_off PORTB &= ~_BV(3)
#define SCK_on PORTB |= _BV(5)   
#define SCK_off PORTB &= ~_BV(5) 
#define CE_on PORTB |= _BV(2)   
#define CE_off PORTB &= ~_BV(2) 
#define CS_on PORTB |= _BV(1)   
#define CS_off PORTB &= ~_BV(1) 
// SPI input
#define  MISO_on (PINB & _BV(4))

#define RF_POWER TX_POWER_5mW 

// motor statuses
#define M_STOP 0
#define M_FORWARD 1
#define M_BACKWARD 2

// empty packages threshold
#define EMPTY_PKG_THR 1000

struct TrxData {
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  char invert;
  char flip;
  char expert;
  char headfree;
  char rth;
  char trim1;
};
TrxData data;

uint8_t transmitterID[2];
uint8_t packet[15];
static volatile uint8_t m1NewStatus, m2NewStatus, m3NewStatus, m4NewStatus;
uint8_t m1CurrentStatus, m2CurrentStatus, m3CurrentStatus, m4CurrentStatus;
uint16_t emptyPacketsCount;
bool headLightsOn;
char trim1Default;
void setup()
{
    pinMode(LED_pin, OUTPUT);
    pinMode(HEADLIGHT1_pin, OUTPUT);
    pinMode(HEADLIGHT2_pin, OUTPUT);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);
    pinMode(MOTOR1_FW, OUTPUT);
    pinMode(MOTOR1_BW, OUTPUT);
    pinMode(MOTOR2_FW, OUTPUT);
    pinMode(MOTOR2_BW, OUTPUT);
    pinMode(MOTOR3_DW, OUTPUT);
    pinMode(MOTOR3_UP, OUTPUT);
    pinMode(MOTOR4, OUTPUT);

    digitalWrite(LED_pin, LOW); 
    //turn on headlights so that end user knows the car is on
    digitalWrite(HEADLIGHT1_pin, HIGH);
    digitalWrite(HEADLIGHT2_pin, HIGH);
    digitalWrite(MOTOR1_FW, LOW);
    digitalWrite(MOTOR1_BW, LOW);
    digitalWrite(MOTOR2_FW, LOW);
    digitalWrite(MOTOR2_BW, LOW);
    digitalWrite(MOTOR3_DW, LOW);
    digitalWrite(MOTOR3_UP, LOW);
    digitalWrite(MOTOR4, LOW);
    
#ifdef ENABLE_SERIAL_COM    
    Serial.begin(115200);
    Serial.println("starting..."); 
#endif
    
    NRF24L01_Reset();
    NRF24L01_Initialize();
    Bayang_init();
    Bayang_bind();

    //To create timer that controls the motors, runs every 30ms
    //This subroutine is called TIMER2_COMPA_vect
    TCCR2A = 0;                    //Make sure that the TCCR2A register is set to zero
    TCCR2B = 0;                    //Make sure that the TCCR2A register is set to zero
    TIMSK2 |= (1 << OCIE2A);       //Set the interupt enable bit OCIE2A in the TIMSK2 register
    TCCR2B |=  (1 << CS22) | (1 << CS21) | (1 << CS20);  //Set prescaller 1024
    OCR2A = 233;                    //The compare register is set to 233 => 30ms / (1s / (8.000.000Hz / 1024)) - 1
    TCCR2A |= (1 << WGM21);        //Set counter 2 to CTC (clear timer on compare) mode

    m1NewStatus = M_STOP;
    m2NewStatus = M_STOP;
    m3NewStatus = M_STOP;
    m4NewStatus = M_STOP;
    m1CurrentStatus = M_STOP;
    m2CurrentStatus = M_STOP;
    m3CurrentStatus = M_STOP;
    m4CurrentStatus = M_STOP;
    
    //turn off headlights after a connection to transmiter is established
    digitalWrite(HEADLIGHT1_pin, LOW);
    digitalWrite(HEADLIGHT2_pin, LOW);
    headLightsOn = false;
}

void loop()
{
    // read next packet
    Bayang_recv_packet(&data);
    
    if (data.roll==0 && data.pitch==0 && data.yaw==0 && data.throttle==0) {
      // bad link
      emptyPacketsCount++;
      if (emptyPacketsCount >= EMPTY_PKG_THR) {
        m1NewStatus = M_STOP;
        m2NewStatus = M_STOP;
        m3NewStatus = M_STOP;
        m4NewStatus = M_STOP;
        emptyPacketsCount = EMPTY_PKG_THR;
        //turn on the headlights so that end user knows there is a problem with the link
        digitalWrite(HEADLIGHT1_pin, HIGH);
        digitalWrite(HEADLIGHT2_pin, HIGH);
        headLightsOn = true;
      }
    }
    else 
    {
      emptyPacketsCount = 0;

      // Expert mode is used to go in reverse      
      if (data.yaw < 200) //turn left
      {
        m1NewStatus = M_STOP;
        if (!data.expert)
          m2NewStatus = M_FORWARD;
        else 
          m2NewStatus = M_BACKWARD;
      }
      else if (data.yaw > 800) //turn right
      {
        m2NewStatus = M_STOP;
        if (!data.expert)
          m1NewStatus = M_FORWARD;
        else 
          m1NewStatus = M_BACKWARD;        
      }
      else if (data.throttle > 600) //go straight
      {
        if (!data.expert) 
        {
          m1NewStatus = M_FORWARD;
          m2NewStatus = M_FORWARD;
        }
        else 
        {
          m1NewStatus = M_BACKWARD;
          m2NewStatus = M_BACKWARD;
        }
      }
      else //stop movement
        {
          m1NewStatus = M_STOP;
          m2NewStatus = M_STOP;
        }

      //check pitch value, if up - make m3 go FW, if down - make m3 go BW
      //if middle stop m3
      if (data.pitch > 800)
        m3NewStatus = M_FORWARD;
      else if (data.pitch < 200)
        m3NewStatus = M_BACKWARD;
      else
        m3NewStatus = M_STOP;

      //make m4 fire
      if (data.headfree)
        m4NewStatus = M_FORWARD;
      else
        m4NewStatus = M_STOP;

      //read initial trim1 value as default
      if (trim1Default == 0)
      {
        trim1Default = data.trim1;    
      }

      //turn on headlights if trim1 value is different than default
      if (data.trim1 != trim1Default)
      {
        if (!headLightsOn)
        {
          digitalWrite(HEADLIGHT1_pin, HIGH);
          digitalWrite(HEADLIGHT2_pin, HIGH);
          headLightsOn = true;        
        }
      }
      else
      {
        if (headLightsOn)
        {
          digitalWrite(HEADLIGHT1_pin, LOW);
          digitalWrite(HEADLIGHT2_pin, LOW);
          headLightsOn = false;
        }
      }
   }
}

//run motors depending on NewStatus variables
ISR(TIMER2_COMPA_vect)
{
  motorControl(m1NewStatus, &m1CurrentStatus, MOTOR1_FW, MOTOR1_BW);
  motorControl(m2NewStatus, &m2CurrentStatus, MOTOR2_FW, MOTOR2_BW);
  motorControl(m3NewStatus, &m3CurrentStatus, MOTOR3_UP, MOTOR3_DW);

  //m4 motor rotates one direction
  if (m4NewStatus == M_FORWARD)
  {
    if (m4CurrentStatus != M_FORWARD)
    {
      digitalWrite(MOTOR4, HIGH);
      m4CurrentStatus = M_FORWARD;
    }
  }
  else 
  {
    if (m4CurrentStatus != M_STOP)
    {
      digitalWrite(MOTOR4, LOW);
      m4CurrentStatus = M_STOP;
    }
  }
}

void motorControl(uint8_t newStatus, uint8_t *currentStatus, uint8_t pinForward, uint8_t pinBackward)
{
  if (newStatus == M_FORWARD)
  {
    if (*currentStatus == M_BACKWARD)
    {
      digitalWrite(pinBackward, LOW);
    }
    
    if (*currentStatus != M_FORWARD)
    {
      delay(25);
      digitalWrite(pinForward, HIGH);
      *currentStatus = M_FORWARD;
    }
  }
  else if (newStatus == M_BACKWARD)
  {
    if (*currentStatus == M_FORWARD)
    {
      digitalWrite(pinForward, LOW);
    }

    if (*currentStatus != M_BACKWARD)
    {
      delay(25);
      digitalWrite(pinBackward, HIGH);
      *currentStatus = M_BACKWARD; 
    }
  }
  else
  {
    if (*currentStatus == M_FORWARD)
      digitalWrite(pinForward, LOW);
    if (*currentStatus == M_BACKWARD)
      digitalWrite(pinBackward, LOW);
    *currentStatus = M_STOP; 
  }
}


