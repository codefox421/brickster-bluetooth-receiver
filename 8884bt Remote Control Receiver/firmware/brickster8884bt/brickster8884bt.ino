/*------------------------------------------------------------------------------
  The MIT License (MIT)

  Copyright (c) 2013 Nick Iaconis et al.
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software, schematic, layout and associated documentaion (the
  "System"), to deal in the System without restriction, including without
  limitation the rights to use, copy, modify, merge, publish, distribute,
  sublicense, and/or sell copies of the System, and to permit persons to whom
  the System is furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the System.
  
  THE SYSTEM IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SYSTEM OR THE USE OR OTHER DEALINGS IN THE
  SYSTEM.
------------------------------------------------------------------------------*/

/*-------------------
  #### ATtiny841 ####
  -------------------
         ____
  VCC  <|*   |> GND
  IO 0 <|    |> IO10
  IO 1 <|    |> IO 9
  RST  <|    |> IO 8
  IO 2 <|    |> IO 7
  IO 3 <|    |> IO 6
  IO 4 <|____|> IO 5

  -------------------*/





/*------------------------------------------------------------------------------

  Override core timers

------------------------------------------------------------------------------*/


const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
{
  NOT_ON_TIMER,
  NOT_ON_TIMER,
  TIMER0A, /* TOCC7 - this is the LED function so let's give it the least desirable timer */
  TIMER2B, /* TOCC6 - this is the AUX function so let's arbitrarily give it TIMER2 */
  TIMER2A, /* TOCC5 - this is the AUX function so let's arbitrarily give it TIMER2 */
  TIMER0B, /* TOCC4 - this is the LED function so let's give it the least desirable timer */
  TIMER1A, /* TOCC3 - this is the MOTOR function so let's arbitrarily give it TIMER1 */
  TIMER1B, /* TOCC2 - this is the MOTOR function so let's arbitrarily give it TIMER1 */
  NOT_ON_TIMER, //This could have pwm, but it's serial 0, and we only get PWM on 6 of the 8 pins at once.
  NOT_ON_TIMER, //see above.
  NOT_ON_TIMER,
  NOT_ON_TIMER,
};

// ATtiny841 Datasheet Table 12-7, TOCCn pins can be assigned various OCnX.
#define CORE_OC0A_PIN  PIN_B2 // TOCC7
#define CORE_OC0B_PIN  PIN_A5 // TOCC4
#define CORE_OC1A_PIN  PIN_A4 // TOCC3
#define CORE_OC1B_PIN  PIN_A3 // TOCC2
#define CORE_OC2A_PIN  PIN_A6 // TOCC5
#define CORE_OC2B_PIN  PIN_A7 // TOCC6

#define CORE_PWM0_PIN       CORE_OC0A_PIN
#define CORE_PWM0_TIMER     0
#define CORE_PWM0_CHANNEL   A

#define CORE_PWM1_PIN       CORE_OC0B_PIN
#define CORE_PWM1_TIMER     0
#define CORE_PWM1_CHANNEL   B

#define CORE_PWM2_PIN       CORE_OC1A_PIN
#define CORE_PWM2_TIMER     1
#define CORE_PWM2_CHANNEL   A

#define CORE_PWM3_PIN       CORE_OC1B_PIN
#define CORE_PWM3_TIMER     1
#define CORE_PWM3_CHANNEL   B

#define CORE_PWM4_PIN       CORE_OC2A_PIN
#define CORE_PWM4_TIMER     2
#define CORE_PWM4_CHANNEL   A

#define CORE_PWM5_PIN       CORE_OC2B_PIN
#define CORE_PWM5_TIMER     2
#define CORE_PWM5_CHANNEL   B



/*------------------------------------------------------------------------------

  Definitions

------------------------------------------------------------------------------*/


// Motor control A
#define mcPin1A 6
#define mcPin2A 7
#define maxA 28
char levelA = 0;
int pwmA = 0;

// Motor control B
#define mcPin1B 3
#define mcPin2B 4
#define maxB 28
char levelB = 0;
int pwmB = 0;

// Channel A commands
#define maskA 83
#define upA 65    // A up         = A
#define downA 66  // A down       = B
#define brakeA 67 // A brake      = C
#define floatA 80 // A float      = P
#define ffwdA 81  // A full fwd   = Q
#define frevA 82  // A full rev   = R

// Channel B commands
#define maskB 108
#define upB 68    // B up         = D
#define downB 72  // B down       = H
#define brakeB 76 // B brake      = L
#define floatB 96 // B float      = `
#define ffwdB 100 // B full fwd   = d
#define frevB 104 // B full rev   = h

// Combine Channel A command with Channel B command using logical OR

// Hardware serial
#define baudRate 9600

// Miscellanious
#define btResetPin 2
#define btConnPin 1
char incomingByte = 0;
char maskedByte = 0;
int pwmLevels[29] = {0, 41,  49,  57,  64, \
                        73,  81,  89,  97, \
                       105, 113, 120, 128, \
                       136, 144, 152, 160, \
                       168, 175, 184, 192, \
                       200, 208, 215, 223, \
                       231, 239, 247, 255};

// Function definitions
void comboMode(const char,      char*, const char, const char, \
               const char, const char, const char, const char, \
               const char, const char, const char, const char);



/*------------------------------------------------------------------------------

  setup

------------------------------------------------------------------------------*/


void setup()
{
  // serial baudrate init
  Serial.begin(baudRate);

  // bluetooth control init
  pinMode(btResetPin, OUTPUT);
  digitalWrite(btResetPin, HIGH); // end Bluetooth reset (enable module)

  // bluetooth connectivity init
  pinMode(btConnPin, INPUT);
  Serial.write("AT+LED1");

  // channel A init
  pinMode(mcPin1A, OUTPUT);
  pinMode(mcPin2A, OUTPUT);
  digitalWrite(mcPin1A, HIGH);
  digitalWrite(mcPin2A, HIGH);

  // channel B init
  pinMode(mcPin1B, OUTPUT);
  pinMode(mcPin2B, OUTPUT);
  digitalWrite(mcPin1B, HIGH);
  digitalWrite(mcPin2B, HIGH);
}



/*------------------------------------------------------------------------------

  loop

------------------------------------------------------------------------------*/


void loop()
{
  // Handle serial communication
  if (Serial.available() > 0)
  {
    // read incoming byte
    incomingByte = Serial.read();


    // Direct PWM mode (fine resolution)
    if (incomingByte & 128) // 1xxx xxxx
    {
      // select proper output: x0xx xxxx -> A / x1xx xxxx -> B
      char* levelX = incomingByte & 64 ? &levelB : &levelA;
      if ((incomingByte & 63) == 0) // xx00 0000 -> float
      {
        // set the level
        *levelX = 0;
        // set the motor control wires
        digitalWrite(incomingByte & 64 ? mcPin1B : mcPin1A, HIGH);
        digitalWrite(incomingByte & 64 ? mcPin2B : mcPin2A, HIGH);
      }
      // xx01 1110, xx01 1111, xx10 0001, xx10 0010  are reserved
      else if ((incomingByte & 63) < 29 || (incomingByte & 63) > 35)
      {
        // set the level (xx1x xxxx are negative and need top 3 bits filled)
        *levelX = (incomingByte & 31) | (incomingByte & 32 ? 224 : 0);
        // motor control wires set at end of input handling for non-stopped levels
      }
      else if ((incomingByte & 63) == 32) // xx10 0000 -> brake
      {
        // set the level
        *levelX = 0;
        // set the motor control wires
        digitalWrite(incomingByte & 64 ? mcPin1B : mcPin1A, LOW);
        digitalWrite(incomingByte & 64 ? mcPin2B : mcPin2A, LOW);
      }
      // notify sender
      Serial.write(*levelX);
    }

    // Combo output mode
    else if (incomingByte & 64) // 01xx xxxx
    {
      // set channel A
      comboMode(incomingByte, &levelA, maxA, maskA, brakeA, floatA, \
                  upA, downA, ffwdA, frevA, mcPin1A, mcPin2A);
      // set channel B
      comboMode(incomingByte, &levelB, maxB, maskB, brakeB, floatB, \
                  upB, downB, ffwdB, frevB, mcPin1B, mcPin2B);
      // notify sender
      maskedByte = abs(levelB >> 2) + (levelB < 0 ? 8 : 0);
      maskedByte = (maskedByte << 4) + abs(levelA >> 2) + (levelA < 0 ? 8 : 0);
      Serial.write(maskedByte);
    }

    // set the pwm
    pwmA = pwmLevels[min(abs(levelA), maxA)];
    pwmB = pwmLevels[min(abs(levelB), maxB)];

    if (pwmA > 0)
    {
      // set the motor control wires (LOW input is HIGH output)
      analogWrite(mcPin1A, (levelA > 0 ? (255 - pwmA) : 255));
      analogWrite(mcPin2A, (levelA < 0 ? (255 - pwmA) : 255));
    }

    if (pwmB > 0)
    {
      // set the motor control wires (LOW input is HIGH output)
      analogWrite(mcPin1B, (levelB > 0 ? (255 - pwmB) : 255));
      analogWrite(mcPin2B, (levelB < 0 ? (255 - pwmB) : 255));
    }
  }
}



/*------------------------------------------------------------------------------

  comboMode handler

------------------------------------------------------------------------------*/



void comboMode(const char incomingByte,      char*  levelX, const char    maxX, \
               const char        maskX, const char  brakeX, const char  floatX, \
               const char          upX, const char   downX, const char   ffwdX, \
               const char        frevX, const char mcPin1X, const char mcPin2X)
{
  maskedByte = incomingByte & maskX;
  // decode command
  if (maskedByte == brakeX || maskedByte == floatX)
  {
    *levelX = 0;
    // set the motor control wires (brake LOW, float HIGH)
    digitalWrite(mcPin1X, (maskedByte == brakeX ? LOW : HIGH));
    digitalWrite(mcPin2X, (maskedByte == brakeX ? LOW : HIGH));
  }
  else
  {
    if (maskedByte == upX)
    {
      if (*levelX >= 0)
      {
        *levelX = min((*levelX + 4) & ~3, maxX);
      }
      else
      {
        if (abs(*levelX) & 3)
          *levelX = -(abs(*levelX) & ~3);
        else
          *levelX += 4;
      }
    }
    else if (maskedByte == downX)
    {
      if (*levelX <= 0)
      {
        *levelX = -min((abs(*levelX) + 4) & ~3, maxX);
      }
      else
      {
        if (*levelX & 3)
          *levelX &= ~3;
        else
          *levelX -= 4;
      }
    }
    else if (maskedByte == ffwdX)
    {
      *levelX = maxX;
    }
    else if (maskedByte == frevX)
    {
      *levelX = -maxX;
    }

    // we have stepped back to float
    if (*levelX == 0)
    {
      digitalWrite(mcPin1X, HIGH);
      digitalWrite(mcPin2X, HIGH);
    }
  }

  // motor control wires set at end of input handling for non-stopped levels
}
