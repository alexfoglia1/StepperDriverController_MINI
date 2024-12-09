#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#include "EEPROM_IMG.h"
#include "Maintenance.h"
#include "Board.h"
#include "Motor.h"

#define BTN_MASK(BTN)(1 << BTN)


typedef enum
{
  BTN_1_PRESSED = 0,
  BTN_2_PRESSED,
  BTN_1_RELEASE,
  BTN_2_RELEASE,

  NO_INPUT
} UserEvent;


bool EEPROM_UPDATE = false;
uint8_t buttonsState = 0;

EEPROM_IMG eepromParams;
EEPROM_IMG eepromDefaults = {MAX_VEL_INT, 800, 2000, 0, BUTTON_1, BUTTON_2, 0};
int lastUserInput = 0;

int VEL_TO_STEP_DELAY[MAX_VEL_INT + 1];
int millisStart = 0;
int millisStop = 0;
bool isStepperMoving = false;
bool isMotorSpinup = false;
int  spinupStep = 0;
int  spinupStopStep = 0;


byte checksum(byte* bytes, int nBytes)
{
  byte res = 0;
  for (int i = 0; i < nBytes; i++)
  {
    res ^= bytes[i];
  }

  return res;
}


void writeEepromParams()
{
  byte curChecksum = checksum(eepromParams.Bytes, EEPROM_SIZE - 1);
  eepromParams.Values.checksum = curChecksum;
  for (uint32_t addr = EEPROM_ADDR; addr < EEPROM_ADDR + EEPROM_SIZE; addr++)
  {
    EEPROM.write(addr, eepromParams.Bytes[addr - EEPROM_ADDR]);
  }  
}


void readEepromParams()
{
  eepromDefaults.Values.checksum = checksum(eepromDefaults.Bytes, EEPROM_SIZE - 1);
  for (uint32_t addr = EEPROM_ADDR; addr < EEPROM_ADDR + EEPROM_SIZE; addr++)
  {
    eepromParams.Bytes[addr - EEPROM_ADDR] = EEPROM.read(addr);
  }
  
  byte curChecksum = checksum(eepromParams.Bytes, EEPROM_SIZE - 1);
  if (curChecksum != eepromParams.Values.checksum)
  {
    memcpy(eepromParams.Bytes, eepromDefaults.Bytes, EEPROM_SIZE);
    writeEepromParams();    
  }

  for (int i = 0; i < MAX_VEL_INT + 1; i++)
  {
    VEL_TO_STEP_DELAY[MAX_VEL_INT - i] = map(i, 0, MAX_VEL_INT, eepromParams.Values.minDelayMicros, eepromParams.Values.maxDelayMicros);
  }
}


void readUserButton(int curMillis)
{
  int valueRead = analogRead(BUTTON_IN);

  if (valueRead + BTN_TOL >= eepromParams.Values.btn1 && valueRead - BTN_TOL <= eepromParams.Values.btn1)
  {
    buttonsState = BTN_MASK(1);
    lastUserInput = curMillis;
  }
  else if (valueRead + BTN_TOL >= eepromParams.Values.btn2 && valueRead - BTN_TOL <= eepromParams.Values.btn2)
  {
    buttonsState = BTN_MASK(2);
    lastUserInput = curMillis;
  }
  else
  {
    buttonsState = 0;
  }
}


UserEvent readUserEvent(int curMillis)
{
  uint8_t oldButtonsState = buttonsState;
  readUserButton(curMillis);
  /** BUTTON 1 **/
  if (buttonsState & BTN_MASK(1))
  {
    //lcdPrint("BTN_1_PRESSED");
    return BTN_1_PRESSED;
  }
  else if (oldButtonsState & BTN_MASK(1))
  {
    //lcdPrint("BTN_1_RELEASED");
    return BTN_1_RELEASE;
  }
  /** BUTTON 2 **/
  else if (buttonsState & BTN_MASK(2))
  {
    return BTN_2_PRESSED;
  }
  else if (oldButtonsState & BTN_MASK(2))
  {
    return BTN_2_RELEASE;
  }  
  else
  {
    return NO_INPUT;
  }
  
}


void setup()
{
  Serial.begin(115200);
  
  pinMode(DIR_P, OUTPUT);
  pinMode(DIR_M, OUTPUT);
  pinMode(STEP_P, OUTPUT);
  pinMode(STEP_M, OUTPUT);
  pinMode(PROBE, OUTPUT);

//  motorPower(false);
  digitalWrite(STEP_M, LOW);
  digitalWrite(DIR_M, LOW);
  
  digitalWrite(STEP_P, HIGH);
  digitalWrite(DIR_P, HIGH);
    
  readEepromParams();
  
  if (eepromParams.Values.curDirection)
  {
    PORTB |= DIR_M_MASK;
  }
  else
  {
    PORTB &= ~DIR_M_MASK;
  }
}



int lastPot = -1;
void loop()
{
  long t0 = micros();
  int curMillis = (int)(t0 / 1000L);

  if (isMotorSpinup)
  {
    if (spinupStep < spinupStopStep)
    {
      int vel = map(spinupStep, 0, 1023, 0, MAX_VEL_INT);
      int stepDelay = VEL_TO_STEP_DELAY[vel];
      motorStep(stepDelay);

      //if (curMillis - millisStart > 20)
      //{
      //  millisStart = curMillis;
        spinupStep += 1;
      //}
    }
    else
    {
      isMotorSpinup = false;
      isStepperMoving = true;
    }
  }
  if (isStepperMoving)
  {
    int aIn = analogRead(POT_PIN);
    if (lastPot == -1 || (abs(lastPot - aIn) < 100))
    {
      lastPot = aIn;
    }
    int vel = map(lastPot, 0, 1023, 0, MAX_VEL_INT);
    int stepDelay = VEL_TO_STEP_DELAY[vel];
    //char prompt[255];
    //sprintf(prompt, "aIn(%d) vel(%d) stepDelay(%d)\n", aIn, vel, stepDelay);
    //Serial.println(prompt);
    motorStep(stepDelay);
  }
  else
  {
    lastPot = -1;
#ifdef BOARD_REVB    
    motorPower(false);
#endif
    isStepperMoving = false;
  }
  
  UserEvent e = readUserEvent(curMillis);
  
  if (e == BTN_1_RELEASE && !isMotorSpinup && !isStepperMoving)
  {
    isMotorSpinup = true;
    spinupStopStep = analogRead(POT_PIN);
    spinupStep = max(spinupStopStep - 512, 0);
    //Serial.print(spinupStep); Serial.print(" "); Serial.println(spinupStopStep);
    millisStart = curMillis;
#ifdef BOARD_REVB    
    motorPower(true);
#endif
  }
  else if (e == BTN_2_PRESSED && isStepperMoving)
  {
    isStepperMoving = false;
    isMotorSpinup = false;
    spinupStep = 0;
#ifdef BOARD_REVB    
    motorPower(false);
#endif  
  }

  if (EEPROM_UPDATE && !isStepperMoving)
  {
    writeEepromParams();
    EEPROM_UPDATE = false;
  }
  
  MAINT_Handler(&EEPROM_UPDATE, &isStepperMoving);
}
