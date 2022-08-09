#include <Arduino.h>
#include <AltSoftSerial.h>


AltSoftSerial Serial2;
#define RXD2 16 // PIN RX2
#define TXD2 17 // PIN TX2

// defines
// special characters used for msg framing
#define ACK 0x06  // ACKNOWLEDGE
#define CAN 0x18  // ACKNOWLEDGE
#define ENQ 0x05  // ASCII 0x05 == ENQUIRY
#define STX 0x02  // START OF TEXT
#define ETX 0x03  // END OF TEXT
#define EOT 0x04  // END OF TRANSMISSION
#define LF 0x0A   // LINEFEED
#define NUL 0x00  // NULL termination character
#define XOFF 0x13 // XOFF Pause transmission
#define XON 0x11  // XON Resume transmission
#define DLE 0x10  // DATA LINE ESCAPE
#define CR 0x0D   // CARRIAGE RETURN \r
#define LF 0x0A   // LINE FEED \n

#define CHAR_REGREAD 0x52
#define CHAR_LOGREAD 0x4C
//
#define MSG_INTERVAL 5000            // send an enquiry every 5-sec
#define CHAR_TIMEOUT 2000            // expect chars within a second (?)
#define BUFF_SIZE 20                 // buffer size (characters)
#define MAX_RX_CHARS (BUFF_SIZE - 2) // number of bytes we allow (allows for NULL term + 1 byte margin)
#define SERIALNUM_CHAR 10
#define KWH_CHAR 4
#define KWH_OUT_CHAR 16

#define LED_BLIP_TIME 300 // mS time LED is blipped each time an ENQ is sent

// state names
#define ST_CON 0
#define ST_LOGIN 1
#define ST_LOGIN_R 2
#define ST_READ 3
#define ST_RX_READ 4
#define ST_LOGOUT 5
#define ST_MSG_TIMEOUT 11

// state reg
#define ST_REG_CON 0
#define ST_REG_SERIAL 1
#define ST_REG_KWH_A 2
#define ST_REG_KWH_B 3
#define ST_REG_KWH_T 4
#define ST_REG_END 5

// constants
// pins to be used may vary with your Arduino for software-serial...
const byte pinLED = LED_BUILTIN; // visual indication of life

unsigned char logonReg[15] = {0x4C, 0x45, 0x44, 0x4D, 0x49, 0x2C, 0x49, 0x4D, 0x44, 0x45, 0x49, 0x4D, 0x44, 0x45, 0x00};
unsigned char logoffReg[1] = {0x58};

byte regSerNum[2] = {0xF0, 0x02};
byte regKWHA[2] = {0x1E, 0x01};
byte regKWHB[2] = {0x1E, 0x02};
byte regKWHTOT[2] = {0x1E, 0x00};

unsigned int interval = 5000;
long previousMillis = 0;
// globals
char
    rxBuffer[14];
unsigned char
    dataRX[BUFF_SIZE];
char
    hexNow;

bool
    bLED;
unsigned long
    timeLED;

char outSerialNumber[SERIALNUM_CHAR];
char outKWHA[KWH_OUT_CHAR];
char outKWHB[KWH_OUT_CHAR];
char outKWHTOT[KWH_OUT_CHAR];

void ReceiveStateMachine(void);
bool GetSerialChar(char *pDest, unsigned long *pTimer);
bool CheckTimeout(unsigned long *timeNow, unsigned long *timeStart);

short gencrc_16(short i)
{
  short j;
  short k;
  short crc;
  k = i << 8;
  crc = 0;
  for (j = 0; j < 8; j++)
  {
    if ((crc ^ k) & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
    k <<= 1;
  }
  return (crc);
}

unsigned short
CalculateCharacterCRC16(unsigned short crc, unsigned char c)
{
  return ((crc << 8) ^ gencrc_16((crc >> 8) ^ c));
}

// SEND TO KWH
void cmdlink_putch(unsigned char ch)
{
  Serial2.write(ch);
}

void send_byte(unsigned char d)
{
  switch (d)
  {
  case STX:
  case ETX:
  case DLE:
  case XON:
  case XOFF:
    cmdlink_putch(DLE);
    cmdlink_putch(d | 0x40);
    break;
  default:
    cmdlink_putch(d);
  }
}

void send_cmd(unsigned char *cmd, unsigned short len)
{
  unsigned short i;
  unsigned short crc;
  /*
     Add the STX and start the CRC calc.
  */
  cmdlink_putch(STX);
  crc = CalculateCharacterCRC16(0, STX);
  /*
     Send the data, computing CRC as we go.
  */
  for (i = 0; i < len; i++)
  {
    send_byte(*cmd);
    crc = CalculateCharacterCRC16(crc, *cmd++);
  }
  /*
     Add the CRC
  */
  send_byte((unsigned char)(crc >> 8));
  send_byte((unsigned char)crc);
  /*
     Add the ETX
  */
  cmdlink_putch(ETX);
}

bool cmd_trigMeter()
{
  cmdlink_putch(STX);
  cmdlink_putch(ETX);
  return true;
}

bool cmd_logonMeter()
{
  send_cmd(logonReg, sizeof(logonReg));
  return true;
}

bool cmd_logoffMeter()
{
  send_cmd(logoffReg, sizeof(logoffReg));
  return true;
}

void cmd_readRegister(const byte *reg)
{
  unsigned char readReg[3] = {0x52, reg[0], reg[1]};
  send_cmd(readReg, sizeof(readReg));
}

short get_char(void)
{
  short rx;
  if (Serial2.available())
  {
    // Serial.println("get_char");
    rx = Serial2.read();
    // delay(500);
    return rx;
  }
  else
  {
    Serial.println("no get_char");
    return (-1);
  }
}

bool get_cmd(unsigned char *cmd_data, unsigned short *len, unsigned short max_len)
{
  short c;
  static unsigned char *cur_pos = 0;
  static unsigned short crc;
  static char DLE_last;
  int cntreg = 0;
  /*
   * check is cur_pos has not been initialised yet.
   */
  if (!cur_pos)
  {
    cur_pos = cmd_data;
    *len = 0;
  }
  /*
   * Get characters from the serial port while they are avialable
   */
  while ((c = get_char()) != -1)
  {
    switch (c)
    {
    case STX:
      Serial.println("STX");
      cur_pos = cmd_data;
      *len = 0;
      crc = CalculateCharacterCRC16(0, c);
      break;
    case ETX:
      Serial.println("ETX");
      if ((crc == 0) && (*len > 2))
      {
        *len -= 2; /* remove crc characters */
        return (true);
      }
      else if (*len == 0)
        return (true);
      break;
    case DLE:
      Serial.println("DLE");
      DLE_last = true;
      break;
    default:
      if (DLE_last)
        c &= 0xBF;
      DLE_last = false;
      if (*len >= max_len)
        break;
      crc = CalculateCharacterCRC16(crc, c);

      rxBuffer[cntreg] = c;
      Serial.print("POS - ");
      Serial.print(cntreg);
      Serial.print(" DATA - ");
      Serial.println(c, HEX);
      *(cur_pos)++ = c;
      (*len)++;
      cntreg++;
    }
  }
  return (false);
}

void textFromHexString(char *hex, char *result)
{
  char temp[3];
  int index = 0;
  temp[2] = '\0';
  while (hex[index])
  {
    strncpy(temp, &hex[index], 2);
    *result = (char)strtol(temp, NULL, 16); // If representations are hex
    result++;
    index += 2;
  }
  *result = '\0';
}

float ConvertB32ToFloat(uint32_t b32)
{
  float result;
  int32_t shift;
  uint16_t bias;

  if (b32 == 0)
    return 0.0;
  // pull significand
  result = (b32 & 0x7fffff); // mask significand
  result /= (0x800000);      // convert back to float
  result += 1.0f;            // add one back
  // deal with the exponent
  bias = 0x7f;
  shift = ((b32 >> 23) & 0xff) - bias;
  while (shift > 0)
  {
    result *= 2.0;
    shift--;
  }
  while (shift < 0)
  {
    result /= 2.0;
    shift++;
  }
  // sign
  result *= (b32 >> 31) & 1 ? -1.0 : 1.0;
  return result;
}

void hexStringtoASCII(char *hex, char *result)
{
  char text[25] = {0}; // another 25
  int tc = 0;

  for (int k = 0; k < strlen(hex); k++)
  {
    if (k % 2 != 0)
    {
      char temp[3];
      sprintf(temp, "%c%c", hex[k - 1], hex[k]);
      int number = (int)strtol(temp, NULL, 16);
      text[tc] = char(number);
      tc++;
    }
  }
  strcpy(result, text);
}

// uint32_t value = strtoul(strCoreData.c_str(), NULL, 16);
// dtostrf(ConvertB32ToFloat(value), 1, 9, s);
bool CheckTimeout(unsigned long *timeNow, unsigned long *timeStart)
{
  if (*timeNow - *timeStart >= CHAR_TIMEOUT)
    return true;
  else
    return false;

} // CheckTimeout

bool GetSerialChar(unsigned long *pTimer)
{
  unsigned char
      chRX;
  unsigned short
      lenPtr;
  if (get_cmd(&chRX, &lenPtr, MAX_RX_CHARS))
  {
    // and reset the character timeout
    *pTimer = millis();

    return true;

  } // if
  else
    return false;

} // GetSerialChar

bool getDataKWH5(void)
{
  static unsigned long
      timeRX;
  unsigned long
      timeNow;
  bool
      returnval = false;

  timeNow = millis();
  digitalWrite(pinLED, HIGH);
  Serial.println(F("SEND TRIGGER"));
  cmd_trigMeter();
  Serial2.flush();
  // delay(500);

  Serial.println(F("DEBUG: ST_LOGIN:"));
  if (GetSerialChar(&timeRX) == true)
  {
    Serial.print(F("CODE - "));
    Serial.println(rxBuffer[0], HEX);
    if (rxBuffer[0] == ACK)
    {
      Serial.println(F("DEBUG: ST_LOGIN: CONNECT ACK."));
      cmd_logonMeter();
      // delay(500);
      returnval = true;
      // return returnval;
    }
  }
  else
  {
    if (CheckTimeout(&timeNow, &timeRX))
    {
      Serial.println(F("DEBUG: TIMEOUT..."));
      returnval = false;
      // return returnval;
    }
  }
  digitalWrite(pinLED, LOW);
  return returnval;
}

bool getDataKWH(void)
{
  char
      ch;
  unsigned char
      chRX;
  unsigned short
      lenPtr;
  static byte
      stateReg = ST_REG_SERIAL,
      stateRegLast = stateReg;
  static byte
      stateRX = ST_CON;
  bool
      recordEnd = false;

  goto st_con;
st_con:
  digitalWrite(pinLED, HIGH);
  cmd_trigMeter();
  stateRX = ST_LOGIN; // wait for the start of text
  Serial.println(F("SEND TRIGGER"));
  // delay(500);
  goto st_login;

st_login:
  Serial.println(F("DEBUG: ST_LOGIN:"));
  get_cmd(&chRX, &lenPtr, MAX_RX_CHARS);
  Serial.print(F("CODE - "));
  Serial.print(rxBuffer[0], HEX);
  if (rxBuffer[0] == ACK)
  {
    Serial.println(F("DEBUG: ST_LOGIN: CONNECT ACK."));
    cmd_logonMeter();
    // delay(500);
    goto st_login_r;
  }
  else
    goto st_con;

st_login_r:
  Serial.println(F("DEBUG: ST_LOGIN_R:"));
  get_cmd(&chRX, &lenPtr, MAX_RX_CHARS);
  Serial.print(F("CODE - "));
  Serial.print(rxBuffer[0], HEX);

  if (rxBuffer[0] == CAN)
  {
    Serial.println(F("DEBUG: ST_READ: CAN CANCEL."));
    goto st_logout;
  }
  if (rxBuffer[0] == ACK)
  {
    Serial.println(F("DEBUG: ST_LOGIN: LOGIN ACK."));
    goto st_read;
  }
  else
    goto st_con;

st_read:
  Serial.println(F("DEBUG: ST_READ:"));
  switch (stateReg)
  {
  case ST_REG_SERIAL:
    Serial.println(F("      READ  - SERIAL NUMBER."));
    cmd_readRegister(regSerNum);
    // delay(500);
    stateRegLast = stateReg;
    break;
  case ST_REG_KWH_A:
    Serial.println(F("      READ  - KWH A."));
    cmd_readRegister(regKWHA);
    // delay(500);
    stateRegLast = stateReg;
    break;
  case ST_REG_KWH_B:
    Serial.println(F("      READ  - KWH B."));
    cmd_readRegister(regKWHB);
    // delay(500);
    stateRegLast = stateReg;
    break;
  case ST_REG_KWH_T:
    Serial.println(F("      READ  - KWH TOTAL."));
    cmd_readRegister(regKWHTOT);
    // delay(500);
    stateRegLast = stateReg;
    break;
  }
  goto st_rx_read;

st_rx_read:
  Serial.println(F("DEBUG: ST_RX_READ:"));
  get_cmd(&chRX, &lenPtr, MAX_RX_CHARS);

  if (rxBuffer[0] == CAN)
  {
    Serial.println(F("DEBUG: ST_RX_READ: CAN CANCEL."));
    goto st_logout;
  }
  else if (rxBuffer[0] == CHAR_REGREAD)
  {
    char buffer[KWH_CHAR];
    uint32_t value;
    switch (stateRegLast)
    {
    case ST_REG_SERIAL:
      Serial.println(F("     DATA SERIAL NUMBER"));
      for (size_t i = 0; i < sizeof(outSerialNumber); i++)
      {
        outSerialNumber[i] = rxBuffer[i + 3];
        Serial.print(F("     DATA ke "));
        Serial.print(i);
        Serial.print(F(" > "));
        Serial.println(outSerialNumber[i], HEX);
      }
      Serial.print(F("     "));
      Serial.println(outSerialNumber);
      stateReg = ST_REG_KWH_A;
      break;
    case ST_REG_KWH_A:
      Serial.println(F("     DATA KWH_A "));
      for (size_t i = 0; i < sizeof(buffer); i++)
      {
        buffer[i] = rxBuffer[i + 2];
        Serial.print(F("     DATA ke "));
        Serial.print(i);
        Serial.print(F(" > "));
        Serial.println(buffer[i], HEX);
      }
      Serial.print(F("     "));
      Serial.println(buffer);
      value = strtoul(buffer, NULL, 16);
      dtostrf(ConvertB32ToFloat(value), 1, 9, outKWHA);
      Serial.println(outKWHA);
      stateReg = ST_REG_KWH_B;
      break;
    case ST_REG_KWH_B:
      Serial.println(F("     DATA KWH_B "));
      for (size_t i = 0; i < sizeof(buffer); i++)
      {
        buffer[i] = rxBuffer[i + 3];
        Serial.print(F("     DATA ke "));
        Serial.print(i);
        Serial.print(F(" > "));
        Serial.println(buffer[i], HEX);
      }
      Serial.print(F("     "));
      Serial.println(buffer);
      value = strtoul(buffer, NULL, 16);
      dtostrf(ConvertB32ToFloat(value), 1, 9, outKWHB);
      Serial.println(outKWHB);
      stateReg = ST_REG_KWH_T;
      break;
    case ST_REG_KWH_T:
      Serial.println(F("     DATA KWH_T "));
      for (size_t i = 0; i < sizeof(buffer); i++)
      {
        buffer[i] = rxBuffer[i + 2];
        Serial.print(F("     DATA ke "));
        Serial.print(i);
        Serial.print(F(" > "));
        Serial.println(buffer[i], HEX);
      }
      Serial.print(F("     "));
      Serial.println(buffer);
      value = strtoul(buffer, NULL, 16);
      dtostrf(ConvertB32ToFloat(value), 1, 9, outKWHTOT);
      Serial.println(outKWHTOT);
      stateReg = ST_REG_END;
      break;
    }
    if (stateReg == ST_REG_END)
    {
      Serial.print(F("DEBUG: ST_RX_READ: READ END: LOGOUT..."));
      cmd_logoffMeter();
      stateReg = ST_REG_SERIAL;
      goto st_logout;
    }
    else
      goto st_read;
  }
  else
    goto st_con;

st_logout:
  Serial.println(F("DEBUG: ST_LOGOUT:"));
  get_cmd(&chRX, &lenPtr, MAX_RX_CHARS);

  if (rxBuffer[0] == ACK)
  {
    Serial.println(F("DEBUG: ST_LOGIN: CONNECT ACK."));
    goto st_con;
    recordEnd = true;
  }

  digitalWrite(LED_BUILTIN, LOW);
  return recordEnd;
}
bool stateNOW = false;
bool getKWH = false;
void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600);

  pinMode(pinLED, OUTPUT);

} // setup

void loop()
{
  stateNOW = true;
  if (stateNOW)
  {
    getKWH = getDataKWH5();
    Serial.println(F("GET KWH FALSE"));
  }

  delay(5000);
  // ReceiveStateMachine(); // messaging with meter
  // ServiceLED();          // for LED timing
  stateNOW = getKWH;
} // loop