#include <Arduino.h>
#define ENQ 0x05  // ASCII 0x05 == ENQUIRY
#define STX 0x02  // START OF T0xE0T
#define ETX 0x03  // END OF T0xE0T
#define EOT 0x04  // END OF TRANSMISSION
#define LF 0x0A   // LINEFEED
#define NUL 0x00  // NULL termination character
#define XOFF 0x13 // XOFF Pause transmission
#define XON 0x11  // XON Resume transmission
#define DLE 0x10  // DATA LINE ESCAPE
#define CR 0x0D   // CARRIAGE RETURN \r

#define LF 0x0A // LINE FEED \n

#define TRUE 1
#define FALSE 0

const unsigned short table_regInstan[41] =
    {
        0xE000 /*Phase A voltage */,
        0xE001 /*Phase B voltage */,
        0xE002 /*Phase C voltage */,
        0xE004 /*Phase A voltage offset*/,
        0xE005 /*Phase B voltage offset*/,
        0xE006 /*Phase C voltage offset*/,
        0xE010 /*Phase A current */,
        0xE011 /*Phase B current */,
        0xE012 /*Phase C current */,
        0xE014 /*Phase A current offset*/,
        0xE015 /*Phase B current offset*/,
        0xE016 /*Phase C current offset*/,
        0xE020 /*Phase angle of A Phase (in degrees, +=lead, -=lag) */,
        0xE021 /*Phase angle of B Phase (in degrees, +=lead, -=lag) */,
        0xE022 /*Phase angle of C Phase (in degrees, +=lead, -=lag) */,
        0xE024 /*Angle between VTA and VTB*/,
        0xE025 /*Angle between VTA and VTC*/,
        0xE026 /*Power factor */,
        0xE027 /*Absolute angle of A Phase Current*/,
        0xE028 /*Absolute angle of B Phase Current*/,
        0xE029 /*Absolute angle of C Phase Current*/,
        0xE02A /*Absolute angle of A Phase Voltage*/,
        0xE02B /*Absolute angle of B Phase Voltage*/,
        0xE02C /*Absolute angle of C Phase Voltage*/,
        0xE030 /*A phase active total power (watts)*/,
        0xE031 /*B phase active total power (watts)*/,
        0xE032 /*C phase active total power (watts)*/,
        0xE033 /*Total active total power (watts)*/,
        0xE034 /*A phase active fundamental power (watts)*/,
        0xE035 /*B phase active fundamental power (watts)*/,
        0xE036 /*C phase active fundamental power (watts)*/,
        0xE037 /*Total active fundamental power (watts) */,
        0xE040 /*A phase reactive power (VArs)*/,
        0xE041 /*B phase reactive power (VArs)*/,
        0xE042 /*C phase reactive power (VArs)*/,
        0xE043 /*Total reactive power (Vars)*/,
        0xE050 /*A phase apparent power (VA)*/,
        0xE051 /*B phase apparent power (VA)*/,
        0xE052 /*C phase apparent power (VA)*/,
        0xE053 /*Total apparent power (VA)*/,
        0xE060 /*Frequency <50.01>*/,
};

const unsigned short table_regPowerMeas[18] =
    {
        0xE007 /*Phase A fundamental voltage */,
        0xE008 /*Phase B fundamental voltage */,
        0xE009 /*Phase C fundamental voltage */,
        0xE00A /*Phase A voltage 100*(RMS-Fund)/(Fund)*/,
        0xE00B /*Phase B voltage 100*(RMS-Fund)/(Fund)*/,
        0xE00C /*Phase C voltage 100*(RMS-Fund)/(Fund)*/,
        0xE00D /*Voltage Zero Sequence */,
        0xE00E /*Voltage Positive Sequence */,
        0xE00F /*Voltage Negative Sequence */,
        0xE017 /*Phase A fundamental current */,
        0xE018 /*Phase B fundamental current */,
        0xE019 /*Phase C fundamental current */,
        0xE01A /*Phase A current 100*(RMS-Fund)/(Fund)*/,
        0xE01B /*Phase B current 100*(RMS-Fund)/(Fund)*/,
        0xE01C /*Phase C current 100*(RMS-Fund)/(Fund)*/,
        0xE01D /*Current Zero Sequence */,
        0xE01E /*Current Positive Sequence */,
        0xE01F /*Current Negative Sequence */
};

const unsigned short table_regEnergy[32] =
    {
        0xE090 /* A phase import Wh */,
        0xE091 /* B phase import Wh */,
        0xE092 /* C phase import Wh */,
        0xE093 /* Total import Wh */,
        0xE094 /* A phase 0xE0port Wh */,
        0xE095 /* B phase export Wh */,
        0xE096 /* C phase export Wh */,
        0xE097 /* Total export Wh */,
        0xE098 /* A phase import varh */,
        0xE099 /* B phase import varh */,
        0xE09A /* C phase import varh */,
        0xE09B /* Total import varh */,
        0xE09C /* A phase export varh */,
        0xE09D /* B phase export varh */,
        0xE09E /* C phase export varh */,
        0xE09F /* Total export varh */,
        0xE0E0 /* A phase import Vah */,
        0xE0E1 /* B phase import Vah */,
        0xE0E2 /* C phase import Vah */,
        0xE0E3 /* Total import Vah */,
        0xE0E4 /* A phase export Vah */,
        0xE0E5 /* B phase export Vah */,
        0xE0E6 /* C phase export Vah */,
        0xE0E7 /* Total export Vah */,
        0xE0E8 /* A ph fund import Wh */,
        0xE0E9 /* B ph fund import Wh */,
        0xE0EA /* C ph fund import Wh */,
        0xE0EB /* Total fund import Wh */,
        0xE0EC /* A ph fund export Wh */,
        0xE0ED /* B ph fund export Wh */,
        0xE0EE /* C ph fund export Wh */,
        0xE0EF /* Total fund export Wh */,
};
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
  Serial.write(ch);
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
   * Add the STX and start the CRC calc.
   */
  cmdlink_putch(STX);
  crc = CalculateCharacterCRC16(0, STX);
  /*
   * Send the data, computing CRC as we go.
   */
  for (i = 0; i < len; i++)
  {
    send_byte(*cmd);
    crc = CalculateCharacterCRC16(crc, *cmd++);
  }
  /*
   * Add the CRC
   */
  send_byte((unsigned char)(crc >> 8));
  send_byte((unsigned char)crc);
  /*
   * Add the ETX
   */
  cmdlink_putch(ETX);
}

// READ REGISTER COMMAND 'R'
// unsgined char register ==>> F0 02 -> 0xF0 0x02
void cmd_readRegister(String reg)
{
  // unsigned char command[2];
  // command[0] = reg.substring(0, 2);
  // command[0] = reg.substring(1, 2);
}

// RECEIVE FROM KWH
short get_char(void)
{
  return (-1);
}
/*
 * Call get_cmd with a data buffer (cmd_data) and the maximum length of
 * the buffer (max_len). get_cmd will return FALSE until a complete
 * command is received. When this happens the length of the data is
 * returned in len. Packets with bad CRC's are discarded.
 */
char get_cmd(unsigned char *cmd_data, unsigned short *len,
             unsigned short max_len)
{
  short c;
  static unsigned char *cur_pos = (void *)0;
  static unsigned short crc;
  static char DLE_last;
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
      cur_pos = cmd_data;
      *len = 0;
      crc = CalculateCharacterCRC16(0, c);
      break;
    case ETX:
      if ((crc == 0) && (*len > 2))
      {
        *len -= 2; /* remove crc characters */
        return (TRUE);
      }
      else if (*len == 0)
        return (TRUE);
      break;
    case DLE:
      DLE_last = TRUE;
      break;
    default:
      if (DLE_last)
        c &= 0xBF;
      DLE_last = FALSE;
      if (*len >= max_len)
        break;
      crc = CalculateCharacterCRC16(crc, c);
      *(cur_pos)++ = c;
      (*len)++;
    }
  }
  return (FALSE);
}

unsigned char command[3] = {0x52, 0xF0, 0x02};

void setup()
{
  Serial.begin(9600);
}
void loop()
{
  send_cmd(command, sizeof(command));
  delay(1000);
}