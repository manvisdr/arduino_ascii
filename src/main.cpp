#include <Arduino.h>
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

#define TRUE 1
#define FALSE 0

unsigned char command[6] = {0x52, 0xF0, 0x10, 0x02, 0xEE, 0x45};
unsigned char comm_login[15] = {0x4C, 0x45, 0x44, 0x4D, 0x49, 0x2C, 0x49, 0x4D, 0x44, 0x45, 0x49, 0x4D, 0x44, 0x45, 0x00};

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

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // send_cmd(comm_login, sizeof(comm_login));

  delay(1000);
}