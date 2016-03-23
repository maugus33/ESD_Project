// This file is for 4 bit mode LCD interfacing with msp430g2553 chip
// 16x2 LCD is used

#ifndef LCD_H_
#define LCD_H_

#define DR P1OUT = P1OUT | BIT4     // define RS high
#define CWR P1OUT = P1OUT & (~BIT4) // define RS low
#define READ P1OUT = P1OUT | BIT5   // define Read signal R/W = 1 for reading
#define WRITE P1OUT = P1OUT & (~BIT5) // define Write signal R/W = 0 for writing
#define ENABLE_HIGH P1OUT = P1OUT | BIT7    // define Enable high signal
#define ENABLE_LOW P1OUT = P1OUT & (~BIT7)  // define Enable Low signal
#define     SECONDLINE          0xC0
#define     FIRSTLINE           0x80

unsigned int i;
unsigned int j;

void delay(unsigned int k)
{
  for(j=0;j<=k;j++)
  {
    for(i=0;i<100;i++);
  }
}

void data_write(void)
{
  ENABLE_HIGH;
  delay(2);
  ENABLE_LOW;
}

void data_read(void)
{
  ENABLE_LOW;
  delay(2);
  ENABLE_HIGH;
}

void check_busy(void)
{
  P1DIR &= ~(BIT3); // make P1.3 as input
  while((P1IN&BIT3)==1)
  {
    data_read();
  }
  P1DIR |= BIT3; // make P1.3 as output
}

void send_command(unsigned char cmd)
{
  check_busy();
  WRITE;
  CWR;
  P1OUT = (P1OUT & 0xF0)|((cmd>>4) & 0x0F); // send higher nibble
  data_write(); // give enable trigger
  P1OUT = (P1OUT & 0xF0)|(cmd & 0x0F); // send lower nibble
  data_write(); // give enable trigger
}

void send_data(unsigned char data)
{
  check_busy();
  WRITE;
  DR;
  P1OUT = (P1OUT & 0xF0)|((data>>4) & 0x0F); // send higher nibble
  data_write(); // give enable trigger
  P1OUT = (P1OUT & 0xF0)|(data & 0x0F); // send lower nibble
  data_write(); // give enable trigger
}

void send_string(char *s)
{
  while(*s)
  {
    send_data(*s);
    s++;
  }
}

//Added on 3/22/16 - To be able to send numbers to the display
void send_number(int number)
{
  char str[32];
  sprintf(str, "%d", number);
  send_string(str);
}

void lcd_init(void)
{
//  P1DIR |= 0xFF;
//  P1OUT &= 0x00;
//Set the display pins as outputs low (P1.0-P1.5 & P1.7)
  P1OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT7);
//Set the display pins as outputs (P1.0-P1.5 & P1.7)
  P1DIR |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT7);
  send_command(0x33);
  send_command(0x32);
  send_command(0x28); // 4 bit mode
  send_command(0x0E); // clear the screen
  send_command(0x01); // display on cursor on
  send_command(0x06); // increment cursor
  send_command(0x80); // row 1 column 1
}

#endif /* LCD_H_ */
