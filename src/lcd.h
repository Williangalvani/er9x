/*
 * Author - Erez Raviv <erezraviv@gmail.com>
 *
 * Based on th9x -> http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef lcd_h
#define lcd_h


/////////////////////////////////////////////////////////////////////////////////////




#define ILI9340_TFTWIDTH  240
#define ILI9340_TFTHEIGHT 320

#define ILI9340_NOP     0x00
#define ILI9340_SWRESET 0x01
#define ILI9340_RDDID   0x04
#define ILI9340_RDDST   0x09

#define ILI9340_SLPIN   0x10
#define ILI9340_SLPOUT  0x11
#define ILI9340_PTLON   0x12
#define ILI9340_NORON   0x13

#define ILI9340_RDMODE  0x0A
#define ILI9340_RDMADCTL  0x0B
#define ILI9340_RDPIXFMT  0x0C
#define ILI9340_RDIMGFMT  0x0A
#define ILI9340_RDSELFDIAG  0x0F

#define ILI9340_INVOFF  0x20
#define ILI9340_INVON   0x21
#define ILI9340_GAMMASET 0x26
#define ILI9340_DISPOFF 0x28
#define ILI9340_DISPON  0x29

#define ILI9340_CASET   0x2A
#define ILI9340_PASET   0x2B
#define ILI9340_RAMWR   0x2C
#define ILI9340_RAMRD   0x2E

#define ILI9340_PTLAR   0x30
#define ILI9340_MADCTL  0x36


#define ILI9340_MADCTL_MY  0x80
#define ILI9340_MADCTL_MX  0x40
#define ILI9340_MADCTL_MV  0x20
#define ILI9340_MADCTL_ML  0x10
#define ILI9340_MADCTL_RGB 0x00
#define ILI9340_MADCTL_BGR 0x08
#define ILI9340_MADCTL_MH  0x04

#define ILI9340_PIXFMT  0x3A

#define ILI9340_FRMCTR1 0xB1
#define ILI9340_FRMCTR2 0xB2
#define ILI9340_FRMCTR3 0xB3
#define ILI9340_INVCTR  0xB4
#define ILI9340_DFUNCTR 0xB6

#define ILI9340_PWCTR1  0xC0
#define ILI9340_PWCTR2  0xC1
#define ILI9340_PWCTR3  0xC2
#define ILI9340_PWCTR4  0xC3
#define ILI9340_PWCTR5  0xC4
#define ILI9340_VMCTR1  0xC5
#define ILI9340_VMCTR2  0xC7

#define ILI9340_RDID1   0xDA
#define ILI9340_RDID2   0xDB
#define ILI9340_RDID3   0xDC
#define ILI9340_RDID4   0xDD

#define ILI9340_GMCTRP1 0xE0
#define ILI9340_GMCTRN1 0xE1
/*
#define ILI9340_PWCTR6  0xFC

*/

// Color definitions
#define	ILI9340_BLACK   0x0000
#define	ILI9340_BLUE    0x001F
#define	ILI9340_RED     0xF800
#define	ILI9340_GREEN   0x07E0
#define ILI9340_CYAN    0x07FF
#define ILI9340_MAGENTA 0xF81F
#define ILI9340_YELLOW  0xFFE0  
#define ILI9340_WHITE   0xFFFF




#define lcd_cs 4
#define lcd_rst 5
#define lcd_dc 6
#define lcd_sclk 1

#define DD_MOSI 2
#define DD_SCK 1
#define clkport PORTB
#define mosiport PORTB
#define dcport PORTB
#define csport PORTB





 //////////////////////////////////////////////////////////////////////////////


#if LCD_OTHER
 #define LCD_MINCONTRAST 0
 #define LCD_MAXCONTRAST 63
 #define LCD_NOMCONTRAST 25
#else
 #define LCD_MINCONTRAST 10
 #define LCD_MAXCONTRAST 45
 #define LCD_NOMCONTRAST 25
#endif


#define DISPLAY_W 128
#define DISPLAY_H  64
#define FW          6
#define FWNUM       5
#define FH          8

/* lcd common flags */
#define INVERS        0x01
#define BLINK         0x02
#define DBLSIZE       0x04
#define CONDENSED     0x08

/* lcd puts flags */
#define BSS           0x10
// putsChnRaw flags
#define MIX_SOURCE    0x10

/* lcd outdez flags */
#define LEADING0      0x10
#define PREC1         0x20
#define PREC2         0x30 /* 4 modes in 2bits! */
#define LEFT          0x40 /* align left */

/* time & telemetry flags */
#define NO_UNIT       0x80

extern uint8_t lcd_lastPos;

#define PLOT_XOR		0
#define PLOT_BLACK	1
#define PLOT_WHITE	2

extern uint8_t plotType ;

//extern unsigned char font_5x8_x20_x7f[];
extern unsigned char displayBuf[DISPLAY_W*DISPLAY_H/8];

extern uint8_t lcd_putcAtt(unsigned char x,unsigned char y,const char c,uint8_t mode);
extern unsigned char lcd_putsAtt(unsigned char x,unsigned char y,const prog_char * s,uint8_t mode);
//extern uint8_t lcd_puts2Att(uint8_t x,uint8_t y,const prog_char * s,const prog_char * t ,uint8_t mode);
//extern void lcd_putsAttIdx_right( uint8_t y,const prog_char * s,uint8_t idx,uint8_t att) ;
extern void lcd_putsAttIdx(uint8_t x,uint8_t y,const prog_char * s,uint8_t idx,uint8_t att) ;
extern void lcd_putsnAtt(unsigned char x,unsigned char y,const prog_char * s,unsigned char len,uint8_t mode);

extern uint8_t lcd_putc(uint8_t x,uint8_t y,const char c);
extern void lcd_puts_Pleft(uint8_t y,const prog_char * s) ;
extern void lcd_puts_Pskip(uint8_t y,const prog_char * s, uint8_t skip) ;
extern void lcd_puts_P(unsigned char x,unsigned char y,const prog_char * s);
extern void lcd_putsn_P(unsigned char x,unsigned char y,const prog_char * s,unsigned char len);
extern void lcd_outhex4(unsigned char x,unsigned char y,uint16_t val);
extern void lcd_outdezAtt(unsigned char x,unsigned char y,int16_t val,uint8_t mode);
extern void lcd_2_digits( uint8_t x, uint8_t y, uint8_t value, uint8_t attr ) ;
uint8_t lcd_outdezNAtt(uint8_t x,uint8_t y,int32_t val,uint8_t mode,int8_t len);
//extern void lcd_outdezAtt(unsigned char x,unsigned char y,int16_t val,uint8_t mode);
extern void lcd_outdez(unsigned char x,unsigned char y,int16_t val);

extern void lcd_hbar( uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t percent ) ;
extern void lcd_plot(unsigned char x,unsigned char y);
extern void lcd_hline(unsigned char x,unsigned char y, signed char w);
extern void lcd_hlineStip(unsigned char x,unsigned char y, signed char w,uint8_t pat);
extern void lcd_vline(unsigned char x,unsigned char y, signed char h);
extern void lcd_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h ) ;
extern void lcd_char_inverse( uint8_t x, uint8_t y, uint8_t w, uint8_t blink ) ;

//extern void lcd_img_f(unsigned char x,unsigned char y);
extern void lcd_img(uint8_t i_x,uint8_t i_y,const prog_uchar * imgdat,uint8_t idx /*,uint8_t mode*/);

extern void setRotation(uint8_t m);
extern void spi_init();
extern void fillScreen(uint16_t color);
extern void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
extern void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);

extern void lcd_init();
extern void lcd_clear();
extern void refreshDiplay();
extern void lcdSetContrast( void ) ;
extern void lcdSetRefVolt(unsigned char val);
#define BLINK_ON_PHASE (g_blinkTmr10ms & (1<<6))
//#define BLINK_SYNC      g_blinkTmr10ms = (3<<5)
#define BLINK_SYNC
#endif
/*eof*/
