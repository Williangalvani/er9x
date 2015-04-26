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

#include "er9x.h"
#include <stdlib.h>

//#define LCD_2_CS		1

extern struct t_rotary Rotary ;

#define DBL_FONT_SMALL	1


uint8_t lcd_lastPos;

uint8_t displayBuf[DISPLAY_W*DISPLAY_H/8];
#define DISPLAY_END (displayBuf+sizeof(displayBuf))

const prog_uint8_t APM _bitmask[]= { 1,2,4,8,16,32,64,128 } ;

#define XBITMASK(bit) pgm_read_byte( _bitmask + bit )

const prog_uchar APM font[] = {
#include "font.lbm"
};

#define font_5x8_x20_x7f (font)

const prog_uchar APM font_dblsize[] = {
#include "font_dblsize.lbm"
};

#define font_10x16_x20_x7f (font_dblsize)


void lcd_clear()
{
  //for(unsigned i=0; i<sizeof(displayBuf); i++) displayBuf[i]=0;
  memset(displayBuf, 0, sizeof(displayBuf));
}


void putsTime(uint8_t x,uint8_t y,int16_t tme,uint8_t att,uint8_t att2)
{
	div_t qr ;

	if ( tme<0 )
	{
		lcd_putcAtt( x - ((att&DBLSIZE) ? FWNUM*6-2 : FWNUM*3),    y, '-',att);
		tme = -tme;
	}

	lcd_putcAtt(x, y, ':',att&att2);
	qr = div( tme, 60 ) ;
	lcd_2_digits( x, y, (uint16_t)qr.quot, att ) ;
//	lcd_outdezNAtt(x/*+ ((att&DBLSIZE) ? 2 : 0)*/, y, (uint16_t)qr.quot, LEADING0|att,2);
	x += (att&DBLSIZE) ? FWNUM*6-4 : FW*3-3;
	lcd_2_digits( x, y, (uint16_t)qr.rem, att2 ) ;
//	lcd_outdezNAtt(x, y, (uint16_t)qr.rem, LEADING0|att2,2);
}

void putsVolts(uint8_t x,uint8_t y, uint8_t volts, uint8_t att)
{
	lcd_outdezAtt(x, y, volts, att|PREC1);
	if(!(att&NO_UNIT)) lcd_putcAtt(lcd_lastPos, y, 'v', att);
}


void putsVBat(uint8_t x,uint8_t y,uint8_t att)
{
    //att |= g_vbat100mV < g_eeGeneral.vBatWarn ? BLINK : 0;
	putsVolts(x, y, g_vbat100mV, att);
}


void lcd_img(uint8_t i_x,uint8_t i_y,const prog_uchar * imgdat,uint8_t idx/*,uint8_t mode*/)
{
  const prog_uchar  *q = imgdat;

  uint8_t w    = pgm_read_byte(q++);
  uint8_t hb   = pgm_read_byte(q++) ;
  hb += 7 ;
  hb /= 8 ;
  uint8_t sze1 = pgm_read_byte(q++);
  q += idx*sze1;
//  bool    inv  = (mode & INVERS) ? true : (mode & BLINK ? BLINK_ON_PHASE : false);
  for(uint8_t yb = 0; yb < hb; yb++){
    uint8_t   *p = &displayBuf[ (i_y / 8 + yb) * DISPLAY_W + i_x ];
    for(uint8_t x=0; x < w; x++){
      uint8_t b = pgm_read_byte(q++);
      *p++ = b;
      //*p++ = inv ? ~b : b;
    }
  }
}


uint8_t lcd_putc(uint8_t x,uint8_t y,const char c )
{
  return lcd_putcAtt(x,y,c,0);
}

// invers: 0 no 1=yes 2=blink
uint8_t lcd_putcAtt(uint8_t x,uint8_t y,const char c,uint8_t mode)
{
#if (DISPLAY_W==128)
  uint8_t *p  = &displayBuf[ (y & 0xF8) * 16 + x ];
#else  
	prog_uint8_t *p  = &displayBuf[ y / 8 * DISPLAY_W + x ];
#endif
    //uint8_t *pmax = &displayBuf[ DISPLAY_H/8 * DISPLAY_W ];
		if ( c < 22 )		// Move to specific x position (c)*FW
		{
			x = c * FW ;
//  		if(mode&DBLSIZE)
//			{
//				x += x ;
//			}
			return x ;
		}
		x += FW ;
    const prog_uchar    *q = &font_5x8_x20_x7f[(c-0x20)*5];
    bool         inv = (mode & INVERS) ? true : (mode & BLINK ? BLINK_ON_PHASE : false);
	if(mode&DBLSIZE)
  {
		if ( (c!=0x2E)) x+=FW; //check for decimal point
	/* each letter consists of ten top bytes followed by
	 * five bottom by ten bottom bytes (20 bytes per 
	 * char) */
		  unsigned char c_mapped ;

#ifdef DBL_FONT_SMALL
			if ( c >= ',' && c <= ':' )
			{
				c_mapped = c - ',' + 1 ;		
			}
  		else if (c>='A' && c<='Z')
			{
				c_mapped = c - 'A' + 0x10 ;
			}
  		else if (c>='a' && c<='z')
			{
				c_mapped = c - 'a' + 0x2B ;
			}
  		else if (c=='_' )
			{
				c_mapped = 0x2A ;
			}
			else
			{
				c_mapped = 0 ;
			}
#else
			c_mapped = c - 0x20 ;
#endif
        q = &font_10x16_x20_x7f[(c_mapped)*20] ;// + ((c-0x20)/16)*160];
        for(char i=11; i>=0; i--){
	    /*top byte*/
            uint8_t b1 = i>1 ? pgm_read_byte(q) : 0;
	    /*bottom byte*/
            uint8_t b3 = i>1 ? pgm_read_byte(10+q) : 0;
	    /*top byte*/
//            uint8_t b2 = i>0 ? pgm_read_byte(++q) : 0;
	    /*bottom byte*/
//            uint8_t b4 = i>0 ? pgm_read_byte(10+q) : 0;
            q++;
            if(inv) {
                b1=~b1;
//                b2=~b2;
                b3=~b3;
//                b4=~b4;
            }

            if(p < DISPLAY_END-(DISPLAY_W+1)){
                p[0]=b1;
//                p[1]=b2;
                p[DISPLAY_W] = b3;
//                p[DISPLAY_W+1] = b4;
                p+=1;
            }
        }
//        q = &dbl_font[(c-0x20)*20];
//        for(char i=0; i<10; i++){
//            uint8_t b = pgm_read_byte(q++);
//            if((p+DISPLAY_W)<DISPLAY_END) *(p+DISPLAY_W) = inv ? ~b : b;
//            b = pgm_read_byte(q++);
//            if(p<DISPLAY_END) *p = inv ? ~b : b;
//            p++;
//        }
//        if(p<DISPLAY_END) *p = inv ? ~0 : 0;
//        if((p+DISPLAY_W)<DISPLAY_END) *(p+DISPLAY_W) = inv ? ~0 : 0;
  }
  else
  {
		uint8_t condense=0;

		if (mode & CONDENSED)
		{
			*p = inv ? ~0 : 0;
			p += 1 ;
			condense=1;
			x += FWNUM-FW ;
		}

        for(char i=5; i!=0; i--){
            uint8_t b = pgm_read_byte(q++);
    	    if (condense && i==4) {
                /*condense the letter by skipping column 4 */
                continue;
            }
            if(p<DISPLAY_END) {*p = inv ? ~b : b; p += 1 ; }
        }
        if(p<DISPLAY_END) *p++ = inv ? ~0 : 0;
    }
		return x ;
}

// Puts sub-string from string options
// First byte of string is sub-string length
// idx is index into string (in length units)
// Output length characters
void lcd_putsAttIdx(uint8_t x,uint8_t y,const prog_char * s,uint8_t idx,uint8_t att)
{
	uint8_t length ;
	length = pgm_read_byte(s++) ;

  lcd_putsnAtt(x,y,s+length*idx,length,att) ;
}

//void lcd_putsAttIdx_right( uint8_t y,const prog_char * s,uint8_t idx,uint8_t att)
//{
//	uint8_t x = 20 - pgm_read_byte(s) ;
//	lcd_putsAttIdx( x, y, s, idx, att ) ;
//}

//uint8_t lcd_putsnAtt(uint8_t x,uint8_t y,const prog_char * s,uint8_t len,uint8_t mode)
void lcd_putsnAtt(uint8_t x,uint8_t y,const prog_char * s,uint8_t len,uint8_t mode)
{
	uint8_t source ;
	source = mode & BSS ;
//	size = mode & DBLSIZE ;
  while(len!=0) {
    char c = (source) ? *s++ : pgm_read_byte(s++);
		if ( c == 0 )
		{
			return ;
		}
    x = lcd_putcAtt(x,y,c,mode);
//    x+=FW;
//		if ((size)&& (c!=0x2E)) x+=FW; //check for decimal point
    len--;
  }
}
void lcd_putsn_P(uint8_t x,uint8_t y,const prog_char * s,uint8_t len)
{
  lcd_putsnAtt( x,y,s,len,0);
}


uint8_t lcd_putsAtt(uint8_t x,uint8_t y,const prog_char * s,uint8_t mode)
{
	uint8_t source ;
	source = mode & BSS ;
  while(1)
	{
    char c = (source) ? *s++ : pgm_read_byte(s++);
    if(!c) break;
		if ( c == 31 )
		{
			if ( (y += FH) >= DISPLAY_H )	// Screen height
			{
				break ;
			}	
			x = 0 ;
		}
		else
		{
    	x = lcd_putcAtt(x,y,c,mode) ;
		}
//    x+=FW;
//		if ((size)&& (c!=0x2E)) x+=FW; //check for decimal point
  }
  return x;
}

void lcd_puts_Pleft(uint8_t y,const prog_char * s)
{
  lcd_putsAtt( 0, y, s, 0);
}

// This routine skips 'skip' strings, then displays the rest
void lcd_puts_Pskip(uint8_t y,const prog_char * s, uint8_t skip)
{
	while ( skip )
	{
    char c = pgm_read_byte(s++);
    if(!c) return ;
		if ( c == 31 )
		{
			skip -= 1 ;
		}
	}
  lcd_putsAtt( 0, y, s, 0);
}

void lcd_puts_P(uint8_t x,uint8_t y,const prog_char * s)
{
  lcd_putsAtt( x, y, s, 0);
}


void lcd_outhex4(uint8_t x,uint8_t y,uint16_t val)
{
	uint8_t i ;
  x+=FWNUM*4;
  for(i=0; i<4; i++)
  {
    x-=FWNUM;
    char c = val & 0xf;
    c = c>9 ? c+'A'-10 : c+'0';
    lcd_putcAtt(x,y,c,c>='A'?CONDENSED:0);
    val>>=4;
  }
}
void lcd_outdez( uint8_t x, uint8_t y, int16_t val)
{
  lcd_outdezAtt(x,y,val,0);
}

void lcd_outdezAtt( uint8_t x, uint8_t y, int16_t val, uint8_t mode )
{
  lcd_outdezNAtt( x,y,val,mode,5);
}

void lcd_2_digits( uint8_t x, uint8_t y, uint8_t value, uint8_t attr )
{
	lcd_outdezNAtt( x, y, value, attr + LEADING0, 2 ) ;
}

#define PREC(n) ((n&0x20) ? ((n&0x10) ? 2 : 1) : 0)
uint8_t lcd_outdezNAtt( uint8_t x, uint8_t y, int32_t val, uint8_t mode, int8_t len )
{
  uint8_t fw = FWNUM;
  uint8_t prec = PREC(mode);
	uint8_t negative = 0 ;
  uint8_t xn = 0;
  uint8_t ln = 2;
  char c;
  uint8_t xinc ;
	uint8_t fullwidth = 0 ;

	mode &= ~NO_UNIT ;
	if ( len < 0 )
	{
		fullwidth = 1 ;
		len = -len ;		
	}

  if ( val < 0 )
	{
		val = -val ;
		negative = 1 ;
	}

  if (mode & DBLSIZE)
  {
    fw += FWNUM ;
    xinc = 2*FWNUM;
    lcd_lastPos = 2*FW;
  }
  else
  {
    xinc = FWNUM ;
    lcd_lastPos = FW;
  }

  if (mode & LEFT) {
//    if (val >= 10000)
//      x += fw;
    if(negative)
    {
      x += fw;
    }
    if (val >= 1000)
      x += fw;
    if (val >= 100)
      x += fw;
    if (val >= 10)
      x += fw;
    if ( prec )
    {
      if ( prec == 2 )
      {
        if ( val < 100 )
        {
          x += fw;
        }
      }
      if ( val < 10 )
      {
        x+= fw;
      }
    }
  }
  else
  {
    x -= xinc;
  }
  lcd_lastPos += x ;

  if ( prec == 2 )
  {
    mode -= LEADING0;  // Can't have PREC2 and LEADING0
  }

  for (uint8_t i=1; i<=len; i++)
	{
		div_t qr ;
		qr = div( val, 10 ) ;
    c = (qr.rem) + '0';
    lcd_putcAtt(x, y, c, mode);
    if (prec==i) {
      if (mode & DBLSIZE) {
        xn = x;
        if( c<='3' && c>='1') ln++;
        uint8_t tn = (qr.quot) % 10;
        if(tn==2 || tn==4) {
          if (c=='4') {
            xn++;
          }
          else {
            xn--; ln++;
          }
        }
      }
      else {
        x -= 2;
        if (mode & INVERS)
          lcd_vline(x+1, y, 7);
        else
          lcd_plot(x+1, y+6);
      }
      if (qr.quot)
        prec = 0;
    }
    val = qr.quot ;
    if (!val)
    {
      if (prec)
      {
        if ( prec == 2 )
        {
          if ( i > 1 )
          {
            prec = 0 ;
          }
        }
        else
        {
          prec = 0 ;
        }
      }
      else if (mode & LEADING0)
			{
				if ( fullwidth == 0 )
				{
        	mode -= LEADING0;
				}
			}
      else
        break;
    }
    x-=fw;
  }
  if (xn) {
    lcd_hline(xn, y+2*FH-4, ln);
    lcd_hline(xn, y+2*FH-3, ln);
  }
  if(negative) lcd_putcAtt(x-fw,y,'-',mode);
	asm("") ;
	return 0 ;		// Stops compiler creating two sets of POPS, saves flash
}

void lcd_hbar( uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t percent )
{
	uint8_t solid ;
	if ( percent > 100 )
	{
		percent = 100 ;
	}
	solid = (w-2) * percent / 100 ;
	lcd_rect( x, y, w, h ) ;

	if ( solid )
	{
		w = y + h - 1 ;
		y += 1 ;
		x += 1 ;
		while ( y < w )
		{
 			lcd_hline(x, y, solid ) ;
			y += 1 ;			
		}
	}
}

// Reverse video 8 pixels high, w pixels wide
// Vertically on an 8 pixel high boundary
void lcd_char_inverse( uint8_t x, uint8_t y, uint8_t w, uint8_t blink )
{
	if ( blink && BLINK_ON_PHASE )
	{
		return ;
	}
	uint8_t end = x + w ;
#if (DISPLAY_W==128)
  uint8_t *p  = &displayBuf[ (y & 0xF8) * 16 + x ];
#else  
	uint8_t *p  = &displayBuf[ y / 8 * DISPLAY_W + x ];
#endif

	while ( x < end )
	{
		*p++ ^= 0xFF ;
		x += 1 ;
	}
}

uint8_t plotType = PLOT_XOR ;

void lcd_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h )
{
}


void lcd_write_bits( uint8_t *p, uint8_t mask )
{
  if(p<DISPLAY_END)
	{
		uint8_t temp = *p ;
		if ( plotType != PLOT_XOR )
		{
			temp |= mask ;
		}
		if ( plotType != PLOT_BLACK )
		{
			temp ^= mask ;
		}
		*p = temp ;
	}
}

void lcd_plot(uint8_t x,uint8_t y)
{
}

void lcd_hlineStip(unsigned char x,unsigned char y, signed char w,uint8_t pat)
{
}

void lcd_hline(uint8_t x,uint8_t y, int8_t w)
{
}

void lcd_vline(uint8_t x,uint8_t y, int8_t h)
{

}

uint8_t EepromActive ;

void lcdSetContrast()
{
//	lcdSetRefVolt(g_eeGeneral.contrast);
}

void lcdSetRefVolt(uint8_t val)
{
}


#define BIT(x) (1 << (x))
#define SETBITS(x,y) ((x) |= (y))
#define CLEARBITS(x,y) ((x) &= (~(y)))
#define SET_BIT(x,y) SETBITS((x), (BIT((y))))
#define CLEAR_BIT(x,y) CLEARBITS((x), (BIT((y))))
#define BITSET(x,y) ((x) & (BIT(y)))
#define BITCLEAR(x,y) !BITSET((x), (y))
#define BITSSET(x,y) (((x) & (y)) == (y))
#define BITSCLEAR(x,y) (((x) & (y)) == 0)
#define BITVAL(x,y) (((x)>>(y)) & 1)


volatile uint8_t LcdLock ;


//////////////////////////////////SPI CONFIG //////////////////////////////////////
void spi_init(void)
{

  SET_BIT(SPSR,SPI2X);
  SET_BIT(SPCR,0);
  SET_BIT(SPCR,1);
  DDRB =  (1<<DDB4)|(1<<DDB5)|(1<<DDB6);
  /* Set MOSI and SCK output, all others input */
  DDRB |= (1<<DD_MOSI)|(1<<DD_SCK);
  /* Enable SPI, Master, set clock rate fck/16 */
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
  PORTB = 0;
}

void spi_write(char cData)
{
  /* Start transmission */
  SPDR = cData;
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
}


/////////////////////////////////////// SEND COMMAND OR DATA ////////////////////////////////////////////
void writecommand(uint8_t c) {
  CLEAR_BIT(PORTB, lcd_dc);
  //digitalWrite(_dc, LOW);
  CLEAR_BIT(PORTB, lcd_sclk);
  //digitalWrite(_sclk, LOW);
  CLEAR_BIT(PORTB, lcd_cs);
  //digitalWrite(_cs, LOW);
  spi_write(c);
  SET_BIT(PORTB, lcd_cs);
  //digitalWrite(_cs, HIGH);
}

void writedata(uint8_t c) {
  SET_BIT(PORTB, lcd_dc);
  //digitalWrite(_dc, LOW);
  CLEAR_BIT(PORTB, lcd_sclk);
  //digitalWrite(_sclk, LOW);
  CLEAR_BIT(PORTB, lcd_cs);
  //digitalWrite(_cs, LOW);
  spi_write(c);
  SET_BIT(PORTB, lcd_cs);
  //digitalWrite(_cs, HIGH);
}

int _width;
int _height;
int rotation;

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {

  writecommand(ILI9340_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART 
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(ILI9340_PASET); // Row addr set
  writedata(y0>>8);
  writedata(y0);     // YSTART
  writedata(y1>>8);
  writedata(y1);     // YEND

  writecommand(ILI9340_RAMWR); // write to RAM
}

// fill a rectangle
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  SET_BIT(dcport, lcd_dc);
  //digitalWrite(_dc, HIGH);
  CLEAR_BIT(csport, lcd_cs);
  //digitalWrite(_cs, LOW);

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spi_write(hi);
      spi_write(lo);
    }
  }
  //digitalWrite(_cs, HIGH);
  SET_BIT(csport, lcd_cs);
}

void fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}


void setRotation(uint8_t m) {

  writecommand(ILI9340_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTWIDTH;
     _height = ILI9340_TFTHEIGHT;
     break;
   case 1:
     writedata(ILI9340_MADCTL_MV | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTHEIGHT;
     _height = ILI9340_TFTWIDTH;
     break;
  case 2:
    writedata(ILI9340_MADCTL_MY | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTWIDTH;
     _height = ILI9340_TFTHEIGHT;
    break;
   case 3:
     writedata(ILI9340_MADCTL_MV | ILI9340_MADCTL_MY | ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTHEIGHT;
     _height = ILI9340_TFTWIDTH;
     break;
  }
}




void refreshDiplay()
{
//  int color = ILI9340_BLACK;
//  uint8_t hi = color >> 8, lo = color;
  uint8_t blackhi = ILI9340_BLACK >> 8, blacklo = ILI9340_BLACK;
  uint8_t whitehi = ILI9340_WHITE >> 8, whitelo = ILI9340_WHITE;
  uint8_t x=0,y=0,w=128,h=54;
 
  setAddrWindow(x, y, x+w*2-1, y+2*h-1);

  SET_BIT(dcport, lcd_dc);
  //digitalWrite(_dc, HIGH);
  CLEAR_BIT(csport, lcd_cs);
  //digitalWrite(_cs, LOW);

  for(y=2*h; y>0; y--) {
    for(x=w; x>0; x--) {
      int16_t index = x + w * (y/16) ;
      uint8_t bit = (y/2)%8;
      if(BITVAL(displayBuf[index],bit))
      {
        spi_write(blackhi);
        spi_write(blacklo);
        spi_write(blackhi);
        spi_write(blacklo);
      }
      else{
        spi_write(whitehi);
        spi_write(whitelo);
        spi_write(whitehi);
        spi_write(whitelo);
        
      }
    }
  }
  //digitalWrite(_cs, HIGH);
  SET_BIT(csport, lcd_cs);
  }



void lcd_init()
{
  spi_init();
    CLEAR_BIT(clkport, lcd_sclk);

  // toggle RST low to reset


  SET_BIT(PORTB, lcd_rst);
  _delay_ms(5);
  CLEAR_BIT(PORTB, lcd_rst);
  _delay_ms(20);
  SET_BIT(PORTB, lcd_rst);
  _delay_ms(150);

  /*
  uint8_t x = readcommand8(ILI9340_RDMODE);
  Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9340_RDMADCTL);
  Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9340_RDPIXFMT);
  Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9340_RDIMGFMT);
  Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9340_RDSELFDIAG);
  Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
  */

  //if(cmdList) commandList(cmdList);
  
  writecommand(0xEF);
  writedata(0x03);
  writedata(0x80);
  writedata(0x02);

  writecommand(0xCF);  
  writedata(0x00); 
  writedata(0XC1); 
  writedata(0X30); 

  writecommand(0xED);  
  writedata(0x64); 
  writedata(0x03); 
  writedata(0X12); 
  writedata(0X81); 
 
  writecommand(0xE8);  
  writedata(0x85); 
  writedata(0x00); 
  writedata(0x78); 

  writecommand(0xCB);  
  writedata(0x39); 
  writedata(0x2C); 
  writedata(0x00); 
  writedata(0x34); 
  writedata(0x02); 
 
  writecommand(0xF7);  
  writedata(0x20); 

  writecommand(0xEA);  
  writedata(0x00); 
  writedata(0x00); 
 
  writecommand(ILI9340_PWCTR1);    //Power control 
  writedata(0x23);   //VRH[5:0] 
 
  writecommand(ILI9340_PWCTR2);    //Power control 
  writedata(0x10);   //SAP[2:0];BT[3:0] 
 
  writecommand(ILI9340_VMCTR1);    //VCM control 
  writedata(0x3e); //�Աȶȵ���
  writedata(0x28); 
  
  writecommand(ILI9340_VMCTR2);    //VCM control2 
  writedata(0x86);  //--
 
  writecommand(ILI9340_MADCTL);    // Memory Access Control 
  writedata(ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);

  writecommand(ILI9340_PIXFMT);    
  writedata(0x55); 
  
  writecommand(ILI9340_FRMCTR1);    
  writedata(0x00);  
  writedata(0x18); 
 
  writecommand(ILI9340_DFUNCTR);    // Display Function Control 
  writedata(0x08); 
  writedata(0x82);
  writedata(0x27);  
 
  writecommand(0xF2);    // 3Gamma Function Disable 
  writedata(0x00); 
 
  writecommand(ILI9340_GAMMASET);    //Gamma curve selected 
  writedata(0x01); 
 
  writecommand(ILI9340_GMCTRP1);    //Set Gamma 
  writedata(0x0F); 
  writedata(0x31); 
  writedata(0x2B); 
  writedata(0x0C); 
  writedata(0x0E); 
  writedata(0x08); 
  writedata(0x4E); 
  writedata(0xF1); 
  writedata(0x37); 
  writedata(0x07); 
  writedata(0x10); 
  writedata(0x03); 
  writedata(0x0E); 
  writedata(0x09); 
  writedata(0x00); 
  
  writecommand(ILI9340_GMCTRN1);    //Set Gamma 
  writedata(0x00); 
  writedata(0x0E); 
  writedata(0x14); 
  writedata(0x03); 
  writedata(0x11); 
  writedata(0x07); 
  writedata(0x31); 
  writedata(0xC1); 
  writedata(0x48); 
  writedata(0x08); 
  writedata(0x0F); 
  writedata(0x0C); 
  writedata(0x31); 
  writedata(0x36); 
  writedata(0x0F); 

  writecommand(ILI9340_SLPOUT);    //Exit Sleep 
  _delay_ms(120);     
  writecommand(ILI9340_DISPON);    //Display on 
       setRotation(3);    
  _delay_ms(500);
  fillScreen(ILI9340_WHITE);
  _delay_ms(500);
}
