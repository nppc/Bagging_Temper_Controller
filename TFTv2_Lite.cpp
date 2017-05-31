/*
 2012 Copyright (c) Seeed Technology Inc.

 Authors: Albert.Miao & Loovee,
 Visweswara R (with initializtion code from TFT vendor)

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc.,51 Franklin St,Fifth Floor, Boston, MA 02110-1301 USA

*/

#include "TFTv2_Lite.h"
#include <SPI.h>
#define FONT_SPACE_BIG 9
#define FONT_Y 13	// the height of the symbol
#define DIGIT_WIDTH 7   // width of the digit (all digits and minus sign must have same width)

void TFT::sendCMD(INT8U index)
{
    TFT_DC_LOW;
    TFT_CS_LOW;
    SPI.transfer(index);
    TFT_CS_HIGH;
}

void TFT::WRITE_DATA(INT8U data)
{
    TFT_DC_HIGH;
    TFT_CS_LOW;
    SPI.transfer(data);
    TFT_CS_HIGH;
}

void TFT::sendData(INT16U data)
{
    INT8U data1 = data>>8;
    INT8U data2 = data&0xff;
    TFT_DC_HIGH;
    TFT_CS_LOW;
    SPI.transfer(data1);
    SPI.transfer(data2);
    TFT_CS_HIGH;
}

void TFT::WRITE_Package(INT16U *data, INT8U howmany)
{
    INT16U    data1 = 0;
    INT8U   data2 = 0;

    TFT_DC_HIGH;
    TFT_CS_LOW;
    INT8U count=0;
    for(count=0;count<howmany;count++)
    {
        data1 = data[count]>>8;
        data2 = data[count]&0xff;
        SPI.transfer(data1);
        SPI.transfer(data2);
    }
    TFT_CS_HIGH;
}

INT8U TFT::Read_Register(INT8U Addr, INT8U xParameter)
{
    INT8U data=0;
    sendCMD(0xd9);                                                      /* ext command                  */
    WRITE_DATA(0x10+xParameter);                                        /* 0x11 is the first Parameter  */
    TFT_DC_LOW;
    TFT_CS_LOW;
    SPI.transfer(Addr);
    TFT_DC_HIGH;
    data = SPI.transfer(0);
    TFT_CS_HIGH;
    return data;
}

void TFT::TFTinit (void)
{
//	TFT_RST_LOW; //Added by Vassilis Serasidis (18 Oct 2013)
//	delay(200);  //Added by Vassilis Serasidis (18 Oct 2013)
//	TFT_RST_HIGH; //Added by Vassilis Serasidis (18 Oct 2013)
    SPI.begin(); 
	SPI.setClockDivider(SPI_CLOCK_DIV2);    // 8Mhz - maximum possible on 16Mhz clock
	//SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0))
    TFT_CS_HIGH;
    TFT_DC_HIGH;
    INT8U i=0;//, TFTDriver=0;
//    for(i=0;i<3;i++)
//    {
//        TFTDriver = readID();
//    }
    delay(500);
    sendCMD(0x01);      // Software Reset
    delay(200);

    sendCMD(0xCF);      // Power Control B
    WRITE_DATA(0x00);
    WRITE_DATA(0x8B);
    WRITE_DATA(0X30);

    sendCMD(0xED);      // Power on sequence control
    WRITE_DATA(0x67);   // 1 Parameter Soft Start Control
    WRITE_DATA(0x03);   // 2 Parameter Power on sequence control
    WRITE_DATA(0X12);   // 3 Parameter Power on sequence control
    WRITE_DATA(0X81);   // 4 Parameter Special enhanced mode

    sendCMD(0xE8);      // Driver timing control A
    WRITE_DATA(0x85);
    WRITE_DATA(0x10);
    WRITE_DATA(0x7A);

    sendCMD(0xCB);      // Power Control A
    WRITE_DATA(0x39);
    WRITE_DATA(0x2C);
    WRITE_DATA(0x00);
    WRITE_DATA(0x34);
    WRITE_DATA(0x02);

    sendCMD(0xF7);      // Pump ratio control
    WRITE_DATA(0x20);

    sendCMD(0xEA);      // Driver timing control B
    WRITE_DATA(0x00);
    WRITE_DATA(0x00);

    sendCMD(0xC0);      // Power control 1
    WRITE_DATA(0x1B);   // VRH[5:0]  

    sendCMD(0xC1);      // Power control 2
    WRITE_DATA(0x10);   // SAP[2:0];BT[3:0]

    sendCMD(0xC5);      // VCOM control 1
    WRITE_DATA(0x3F);
    WRITE_DATA(0x3C);

    sendCMD(0xC7);      // VCOM control 2
    WRITE_DATA(0XB7);

    sendCMD(0x36);      // Memory Access Control
    WRITE_DATA(32 | 128 | 8);	//8

    sendCMD(0x3A);      // COLMOD: Pixel Format Set
    WRITE_DATA(0x55);   // 16bit per pixel

    sendCMD(0xB1);      // Frame rate control (in Normal Mode/Full colors)
    WRITE_DATA(0x00);   // DIVA: fosc (no division)
    WRITE_DATA(0x1B);   // RTNA: 70Hz (default)

    sendCMD(0xB6);      // Display Function Control
    WRITE_DATA(0x0A);
    WRITE_DATA(0xA2);


    sendCMD(0xF2);      // 3Gamma Function Disable 
    WRITE_DATA(0x00);

    sendCMD(0x26);      // Gamma curve selected
    WRITE_DATA(0x01);

    sendCMD(0xE0);      // Set Gamma 
    WRITE_DATA(0x0F);
    WRITE_DATA(0x2A);
    WRITE_DATA(0x28);
    WRITE_DATA(0x08);
    WRITE_DATA(0x0E);
    WRITE_DATA(0x08);
    WRITE_DATA(0x54);
    WRITE_DATA(0XA9);
    WRITE_DATA(0x43);
    WRITE_DATA(0x0A);
    WRITE_DATA(0x0F);
    WRITE_DATA(0x00);
    WRITE_DATA(0x00);
    WRITE_DATA(0x00);
    WRITE_DATA(0x00);

    sendCMD(0xE1);      // Set Gamma
    WRITE_DATA(0x00);
    WRITE_DATA(0x15);
    WRITE_DATA(0x17);
    WRITE_DATA(0x07);
    WRITE_DATA(0x11);
    WRITE_DATA(0x06);
    WRITE_DATA(0x2B);
    WRITE_DATA(0x56);
    WRITE_DATA(0x3C);
    WRITE_DATA(0x05);
    WRITE_DATA(0x10);
    WRITE_DATA(0x0F);
    WRITE_DATA(0x3F);
    WRITE_DATA(0x3F);
    WRITE_DATA(0x0F);

    sendCMD(0x11);        // Exit Sleep 
    delay(120);
    sendCMD(0x29);        // Display on
	X=0;Y=0;spacing=FONT_SPACENORM;bold=FONT_NORMAL;size=1;
    //fillScreen();
}

void TFT::invertDisplay(byte ccomand)
{
        sendCMD(ccomand);        // Send mode
}

INT8U TFT::readID(void)
{
    INT8U i=0;
    INT8U data[3] ;
    INT8U ID[3] = {0x00, 0x93, 0x41};
    INT8U ToF=1;
    for(i=0;i<3;i++)
    {
        data[i]=Read_Register(0xd3,i+1);
        if(data[i] != ID[i])
        {
            ToF=0;
        }
    }
    if(!ToF)                                                            /* data!=ID                     */
    {
        Serial.print("Read TFT ID failed, ID should be 0x09341, but read ID = 0x");
        for(i=0;i<3;i++)
        {
            Serial.print(data[i],HEX);
        }
        Serial.println();
    }
    return ToF;
}

void TFT::setCol(INT16U StartCol,INT16U EndCol)
{
    sendCMD(0x2A);                                                      /* Column Command address       */
    sendData(StartCol);
    sendData(EndCol);
}

void TFT::setPage(INT16U StartPage,INT16U EndPage)
{
    sendCMD(0x2B);                                                      /* Column Command address       */
    sendData(StartPage);
    sendData(EndPage);
}

void TFT::fillScreen(INT16U XL, INT16U XR, INT16U YU, INT16U YD, INT16U color)
{
    unsigned long  XY=0;
    unsigned long i=0;

    if(XL > XR)
    {
        XL = XL^XR;
        XR = XL^XR;
        XL = XL^XR;
    }
    if(YU > YD)
    {
        YU = YU^YD;
        YD = YU^YD;
        YU = YU^YD;
    }
    XL = constrain(XL, MIN_X,MAX_X);
    XR = constrain(XR, MIN_X,MAX_X);
    YU = constrain(YU, MIN_Y,MAX_Y);
    YD = constrain(YD, MIN_Y,MAX_Y);

    XY = (XR-XL+1);
    XY = XY*(YD-YU+1);

    Tft.setCol(XL,XR);
    Tft.setPage(YU, YD);
    Tft.sendCMD(0x2c);                                                  /* start to write to display ra */
                                                                        /* m                            */

    TFT_DC_HIGH;
    TFT_CS_LOW;

    INT8U Hcolor = color>>8;
    INT8U Lcolor = color&0xff;
    for(i=0; i < XY; i++)
    {
        SPI.transfer(Hcolor);
        SPI.transfer(Lcolor);
    }

    TFT_CS_HIGH;
}

void TFT::fillScreen(INT16U color)
{
    INT8U Hcolor = color>>8;
    INT8U Lcolor = color&0xff;

    Tft.setCol(0, 319);
    Tft.setPage(0, 239);
    Tft.sendCMD(0x2c);                                                  /* start to write to display ra */
                                                                        /* m                            */

    TFT_DC_HIGH;
    TFT_CS_LOW;
    for(INT16U i=0; i<38400; i++)
    {
        SPI.transfer(Hcolor);
        SPI.transfer(Lcolor);
        SPI.transfer(Hcolor);
        SPI.transfer(Lcolor);
    }
    TFT_CS_HIGH;
}


void TFT::setXY(INT16U poX, INT16U poY)
{
    X=poX;Y=poY;
	setCol(poX, poX);
    setPage(poY, poY);
    sendCMD(0x2c);
}

void TFT::setPixel(INT16U poX, INT16U poY,INT16U color)
{
    setXY(poX, poY);
    sendData(color);
}


void TFT::drawCharPGM( INT8U ascii) {
	// size of the digit: 24x40
	//unsigned int addr_PGM;
	if((ascii>47)&&(ascii<58)) {
		drawBitmap(X, Y, bitmapBigNum[ascii-48], 24, 40); // print big number from bitmap
		X+=24+(24 / 4);  // recalculate X,Y coordinates 
	} else if(ascii==45) {	// draw a minus sign 
		fillScreen(X, X+24, Y+40/2-40/32, Y+40/2+40/16, Fcolor); // draw a minus
		X+=24+(24 / 4);  // recalculate X,Y coordinates 
	} else if(ascii==46) {	// draw a dot from bitmap
		drawBitmap(X, Y+36, bitmapBigDot, 4, 4); // print dot from bitmap
		X+=24+(24 / 4);  // recalculate X,Y coordinates 
	}
}


void TFT::drawChar( INT8U ascii) {
   unsigned int charLine;
   unsigned int char9bitData= 0;
    byte colorF1 = Fcolor>>8;
    byte colorF2 = Fcolor&0xff;
    byte colorB1 = Bcolor>>8;
    byte colorB2 = Bcolor&0xff;
   if((ascii<32)&&(ascii>127)) {ascii = '?'-32;}
   INT8U charInfo = pgm_read_byte(&simpleFontBig[ascii-0x20][FONT_Y]);		// last byte is information byte (width of char and 9th bit data
   // calculate the size (matrix) of the char and define RAm window for it
   byte char_w = (charInfo & 0x0f)+bold+spacing;	// 4 first bits - width of a symbol + bold + chars spacing
   byte char9bitNum = charInfo >> 4;
   if(char9bitNum>0) {char9bitData=pgm_read_word(&simpleFontBig_9bit[(char9bitNum-1)*2]);}    // read two bytes
   Tft.setCol(X, X+char_w*size-1);
   Tft.setPage(Y, Y+FONT_Y*size-1);
   Tft.sendCMD(0x2c);                                                  /* start to write to display ram */
   TFT_DC_HIGH;
   TFT_CS_LOW;
   // read char lines and send them to the TFT
   for (int i =0; i<FONT_Y; i++ ) {
		INT8U temp = pgm_read_byte(&simpleFontBig[ascii-0x20][i]);
		// if needed add 9th bit, make bold etc
		charLine=temp;
		if(char9bitData&0x01) {charLine=charLine | 0x0100;}	// set 9th bit if needed
		char9bitData=char9bitData>>1;		// proceed to next bit
		if(bold==FONT_BOLD){charLine=charLine|(charLine<<1);}		// apply bold attribute
		// start drawing
	   for(INT8U s1=0; s1<size; s1++) {
		   int tmpCharLine=charLine;
		   for(INT8U f=0;f<char_w;f++) {
			   for(INT8U s=0; s<size; s++) {
				   if((tmpCharLine)&0x01) {
						SPI.transfer(colorF1);
						SPI.transfer(colorF2);
				   } else {
						SPI.transfer(colorB1);
						SPI.transfer(colorB2);
				   }
			   }
			   tmpCharLine=tmpCharLine>>1;
		   }
		}
   }
   X+=char_w*size;
   //return char_w*size;
}

void TFT::drawString(char *string)
{
   while(*string)
   {
       drawChar(*string);
       *string++;

           //X += charwidth;  // Move cursor right 
   }
}

void TFT::drawString_P(const char *string)
{
   while(pgm_read_byte(string)!='\0')
   {
       drawChar(pgm_read_byte(string));
       *string++;

           //X += charwidth;  // Move cursor right 
   }
}

void TFT::drawStringPGM(char *string)
{
   while(*string)
   {
       drawCharPGM(*string);
       *string++;
   }
}

//fillRectangle(poX+i*size, poY+f*size, size, size, fgcolor);
void TFT::fillRectangle(INT16U poX, INT16U poY, INT16U length, INT16U width)
{
    fillScreen(poX, poX+length, poY, poY+width, Bcolor);
}

void  TFT::drawHorizontalLine( INT16U poX, INT16U poY, INT16U length)
{
    setCol(poX,poX + length);
    setPage(poY,poY);
    sendCMD(0x2c);
    for(int i=0; i<length; i++)
    sendData(Fcolor);
}

void TFT::drawLine( INT16U x0,INT16U y0,INT16U x1, INT16U y1)
{

    int xt = x1-x0;
    int yt = y1-y0;
    int dx = abs(xt), sx = x0<x1 ? 1 : -1;
    int dy = -abs(yt), sy = y0<y1 ? 1 : -1;
    int err = dx+dy, e2;                                                /* error value e_xy             */
    for (;;){                                                           /* loop                         */
        setPixel(x0,y0,Fcolor);
        e2 = 2*err;
        if (e2 >= dy) {                                                 /* e_xy+e_x > 0                 */
            if (x0 == x1) break;
            err += dy; x0 += sx;
        }
        if (e2 <= dx) {                                                 /* e_xy+e_y < 0                 */
            if (y0 == y1) break;
            err += dx; y0 += sy;
        }
    }

}

void TFT::drawVerticalLine( INT16U poX, INT16U poY, INT16U length)
{
    setCol(poX,poX);
    setPage(poY,poY+length);
    sendCMD(0x2c);
    for(int i=0; i<length; i++)
		sendData(Fcolor);
}

void TFT::drawRectangle(INT16U poX, INT16U poY, INT16U width, INT16U height)
{
    drawHorizontalLine(poX, poY, width);
    drawHorizontalLine(poX, poY+height, width);
    drawVerticalLine(poX, poY, height);
    drawVerticalLine(poX + width, poY, height);

}

void TFT::drawCircle(int poX, int poY, int r)
{
    int xt = -r, yt = 0, err = 2-2*r, e2;
    do {
        setPixel(poX-xt, poY+yt,Fcolor);
        setPixel(poX+xt, poY+yt,Fcolor);
        setPixel(poX+xt, poY-yt,Fcolor);
        setPixel(poX-xt, poY-yt,Fcolor);
        e2 = err;
        if (e2 <= yt) {
            err += ++yt*2+1;
            if (-xt == yt && e2 <= xt) e2 = 0;
        }
        if (e2 > xt) err += ++xt*2+1;
    } while (xt <= 0);
}

void TFT::fillCircle(int poX, int poY, int r)
{
    int xt = -r, yt = 0, err = 2-2*r, e2;
    do {

        drawVerticalLine(poX-xt, poY-yt, 2*yt);
        drawVerticalLine(poX+xt, poY-yt, 2*yt);

        e2 = err;
        if (e2 <= yt) {
            err += ++yt*2+1;
            if (-xt == yt && e2 <= xt) e2 = 0;
        }
        if (e2 > xt) err += ++xt*2+1;
    } while (xt <= 0);

}

void TFT::drawTriangle( int poX1, int poY1, int poX2, int poY2, int poX3, int poY3)
{
    drawLine(poX1, poY1, poX2, poY2);
    drawLine(poX1, poY1, poX3, poY3);
    drawLine(poX2, poY2, poX3, poY3);
}

void TFT::drawNumberR(long long_num,INT8U width)
{
    char char_buffer[10] = "________0";
    INT8U f = width;
    byte sign=0;

    char_buffer[f]=0;      // terminate a string
	f--;
    
    if (long_num < 0)
    {
        sign=1;
        long_num = -long_num;
    }

    while (long_num > 0)
    {
        char_buffer[f] = '0' + long_num % 10;
		f--;
        long_num /= 10;
    }
    
    // draw a sign if needed
    if (sign==1) {char_buffer[f]='-';}
    
    drawString(char_buffer);

    //return width * (DIGIT_WIDTH + spacing * size);
}


void TFT::drawNumber(long long_num)
{
    char char_buffer[10];// = "012345678";
    INT8U f = 8;
    byte sign=0;

    char_buffer[9]=0;      // terminate a string
    if (long_num==0){
		char_buffer[8]='0';
		f--;
	}
    
	if (long_num < 0)
    {
        sign=1;
        long_num = -long_num;
    }

    while (long_num > 0)
    {
        char_buffer[f] = '0' + long_num % 10;
		f--;
        long_num /= 10;
    }
    
    // draw a sign if needed
    if (sign==1) {char_buffer[f]='-';}
    
    drawString(char_buffer+f+1);
}

void TFT::drawNumberPGM(long long_num)
{
    char char_buffer[10];// = "012345678";
    INT8U f = 8;
    byte sign=0;

    char_buffer[9]=0;      // terminate a string
    if (long_num==0){
		char_buffer[8]='0';
		f--;
	}
    
	if (long_num < 0)
    {
        sign=1;
        long_num = -long_num;
    }

    while (long_num > 0)
    {
        char_buffer[f] = '0' + long_num % 10;
		f--;
        long_num /= 10;
    }
    
    // draw a sign if needed
    if (sign==1) {char_buffer[f]='-';}
    
    drawStringPGM(char_buffer+f+1);
}

void TFT::drawFloat(float floatNumber,INT8U decimal)
{
    INT16U temp=0;
    float decy=0.0;
    float rounding = 0.5;
    if(floatNumber<0.0)
    {
        drawChar('-');
        floatNumber = -floatNumber;
    }
    for (INT8U i=0; i<decimal; ++i)
    {
        rounding /= 10.0;
    }
    floatNumber += rounding;

    temp = (INT16U)floatNumber;
    drawNumber(temp);

    if(decimal>0)
    {
        drawChar('.');
		decy = floatNumber-temp;                                            // decimal part,  4             
		while(decimal--)
		   decy *=10;
		drawNumber(decy);
    }
}

void TFT::drawBitmap(int16_t xt, int16_t yt, const uint8_t *bitmap, int16_t w, int16_t h) {

    byte colorF1 = Fcolor>>8;
    byte colorF2 = Fcolor&0xff;
    byte colorB1 = Bcolor>>8;
    byte colorB2 = Bcolor&0xff;


    Tft.setCol(xt, xt+w-1);
    Tft.setPage(yt, yt+h-1);
    Tft.sendCMD(0x2c);                                                  /* start to write to display ram */
    TFT_DC_HIGH;
    TFT_CS_LOW;

    int16_t i, j, byteWidth = (w + 7) / 8;
    for(j=0; j<h; j++) {
        for(i=0; i<w; i++ ) {
            if(pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
                SPI.transfer(colorF1);
                SPI.transfer(colorF2);
            } else {
                SPI.transfer(colorB1);
                SPI.transfer(colorB2);
            }
        }
    }
    TFT_CS_HIGH;
}


uint16_t TFT::Color565(uint8_t r, uint8_t g, uint8_t b) {
 return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// part of a sequential read routimes	
TFT Tft=TFT();
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
