#ifndef __LQ12864_H__
#define __LQ12864_H__

extern void LCD_P6x8Char(unsigned char x,unsigned char y,unsigned char ch);//纵坐标y，1代表8像素,横坐标1代表像素1
extern void LCD_P6x8Num(unsigned char x,unsigned char y, uint32 num);
extern void LCD_P8x16Char(unsigned char x,unsigned char y,unsigned char ch);
extern void LCD_P8x16Num(unsigned char x,unsigned char y, uint32 num);

extern void LCD_Init(void);
extern void LCD_CLS(void);
extern void LCD_P6x8Str(unsigned char x,unsigned char y,unsigned char ch[]);
extern void LCD_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
extern void LCD_P14x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
extern void LCD_Print(unsigned char x, unsigned char y, unsigned char ch[]);
extern void LCD_PutPixel(unsigned char x,unsigned char y);
extern void LCD_Rectangle(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2,unsigned char gif);
extern void Draw_LQLogo(void);
extern void Draw_LibLogo(void);
extern void Draw_BMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char * bmp); 

extern void LCD_PrintU16(unsigned char x,unsigned char y,unsigned int num);

#endif

