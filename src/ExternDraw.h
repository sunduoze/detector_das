#ifndef ExternDraw
#define ExternDraw

#include "task.h"

// #define DEBUG_EXT_DRAW
#ifdef DEBUG_EXT_DRAW
#WARNING "DEBUG_EXT_DRAW"
#endif

enum MESSAGETYPE
{
    LOG_INFO = 0,
    LOG_OK,
    LOG_FAILED,
    LOG_WARNING,
    LOG_ERROR,
};

void EnterLogo(void);
void ShowBootMsg(void);

extern uint8_t DisplayFlashTick;
void Display(void);

void Draw_Utf(int x, int y, char *s);
void Blur(int sx, int sy, int ex, int ey, int f, int delay);
void Draw_Scale(int x, int y, int w, int h, int s, int v);
void Draw_Num_Bar(float i, float a, float b, int x, int y, int w, int h, int c);
void Draw_Pixel_Resize(int x, int y, int ox, int oy, int w, int h);
void Draw_Slow_Bitmap(int x, int y, const unsigned char *bitmap, unsigned char w, unsigned char h);
void Draw_Slow_Bitmap_Resize(int x, int y, uint8_t *bitmap, int w1, int h1, int w2, int h2);
void DrawStatusBar(bool color);

void DrawIntensiveComputingLine(void);
void DrawIntensiveComputing(void);

void DrawMsgBox(char *s);
void DrawHighLightText(int x, int y, char *s);

void TextEditor(char *title, char *text);

uint32_t Get_UTF8_Ascii_Pix_Len(uint8_t size, char *s);

void Log(MESSAGETYPE type, char *s);
#endif