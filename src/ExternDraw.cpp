#include "externdraw.h"

void enter_logo(void)
{
    Draw_Slow_Bitmap(0, 0, Logo, 128, 64);
    Display();
    delay(3000);

    for (int16_t x = -128; x < 128; x += 12)
    {
        // 绘制Logo
        oled.setDrawColor(1);
        Draw_Slow_Bitmap(0, 0, Logo, 128, 64);
        // 转场特效
        oled.setBitmapMode(1);
        oled.setDrawColor(0);

        oled.drawXBM(x, 0, 128, 64, TranAnimation2);
        if (x < 0)
            oled.drawBox(128 + x, 0, -x, 64);

        oled.setBitmapMode(0);
        Display();
    }
    oled.setDrawColor(1);

    float rate, i = 1;
    int x, y, w;
    uint8_t flag = 0;

    while (flag != 2)
    {
        oled.clearBuffer();

        switch (flag)
        {
        case 0:
            if (i < 80)
                i += 0.3 * i;
            else
                flag++;
            break;
        case 1:
            if (i > 64)
                i -= 0.05 * i;
            else
                flag++;
            break;
        }

        rate = i / 64.0;
        w = 64 * rate;
        x = (64 - w) / 2;
        y = (64 - i - 1) / 2;
        Draw_Slow_Bitmap_Resize(x, y, Logo2, 64, 64, w, i);

        Display();
    }

    for (int16_t xx = -64; xx < 64; xx += 12)
    {
        // GetADC0(); // 播放动画是可以同时初始化软件滤波

        oled.clearBuffer();
        // 绘制Logo
        oled.setDrawColor(1);
        Draw_Slow_Bitmap_Resize(x, y, Logo2, 64, 64, w, i);
        // 转场特效
        oled.setBitmapMode(1);
        oled.setDrawColor(0);

        oled.drawXBM(xx, 0, 128, 64, TranAnimation2);
        if (xx > 0)
            oled.drawBox(0, 0, xx, 64);

        oled.setBitmapMode(0);
        Display();
    }
    oled.setDrawColor(1);
}

void disp_boot_info(void)
{
    char buffer[50];
    oled.clearBuffer();
    for (uint8_t i = 0; i < 5; i++)
    {
        oled.setCursor(0, 12 * i + 1);

        switch (i)
        {
        case 0:
            sprintf(buffer, "[启动信息] 编译时间");
            break;
        case 1:
            sprintf(buffer, "%s %s", __DATE__, __TIME__);
            break;
        case 2:
            sprintf(buffer, "MAC %s", ChipMAC_S);
            break;
        case 3:
            sprintf(buffer, "CPU频率:%u MHZ", ESP.getCpuFreqMHz());
            break;
        case 4:
            sprintf(buffer, "%s", ESP.getSdkVersion());
            break;
        }
        oled.print(buffer);
    }
    Display();
}

void Display(void)
{
    PlaySoundLoop();
    // ShellLoop();
    // OLED_ScreenshotPrint();
    oled.sendBuffer();
}

void Draw_Utf(int x, int y, char *s)
{
    // oled.setCursor(x,y + 1);
    // oled.print(s);
    oled.drawUTF8(x, y + 1, s);
}
/*
@ 作用：抖动1
*/
void Blur(int sx, int sy, int ex, int ey, int f, int delayMs)
{
    for (int i = 0; i < f; i++)
    {
        for (int y = 0; y < ey; y++)
        {
            for (int x = 0; x < ex; x++)
            {
                if (x % 2 == y % 2 && x % 2 == 0 && x >= sx && x <= ex && y >= sy && y <= ey)
                    oled.drawPixel(x + (i > 0 && i < 3), y + (i > 1));
                // else oled.drawPixel(x + (i > 0 && i < 3), y + (i > 1), 0);
            }
        }
        if (delayMs)
        {
            oled.sendBuffer();
            delay(delayMs);
        }
    }
}
/*
@ 作用：画滚动条
*/
void Draw_Scale(int x, int y, int w, int h, int s, int v)
{
    //((h / s >= 4)?(h / (float)s):4)
    oled.setDrawColor(1);
    if (w < h)
    {
        oled.drawVLine(x + w / 2.0, y, h);
        if (s < h && h / s >= 4)
            for (int i = 0; i < s + 1; i++)
            {
                if (i % 2)
                    oled.drawHLine(x + w / (float)s, y + (h / (float)s) * i, w / 2.0 + 1);
                else
                    oled.drawHLine(x, y + (h / (float)s) * i, w);
            }
        if (s > h)
            s = h;
        oled.drawBox(x, v, w, h / (float)s);
    }
    else
    {
        oled.drawHLine(x, y + h / 2.0, w);
        if (s < h && h / s >= 4)
            for (int i = 0; i < s + 1; i++)
            {
                if (i % 2)
                    oled.drawVLine(x + (w / (float)s) * i, y + (h / (float)s), h / 2.0 + 1);
                else
                    oled.drawVLine(x + (w / (float)s) * i, y, h);
            }
        if (s > w)
            s = w;
        oled.drawBox(v, y, w / (float)s, w);
    }
}

/*
    @作用：绘制数值条
    @输入：i=值 a=值的最小值 b=值的最大值 x=左上顶点x轴坐标 y=左上顶点y轴坐标 w=宽度 h=高度 c=颜色
*/
void Draw_Num_Bar(float i, float a, float b, int x, int y, int w, int h, int c)
{
    char buffer[20];
    sprintf(buffer, "%.6f", i);
    uint8_t textWidth = oled.getUTF8Width(buffer) + 3;

    oled.setDrawColor(c);
    oled.drawFrame(x, y, w - textWidth - 2, h);
    oled.drawBox(x + 2, y + 2, (u8g2_uint_t)map_f(i, a, b, 0, w - textWidth - 6), h - 4);

    oled.drawStr(x + w - textWidth, y - 1, buffer);
    // 进行去棱角操作:增强文字视觉焦点
    oled.setDrawColor(0);
    oled.setDrawColor(1);
}

void Draw_Pixel_Resize(int x, int y, int ox, int oy, int w, int h)
{
    int xi = x - ox;
    int yi = y - oy;

    oled.drawBox(ox + xi * w, oy + yi * h, w, h);
}

void Draw_Slow_Bitmap(int x, int y, const unsigned char *bitmap, unsigned char w, unsigned char h)
{
    uint8_t color = oled.getDrawColor();
    int xi, yi, intWidth = (w + 7) / 8;
    for (yi = 0; yi < h; yi++)
    {
        for (xi = 0; xi < w; xi++)
        {
            if (pgm_read_byte(bitmap + yi * intWidth + xi / 8) & (128 >> (xi & 7)))
            {
                oled.drawPixel(x + xi, y + yi);
            }
            else if (color != 2)
            {
                oled.setDrawColor(0);
                oled.drawPixel(x + xi, y + yi);
                oled.setDrawColor(color);
            }
        }
    }
}

// 位图缩放 代码片段改自arduboy2
void Draw_Slow_Bitmap_Resize(int x, int y, uint8_t *bitmap, int w1, int h1, int w2, int h2)
{
    uint8_t color = oled.getDrawColor();
    // Serial.print("颜色");
    // Serial.println(color);
    float mw = (float)w2 / w1;
    float mh = (float)h2 / h1;
    uint8_t cmw = ceil(mw);
    uint8_t cmh = ceil(mh);
    int xi, yi, byteWidth = (w1 + 7) / 8;
    for (yi = 0; yi < h1; yi++)
    {
        for (xi = 0; xi < w1; xi++)
        {
            if (pgm_read_byte(bitmap + yi * byteWidth + xi / 8) & (1 << (7 - (xi & 7))))
            {
                oled.drawBox(x + xi * mw, y + yi * mh, cmw, cmh);
            }
            else if (color != 2)
            {
                oled.setDrawColor(0);
                oled.drawBox(x + xi * mw, y + yi * mh, cmw, cmh);
                oled.setDrawColor(color);
            }
        }
    }
}

// 绘制屏保-密集运算线条
void DrawIntensiveComputingLine(void)
{
    static uint8_t Line[4];
    for (uint8_t a = 0; a < 4; a++)
    {
        Line[a] += rand() % 2 - 1;
        if (Line[a] > 128)
            Line[a] -= 128;
        for (uint8_t b = 0; b < rand() % 3 + 3; b++)
        {
            oled.drawHLine(0, Line[a] + rand() % 20 - 10, 128); // 水平线
            oled.drawVLine(Line[a] + rand() % 20 - 10, 0, 64);  // 垂直线
        }
    }
}
// FP 密集运算屏保
void DrawIntensiveComputing(void)
{
    float calculate;

    // 随机线条
    DrawIntensiveComputingLine();

    calculate = sin(millis() / 4000.0);
    // 模拟噪点
    for (int i = 0; i < calculate * 256 + 256; i++)
        oled.drawPixel(rand() % 128, rand() % 64);
    // 波浪警告声
    // SetTone(64 + calculate * 64 + rand() % 16 - 8);
    SetTone(1500 + calculate * 500 + rand() % 64 - 32 - (((millis() / 1000) % 2 == 1) ? 440 : 0));
}
uint32_t Get_UTF8_Ascii_Pix_Len(uint8_t size, char *s)
{
    return oled.getUTF8Width(s);
}
/***
 * @description: 在屏幕中心绘制文本
 * @param {*}
 * @return {*}
 */
void DrawMsgBox(char *s)
{
    int w = Get_UTF8_Ascii_Pix_Len(1, s) + 2;
    int h = 12;
    int x = (SCREEN_COLUMN - w) / 2;
    int y = (SCREEN_ROW - h) / 2;

    oled.setDrawColor(0);

    oled.setDrawColor(0);
    Blur(0, 0, SCREEN_COLUMN, SCREEN_ROW, 3, 0);
    oled.drawFrame(x - 1, y - 3, w + 1, h + 3);
    oled.setDrawColor(1);
    oled.drawRBox(x, y - 2, w, h + 2, 2);
    oled.setDrawColor(0);
    Draw_Utf(x + 1, y - 1, s);
    oled.setDrawColor(1);
}
/***
 * @description: 绘制高亮文本
 * @param {int} x
 * @param {int} y
 * @param {char} *s
 * @return {*}
 */
void DrawHighLightText(int x, int y, char *s)
{
    int TextWidth = oled.getUTF8Width(s);
    int TextHigh = oled.getMaxCharHeight();
    uint8_t color = oled.getDrawColor();

    if (color == 2)
    {
        oled.drawUTF8(x + 1, y + 2, s);
        oled.drawRBox(x, y, TextWidth + 2, TextHigh, 3);
    }
    else
    {
        oled.drawRBox(x, y, TextWidth + 2, TextHigh, 3);
        oled.setDrawColor(!color);
        oled.drawUTF8(x + 1, y + 2, s);
        oled.setDrawColor(color);
    }
}

void Log(MESSAGETYPE type, char *s)
{
#ifdef DEBUG_EXT_DRAW
    switch (type)
    {
    case LOG_INFO:
        Serial.printf("[INFO]");
        break;
    case LOG_OK:
        Serial.printf("[OK]");
        break;
    case LOG_FAILED:
        Serial.printf("[FAILED]");
        break;
    case LOG_WARNING:
        Serial.printf("[WARNING]");
        break;
    case LOG_ERROR:
        Serial.printf("[ERROR]");
        break;
    }
    Serial.printf("%s\n", s);
#endif
    // Pop_Windows(s);
}

/***
 * @description: 短文本编辑器
 * @param {*}
 * @return {*}
 */
void TextEditor(char *title, char *text)
{
    // Pop_Windows("双击保存 长按退出");
    // delay(1000);
    char newText[20] = {0};
    strcpy(newText, text);

    uint8_t charCounter = 0; // 光标指针
    char editChar = 'A';

    bool exitRenameGUI = false;
    bool editFlag = 0, lastEditFlag = 1; // 编辑器状态：0:选择要编辑的字符    1:选择ASCII

    uint8_t key = 0; // 存储按键状态

    while (!exitRenameGUI)
    {

        // 设置编码器
        if (editFlag != lastEditFlag)
        {
            if (editFlag == 0)
                sys_Counter_Set(0, 19, 1, charCounter);
            else
                sys_Counter_Set(0, 255, 1, newText[charCounter]);

            lastEditFlag = editFlag;
        }

        // 获取编码器输入
        switch (editFlag)
        {
        case 0:
            charCounter = sys_Counter_Get();
            break;
        case 1:
            editChar = sys_Counter_Get();
            newText[charCounter] = editChar;
            break;
        }

        //////////////////////////////////////////////////////////
        oled.clearBuffer();

        oled.setDrawColor(1);
        // 第一行显示标题
        oled.drawUTF8(0, 1, title);
        // 第二行显示编辑文本
        oled.drawUTF8(0, 12 + 1, newText);

        // 显示当前选中的ASCII
        oled.setDrawColor(1);
        oled.setFont(u8g2_font_logisoso26_tf);

        oled.setCursor(0, 34);
        oled.printf("%c", newText[charCounter]);

        oled.setCursor(32, 34);
        oled.printf("0X%02X", newText[charCounter]);

        oled.setFont(u8g2_font_wqy12_t_gb2312);

        // 反色显示光标
        oled.setDrawColor(2);
        if (editFlag)
        {
            // 选择字符时光标闪烁
            if ((millis() / 250) % 2)
                oled.drawBox(charCounter * 6, 12, 6, 12);
        }
        else
            oled.drawBox(charCounter * 6, 12, 6, 12);

        // 字符选择区反色高亮
        oled.drawBox(0, 32, 32, 32);

        Display();

        //////////////////////////////////////////////////////////

        // 处理按键事件
        key = sys_KeyProcess();
        switch (key)
        {
        // 单击切换编辑器状态
        case 1:
            editFlag = !editFlag;
            break;

        case 2:
        case 3:
            // 保存并退出
            strcpy(text, newText);
            // Pop_Windows("已保存");
            exitRenameGUI = true;
            break;
        }
    }
}