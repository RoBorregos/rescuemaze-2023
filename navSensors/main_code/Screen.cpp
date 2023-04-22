#include "Screen.h"
#include "CommonK.h"

Screen::Screen()
{
}

void Screen::init()
{

    if (!screen.begin(i2c_Address, true))
    {
        if (!CK::kusingROS)
            Serial.println("ERROR Screen");
    }
    screen.clearDisplay();
    screen.setTextSize(1);
    screen.setTextColor(SH110X_WHITE);
    screen.setCursor(0, 0);
    screen.display();
}

void Screen::setStyle()
{
    // Implement specific style if needed.
}

void Screen::display(String message, int x, int y)
{
    y *= 7;
    if (!CK::debugOled)
        return;
    resetLine(y);

    screen.setCursor(x, y);
    screen.println(message);
    screen.display();
}

void Screen::display(String message, double num, String divider, int x, int y)
{
    String newMessage = message + divider + String(num);
    display(newMessage, x, y);
}

void Screen::display(double num, String message, String divider, int x, int y)
{
    String newMessage = String(num) + divider + message;
    display(newMessage, x, y);
}

void Screen::resetLine(int start)
{
    int min = start;
    int max = min + 7;
    for (int y = min; y < max; y++)
    {
        for (int x = 0; x < 127; x++)
        {
            screen.drawPixel(x, y, SH110X_BLACK);
        }
    }
}

void Screen::resetScreen()
{
    screen.clearDisplay();
}

void Screen::testdraw()
{
    for (int y = 0; y < 5; y++)
    {
        for (int x = 0; x < 127; x++)
        {
            screen.drawPixel(x, y + 50, SH110X_WHITE);
        }
    }

    screen.display();

    delay(5000);
    for (int y = 2; y < 3; y++)
    {
        for (int x = 0; x < 127; x++)
        {
            screen.drawPixel(x, y + 50, SH110X_BLACK);
        }
    }
    screen.display();
}
