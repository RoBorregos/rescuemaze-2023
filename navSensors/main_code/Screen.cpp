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
}

void Screen::setStyle()
{
    // Implement specific style if needed.
}

void Screen::display(String message, int x, int y)
{
    if (!CK::debugOled)
        return;
    screen.clearDisplay();
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
