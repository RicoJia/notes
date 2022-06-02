/**
1. Manually install liquid_crystal lib: https://github.com/johnrickman/LiquidCrystal_I2C
2. Run I2C_Scanner to find the Address of the I2C device!
**/

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2); // If this address is not working for your I2C backpack,
// run the address scanner sketch to determine the actual
// address.

void setup()
{
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Hello world!");
  lcd.setCursor(0, 1);
  lcd.print("Row number: ");
  lcd.setCursor(12, 1);
  lcd.print("2");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Backlight On");
  lcd.backlight();
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Backlight Off");
  lcd.noBacklight();
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Backlight On");
  lcd.backlight();
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Scroll");
  for (int i = 0; i < 5; i++)
  {
    delay(500);
    lcd.scrollDisplayRight();
  }
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Demo finished");

}
void loop()
{

}
