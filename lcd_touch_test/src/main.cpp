#include "mbed.h"
#include "../lib/LCD_DISCO/TS_DISCO_F429ZI.h"
#include "../lib/LCD_DISCO/LCD_DISCO_F429ZI.h"

LCD_DISCO_F429ZI lcd;
TS_DISCO_F429ZI ts;

bool start_status = false;

void DrawButtons() {
  int x, y, width, height;
  char* label;
  if(!start_status) {
    x=10, y=10;
    width=100, height=50;
    lcd.DrawRect(x, y, width, height);
    label = "Start";
    lcd.DisplayStringAt(x+width/2 - strlen(label)*7, y+height/2-8, (uint8_t *)label, LEFT_MODE);
  }
  else {
    x=10, y=10;
    width=100, height=50;
    lcd.DrawRect(x, y, width, height);
    label = "Stop";
    lcd.DisplayStringAt(x+width/2 - strlen(label)*7, y+height/2-8, (uint8_t *)label, LEFT_MODE);
  }

  x=130, y=10;
  width=100, height=50;
  lcd.DrawRect(x, y, width, height);
  label = "Stats";
  lcd.DisplayStringAt(x+width/2 - strlen(label)*7, y+height/2-8, (uint8_t *)label, LEFT_MODE);
}

int main()
{
    TS_StateTypeDef TS_State;
    uint16_t x, y;
    uint8_t text[30];
    uint8_t status;
  
    BSP_LCD_SetFont(&Font20);
  
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"TOUCHSCREEN", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"DEMO", CENTER_MODE);
    HAL_Delay(100);
  
    status = ts.Init(lcd.GetXSize(), lcd.GetYSize());
  
    if (status != TS_OK)
    {
      lcd.Clear(LCD_COLOR_RED);
      lcd.SetBackColor(LCD_COLOR_RED);
      lcd.SetTextColor(LCD_COLOR_WHITE);
      lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"TOUCHSCREEN", CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"INIT FAIL", CENTER_MODE);
    }
    else
    {
      lcd.Clear(LCD_COLOR_GREEN);
      lcd.SetBackColor(LCD_COLOR_GREEN);
      lcd.SetTextColor(LCD_COLOR_WHITE);
      lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"TOUCHSCREEN", CENTER_MODE);
      lcd.DisplayStringAt(0, LINE(6), (uint8_t *)"INIT OK", CENTER_MODE);
    }
    
    HAL_Delay(100);
    lcd.Clear(LCD_COLOR_BLUE);
    lcd.SetBackColor(LCD_COLOR_BLUE);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    
    int xsize = lcd.GetXSize();
    int ysize = lcd.GetYSize();

    while(1)
    {
      DrawButtons();
      thread_sleep_for(300);
      ts.GetState(&TS_State);      
      if (TS_State.TouchDetected)
      {
        x = TS_State.X;
        y = TS_State.Y;
        if(x >= 10 && x <= 100 && y >= 250 && y <= 310) {
          start_status = !start_status;
          lcd.Clear(LCD_COLOR_BLUE);
        }
        sprintf((char*)text, "x=%d y=%d    ", x, y);
        lcd.DisplayStringAt(0, LINE(10), (uint8_t *)&text, LEFT_MODE);
      }
    }
}
