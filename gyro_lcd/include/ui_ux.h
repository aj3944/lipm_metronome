#include "mbed.h"

int led_init = 0;
// LCD Touchscreen Configurations
LCD_DISCO_F429ZI lcd;
TS_DISCO_F429ZI ts;
bool start_status = false;
bool stats_status = false;

// location and dimensions of start button
#define START_X 10
#define START_Y 10
#define START_WIDTH 100
#define START_HEIGHT 50
// location and dimensions of stats button
#define STATS_X 130
#define STATS_Y 10
#define STATS_WIDTH 100
#define STATS_HEIGHT 50

bool last_draw_graph = false;


void DrawButtons() {
    int x_placement, y_placement;
    char* label;
    // checking start label status
    if(!start_status) {label = (char *)"Start";}
    else {label = (char *)"Reset";}
    // Drawing the START/STOP button box
    lcd.DrawRect(START_X, START_Y, START_WIDTH, START_HEIGHT);
    x_placement = START_X + START_WIDTH/2 - strlen(label)*6;
    y_placement = START_Y + START_HEIGHT/2 - 8;
    // Writing START/STOP on the button
    lcd.DisplayStringAt(x_placement, y_placement, (uint8_t *)label, LEFT_MODE);

    // checking show stats status
    if(!stats_status) {label=(char *)"Stats";}
    else {label=(char *)"Back";}
    // Drawing the STATS button box
    lcd.DrawRect(STATS_X, STATS_Y, STATS_WIDTH, STATS_HEIGHT);
    x_placement = STATS_X + STATS_WIDTH/2 - strlen(label)*6;
    y_placement = STATS_Y + STATS_HEIGHT/2 - 8;
    // Writing STATS on the button
    lcd.DisplayStringAt(x_placement, y_placement, (uint8_t *)label, LEFT_MODE);
}

int graph_sector( int val, int axis){
  int dx = val/15;

  if(dx > 50) dx = 50;
  if(dx < -50) dx = -50;

  return 20 + 100 + axis*50 + dx;
}

void lcd_clear(){
    
    // LCD INIT
    lcd.Clear(LCD_COLOR_BLUE);
    lcd.SetBackColor(LCD_COLOR_BLUE);
    lcd.SetTextColor(LCD_COLOR_WHITE);
    lcd.DisplayStringAt(0, LINE(17), (uint8_t *)"RTES/F23/MBED_CHLLNGEANGE", LEFT_MODE);
    // lcd.DisplayStringAt(0, 30, (uint8_t *)"X", LEFT_MODE);

    if(led_init && !start_status){
        lcd.DisplayStringAt(10, LINE(10), (uint8_t *)"Press Start to begin!", LEFT_MODE);
    }

    if(!led_init){
        led_init = 1;
        lcd.DisplayStringAt(10, LINE(10), (uint8_t *)"wait..init..", LEFT_MODE);
    }

    // lcd.DisplayStringAt(0, 130, (uint8_t *)"Y", LEFT_MODE);
    // lcd.DisplayStringAt(0, 230, (uint8_t *)"Z", LEFT_MODE);
    lcd.DisplayStringAt(0, LINE(19), (uint8_t *)"ae2405", LEFT_MODE);
    lcd.DisplayStringAt(0, LINE(19), (uint8_t *)"aj3944", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(19), (uint8_t *)"ms14845", RIGHT_MODE);
    // BSP_LCD_SetFont(&Font20);
    // BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
}

void lcd_dist_update(int dist){
    // LCD INIT
    char rs[20] ;
    
    snprintf(rs,20,"DIST(cm):%d",dist);
    printf(rs);
    // lcd.DisplayStringAt(50, 305, (uint8_t *)"DIST(m):", LEFT_MODE);
    lcd.DisplayStringAt(0, 305, (uint8_t *)rs, LEFT_MODE);

}

void do_touch(){
    uint16_t x_touch, y_touch;
    TS_StateTypeDef TS_State;

    ts.GetState(&TS_State);

    // int lcd_xsize = lcd.GetXSize();
    int lcd_ysize = lcd.GetYSize();

    if (TS_State.TouchDetected){
        x_touch = TS_State.X;
        y_touch = TS_State.Y;
        if (x_touch >= START_X && x_touch <= START_X+START_WIDTH && y_touch >= lcd_ysize-START_Y-START_HEIGHT && y_touch <= lcd_ysize-START_Y) {
            start_status = !start_status;
            lcd_clear();
            DrawButtons();
        }
        if (x_touch >= STATS_X && x_touch <= STATS_X+STATS_WIDTH && y_touch >= lcd_ysize-STATS_Y-STATS_HEIGHT && y_touch <= lcd_ysize-STATS_Y) {
            stats_status = !stats_status;
            lcd_clear();
            DrawButtons();
        }
    }
    // if (!stats_status) {HomeDisplay(L, VLinear[w][0]);}
    // if (start_status)    
}



void HomeDisplay(int L, float vel) {
    if(stats_status){
        uint8_t text[30];
        sprintf((char*)text, "Distance Travelled");
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)text, CENTER_MODE);
        sprintf((char*)text, "%d cm", L);
        lcd.DisplayStringAt(0, LINE(6), (uint8_t *)text, CENTER_MODE);
        sprintf((char*)text, "Current Velocity");
        lcd.DisplayStringAt(0, LINE(8), (uint8_t *)text, CENTER_MODE);
        sprintf((char*)text, "%f m/s", vel);
        lcd.DisplayStringAt(0, LINE(9), (uint8_t *)text, CENTER_MODE);
        last_draw_graph = false;
    }
}
int prev_x = 0,prev_y = 0,prev_z = 0;

void AddPoint(int x,int y,int z,int t){
    
    if(!last_draw_graph && !stats_status){
        lcd_clear();
        DrawButtons();
        last_draw_graph = true;
    }
    if(!stats_status && start_status){
        lcd.SetTextColor(LCD_COLOR_RED);
        lcd.DrawLine(t-1,graph_sector(prev_x,0),t,graph_sector(x,0));
        lcd.SetTextColor(LCD_COLOR_GREEN);
        lcd.DrawLine(t-1,graph_sector(prev_y,1),t,graph_sector(y,1));
        lcd.SetTextColor(LCD_COLOR_LIGHTCYAN);
        lcd.DrawLine(t-1,graph_sector(prev_z,2),t,graph_sector(z,2));
    }
    prev_x = x;
    prev_y = y;
    prev_z = z;
    
}
