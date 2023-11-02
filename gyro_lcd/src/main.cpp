#include "mbed.h"

// printf("LOLALOLA");

// #include "../lib/GYRO_DISCO/GYRO_DISCO_F429ZI.h"
#include "../lib/LCD_DISCO/LCD_DISCO_F429ZI.h"

SPI spi(PF_9, PF_8, PF_7); // mosi, miso, sclk
DigitalOut cs(PC_1);
 



int writeReg(int address, int data){
    cs = 0;
    spi.write(address);
    int reg_write = data;
    spi.write(reg_write);
    printf("W [%x]=%x\n", address,reg_write);
    cs = 1;
    thread_sleep_for(1);
    return 1;
}
int readReg(int address){
    cs = 0;
    spi.write(address);
    int reg_read = spi.write(0x00);
    // printf("R [%x]=%x\n", address,reg_read);
    cs = 1;
    // thread_sleep_for(1);
    return reg_read;
}

LCD_DISCO_F429ZI lcd;



int graph_sector( int val, int axis){
  int dx = val/15;

  if(dx > 50) dx = 50;
  if(dx < -50) dx = -50;

  return 20 + 50 + axis*100 + dx;
}
void lcd_clear(){

    // LCD INIT
    lcd.Clear(LCD_COLOR_WHITE);
    lcd.SetBackColor(LCD_COLOR_WHITE);
    lcd.SetTextColor(LCD_COLOR_MAGENTA);
    lcd.DisplayStringAt(0, LINE(0), (uint8_t *)"RTES/F23/MBED_CHLLNGEANGE", LEFT_MODE);
    lcd.DisplayStringAt(0, 30, (uint8_t *)"X", LEFT_MODE);
    lcd.DisplayStringAt(0, 130, (uint8_t *)"Y", LEFT_MODE);
    lcd.DisplayStringAt(0, 230, (uint8_t *)"Z", LEFT_MODE);
    lcd.DisplayStringAt(150, 305, (uint8_t *)"aj3944", LEFT_MODE);

}
int main()
{
    spi.format(8,3);
    spi.frequency(1000000);
    // INITALIZE I3D4250G
    writeReg(0x20,0xFF);
    writeReg(0x21,0x1A);
    writeReg(0x2E,0x00);

    int out_vals[6];
    int out_xyz[3];



    int last_sane_xyz[3] = { 0 };
    int smooth_xyz[3] = { 0 };
    int normal_xyz[3] = { 0 };
    int avg_xyz[240][3] = { 0 };
    int t = 0;
    int rs = 10;

    lcd_clear();
    while(1)
    {
      t = t%240 + 1;
      // if(t == 120){
      //   lcd_clear();
      // }
      for(int i = 0 ; i < 6; i++){
        int ad = 0xA8 + i;
        out_vals[i] = readReg(ad);
      }
      for(int i = 0; i < 3; i++){
        out_xyz[i] = out_vals[i] + (out_vals[i + 1] ^ 0x80) - (out_vals[i + 1] & 0x80);
        last_sane_xyz[i] = out_xyz[i];
        smooth_xyz[i] = (rs - 1)* smooth_xyz[i]/(rs) + last_sane_xyz[i]/(rs);
        normal_xyz[i] = last_sane_xyz[i] - smooth_xyz[i];
        avg_xyz[t][i] = avg_xyz[t-1][i]/2 + normal_xyz[i]/2;
      }
      for(int i = 0 ; i < 3; i++){
        char XYZ[3][20];
        snprintf(XYZ[i], sizeof(XYZ[i]), ">[%x]:%d\n",i,normal_xyz[i]);
        if(i==0)     lcd.SetTextColor(LCD_COLOR_RED);
        if(i==1)     lcd.SetTextColor(LCD_COLOR_GREEN);
        if(i==2)     lcd.SetTextColor(LCD_COLOR_BLUE);
        lcd.DrawRect(0, 20+(i)*100, 239, 100*(i+1));
        lcd.DrawLine(t,graph_sector(avg_xyz[t-1][i],i),t+1,graph_sector(avg_xyz[t][i],i));
        printf(XYZ[i]);
      }
      printf("\n");

    }
}


// GYRO_DISCO_F429ZI gyro;

// int main()
// {
//   // thread_sleep_for(5);
//   // printf("LOLALOLA");
//   // float GyroBuffer[3];




//   // writeReg(0x20,0xFF);
//   // writeReg(0x21,0x17);
//   // writeReg(0x2E,0x00);
  



//   // gyro.Init();
  
//   // printf("MOMAMOAM");
//   while(1) {

//       // printf("REDF");
//       // Read Gyroscope values
//       // gyro.GetXYZ(GyroBuffer);
//       // //       Display values
//       // // wait_us(50);
//       // printf("X = %f\n", GyroBuffer[0]);
//       // printf("Y = %f\n", GyroBuffer[1]);
//       // printf("Z = %f\n", GyroBuffer[2]);
//   }
// }
