#include "mbed.h"
#include "../lib/LCD_DISCO/LCD_DISCO_F429ZI.h"
#include "../lib/Servo/Servo.h"


#define PI 3.14159

SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); // mosi, miso, sclk, cs



Servo myservo(PB_4);


//address of first register with gyro data
#define OUT_X_L 0x28
#define REFERENCE 0x25

//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20

//configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1

//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG2 0x21

//configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
#define CTRL_REG2_CONFIG 0b00'00'0'1'1'1

//register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG4 0x23

//configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

#define SPI_FLAG 1

uint8_t write_buf[32]; 
uint8_t read_buf[32];

EventFlags flags;
//The spi.transfer() function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);
  

}

LCD_DISCO_F429ZI lcd;



int graph_sector( int val, int axis){
  int dx = val/15;

  if(dx > 50) dx = 50;
  if(dx < -50) dx = -50;

  return 20 + 50 + axis*100 + dx;
}



// int graph_sector( int val, int axis){
//   int dx = val/15;

//   if(dx > 50) dx = 50;
//   if(dx < -50) dx = -50;

//   return 20 + 50 + axis*100 + dx;
// }


float goal_position = 0.;


void move_servo(float goal_delta){
    // for(float p=0; p<1.0; p += 0.1) {
      // goal_position += goal_delta;
      myservo.position(goal_delta);

      printf("<<<SERVO MOVED TO %2.2f>>>\n",goal_position);
      // myservo.position(0.5);
      // thread_sleep_for(20);
    // }
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
    spi.frequency(1'000'000);


    myservo.calibrate(0.005,150);

    write_buf[0]=CTRL_REG1;
    write_buf[1]=CTRL_REG1_CONFIG;
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    // write_buf[0]=CTRL_REG2;
    // write_buf[1]=CTRL_REG2_CONFIG;
    // spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    // flags.wait_all(SPI_FLAG);

    write_buf[0]=CTRL_REG4;
    write_buf[1]=CTRL_REG4_CONFIG;
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG); 


      // write_buf[0]=REFERENCE|0x80|0x40;
      // //start sequential sample reading
      // spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
      // flags.wait_all(SPI_FLAG);
      

    // int out_vals[6];
    float out_xyz[3];



    // float last_sane_xyz[3] = { 0 };
    // float smooth_xyz[3] = { 0 };
    float normal_xyz[3] = { 0 };
    float avg_xyz[240][3] = { 0 };
    int t = 0;
    // int rs = 10;

    // int freq_max = 120;


    // lcd_clear();

    // int norm_freq;
    int val;


    // double ang_veloc = 0.0;

    for(int i = 0; i < 3; i++){
      if(i==0)     lcd.SetTextColor(LCD_COLOR_RED);
      if(i==1)     lcd.SetTextColor(LCD_COLOR_GREEN);
      if(i==2)     lcd.SetTextColor(LCD_COLOR_BLUE);
      lcd.DrawRect(0, 20+(i)*100, 239, 100*(i+1));
            
    }


    float z_position = 0.5;
    float error = 0.0;
    float error_I = 0.0;
    float error_D = 0.0;
    // complex
    while(1)
    {
      t = t%235 + 1;

      if(t == 1){
        lcd_clear();
      }


      int16_t raw_gx,raw_gy,raw_gz;
      float gx, gy, gz;
      //prepare the write buffer to trigger a sequential read
      write_buf[0]=OUT_X_L|0x80|0x40;
      //start sequential sample reading
      spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
      flags.wait_all(SPI_FLAG);
      //read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
      //Put the high and low bytes in the correct order lowB,HighB -> HighB,LowB
      raw_gx=( ( (uint16_t)read_buf[2] ) <<8 ) | ( (uint16_t)read_buf[1] );
      raw_gy=( ( (uint16_t)read_buf[4] ) <<8 ) | ( (uint16_t)read_buf[3] );
      raw_gz=( ( (uint16_t)read_buf[6] ) <<8 ) | ( (uint16_t)read_buf[5] );

      //printf("RAW|\tgx: %d \t gy: %d \t gz: %d\t",raw_gx,raw_gy,raw_gz);

      gx=((float)raw_gx)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
      gy=((float)raw_gy)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
      gz=((float)raw_gz)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
      // printf("Actual|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n",gx,gy,gz);

      z_position += gz*180/PI;

      printf("\t>GZ:%4.5f\n",gz);
      printf("\t>ZPOS:%4.5f\n",z_position);
      
      // if( gz > 0.01 || 1){
      //   // printf("\t\t\tGZ|\tposition: %4.5f \n",gz);
      // }

      error = 0.5 - z_position;
      printf("\t\t error_b>:%4.5f\n",error);



      float p = 2.5,i = 10, d =6;
      if(error*error > 0.001){
        float delta = error;
        error -= delta;
        printf("\t\t deltaL>:%4.5f\n",delta);
        printf("\t\t error>:%4.5f\n",error);
        move_servo(delta);
      }


      out_xyz[0] = gx;
      out_xyz[1] = gy;
      out_xyz[2] = gz;

      for(int i = 0; i < 3; i++){
        // last_sane_xyz[i] = out_xyz[i];
        // smooth_xyz[i] = (rs - 1)* smooth_xyz[i]/(rs) + last_sane_xyz[i]/(rs);
        // normal_xyz[i] = last_sane_xyz[i] - smooth_xyz[i];
        normal_xyz[i] = out_xyz[i];
        val = avg_xyz[t-1][i]/2 + normal_xyz[i]/2;
        avg_xyz[t][i] = val;
      }

      for(int i = 0 ; i < 3; i++){
        lcd.DrawLine(t-1,graph_sector(avg_xyz[t-1][i]*100,i),t,graph_sector(avg_xyz[t][i]*100,i)+1);
      }
      thread_sleep_for(5);
    }
}


