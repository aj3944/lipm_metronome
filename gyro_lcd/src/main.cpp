#include "mbed.h"
#include "../lib/LCD_DISCO/LCD_DISCO_F429ZI.h"

#define PI  3.14159265358979323846 
#define EPSILON 0.000001

//#define N 5  //number of samples provided by gyro per x-axis = inf/d
#define M 64 //number of samples in buffer per x-axis, M practically  M<N 
#define F M-1 //Filter coefficients, 1 sample + 1 delay, F <=M , Fmax=M
#define O F-1 //Filter order
#define D F-1 //Number of delay line samples, D=F-1 , Dmax = M-1, D<=M-1
#define d 3 //number of dim
//#define inf (M+1)*d //number of samples taken from gyro, should be infinity



SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); // mosi, miso, sclk, cs

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

#define BUF_LEN 100


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

void make_filter(float coeff_array[]){
  for(int i = 0; i < BUF_LEN; i ++){
    float del_t = (2*PI*(float)i/10);
    coeff_array[i] = sin(del_t)/(PI *(del_t + EPSILON));
  }
}

//

void printArri(int32_t *a, int8_t K){
    printf("Start printArri\n============\n");
    uint8_t i=0;
    //printf("%d \t\n",S);
    for (i=0 ; i<K ; i++){
        printf("%d \t %d \n",i,*(a+i));
    }
    printf("End printArri\n============\n");
}

float dotProduct(float *an, float *bn, int8_t f){
    //if (debug) printf("Start dotProduct============\n");
    int8_t i=0;
    float r=0;
    for (i=0 ; i<f; i++){
        //if (debug) printf("dotI\t[%d]\t a=%4.5f\t b=%4.5f\n",i,*(an+i),*(bn+i));
        r= r + *(an+i) * *(bn+i);
        
    }
    //if (debug) printf("r=%4.5f\n",r);
    //if (debug) printf("End dotProduct============\n");
    return r;
    
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
      
    /////////
    int out_vals[6];
    float out_xyz[3];

    float circular_buffer[BUF_LEN][3] = { 0 };
    float coeff_filter[BUF_LEN] = { 0 };

    make_filter(coeff_filter);


    float last_sane_xyz[3] = { 0 };
    float avg_xyz[BUF_LEN][3] = { 0 };
    //int t = 0;



    // lcd_clear();

    int val;
    ///////////////


    float gyro[d][M] = {0}; //initialie gyro buffer
    int16_t gyroRead[d][M] = {0}; //initialie gyro direct reading
    float I[d][F] = {0}; //initialie filter input samples, slices of gyro buffer, to be multiplied by the filter coefficients
    float H[d][F]; //initialize Filter Coefficients1
    float g[d][M]={0}; //initialize filtered gyro buffer, output of dot product after being processed by filters.
    int8_t i=0,j=0,k=0, t=0; //i for gyro readings counter, j for dim counter, k for filter coefficients, t for sample number
    bool debug=0;
    int8_t w=0, r1=0 ,r2=0; //write and 2 read pointers
    r1=2;
    r2=r1;
    
    printf("Start \n");
    for (i=0 ; i<F ; i++){
      for (j=0 ; j<d ; j++){
            H[j][i]= 0.015625 ; //1/F;
        }
    }
    //printf("M=%d \tF=%d \tD=%d \tdim=%d \tinf=%d \tsamplPerAxs=%d \n", M, F, D, d, inf , N );
    //printf("M=%d \tF=%d \tD=%d \tdim=%d \tinf=%d \tsamplPerAxs=%d \n", M, F, D, d);
    printf("w=%d \tr1=%d \tr2=%d\n", w,r1,r2 );


    for(int i = 0; i < 3; i++){
      if(i==0)     lcd.SetTextColor(LCD_COLOR_RED);
      if(i==1)     lcd.SetTextColor(LCD_COLOR_GREEN);
      if(i==2)     lcd.SetTextColor(LCD_COLOR_BLUE);
      lcd.DrawRect(0, 20+(i)*100, 239, 100*(i+1));
            
    }

    // complex
    while(1)
    {
      //t = t%BUF_LEN + 1;
      //make_filter(coeff_filter);
      //printf(">coeff:%4.5f\n>buf:%4.5f\n",coeff_filter[t-1],circular_buffer[t-1][0]);
    printf("Buffer Size=%d \tFilter Coeff=%d \tDelay Lines=%d \tDim=%d   \n", M, F, D, d  );
    printf("w=%d \tr1=%d \tr2=%d \tt=%d \n", w,r1,r2, t );



      /*      int16_t raw_gx,raw_gy,raw_gz;
      float gx, gy, gz;
      */
      //prepare the write buffer to trigger a sequential read
      write_buf[0]=OUT_X_L|0x80|0x40;
      //start sequential sample reading
      spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
      flags.wait_all(SPI_FLAG);
      
      /*
      //read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
      //Put the high and low bytes in the correct order lowB,HighB -> HighB,LowB
      raw_gx=( ( (uint16_t)read_buf[2] ) <<8 ) | ( (uint16_t)read_buf[1] );
      raw_gy=( ( (uint16_t)read_buf[4] ) <<8 ) | ( (uint16_t)read_buf[3] );
      raw_gz=( ( (uint16_t)read_buf[6] ) <<8 ) | ( (uint16_t)read_buf[5] );
      //printf("RAW|\tgx: %d \t gy: %d \t gz: %d\t",raw_gx,raw_gy,raw_gz);
      */
      for (j=0 ; j<d ; j++){
        //gyro[j][w]=i; //write op
        gyroRead[j][w] = ( ( (uint16_t)read_buf[2*j+1+1] ) <<8 ) | ( (uint16_t)read_buf[2*j+1] );
        gyro[j][w] = ((float) gyroRead[j][w])*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
      /*
      gx=((float)raw_gx)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
      gy=((float)raw_gy)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
      gz=((float)raw_gz)*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
      printf(">gx:%4.5f\n>gy:%4.5f\n>gz:%4.5f\n",gx,gy,gz);
      */
        printf("> gyro[%d][%d]=%4.5f \n",j,w,gyro[j][w]);
        for (k=r1 ; k<r1+F ; k++ ){
          r2=k%M;
          I[j][(k-r1)%F]=gyro[j][r2];
          if (debug) printf("r1=%d \t r2=%d \t k=%d \t I[%d][%d]=%4.5f \n",r1,r2,k,  j,(k-r1)%F,gyro[j][r2]);
        }
      g[j][w]=dotProduct(&I[j][0], &H[j][0],F) * 100;
      //if (debug) printf("g[%d][%d]=%4.5f\n",j,i,g[j][i]);
      lcd.DrawLine(w-1,graph_sector(g[j][w-1]*100,j),w,graph_sector(g[j][w]*100,j)+1);
      }
      
    w=(w+1)%M;
    r1=(r1+1)%M;
    t=(t+1);
    thread_sleep_for(50);
    if((t%M) == 0){
        lcd_clear();
    }
    /*
    }
    
      for(int j = 0; j < 3; j++){
        // last_sane_xyz[i] = out_xyz[i];
        // smooth_xyz[i] = (rs - 1)* smooth_xyz[i]/(rs) + last_sane_xyz[i]/(rs);
        // normal_xyz[i] = last_sane_xyz[i] - smooth_xyz[i];
        circular_buffer[t][i] = out_xyz[i];
        // normal_xyz[i] = out_xyz[i];
        val = 0;
        for(int j = 0; j < BUF_LEN; j++){
          val += circular_buffer[t+j][i]*coeff_filter[j];
        }
        avg_xyz[t][i] = val*100;
      }
      printf(">fx:%4.9f\n>fy:%4.9f\n>fz:%4.9f\n",avg_xyz[t][0],avg_xyz[t][1],avg_xyz[t][2]);

    
      for(int i = 0 ; i < 3; i++){
        lcd.DrawLine(t-1,graph_sector(avg_xyz[t-1][i]*100,i),t,graph_sector(avg_xyz[t][i]*100,i)+1);
      }
      thread_sleep_for(50);

        if(t == 1){
        lcd_clear();
      }
    
    }
    */    
}}
