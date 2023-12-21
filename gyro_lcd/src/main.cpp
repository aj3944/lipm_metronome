#include "mbed.h"
#include "../lib/LCD_DISCO/LCD_DISCO_F429ZI.h"

#define PI  3.14159265358979323846
#define EPSILON 0.000001

//Normalized
#define xdim 1
#define ydim 1
#define zdim 1

#define d 3 //number of dimentions in the Gyroscope
#define M 200 //Buffer Length, number of samples in buffer per x-axis, M practically  M<N

//Use Only in Debugging
#define N 7  //number of readings provided by gyro per x-axis = inf/d
#define inf ((N)*1) //number of samples taken from gyro, should be infinity
//
 
#define D (M-1) //Supported Delay Lines
#define F (M) //Supported Filter Coefficients

#define Tdelay (0.5/M) //For Testing
#define Ts2 Tdelay //For Testing

#define xHThreshold 10
#define xLThreshold -10

#define F1 (M) //Filter coefficients, n Delays + 1 current, F <=M , Fmax=M
#define D1 (F1-1) //Filter Order, Number of delay line samples, D=F-1 , Dmax = M-1, D<=M-1, this is for the Filter delay

#define F2 (M) //integrator coefficients , n delay units, similar to F, odd , 3 is min for integrator
#define D2 (F2-1) //integrator delay , n delay units, similar to D

#define F3 (3) //First  Difference, x(n)-x(n-1)
#define D3 (F3-1) //

#define FL (M) //Trapezoidal integrator
#define DL (FL-1) //



SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); // mosi, miso, sclk, cs
InterruptIn int1(PA_1,PullDown);
InterruptIn int2(PA_2,PullDown);

//address of first register with gyro data
#define OUT_X_L 0x28

//Location of Reference register
#define REFERENCE 0x25

//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20

//configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
//DR=01 , BW=10 --> 100Hz (100 readings per second) and 12.5Hz Cutoff (Is this the recognized rate of change in w readings) , PD=1 , Zen=1 , Xen=1 , Yen=1
#define CTRL_REG1_CONFIG 0b01'00'1'0'0'1

//register fields(bits): 00(2),HPM_MODE(2),CUTOFF(4)
#define CTRL_REG2 0x21

//configuration: Reserved=00 , HPM = 00 (Normal Mode Reset Filter), HPCF=0111 (at ODR=190 then cutoff = 0.09 Hz)
#define CTRL_REG2_CONFIG 0b00'00'0'0'0'0

//register fields(bits): I1_Int1 (1), I1_Boot(1), H_Lactive(1), PP_OD(1), I2_DRDY(1), I2_WTM(1), I2_ORun(1), I2_Empty(1)
#define CTRL_REG3 0x22

//configuration: Int1 disabled, Boot status disabled, active high interrupts, push-pull, enable Int2 data ready, disable fifo interru
#define CTRL_REG3_CONFIG 0b10'00'1'0'0'0


//register fields(bits): 00(2),HPM_MODE(2),CUTOFF(4)
#define CTRL_REG4 0x23

//configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0


// BOOT(1)-FIFO_EN(1)-(1)-HPen(1)-INT1_Sel1(1)-INT1_Sel0(1)-Out_Sel1(1)-Out_Sel0(1)
#define CTRL_REG5 0x24

//FIFO EN //hpass en //INT HPASS AND LOW PASS
#define CTRL_REG5_CONFIG 0b01'01'1'0'1'0

// BOOT(1)-FIFO_EN(1)-(1)-HPen(1)-INT1_Sel1(1)-INT1_Sel0(1)-Out_Sel1(1)-Out_Sel0(1)
#define REFERENCE 0x25
#define OUT_TEMP 0x26
#define STATUS_REG 0x27
#define FIFO_CTRL_REG 0x27
#define FIFO_CTRL_CONFIG 0b00'11'1'0'1'0


#define FIFO_CTRL_REG 0x2e
#define FIFO_CTRL_CONFIG 0b00'11'1'0'1'0


#define FIFO_CTRL_REG 0x2f
// #define FIFO_CTRL_CONFIG 0b00'11'1'0'1'0

#define INT1_CFG 0x30
#define INT1_CFG_CONFIG 0b01'00'0'0'1'0


#define INT1_SRC 0x31


#define INT1_THS_XH 0x32
#define INT1_THS_XH_CONFIG 0b00'00'1'0'0'0
#define INT1_THS_XL 0x33
#define INT1_THS_XH_CONFIG 0b00'00'0'1'0'0

#define INT1_THS_YH 0x34
#define INT1_THS_YL 0x35

#define INT1_THS_ZH 0x36
#define INT1_THS_ZL 0x37


#define INT1_DURATION 0x38
#define INT1_DURATION_CONFIG 0b10'00'0'1'0'0





// #define CTRL_REG4 0x23

//configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
// Block Data Update = 0 (Continous value) , BLE = 0 LSB data at Lower Address , FS=01 (500 DPS) , Registered = 0 , Fixed = 00 , SIM = 0 (SPI interface Mode Selection = 4-Wire interface)

//We increased the range to 2000 DPS by setting FS=11
//#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0 this was the lecture value
#define CTRL_REG4_CONFIG 0b0'0'11'0'00'0

#define SPI_FLAG 1
#define DATA_READY_FLAG 2

static const bool debugSwitch=0;
static const bool debugSwitch2=0;
static const bool debugSwitch3=0;

uint8_t write_buf[32];
uint8_t read_buf[32];

EventFlags flags;
//The spi.transfer() function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);


}

LCD_DISCO_F429ZI lcd;

void data_cb(){
  flags.set(DATA_READY_FLAG);
};


// int1.rise(&data_cb);


// convert values from +/- 500 to LCD compatible per axis.


int graph_sector( int val, int axis){
  int dx = val/15;

  if(dx > 50) dx = 50;
  if(dx < -50) dx = -50;

  return 20 + 50 + axis*100 + dx;
}


// FUNCTION printArri
// PRINT INT ARRAY POINTER 
// (ARRAY* , N) -> PRINT
void printArri(float *a, int8_t K){
    if (debugSwitch) {
        printf("Start printArri\n============\n");
        uint8_t i=0;
        for (i=0 ; i<K ; i++){
        printf("%d \t %f \n",i,*(a+i));
        }
        printf("End printArri\n============\n");
    }
}
// FUNCTION printArr2D
// PRINT 2D FLOAT ARRAY POINTER 
// (ARRAY* , N, M) -> PRINT
void printArr2d(float *a, int8_t k1, int8_t k2){ //k1 row, k2 col
    if (debugSwitch) {
        printf("Start printArr2d\n============\n");
        uint8_t i=0,j=0,k=0;
        for (i=0 ; i<k1 ; i++){
            for (j=0; j<k2 ; j++){
                printf("%f \t",*(a+k));
                k=k+1;
            }
                printf("\n");
        }
        printf("End printArr2d\n============\n");
    }
}
// FUNCTION printArr2D
// PRINT 2D INT ARRAY POINTER 
// (ARRAY* , N, M) -> PRINT
void printArr2dInt(int16_t *a, int8_t k1, int8_t k2){ //k1 row, k2 col
    if (debugSwitch) {
        printf("Start---printArr2dInt\n");
        uint8_t i=0,j=0,k=0;
        for (i=0 ; i<k1 ; i++){
            for (j=0; j<k2 ; j++){
                printf("%d \t",*(a+k));
                k=k+1;
            }
                printf("\n");
        }
        printf("End printArr2dInt\n============\n");
    }
}


// FUNCTION dotProduct2
// DO ARRAY POINTER DOT ARRAY POINTER for the given DIM 
// (A* , B*, N, DIM) -> FLOAT

float dotProduct2(float *an, float *bn, int8_t f, int8_t dim){
    if (debugSwitch) {printf("Start dotProduct============\n");}
    int8_t i=0;
    float dotP=0;
    for (i=0 ; i<f; i++){
        dotP= dotP + *(an+dim*i) * *(bn+dim*i);
        if (debugSwitch) printf("dotI\t[%d]\t a=%f\t b=%f a*b=%f\n",i,*(an+dim*i),*(bn+dim*i), *(an+dim*i) * *(bn+dim*i));

    }
    if (debugSwitch) {printf("dot product Result dotP=%f\n",dotP);}
    if (debugSwitch) {printf("End dotProduct============\n");}
    return dotP;

}


// FUNCTION crossProduct3d
// DO ARRAY POINTER CROSS LIPM DIMENTIONS for 3D
// (ARRAY* , RESULT*) -> VOID

void crossProduct3d(float *an, float *vn){
    //float ax=*(an);
    //float ay=*(an+1);
    //float az=*(an+2);

    if (debugSwitch) {printf("Start Crossproduct====\n");}
    *(vn) =   (*(an+1)) * zdim - (*(an+2)) * ydim;
    *(vn+1) = (*(an+2)) * xdim - (*(an))   * zdim;
    *(vn+2) = (*(an))   * ydim - (*(an+1)) * xdim;

    //if (debugSwitch) {("Cross Product Input an Address : %p \t %p \t %p \n", an , an+1 , an+2);}
    if (debugSwitch) {printf("Cross Product Input an : [%f \t %f \t %f] \n", *(an) , *(an+1) , *(an+2));}
    if (debugSwitch) {printf("Cross Product Result vn : [%f \t %f \t %f] \n",*(vn),*(vn+1),*(vn+2));}
    if (debugSwitch) {printf("End Crossproduct====\n");}

}


/// @brief filterRectangle RECTANGLE: 1/F VALUES IN DIMXF Rectangle 
/// @param f SIZE
/// @param dim DIMENTIONS
/// @param arr ARRAY 
void filterRectangle( uint16_t f, uint16_t dim, float (*arr)[3]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
            arr[i][j]=(float) 1/f;
        }
    }
}

/// @brief integratorTrapezoidal 0.5 from first and last, Everything in between
/// @param f SIZE
/// @param dim DIMENTIONS
/// @param arr ARRAY 
void integratorTrapezoidal( uint16_t f, uint16_t dim, float (*arr)[3]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
            if (i==0 || i==f-1){
                arr[i][j]=(float) ((float)1/2) * Ts2; // Ts2 is multiplied to accomodate the sampling frequency
            }
            else {arr[i][j]=(float) 1;}
        }
    }
}
/// @brief integratorTrapezoidal1D  0.5 from first and last, Everything in between
/// @param f SIZE
/// @param dim DIMENTIONS
/// @param arr ARRAY 
void integratorTrapezoidal1D( uint16_t f, uint16_t dim, float (*arr)[1]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
            if (i==0 || i==f-1){
                arr[i][j]=(float) ((float)1/2) * Ts2; // Ts2 is multiplied to accomodate the sampling frequency
            }
            else {arr[i][j]=(float) 1;}
        }
    }
}

/// @brief MAKE A BLACKMAN WINDOW
/// @param f SIZE
/// @param dim DIMENTIONS
/// @param arr ARRAY
void blackmanWindow( uint16_t f, uint16_t dim, float (*arr)[3]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 0.42 - 0.5*cos(2*3.14*i/(f-1)) + 0.08*cos(4*3.14*i/(f-1));
        }
    }
}
/// @brief MAKE A 1D BLACKMAN WINDOW
/// @param f SIZE
/// @param dim DIMENTIONS
/// @param arr ARRAY
void blackmanWindow1D( uint16_t f, uint16_t dim, float (*arr)[1]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 0.42 - 0.5*cos(2*3.14*i/(f-1)) + 0.08*cos(4*3.14*i/(f-1));
        }
    }
}
/// @brief MAKE A UNIT WINDOW
/// @param f SIZE
/// @param dim DIMENTIONS
/// @param arr ARRAY
void unitWindow( uint16_t f, uint16_t dim, float (*arr)[3]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 1;
        }
    }
}
/// @brief MAKE A 1D UNIT WINDOW
/// @param f SIZE
/// @param dim DIMENTIONS
/// @param arr ARRAY
void unitWindow1D( uint16_t f, uint16_t dim, float (*arr)[1]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 1;
        }
    }
}


long long int FACT_ARRAY[M];


/// @brief RETURN A FACTORIAL
/// @param f SIZE
/// @param dim DIMENTIONS
/// @param arr ARRAY
long long int factorial(uint16_t *n){
    if(!FACT_ARRAY[*n] == 0){
        uint16_t i=0, fact=1;
        for(i=1 ; i<=*n ; i++){
        fact=fact*i;
        }
        FACT_ARRAY[*n] = fact;
        return fact;
    }else{
        return FACT_ARRAY[*n];
    }
}


float COMB_ARRAY[M][M];


/// @brief A Choose B 
/// @param a 
/// @param b 
/// @return com bination 
float combination(uint16_t *a, uint16_t *b){
    if(!COMB_ARRAY[*a][*b] == 0){
        float comb=1;
        uint16_t diff = *a - *b ;
        if (debugSwitch) {printf("--a=%d \ta!=%d \n",  *a,factorial(a) );}
        if (debugSwitch) {printf("--b=%d \tb!=%d \n",  *b,factorial(b) );}
        if (debugSwitch) {printf("--a-b=%d \t(a-b)!=%d \n", diff, factorial(&diff) );}
        comb = factorial(a)/(factorial(b) *factorial(&diff) ) ;///(factorial (b) * factorial (a-b));
        if (debugSwitch) {printf ("combination= %f \n", comb);}
        COMB_ARRAY[*a][*b] = comb;
        return comb;
    }else{
        return COMB_ARRAY[*a][*b];
    }
}

/// @brief FIR DIFF FILTER GENERATOR  
/// @param f SIZE
/// @param dim DIM
/// @param arr 
void differentiator( uint16_t f, uint16_t dim, float (*arr)[3] ){//dim
    uint16_t i=0,j=0, n=f-1;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=pow(-1,i) * combination(&n,&i) * (1/pow(Ts2,f-1)); //Multiplied by the 1/Tsample
        }
    }
}


/// @brief MULT D1 = S1 (*) S2
/// @param f SIZE
/// @param dim DIM
/// @param arr1 S1
/// @param arr2 S2 
/// @param arr3 D1 
void hadamardProduct(uint16_t f, uint16_t dim, float (*arr1)[3] , float (*arr2)[3] , float (*arr3)[3] ){//dim
    if (debugSwitch) {printf("Start Haddamard Product======\n");}
    if (debugSwitch) {printf("f=%d , dim=%d\n",f,dim);}
    int i=0, j=0 ;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr3[i][j]= arr1[i][j] * arr2[i][j] ;
                if (debugSwitch) {printf("i=%d\t j=%d\t arr1[%d][%d]=%f\t arr2[%d][%d]=%f\t product=%f\n",i,j, i,j, arr1[i][j], i,j, arr2[i][j], arr3[i][j] );}

        }
    }
    if (debugSwitch) {printf("End Haddamard Product======\n");}
}


/// @brief MULT D1 = S1 (*) S2
/// @param f SIZE
/// @param dim DIM
/// @param arr1 S1
/// @param arr2 S2 
/// @param arr3 D1 
void hadamardProduct1D(uint16_t f, uint16_t dim, float (*arr1)[1] , float (*arr2)[1] , float (*arr3)[1] ){//dim
    if (debugSwitch) {printf("Start Haddamard Product======\n");}
    if (debugSwitch) {printf("f=%d , dim=%d\n",f,dim);}
    int i=0, j=0 ;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr3[i][j]= arr1[i][j] * arr2[i][j] ;
                if (debugSwitch) {printf("i=%d\t j=%d\t arr1[%d][%d]=%f\t arr2[%d][%d]=%f\t product=%f\n",i,j, i,j, arr1[i][j], i,j, arr2[i][j], arr3[i][j] );}

        }
    }
    if (debugSwitch) {printf("End Haddamard Product======\n");}
}

/// @brief CLEAR LCD AND MAKE XYZ BOXES
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
void lcd_dist_update(int dist){

    // LCD INIT
    char rs[20] ;
    
    snprintf(rs,20,"DIST(m):%d",dist);
    // printf(rs);
    // lcd.DisplayStringAt(50, 305, (uint8_t *)"DIST(m):", LEFT_MODE);
    lcd.DisplayStringAt(0, 305, (uint8_t *)rs, LEFT_MODE);

}

int main()
{
    spi.format(8,3);
    spi.frequency(1'000'000);

    write_buf[0]=CTRL_REG1;
    write_buf[1]=CTRL_REG1_CONFIG;
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    write_buf[0]=CTRL_REG2;
    write_buf[1]=CTRL_REG2_CONFIG;
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    write_buf[0]=CTRL_REG4;
    write_buf[1]=CTRL_REG4_CONFIG;
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);


    ///INTRRRUPTS
    int2.rise(&data_cb);
    if(!(flags.get()&DATA_READY_FLAG)&&(int2.read()==1)){
        flags.set(DATA_READY_FLAG);
    }    

    write_buf[0]=CTRL_REG3;
    write_buf[1]=CTRL_REG3_CONFIG;
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);




    // write to reference register , then 1100, xplain ?
    write_buf[0]=REFERENCE|0x80|0x40;
    //start sequential sample reading
    spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    /////////


    lcd_clear();

    for(int i = 0; i < d; i++){
      if(i==0)     lcd.SetTextColor(LCD_COLOR_RED);
      if(i==1)     lcd.SetTextColor(LCD_COLOR_GREEN);
      if(i==2)     lcd.SetTextColor(LCD_COLOR_BLUE);
      lcd.DrawRect(0, 20+(i)*100, 239, 100*(i+1));
    }

    //time_t seconds = time(NULL);
    //time_t tr;
    //time(&tr);
    set_time(1256729737);  // Set RTC time to Wed, 28 Oct 2009 11:35:37

    float gyro[d][M] = {0}; //initialie gyro buffer
    int16_t gyroRead[d][M] = {0}; //initialie gyro direct reading

    float gyroI[d][M] = {0}; //initialie gyro buffer
    float gyroD[d][M] = {0}; //initialie gyro buffer
    float gyroP[d][M] = {0}; //initialie gyro buffer

    int8_t i=0,j=0,k=0, t=0; //i for gyro readings counter, j for dim counter, k for filter coefficients, t for sample number


    int8_t w=0, r=(M+w-D)%M , v=0;

    float delta_v = 0.0;

    int v_x = 0;
    int v_y = 0;
    int v_z = 0;
    float distance_travelled = 0.;
    unsigned int mills = 0;
    time_t seconds = time(NULL);
    unsigned int mills_o = (unsigned int)seconds*1000;
    bool do_sample = false;
    int last_sample = 0;
    while(1)
    {
        time_t seconds = time(NULL);
        mills = (unsigned int)seconds*1000;
        if ( mills - mills_o > 500 ){
            mills_o = (unsigned int)seconds*1000;
            mills = mills_o;
            do_sample = true;
            last_sample = w;
        }
        //prepare the write buffer to trigger a sequential read
        write_buf[0]=OUT_X_L|0x80|0x40;
        //start sequential sample reading
        flags.wait_all(DATA_READY_FLAG);
        spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
        flags.wait_all(SPI_FLAG);


        for (j=0 ; j<d ; j++){
        gyroRead[j][w] = ( ( (uint16_t)read_buf[2*j+1+1] ) <<8 ) | ( (uint16_t)read_buf[2*j+1] );
        gyro[j][w] = ((float) gyroRead[j][w])*(17.5f*0.017453292519943295769236907684886f / 1000.0);
        printf(">gyro[%d]:%f\n",j,gyro[j][w] );
        }

        gyroI[0][w]=gyro[0][w];
        gyroI[1][w]=gyro[1][w];
        gyroI[2][w]=gyro[2][w];


        // int dist_int = (int)distance_travelled;
        // lcd_dist_update(dist_int);

        // v_x = graph_sector(dist_int,1);
        // lcd.DrawLine(w-1,v_x,w,v_x);

        //Increment Pointers
        w=(w+1)%M;
        r=(r+1)%M;
        t=(t+1);

        if(do_sample){
            float av = 0.;
            // int diff = w - last_sample;
            // if(diff)
            for(int xi = 0; xi < M; xi++ ){
                av +=  gyroI[0][xi];
            }
            av /= (float)M;
            delta_v = av;
            if(delta_v*delta_v > 1){
                delta_v = 1;
            }
            distance_travelled = (float)distance_travelled + delta_v*delta_v;
            printf(">mills:%d\n",mills);
            printf(">Total Distance:%f\n",distance_travelled);
            do_sample = false;
        }

        thread_sleep_for(1);



        if((w%M) == 0){
            lcd_clear();
        }
    }

}
