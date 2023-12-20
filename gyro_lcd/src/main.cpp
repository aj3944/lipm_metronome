#include "mbed.h"
#include "../lib/LCD_DISCO/LCD_DISCO_F429ZI.h"

#define PI  3.14159265358979323846 
#define EPSILON 0.000001

//Normalized
#define xdim 1
#define ydim 1
#define zdim 1

#define d 3 //number of dim
#define M 4 //Buffer Length, number of samples in buffer per x-axis, M practically  M<N 

//Use Only in Debugging
#define N 7  //number of readings provided by gyro per x-axis = inf/d
#define inf ((N)*1) //number of samples taken from gyro, should be infinity
//

#define D (M-1) //Supported Delay Lines
#define F (M) //Supported Filter Coefficients

//#define Ts2 0.05000 Required
#define Tdelay 10 //For Testing
#define Ts2 1 //For Testing
#define Delta2 (1/Ts2) //For the integrator

#define F1 (M) //Filter coefficients, n Delays + 1 current, F <=M , Fmax=M
#define D1 (F1-1) //Filter Order, Number of delay line samples, D=F-1 , Dmax = M-1, D<=M-1, this is for the Filter delay

#define F2 (M) //integrator coefficients , n delay units, similar to F, odd , 3 is min for integrator
#define D2 (F2-1) //integrator delay , n delay units, similar to D

#define F3 (3) //First  Difference, x(n)-x(n-1)
#define D3 (F3-1) //

#define FL (M) //Trapezoidal integrator
#define DL (FL-1) //



SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); // mosi, miso, sclk, cs

//address of first register with gyro data
#define OUT_X_L 0x28

//Location of Reference register
#define REFERENCE 0x25

//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20

//configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
//DR=01 , BW=10 --> 190Hz (190 readings per second) and 50Hz Cutoff (Is this the recognized rate of change in w readings) , PD=1 , Zen=1 , Xen=1 , Yen=1
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1

//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG2 0x21

//configuration: Reserved=00 , HPM = 00 (Normal Mode Reset Filter), HPCF=0111 (at ODR=190 then cutoff = 0.09 Hz)
#define CTRL_REG2_CONFIG 0b00'00'0'1'1'1

//register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG4 0x23

//configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
// Block Data Update = 0 (Continous value) , BLE = 0 LSB data at Lower Address , FS=01 (500 DPS) , Registered = 0 , Fixed = 00 , SIM = 0 (SPI interface Mode Selection = 4-Wire interface)  

//We increased the range to 2000 DPS by setting FS=11
//#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0 this was the lecture value
#define CTRL_REG4_CONFIG 0b0'0'11'0'00'0

#define SPI_FLAG 1

static const bool debugSwitch=1;

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


void printArri(float *a, int8_t K){
    if (debugSwitch) {printf("Start printArri\n============\n");}
    uint8_t i=0;
    //printf("%d \t\n",S);
    for (i=0 ; i<K ; i++){
        printf("%d \t %f \n",i,*(a+i));
    }
    if (debugSwitch) {printf("End printArri\n============\n");}
}

void printArr2d(float *a, int8_t k1, int8_t k2){ //k1 row, k2 col
    if (debugSwitch) {printf("Start printArr2d\n============\n");}
    uint8_t i=0,j=0,k=0;
    for (i=0 ; i<k1 ; i++){
        for (j=0; j<k2 ; j++){
            printf("%f \t",*(a+k));
            k=k+1;
        }
            printf("\n");
    }
    if (debugSwitch) {printf("End printArr2d\n============\n");}
}

void printArr2dInt(int16_t *a, int8_t k1, int8_t k2){ //k1 row, k2 col
    if (debugSwitch) {printf("Start---printArr2dInt\n");}
    uint8_t i=0,j=0,k=0;
    for (i=0 ; i<k1 ; i++){
        for (j=0; j<k2 ; j++){
            printf("%d \t",*(a+k));
            k=k+1;
        }
            printf("\n");
    }
    if (debugSwitch) {printf("End printArr2dInt\n============\n");}
}




float dotProduct2(float *an, float *bn, int8_t f, int8_t dim){
    if (debugSwitch) {printf("Start dotProduct============\n");}
    int8_t i=0;
    float r=0;
    for (i=0 ; i<f; i++){
        r= r + *(an+dim*i) * *(bn+dim*i);
        if (debugSwitch) printf("dotI\t[%d]\t a=%f\t b=%f a*b=%f\n",i,*(an+dim*i),*(bn+dim*i), *(an+dim*i) * *(bn+dim*i));
        
    }
    if (debugSwitch) {printf("r=%f\n",r);}
    if (debugSwitch) {printf("End dotProduct============\n");}
    return r;
    
}


void crossProduct3d(float *an, float *vn){
    //float ax=*(an);
    //float ay=*(an+1);
    //float az=*(an+2);

    if (debugSwitch) {printf("Start Crossproduct====\n");}
    *(vn) =   (*(an+1)) * zdim - (*(an+2)) * ydim;
    *(vn+1) = (*(an+2)) * xdim - (*(an))   * zdim;
    *(vn+2) = (*(an))   * ydim - (*(an+1)) * xdim;

    //if (debugSwitch) {("A1 Ads : %p \t %p \t %p \n", an , an+1 , an+2);}
    if (debugSwitch) {printf("A1 : [%f \t %f \t %f] \n", *(an) , *(an+1) , *(an+2));}
    if (debugSwitch) {printf("Cross Product V : [%f \t %f \t %f] \n",*(vn),*(vn+1),*(vn+2));}
    if (debugSwitch) {printf("End Crossproduct====\n");}
    
}



void filterRectangle( uint16_t f, uint16_t dim, float (*arr)[3]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
            arr[i][j]=(float) 1/f;
        }
    }
}


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


void blackmanWindow( uint16_t f, uint16_t dim, float (*arr)[3]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 0.42 - 0.5*cos(2*3.14*i/(f-1)) + 0.08*cos(4*3.14*i/(f-1));
        }
    }
}

void blackmanWindow1D( uint16_t f, uint16_t dim, float (*arr)[1]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 0.42 - 0.5*cos(2*3.14*i/(f-1)) + 0.08*cos(4*3.14*i/(f-1));
        }
    }
}


void unitWindow( uint16_t f, uint16_t dim, float (*arr)[3]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 1;
        }
    }
}

void unitWindow1D( uint16_t f, uint16_t dim, float (*arr)[1]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 1;
        }
    }
}

int factorial(uint16_t *n){
    uint16_t i=0, fact=1;
    for(i=1 ; i<=*n ; i++){    
      fact=fact*i;    
    }
    return fact;
}


float combination(uint16_t *a, uint16_t *b){
    float comb=1;
    uint16_t diff = *a - *b ;
    if (debugSwitch) {printf("--a=%d \ta!=%d \n",  *a,factorial(a) );}
    if (debugSwitch) {printf("--b=%d \tb!=%d \n",  *b,factorial(b) );}
    if (debugSwitch) {printf("--a-b=%d \t(a-b)!=%d \n", diff, factorial(&diff) );}
    comb = factorial(a)/(factorial(b) *factorial(&diff) ) ;///(factorial (b) * factorial (a-b));
    if (debugSwitch) {printf ("combination= %f \n", comb);}
    return comb;
}

void differentiator( uint16_t f, uint16_t dim, float (*arr)[3] ){//dim
    uint16_t i=0,j=0, n=f-1;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=pow(-1,i) * combination(&n,&i) * (1/pow(Ts2,f-1)); //Multiplied by the 1/Tsample
        }
    }
}

void hadamardProduct(uint16_t f, uint16_t dim, float (*arr1)[3] , float (*arr2)[3] , float (*arr3)[3] ){//dim
    if (debugSwitch) {printf("Start Haddamard Product======");}
    if (debugSwitch) {printf("f=%d , dim=%d",f,dim);}
    int i=0, j=0 ;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr3[i][j]= arr1[i][j] * arr2[i][j] ;
                if (debugSwitch) {printf("i=%d\t j=%d\t arr1[%d][%d]=%f\t arr2[%d][%d]=%f\t product=%f",i,j, i,j, arr1[i][j], i,j, arr2[i][j], arr3[i][j] );}

        }
    }
    if (debugSwitch) {printf("End Haddamard Product======");}
}


void hadamardProduct1D(uint16_t f, uint16_t dim, float (*arr1)[1] , float (*arr2)[1] , float (*arr3)[1] ){//dim
    if (debugSwitch) {printf("Start Haddamard Product======");}
    if (debugSwitch) {printf("f=%d , dim=%d",f,dim);}
    int i=0, j=0 ;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr3[i][j]= arr1[i][j] * arr2[i][j] ;
                if (debugSwitch) {printf("i=%d\t j=%d\t arr1[%d][%d]=%f\t arr2[%d][%d]=%f\t product=%f",i,j, i,j, arr1[i][j], i,j, arr2[i][j], arr3[i][j] );}

        }
    }
    if (debugSwitch) {printf("End Haddamard Product======");}
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

    write_buf[0]=CTRL_REG2;
    write_buf[1]=CTRL_REG2_CONFIG;
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    write_buf[0]=CTRL_REG4;
    write_buf[1]=CTRL_REG4_CONFIG;
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
    time_t tr;
    time(&tr);

    float gyro[M][d] = {0}; //initialie gyro buffer
    int16_t gyroRead[M][d] = {0}; //initialie gyro direct reading

    /*
    float H1[F1][d]; //initialize Filter Coefficients1
    float a1[F1][d] = {0}; //initialie filter input samples, multiplied by the filter coefficients
    float g1[M][d]={0}; //initialize filtered gyro buffer, output of dot product after being processed by filters.
    float W1[F1][d]; //initialize Blackman Window Coefficients
    */

    float H2[F2][d]; //initialize Filter Coefficients1
    float a2[F2][d] = {0}; //initialie filter input samples, multiplied by the filter coefficients
    float g2[M][d]={0}; //initialize filtered gyro buffer, output of dot product after being processed by filters.
    float W2[F2][d]; //initialize Filter Coefficients1

    /*
    float H3[F3][d]; //initialize Filter Coefficients1
    float a3[F3][d] = {0}; //initialie filter input samples, multiplied by the filter coefficients
    float g3[M][d]={0}; //initialize filtered gyro buffer, output of dot product after being processed by filters.
    float W3[F3][d]; //initialize Filter Coefficients1
    */

    float V[M][d]={0}; //Linear Velocity x,y,z components calculated from corss angular with chip dim
    float VLinear[M][1]={0}; //Linear Velocity Magnitude
    float HL[FL][1]={0}; //initialize Filter Coefficients for processing Linear Velocity Delayed Samples
    float aL[M][1]={0}; //List of Linear Velocity Magnitudes current and delayed to be used in processing
    float WL[FL][1]; //initialize Blackman Coefficients for Windowing the Filters Coefficients

    float gL[M][1]={0}; //Distance instantaneous resulted from integrating or corss product of VLinear
    float L=0, vL2 ; //Total Distance , and Linear Velocity Squared accumulator 



    int8_t i=0,j=0,k=0, t=0; //i for gyro readings counter, j for dim counter, k for filter coefficients, t for sample number
    

    int8_t w=0, r=(M+w-D)%M , v=0;


    printf("Start Init \n");
    
    /*
    filterRectangle(F1,d,H1);
    printf("--Moving Average Filter--\n");
    printArr2d(&H1[0][0],F1,d);// Filters
    */
    
    
    integratorTrapezoidal(F2,d,H2);
    printf("--Trapezoidal Integrator Filter--\n");
    printArr2d(&H2[0][0],F2,d);// Filters

    
    /*
    differentiator(F3,d,H3);
    printf("--Differentiator Filter--\n");
    printArr2d(&H3[0][0],F3,d);// Filters
    */
    
    integratorTrapezoidal1D(FL,1,HL);
    printf("--1 Dimensional Integrator Filter--\n");
    printArr2d(&HL[0][0],FL,1);// Filters
    
    /*
    blackmanWindow(F1,d,W1);
    printf("--Blackman Window Coefficients for Moving Average Filter--\n");
    printArr2d(&W1[0][0],F1,d);// Filters
    printf("--Moving Average Filter with Blackman Window--\n");
    hadamardProduct(F1,d,W1,H1,H1);
    printArr2d(&H1[0][0],F1,d);// Filters
    */

    blackmanWindow(F2,d,W2);
    unitWindow(F2,d,W2);
    printf("--Blackman Window Coefficients for Trapezoidal Integrator--\n");
    printArr2d(&W2[0][0],F2,d);// Filters
    hadamardProduct(F2,d,W2,H2,H2);
    printf("--Trapezoidal Integrator Filter with Blackman Window--\n");
    printArr2d(&H2[0][0],F2,d);// Filters

    /*
    blackmanWindow(F3,d,W3);
    printf("--Blackman Window Coefficients for Differentiator--\n");
    printArr2d(&W3[0][0],F3,d);// Filters
    hadamardProduct(F3,d,W3,H3,H3);
    printArr2d(&H3[0][0],F3,d);// Filters
    */

    blackmanWindow1D(FL,1,WL);
    unitWindow1D(FL,1,WL);
    printf("--Blackman Window Coefficients for Differentiator--\n");
    printArr2d(&WL[0][0],FL,1);// Filters
    hadamardProduct1D(FL,1,WL,HL,HL);
    printArr2d(&HL[0][0],FL,1);// Filters


    printf("Initial\n");
    //printf("---- M=%d \tF=%d \tD=%d \tdim=%d \tinf=%d \tsamplPerAxs=%d \n", M, F, D, d, inf , N );
    //printf("---- M=%d \tF=%d \tD=%d \tdim=%d \tinf=%d \tsamplPerAxs=%d \n", M, F, D, d, inf , N );
    printf("--w=%d \tr=%d \tv=%d \tt=%d\n", w,r,v,t );
    printf("Start Gyro\n");


    

    while(1)
    {
    if (debugSwitch) {printf("===Start Gyro Reading====\n" );}

    //printf("M=%d \tF=%d \tD=%d \tdim=%d \tinf=%d \tsamplPerAxs=%d \n", M, F1, D1, d, inf , N );




      //prepare the write buffer to trigger a sequential read
      write_buf[0]=OUT_X_L|0x80|0x40;
      //start sequential sample reading
      spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
      flags.wait_all(SPI_FLAG);
      
 
      for (j=0 ; j<d ; j++){

        gyroRead[j][w] = ( ( (uint16_t)read_buf[2*j+1+1] ) <<8 ) | ( (uint16_t)read_buf[2*j+1] );
        
        //Threshold if used ;
        //if (gyroRead[j][w] <= 1700 && gyroRead[j][w] >= -1700) {continue;}
        //

        gyro[j][w] = ((float) gyroRead[j][w])*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
        
        //current_time = localtime(&seconds);
        if (debugSwitch) {printf(" t=%d\t, w=%d\t, r=%d\t,  j=%d\t,         gyroRead[%d][%d]=%d\t, gyro[%d][%d]=%f \n",          t, w ,r  , j ,    w,j, gyroRead[w][j],     w,j ,gyro[w][j]);}



        //printf(">input \n t=%d \ti=%d \tj=%d \tw=%d \t r=%d \t v=%d \tgyro[%d][%d]=%f\n", t,i,j,w,r,v,w,j,j,gyro[w][j] );

        /*
        //Start Signal Prep block
        v=D1;
        //printf("===Start Prep\n");
        for (k=r+(D-D1) ; k < r+(D-D1)+F1 ; k++ ){
            a1[v][j]=gyro[k%M][j];
            //printf("w=%d \tr=%d \tk=%d \tk%M=%d \t deltaDelay=%d  \t v=%d \t D=%d, Di=%d, a1[%d][%d]=%f \t gyro[%d][%d]=%f \t ads=%p\n", w, r,k,k%M, (D-D1) ,v ,D,D1,v,j ,a1[v][j],    k%M , j, gyro[k%M][j] , &a1[k][j]);
            v=v-1;
        }
        //printf("===End Prep\n");
        //printf("===Start Filtered\n");
        g1[w][j]= dotProduct2(&a1[0][j], &H1[0][j],F1,d); 
        //printf(">i=%d \t w=%d \t r1=%d \t r1+(D1-D1)=%d \t g2[%d][%d]=%f \n",i, w,r1,r1+(D1-D2), w,j,g3[w][j]);
        //printf("w=%d \tr=%d \t(Delta Delay)=%d \t ri=r+deltaDelay=%d \tg1[%d][%d]=%f ads=%p\n", w, r, (D-D1) , r+(D-D1),w ,j ,g1[w][j], &g1[w][j]    );
        //printf("===End Filtered\n");
        //End Signal Prop Block
        */
        
        //Start Signal Prep block
        v=D2;
        if (debugSwitch) {printf("===Start Prep for Filter 2 using D2 Delays\n");}
        for (k=r+(D-D2) ; k < r+(D-D2)+F2 ; k++ ){
            a2[v][j]=gyro[k%M][j];
            if (debugSwitch) {printf("w=%d \tr=%d \tk=%d \t(k mod M)=%d \t deltaDelay=%d  \t v=%d \t D=%d, Di=%d, a1[%d][%d]=%f \t gyro[%d][%d]=%f \t ads=%p\n",             w, r,k,k%M, (D-D2) ,v ,D,D2,v,j ,a2[v][j],    k%M , j, gyro[k%M][j] , &a2[k][j]);}
            v=v-1;
        }
        if (debugSwitch) {printf("===End Prep for Filter 2 using D2 Delays\n");}
        if (debugSwitch) {printf("===Start Applying Filter H2\n");}
        g2[w][j]= dotProduct2(&a2[0][j], &H2[0][j],F2,d); 
        //printf(">i=%d \t w=%d \t r1=%d \t r1+(D1-D1)=%d \t g2[%d][%d]=%f \n",i, w,r1,r1+(D1-D2), w,j,g3[w][j]);
        if (debugSwitch) {printf("w=%d \tr=%d \t(Delta Delay)=%d \t ri=r+deltaDelay=%d \tg1[%d][%d]=%f ads=%p\n", w, r, (D-D2) , r+(D-D2),w ,j ,g2[w][j], &g2[w][j]    );}
        if (debugSwitch) {printf("===End Applying Filter H2\n");}
        //End Signal Prop Block
        
        /*
        //Start Signal Prep block
        v=D3;
        printf("===Start Prep\n");
        for (k=r+(D-D3) ; k < r+(D-D3)+F3 ; k++ ){
            a3[v][j]=gyro[k%M][j];
            printf("w=%d \tr=%d \tk=%d \tk%M=%d \t deltaDelay=%d  \t v=%d \t D=%d, Di=%d, a3[%d][%d]=%f \t gyro[%d][%d]=%f \t ads=%p\n", w, r,k,k%M, (D-D3) ,v ,D,D3,v,j ,a3[v][j],    k%M , j, gyro[k%M][j] , &a3[k][j]);
            v=v-1;
        }
        printf("===End Prep\n");
        printf("===Start Filtered\n");
        g3[w][j]= dotProduct2(&a3[0][j], &H3[0][j],F3,d); 
        printf("w=%d \tr=%d \t(Delta Delay)=%d \t ri=r+deltaDelay=%d \tg3[%d][%d]=%f ads=%p\n", w, r, (D-D3) , r+(D-D3),w ,j ,g3[w][j], &g3[w][j]    );
        printf("===End Filtered\n");
        //End Signal Prop Block
        */

    if (debugSwitch) {printf("==End Read Gyro dim \n");}
    }

    //Logic for Pulse
    //pulse=pulse+1;
    //if (pulse >= 3) {step=step+1; distance=step*stepDistance;printf("step=%d , distance=%f",step, distance)} 
    //

    if (debugSwitch) {printf("==3 Dimensional Gyro Readings and Their Conversions\n");}
    if (debugSwitch) {printf("gyroRead[%d][j]=[%d\t %d\t %d\t]\n",w,gyroRead[w][0],gyroRead[w][1],gyroRead[w][2]);}
    printf(">gyroReadx:%dg\n",gyroRead[w][0]);
    printf(">gyroReady:%dg\n",gyroRead[w][1]);
    printf(">gyroReadz:%dg\n",gyroRead[w][2]);
    if (debugSwitch) {printf("gyro[%d][j]=[%f\t %f\t %f\t]",w,gyro[w][0],gyro[w][1],gyro[w][2]);}
    printf(">gyrox:%f\n",gyro[w][0]);
    printf(">gyroy:%f\n",gyro[w][1]);
    printf(">gyroz:%f\n",gyro[w][2]);

    //Section to calculate Instantaneous Velocity in x,y,z 
    if (debugSwitch) {printf("===Start Cross Product of Instanenous gyro g2 to get Velocity in XYX\n");}
    if (d==3) {
        crossProduct3d(&gyro[w][0] , &V[w][0]);
        
        //Equivalent to Cross product
        //V[w][0] = g2[w][1] * zdim - g2[w][2] * ydim ;
        //V[w][1] = g2[w][2] * xdim - g2[w][0] * zdim ;
        //V[w][2] = g2[w][0] * ydim - g2[w][1] * xdim ;
        //
        
    }
    else if (d==2) {V[w][0] =g2[w][0]*ydim ; V[w][1] = g2[w][1]*xdim ; }
    else { V[w][0] = g2[w][0]*xdim ; }
    if (debugSwitch) {printf("Instantaneous Linear Velocity in X,Y,Z after corss V[w][j]=V[%d][j]= [%f \t%f \t%f]\n",w,V[w][0],V[w][1],V[w][2]);}// Filters
    if (debugSwitch) {printf("===End Cross Product of Instanenous gyro g2 to get Velocity in XYX\n");}


    //Calculate Instantaneous Velocity Magnitude
    if (debugSwitch) {printf("===Start Calculate Instantaneous Velocity Magnitude\n");}
    vL2=0;
    for (j=0 ; j<d ; j++){
        vL2=vL2+pow(V[w][j],2);
    }
    VLinear[w][0]=sqrt(vL2);
    if (debugSwitch) {printf("Instantaneous Linear Velocity Magnitude after Squaring = VLinear[w][0]=VLinear[%d][0]= %f\n",w,VLinear[w][0]);}// Filters
    if (debugSwitch) {printf("===Instantaneous Linear Velocity Magnitude Array VLinear[M][0]\n");}
    if (debugSwitch) {printArr2d(&VLinear[0][0], M,1);}
    if (debugSwitch) {printf("===End Calculate Instantaneous Velocity Magnitude\n");}

    
    //Integrator based on trapezoidal
    if (debugSwitch) {printf("===Start Calculate Instantaneous Distance Magnitude based on Integration of Linear Velocity Magnitude\n");}
    v=DL;
    for (k=r+(D-DL) ; k < r+(D-DL)+FL ; k++ ){
        aL[v][0]=VLinear[k%M][0];
        if (debugSwitch) {printf("w=%d \tr=%d \tk=%d \t (k mod M)=%d \t deltaDelay=%d  \t v=%d \t D=%d, Di=%d, aL[%d][0]=%f \t vLinear[%d][0]=%f \t ads=%p\n",         w, r,k,k%M, (D-DL) ,v ,D,DL,v ,aL[v][0],    k%M , VLinear[k%M][0] , &aL[v][0]);}
        v=v-1;
    }
    gL[w][0]= dotProduct2(&aL[0][0], &HL[0][0],FL,1); //Integration of Velocity Magnitude 
    if (debugSwitch) {printf("Instantaneous Linear Distance Magnitude gL[w][0]=gL[%d][0]= %f\n",w,gL[w][0]);}// Filters
    if (debugSwitch) {printf("===Instantaneous Linear Distance Magnitude Array VLinear[M][0]\n");}
    if (debugSwitch) {printArr2d(&gL[0][0], M,1);} //filtered
    printf("===End Calculate Instantaneous Linear Distance Magnitude based on Integration of Linear Velocity Magnitude Array\n");
    

    if (debugSwitch) {printf("Accumulated Distance Before Adding current Disantce= L = %f\n \t Instantaneous Linear Distance Magnitude=gL[w][0]=gL[%d][0]=%f\n", L,w,gL[w][0] );}
    L=L+gL[w][0];
    if (debugSwitch) {printf("Accumulated Distance After Adding current Disantce= L = %f\n \t Instantaneous Linear Distance Magnitude=gL[w][0]=gL[%d][0]=%f\n", L,w,gL[w][0] );}
    if (debugSwitch) {printf(">Total Distance:%f\n",L);}
    if (debugSwitch) {printf(">Instant Distance:%f\n", gL[w][0]);}
    

    if (debugSwitch){
    printf("--Buffer Gyro Readings float--\n");
    printArr2dInt(&gyroRead[0][0],M,d);// Filters


    printf("--Buffer Gyro Readings Converted and down scaled--\n");
    printArr2d(&gyro[0][0],M,d);// Filters


    printf("--Delayed Gyro Input to Filter--\n");
    printArr2d(&a2[0][0],F2,d);// Filters
    printf("--Trapezoidal Integrator Filter with Unit Window--\n");
    printArr2d(&H2[0][0],F2,d);// Filters
    printf("--Buffer Filtered Gyro Output Buffer--\n");
    printArr2d(&g2[0][0],M,d);// Filters
    
    
    printf("--Buffer 3D Linear Velocity instant in x,y,z--\n");
    printArr2d(&V[0][0],M,d);// Filters
    printf("--Buffer 1D Linear Velocity instant from corss product g2--\n");
    printArr2d(&VLinear[0][0],M,1);// Filters

    printf("--Delayed 1D Linear Velocity instant--\n");
    printArr2d(&aL[0][0],FL,1);// Filters
    printf("--Integrator Filter for 1D Linear Velocity--\n");
    printArr2d(&HL[0][0],FL,1);// Filters

    printf("--Buffer Distance from integrated 1D Linear Velocity--\n");
    printArr2d(&gL[0][0],M,1);// Filters
    }
       
    //Increment Pointers
    w=(w+1)%M;
    r=(r+1)%M;
    t=t+1;

    thread_sleep_for(Tdelay * 1000);
    if((t%M) == 0){
        lcd_clear();
    }
    if (debugSwitch) {printf("===End Gyro Reading====\n");}
  }

}
