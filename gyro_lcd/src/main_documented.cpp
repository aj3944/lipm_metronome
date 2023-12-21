#include "mbed.h"
#include "../lib/LCD_DISCO/LCD_DISCO_F429ZI.h"


//Normalized coordinates for the radius in ; Linear Velocity = Angular Velocity X Radius, Radius values can be calibrated per indvidual length
#define xdim 1
#define ydim 1
#define zdim 1

#define d 3 //number of dim of gyroscope readings
//Circular Buffer Length, number of samples in buffer per axis, The buffer will hold M samples (x[n], x[n-1],....,x[n-(M-1)])
//So the buffer can provide up to D=M-1 Delayed samples, The initial status of write pointer will point to the current sample x[n] and initially will be at index=0 , 
//the read pointer will point to the max delayed unit x[n - (M-1)] and will be at ; p = (w-D)%M = (0-D)%M = (M-D )%M = (1)%M = 1
//Pointers will be updated as ; w=(w+1)%M and r=(r+1)%M
//If we need to extract samples with Di delays, then we need Di+1 = Fi coefficients, so we need Fi samples = x[n],x[n-1],...x[n-(Di-1)] ,
//So we can use the same buffer but with counter to the initial read pointer from k=r+(D-Di) till k=r+(D-Di)+Fi .
#define M 16 
#define F (M) //Supported Filter Coefficients , Number of Filter coefficients= (M-1) Delayed samples + 1 current , Fmax = M 
#define D (M-1) //Supported Delayed Coefficients , The filter order 


//Tune : Tsample
//Human step size has a max frequency of 1.5 Hz (1.5 step per second), the challenge requirement was to use 2 Hz sampling (1 sample per 0.5 seconds)
//So we choose to process M-samples per 0.5 second, and aggregate the processing results in a slower buffer of length 40 as per the challenge requirements
//The faster sampling rate is to provide smoother results with the DSP filters, so each processed M samples will construct a 1 2Hz sample in the 40-samples slower buffer.
//The faster sampling rate will have Tdelay = 0.5/M 
#define Tdelay (0.5/M)
#define Ts2 Tdelay //We will use the same delay for the integrator, we could different sampling rates for the integrator and the data collection but we choose to align both


//Tune : Use Only in Debugging when manual values are inserted
//#define N 7  //number of readings provided by gyro per x-axis = inf/d
//#define inf ((N)*1) //number of samples taken from gyro, should be infinity
//



//Tune : Gyro Thresholds in g to avoid the static noise and transients
#define xHThreshold 500
#define xLThreshold -500


//We Define the filters orders separately to allow different tuning for the filters design

//Tune : A full line filter , Not used in this current version
//#define F1 (M) //Filter coefficients, n Delays + 1 current, F <=M , Fmax=M
//#define D1 (F1-1) //Filter Order, Number of delay line samples, D=F-1 , Dmax = M-1, D<=M-1, this is for the Filter delay

#define F2 (M) //Trapezoidal integrator coefficients array , (M-1) delay units + 1 current sample to give total M samples, should be odd to staisfiy Linear FIR requirements, so min integrator = 3
#define D2 (F2-1) //The number of supported delay units, which is the filter order.

//Tune : Use Differentiator to detect peaks
#define F3 (M-1) // F3= 1 is the First  Difference, x(n)-x(n-1)
#define D3 (F3-1) //The delay lines or filter order

#define FL (M) //Trapezoidal integrator coefficients array, used to integrate the velocity to get the instantaneous distance
#define DL (FL-1) //The number of supported delay units, which is the filter order.



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
//#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0 this was the lecture value, we increased the range to be able to detect peaks
#define CTRL_REG4_CONFIG 0b0'0'11'0'00'0

#define SPI_FLAG 1

static const bool debugSwitch=0;  //Print functiosn processing and outputs if enabled
static const bool debugSwitch2=1; //Print the fast buffered signals M-samples, sampled at 0.5/M seconds in teleplot-compatible format
static const bool debugSwitch3=0; //Print the Arrays on every reading
static const bool debugSwitch4=1; //Print the slow buffer(40-samples), sampled at 0.5 seconds in teleplot-compatible format

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



void printArr2d(float *a, int8_t k1, int8_t k2){ 
    //Prints a 2-dimensional float array with k1 rows, k2 columns
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

void printArr2dInt(int16_t *a, int8_t k1, int8_t k2){ 
    //Prints a 2-dimensional integer array with k1 rows, k2 columns
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
    //returns dotP = a dot product b , where a and b are 2-dim arrays with f rows and dim columns,
    //this version requires dim=1 always
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


void crossProduct3d(float *an, float *vn){ 
    //returns vn vector = an corss product the x,y,z dimensions defined in the #define section
    //mainly used to calculate the linear velocity components vx,vy and vz from angular velocity gx,gy,gz
    //In this version the angular velocity provided as an argument is typyically an integrated version g2 to provide smoother angular velocities
    // The x,y,z coordinates of vector an (the gyro filtered signal) is ;
    // ax=*(an);
    // ay=*(an+1);
    // az=*(an+2);

    if (debugSwitch) {printf("Start Crossproduct====\n");}
    *(vn) =   (*(an+1)) * zdim - (*(an+2)) * ydim;
    *(vn+1) = (*(an+2)) * xdim - (*(an))   * zdim;
    *(vn+2) = (*(an))   * ydim - (*(an+1)) * xdim;

    if (debugSwitch) {printf("Cross Product Input an Address : %p \t %p \t %p \n", an , an+1 , an+2);}
    if (debugSwitch) {printf("Cross Product Input an : [%f \t %f \t %f] \n", *(an) , *(an+1) , *(an+2));}
    if (debugSwitch) {printf("Cross Product Result vn : [%f \t %f \t %f] \n",*(vn),*(vn+1),*(vn+2));}
    if (debugSwitch) {printf("End Crossproduct====\n");}
    
}



void filterRectangle( uint16_t f, uint16_t dim, float (*arr)[3]){
    //fills in the array arr witha an f filter coefficients x dim dimensions rectangular filter, with Tsampling considered
    //e.g. to generate  matrix arr with a 3rd order (4-delayed samples) x 3 dim filter ==> filterRectangle(4,3,&arr[0][0]) :
    //gives a 4x3 matrix where each column = Tsampling * [1/4, 1/4 , 1/4 , 1/4] for each dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
            arr[i][j]=(float) 1/f * Ts2;
        }
    }
}


void integratorTrapezoidal( uint16_t f, uint16_t dim, float (*arr)[3]){
    //fills in the array arr witha an f filter coefficients x dim dimensions trapezoidal filter for integrator, with Tsampling considered
    //e.g. to generate a matrix arr with a 3rd order (4-delayed samples) x 3 dim filter ==> integratorTrapezoidal(4,3,&arr[0][0]) : 
    //gives a 4x3 matrix where each column = Tsampling * [1/2, 1 , 1 , 1/2] for each dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
            if (i==0 || i==f-1){
                arr[i][j]=(float) ((float)1/2) * Ts2; // Ts2 is multiplied to accomodate the sampling frequency
            }
            else {arr[i][j]=(float) 1 * Ts2;}
        }
    }
}

void integratorTrapezoidal1D( uint16_t f, uint16_t dim, float (*arr)[1]){
    //fills in the array arr witha an f filter coefficients 1 dim dimensions trapezoidal filter for integrator, with Tsampling considered
    //e.g. to generate a matrix arr with a 3rd order (4-delayed samples) x 1 dim filter ==> integratorTrapezoidal1D(4,1,&arr[0][0]) : 
    // gives a 4x1 matrix where the only column = Tsampling * [1/2, 1 , 1 , 1/2]
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
            if (i==0 || i==f-1){
                arr[i][j]=(float) ((float)1/2) * Ts2; // Ts2 is multiplied to accomodate the sampling frequency
            }
            else {arr[i][j]=(float) 1 * Ts2;}
        }
    }
}


void blackmanWindow( uint16_t f, uint16_t dim, float (*arr)[3]){
    //fills in the array arr with a fxdim matrix of Blackman Window coefficients, Window functions are used to mitigate the finite window effect instead of truncating the filters which
    //could cause high frequency components due to the abrupt changes
    //Blackman window is used as it one of the common window functions
    //e.g. to generate a matrix arr with a 3rd order (4-delayed samples) x 3 dim filter ==> blackmanWindow(4,3,&arr[0][0]) :
    // gives a 4x3 matrix where each column = [4 blackman coefficients]
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 0.42 - 0.5*cos(2*3.14*i/(f-1)) + 0.08*cos(4*3.14*i/(f-1));
        }
    }
}

void blackmanWindow1D( uint16_t f, uint16_t dim, float (*arr)[1]){
    //same like blackmanWindow but for a 1-dimensional array, i.e.
    //e.g. to generate a matrix arr with a 3rd order (4-delayed samples) x 1 dim filter ==> blackmanWindow(4,1,&arr[0][0]) :
    // gives a 4x1 matrix where each column = [4 blackman coefficients]
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 0.42 - 0.5*cos(2*3.14*i/(f-1)) + 0.08*cos(4*3.14*i/(f-1));
        }
    }
}


void unitWindow( uint16_t f, uint16_t dim, float (*arr)[3]){
    //Used for debugging only, fills arr with a unit window coefficients giving a fxdim all "1"s matrix , 
    //to allow easier debugging of integrator/differentiator filters
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 1;
        }
    }
}

void unitWindow1D( uint16_t f, uint16_t dim, float (*arr)[1]){
    //Used for debugging only, similar to unitWindow but for 1D , fills arr with unit window coefficients giving a fx1 all "1"s matrix , to
    // allow easier debugging of integrator/differentiator filters
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 1;
        }
    }
}


int factorial(uint16_t *n){
    //returns fact(n) = n! , used in differentiator coefficients
    uint16_t i=0, fact=1;
    for(i=1 ; i<=*n ; i++){    
      fact=fact*i;    
    }
    return fact;
}


float combination(uint16_t *a, uint16_t *b){
    //returns comb(a,b) = a!/(b! (a-b)!) , used in differentiator coefficients
    float comb=1;
    uint16_t diff = *a - *b ;
    if (debugSwitch) {printf("--a=%d \ta!=%d \n",  *a,factorial(a) );}
    if (debugSwitch) {printf("--b=%d \tb!=%d \n",  *b,factorial(b) );}
    if (debugSwitch) {printf("--a-b=%d \t(a-b)!=%d \n", diff, factorial(&diff) );}
    comb = factorial(a)/(factorial(b) *factorial(&diff) ) ;///(factorial (b) * factorial (a-b));
    if (debugSwitch) {printf ("combination= %f \n", comb);}
    return comb;
}

void differentiator( uint16_t f, uint16_t dim, float (*arr)[3] ){
    //Could be used on the gyro readings instead of the threshold to suppress the static noise
    //fills in the array arr with a fxdim matrix of differentiator coefficients,differnetiators are peak detectors and can act as BPF
    //e.g. to generate a matrix arr with 3rd order (4-delayed samples) x 3 dim filter ==> differentiator(4,3,&arr[0][0]) :
    // gives a 4x3 matrix where each column = [4 blackman coefficients]
    uint16_t i=0,j=0, n=f-1;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=pow(-1,i) * combination(&n,&i) * (1/pow(Ts2,f-1)); //Multiplied by the 1/Tsample
        }
    }
}

void hadamardProduct(uint16_t f, uint16_t dim, float (*arr1)[3] , float (*arr2)[3] , float (*arr3)[3] ){
    // hadamard product is the element-wise multiplication, used to multiply window function coefficients with filter coefficients to "window" the filters coefficients
    // usage hadamardProduct(F2,d,W2,H2,H2); H2 = W2 * H2 element-wise , where all matrices are F2xd dimensions,
    // W2 is a window function matrix, H2 is a matrix for filter coefficients
    if (debugSwitch) {printf("Start hadamardProduct Product======\n");}
    if (debugSwitch) {printf("f=%d , dim=%d\n",f,dim);}
    int i=0, j=0 ;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr3[i][j]= arr1[i][j] * arr2[i][j] ;
                if (debugSwitch) {printf("i=%d\t j=%d\t arr1[%d][%d]=%f\t arr2[%d][%d]=%f\t product=%f\n",i,j, i,j, arr1[i][j], i,j, arr2[i][j], arr3[i][j] );}

        }
    }
    if (debugSwitch) {printf("End hadamardProduct Product======\n");}
}


void hadamardProduct1D(uint16_t f, uint16_t dim, float (*arr1)[1] , float (*arr2)[1] , float (*arr3)[1] ){//dim
    // hadamardProduct1D this is similar to hadamardProduct but for 1 dimensions
    // usage hadamardProduct1D(F2,1,W2,H2,H2); H2 = W2 * H2 element-wise , where all matrices are F2x1 dimensions,
    // W2 is a window function matrix, H2 is a matrix for filter coefficients
    if (debugSwitch) {printf("Start hadamardProduct Product======\n");}
    if (debugSwitch) {printf("f=%d , dim=%d\n",f,dim);}
    int i=0, j=0 ;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr3[i][j]= arr1[i][j] * arr2[i][j] ;
                if (debugSwitch) {printf("i=%d\t j=%d\t arr1[%d][%d]=%f\t arr2[%d][%d]=%f\t product=%f\n",i,j, i,j, arr1[i][j], i,j, arr2[i][j], arr3[i][j] );}

        }
    }
    if (debugSwitch) {printf("End hadamardProduct Product======\n");}
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

    //Tune : Use Local time in display
    //time_t seconds = time(NULL);
    //time_t tr;
    //time(&tr);

    float gyro[M][d] = {0}; //Buffer to hold gyro scaled readings
    int16_t gyroRead[M][d] = {0}; //initialie gyro direct reading

    /* Tune : Use a full length integrator, i.e. Delay units = Buffer Size M - 1
    float H1[F1][d]; //initialize Filter Coefficients1
    float a1[F1][d] = {0}; //initialie filter input samples, multiplied by the filter coefficients
    float g1[M][d]={0}; //initialize filtered gyro buffer, output of dot product after being processed by filters.
    float W1[F1][d]; //initialize Blackman Window Coefficients
    */


    //For Trapezoidal integrator applied to the gyro scaled values
    float H2[F2][d]; //initialize Non-Windowed Filter Coefficients for integrator,  filter size is F2 that can <= M , in this example F2=M ==> have M-1 delays and 1 current
    float W2[F2][d]; //initialize Window Coefficients that will be used to "window" the filter coefficients to avoid abrupt truncation
    float a2[F2][d] = {0}; //initialize a buffer to keep the last (D2) samples + the current samples (x[n],x[n-1],..,x[n-(D1-1)]) to be used in integrator 
    float g2[M][d]={0}; //initialize a buffer to hold the filtered gyro samples After applying the windowed filters

    
    // Tune : Use if needed
    //For Differentiator
    float H3[F3][d]; 
    float a3[F3][d] = {0}; 
    float g3[M][d]={0};
    float W3[F3][d]; 
    

    //For the x,y,z velocity, Linear velocity magnitude, instantaneous distance and total distance
    float V[M][d]={0}; //Linear Velocity x,y,z components calculated from corss angular with chip dim
    float VLinear[M][1]={0}; //Linear Velocity Magnitude, V = Squrt (Vx^2 + Vy^2 + Vz^2)
    float HL[FL][1]={0}; //initialize Filter Coefficients for the trapezoidal integrator filter coefficients
    float WL[FL][1]; //initialize Blackman Coefficients for Windowing the Filters Coefficients
    float aL[FL][1]={0}; //initialize a buffer to keep the last FL delayed Linear Velocity Magnitude Samples to be used in the integration 
    float gL[M][1]={0}; //Distance instantaneous resulted from integrating the delayed FL samples from VLinear (integration of aL)
    float L=0, vL2=0 ; //Total Distance , and Linear Velocity Squared accumulator 


    //Buffer of 40 samples filled every 0.5 seconds by the M-th processed faster buffer samples.
    //i.e. the Main fast buffer samples and processes the gyro readings every 0.5/M seconds, so every M-gyro readings will happen every 0.5 seconds ,and this
    //M-sample from the fast buffer will fill in 1 sample from the slower 40-samples buffer
    //So all of these buffers are the processed values from the faster buffer, but slower
    int16_t gyroReadSlow[40][d]={0} ; //to hold gyro readings
    float gyroSlow[40][d]={0} ;   //to hold scaled gyro samples
    float g2Slow[40][d]={0} ;   //to hold integrated gyro samples
    float VSlow[40][d]={0} ;  //to hold Velocity in x,y,z 
    float VLinearSlow[40][1]={0} ;  //to hold Linear Velocity Magnitude
    float gLSlow[40][1]={0} ;   //To hold instantenous distance
    float LSlow=0 ;  //to hold the total distance


    int8_t i=0,j=0,k=0, t=0; //i for gyro readings counter to fill the slow buffer , j for dimension counter x,y and z , k for filter coefficients counter, t for sample number
    

    //Initial pointers location
    // write will start from the begninning of the buffer so w=0 , read will point to Max Delay x(n-(M-1)), so r=(w-D)%M = 1
    // v is a redundant counter and is only used to make sure that the slice of the ciruclar buffer x(n) will match h(0) for filter coefficients
    // without v , the h(0) in filter coefficients will match x(n-(M-1)) which is not an issue for symmetric buffer but will require reversing for non-symmetric buffer
    int8_t w=0, r=(M+w-D)%M , v=0; 


    printf("Start Init \n");
    
    /* Tune : Using Averaging
    filterRectangle(F1,d,H1);
    printf("--Moving Average Filter--\n");
    printArr2d(&H1[0][0],F1,d);
    blackmanWindow(F1,d,W1);
    if (debugSwitch) {unitWindow(F1,d,W1);}
    printf("--Blackman Window Coefficients for Moving Average Filter--\n");
    printArr2d(&W1[0][0],F1,d);
    printf("--Moving Average Filter with Blackman Window--\n");
    hadamardProduct(F1,d,W1,H1,H1);
    printArr2d(&H1[0][0],F1,d);
    */   


    // Prepare integrator for scaled gyro values,
    // H2 : Filter integrator coefficients, F2: the filter order+1 = size of filter , W2 : The Window Coefficients , all with size F2 * d matrices
    integratorTrapezoidal(F2,d,H2);
    printf("--Trapezoidal Integrator Filter--\n");
    printArr2d(&H2[0][0],F2,d);
    blackmanWindow(F2,d,W2);
    if (debugSwitch) {unitWindow(F2,d,W2);}
    printf("--Blackman Window Coefficients for Trapezoidal Integrator--\n");
    printArr2d(&W2[0][0],F2,d);
    hadamardProduct(F2,d,W2,H2,H2);
    printf("--Trapezoidal Integrator Filter with Blackman Window--\n");
    printArr2d(&H2[0][0],F2,d);

    
    
    // Prepare Differentiator if used
    differentiator(F3,d,H3);
    printf("--Differentiator Filter--\n");
    printArr2d(&H3[0][0],F3,d);
    blackmanWindow(F3,d,W3);
    if (debugSwitch) {unitWindow(F3,d,W3);}
    printf("--Blackman Window Coefficients for Differentiator--\n");
    printArr2d(&W3[0][0],F3,d);
    hadamardProduct(F3,d,W3,H3,H3);
    printArr2d(&H3[0][0],F3,d);
    

    // Prepare integrator for Linear Velocity in 1 dimension,
    // HL : Filter integrator coefficients, FL: the filter order+1 = size of filter , WL : The Window Coefficients , all with size FL * 1 matrices    
    integratorTrapezoidal1D(FL,1,HL);
    printf("--1 Dimensional Integrator Filter--\n");
    printArr2d(&HL[0][0],FL,1);
    blackmanWindow1D(FL,1,WL);
    if (debugSwitch) {unitWindow1D(FL,1,WL);}
    printf("--Blackman Window Coefficients for Differentiator--\n");
    printArr2d(&WL[0][0],FL,1);
    hadamardProduct1D(FL,1,WL,HL,HL);
    printArr2d(&HL[0][0],FL,1);
        

    printf("Initial\n");
    printf("--w=%d \tr=%d \tv=%d \tt=%d\n", w,r,v,t );
    printf("Start Gyro\n");


    while(1)
    {
    if (debugSwitch) {printf("===Start Gyro Reading===================================================\n" );}

      //prepare the write buffer to trigger a sequential read
      write_buf[0]=OUT_X_L|0x80|0x40;
      //start sequential sample reading
      spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
      flags.wait_all(SPI_FLAG);
      
    
        //For each dimension , process each dimension reading sequentially
      for (j=0 ; j<d ; j++){

        gyroRead[w][j] = ( ( (uint16_t)read_buf[2*j+1+1] ) <<8 ) | ( (uint16_t)read_buf[2*j+1] );
        
        //Threshold if used ;
        //if (gyroRead[j][w] <= xHThreshold && gyroRead[j][w] >= xLThreshold) {gyroRead[j][w]=0;} //If within threshold then reset the readings to eliminate static noise
        //

        //Scale Gyro readings into rad/sec
        gyro[w][j] = ((float) gyroRead[w][j])*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
        
        if (debugSwitch) {printf(" t=%d\t, w=%d\t, r=%d\t,  j=%d\t,         gyroRead[%d][%d]=%d\t, gyro[%d][%d]=%f \n",          t, w ,r  , j ,    w,j, gyroRead[w][j],     w,j ,gyro[w][j]);}


        /* Tune : Apply a full length integrator or not
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
        
        //Integrating the gyro readings with filter order D2 (D2 delayed samples and 1 current sample) with trapezopidal integrator ([1/2,1,1,1...,1/2]) ,
        //and Ts2 sampling frequency
        // with Blackman Window to smooth the signal
        //Start Signal Prep block

        //Slice a part of ciruclar buffer that matches filter size
        v=D2;
        if (debugSwitch) {printf("===Start Prep for Filter 2 using D2 Delays\n");}
        for (k=r+(D-D2) ; k < r+(D-D2)+F2 ; k++ ){
            a2[v][j]=gyro[k%M][j];
            if (debugSwitch) {printf("w=%d \tr=%d \tk=%d \t(k mod M)=%d \t deltaDelay=%d  \t v=%d \t D=%d, Di=%d, a2[%d][%d]=%f \t gyro[%d][%d]=%f \t ads=%p\n",             w, r,k,k%M, (D-D2) ,v ,D,D2,v,j ,a2[v][j],    k%M , j, gyro[k%M][j] , &a2[k][j]);}
            v=v-1; //Decrment V to make sure the first element in the loop will match the last element of the slice to align slice with filter coefficients
        }
        if (debugSwitch) {printf("===End Prep for Filter 2 using D2 Delays\n");}
        if (debugSwitch) {printf("===Start Applying Filter H2\n");}
        
        //g2[n][x or y or z dimension] signal is the result of applying Windowed Filter H2 on the slice a2
        g2[w][j]= dotProduct2(&a2[0][j], &H2[0][j],F2,d); 
        if (debugSwitch) {printf("w=%d \tr=%d \t(Delta Delay)=%d \t ri=r+deltaDelay=%d \tg2[%d][%d]=%f ads=%p\n", w, r, (D-D2) , r+(D-D2),w ,j ,g2[w][j], &g2[w][j]    );}
        if (debugSwitch) {printf("===End Applying Filter H2\n");}
        //End Signal Prop Block
        
        // Tune : Apply Differentiator to detect the spikes (act as BPF especially for higher order D3>2 )
        //Start Signal Prep block
        v=D3;
        if (debugSwitch) {printf("===Start Prep for Filter 3 using D3 Delays\n");}
        for (k=r+(D-D3) ; k < r+(D-D3)+F3 ; k++ ){
            a3[v][j]=gyro[k%M][j];
            if (debugSwitch) {printf("w=%d \tr=%d \tk=%d \t(k mod M)=%d \t deltaDelay=%d  \t v=%d \t D=%d, Di=%d, a3[%d][%d]=%f \t gyro[%d][%d]=%f \t ads=%p\n",             w, r,k,k%M, (D-D2) ,v ,D,D2,v,j ,a2[v][j],    k%M , j, gyro[k%M][j] , &a2[k][j]);}
            v=v-1;
        }
        if (debugSwitch) {printf("===End Prep for Filter 3 using D3 Delays\n");}
        if (debugSwitch) {printf("===Start Applying Filter H3\n");}
        g3[w][j]= dotProduct2(&a3[0][j], &H3[0][j],F3,d); 
        //printf(">i=%d \t w=%d \t r1=%d \t r1+(D1-D1)=%d \t g3[%d][%d]=%f \n",i, w,r1,r1+(D1-D3), w,j,g3[w][j]);
        if (debugSwitch) {printf("w=%d \tr=%d \t(Delta Delay)=%d \t ri=r+deltaDelay=%d \tg3[%d][%d]=%f ads=%p\n", w, r, (D-D3) , r+(D-D3),w ,j ,g3[w][j], &g3[w][j]    );}
        if (debugSwitch) {printf("===End Applying Filter H3\n");}
        //End Signal Prop Block
        

    if (debugSwitch) {printf("==End Read Gyro dim \n");}
    }

    //Tune : Logic for Pulse
    //pulse=pulse+1;
    //if (pulse >= 3) {step=step+1; distance=step*stepDistance;printf("step=%d , distance=%f",step, distance)} 
    //

    if (debugSwitch) {printf("==3 Dimensional Gyro Readings and Their Conversions\n");}
    if (debugSwitch) {printf("gyroRead[%d][j]=[%d\t %d\t %d\t]\n",w,gyroRead[w][0],gyroRead[w][1],gyroRead[w][2]);}
    if (debugSwitch) {printf("gyro[%d][j]=[%f\t %f\t %f\t]\n",w,gyro[w][0],gyro[w][1],gyro[w][2]);}
    

    //Section to calculate Instantaneous Velocity in x,y,z 
    if (debugSwitch) {printf("===Start Cross Product of Instanenous gyro g2 to get Velocity in XYX\n");}
    if (d==3) {
        //Tune: Use a function to do the cross product ; g (gyro filtered signal) X r (normalized or calibrated radius) = V (Linear Velocity in x,y,z)
        //crossProduct3d(&g2[w][0] , &V[w][0]);
        
        //Equivalent to Cross product
        V[w][0] = g2[w][1] * zdim - g2[w][2] * ydim ;
        V[w][1] = g2[w][2] * xdim - g2[w][0] * zdim ;
        V[w][2] = g2[w][0] * ydim - g2[w][1] * xdim ;
        //
        
    }
    //Section is only used if the gyro will provide only 2 or 1 dimensions
    else if (d==2) {V[w][0] =g2[w][0]*ydim ; V[w][1] = g2[w][1]*xdim ; }
    else { V[w][0] = g2[w][0]*xdim ; }
    if (debugSwitch) {printf("Instantaneous Linear Velocity in X,Y,Z after corss V[w][j]=V[%d][j]= [%f \t%f \t%f]\n",w,V[w][0],V[w][1],V[w][2]);}// Filters
    if (debugSwitch) {printf("===End Cross Product of Instanenous gyro g2 to get Velocity in XYX\n");}


    //Calculate Instantaneous Velocity Magnitude from the x-y-z velocity components
    if (debugSwitch) {printf("===Start Calculate Instantaneous Velocity Magnitude\n");}
    vL2=0;
    for (j=0 ; j<d ; j++){
        vL2=vL2+pow(V[w][j],2);
    }

    //Linear Velocity Magnitude from the magnitudes Linear Velocity in x,y,z
    VLinear[w][0]=sqrt(vL2); 
    if (debugSwitch) {printf("Instantaneous Linear Velocity Magnitude after Squaring = VLinear[w][0]=VLinear[%d][0]= %f\n",w,VLinear[w][0]);}// Filters
    if (debugSwitch) {printf("===Buffer Instantaneous Linear Velocity Magnitude Array VLinear[w][0]\n");}
    if (debugSwitch) {printArr2d(&VLinear[0][0], M,1);}
    if (debugSwitch) {printf("===End Calculate Instantaneous Velocity Magnitude\n");}

    
    // Instantaneous Distance gL is generated from the integration of the Linear Velocity using a,
    // trapezoidal integration filter with order DL (DL delayed samples + 1 current sample)
    if (debugSwitch) {printf("===Start Calculate Instantaneous Distance Magnitude based on Integration of Linear Velocity Magnitude\n");}
    //v is used same way as in previous settings to align the coefficients in the correct order with Filter Coefficients
    v=DL;
    for (k=r+(D-DL) ; k < r+(D-DL)+FL ; k++ ){
        aL[v][0]=VLinear[k%M][0];
        if (debugSwitch) {printf("w=%d \tr=%d \tk=%d \t (k mod M)=%d \t deltaDelay=%d  \t v=%d \t D=%d, Di=%d, aL[%d][0]=%f \t vLinear[%d][0]=%f \t ads=%p\n",         w, r,k,k%M, (D-DL) ,v ,D,DL,v ,aL[v][0],    k%M , VLinear[k%M][0] , &aL[v][0]);}
        v=v-1;
    }

    //gL is the results of integrating the velocity magnitude, which is the instantaneous Distance
    gL[w][0]= dotProduct2(&aL[0][0], &HL[0][0],FL,1); //Integration of Velocity Magnitude to give instantenous distance covered
    if (debugSwitch) {printf("Instantaneous Linear Distance Magnitude gL[w][0]=gL[%d][0]= %f\n",w,gL[w][0]);}
    if (debugSwitch) {printf("===Instantaneous Linear Distance Magnitude Array VLinear[M][0]\n");}
    if (debugSwitch) {printArr2d(&gL[0][0], M,1);} 
    if (debugSwitch) {printf("===End Calculate Instantaneous Linear Distance Magnitude based on Integration of Linear Velocity Magnitude Array\n");}
    
    if (debugSwitch) {printf(">Instant Distance:%f\n", gL[w][0]);}
    if (debugSwitch) {printf("Accumulated Distance Before Adding current Disantce= L = %f\n", L );}

    //The total distance covered is the summation of the instantenous distances
    L=L+gL[w][0];
    if (debugSwitch) {printf("Accumulated Distance After Adding current Disantce= L = %f\n", L );}
    if (debugSwitch) {printf(">Total Distance:%f\n",L);}



    


    //Increment Read and Write Pointers
    w=(w+1)%M;
    r=(r+1)%M;

    //Avoid overflow for the number of samples counter, so we set 2048 as the max, so total distance will reset every 1024*8*0.5/M seconds ()
    t=(t+1)%1024*8;
    if (t==0) {L=0; LSlow=0;} //Clear the Distance covered registers every 1024*8*Tsample = 1024*8 * 0.5/M

    //To satisfy the challenge requirements, on every 0.5 seconds , the circular buffers will rotate to the first element, at this time
    //we will take the current filtered signals samples and keep them in a size 40-slower buffer
    //we will i-pointer to rotate around the 40-samples slower buffers
    if (w==0){
        for (j=0 ; j<d ; j++){
            gyroReadSlow[i][j]=gyroRead[w][j];
            gyroSlow[i][j]=gyro[w][j];
            g2Slow[i][j]=g2[w][j];
            VSlow[i][j]=V[w][j];
        }
            VLinearSlow[i][0]=VLinear[w][0];
            gLSlow[i][0]=gL[w][0];
            LSlow=LSlow+gLSlow[i][0];
            if (debugSwitch){
                printf("--Buffer Slow Gyro Readings float--\n");
                printArr2dInt(&gyroReadSlow[0][0],40,d);
            }
        i=(i+1)%40;
    }


    //Tune : Sample the gyro readings every Tdelay, ideally Tdelay = 0.5/M so that every M-Samples will be processed to yield a 0.5 sample as per the challenge requirements.
    thread_sleep_for(Tdelay * 1000);
    if((t%M) == 0){
        lcd_clear();
    }
    if (debugSwitch) {printf("===End Gyro Reading====\n");}
  }
    if (debugSwitch3){
        printf("--Pointers--\n");
        printf("w=%d\t r=%d\t i=%d\t t=%d\n",w,r,i,t);

        printf("--Buffer Gyro Readings float--\n");
        printArr2dInt(&gyroRead[0][0],M,d);


        printf("--Buffer Gyro Readings Converted and down scaled--\n");
        printArr2d(&gyro[0][0],M,d);


        printf("--Delayed Gyro Input to Filter--\n");
        printArr2d(&a2[0][0],F2,d);
        printf("--Trapezoidal Integrator Filter with Unit Window--\n");
        printArr2d(&H2[0][0],F2,d);
        printf("--Buffer Filtered Gyro Output Buffer--\n");
        printArr2d(&g2[0][0],M,d);
        
        
        printf("--Buffer 3D Linear Velocity instant in x,y,z--\n");
        printArr2d(&V[0][0],M,d);
        printf("--Buffer 1D Linear Velocity instant from corss product g2--\n");
        printArr2d(&VLinear[0][0],M,1);

        printf("--Delayed 1D Linear Velocity instant--\n");
        printArr2d(&aL[0][0],FL,1);
        printf("--Integrator Filter for 1D Linear Velocity--\n");
        printArr2d(&HL[0][0],FL,1);

        printf("--Buffer Distance from integrated 1D Linear Velocity--\n");
        printArr2d(&gL[0][0],M,1);
    }

    if (debugSwitch2){
        printf(">gyroRead_x:%d\n",gyroRead[w][0]);
        printf(">gyroRead_y:%d\n",gyroRead[w][1]);
        printf(">gyroRead_z:%d\n",gyroRead[w][2]);
        printf(">gyro_x:%f\n",gyro[w][0]);
        printf(">gyro_y:%f\n",gyro[w][1]);
        printf(">gyro_z:%f\n",gyro[w][2]);
        printf(">g2_x:%f\n",g2[w][0]);
        printf(">g2_y:%f\n",g2[w][1]);
        printf(">g2_z:%f\n",g2[w][2]);
        printf(">g3_x:%f\n",g3[w][0]);
        printf(">g3_y:%f\n",g3[w][1]);
        printf(">g3_z:%f\n",g3[w][2]);
        printf(">V_x:%f\n",V[w][0]);
        printf(">V_y:%f\n",V[w][1]);
        printf(">V_z:%f\n",V[w][2]);
        printf(">VLinear:%f\n",VLinear[w][0]);
        printf(">L_instant:%f\n",gL[w][0]);
        printf(">L_total:%f\n",L);
    }

    if (debugSwitch4){
        printf(">gyroReadSlow_x:%d\n",gyroReadSlow[i][0]);
        printf(">gyroReadSlow_y:%d\n",gyroReadSlow[i][1]);
        printf(">gyroReadSlow_z:%d\n",gyroReadSlow[i][2]);
        printf(">gyroSlow_x:%d\n",gyroReadSlow[i][0]);
        printf(">gyroSlow_y:%d\n",gyroReadSlow[i][1]);
        printf(">gyroSlow_z:%d\n",gyroReadSlow[i][2]);
        printf(">g2x:%f\n",g2[i][0]);
        printf(">g2y:%f\n",g2[i][1]);
        printf(">g2z:%f\n",g2[i][2]);
        printf(">g3x:%f\n",g3[i][0]);
        printf(">g3y:%f\n",g3[i][1]);
        printf(">g3z:%f\n",g3[i][2]);
        printf(">VSlow_x:%f\n",VSlow[i][0]);
        printf(">VSlow_y:%f\n",VSlow[i][1]);
        printf(">VSlow_z:%f\n",VSlow[i][2]);
        printf(">VLinearSlow:%f\n",VLinearSlow[i][0]);
        printf(">L_instantSlow:%f\n",gLSlow[i][0]);
        printf(">L_total:%f\n",LSlow);
    }
    


}