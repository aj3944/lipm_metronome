#include "mbed.h"


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

//Human step size has a max frequency of 1.5 Hz (1.5 step per second), the challenge requirement was to use 2 Hz sampling (1 sample per 0.5 seconds)
//So we choose to process M-samples per 0.5 second, and aggregate the processing results in a slower buffer of length 40 as per the challenge requirements
//The faster sampling rate is to provide smoother results with the DSP filters, so each processed M samples will construct a 1 2Hz sample in the 40-samples slower buffer.

//The faster sampling rate will have Tdelay = 0.5/M 
#define Tdelay ((int) 0.5/M) //in seconds, in milliseconds = (0.5/(M)*1000)
#define Ts2 1 //Normalized and compensated in the end by scaling, We tested other values like Ts2=Tdelay
//We will use the same delay for the integrator, we could different sampling rates for the integrator and the data collection but we choose to align both


//We Define the filters orders separately to allow different tuning for the filters design

//Filters Template :
//A Full line filter that is using all delay units can be defined in this templte ; 
//#define F1 (M) //Filter coefficients, n Delays + 1 current, F <=M , Fmax=M
//#define D1 (F1-1) //Filter Order, Number of delay line samples, D=F-1 , Dmax = M-1, D<=M-1, this is for the Filter delay

#define F2 (M) //Trapezoidal integrator coefficients array , (M-1) delay units + 1 current sample to give total M samples, should be odd to staisfiy Linear FIR requirements, so min integrator = 3
#define D2 (F2-1) //The number of supported delay units, which is the filter order.



#define FL (M) //Trapezoidal integrator coefficients array, used to integrate the velocity to get the instantaneous distance
#define DL (FL-1) //The number of supported delay units, which is the filter order.



static const bool debugSwitch=1;  //Print functiosn processing and outputs if enabled
static const bool debugSwitch2=0; //Print the fast buffered signals M-samples, sampled at 0.5/M seconds in teleplot-compatible format
static const bool debugSwitch3=1; //Print the Arrays on every reading
static const bool debugSwitch4=0; //Print the slow buffer(40-samples), sampled at 0.5 seconds in teleplot-compatible format


int16_t gyroRead[M][d] = {0}; //initialie gyro direct reading
float gyro[M][d] = {0}; //Buffer to hold gyro scaled readings

/* Template : Use a full length integrator, i.e. Delay units = Buffer Size M - 1
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


//For the x,y,z velocity, Linear velocity magnitude, instantaneous distance and total distance
float V[M][d]={0}; //Linear Velocity x,y,z components calculated from corss angular with chip dim
float VLinear[M][1]={0}; //Linear Velocity Magnitude, V = Squrt (Vx^2 + Vy^2 + Vz^2)
float HL[FL][1]={0}; //initialize Filter Coefficients for the trapezoidal integrator filter coefficients
float WL[FL][1]; //initialize Blackman Coefficients for Windowing the Filters Coefficients
float aL[FL][1]={0}; //initialize a buffer to keep the last FL delayed Linear Velocity Magnitude Samples to be used in the integration 
float gL[M][1]={0}; //Distance instantaneous resulted from integrating the delayed FL samples from VLinear (integration of aL)
float L=0, vL2=0 ; //Total Distance , and Linear Velocity Squared accumulator 

// Lint is defined hold the approximate scaled total distance, the scaling can be calibrated and it compensates for 
// the filters gains and the non fixed radius in the cross product equation  VLinear = angular fequency x radius
int Lint = 0; 

//s for gyro readings counter to fill the slow buffer , j for dimension counter x,y and z , k for filter coefficients counter, t for sample number
uint8_t i=0,j=0,k=0, t=0; 
uint16_t w=0, r=(M+w-D)%M , v=0;


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


void init_filters(){


    printf("Start Init \n");
    
    /* Template for Defining Filters with Window functions : Using Averaging
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

 
}


