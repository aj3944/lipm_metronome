#include "mbed.h"

#define PI  3.14159265358979323846 
#define EPSILON 0.000001


//Normalized coordinates for the radius in ; Linear Velocity = Angular Velocity X Radius, Radius values can be calibrated per indvidual length
#define xdim 1
#define ydim 0.1
#define zdim 0.01


#define d 3 //number of dim of gyroscope readings
#define M 16 //Circular Buffer Length, number of samples in buffer per axis,  

//Tune : Use Only in Debugging when manual values are inserted
#define N 7  //number of readings provided by gyro per x-axis = inf/d
#define inf ((N)*1) //number of samples taken from gyro, should be infinity
//

#define D (M-1) //Supported Delay Lines
#define F (M) //Supported Filter Coefficients

//Tune : Tsample
#define Tdelay (0.5/M) //For Testing
#define Ts2 Tdelay //For Testing

#define F1 (M) //Filter coefficients, n Delays + 1 current, F <=M , Fmax=M
#define D1 (F1-1) //Filter Order, Number of delay line samples, D=F-1 , Dmax = M-1, D<=M-1, this is for the Filter delay

#define F2 (M) //integrator coefficients , n delay units, similar to F, odd , 3 is min for integrator
#define D2 (F2-1) //integrator delay , n delay units, similar to D

//Tune : Use Differentiator or not
#define F3 (3) //First  Difference, x(n)-x(n-1)
#define D3 (F3-1) //

#define FL (M) //Trapezoidal integrator
#define DL (FL-1) //


static const bool debugSwitch=0;
static const bool debugSwitch2=0;
static const bool debugSwitch3=0;



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
    // The x,y,z coordinates of an ;
    // ax=*(an);
    // ay=*(an+1);
    // az=*(an+2);

    if (debugSwitch) {printf("Start Crossproduct====\n");}
    *(vn) =   (*(an+1)) * zdim - (*(an+2)) * ydim;
    *(vn+1) = (*(an+2)) * xdim - (*(an))   * zdim;
    *(vn+2) = (*(an))   * ydim - (*(an+1)) * xdim;

    // if (debugSwitch) {("Cross Product Input an Address : %p \t %p \t %p \n", an , (an+1) , (an+2));}
    if (debugSwitch) {printf("Cross Product Input an : [%f \t %f \t %f] \n", *(an) , *(an+1) , *(an+2));}
    if (debugSwitch) {printf("Cross Product Result vn : [%f \t %f \t %f] \n",*(vn),*(vn+1),*(vn+2));}
    if (debugSwitch) {printf("End Crossproduct====\n");}
    
}



void filterRectangle( uint16_t f, uint16_t dim, float (*arr)[3]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
            arr[i][j]=(float) 1/f * Ts2;
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
            else {arr[i][j]=(float) 1 * Ts2;}
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
            else {arr[i][j]=(float) 1 * Ts2;}
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
long long int FACT_ARRAY[M];

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

float combination(uint16_t *a, uint16_t *b){
    if(!COMB_ARRAY[*a][*b] == 0){
        float comb=1;
        uint16_t diff = *a - *b ;
        if (debugSwitch) {printf("--a=%d \ta!=%lld \n",  *a,factorial(a) );}
        if (debugSwitch) {printf("--b=%d \tb!=%lld \n",  *b,factorial(b) );}
        if (debugSwitch) {printf("--a-b=%d \t(a-b)!=%lld \n", diff, factorial(&diff) );}
        comb = factorial(a)/(factorial(b) *factorial(&diff) ) ;///(factorial (b) * factorial (a-b));
        if (debugSwitch) {printf ("combination= %f \n", comb);}
        COMB_ARRAY[*a][*b] = comb;
        return comb;
    }else{
        return COMB_ARRAY[*a][*b];
    }
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

float gyro[M][d] = {0}; //initialie gyro buffer
int16_t gyroRead[M][d] = {0}; //initialie gyro direct reading

/* Tune : Use a full length integrator, i.e. Delay units = Buffer Size M - 1
float H1[F1][d]; //initialize Filter Coefficients1
float a1[F1][d] = {0}; //initialie filter input samples, multiplied by the filter coefficients
float g1[M][d]={0}; //initialize filtered gyro buffer, output of dot product after being processed by filters.
float W1[F1][d]; //initialize Blackman Window Coefficients
*/

float H2[F2][d]; //initialize Non-Windowed Filter Coefficients for integrator
float a2[F2][d] = {0}; //initialize a buffer to keep the last (D2) samples + the current samples (x[n],x[n-1],..,x[n-(D1-1)]) to be used in integrator 
float g2[M][d]={0}; //initialize filtered gyro buffer, output of dot product after being processed by filters.
float W2[F2][d]; //initialize Filter Coefficients1


float H3[F3][d]; //initialize Filter Coefficients1
// float a3[F3][d] = {0}; //initialie filter input samples, multiplied by the filter coefficients
float g3[M][d]={0}; //initialize filtered gyro buffer, output of dot product after being processed by filters.
float W3[F3][d]; //initialize Filter Coefficients1


float V[M][d]={0}; //Linear Velocity x,y,z components calculated from corss angular with chip dim
float VLinear[M][1]={0}; //Linear Velocity Magnitude
float HL[FL][1]={0}; //initialize Filter Coefficients for processing Linear Velocity Delayed Samples
float aL[M][1]={0}; //List of Linear Velocity Magnitudes current and delayed to be used in processing
float WL[FL][1]; //initialize Blackman Coefficients for Windowing the Filters Coefficients

float gL[M][1]={0}; //Distance instantaneous resulted from integrating or corss product of VLinear
float L=0, vL2 ; //Total Distance , and Linear Velocity Squared accumulator 

int16_t gyroReadSlow[40][d]={0} ;  
float gyroSlow[40][d]={0} ;  
float g2Slow[40][d]={0} ;  
float VSlow[40][d]={0} ;  
float VLinearSlow[40][1]={0} ;  
float gLSlow[40][1]={0} ;  
float LSlow=0 ;  

int Lint = 0;

int8_t i=0,j=0,k=0, t=0; //i for gyro readings counter, j for dim counter, k for filter coefficients, t for sample number


int8_t w=0, r=(M+w-D)%M , v=0;


void init_filters(){


    printf("Start Init \n");
    
    /*
    filterRectangle(F1,d,H1);
    printf("--Moving Average Filter--\n");
    printArr2d(&H1[0][0],F1,d);// Filters
    */
    
    
    integratorTrapezoidal(F2,d,H2);
    printf("--Trapezoidal Integrator Filter--\n");
    printArr2d(&H2[0][0],F2,d);// Filters

    
    
    differentiator(F3,d,H3);
    printf("--Differentiator Filter--\n");
    printArr2d(&H3[0][0],F3,d);// Filters
    
    
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
    //unitWindow(F2,d,W2);
    printf("--Blackman Window Coefficients for Trapezoidal Integrator--\n");
    printArr2d(&W2[0][0],F2,d);// Filters
    hadamardProduct(F2,d,W2,H2,H2);
    printf("--Trapezoidal Integrator Filter with Blackman Window--\n");
    printArr2d(&H2[0][0],F2,d);// Filters

    
    blackmanWindow(F3,d,W3);
    //unitWindow(F2,d,W2);
    printf("--Blackman Window Coefficients for Differentiator--\n");
    printArr2d(&W3[0][0],F3,d);// Filters
    hadamardProduct(F3,d,W3,H3,H3);
    printArr2d(&H3[0][0],F3,d);// Filters
    

    blackmanWindow1D(FL,1,WL);
    //unitWindow1D(FL,1,WL);
    printf("--Blackman Window Coefficients for Differentiator--\n");
    printArr2d(&WL[0][0],FL,1);// Filters
    hadamardProduct1D(FL,1,WL,HL,HL);
    printArr2d(&HL[0][0],FL,1);// Filters


    printf("Initial\n");
    //printf("---- M=%d \tF=%d \tD=%d \tdim=%d \tinf=%d \tsamplPerAxs=%d \n", M, F, D, d, inf , N );
    printf("--w=%d \tr=%d \tv=%d \tt=%d\n", w,r,v,t );
    printf("Start Gyro\n");    
}


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
        