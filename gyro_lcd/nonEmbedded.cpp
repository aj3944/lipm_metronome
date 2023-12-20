//// Non Embbedded Version ; this was the base testing for the Test1 Branch , You can run it on any C compiler

// Online C compiler to run C program online
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define xdim 4
#define ydim 4
#define zdim 1

#define d 1 //number of dim
#define M 3 //Buffer Length, number of samples in buffer per x-axis, M practically  M<N 
#define N 7  //number of readings provided by gyro per x-axis = inf/d
#define inf ((N)*1) //number of samples taken from gyro, should be infinity
#define D (M-1) //Supported Delay Lines
#define F (M) //Supported Filter Coefficients

#define F1 (M) //Filter coefficients, n Delays + 1 current, F <=M , Fmax=M
//#define O (F-1) //Filter order
#define D1 (F1-1) //Filter Order, Number of delay line samples, D=F-1 , Dmax = M-1, D<=M-1, this is for the Filter delay
#define F2 (M-1+1) //integrator coefficients , n delay units, similar to F, odd , 3 is min for integrator
#define D2 (F2-1) //integrator delay , n delay units, similar to D
#define Ts2 (0.5)
#define Delta2 (Ts2)/(F2) //For the non-normalized 

#define F3 (3) //First  Difference, x(n)-x(n-1)
#define D3 (F3-1) //
#define Delta3 (pow(Ts,F3)) //

#define FL (M) //Trapezoidal integrator
#define DL (FL-1) //

void printArri(float *a, int8_t K){
    printf("Start printArri\n============\n");
    uint8_t i=0;
    //printf("%d \t\n",S);
    for (i=0 ; i<K ; i++){
        printf("%d \t %f \n",i,*(a+i));
    }
    printf("End printArri\n============\n");
}

void printArr2d(float *a, int8_t k1, int8_t k2){ //k1 row, k2 col
    printf("Start---Arr2d\n");
    uint8_t i=0,j=0,k=0,K=k1*k2;
    for (i=0 ; i<k1 ; i++){
        for (j=0; j<k2 ; j++){
            printf("%4.5f \t",*(a+k));
            k=k+1;
        }
            printf("\n");
    }
    printf("End pintArr2d\n============\n");
}




float dotProduct2(float *an, float *bn, int8_t f, int8_t dim){
    bool debug=1;
    if (debug) printf("Start dotProduct============\n");
    int8_t i=0;
    float r=0;
    for (i=0 ; i<f; i++){
        r= r + *(an+dim*i) * *(bn+dim*i);
        if (debug) printf("dotI\t[%d]\t a=%4.5f\t b=%4.5f \ta=%p\t b=%p \n",i,*(an+dim*i),an+dim*i,*(bn+dim*i),an+dim*i,bn+dim*i);
        
    }
    if (debug) printf("r=%4.5f\n",r);
    if (debug) printf("End dotProduct============\n");
    return r;
    
}


void crossProduct3d(float *an, float *vn){
    //float ax=*(an);
    //float ay=*(an+1);
    //float az=*(an+2);
    //printf("A1=%f \t %f \t %f \n",ax,ay,az);

    printf("Start Crossproduct====\n");
    *(vn) =   (*(an+1)) * zdim - (*(an+2)) * ydim;
    *(vn+1) = (*(an+2)) * xdim - (*(an))   * zdim;
    *(vn+2) = (*(an))   * ydim - (*(an+1)) * xdim;

    printf("A1 Ads = %p \t %p \t %p \n", an , an+1 , an+2);
    printf("A1 = [%f \t %f \t %f] \n", *(an) , *(an+1) , *(an+2));
    printf("Cross Product V = [%f \t %f \t %f] \n",*(vn),*(vn+1),*(vn+2));
    printf("End Crossproduct====\n");
}



void filterRectangle( uint16_t f, uint16_t dim, float (*arr)[dim]){
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
            arr[i][j]=(float) 1/f;
        }
    }
}


void integratorTrapezoidal( uint16_t f, uint16_t dim, float (*arr)[dim]){
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
            if (i==0 | i==f-1){
                arr[i][j]=(float) 1/2;
            }
            else {arr[i][j]=(float) 1;}
        }
    }
}

void blackmanWindow( uint16_t f, uint16_t dim, float (*arr)[dim]){
    uint8_t i=0,j=0;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                arr[i][j]=(float) 0.42 - 0.5*cos(2*3.14*i/(f-1)) + 0.08*cos(4*3.14*i/(f-1));
                printf("bi=%f \n",0.42 - 0.5*cos(2*3.14*i/(f-1)) + 0.08*cos(4*3.14*i/(f-1)));
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
    //printf("--a=%d \ta!=%d \n",  *a,factorial(a) );
    //printf("--b=%d \tb!=%d \n",  *b,factorial(b) );
    //printf("--a-b=%d \t(a-b)!=%d \n", diff, factorial(&diff) );
    //comb = factorial(a)/(factorial(b) * factorial(a-b) ) ;///(factorial (b) * factorial (a-b));
    comb = factorial(a)/(factorial(b) *factorial(&diff) ) ;///(factorial (b) * factorial (a-b));
    //printf ("combination= %f \n", comb);
    return comb;
}

void differentiator( uint16_t f, uint16_t dim, float (*arr)[dim] ){
    //printf("f=%d \t dim=%d \n", f, dim);
    int i=0,j=0, n=f-1;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                //printf("i=%d \n",i);
                arr[i][j]=pow(-1,i) * combination(&n,&i);
        }
    }
}

void hadamardProduct(uint16_t f, uint16_t dim, float (*arr1)[dim] , float (*arr2)[dim] , float (*arr3)[dim] ){
    int i=0, j=0 ;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                //printf("i=%d \n",i);
                arr3[i][j]= arr1[i][j] * arr2[i][j] ;
        }
    }
}

bool debug=1;
int main() {
    // Write C code here
    float gyro[M][d] = {0}; //initialie gyro buffer
    int16_t gyroRead[M][d] = {0}; //initialie gyro direct reading
    float H1[F1][d]; //initialize Filter Coefficients1
    float a1[F1][d] = {0}; //initialie filter input samples, multiplied by the filter coefficients
    float g1[M][d]={0}; //initialize filtered gyro buffer, output of dot product after being processed by filters.

    float H2[F2][d]; //initialize Filter Coefficients1
    float a2[F2][d] = {0}; //initialie filter input samples, multiplied by the filter coefficients
    //float I2[F2][d] = {0}; //initialie filter input samples, multiplied by the filter coefficients
    float g2[M][d]={0}; //initialize filtered gyro buffer, output of dot product after being processed by filters.

    float H3[F3][d]; //initialize Filter Coefficients1
    float a3[F3][d] = {0}; //initialie filter input samples, multiplied by the filter coefficients
    float g3[M][d]={0}; //initialize filtered gyro buffer, output of dot product after being processed by filters.


    float V[M][d]={0}; //Linear Velocity x,y,z components calculated from corss angular with chip dim
    float VLinear[M][1]={0}; //Linear Velocity Magnitude
    float HL[FL][1]={0}; //initialize Filter Coefficients for
    float aL[M][1]={0}; //List of Linear Velocity Magnitudes current and delayed
    float gL[M][1]={0}; //Distance instantaneous
    float L=0, vL2 ; //Total Distance , and Linear Velocity Squared accumulator

    float W[F1][d]; //initialize Filter Coefficients1


    int8_t i=0,j=0,k=0, t=0; //i for gyro readings counter, j for dim counter, k for filter coefficients, t for sample number
    bool debug=0;
    
    

    /*
    printf("&a1[0][0] %p \n",&a1[0][0]);
    printf("&a1[0][1]) %p \n",&a1[0][1]);
    printf("&a1[1][0] %p \n",&a1[1][0]);
    printf("&a1[1][1] %p \n",&a1[1][1]);
    printf("&a1[2][0] %p \n",&a1[2][0]);
    printf("&a1[2][1] %p \n",&a1[2][1]);
    printf(" \n");

    printf("%p \n",&a1[0][0]);
    printf("%p \n",&(a1[0][0])+1);
    printf("%p \n",&(a1[0][0])+2);
    printf("%p \n",&(a1[0][0])+3);
    printf("%p \n",&(a1[0][0])+4);
    printf("%p \n",&(a1[0][0])+5);
    printf("%p \n",&(a1[0][0])+6);
    */

 
    int8_t w=0, r=(M+w-D)%M , v=0;
    //int8_t r1=(M-D1)%M;

    printf("Start \n");
    
    filterRectangle(F1,d,H1);
    printArr2d(&H1[0][0],F1,d);// Filters

    
    
    integratorTrapezoidal(F2,d,H2);
    printArr2d(&H2[0][0],F2,d);// Filters


    
    
    differentiator(F3,d,H3);
    printArr2d(&H3[0][0],F3,d);// Filters

    

    integratorTrapezoidal(FL,1,HL);
    printArr2d(&HL[0][0],FL,d);// Filters
    
    printf("--Blackman--\n");
    blackmanWindow(F1,d,W);
    printArr2d(&W[0][0],F1,d);// Filters

    printf("--Haddamard--\n");
    hadamardProduct(F1,d,W,H1,H1);
    printArr2d(&H1[0][0],F1,d);// Filters

    /*
    void  (*Ptr_f1)(void);
    void  (*ptr_crossProduct3d)(float*, float*);
    float (*ptr_dotProduct2)(float*, float*, int8_t, int8_t);
    ptr_crossProduct3d=&crossProduct3d;
    ptr_dotProduct2=&dotProduct2;
    
    if (d==3) { Ptr_f1 = ptr_crossProduct3d;}
    */

    printf("Initial\n");
    printf("---- M=%d \tF=%d \tD=%d \tdim=%d \tinf=%d \tsamplPerAxs=%d \n", M, F, D, d, inf , N );
    printf("--w=%d \tr=%d \tv=%d \tt=%d\n", w,r,v,t );
    printf("Start Gyro\n");

    for (i=0 ; i<inf ; i++){
        printf("==Start Read Gyro XYZ \n");
        for (j=0 ; j<d ; j++){
            printf("==Start Read Gyro dim \n");
            gyro[w][j]=(i+0); //write op
            printf(">input \n t=%d \ti=%d \tj=%d \tw=%d \t r=%d \t v=%d \tgyro[%d][%d]=%f\n", t,i,j,w,r,v,w,j,j,gyro[w][j] );

        /*
        //Start Signal Prep block
        v=D1;
        printf("===Start Prep\n");
        for (k=r+(D-D1) ; k < r+(D-D1)+F1 ; k++ ){
            a1[v][j]=gyro[k%M][j];
            printf("w=%d \tr=%d \tk=%d \tk%M=%d \t deltaDelay=%d  \t v=%d \t D=%d, Di=%d, a1[%d][%d]=%4.2f \t gyro[%d][%d]=%4.2f \t ads=%p\n", w, r,k,k%M, (D-D1) ,v ,D,D1,v,j ,a1[v][j],    k%M , j, gyro[k%M][j] , &a1[k][j]);
            v=v-1;
        }
        printf("===End Prep\n");
        printf("===Start Filtered\n");
        g1[w][j]= dotProduct2(&a1[0][j], &H1[0][j],F1,d); //*Delta2 
        //printf(">i=%d \t w=%d \t r1=%d \t r1+(D1-D1)=%d \t g2[%d][%d]=%f \n",i, w,r1,r1+(D1-D2), w,j,g3[w][j]);
        printf("w=%d \tr=%d \t(Delta Delay)=%d \t ri=r+deltaDelay=%d \tg1[%d][%d]=%f ads=%p\n", w, r, (D-D1) , r+(D-D1),w ,j ,g1[w][j], &g1[w][j]    );
        printf("===End Filtered\n");
        //End Signal Prop Block
        
        
        //Start Signal Prep block
        v=D2;
        printf("===Start Prep\n");
        for (k=r+(D-D2) ; k < r+(D-D2)+F2 ; k++ ){
            a2[v][j]=gyro[k%M][j];
            printf("w=%d \tr=%d \tk=%d \tk%M=%d \t deltaDelay=%d  \t v=%d \t D=%d, Di=%d, a1[%d][%d]=%4.2f \t gyro[%d][%d]=%4.2f \t ads=%p\n", w, r,k,k%M, (D-D2) ,v ,D,D2,v,j ,a2[v][j],    k%M , j, gyro[k%M][j] , &a2[k][j]);
            v=v-1;
        }
        printf("===End Prep\n");
        printf("===Start Filtered\n");
        g2[w][j]= dotProduct2(&a2[0][j], &H2[0][j],F2,d); //*Delta2 
        //printf(">i=%d \t w=%d \t r1=%d \t r1+(D1-D1)=%d \t g2[%d][%d]=%f \n",i, w,r1,r1+(D1-D2), w,j,g3[w][j]);
        printf("w=%d \tr=%d \t(Delta Delay)=%d \t ri=r+deltaDelay=%d \tg1[%d][%d]=%f ads=%p\n", w, r, (D-D2) , r+(D-D2),w ,j ,g2[w][j], &g2[w][j]    );
        printf("===End Filtered\n");
        //End Signal Prop Block
        
        
        //Start Signal Prep block
        v=D3;
        printf("===Start Prep\n");
        for (k=r+(D-D3) ; k < r+(D-D3)+F3 ; k++ ){
            a3[v][j]=gyro[k%M][j];
            printf("w=%d \tr=%d \tk=%d \tk%M=%d \t deltaDelay=%d  \t v=%d \t D=%d, Di=%d, a3[%d][%d]=%4.2f \t gyro[%d][%d]=%4.2f \t ads=%p\n", w, r,k,k%M, (D-D3) ,v ,D,D3,v,j ,a3[v][j],    k%M , j, gyro[k%M][j] , &a3[k][j]);
            v=v-1;
        }
        printf("===End Prep\n");
        printf("===Start Filtered\n");
        g3[w][j]= dotProduct2(&a3[0][j], &H3[0][j],F3,d); //*Delta2 
        printf("w=%d \tr=%d \t(Delta Delay)=%d \t ri=r+deltaDelay=%d \tg3[%d][%d]=%f ads=%p\n", w, r, (D-D3) , r+(D-D3),w ,j ,g3[w][j], &g3[w][j]    );
        printf("===End Filtered\n");
        //End Signal Prop Block
        */

    printf("==End Read Gyro dim \n");
    }
    
    //Section to calculate values based on the 3 existing 
    if (d==3) {
        crossProduct3d(&gyro[w][0] , &V[w][0]);
        /*
        V[w][0] = gyro[w][1] * zdim - gyro[w][2] * ydim ;
        V[w][1] = gyro[w][2] * xdim - gyro[w][0] * zdim ;
        V[w][2] = gyro[w][0] * ydim - gyro[w][1] * xdim ;
        */
        
    }
    else if (d==2) {V[w][0] =gyro[w][0]*ydim ; V[w][1] = gyro[w][1]*xdim ; }
    else { V[w][0] = gyro[w][0]*xdim ; }

    //Calculate Veloicity
    //float vL2=0;
    for (j=0 ; j<d ; j++){
    //VLinear[w]=sqrt(pow(V[w][0],2) + pow(V[w][1],2) + pow(V[w][2],2));
        vL2=vL2+pow(V[w][j],2);
    }
    VLinear[w][0]=sqrt(vL2);
    printf("VLinear[%d][0] = %f \n", w, VLinear[w][0]);
    printArr2d(&VLinear[0][0], M,d); //filtered
    //Integrator based on trapezoidal
    v=DL;
    printf("===Start Final Processing\n");
    for (k=r+(D-DL) ; k < r+(D-DL)+FL ; k++ ){
        aL[v][0]=VLinear[k%M][0];
        printf("k=%d \n",k);
        printf("w=%d \tr=%d \tk=%d \tk%M=%d \t deltaDelay=%d  \t v=%d \t D=%d, Di=%d, aL[%d][0]=%4.2f \t vLinear[%d][0]=%4.2f \t ads=%p\n", w, r,k,k%M, (D-DL) ,v ,D,DL,v ,aL[v][0],    k%M , VLinear[k%M][0] , &aL[v][0]);
        v=v-1;
    }
    gL[w][0]= dotProduct2(&aL[0][0], &HL[0][0],F2,1); //*Delta2 
    printf("Distance w=%d \tr=%d \t(Delta Delay)=%d \t ri=r+deltaDelay=%d \tgL[%d]=%f ads=%p\n", w, r, (D-DL) , r+(D-DL),w  ,gL[w][0], &gL[w][0]    );
    L=L+gL[w][0];
    printf("Total Distance=%f \n",L);


    
    
    
    
    //Increment Pointers
    w=(w+1)%M;
    r=(r+1)%M;
    t=t+1;
    printf("==End Read Gyro XYZ \n");

}
    printf("End Reading Gyro\n");
    printf("--w=%d \tr=%d \tv=%d \tt=%d\n", w-1,r-1,v-1,t-1 );

    printArr2d(&H1[0][0], F1,d);// 
    printArr2d(&H2[0][0], F2,d);// 
    printArr2d(&H3[0][0], F3,d);// 
    printArr2d(&gyro[0][0], M,d);// Readings
    //printArri(&gyro[0][0], M*d);// Readings
    ////printArr2d(&gyro[0][0], M,d);
    printArr2d(&g1[0][0], F1,d); //filtered
    printArr2d(&g2[0][0], F2,d); //filtered
    printArr2d(&g3[0][0], F3,d); //filtered
    printArr2d(&V[0][0], M,d); //filtered
    printArr2d(&VLinear[0], 1,d); //filtered
    printArr2d(&gL[0], 1,d); //filtered
    //printArrd(&g2[0][0], d*F2); //filtered
    //printArri(&I[0][0], d*F); //Last input to t product
    //printf("I[0][0]=%f \n",I[0][0] );
    //printf("I[0][0]=%f \n",I[1][0] );
    //printf("I[0][0]=%d \n",I[0][0] );
    //printf("I[0][0]=%d \n",I[1][0] );
    //printf("g1[0][0]=%f \n",g1[0][0] );
    //printf("g1[1][0]=%f \n",g1[0][1] );
    //printf("g1[0][1]=%f \n",g1[1][0] );
    //printf("g1[1][1]=%f \n",g1[1][1] );
    //printf("g1[0][2]=%f \n",g1[2][0] );
    //printf("g1[1][2]=%f \n",g1[2][1] );
    
    //printf(">>g1[0][0]=%p \n", &g1[0][0] );
    //printf(">>g1[1][0]=%p \n", &g1[0][1] );
    //printf(">>g1[0][1]=%p \n", &g1[1][0] );
    //printf(">>g1[0][1]=%p \n", &g1[1][1] );

    
    //printf(">g1[0][0]=%f \n",*(&g1[0][0]) );
    //printf(">g1[0][0]=%f \n",*(&g1[0][0]+1) );
    //printf(">g1[0][0]=%f \n",*(&g1[0][0]+2) );
    //printf(">g1[0][0]=%f \n",*(&g1[0][0]+3) );


}
