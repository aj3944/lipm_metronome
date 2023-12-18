#include "mbed.h"
#include "../lib/LCD_DISCO/LCD_DISCO_F429ZI.h"

#define PI  3.14159265358979323846 
#define EPSILON 0.000001

#define xdim 4
#define ydim 4
#define zdim 1

#define d 3 //number of dim
#define M 32 //Buffer Length, number of samples in buffer per x-axis, M practically  M<N 
/*Use in Testing Only
#define N 7  //number of readings provided by gyro per x-axis = inf/d
#define inf ((N)*1) //number of samples taken from gyro, should be infinity
*/
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
            if (i==0 | i==f-1){
                arr[i][j]=(float) 1/2;
            }
            else {arr[i][j]=(float) 1;}
        }
    }
}

void integratorTrapezoidal1D( uint16_t f, uint16_t dim, float (*arr)[1]){//dim
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


void blackmanWindow( uint16_t f, uint16_t dim, float (*arr)[3]){//dim
    uint8_t i=0,j=0;
        for (j=0; j<3 ; j++){
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

void differentiator( uint16_t f, uint16_t dim, float (*arr)[3] ){//dim
    //printf("f=%d \t dim=%d \n", f, dim);
    uint16_t i=0,j=0, n=f-1;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                //printf("i=%d \n",i);
                arr[i][j]=pow(-1,i) * combination(&n,&i);
        }
    }
}

void hadamardProduct(uint16_t f, uint16_t dim, float (*arr1)[3] , float (*arr2)[3] , float (*arr3)[3] ){//dim
    int i=0, j=0 ;
        for (j=0; j<dim ; j++){
            for (i=0 ; i<f ; i++){
                //printf("i=%d \n",i);
                arr3[i][j]= arr1[i][j] * arr2[i][j] ;
        }
    }
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

    float gyro[d][M] = {0}; //initialie gyro buffer
    //float gyro[M][d] = {0}; //initialie gyro buffer
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

    int8_t w=0, r=(M+w-D)%M , v=0;
    //int8_t r1=(M-D1)%M;

    printf("Start \n");
    
    filterRectangle(F1,d,H1);
    printArr2d(&H1[0][0],F1,d);// Filters

    
    
    integratorTrapezoidal(F2,d,H2);
    printArr2d(&H2[0][0],F2,d);// Filters


    
    
    differentiator(F3,d,H3);
    printArr2d(&H3[0][0],F3,d);// Filters

    

    integratorTrapezoidal1D(FL,1,HL);
    printArr2d(&HL[0][0],FL,d);// Filters
    
    printf("--Blackman--\n");
    blackmanWindow(F1,d,W);
    printArr2d(&W[0][0],F1,d);// Filters

    printf("--Haddamard--\n");
    hadamardProduct(F1,d,W,H1,H1);
    printArr2d(&H1[0][0],F1,d);// Filters




    printf("Initial\n");
    //printf("---- M=%d \tF=%d \tD=%d \tdim=%d \tinf=%d \tsamplPerAxs=%d \n", M, F, D, d, inf , N );
    //printf("---- M=%d \tF=%d \tD=%d \tdim=%d \tinf=%d \tsamplPerAxs=%d \n", M, F, D, d, inf , N );
    printf("--w=%d \tr=%d \tv=%d \tt=%d\n", w,r,v,t );
    printf("Start Gyro\n");


    

    while(1)
    {
    //printf("M=%d \tF=%d \tD=%d \tdim=%d \tinf=%d \tsamplPerAxs=%d \n", M, F1, D1, d, inf , N );
    //printf("M=%d \tF=%d \tD=%d \tdim=%d \tinf=%d \tsamplPerAxs=%d \n", M, F1, D1, d, inf , N );
    //printf("w=%d \tr1=%d \tr2=%d \tr3=%d\n", w,r1,r2,r3 );




      //prepare the write buffer to trigger a sequential read
      write_buf[0]=OUT_X_L|0x80|0x40;
      //start sequential sample reading
      spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
      flags.wait_all(SPI_FLAG);
      
 
      for (j=0 ; j<d ; j++){
        //gyro[j][w]=i; //write op
        gyroRead[j][w] = ( ( (uint16_t)read_buf[2*j+1+1] ) <<8 ) | ( (uint16_t)read_buf[2*j+1] );
        gyro[j][w] = ((float) gyroRead[j][w])*(17.5f*0.017453292519943295769236907684886f / 1000.0f);


        printf(">input \n t=%d \ti=%d \tj=%d \tw=%d \t r=%d \t v=%d \tgyro[%d][%d]=%f\n", t,i,j,w,r,v,w,j,j,gyro[w][j] );

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
        
        /*
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

    thread_sleep_for(50);
    if((t%M) == 0){
        lcd_clear();
    }

}}
