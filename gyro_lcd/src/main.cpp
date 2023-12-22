#include "mbed.h"
#include "../lib/LCD_DISCO/LCD_DISCO_F429ZI.h"
#include "../lib/LCD_DISCO/TS_DISCO_F429ZI.h"
#include "../include/filtering.h"
#include "../include/ui_ux.h"
#include "../include/disconfig.h"

#define SAMPLING_FREQ 10 //Hz
#define SAMPLING_RATE (1000/SAMPLING_FREQ) //ms


//Tune : Gyro Thresholds in g
#define xHThreshold 500
#define xLThreshold -500



/// TOUCH AND DRAW THEADS
// TOUCH AT 5Hz
// DRAW AT 24Hz
Thread touch_thread;

Thread gyro_thread;


bool do_sample = true;

int yy = 0;
// void update_distance(){

// }
void touch_actions(){
    while(1){
        do_touch();
        DrawButtons();
        ThisThread::sleep_for(300);
    }
}

void gyro_actions(){
    time_t seconds = time(NULL);   
    unsigned int mills_o = (unsigned int)seconds*1000;
    seconds = time(NULL);   
    int x_int = 0,y_int = 0,z_int = 0;

    while(1)
    {
        unsigned int mills_now = (unsigned int)seconds*1000;
        // if(mills_now - mills_o > SAMPLING_RATE){
        //     do_sample = true;
        //     mills_o =  (unsigned int)seconds*1000;
        // }
        flags.wait_all(DATA_READY_FLAG);
        //prepare the write buffer to trigger a sequential read
        write_buf[0]=OUT_X_L|0x80|0x40;
        //start sequential sample reading
        spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
        flags.wait_all(SPI_FLAG);


        for (j=0 ; j<d ; j++){//For each dimension , process each dimension reading sequentially
            gyroRead[w][j] = ( ( (uint16_t)read_buf[2*j+1+1] ) <<8 ) | ( (uint16_t)read_buf[2*j+1] );
            if (gyroRead[w][j] <= xHThreshold && gyroRead[w][j] >= xLThreshold) {gyroRead[w][j]=0;} //If within threshold then reset the readings to eliminate static noise
            gyro[w][j] = ((float) gyroRead[w][j])*(17.5f*0.017453292519943295769236907684886f / 1000.0f);
            v=D2;
            for (k=r+(D-D2) ; k < r+(D-D2)+F2 ; k++ ){
                a2[v][j]=gyro[k%M][j];
                v=v-1;
            }
            g2[w][j]= dotProduct2(&a2[0][j], &H2[0][j],F2,d); 
        }

        if (d==3) {
            V[w][0] = g2[w][1] * zdim - g2[w][2] * ydim ;
            V[w][1] = g2[w][2] * xdim - g2[w][0] * zdim ;
            V[w][2] = g2[w][0] * ydim - g2[w][1] * xdim ;        
        }
        else if (d==2) {V[w][0] =g2[w][0]*ydim ; V[w][1] = g2[w][1]*xdim ; }
        else { V[w][0] = g2[w][0]*xdim ; }
        //Calculate Instantaneous Velocity Magnitude
        vL2=0;
        for (j=0 ; j<d ; j++){
            vL2=vL2+pow(V[w][j],2);
        }
        VLinear[w][0]=sqrt(vL2); //Linear Velocity Magnitude for Linear Velocity in x,y,z

        v=DL;
        for (k=r+(D-DL) ; k < r+(D-DL)+FL ; k++ ){
            aL[v][0]=VLinear[k%M][0];
            v=v-1;
        }
        gL[w][0]= dotProduct2(&aL[0][0], &HL[0][0],FL,1); //Integration of Velocity Magnitude to give instantenous distance covered
        //The total distance covered is the summation of the instantenous distances
        L=L+gL[w][0];
        printf(">Total Distance:%f\n",L);
        if(do_sample){
                printf(">Interger Distance:%d\n",Lint);
                Lint = Lint +  (int)(14.3*L); //magic number for human gait
                // do_sample = false;
        }
        if(start_status){
            HomeDisplay(Lint,L);
            x_int = (int)(5000.0*g2[w][0]);
            y_int = (int)(5000.0*g2[w][1]);
            z_int = (int)(5000.0*g2[w][2]);
            AddPoint((int)x_int,(int)y_int,(int)z_int,yy);
        }


        //Increment Pointers
        w=(w+1)%M;
        r=(r+1)%M;
        t=(t+1)%1024;
        yy=(yy+1)%240;
        if(yy==0){
            lcd_clear();
        }
        if (w==0){
            for (j=0 ; j<d ; j++){
                gyroSlow[i][j]=gyro[w][j];
                gyroReadSlow[i][j]=gyroRead[w][j];
                g2Slow[i][j]=g2[w][j];
                VSlow[i][j]=V[w][j];
            }
                VLinearSlow[i][0]=VLinear[w][0];
                gLSlow[i][0]=gL[w][0];
                LSlow=LSlow+gLSlow[i][0];
            i=(i+1)%40;
            //Tune : Clear the Distance covered every 40*Tsample = 40 * M * 0.5/M = 20 seconds
            L=0;//Periodic Clearing of Accumulated Distance
        }
        ThisThread::sleep_for(100ms);        
    }
}

int main()
{

    lcd_clear();
    /////INIT TOUCH AND GYRO ////
    touch_thread.start(touch_actions);
    set_time(1256729737);  // Set RTC time to Wed, 28 Oct 2009 11:35:37
    init_filters();
    init_spi();
    gyro_thread.start(gyro_actions);
    while(1)
    {
        thread_sleep_for(100);// DO SOMETHING WITH GYRO DATA
    }

}
