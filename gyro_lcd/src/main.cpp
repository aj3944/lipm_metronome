#include "mbed.h"
#include "../lib/LCD_DISCO/LCD_DISCO_F429ZI.h"
#include "../lib/LCD_DISCO/TS_DISCO_F429ZI.h"
#include "../include/filtering.h"
#include "../include/ui_ux.h"
#include "../include/disconfig.h"

#define SAMPLING_FREQ 10 //Hz
#define SAMPLING_RATE (1000/SAMPLING_FREQ) //ms


//Calibrate : Gyro Thresholds in g to avoid the static noise and transients
#define xHThreshold 500
#define xLThreshold -500

//Calibrate : Scaling factor to accomodate 
#define scalingCoefficient 23.9


/// TOUCH AND DRAW THEADS
// TOUCH AT 5Hz
// DRAW AT 24Hz
Thread touch_thread;

Thread gyro_thread;


//bool do_sample = true;

//Counter to clear the screen and rotate the samples count
int8_t t = 0;



void touch_actions(){
    while(1){
        do_touch();
        DrawButtons();
        ThisThread::sleep_for(200ms);
    }
}

void gyro_actions(){
    time_t seconds = time(NULL);   
    unsigned int mills_o = (unsigned int)seconds*1000;
    seconds = time(NULL);   
    int x_int = 0,y_int = 0,z_int = 0;

    while(1)
    {
        // unsigned int mills_now = (unsigned int)seconds*1000;
        // if((mills_now - mills_o) > SAMPLING_RATE){
        //     do_sample = true;
        //     mills_o =  (unsigned int)seconds*1000;
        // }
        flags.wait_all(DATA_READY_FLAG);
        //prepare the write buffer to trigger a sequential read
        write_buf[0]=OUT_X_L|0x80|0x40;
        //start sequential sample reading
        spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
        flags.wait_all(SPI_FLAG);

		//For each sample , process each dimension reading sequentially
        for (j=0 ; j<d ; j++){
            gyroRead[w][j] = ( ( (uint16_t)read_buf[2*j+1+1] ) <<8 ) | ( (uint16_t)read_buf[2*j+1] );
        
		
		//Threshold to throttle the readings ;
		//If within threshold then reset the readings to eliminate static noise
        if (gyroRead[w][j] <= xHThreshold && gyroRead[w][j] >= xLThreshold) {gyroRead[w][j]=0;} 
        //
		
		if (debugSwitch) {printf(" t=%d\t, w=%d\t, r=%d\t,  j=%d\t,gyroRead[%d][%d]=%d\t, gyro[%d][%d]=%f \n",t, w ,r  , j ,w,j, gyroRead[w][j],w,j ,gyro[w][j]);}
	
		//Scale Gyro readings into rad/sec
		gyro[w][j] = ((float) gyroRead[w][j])*(17.5f*0.017453292519943295769236907684886f / 1000.0f);

		//Start Integration for a slice of the gyro samples with size F2 (D2 Delay elements + current sample,
		// For each dimension, integrate gyro[n],gyro[n-1],..,gyro[n-(F2-1)]
        //Slice a part of ciruclar buffer that matches filter size F2
		v=D2;
		for (k=r+(D-D2) ; k < r+(D-D2)+F2 ; k++ ){
				a2[v][j]=gyro[k%M][j];
				//Decrment V to make sure the first element in the loop will match the last element of the slice to align slice with filter coefficients
				v=v-1; 		
				}
        if (debugSwitch) {printf("===End Prep for Filter 2 using D2 Delays\n");}
        if (debugSwitch) {printf("===Start Applying Filter H2\n");}

        //g2[n][x or y or z dimension] signal is the result of applying Windowed Filter H2 on the slice a2
        g2[w][j]= dotProduct2(&a2[0][j], &H2[0][j],F2,d); 
        if (debugSwitch) {printf("w=%d \tr=%d \t(Delta Delay)=%d \t ri=r+deltaDelay=%d \tg2[%d][%d]=%f ads=%p\n", w, r, (D-D2) , r+(D-D2),w ,j ,g2[w][j], &g2[w][j]    );}
        if (debugSwitch) {printf("===End Applying Filter H2\n");}
        //End Signal Prop Block

        }
		if (debugSwitch) {printf("==End Read Gyro dim \n");}
		if (debugSwitch) {printf("==3 Dimensional Gyro Readings and Their Conversions\n");}
		if (debugSwitch) {printf("gyroRead[%d][j]=[%d\t %d\t %d\t]\n",w,gyroRead[w][0],gyroRead[w][1],gyroRead[w][2]);}
		if (debugSwitch) {printf("gyro[%d][j]=[%f\t %f\t %f\t]\n",w,gyro[w][0],gyro[w][1],gyro[w][2]);}

		//Section to calculate Instantaneous Velocity in x,y,z 
		if (debugSwitch) {printf("===Start Cross Product of Instanenous gyro g2 to get Velocity in XYX\n");}
		if (debugSwitch) {printf("Filtered Gyro Readings G2[%d][j]=[%f\t %f\t %f\t]\n",w,g2[w][0],g2[w][1],g2[w][2]);}
		if (debugSwitch) {printf("Cross product with r[%d][j]=[%d\t %d\t %d\t]\n",w,xdim, ydim, zdim);}
       
	   
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
		if (debugSwitch) {printf("Instantaneous Linear Velocity in X,Y,Z after corss V[w][j]=V[%d][j]= [%f \t%f \t%f]\n",w,V[w][0],V[w][1],V[w][2]);}
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
		
		if (debugSwitch) {printf(">Instant Distance gL[%d][0]:%f\n",w, gL[w][0]);}
		if (debugSwitch) {printf("Accumulated Distance Before Adding current Disantce= L = %f\n", L );}
		//The total distance covered is the summation of the instantenous distances
		L= L + gL[w][0];
		if (debugSwitch) {printf("Accumulated Distance After Adding current Disantce= L = %f\n", L );}
		if (debugSwitch) {printf("Total Distance L:%f\n",L);}

        Lint = (int)(scalingCoefficient*L); //magic number for human gait
		if (debugSwitch) {printf("Total Scaled Integer Distance L:%d\n",Lint);}
        
		//Reset the distance 
		if(!start_status){
            Lint = 0;
        }
        printf(">Interger Distance:%d\n",Lint);
        if(do_sample){       
            HomeDisplay(Lint,L);
            x_int = (int)(5000.0*g2[w][0]);
            y_int = (int)(5000.0*g2[w][1]);
            z_int = (int)(5000.0*g2[w][2]);
            AddPoint((int)x_int,(int)y_int,(int)z_int,t);
            // do_sample = false;
        }


        //Increment Pointers
        w=(w+1)%M;
        r=(r+1)%M;
        t=(t+1)%240;
        if(t==0){
            lcd_clear();
        }

		//To satisfy the challenge requirements, on every 0.5 seconds , the circular buffers will rotate to the first element, at this time
		//we will take the current filtered signals samples and keep them in a size 40-slower buffer
		//we will i-pointer to rotate around the 40-samples slower buffers
		if (w==0){
			for (j=0 ; j<d ; j++){
				gyroReadSlow[s][j]=gyroRead[w][j];
				gyroSlow[s][j]=gyro[w][j];
				g2Slow[s][j]=g2[w][j];
				VSlow[s][j]=V[w][j];
			}
				VLinearSlow[s][0]=VLinear[w][0];
				gLSlow[s][0]=gL[w][0];
				
				LSlow=LSlow+gLSlow[s][0];
				if (debugSwitch){
					printf("--Buffer Slow Gyro Readings float--\n");
					printArr2dInt(&gyroReadSlow[0][0],40,d);
				}
			s=(s+1)%40;


			if (debugSwitch3){
			printf("--Pointers--\n");
			printf("w=%d\t r=%d\t i=%d\t t=%d\n",w,r,s,t);

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
			printf("--Buffer Accumulated Distance--\n");
			printf("L=%f",L);

			printf("==========Printing Slow Buffers===========\n");
			printf("--Slow Buffer Gyro Reading Buffers--\n");
			printArr2dInt(&gyroReadSlow[0][0],40,d);
			printf("--Slow Buffer Gyro Scaled--\n");
			printArr2d(&gyroSlow[0][0],40,d);
			//printf("--Slow Buffer Filtered Gyro Readings--\n");
			//printArr2d(&g2Slow[0][0],40,d);
			printf("--Slow Buffer Velocity in X,Y,Z--\n");
			printArr2d(&VSlow[0][0],40,d);
			printf("--Slow Buffer Linear Velocity Magnitude--\n");
			printArr2d(&VLinearSlow[0][0],40,1);
			printf("--Slow Buffer Integrated Instantaneous Distance--\n");
			printArr2d(&gLSlow[0][0],40,1);
			printf("--Slow Accumulated Distance--\n");
			printf(">LSlow=%f",LSlow);
    		}
		

		if (debugSwitch2){
			printf(">write Pointer w:%d\n",w);
			printf(">read Pointer r:%d\n",r);
			printf(">write slow Pointer s:%d\n",s);
			printf(">Tdelayf:%f\n",Tdelay);
			printf(">Tdelayd:%d\n",Tdelay);
			printf(">Ts2d:%d\n",Ts2);
			printf(">Ts2f:%d\f",Ts2);
			printf(">t sample:%d\n",t);
			printf(">gyroRead_x:%d\n",gyroRead[w][0]);
			printf(">gyroRead_y:%d\n",gyroRead[w][1]);
			printf(">gyroRead_z:%d\n",gyroRead[w][2]);
			printf(">gyro_x:%f\n",gyro[w][0]);
			printf(">gyro_y:%f\n",gyro[w][1]);
			printf(">gyro_z:%f\n",gyro[w][2]);
			printf(">g2_x:%f\n",g2[w][0]);
			printf(">g2_y:%f\n",g2[w][1]);
			printf(">g2_z:%f\n",g2[w][2]);
			//printf(">g3_x:%f\n",g3[w][0]);
			//printf(">g3_y:%f\n",g3[w][1]);
			//printf(">g3_z:%f\n",g3[w][2]);
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
			printf(">gyroSlow_x:%f\n",gyroSlow[i][0]);
			printf(">gyroSlow_y:%f\n",gyroSlow[i][1]);
			printf(">gyroSlow_z:%f\n",gyroSlow[i][2]);
			// printf(">g2Slow_x:%f\n",g2Slow[i][0]);
			//printf(">g2Slow_y:%f\n",g2Slow[i][1]);
			//printf(">g2Slow_z:%f\n",g2Slow[i][2]);
			printf(">VSlow_x:%f\n",VSlow[i][0]);
			printf(">VSlow_y:%f\n",VSlow[i][1]);
			printf(">VSlow_z:%f\n",VSlow[i][2]);
			printf(">VLinearSlow:%f\n",VLinearSlow[i][0]);
			printf(">L_instantSlow:%f\n",gLSlow[i][0]);
			printf(">L_total_Slow:%f\n",LSlow);
			}



    }




        ThisThread::sleep_for(100ms);        
    }
}

int main()
{

    lcd_clear();
    /////INIT TOUCH AND GYRO ////
    touch_thread.start(touch_actions);

    init_filters();
    init_spi();
    gyro_thread.start(gyro_actions);
    thread_sleep_for(1000);// DO SOMETHING WITH GYRO DATA
    lcd_clear();

    while(1)
    {
        thread_sleep_for(10);// DO SOMETHING WITH GYRO DATA
    }

}
