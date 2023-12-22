// Online C compiler to run C program online
#include <stdio.h>
#include <stdint.h>
#define N1 5 //buffer length
#define N2 3 
//New Readings batch - Sliding Window Shift, N2<N1
#define N3 4 //filter length N3 <= (N1+N2)



void shiftR(int32_t *xn, int32_t *x_){//Shift Left by a single value
    int8_t i=0;//Use signed due to the decremented index
    
    //Start from Right to Left
    for (i=(N1-1)-1 ; i>=0 ; i--){
        //printf("%d\t%d\n",i,*(xn+i));
        *(xn+i+1)= *(xn+i);
    }
    *(xn)=*(x_);
    
    printf("============\n");
}


void shiftR2(int32_t *xn, int32_t *x_){ //Shift Left by an array
    int8_t i=0;
    //start=0;
    //shift=N2;
    //width=N1-N2;
    
    //i from width index which is width-1 till start 0, all shifted by Shift
    //we started from the end element to preserve the order of operations
    for (i=(N1-N2)-1 ; i>=0 ; i--){
        *(xn+i+N2)= *(xn+i);
    }

    //Replace the shifted elements
    for (i=0 ; i<=(N1-N2) ; i++){
        *(xn+i)=*(x_+i);
    }
    printf("============\n");
}

void shiftL(int32_t *xn, int32_t *x_){//Shift Left by a single value
    int8_t i=0;//Use signed due to the decremented index
    
    //Start from Left to Right
    for (i=1 ; i<=(N1-1) ; i++){
        //printf("%d\t%d\n",i,*(xn+i));
        *(xn+i-1)= *(xn+i);
    }
    *(xn+(N1-1))=*(x_);
    
    printf("============\n");
}


void shiftL2(int32_t *xn, int32_t *x_){ //Shift Left by an array
    int8_t i=0;
    //shift=N2;
    //width=N1-N2;
    //since shift left then start from most left;
    // Range ; (N1-1 - width)--(N1-1)= (N1-1 -N1+N2)--(N1-1) == (N2-1)--(N1-1)
    
    for (i=(N2)-1 ; i<=(N1-1) ; i++){
        *(xn+i-N2)= *(xn+i);
    }

    //Replace the shifted elements
    //Reverse
    for (i=N1-1 ; i>=(N2-1) ; i--){
        *(xn+i)=*(x_+i-(N2-1));
    }
    printf("============\n");
}



void dotProduct(int32_t *an, int32_t *bn, int32_t *r){
    *r=0; //init to avoid unexpected values if you called the same function with same r sequential
    int8_t i=0;
    for (i=0 ; i<N3; i++){
        printf("%d\t%d\t%d\n",i,*(an+i),*(bn+i));
        *r= *r + *(an+i) * *(bn+i);
        
    }
    printf("%d\n",*r);
    printf("============\n");

}

void timeConv1R(int32_t *x, int32_t *h ,int32_t *yy){///Sample by Sample
    
    dotProduct(h, x, yy);
    printf("============\n");
    }


void timeConv1L(int32_t *x, int32_t *h ,int32_t *yy){///Sample by SamplerPtr=&r[0];
    int32_t *xr;
    xr=(int32_t *) x+(N1-1)-(N3)+1; //
    printf("%d\n",*xr);
    dotProduct(h, xr, yy);
    printf("============\n");
    }


void printArr(int32_t *a, int8_t K){
    uint8_t i=0;
    //printf("%d \t\n",S);
    for (i=0 ; i<K ; i++){
        printf("%d \t %d \n",i,*(a+i));
    }
    printf("============\n");
}

void printArrR(int32_t *a, int8_t K){
    int8_t i=0;
    //printf("%d \t\n", K);
    for (i=-(K-1) ; i<=0 ; i++){
        printf("%d \t %d \n",i,*(a+i));
    }
    printf("============\n");
}





int main() {
    // Write C code here
    int32_t x[N1] = {0,1,2,3,4} ;    
    int32_t x_[N2]={6,7,8};
    uint32_t h[N3]={1,0,1,1};
    uint32_t y[N2]={0};
    uint32_t yy[1]={0};
    //uint32_t buff[N1+N2]={0};
    uint8_t i=0;
    int32_t xx[1]={5};
    int32_t r[1]={0};

    
    
    int32_t *xPtr;
    int32_t *xxPtr;
    int32_t *yPtr, *hPtr, *xnPtr, *x_ptr, *yyPtr;
    //int32_t  *buffPTR;
    int32_t *rPtr;
    
    yyPtr=&yy[0];
    //rPtr=&r[0];
    xPtr=&x[0];
    xxPtr=&xx[0];
    yPtr=&y[0];
    hPtr=&h[0];
    xnPtr=&x[N1-1];
    x_ptr=&x_[0];
    //buffPTR=&buff[0];
    
    //dotProduct(xPtr,xPtr,rPtr);
    
    //shiftL(xPtr, xxPtr);
    //shiftL2(xPtr, x_ptr);

    //shiftR(xPtr, xxPtr);
    //shiftR2(xPtr, x_ptr);

    printArr(xPtr,N1);
    timeConv1R(xPtr, hPtr, yyPtr); //assuming h={h0, h1,... hN3}} 
    //and x is right shifted={x(0), x(-1), x{-2},...}, i.e. Latest sample is the most left
    
    timeConv1L(xPtr, hPtr, yyPtr); //assuming h={hN3,.....,h1, h0}} 
    //and x is left shifted={....,x(-2), x(-1), x{0}}, i.e. Latest sample is the most right
    

    //printArr(yPtr,N1);
    //printArrR(xnPtr, N1);
    //printArr(xPtr,N1,1);
    //printf("==%d\n", *xnPtr);
    //printf("==%d\n", *(xnPtr-1));
    //printf("==%d\n", *(xnPtr-2));

    
    //for (i=0 ; i<N1 ; i++){
      //  printf("%d \t \n",*(buff+i));
    //}
    return 0;
}
