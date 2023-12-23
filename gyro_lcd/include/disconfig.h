#include "mbed.h"

//address of first register with gyro data
#define OUT_X_L 0x28

//Location of Reference register
#define REFERENCE 0x25

//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20

//configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
//DR=01 , BW=10 --> 190Hz (190 readings per second) and 50Hz Cutoff , PD=1 , Zen=1 , Xen=1 , Yen=1
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1

//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG2 0x21

//configuration: Reserved=00 , HPM = 00 (Normal Mode Reset Filter), HPCF=0111 (at ODR=190 then cutoff = 0.09 Hz)
#define CTRL_REG2_CONFIG 0b00'00'0'1'1'1


//register fields(bits): I1_Int1 (1), I1_Boot(1), H_Lactive(1), PP_OD(1), I2_DRDY(1), I2_WTM(1), I2_ORun(1), I2_Empty(1)
#define CTRL_REG3 0x22

//configuration: Int1 disabled, Boot status disabled, active high interrupts, push-pull, enable Int2 data ready, disable fifo interru
#define CTRL_REG3_CONFIG 0b10'00'1'0'0'0


//register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG4 0x23

//configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
// Block Data Update = 0 (Continous value) , BLE = 0 LSB data at Lower Address , FS=01 (500 DPS) , Registered = 0 , Fixed = 00 , SIM = 0 (SPI interface Mode Selection = 4-Wire interface)  

//CTRL_REG4_CONFIG 0b0'0'01'0'00'0 this was the lecture value
//We increased the range to 2000 DPS by setting FS=11
#define CTRL_REG4_CONFIG 0b0'0'11'0'00'0

// BOOT(1)-FIFO_EN(1)-(1)-HPen(1)-INT1_Sel1(1)-INT1_Sel0(1)-Out_Sel1(1)-Out_Sel0(1)
#define CTRL_REG5 0x24

//FIFO EN //hpass en //INT HPASS AND LOW PASS
#define CTRL_REG5_CONFIG 0b01'01'1'0'1'0

// BOOT(1)-FIFO_EN(1)-(1)-HPen(1)-INT1_Sel1(1)-INT1_Sel0(1)-Out_Sel1(1)-Out_Sel0(1)
#define REFERENCE 0x25
#define OUT_TEMP 0x26
#define STATUS_REG 0x27

#define FIFO_CTRL_REG 0x2e
#define FIFO_CTRL_CONFIG 0b00'11'1'0'1'0


#define FIFO_SRC_REG 0x2f

#define INT1_CFG 0x30
#define INT1_CFG_CONFIG 0b01'00'0'0'1'0


#define INT1_SRC 0x31


#define INT1_THS_XH 0x32
#define INT1_THS_XH_CONFIG 0b00'00'1'0'0'0
#define INT1_THS_XL 0x33
#define INT1_THS_XL_CONFIG 0b00'00'0'1'0'0

#define INT1_THS_YH 0x34
#define INT1_THS_YL 0x35

#define INT1_THS_ZH 0x36
#define INT1_THS_ZL 0x37


#define INT1_DURATION 0x38
#define INT1_DURATION_CONFIG 0b10'00'0'1'0'0




#define SPI_FLAG 1
#define DATA_READY_FLAG 2

SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); // mosi, miso, sclk, cs
InterruptIn int2(PA_2,PullDown);

EventFlags flags;

uint8_t write_buf[32]; 
uint8_t read_buf[32];

void spi_cb(int event){
  flags.set(SPI_FLAG);
}

void data_cb(){
  flags.set(DATA_READY_FLAG);
}

void init_spi(){
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


    // write to reference register , then 1100
    write_buf[0]=REFERENCE|0x80|0x40;
    
    //start sequential sample reading
    spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);   

    int2.rise(&data_cb);
    if(!(flags.get()&DATA_READY_FLAG)&&(int2.read()==1)){
        flags.set(DATA_READY_FLAG);
    }  

}