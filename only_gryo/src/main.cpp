#include "mbed.h"

// #define 
SPI spi(PF_9, PF_8, PF_7); // mosi, miso, sclk
DigitalOut cs(PC_1);
 



int writeReg(int address, int data){
    cs = 0;
    spi.write(address);
    int reg_write = data;
    spi.write(reg_write);
    // printf("W [%x]=%x\n", address,reg_write);
    cs = 1;
    // thread_sleep_for(1);
    return 1;
}


int readReg(int address){
    cs = 0;
    spi.write(address);
    int reg_read = spi.write(0x00);
    // printf("R [%x]=%x\n", address,reg_read);
    cs = 1;
    // thread_sleep_for(1);
    return reg_read;
}

int main() {
    // Chip must be deselected
    cs = 1;

    // Setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate
    spi.format(8,3);
    spi.frequency(1000000);
 
    // Select the device by seting chip select low
    // cs = 0;
 
    // Send 0x8f, the command to read the WHOAMI register
 
    // Send a dummy byte to receive the contents of the WHOAMI register

    writeReg(0x20,0xFF);
    writeReg(0x21,0x1A);
    writeReg(0x2E,0x00);
    // int ctrl_reg1 = readReg(0xA0);
    // printf("REG1 = 0x%X\n", ctrl_reg1);
    // int ctrl_reg2 = readReg(0xA1);
    // printf("REG2 = 0x%X\n", ctrl_reg2);

    int out_vals[6];
    int out_xyz[3];



    int last_sane_xyz[3] = { 0 };
    int smooth_xyz[3] = { 0 };
    int avg_xyz[3] = { 0 };
    int normal_xyz[3] = { 0 };

    int rs = 10;
    
        // int whois_reg = readReg(0x8f);
        // printf("WHOA AM I= 0x%X\n", whois_reg);
        // int status_reg = readReg(0xA7);
        // printf("STATUS REG = 0x%X\n", status_reg);
        // int fifo_src_reg = readReg(0xAf);
        // printf("FIFO_SRC REG = 0x%X\n", fifo_src_reg);
    while(1){

        for(int i = 0 ; i < 6; i++){
            int ad = 0xA8 + i;
            out_vals[i] = readReg(ad);
        }
        for(int i = 0; i < 3; i++){
            out_xyz[i] = out_vals[i] + (out_vals[i + 1] ^ 0x80) - (out_vals[i + 1] & 0x80);
            last_sane_xyz[i] = out_xyz[i];
            smooth_xyz[i] = (rs - 1)* smooth_xyz[i]/(rs) + last_sane_xyz[i]/(rs);
            normal_xyz[i] = last_sane_xyz[i] - smooth_xyz[i];
            avg_xyz[i] = avg_xyz[i]/2 + normal_xyz[i]/2;
        }
        for(int i = 0 ; i < 3; i++){
            printf(">[%x]:%d\n",i,normal_xyz[i]);
        }
        printf("\n");
        // thread_sleep_for(200);
    };  
}