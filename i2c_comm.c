#include <hardware/i2c.h>
#include <pico/i2c_slave.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>

/*
    * Example to use the RPI Pico as i2c Host
    * Dummy firwmare to debug i2c communication with a Raspberry Pi
 */

static const uint I2C_SLAVE_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 100000; // 100 kHz

// Raspberry Pi Pico is running as i2c host
static const uint I2C_SLAVE_SDA_PIN = 0; // 
static const uint I2C_SLAVE_SCL_PIN = 1; // 

static struct
{
    uint8_t mem[256];
    uint8_t mem_address;
    bool mem_address_written;
} uut_mem;

uint8_t command[2];
int bytecount = 0;
bool cmd = false;

/**
 * In this example, the master will send two bytes [127, 3] 
 * to let the Pico know we want to read or manipulate information in its memory
*/

// Function that handles the i2c interrupts
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    // This interruption will happen for as long as there are bytes to process 
    switch (event) 
    {
    case I2C_SLAVE_RECEIVE: 
        // Host is not ready for commands, thus will check if this inbound data is [127, 3]
        if (!cmd && bytecount < 2)
        {
            command[bytecount] = i2c_read_byte_raw(i2c);
            bytecount++;
        }
        // Host is ready for commands since [127, 3] was sent already
        else if (cmd) {            
            // The first byte is the data's address (or register)
            if (!uut_mem.mem_address_written) // If first byte has not been processed yet...
            {
                uut_mem.mem_address = i2c_read_byte_raw(i2c); //define the 1st byte as the starting address
                uut_mem.mem_address_written = true;
            }
            else  // This is now the actual data 
            {
                uut_mem.mem[uut_mem.mem_address] = i2c_read_byte_raw(i2c); // write to the address each individual byte
                uut_mem.mem_address++; // increase to the next address
            }

        }
        // most likely a read request, so just move around the pointer to the next data
        else {
                uut_mem.mem_address = i2c_read_byte_raw(i2c);
        }
        break;
    case I2C_SLAVE_REQUEST: // i2c Master is requesting data, for each read request, there's at least 1 byte received, the reg address. 
        // Read data from UUT memory               
        i2c_write_byte_raw(i2c, uut_mem.mem[uut_mem.mem_address]); //uut_mem.mem_address is set by the master's reg address 
        uut_mem.mem_address++;
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart        
        // This is an acknowledged command [127, 0]
        if (!cmd && command[0] == 127 && command[1] == 3){
            cmd = true;
            // Reset command buffer
            command[0] = 0x00;
            command[1] = 0x00;
        }
        // Other type of command was received
        else { 
            cmd = false;
        }
        bytecount = 0;
        uut_mem.mem_address_written = false;
        break;
    default:
        break;
    }
}

static void setup_slave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

int main() {
    
    // Firmware Version 0.5
    uut_mem.mem[39] = 0x00;
    uut_mem.mem[40] = 0x05;
    
    // DSP version
    uut_mem.mem[182] = 30; //0
    uut_mem.mem[183] = 31; //1
    uut_mem.mem[184] = 32; //2
    uut_mem.mem[185] = 33; //3

    // Serial number
    // RPI-PICO010203
    uut_mem.mem[166] = 0x52; //R
    uut_mem.mem[167] = 0x50; //P
    uut_mem.mem[168] = 0x49; //I
    uut_mem.mem[169] = 0x2d; //-
    uut_mem.mem[170] = 0x50; //P
    uut_mem.mem[171] = 0x49; //I
    uut_mem.mem[172] = 0x43; //C
    uut_mem.mem[173] = 0x4F; //O

    // Part Number
    // Part number RP0002
    uut_mem.mem[148] = 0x52;
    uut_mem.mem[149] = 0x50;
    uut_mem.mem[150] = 0x30;
    uut_mem.mem[151] = 0x30;
    uut_mem.mem[152] = 0x30;
    uut_mem.mem[153] = 0x32;    
    
    stdio_init_all();
    // Write to UART, unused as I dont have the UART connected in the RPi 
    // puts("\ni2c Host FW: %x.%x",&uut_mem[39],&uut_mem[40]);
    setup_slave();

/**
 * Copied from the blink example, let the user know a cmd [127, 3] was acknowledged by the Pico, is ready to r/w operations
*/

#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // If the command was received, update the LED
    while (true) {
        gpio_put(LED_PIN, cmd);
        sleep_ms(500);
    }

#endif

}
