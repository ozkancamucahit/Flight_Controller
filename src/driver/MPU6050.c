/*
 * MPU6050.c
 *
 * Created: 03/01/2021 16:29:44
 *  Author: mmuca
 */ 

#include "MPU6050.h"

    uint8_t data = 0;

    twi_packet_t packet = 
    {
        .chip_addr = (uint32_t)MPU6050_DEFAULT_ADDRESS,
        .addr[0] = 0,
        .addr_length = 0,
        .buffer = &data,
        .length = 1
    };

void setupMPU(void)
{
    /* WAKEUP BRUH... */
    while ( twi_master_write(TWI1, &packet) != TWI_SUCCESS );
    
    /* +- 250degree */
    packet.addr[0] = MPU6050_RA_GYRO_CONFIG;
    while ( twi_master_write(TWI1, &packet) != TWI_SUCCESS );
    /* +-2g */
    packet.addr[0] = MPU6050_RA_ACCEL_CONFIG;
    while ( twi_master_write(TWI1, &packet) != TWI_SUCCESS );


}
void recordAccelRegisters(MPU_data_t* data)
{
    uint8_t dataToRead[6];
    packet.addr[0] = MPU6050_RA_ACCEL_XOUT_H;
    //packet.length = 1;
    
    // TODO: Dogru mu ?
    //while ( twi_master_write(TWI1, &packet) != TWI_SUCCESS );
	
    packet.buffer = dataToRead;
    packet.length = 6; // bytes of data
    if ( twi_master_read( TWI1, &packet ) == TWI_SUCCESS )
        {
            data->accelX = dataToRead[0] << 8 | dataToRead[1];
            data->accelY = dataToRead[2] << 8 | dataToRead[3];
            data->accelZ = dataToRead[4] << 8 | dataToRead[5];
            processAccelData( data );
        }
    else {
        // TODO: lol
        printf("\tFAIL Reading accel data\n");
    }  
}
void recordGyroRegisters(MPU_data_t* data)
{
    uint8_t dataToRead[6];
    packet.addr[0] = MPU6050_RA_GYRO_XOUT_H;
    //packet.length = 1;
    

    //while ( twi_master_write(TWI1, &packet) != TWI_SUCCESS );

    packet.buffer = dataToRead;
    packet.length = 6; // bytes of data

    if ( twi_master_read( TWI1, &packet ) == TWI_SUCCESS )
         {
            data->gyroX = dataToRead[0] << 8 | dataToRead[1];
            data->gyroY = dataToRead[2] << 8 | dataToRead[3];
            data->gyroZ = dataToRead[4] << 8 | dataToRead[5];
            processGyroData( data );
        }
    else {
        // TODO:
        printf("\tFAIL Reading GYRO data\n");

    }  

}


void processGyroData(MPU_data_t* data)
{
    data->rotX = data->gyroX / 131.0;
    data->rotY = data->gyroY / 131.0; 
    data->rotZ = data->gyroZ / 131.0;
}
void processAccelData(MPU_data_t* data)
{
    data->gForceX = data->accelX / 16384.0;
    data->gForceY = data->accelY / 16384.0; 
    data->gForceZ = data->accelZ / 16384.0;
}



void printData(MPU_data_t* data)
{
    printf("Gyro (deg) X: %f\tY:%f\tZ:%f\tAccel(g) X: %f\tY: %f\t Z: %f\n",data->rotX, data->rotY, data->rotZ, data->gForceX, data->gForceY,data->gForceZ );
	//printf("lol %f", 13.2);
	//printf("selam");
}










