/*
 * MPU6050.c
 *
 * Created: 03/01/2021 16:29:44
 *  Author: mmuca
 */ 

#include "MPU6050.h"


#define BUFFER_LENGTH 64


const uint32_t XMIT_TIMEOUT = 100000;
const uint32_t RECV_TIMEOUT = 100000;


    uint8_t rxBuffer[BUFFER_LENGTH];
	uint8_t rxBufferIndex;
	uint8_t rxBufferLength;

	// TX Buffer
	uint8_t txAddress;
	uint8_t txBuffer[BUFFER_LENGTH];
	uint8_t txBufferLength;

	// Service buffer
	uint8_t srvBuffer[BUFFER_LENGTH];
	uint8_t srvBufferIndex;
	uint8_t srvBufferLength;

    twi_options_t opt =
    { 
	.smbus = 0,
    .chip = MPU6050_ADDRESS_AD0_LOW,
    .speed = 400000UL,
    .master_clk = 84000000UL
    };
    
    twi_packet_t packet = 
    {
        .chip_addr = MPU6050_DEFAULT_ADDRESS,
        .iaddr[0] = 0,
        .iaddr_length = 0,
        .buffer = 0,
        .length = 1
    };

    

void setupMPU(Twi* p_twi)
{
    // Disable PDC channel	
    
     if ( twi_master_init(p_twi, &opt) != TWI_SUCCESS ) printf("FAILED master I2C init\n");
    static uint8_t buffer_t [] = {MPU6050_RA_PWR_MGMT_1, 0x00U};
    packet.buffer = buffer_t;
    packet.length = 2;
    if ( twi_master_write(p_twi, &packet) != TWI_SUCCESS ) printf("Setup write FAIL\n");
    return;
}
void recordAccelRegisters(MPU_data_t* data, Twi* p_twi)
{
    // begin_TX(MPU6050_DEFAULT_ADDRESS);
    // write(MPU6050_RA_ACCEL_XOUT_H);
    // end_TX(p_twi);
    // reqFrom(MPU6050_DEFAULT_ADDRESS, 6, &packet, p_twi);

    // data->accelX = read() << 8 | read();
    // data->accelY = read() << 8 | read();
    // data->accelZ = read() << 8 | read();

    uint8_t buff_read [] = {0U,0U,0U,0U,0U,0U};
    uint8_t accel_read_start = MPU6050_RA_ACCEL_XOUT_H;

    packet.buffer = &accel_read_start;
    packet.length = 1UL;
    
    if ( twi_master_write(p_twi, &packet) != TWI_SUCCESS )
    {
        printf("FAIL\n");
        while (1);
    }
        

    packet.buffer = buff_read;
    packet.length = 6UL;

    if ( twi_master_read(p_twi, &packet) != TWI_SUCCESS )
    {
        printf("FAIL\n");
        while (1);
        return;
    }
       

    data->accelX = buff_read[0] << 8 | buff_read[1];
    data->accelY = buff_read[2] << 8 | buff_read[3];
    data->accelZ = buff_read[4] << 8 | buff_read[5];
    processAccelData( data );
}
void recordGyroRegisters(MPU_data_t* data, Twi* p_twi)
{
    // begin_TX(MPU6050_DEFAULT_ADDRESS);
    // write(MPU6050_RA_GYRO_XOUT_H);
    // end_TX(p_twi);
    // reqFrom(MPU6050_DEFAULT_ADDRESS, 6, &packet, p_twi);


    // data->gyroX = read() << 8 | read();
    // data->gyroY = read() << 8 | read();
    // data->gyroZ = read() << 8 | read();

    uint8_t buff_read [] = {0U,0U,0U,0U,0U,0U};
    uint8_t gyro_read_start = MPU6050_RA_GYRO_XOUT_H;

    packet.buffer = &gyro_read_start;
    packet.length = 1UL;

    if ( twi_master_write(p_twi, &packet) != TWI_SUCCESS )
    {
        printf("FAIL\n");
        while (1);
        return;
    }

    packet.buffer = buff_read;
    packet.length = 6UL;

    if ( twi_master_read(p_twi, &packet) != TWI_SUCCESS )
    {
        printf("FAIL\n");
        while (1);
        return;
    }

    data->gyroX = buff_read[0] << 8 | buff_read[1];
    data->gyroY = buff_read[2] << 8 | buff_read[3];
    data->gyroZ = buff_read[4] << 8 | buff_read[5];
    processGyroData( data );
        

}


void processGyroData(MPU_data_t* data)
{
    data->rotX = data->gyroX / 131.0F;
    data->rotY = data->gyroY / 131.0F; 
    data->rotZ = data->gyroZ / 131.0F;
}
void processAccelData(MPU_data_t* data)
{
    data->gForceX = data->accelX / 16384.0F;
    data->gForceY = data->accelY / 16384.0F; 
    data->gForceZ = data->accelZ / 16384.0F;
}


void printData(MPU_data_t* data)

{
    // AccERROR X: 0.538004	AccERROR Y: -1.709946	GyroERROR X: -2.885879	GyroERROR Y: 1.171870	GyroERROR Z: 0.669924
    //printf("Gyro (deg) X: ");
    //printf(" ");
    print_Float(data->rotX /*- data->GyroErrorX*/, PRINT_SIZE);
    printf(", ");

    //printf("Y: ");
    print_Float(data->rotY /*- data->GyroErrorY*/, PRINT_SIZE);
    printf(", ");

    //printf("Z: ");
    print_Float(data->rotZ /*- data->GyroErrorZ*/, PRINT_SIZE);
    printf(", ");

    //printf("ACCEL (g) X: ");
    print_Float(data->gForceX /*- data->accErrorX*/, PRINT_SIZE);
    printf(", ");

    //printf("Y: ");
    print_Float(data->gForceY /*- data->accErrorY*/, PRINT_SIZE);
    printf(", ");

    //printf("Z: ");
    print_Float(data->gForceZ, PRINT_SIZE);
    printf("\r\n");
}

void calculate_offset(Twi* p_twi, MPU_data_t* p_data,const uint16_t samples)
{
    // AccERROR X: 0.538004	AccERROR Y: -1.709946	GyroERROR X: -2.885879	GyroERROR Y: 1.171870	GyroERROR Z: 0.669924

    uint16_t count = 0;

    p_data->accErrorX = 0.0f; p_data->accErrorY = 0.0f; p_data->GyroErrorX = 0.0f; p_data->GyroErrorY = 0.0f; p_data->GyroErrorZ = 0.0F;
    p_data->gyroAngleX = 0.0F, p_data->gyroAngleY=0.0F, p_data->yaw = 0.0F;

    while ( count < samples )
    {
        recordAccelRegisters( p_data, p_twi );
        // sum all
        p_data->accErrorX +=  ((atan((p_data->gForceY) / sqrt(pow((p_data->gForceX), 2) + pow((p_data->gForceZ), 2))) * 180 / M_PI));
        p_data->accErrorY +=  ((atan(-1 * (p_data->gForceX) / sqrt(pow((p_data->gForceY), 2) + pow((p_data->gForceZ), 2))) * 180 / M_PI));
        ++count;
    }

    // printf("AccERROR before sumX: "); print_Float(p_data->accErrorX, PRINT_SIZE); printf("\t");
    // printf("AccERROR before sumY: "); print_Float(p_data->accErrorY, PRINT_SIZE); printf("\n");

    p_data->accErrorX = (p_data->accErrorX / samples);//*2;
    p_data->accErrorY = (p_data->accErrorY / samples);//*2;

    count = 0;

    while ( count < samples )
    {
        recordGyroRegisters( p_data, p_twi );
        p_data->GyroErrorX = p_data->GyroErrorX + p_data->rotX;
        p_data->GyroErrorY = p_data->GyroErrorY + p_data->rotY;
        p_data->GyroErrorZ = p_data->GyroErrorZ + p_data->rotZ;
        ++count;
    }

    // printf("GyroERROR before X: "); print_Float(p_data->GyroErrorX, PRINT_SIZE); printf("\t");
    // printf("GyroERROR before Y: "); print_Float(p_data->GyroErrorY, PRINT_SIZE); printf("\t");
    // printf("GyroERROR before Z: "); print_Float(p_data->GyroErrorZ, PRINT_SIZE); printf("\n");

    p_data->GyroErrorX = (p_data->GyroErrorX / samples);//*2;
    p_data->GyroErrorY = (p_data->GyroErrorY / samples);//*2;
    p_data->GyroErrorZ = (p_data->GyroErrorZ / samples);//*2;

    // printf("AccERROR X: "); print_Float(p_data->accErrorX, PRINT_SIZE); printf("\t");
    // printf("AccERROR Y: "); print_Float(p_data->accErrorY, PRINT_SIZE); printf("\t");
    // printf("GyroERROR X: "); print_Float(p_data->GyroErrorX, PRINT_SIZE); printf("\t");
    // printf("GyroERROR Y: "); print_Float(p_data->GyroErrorY, PRINT_SIZE); printf("\t");
    // printf("GyroERROR Z: "); print_Float(p_data->GyroErrorZ, PRINT_SIZE); printf("\n");


}

void calculate_roll_pitch_yaw( MPU_data_t* p_data, float elapsed_time )
{
    // calculate roll and pitch & correct outputs
    p_data->accAngleX = (atan(p_data->gForceY / sqrt(pow(p_data->gForceX, 2) + pow(p_data->gForceZ, 2))) * 180 / M_PI) - p_data->accErrorX; 
    p_data->accAngleY = (atan(-1 * p_data->gForceX / sqrt(pow(p_data->gForceY, 2) + pow(p_data->gForceZ, 2))) * 180 / M_PI) - p_data->accErrorY;

    // printf("accangleX : ");
    // print_Float(p_data->accAngleX, PRINT_SIZE);
    // printf("\taccangleY : ");
    // print_Float(p_data->accAngleY, PRINT_SIZE);
    // printf("\n");

    p_data->rotX -= p_data->GyroErrorX;
    p_data->rotY -= p_data->GyroErrorY;
    p_data->rotZ -= p_data->GyroErrorZ;

    // printf("gyroX : ");
    // print_Float(p_data->rotX, PRINT_SIZE);
    // printf("\tgyroY : ");
    // print_Float(p_data->rotY, PRINT_SIZE);
    // printf("\tgyroZ : ");
    // print_Float(p_data->rotZ, PRINT_SIZE);
    // printf("\n");


    
    p_data->gyroAngleX += p_data->rotX * (elapsed_time);
    p_data->gyroAngleY += p_data->rotY * (elapsed_time);
    p_data->yaw += p_data->rotZ * (elapsed_time);

    // printf("gyroAngleX : ");
    // print_Float(p_data->gyroAngleX, PRINT_SIZE);
    // printf("\tgyroAngleY : ");
    // print_Float(p_data->gyroAngleY, PRINT_SIZE);
    // printf("\tgyroAngleZ : ");
    // print_Float(p_data->gyroAngleZ, PRINT_SIZE);
    // printf("\n");

      // Complementary filter - combine acceleromter and gyro angle values
    p_data->roll = 0.96F * p_data->gyroAngleX + 0.04F * p_data->accAngleX;
    p_data->pitch = 0.96F * p_data->gyroAngleY + 0.04F * p_data->accAngleY;

    //printf(" ");
    print_Float(p_data->roll, PRINT_SIZE); printf(", ");
    print_Float(p_data->pitch, PRINT_SIZE); printf(", ");
    print_Float(p_data->yaw, PRINT_SIZE); printf("\r\n");


}


/**
 * \brief Test with twi packet & read WHO_AM_I bit
 * 
 */
void test_Connection(Twi* p_twi)
{
    
    // begin_TX(MPU6050_DEFAULT_ADDRESS);
    // write(MPU6050_RA_WHO_AM_I);
    // end_TX(TWI1);
    // reqFrom(MPU6050_DEFAULT_ADDRESS, 1, &packet, TWI1);

    uint8_t dizi [] = {MPU6050_RA_WHO_AM_I};
    uint8_t who_r_u = 0U, who_am_i = MPU6050_RA_WHO_AM_I;
    packet.buffer = &dizi[0];
    packet.length = 1U;

    if ( twi_master_write(p_twi, &packet) != TWI_SUCCESS )
        return;

    packet.buffer = &who_r_u;
    packet.length = 1UL;

    if ( twi_master_read(p_twi, &packet) != TWI_SUCCESS )
        return;

    if ( who_r_u ==  MPU6050_DEFAULT_ADDRESS ) printf("TEST SUCCESS %u read\n\r", who_r_u);

    else printf("TEST FAIL %u read\n\r", who_r_u);

    return;
}


void begin_TX(uint8_t addr)
{
    status = MASTER_SEND;
    txAddress = addr;
	txBufferLength = 0; // empty buffer
}

size_t write( uint8_t data )
{
    if (status == MASTER_SEND) {
		if (txBufferLength >= BUFFER_LENGTH)
			return 0;
		txBuffer[txBufferLength++] = data;
		return 1;
	} else {
		if (srvBufferLength >= BUFFER_LENGTH)
			return 0;
		srvBuffer[srvBufferLength++] = data;
		return 1;
	}
}

uint8_t end_TX(Twi* twi)
{
    uint8_t error = 0;
	// transmit buffer (blocking)
	twi_start_write(twi, &packet, txBuffer[0]);
	if (!twi_WaitByteSent(twi, XMIT_TIMEOUT))
		error = 2;	// error, got NACK on address transmit
	
	if (error == 0) {
		uint16_t sent = 1;
		while (sent < txBufferLength) {
			twi_write_byte(twi, txBuffer[sent++]);
			if (!twi_WaitByteSent(twi, XMIT_TIMEOUT))
				error = 3;	// error, got NACK during data transmmit
		}
	}
	
	if (error == 0) {
		Send_Stop(twi);
		if (!twi_WaitTransferComplete(twi, XMIT_TIMEOUT))
			error = 4;	// error, finishing up
	}

	txBufferLength = 0;		// empty buffer
	status = MASTER_IDLE;
	return error;
}

uint8_t reqFrom(uint8_t addr, uint8_t length, const twi_packet_t* p_packet, Twi* p_twi)
{
    if (length > BUFFER_LENGTH)
		length = BUFFER_LENGTH;

	int readed = 0;
    twi_start_read(p_twi, &packet);

    do {
		// Stop condition must be set during the reception of last byte
		if (readed + 1 == length)
			Send_Stop( p_twi );

		if (twi_WaitByteReceived(p_twi, RECV_TIMEOUT))
			rxBuffer[readed++] = twi_read_byte(p_twi);
		else
			break;
	} while (readed < length);
	twi_WaitTransferComplete(p_twi, RECV_TIMEOUT);

	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = readed;

	return readed;
    

}


int read(void)
{
    if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex++];
	return -1;
}

size_t print_Float(float number, uint8_t digits)
{
    size_t n = 0;

    if ( __isnanf(number) ) return printf("nan");
    if ( __isinff(number) ) return printf("inf");

    if (number > 4294967040.0) return printf ("ovf");
    if (number <-4294967040.0) return printf ("ovf");

    if (number < 0.0)
    {
     n += printf("-");
     number = -number;
    }

    double rounding = 0.5;
    for (uint8_t i=0; i<digits; ++i)
        rounding /= 10.0;
  
    number += rounding;

    // Extract the integer part of the number and printf it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    n += printf("%lu", int_part);

    // Printf the decimal point, but only if there are digits beyond
    if (digits > 0) {
        n += printf("."); 
    }

    // Extract digits from the remainder one at a time
    while (digits-- > 0)
    {
    remainder *= 10.0;
    unsigned int toPrint = (unsigned int)remainder;
    n += printf("%u",toPrint);
    remainder -= toPrint; 
    } 
  
  return n;








}



