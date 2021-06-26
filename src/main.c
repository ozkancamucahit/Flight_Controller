/*
 * GccApplicationCEXE.c
 *
 * Created: 21/12/2020 01:32:58
 * Author : mmuca
 */ 

#include  "Helper.h" 

#define OFFSET_SAMPLES 200U

void configure_Console(void)
{
    const usart_rs232_options_t uart_serial_options =
    {
        .baudrate = 115200U,
        .paritytype = CONF_UART_PARITY,
        .charlength = US_MR_CHRL_8_BIT,
        .stopbits = US_MR_NBSTOP_1_BIT
    };
	// pio_configure_pin_group(PINS_UART_PIO, PINS_UART, PINS_UART_FLAGS);
    // pmc_enable_periph_clk(ID_UART);
    // uart_init(UART, &params);
    stdio_serial_init( CONF_UART, &uart_serial_options);
}


static uint32_t gs_ul_captured_pulses;
static uint32_t gs_ul_captured_ra;
static uint32_t gs_ul_captured_rb;

/** Current wave configuration*/
uint8_t gs_uc_configuration = 0;
   /** TC waveform configurations */
static const waveconfig_t waveconfig[] = {
	{TC_CMR_TCCLKS_TIMER_CLOCK4, 50, 7}, // current wave config
	{TC_CMR_TCCLKS_TIMER_CLOCK3, 50, 2},
	{TC_CMR_TCCLKS_TIMER_CLOCK3, 50, 12},
	{TC_CMR_TCCLKS_TIMER_CLOCK2, 1000, 80},
	{TC_CMR_TCCLKS_TIMER_CLOCK2, 4000, 50}
    };

/** Number of available wave configurations 5*/
const uint8_t gc_uc_nbconfig = sizeof(waveconfig)
		/ sizeof(waveconfig_t);

static void display_menu(void)
{
	uint8_t i;
	puts("\n\rMenu :\n\r"
			"------\n\r"
			"  Output waveform property:\r");
	for (i = 0; i < gc_uc_nbconfig; i++) {
		printf("  %d: Set Frequency = %4u Hz, Duty Cycle = %2u%%\n\r", i,
				(unsigned int)waveconfig[i].us_frequency,
				(unsigned int)waveconfig[i].us_dutycycle);
	}
	printf("  -------------------------------------------\n\r"
			"  c: Capture waveform from TC%d channel %d\n\r"
			"  s: Stop capture and display captured information \n\r"
			"  h: Display menu \n\r"
			"------\n\r\r", TC_PERIPHERAL,TC_CHANNEL_CAPTURE0);
}

void TC0_Handler(void)
{
	if ((tc_get_status(TC, TC_CHANNEL_CAPTURE0) & TC_SR_LDRBS) 
    == TC_SR_LDRBS) 
    {
		
		gs_ul_captured_pulses++;
		
		gs_ul_captured_ra = tc_read_ra(TC, TC_CHANNEL_CAPTURE0);
		
		
		gs_ul_captured_rb = tc_read_rb(TC, TC_CHANNEL_CAPTURE0);
		
	}

}

void SysTick_Handler(void)
{
	TimeTick_Increment();
}

int lol(void)
{
    
    SystemInit();
	WDT->WDT_MR = WDT_MR_WDDIS;
	float pi = 22.0F/7.0F;
	int ipi = pi;
	pio_configure_pin_group (PINS_UART_PIO, PINS_UART, PINS_UART_FLAGS);
    
    configure_Console(); /* < UART config */

	//printf("wd disabled\n\r");
	//printf("PI after test dot: %.5f\n", pi);
	// printf("iPI before all: %d\n\r", ipi);
	// printf("PI with print float ");
	// print_Float(pi,5);
	// printf("\n");
	
    pmc_enable_periph_clk(ID_PIOB);
    pio_set_output(PIOB, PIO_PB27, LOW, DISABLE, NO_PULLUP);
	//pio_configure_pin(PIO_PB27_IDX, PIO_TYPE_PIO_OUTPUT_1 | PIO_DEFAULT );
	//printf("led configured\n\r");
	
	NVIC_DisableIRQ(TWI1_IRQn);
	NVIC_ClearPendingIRQ(TWI1_IRQn);
	NVIC_SetPriority(TWI1_IRQn, 0);
	NVIC_EnableIRQ(TWI1_IRQn);

	//printf("TWI NVIC disabled\n\r");

	
    pmc_enable_periph_clk(ID_TWI1);
	pio_configure_pin(TWI1_DATA_GPIO, TWI1_DATA_FLAGS);
    pio_configure_pin(TWI1_CLK_GPIO, TWI1_CLK_FLAGS);

	//printf("TWI configured\n\r");


	// disable pdc
	//TWI1->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
    
    // twi_options_t opt;
	// opt.smbus = 0;
    // opt.chip = MPU6050_ADDRESS_AD0_LOW;
    // opt.speed = 100000UL;
    // opt.master_clk = F_CPU;

    // if ( twi_master_init(TWI1, &opt) == TWI_SUCCESS ) printf("Master init successfull\n");
	// else printf("Master init FAILED\n");
    pio_set_pin_low(PIO_PB27_IDX);
	
    //uart_write(UART, '1');

    //unsigned char* s = "HEyoo mucahit\n\r"; 
    //size_t length = strlen(s);
	//usart_serial_write_packet((Usart*)UART, (uint8_t*)s, length);

    
    //printf("selam canim %d\n\r", 17);
    //display_menu();

    pio_configure_pin(PIN_TC_WAVEFORM, PIN_TC0_TIOA1_FLAGS);
    disable_pin(PIN_TC_WAVEFORM);

    // TODO: disable pin
    pio_configure_pin(PIN_TC_CAPTURE, PIN_TC0_TIOA0_FLAGS);
    disable_pin(PIN_TC_CAPTURE);

    /* Configure TC TC_CHANNEL_WAVEFORM as waveform operating mode */
	//printf("Configure TC%d channel %d as waveform operating mode \n\r",
			//TC_PERIPHERAL, TC_CHANNEL_WAVEFORM1);
	//! [tc_waveform_init_call]
	//tc_waveform_initialize(ID_TC0, waveconfig, gs_uc_configuration);
	//! [tc_waveform_init_call]
        
	/* Configure TC TC_CHANNEL_CAPTURE as capture operating mode */
	//printf("Configure TC%d channel %d as capture operating mode \n\r",
			//TC_PERIPHERAL, TC_CHANNEL_CAPTURE0);
	//! [tc_capture_init_call]
	tc_capture_initialize(ID_TC0);
	//! [tc_capture_init_call]

	//! [tc_capture_init_irq]
	/** Configure TC interrupts for TC TC_CHANNEL_CAPTURE only */
	// NVIC_DisableIRQ(TC_IRQn);
	// NVIC_ClearPendingIRQ(TC_IRQn);
	// NVIC_SetPriority(TC_IRQn, 0);
	// NVIC_EnableIRQ(TC_IRQn);

    uint8_t key;
	uint16_t frequence, dutycycle;

    // while (1) 
    // {
	// 	scanf("%c", (char *)&key);

	// 	switch (key) 
    //     {
	// 	case 'h':
	// 		display_menu();
	// 		break;

	// 	case 's':
	// 		if (gs_ul_captured_pulses) {
	// 			tc_disable_interrupt(TC, TC_CHANNEL_CAPTURE0, TC_IDR_LDRBS);
	// 			printf("Captured %u pulses from TC%d channel %d, RA = %u, RB = %u \n\r",
	// 					(unsigned)gs_ul_captured_pulses, TC_PERIPHERAL,
	// 					TC_CHANNEL_CAPTURE0,	(unsigned)gs_ul_captured_ra,
	// 					(unsigned)gs_ul_captured_rb);
	// 			frequence = (F_CPU /
	// 					divisors[TC_CAPTURE_TIMER_SELECTION]) /
	// 					gs_ul_captured_rb;
	// 			dutycycle
	// 				= (gs_ul_captured_rb - gs_ul_captured_ra) * 100 /
	// 					gs_ul_captured_rb;
	// 			printf("Captured wave frequency = %d Hz, Duty cycle = %d%% \n\r",
	// 					frequence, dutycycle);

	// 			gs_ul_captured_pulses = 0;
	// 			gs_ul_captured_ra = 0;
	// 			gs_ul_captured_rb = 0;
	// 		}
	// 		else {
	// 			puts("No waveform has been captured\r");
	// 		}

	// 		puts("\n\rPress 'h' to display menu\r");
	// 		break;

	// 	case 'c':
	// 		puts("Start capture, press 's' to stop \r");
			
	 		tc_enable_interrupt(TC, TC_CHANNEL_CAPTURE0, TC_IER_LDRBS);
	 		tc_start(TC, TC_CHANNEL_CAPTURE0);
			
	// 		break;

	// 	default:
	// 		/* Set waveform configuration #n */
	// 		if ((key >= '0') && (key <= ('0' + gc_uc_nbconfig - 1))) 
    //         { // 0-4 araligi
    //             printf("Tercih : %d\r\n", key);
	// 			if (!gs_ul_captured_pulses) {
	// 				gs_uc_configuration = key - '0';
	// 				tc_waveform_initialize(ID_TC0,
    //                 waveconfig, gs_uc_configuration);
	// 			} else {
	// 				puts("Capturing ... , press 's' to stop capture first \r");
	// 			}
	// 		}
    //         break;
	// 	}
	// }

	
    MPU_data_t mpu_data;
	//printf("setting up mpu6050...\n\r");
	delay_ms(10);
    setupMPU(TWI1);
	delay_ms(10);
	
	

    uint8_t dataREAD=17;
    twi_packet_t package = 
    {
        .iaddr[0] = MPU6050_RA_WHO_AM_I,
        .iaddr_length = 1,
        .buffer = &dataREAD, 
        .chip_addr = MPU6050_DEFAULT_ADDRESS,
        .length = 1
    };
	
	// uint8_t write_sequence []= {MPU6050_RA_WHO_AM_I};

	// package.iaddr_length = 0UL;
	// package.buffer = write_sequence;
	// package.length = 1UL;

	// if( twi_master_write(TWI1, &package) == TWI_SUCCESS ) printf("Master write success\n\r");
	// else printf("Master write FAIL\n\r");
	// package.buffer = &dataREAD;
	// package.length = 1UL;

	// if ( twi_master_read(TWI1, &package) == TWI_SUCCESS ) printf("Master read success\n\r");
	// else printf("Master read FAIL\n\r");

	//printf("DAta read : %u\n\r", dataREAD);
	//printf("PI with print float ");
	//print_Float(pi,5);
	//printf("\n");
	//test_Connection(TWI1);

    // while (1)
    //  { 
	// 	 if ( twi_master_read(TWI1, &package) == TWI_SUCCESS)
	// 		printf("READ : %d\n", dataREAD);
	// 	else
	// 		printf("FAIL reading WHO AM I\n");
	// }
    
    //  if ( twi_probe(TWI1, opt.chip) == TWI_SUCCESS )
    //     {   
    //         printf("PROBE SUCCESS\n");
    //         pio_set_pin_high(PIO_PB27_IDX);
    //         for (int i =0; i< 5500000; i++);
    //         pio_set_pin_low(PIO_PB27_IDX);
    //         for (int i =0; i< 5500000; i++);
    //         pio_set_pin_high(PIO_PB27_IDX);
    //         for (int i =0; i< 5500000; i++);
    //         pio_set_pin_low(PIO_PB27_IDX);
    //         for (int i =0; i< 5500000; i++);
			
    //     }
    // else {
    //         pio_set_pin_high(PIO_PB27_IDX);
    //         for (int i =0; i< 5500000; i++);
    //         pio_set_pin_low(PIO_PB27_IDX);
    //         for (int i =0; i< 5500000; i++);
    // }
	

	SysTick_Config(F_CPU / 1000UL);

	calculate_offset(TWI1,&mpu_data, OFFSET_SAMPLES);
	// AccERROR X: 0.538004	AccERROR Y: -1.709946	GyroERROR X: -2.885879	GyroERROR Y: 1.171870	GyroERROR Z: 0.669924
	delay_s(3);

	float currentTime = 0.0f, previousTime, elapsedTime;
	float roll, pitch, yaw;

     
    while (1)
     {


	    recordAccelRegisters(&mpu_data, TWI1);

		previousTime = currentTime;
		currentTime = millis();
		elapsedTime = (currentTime - previousTime) / 1000U;

	    recordGyroRegisters(&mpu_data, TWI1);

		
		//printf("Elapsed time: %lu\n", elapsedTime);
		calculate_roll_pitch_yaw( &mpu_data, elapsedTime);
	    //printData(&mpu_data);
		
		 //if (elapsedTime % 5 == 0 || elapsedTime % 7 == 0) print_Float(elapsedTime, PRINT_SIZE);
		 //printf("Time : ");
		 //uint32_t time = millis();
		 //printf("%lu\n", time);
		 delay_ms(10);

		
	    
     }

}
    
