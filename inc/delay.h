/*
 * delay.h
 *
 * Created: 24/12/2020 13:22:24
 *  Author: mmuca
 */ 


#ifndef DELAY_H_
#define DELAY_H_

#ifndef F_CPU
#       define F_CPU 84000000UL
#endif

/**
 * \brief Delay loop to delay n number of cycles
 * \param n Number of cycles
 */
void portable_delay_cycles(unsigned long n);


#  define cpu_ms_2_cy(ms, f_cpu)  \
	(((uint64_t)(ms) * (f_cpu) + (uint64_t)(14e3 - 1ul)) / (uint64_t)14e3)
#  define cpu_us_2_cy(us, f_cpu)  \
	(((uint64_t)(us) * (f_cpu) + (uint64_t)(14e6 - 1ul)) / (uint64_t)14e6)


#define delay_cycles               portable_delay_cycles

#define cpu_delay_ms(delay, f_cpu) delay_cycles(cpu_ms_2_cy(delay, f_cpu))
#define cpu_delay_us(delay, f_cpu) delay_cycles(cpu_us_2_cy(delay, f_cpu))

/**
 * @def delay_s
 * @brief Delay in seconds.
 * @param delay Delay in seconds
 */
#define delay_s(delay)      ((delay) ? cpu_delay_ms(1000 * delay, F_CPU) : cpu_delay_us(1, F_CPU))

/**
 * @def delay_ms
 * @brief Delay in milliseconds.
 * @param delay Delay in milliseconds
 */
#define delay_ms(delay)     ((delay) ? cpu_delay_ms(delay, F_CPU) : cpu_delay_us(1, F_CPU))

/**
 * @def delay_us
 * @brief Delay in microseconds.
 * @param delay Delay in microseconds
 */
#define delay_us(delay)     ((delay) ? cpu_delay_us(delay, F_CPU) : cpu_delay_us(1, F_CPU))



#endif /* DELAY_H_ */