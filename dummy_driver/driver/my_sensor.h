#ifndef MY_SENSOR_H_
#define MY_SENSOR_H_

#include <stdint.h>
#include <zephyr/kernel.h>

/* Configuration constante (provenant du DTS) */
struct my_sensor_config
{
	int32_t initial_mc;
	int32_t maximum_mc;
};

/* Etat propre a chaque instance (mutable) */
struct my_sensor_data
{
	int32_t next_mc;
};

/*
 * Format interne du buffer encode :
 *
 * Octets 0..7  : timestamp en nanosecondes, little-endian.
 * Octets 8..11 : temperature en millicelsius, little-endian.
 */
#define MY_SENSOR_FRAME_SIZE 12

/*
 * Echelle du resultat Q31 :
 * temperature en degres = valeur_q31 * 2^(shift - 31).
 *
 * shift = 7 couvre les temperatures de ce TP : 0 a 100 degres.
 *
 * Q31 avec shift 7: sxxx xxxx yyyy yyyy yyyy yyyy yyyy yyyy
 * 1 bit de signe
 * 7 bits pour la partie entière
 * 24 pour la partie après la virgule
 *
 * 0000 1000 0000 0000 0000 0000 1000 0010
 * s = 0
 * x = 8
 * y = 130 / 2^24 = ,0000077
 * donc 8,0000077
 *
 *
 * pour transformer n en y : n * 2^24. pour n = 50:
 * 50 * 2^24 = 838 860 800 = 0x32000000
 * 0011 0010 0000 0000 0000 0000 0000 0000
 * s = 0
 * x = 011 0010 = 50
 * y = 0
 *
 * pour t = 50,5 :
 * 50,5 * 2^24 = 847 249 408 = 0x32800000
 * 0011 0010 1000 0000 0000 0000 0000 0000
 * s = 0
 * x = 50
 * y = 0x800000 / 2^24 = ,5
 */
#define MY_SENSOR_Q31_SHIFT 7

#endif