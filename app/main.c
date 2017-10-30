#include <stdio.h>
#include <rtt_stdio.h>
#include "xtimer.h"
#include <string.h>
#include "net/gnrc/udp.h"
#include "phydat.h"
#include "saul_reg.h"
#include "periph/adc.h"
#include "periph/i2c.h"
#include "periph/spi.h"
//#include "periph/dmac.h"
#include "periph/timer.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#ifndef SAMPLE_INTERVAL
#define SAMPLE_INTERVAL (20000000UL)
#endif
#define SAMPLE_JITTER    (2000000UL)

#define TYPE_FIELD 8


void send_udp(char *addr_str, uint16_t port, uint8_t *data, uint16_t datalen);

#define AES_SKIP_START_BYTES 4

typedef struct __attribute__((packed,aligned(4))) {
  uint16_t type;
  uint16_t serial;
  //From below is encrypted
  //We use a zero IV, so it is important that the first AES block
  //is completely unique, which is why we include uptime.
  //It is expected that hamilton nodes never reboot and that
  //uptime is strictly monotonic
  uint64_t uptime;
  uint16_t flags; //which of the fields below exist
  int16_t  acc_x;
  int16_t  acc_y;
  int16_t  acc_z;
  int16_t  mag_x;
  int16_t  mag_y;
  int16_t  mag_z;
  int16_t  radtemp;
  int16_t  temp;
  int16_t  hum;
  int16_t  light_lux;
  uint16_t buttons;
  uint16_t occup;
  uint32_t reserved1;
  uint32_t reserved2;
  uint32_t reserved3;
} ham7c_t;

saul_reg_t *sensor_radtemp_t = NULL;
saul_reg_t *sensor_temp_t    = NULL;
saul_reg_t *sensor_hum_t     = NULL;
saul_reg_t *sensor_mag_t     = NULL;
saul_reg_t *sensor_accel_t   = NULL;
saul_reg_t *sensor_light_t   = NULL;
saul_reg_t *sensor_occup_t   = NULL;
saul_reg_t *sensor_button_t  = NULL;

//Uptime is mandatory (for AES security)
#define FLAG_ACC    (1<<0)
#define FLAG_MAG    (1<<1)
#define FLAG_TMP    (1<<2)
#define FLAG_HDC    (1<<3)
#define FLAG_LUX     (1<<4)
#define FLAG_BUTTONS  (1<<5)
#define FLAG_OCCUP    (1<<6)

#if defined(MODEL_3C)
#define PROVIDED_FLAGS (0x3F)
#elif defined(MODEL_7C)
#define PROVIDED_FLAGS (0x7F)
#else
#error "No model defined"
#endif

void critical_error(void) {
    DEBUG("CRITICAL ERROR, REBOOT\n");
    NVIC_SystemReset();
    return;
}

void sensor_config(void) {
    sensor_radtemp_t = saul_reg_find_type(SAUL_SENSE_RADTEMP);
    if (sensor_radtemp_t == NULL) {
        DEBUG("[ERROR] Failed to init RADTEMP sensor\n");
        critical_error();
    } else {
        DEBUG("TEMP sensor OK\n");
    }

    sensor_hum_t     = saul_reg_find_type(SAUL_SENSE_HUM);
    if (sensor_hum_t == NULL) {
        DEBUG("[ERROR] Failed to init HUM sensor\n");
        critical_error();
    } else {
        DEBUG("HUM sensor OK\n");
    }

    sensor_temp_t    = saul_reg_find_type(SAUL_SENSE_TEMP);
    if (sensor_temp_t == NULL) {
		DEBUG("[ERROR] Failed to init TEMP sensor\n");
		critical_error();
	} else {
		DEBUG("TEMP sensor OK\n");
	}

    sensor_mag_t     = saul_reg_find_type(SAUL_SENSE_MAG);
    if (sensor_mag_t == NULL) {
		DEBUG("[ERROR] Failed to init MAGNETIC sensor\n");
		critical_error();
	} else {
		DEBUG("MAGNETIC sensor OK\n");
	}

    sensor_accel_t   = saul_reg_find_type(SAUL_SENSE_ACCEL);
    if (sensor_accel_t == NULL) {
		DEBUG("[ERROR] Failed to init ACCEL sensor\n");
		critical_error();
	} else {
		DEBUG("ACCEL sensor OK\n");
	}

    sensor_light_t   = saul_reg_find_type(SAUL_SENSE_LIGHT);
	if (sensor_light_t == NULL) {
		DEBUG("[ERROR] Failed to init LIGHT sensor\n");
		critical_error();
	} else {
		DEBUG("LIGHT sensor OK\n");
	}

    sensor_occup_t   = saul_reg_find_type(SAUL_SENSE_OCCUP);
	if (sensor_occup_t == NULL) {
		DEBUG("[ERROR] Failed to init OCCUP sensor\n");
		critical_error();
	} else {
		DEBUG("OCCUP sensor OK\n");
	}

    sensor_button_t  = saul_reg_find_type(SAUL_SENSE_COUNT);
    if (sensor_button_t == NULL) {
        DEBUG("[ERROR] Failed to init BUTTON sensor\n");
        critical_error();
    } else {
        DEBUG("BUTTON sensor OK\n");
    }
}

/* ToDo: Sampling sequence arrangement or thread/interrupt based sensing may be better */
void sample(ham7c_t *m) {
    phydat_t output; /* Sensor output data (maximum 3-dimension)*/
	   int dim;         /* Demension of sensor output */

    /* Occupancy 1-dim */
    #if ((PROVIDED_FLAGS & FLAG_OCCUP) != 0)
    dim = saul_reg_read(sensor_occup_t, &output);
    if (dim > 0) {
        m->occup = output.val[0];
        // printf("\nDev: %s\tType: %s\n", sensor_occup_t->name,
        //         saul_class_to_str(sensor_occup_t->driver->type));
        // phydat_dump(&output, dim);
    } else {
        DEBUG("[ERROR] Failed to read Occupancy\n");
    }
    #endif

    /* Push button events 1-dim */
    dim = saul_reg_read(sensor_button_t, &output);
    if (dim > 0) {
        m->buttons = output.val[0];
        // printf("\nDev: %s\tType: %s\n", sensor_button_t->name,
        //         saul_class_to_str(sensor_button_t->driver->type));
        //phydat_dump(&output, dim);
    } else {
        DEBUG("[ERROR] Failed to read button events\n");
    }

    /* Illumination 1-dim */
	dim = saul_reg_read(sensor_light_t, &output);
	if (dim > 0) {
		m->light_lux = output.val[0];
		// printf("\nDev: %s\tType: %s\n", sensor_light_t->name,
		// 				saul_class_to_str(sensor_light_t->driver->type));
		//phydat_dump(&output, dim);
	} else {
		DEBUG("[ERROR] Failed to read Illumination\n");
	}

    /* Magnetic field 3-dim */
    dim = saul_reg_read(sensor_mag_t, &output);
    if (dim > 0) {
        m->mag_x = output.val[0]; m->mag_y = output.val[1]; m->mag_z = output.val[2];
        // printf("\nDev: %s\tType: %s\n", sensor_mag_t->name,
        //         saul_class_to_str(sensor_mag_t->driver->type));
        //phydat_dump(&output, dim);
    } else {
        DEBUG("[ERROR] Failed to read magnetic field\n");
    }

    /* Acceleration 3-dim */
    dim = saul_reg_read(sensor_accel_t, &output);
    if (dim > 0) {
        m->acc_x = output.val[0]; m->acc_y = output.val[1]; m->acc_z = output.val[2];
        // printf("\nDev: %s\tType: %s\n", sensor_accel_t->name,
        //         saul_class_to_str(sensor_accel_t->driver->type));
        //phydat_dump(&output, dim);
    } else {
        printf("[ERROR] Failed to read Acceleration\n");
    }

    /* Radient temperature 1-dim */
    dim = saul_reg_read(sensor_radtemp_t, &output); /* 500ms */
    if (dim > 0) {
        m->temp = output.val[0];
        m->radtemp = output.val[1];
        // printf("\nDev: %s\tType: %s\n", sensor_radtemp_t->name,
        //         saul_class_to_str(sensor_radtemp_t->driver->type));
        //phydat_dump(&output, dim);
    } else {
        DEBUG("[ERROR] Failed to read Radient Temperature\n");
    }

    /* Temperature 1-dim */
    dim = saul_reg_read(sensor_temp_t, &output); /* 15ms */
    if (dim > 0) {
        m->temp = output.val[0];
        // printf("\nDev: %s\tType: %s\n", sensor_temp_t->name,
        //         saul_class_to_str(sensor_temp_t->driver->type));
        //phydat_dump(&output, dim);
    } else {
        DEBUG("[ERROR] Failed to read Temperature\n");
    }

    /* Humidity 1-dim */
    LED_ON;
    dim = saul_reg_read(sensor_hum_t, &output); /* 15ms */
    if (dim > 0) {
        m->hum = output.val[0];
        // printf("\nDev: %s\tType: %s\n", sensor_hum_t->name,
        //         saul_class_to_str(sensor_hum_t->driver->type));
        //phydat_dump(&output, dim);
    } else {
        DEBUG("[ERROR] Failed to read Humidity\n");
    }
    LED_OFF;

    /* Time from start */
    m->uptime = xtimer_usec_from_ticks64(xtimer_now64());

    /* Others */
    m->serial = *fb_device_id;
    m->type   = TYPE_FIELD;
    m->flags  = PROVIDED_FLAGS;

    //puts("\n##########################");
}

uint32_t interval_with_jitter(void)
{
    int32_t t = SAMPLE_INTERVAL;
    t += rand() % SAMPLE_JITTER;
    t -= (SAMPLE_JITTER >> 1);
    return (uint32_t)t;
}

ham7c_t frontbuf;

uint8_t obuffer [sizeof(ham7c_t)];
uint8_t iv [16];

#include "crypto/ciphers.h"
#include "crypto/modes/cbc.h"
cipher_t aesc;

void crypto_init(void) {
	//While this appears absurd, don't worry too much about it.
	//The first block is guaranteed to be unique so we don't really
	//need the IV
	for (int i = 0; i < 16; i++) {
		iv[i] = i;
	}
	int rv = cipher_init(&aesc, CIPHER_AES_128, fb_aes128_key, 16);
	if (rv != CIPHER_INIT_SUCCESS) {
		DEBUG("[ERROR] Failed to init Cipher\n");
		critical_error();
	}
}
void aes_populate(void) {
	cipher_encrypt_cbc(&aesc, iv, ((uint8_t*)&frontbuf) + AES_SKIP_START_BYTES,
					 sizeof(ham7c_t)-AES_SKIP_START_BYTES, &obuffer[AES_SKIP_START_BYTES]);
	memcpy(obuffer, ((uint8_t*)&frontbuf), AES_SKIP_START_BYTES);
}


int main(void) {
    //This value is good randomness and unique per mote
    srand(*((uint32_t*)fb_aes128_key));
    crypto_init();
    sensor_config();

    /* Enable dma */
#if DMAC_ENABLE
    dmac_init();
    adc_set_dma_channel(DMAC_CHANNEL_ADC);
    i2c_set_dma_channel(I2C_0,DMAC_CHANNEL_I2C);
    spi_set_dma_channel(0,DMAC_CHANNEL_SPI_TX,DMAC_CHANNEL_SPI_RX);
#endif
	  LED_OFF;

    while (1) {
		//Sample
	    sample(&frontbuf);
#if CLOCK_USE_ADAPTIVE
        sysclk_change(true);
#endif
		aes_populate();
		//Send
		send_udp("ff02::1",4747,obuffer,sizeof(obuffer));
#if CLOCK_USE_ADAPTIVE
        sysclk_change(false);
#endif
		//Sleep
		xtimer_usleep(interval_with_jitter());
    }

    return 0;
}
