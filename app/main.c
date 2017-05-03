#include <stdio.h>
#include <rtt_stdio.h>
#include "thread.h"
#include "xtimer.h"
#include <string.h>

#include "msg.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/netreg.h"

#include <tmp006.h>
#include <hdc1000.h>
#include <hdc1000_params.h>
#include <fxos8700.h>

#include <periph/gpio.h>
#include <periph/i2c.h>
#include <periph/adc.h>

//#define SAMPLE_INTERVAL ( 5000000UL)
#ifndef SAMPLE_INTERVAL
#define SAMPLE_INTERVAL ( 10000000UL)
#endif
#define SAMPLE_JITTER   ( 2000000UL)

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
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
  uint16_t tmp_die;
  uint16_t tmp_val;
  int16_t hdc_tmp;
  int16_t hdc_hum;
  uint16_t light_lux;
  uint16_t buttons;
  uint16_t occup;
  uint32_t reserved1;
  uint32_t reserved2;
  uint32_t reserved3;
} ham7c_t;


//Uptime is mandatory (for AES security)
#define FLAG_ACC    (1<<0)
#define FLAG_MAG    (1<<1)
#define FLAG_TMP    (1<<2)
#define FLAG_HDC    (1<<3)
#define FLAG_LUX     (1<<4)
#define FLAG_BUTTONS  (1<<5)
#define FLAG_OCCUP    (1<<6)

//All of the flags
#ifndef HAMILTON_3C_ONLY
#define PROVIDED_FLAGS (0x7F)
#else
#define PROVIDED_FLAGS (0x3F)
#endif

//It's actually 6.5*2ms but lets give it 15ms to account for oscillator etc
#define HDC_ACQUIRE_TIME (15000UL)
//#define HDC_ACQUIRE_TIME (40000UL)

//It's actually 500ms, but lets give it 10% extra
#define TMP_ACQUIRE_TIME (550000UL)

//We start the TMP before
#define EMPIRICAL_ACQUISITION_DELAY (22000UL)
#define TMP_OFFSET_ACQUIRE_TIME (TMP_ACQUIRE_TIME-EMPIRICAL_ACQUISITION_DELAY)

tmp006_t tmp006;
hdc1000_t hdc1080;
fxos8700_t fxos8700;

uint16_t occupancy_events;
uint16_t button_events;

bool pir_high;
uint64_t pir_rise_time;
uint32_t acc_pir_time;
uint64_t last_pir_reset;

void dutycycling_init(void) {
  /* Leaf nodes: Low power operation and auto duty-cycling */
  kernel_pid_t radio[GNRC_NETIF_NUMOF];
  uint8_t radio_num = gnrc_netif_get(radio);
  netopt_enable_t dutycycling;
  if (DUTYCYCLE_SLEEP_INTERVAL) {
    dutycycling = NETOPT_ENABLE;
  } else {
    dutycycling = NETOPT_DISABLE;
  }
  for (int i=0; i < radio_num; i++)
    gnrc_netapi_set(radio[i], NETOPT_DUTYCYCLE, 0, &dutycycling, sizeof(netopt_t));
}

void critical_error(void) {
  printf("CRITICAL ERROR, REBOOT\n");
  NVIC_SystemReset();
  return;
}

void on_pir_trig(void* arg) {
  int pin_now = gpio_read(GPIO_PIN(0, 6));

  //We were busy counting
  if (pir_high) {
    //Add into accumulation
    uint64_t now = xtimer_usec_from_ticks64(xtimer_now64());
    uint32_t delta = (uint32_t)(now - pir_rise_time);
    acc_pir_time += delta;
  }
  //Pin is rising
  if (pin_now) {
    pir_rise_time = xtimer_usec_from_ticks64(xtimer_now64());
    pir_high = true;
  } else {
    pir_high = false;
  }
}
void on_button_trig(void* arg) {
  button_events ++;
}

hdc1000_params_t hdcp = {
  I2C_0,
  0x40,
  HDC1000_14BIT,
};


void low_power_init(void) {
    // Light sensor off
    gpio_init(GPIO_PIN(0,28), GPIO_OUT);
    gpio_init(GPIO_PIN(0,19), GPIO_OUT);
    gpio_write(GPIO_PIN(0, 28), 1);
    gpio_write(GPIO_PIN(0, 19), 0);
    int rv;

    rv = fxos8700_init(&fxos8700, I2C_0, 0x1e);
    if (rv != 0) {
      printf("failed to initialize FXO on %d\n", rv);
      critical_error();
      return;
    }

    //Init PIR accounting
    pir_high = false;
    last_pir_reset = xtimer_usec_from_ticks64(xtimer_now64());
    acc_pir_time = 0;
    button_events = 0;
    rv = hdc1000_init(&hdc1080, &hdcp);
    if (rv != 0) {
      printf("failed to initialize HDC1080 %d\n", rv);
      critical_error();
      return;
    }

    rv = tmp006_init(&tmp006, I2C_0, 0x44, TMP006_CONFIG_CR_AS2);
    if (rv != 0) {
      printf("failed to initialize TMP006\n");
      critical_error();
      return;
    } else {
      printf("TMP006 init ok\n");
    }
    rv = tmp006_test(&tmp006);
    if (rv != 0) {
      printf("tmp006 failed self test\n");
      critical_error();
      return;
    } else {
      printf("TMP006 self test ok\n");
    }
    rv = tmp006_set_standby(&tmp006);
    if (rv != 0) {
      printf("failed to standby TMP006\n");
      critical_error();
      return;
    }

    gpio_init_int(GPIO_PIN(PA, 18), GPIO_IN_PU, GPIO_FALLING, on_button_trig, 0);
    #ifndef HAMILTON_3C_ONLY
    //gpio_init(GPIO_PIN(PA, 6), GPIO_IN_PD);
    gpio_init_int(GPIO_PIN(PA, 6), GPIO_IN_PD, GPIO_BOTH, on_pir_trig, 0);
    #endif
    adc_init(ADC_PIN_PA08);

}

void sample(ham7c_t *m) {
    uint8_t drdy;

    /* turn on light sensor and let it stabilize */
    gpio_write(GPIO_PIN(0, 28), 0);

    /* start the TMP acquisition */
    if (tmp006_set_active(&tmp006)) {
        printf("failed to active TMP006\n");
        critical_error();
        return;
    }

    /* start the HDC acquisition */
    hdc1000_trigger_conversion(&hdc1080);

    /* turn on LED */
    gpio_write(GPIO_PIN(0, 19), 1);

    /* wait for HDC */
    xtimer_usleep(HDC_ACQUIRE_TIME);

    /* turn off LED */
    gpio_write(GPIO_PIN(0, 19), 0);

    hdc1000_get_results(&hdc1080, &m->hdc_tmp, &m->hdc_hum);
    m->light_lux = (int16_t) adc_sample(ADC_PIN_PA08, ADC_RES_16BIT);

    /* Turn off light sensor */
    gpio_write(GPIO_PIN(0, 28), 1);

    /* sample accel/mag */
    fxos8700_set_active(&fxos8700);
    fxos8700_measurement_t fm;
    if (fxos8700_read(&fxos8700, &fm)) {
      printf("failed to sample fxos8700\n");
      critical_error();
      return;
    }
    fxos8700_set_idle(&fxos8700);

    xtimer_usleep(TMP_OFFSET_ACQUIRE_TIME);
    if (tmp006_read(&tmp006, (int16_t*)&m->tmp_val, (int16_t*)&m->tmp_die, &drdy)) {
        printf("failed to sample TMP %d\n", drdy);
        critical_error();
        return;
    }
    if (tmp006_set_standby(&tmp006)) {
        printf("failed to standby the TMP\n");
        critical_error();
        return;
    }

    m->serial = *fb_device_id;
    m->acc_x = fm.acc_x;
    m->acc_y = fm.acc_y;
    m->acc_z = fm.acc_z;
    m->mag_x = fm.mag_x;
    m->mag_y = fm.mag_y;
    m->mag_z = fm.mag_z;
    m->uptime = xtimer_usec_from_ticks64(xtimer_now64());
    #ifndef HAMILTON_3C_ONLY
    m->occup = (uint16_t)(((uint64_t) acc_pir_time * 32767) / (m->uptime - last_pir_reset));
    last_pir_reset = m->uptime;
    acc_pir_time = 0;
    #else
    m->occup = 0;
    #endif
    m->type = TYPE_FIELD;
    m->flags = PROVIDED_FLAGS;
    m->buttons = button_events;

    // int32_t t = (((int32_t)m->tmp_die)*3125)/4000;
    // printf("drdy %d\n", drdy);
    // printf("sampled lux: %d\n", (int)m->light_lux);
    // printf("sampled temp ok hdct=%d hdch=%d \n", (int)m->hdc_tmp, (int)m->hdc_hum);
    // printf("sampled rad ok tdie=%d tval=%d \n", (int)t, (int)m->tmp_val);
    // printf("sampled PIR %d\n", (int)m->occup);
    // printf("buttone %d\n", (int)m->buttons);
    // printf("sampled acc x=%d y=%d z=%d\n", (int)m->acc_x, (int)m->acc_y, (int)m->acc_z);
    // printf("sampled mag x=%d y=%d z=%d\n", (int)m->mag_x, (int)m->mag_y, (int)m->mag_z);
}

uint32_t interval_with_jitter(void)
{
    int32_t t = SAMPLE_INTERVAL;
    t += rand() % SAMPLE_JITTER;
    t -= SAMPLE_JITTER / 2;
    return (uint32_t)t;
}

ham7c_t frontbuf;

uint8_t obuffer [sizeof(ham7c_t)];
uint8_t iv [16];

#include "crypto/ciphers.h"
#include "crypto/modes/cbc.h"
cipher_t aesc;

void crypto_init(void){
  //While this appears absurd, don't worry too much about it.
  //The first block is guaranteed to be unique so we don't really
  //need the IV
  for (int i = 0; i < 16; i++) {
    iv[i] = i;
  }
  //printf("us: %d\n", *fb_device_id);
  //printf("key: ");
  //for (int i = 0; i < 16; i++) {
  //  printf("%02x", fb_aes128_key[i]);
  //}
  //printf("\n");
  int rv = cipher_init(&aesc, CIPHER_AES_128, fb_aes128_key, 16);
  if (rv != CIPHER_INIT_SUCCESS) {
    printf("failed to init cipher\n");
    critical_error();
  }
}
void aes_populate(void) {
  cipher_encrypt_cbc(&aesc, iv, ((uint8_t*)&frontbuf) + AES_SKIP_START_BYTES, sizeof(ham7c_t)-AES_SKIP_START_BYTES, &obuffer[AES_SKIP_START_BYTES]);
  memcpy(obuffer, ((uint8_t*)&frontbuf), AES_SKIP_START_BYTES);
}
int main(void)
{
    dutycycling_init();
    //This value is good randomness and unique per mote
    srand(*((uint32_t*)fb_aes128_key));
    crypto_init();
    low_power_init();

    while (1) {
      //Sample
      sample(&frontbuf);
      aes_populate();
      //Send
      send_udp("ff02::1",4747,obuffer,sizeof(obuffer));

      //Sleep
      xtimer_usleep(interval_with_jitter());
    }

    return 0;
}
