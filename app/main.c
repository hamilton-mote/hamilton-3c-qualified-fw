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

// #define SAMPLE_INTERVAL ( 30000000UL)
#define SAMPLE_INTERVAL ( 10000000UL)
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
} ham7c_t;

//Uptime is mandatory (for AES security)
#define FLAG_ACC_X    (1<<0)
#define FLAG_ACC_Y    (1<<1)
#define FLAG_ACC_Z    (1<<2)
#define FLAG_MAG_X    (1<<3)
#define FLAG_MAG_Y    (1<<4)
#define FLAG_MAG_Z    (1<<5)
#define FLAG_TMP_DIE  (1<<6)
#define FLAG_TMP_VAL  (1<<7)
#define FLAG_HDC_TMP  (1<<8)
#define FLAG_HDC_HUM  (1<<9)
#define FLAG_LUX      (1<<10)
#define FLAG_BUTTONS  (1<<11)
#define FLAG_OCCUP    (1<<12)

//All of the flags
#define PROVIDED_FLAGS ((1<<13)-1)

//It's actually 6.5*2ms but lets give it 20ms to account for oscillator etc
#define HDC_ACQUIRE_TIME (20000UL)

//It's actually 1 second, but lets give it 10% extra
#define TMP_ACQUIRE_TIME (1100000UL)

tmp006_t tmp006;
hdc1000_t hdc1080;
fxos8700_t fxos8700;

uint16_t occupancy_events;
uint16_t button_events;

bool pir_high;
uint64_t pir_rise_time;
uint32_t acc_pir_time;
uint64_t last_pir_reset;

void critical_error(void) {
  printf("CRITICAL ERROR, REBOOT\n");
  return;
  NVIC_SystemReset();
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

    rv = tmp006_init(&tmp006, I2C_0, 0x44, TMP006_CONFIG_CR_AS4);
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
    gpio_init_int(GPIO_PIN(PA, 6), GPIO_IN, GPIO_BOTH, on_pir_trig, 0);
    adc_init(ADC_PIN_PA08);

}

void sample(ham7c_t *m) {
    uint8_t drdy;

    /* turn on light sensor and let it stabilize */
    gpio_write(GPIO_PIN(0, 28), 0);

    if (tmp006_set_active(&tmp006)) {
        printf("failed to active TMP006\n");
        critical_error();
        return;
    }
    hdc1000_trigger_conversion(&hdc1080);

    /* turn on LED */
    gpio_write(GPIO_PIN(0, 19), 1);

    xtimer_usleep(HDC_ACQUIRE_TIME);

    /* turn off LED */
    gpio_write(GPIO_PIN(0, 19), 0);

    hdc1000_read(&hdc1080, &m->hdc_tmp, &m->hdc_hum);
    m->light_lux = (int16_t) adc_sample(ADC_PIN_PA08, ADC_RES_16BIT);

    /* Turn off light sensor */
    gpio_write(GPIO_PIN(0, 28), 1);

    xtimer_usleep(TMP_ACQUIRE_TIME);
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

    fxos8700_set_active(&fxos8700);
    fxos8700_measurement_t fm;
    if (fxos8700_read(&fxos8700, &fm)) {
      printf("failed to sample fxos8700\n");
      critical_error();
      return;
    }
    fxos8700_set_idle(&fxos8700);
    m->acc_x = fm.acc_x;
    m->acc_y = fm.acc_y;
    m->acc_z = fm.acc_z;
    m->mag_x = fm.mag_x;
    m->mag_y = fm.mag_y;
    m->mag_z = fm.mag_z;
    m->uptime = xtimer_usec_from_ticks64(xtimer_now64());
    m->occup = ((uint64_t) acc_pir_time * 32768) / (m->uptime - last_pir_reset);
    last_pir_reset = m->uptime;
    acc_pir_time = 0;
    m->type = TYPE_FIELD;
    m->flags = PROVIDED_FLAGS;
    m->buttons = button_events;

    int32_t t = (((int32_t)m->tmp_die)*3125)/4000;
    printf("drdy %d\n", drdy);
    printf("sampled lux: %d\n", (int)m->light_lux);
    printf("sampled temp ok hdct=%d hdch=%d \n", (int)m->hdc_tmp, (int)m->hdc_hum);
    printf("sampled rad ok tdie=%d tval=%d \n", (int)t, (int)m->tmp_val);
    printf("sampled PIR %d\n", (int)m->occup);
    xtimer_usleep(100000);
    printf("buttone %d\n", (int)m->buttons);
    printf("sampled acc x=%d y=%d z=%d\n", (int)m->acc_x, (int)m->acc_y, (int)m->acc_z);
    printf("sampled mag x=%d y=%d z=%d\n", (int)m->mag_x, (int)m->mag_y, (int)m->mag_z);
}

uint32_t interval_with_jitter(void)
{
    int32_t t = SAMPLE_INTERVAL;
    t += rand() % SAMPLE_JITTER;
    t -= SAMPLE_JITTER / 2;
    return (uint32_t)t;
}

ham7c_t frontbuf;

int main(void)
{
    netopt_state_t radio_state = NETOPT_STATE_SLEEP;

    //This value is good randomness and unique per mote
    srand(*((uint32_t*)fb_aes128_key));
    low_power_init();
    kernel_pid_t radio[GNRC_NETIF_NUMOF];
    uint8_t radio_num = gnrc_netif_get(radio);

    while (1) {
      //Sample
      sample(&frontbuf);
      //Send
      send_udp("ff02::1",4747,(uint8_t*)&frontbuf,sizeof(frontbuf));
      //Radio off
      for (int i=0; i < radio_num; i++)
        gnrc_netapi_set(radio[i], NETOPT_STATE, 0, &radio_state, sizeof(netopt_state_t));
      //Sleep
      xtimer_usleep(interval_with_jitter());
    }

    return 0;
}
