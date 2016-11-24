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
#include <periph/gpio.h>
#include <periph/i2c.h>
#include <periph/adc.h>

#define SAMPLE_INTERVAL ( 10000000UL)
#define SAMPLE_JITTER   ( 2000000UL)

#define MAG_ACC_TYPE_FIELD 5
#define TEMP_TYPE_FIELD 6

void send_udp(char *addr_str, uint16_t port, uint8_t *data, uint16_t datalen);

uint32_t sample_counter;

typedef struct __attribute__((packed)) {
  uint16_t type;
  int16_t flags; //which of the fields below exist, bit 0 is acc_x
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
  uint64_t uptime;
} mag_acc_measurement_t;

typedef struct __attribute__((packed)) {
  uint16_t type;
  uint16_t flags; //which of the fields below exist
  uint16_t tmp_die;
  uint16_t tmp_val;
  uint16_t hdc_tmp;
  uint16_t hdc_hum;
  uint16_t light_lux;
  uint16_t buttons;
  uint64_t uptime;
  uint16_t occup;
} temp_measurement_t;

void sample_mag(mag_acc_measurement_t *m);
void sample_temp(temp_measurement_t *m);

#define FLAG_MAG_HAS_ACC_X  0x01
#define FLAG_MAG_HAS_ACC_Y  0x02
#define FLAG_MAG_HAS_ACC_Z  0x04
#define FLAG_MAG_HAS_MAG_X  0x08
#define FLAG_MAG_HAS_MAG_Y  0x10
#define FLAG_MAG_HAS_MAG_Z  0x20
#define FLAG_MAG_HAS_UPTIME  0x40

#define FLAG_TEMP_HAS_TMP_DIE  0x01
#define FLAG_TEMP_HAS_TMP_VAL  0x02
#define FLAG_TEMP_HAS_HDC_TMP  0x04
#define FLAG_TEMP_HAS_HDC_HUM  0x08
#define FLAG_TEMP_HAS_LUX      0x10
#define FLAG_TEMP_HAS_BUTTONS  0x20
#define FLAG_TEMP_HAS_UPTIME   0x40
#define FLAG_TEMP_HAS_OCCUP    0x80

#define FLAGS_COMBO  (FLAG_TEMP_HAS_HDC_TMP |\
                      FLAG_TEMP_HAS_HDC_HUM |\
                      FLAG_TEMP_HAS_LUX |\
                      FLAG_TEMP_HAS_OCCUP |\
                      FLAG_TEMP_HAS_BUTTONS |\
                      FLAG_TEMP_HAS_UPTIME)

//It's actually 6.5*2ms but lets give it 20ms to account for oscillator etc
#define HDC_ACQUIRE_TIME (20000UL)

tmp006_t tmp006;
hdc1000_t hdc1080;

uint16_t occupancy_events;
uint16_t button_events;

bool pir_high;
uint64_t pir_rise_time;
uint32_t acc_pir_time;
uint64_t last_pir_reset;

temp_measurement_t tm __attribute__((aligned(4)));
mag_acc_measurement_t am __attribute__((aligned(4)));

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
void low_power_init(void) {
    // Light sensor off
    gpio_init(GPIO_PIN(0,28), GPIO_OUT);
    gpio_init(GPIO_PIN(0,19), GPIO_OUT);
    gpio_write(GPIO_PIN(0, 28), 1);
    gpio_write(GPIO_PIN(0, 19), 0);
    int rv;

    //Init PIR accounting
    pir_high = false;
    last_pir_reset = xtimer_usec_from_ticks64(xtimer_now64());
    acc_pir_time = 0;
    button_events = 0;
    rv = hdc1000_init(&hdc1080, I2C_0, 0x40);
    if (rv != 0) {
      printf("failed to initialize HDC1008\n");
      critical_error();
      return;
    }

    rv = hdc1000_test(&hdc1080);
    if (rv != 0) {
      printf("hdc1080 failed self test\n");
      critical_error();
      return;
    } else {
      printf("hdc selftest passed");
    }

    // rv = tmp006_init(&tmp006, I2C_0, 0x44, TMP006_CONFIG_CR_AS4);
    // if (rv != 0) {
    //   printf("failed to initialize TMP006\n");
    //   critical_error();
    //   return;
    // }
    // rv = tmp006_test(&tmp006);
    // if (rv != 0) {
    //   printf("tmp006 failed self test\n");
    //   critical_error();
    //   return;
    // }
    // rv = tmp006_set_standby(&tmp006);
    // if (rv != 0) {
    //   printf("failed to standby TMP006\n");
    //   critical_error();
    //   return;
    // }

    gpio_init_int(GPIO_PIN(PA, 18), GPIO_IN_PU, GPIO_FALLING, on_button_trig, 0);
    gpio_init_int(GPIO_PIN(PA, 6), GPIO_IN, GPIO_BOTH, on_pir_trig, 0);
    adc_init(ADC_PIN_PA08);
}

void sample_mag(mag_acc_measurement_t *m) {
    m->type = MAG_ACC_TYPE_FIELD;
  //  m->flags = FLAG_HAS_TEMP;
    m->flags |= FLAG_MAG_HAS_UPTIME;
    m->uptime = xtimer_usec_from_ticks64(xtimer_now64());
}

void sample_temp(temp_measurement_t *m) {

    /* turn on light sensor and let it stabilize */
    gpio_write(GPIO_PIN(0, 28), 0);

    /* turn on LED */
    gpio_write(GPIO_PIN(0, 19), 1);

  /*  if (tmp006_set_active(&tmp006)) {
        printf("failed to active TMP006\n");
        critical_error();
        return;
    }*/
    if (hdc1000_startmeasure(&hdc1080)) {
        printf("failed to start hdc1080 measurement\n");
        critical_error();
        return;
    }
    xtimer_usleep(HDC_ACQUIRE_TIME);
    if(hdc1000_read(&hdc1080, &m->hdc_tmp, &m->hdc_hum)) {
        printf("failed to sample HDC\n");
        critical_error();
        return;
    }
  //  int adcrv = 5;
    int adcrv = adc_sample(ADC_PIN_PA08, ADC_RES_16BIT);
    printf("adcrv: %d\n", adcrv);
    m->light_lux = (int16_t) adcrv;
    /* Turn off light sensor */
    gpio_write(GPIO_PIN(0, 28), 1);
    /* turn off LED */
    gpio_write(GPIO_PIN(0, 19), 0);
  /*  if (tmp006_read(&tmp006, (int16_t*)&m->tmp_val, (int16_t*)&m->tmp_die, &drdy)) {
        printf("failed to sample TMP %d\n", drdy);
        critical_error();
        return;
    }
    if (tmp006_set_standby(&tmp006)) {
        printf("failed to standby the TMP\n");
        critical_error();
        return;
    }*/
    sample_counter++;
    m->uptime = xtimer_usec_from_ticks64(xtimer_now64());
    m->occup = ((uint64_t) acc_pir_time * 32768) / (m->uptime - last_pir_reset);
    last_pir_reset = m->uptime;
    acc_pir_time = 0;
    m->type = TEMP_TYPE_FIELD;
    m->flags = FLAGS_COMBO;
    m->buttons = button_events;

  //  printf("drdy %d\n", drdy);
      printf("sampled lux: %d\n", (int)m->light_lux);
      printf("sampled temp ok hdct=%d hdch=%d \n", (int)m->hdc_tmp, (int)m->hdc_hum);
      printf("sampled PIR %d\n", (int)m->occup);
      printf("buttone %d\n", (int)m->buttons);
}

uint32_t interval_with_jitter(void)
{
    int32_t t = SAMPLE_INTERVAL;
    t += rand() % SAMPLE_JITTER;
    t -= SAMPLE_JITTER / 2;
    return (uint32_t)t;
}



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
      sample_temp(&tm);
      //Send
      send_udp("ff02::1",4747,(uint8_t*)&tm,sizeof(temp_measurement_t));
      //Radio off
      for (int i=0; i < radio_num; i++)
        gnrc_netapi_set(radio[i], NETOPT_STATE, 0, &radio_state, sizeof(netopt_state_t));
      //Sleep
      xtimer_usleep(interval_with_jitter());
    }

    return 0;
}
