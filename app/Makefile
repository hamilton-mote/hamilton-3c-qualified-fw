APPLICATION = hamilton_3c_qfw
RIOTBASE = $(CURDIR)/../RIOT-OS
BOARD = hamilton

CFLAGS += -DMODEL_7C

# System functions
USEMODULE += random
USEMODULE += xtimer
USEMODULE += rtt_stdio
CFLAGS    += -DRTT_STDIO_DISABLE_STDIN
CFLAGS    += -DTHREAD_STACKSIZE_DEFAULT=2048

# CPU clock speed
CFLAGS    += -DCLOCK_USE_OSCULP32_DFLL=1
CFLAGS    += -DCLOCK_USE_ADAPTIVE=0
CFLAGS    += -DDMAC_ENABLE=0

# Sensors
USEMODULE += saul_reg        # SAUL: sensor/actuator abstraction layer
USEMODULE += auto_init_saul
USEMODULE += tmp006          # ambient temperature
USEMODULE += hdc1000         # humidity and temperature
USEMODULE += fxos8700        # acceleration and magnetic field
USEMODULE += apds9007        # illumination
USEMODULE += ekmb            # pir-based occupancy
USEMODULE += pulse_counter   # simple push button

# Security
USEMODULE += crypto
USEMODULE += cipher_modes
CFLAGS    += -DCRYPTO_AES

# Radio
USEMODULE += at86rf2xx
CFLAGS    += -DIEEE802154_DEFAULT_CHANNEL=25
CFLAGS    += -DAUTO_CSMA_EN=1

# Network
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif
# Duty-cycling
USEMODULE += gnrc_lasmac
CFLAGS    += -DDUTYCYCLE_SLEEP_INTERVAL=00000UL # If it is ZERO, no duty-cycling for packet reception
CFLAGS    += -DLEAF_NODE=1                       # Default is 1
# Specify the mandatory networking modules for IPv6 and UDP
USEMODULE += gnrc_ipv6
USEMODULE += gnrc_udp
CFLAGS    += -DSOCK_HAS_IPV6
# Add also the shell, some shell commands
USEMODULE += netstats_l2
USEMODULE += netstats_ipv6


# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
# CFLAGS += -DDEVELHELP

QUIET ?= 1

FEATURES_REQUIRED += periph_timer

include $(RIOTBASE)/Makefile.include
