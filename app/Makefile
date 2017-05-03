APPLICATION = hamilton_3c_qfw
RIOTBASE = $(CURDIR)/../RIOT-OS
BOARD = hamilton

# If you define this, PIR will not be sampled (Hamilton-3C)
# if it is not defined, it will be (Hamilton-7C)
CFLAGS += -DHAMILTON_3C_ONLY

USEMODULE += rtt_stdio
CFLAGS += -DRTT_STDIO_DISABLE_STDIN
CFLAGS += -DSOCK_HAS_IPV6
CFLAGS += -DTHREAD_STACKSIZE_DEFAULT=2048

CFLAGS += -DCRYPTO_AES

CFLAGS += -DDUTYCYCLE_SLEEP_INTERVAL=0
# Radio
USEMODULE += at86rf233

# Sensors
USEMODULE += tmp006
USEMODULE += hdc1000
USEMODULE += fxos8700
USEMODULE += gnrc_dutymac
USEMODULE += random
USEMODULE += xtimer
USEMODULE += crypto
USEMODULE += cipher_modes

# networking
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif
# Specify the mandatory networking modules for IPv6 and UDP
USEMODULE += gnrc_ipv6
USEMODULE += gnrc_udp
# Add also the shell, some shell commands
USEMODULE += netstats_l2
USEMODULE += netstats_ipv6

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
# CFLAGS += -DDEVELHELP

DEFAULT_CHANNEL = 25
CFLAGS += -DIEEE802154_DEFAULT_CHANNEL=$(DEFAULT_CHANNEL)

QUIET ?= 1

FEATURES_REQUIRED += periph_timer

include $(RIOTBASE)/Makefile.include
