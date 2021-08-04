CONTIKI_PROJECT = dimmer-app

#LDFLAGS += -Wl,--section-start -Wl,.testbedConfigSection=0xd400
#LDFLAGS += -Wl,--section-start -Wl,.rodata=0xd500


CONTIKI = ../..
DESCRIPTION ?= Dimmer app example

# for convenience only
ifeq ($(TARGET), dpp)
  override TARGET = dpp-cc430
endif

TARGET_LIBFILES+=-lm

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
MODULES += os/net/mac/lwb
MODULES += os/net/mac/dimmer
PROJECT_SOURCEFILES += rtimer-ext.c glossy.c
#PROJECT_CONF_PATH = project-conf.h
CFLAGS += -DPLATFORM_$(shell echo $(TARGET) | tr a-z\- A-Z_)

all: $(CONTIKI_PROJECT)
	$(info compiled for target platform $(TARGET) $(BOARD))
	@msp430-size $(CONTIKI_PROJECT).$(TARGET)

upload: $(CONTIKI_PROJECT).upload

include ../../tools/flocklab/Makefile.flocklab
include $(CONTIKI)/Makefile.include
