RTKLIB_ROOT ?= ../../third_party/RTKLIB
CC ?= gcc
CFLAGS ?= -O3 -Wall -Wextra
LIBDIR ?= $(CURDIR)/lib
RTKLIB_SO := $(LIBDIR)/librtklib_legsa.so
BRIDGE_SO := $(LIBDIR)/liblegsa_rtklib_bridge.so
CORRECTIONS_SO := $(LIBDIR)/libgnss_corrections.so
DEFS := -DTRACE -DENAGLO -DENAQZS -DENAGAL -DENACMP -DENAIRN -DNFREQ=5 -DNEXOBS=3
RTKLIB_SOURCES := \
	$(RTKLIB_ROOT)/src/rtkcmn.c $(RTKLIB_ROOT)/src/rinex.c \
	$(RTKLIB_ROOT)/src/sbas.c $(RTKLIB_ROOT)/src/preceph.c \
	$(RTKLIB_ROOT)/src/rcvraw.c $(RTKLIB_ROOT)/src/convrnx.c \
	$(RTKLIB_ROOT)/src/rtcm.c $(RTKLIB_ROOT)/src/rtcm2.c \
	$(RTKLIB_ROOT)/src/rtcm3.c $(RTKLIB_ROOT)/src/rtcm3e.c \
	$(RTKLIB_ROOT)/src/pntpos.c $(RTKLIB_ROOT)/src/ephemeris.c \
	$(RTKLIB_ROOT)/src/ionex.c $(RTKLIB_ROOT)/src/lambda.c \
	$(RTKLIB_ROOT)/src/rcv/novatel.c $(RTKLIB_ROOT)/src/rcv/ss2.c \
	$(RTKLIB_ROOT)/src/rcv/ublox.c $(RTKLIB_ROOT)/src/rcv/crescent.c \
	$(RTKLIB_ROOT)/src/rcv/skytraq.c $(RTKLIB_ROOT)/src/rcv/javad.c \
	$(RTKLIB_ROOT)/src/rcv/nvs.c $(RTKLIB_ROOT)/src/rcv/binex.c \
	$(RTKLIB_ROOT)/src/rcv/rt17.c $(RTKLIB_ROOT)/src/rcv/septentrio.c \
	rtklib_compat.c

.PHONY: all
all: $(BRIDGE_SO) $(CORRECTIONS_SO)

$(LIBDIR):
	mkdir -p $@

$(RTKLIB_SO): $(RTKLIB_SOURCES) | $(LIBDIR)
	$(CC) -shared -fPIC $(CFLAGS) -Wno-unused-but-set-variable \
		-I$(RTKLIB_ROOT)/src $(DEFS) $^ -lm -lrt -o $@

$(BRIDGE_SO): legsa_rtklib_bridge.c legsa_rtklib_bridge.h $(RTKLIB_SO)
	$(CC) -shared -fPIC $(CFLAGS) -I$(RTKLIB_ROOT)/src $(DEFS) \
		legsa_rtklib_bridge.c -L$(LIBDIR) -lrtklib_legsa \
		-Wl,-rpath,'$$ORIGIN' -o $@

$(CORRECTIONS_SO): rtklib_corrections.c $(RTKLIB_SO)
	$(CC) -shared -fPIC $(CFLAGS) -I$(RTKLIB_ROOT)/src $(DEFS) \
		rtklib_corrections.c -L$(LIBDIR) -lrtklib_legsa \
		-Wl,-rpath,'$$ORIGIN' -o $@
