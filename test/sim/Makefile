# Host simulation of the I3C target (protocol layer + PIO programs).
#   make            build and run the test suite
#   make sweep      try a range of SCL frequencies / CPU reaction times
# Needs pioasm (built by the Pico SDK into build/pioasm-install/pioasm/).

ROOT    := ../..
PIOASM  ?= $(firstword $(wildcard $(ROOT)/build/pioasm-install/pioasm/pioasm $(ROOT)/build/pioasm/pioasm) pioasm)
CC      ?= cc
CFLAGS  ?= -O2 -g -Wall -Wextra -Wno-unused-parameter -std=c11
CFLAGS  += -I. -I$(ROOT)/src

SRCS := sim.c pio_sim.c $(ROOT)/src/i3c_target.c $(ROOT)/src/i3c_log.c

all: run

i3c_target.pio.h: $(ROOT)/src/i3c_target.pio
	$(PIOASM) -o c-sdk $< $@

sim: $(SRCS) i3c_target.pio.h pio_sim.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

run: sim
	./sim

sweep: sim
	./sim --sweep

clean:
	rm -f sim i3c_target.pio.h

.PHONY: all run sweep clean
