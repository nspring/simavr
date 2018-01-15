# 
# this makefile derived from example makefile in simavr to
# construct a system that provides timed uart serial input
# to the 32u4 and traces the output signals using vcd and
# captures serial output for postprocessing by python or
# ruby.
# 

target=	harness-216
simavr = ../
SIMAVR=../

IPATH = .
# unused IPATH += ../examples/parts
IPATH += ${simavr}/include
IPATH += ${simavr}/simavr/sim

VPATH = .
# unused VPATH += ../examples/parts

LDFLAGS += -lpthread

# include ../Makefile.opengl

all: obj ${target}

include ${simavr}/Makefile.common

board = ${OBJ}/${target}.elf

${board} : ${OBJ}/${target}.o

${target}: ${board}
	@echo $@ done

clean: clean-${OBJ}
	rm -rf *.a *.axf ${target} *.vcd *.hex

run: ${board}
	${board} ../../build-circuitplay32u4cat/circuit-playground.elf
