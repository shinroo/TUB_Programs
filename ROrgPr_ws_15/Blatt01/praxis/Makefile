
# <- KOMMENTARZEICHEN
# ZUM WECHSELN DER TESTBENCH DIE FOLGENDEN ZEILEN ENTSPRECHEND AUSKOMMENTIEREN/KOMMENTIEREN
TOPLEVEL := logic_tb.vhd
#TOPLEVEL := mux_tb.vhd

VCD := waveform.vcd

# EXTRA OPTIONEN, ANGEGEBEN SIND DIE DEFAULT-WERTE
#TOPLEVEL_ENTITY := $(basename $(TOPLEVEL))
#SUFFIX := vhd
#RUN := -all
#VCD := $(TOPLEVEL:%.$(SUFFIX):=%.vcd)
#WLF := $(TOPLEVEL:%.$(SUFFIX)=%.wlf)
#TRANSCRIPT := $(TOPLEVEL:%.$(SUFFIX):=%.vsim)
#LIB := work
#VSIM := vsim -novopt
#VCOM := vcom
#VERBOSE := 0

-include ./modelsim.mk

logic_tb.vcom: and2.vcom xor2.vcom xnor2.vcom simlib
mux_tb.vcom: mux2.vcom simlib

and2.vcom: $(wildcard and2.vhd)
xor2.vcom: $(wildcard xor2.vhd)
xnor2.vcom: $(wildcard xnor2.vhd)

simlib:
	vmap RorgPrSimLib  /afs/tu-berlin.de/units/Fak_IV/aes/tools/rorg-pr/ROrgPrSimLib
