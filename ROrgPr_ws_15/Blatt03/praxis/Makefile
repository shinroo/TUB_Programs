#RUN := -all
#LIB := work
#VSIM := vsim -novopt
#VCOM := vcom
VMAP := RorgPrSimLib:/afs/tu-berlin.de/units/Fak_IV/aes/tools/rorg-pr/ROrgPrSimLib

-include modelsim.mk

all: clkReduce_tb mux4_tb

clkReduce_tb: vsim/clkReduce_tb
	ln -fs vsim/clkReduce_tb.vcd waveform.vcd
vsim/clkReduce_tb: vcom/clkReduce_tb.vhd
vcom/clkReduce_tb.vhd: vcom/clkReduce.vhd

mux4_tb: vsim/mux4_tb
	ln -fs vsim/mux4_tb.vcd waveform.vcd
vsim/mux4_tb: vcom/mux4_tb.vhd
vcom/mux4_tb.vhd: vcom/mux4.vhd

.PHONY:
clean:
	rm -f waveform.vcd
