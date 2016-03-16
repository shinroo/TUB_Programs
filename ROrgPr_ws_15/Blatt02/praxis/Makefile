#RUN := -all
#LIB := work
#VSIM := vsim -novopt
#VCOM := vcom
VMAP := RorgPrSimLib:/afs/tu-berlin.de/units/Fak_IV/aes/tools/rorg-pr/ROrgPrSimLib

-include modelsim.mk

all: neq4_tb bin2Char_tb

neq4_tb: vsim/neq4_tb
	ln -fs vsim/neq4_tb.vcd waveform.vcd
vsim/neq4_tb: vcom/neq4_tb.vhd
vcom/or2.vhd: $(wildcard or2.vhd)
vcom/xor2.vhd: $(wildcard xor2.vhd)
vcom/neq4.vhd: vcom/or2.vhd vcom/xor2.vhd
vcom/neq4_tb.vhd: vcom/neq4.vhd

bin2Char_tb: vsim/bin2Char_tb
	ln -fs vsim/bin2Char_tb.vcd waveform.vcd
vsim/bin2Char_tb: vcom/bin2Char_tb.vhd
vcom/bin2Char_tb.vhd: vcom/bin2Char.vhd

.PHONY:
clean:
	rm -f waveform.vcd
