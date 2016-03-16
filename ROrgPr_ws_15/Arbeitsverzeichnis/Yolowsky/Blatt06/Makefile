#RUN := -all
#LIB := work
#VSIM := vsim -novopt
#VCOM := vcom
VMAP := RorgPrSimLib:/afs/tu-berlin.de/units/Fak_IV/aes/tools/rorg-pr/ROrgPrSimLib

-include modelsim.mk

all: dff_tb reg_tb ram_tb

dff_tb: vsim/dff_tb
	ln -fs vsim/dff_tb.vcd waveform.vcd
vsim/dff_tb: vcom/dff_tb.vhd
vcom/dff_tb.vhd: vcom/dff.vhd

reg_tb: vsim/reg_tb
	ln -fs vsim/reg_tb.vcd waveform.vcd
vsim/reg_tb: vcom/reg_tb.vhd
vcom/reg_tb.vhd: vcom/reg.vhd

ram_tb: vsim/ram_tb
	ln -fs vsim/ram_tb.vcd waveform.vcd
vsim/ram_tb: vcom/ram_tb.vhd
vcom/ram_tb.vhd: vcom/ram.vhd

.PHONY:
clean:
	rm -f waveform.vcd
