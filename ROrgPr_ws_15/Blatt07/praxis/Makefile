#RUN := -all
#LIB := work
#VSIM := vsim -novopt
#VCOM := vcom
VMAP := RorgPrSimLib:/afs/tu-berlin.de/units/Fak_IV/aes/tools/rorg-pr/ROrgPrSimLib

-include modelsim.mk

all: alu_1bit_tb aluCtrl_tb csAlu_tb

alu_1bit_tb: vsim/alu_1bit_tb
	ln -fs vsim/alu_1bit_tb.vcd waveform.vcd
vsim/alu_1bit_tb: vcom/alu_1bit_tb.vhd
vcom/alu_1bit_tb.vhd: vcom/alu_1bit.vhd
vcom/alu_1bit.vhd: vcom/adder_1bit.vhd vcom/mux2.vhd vcom/mux4.vhd

aluCtrl_tb: vsim/aluCtrl_tb
	ln -fs vsim/aluCtrl_tb.vcd waveform.vcd
vsim/aluCtrl_tb: vcom/aluCtrl_tb.vhd
vcom/aluCtrl_tb.vhd: vcom/aluCtrl.vhd

csAlu_tb: vsim/csAlu_tb
	ln -fs vsim/aluCtrl_tb.vcd waveform.vcd
vsim/csAlu_tb: vcom/csAlu_tb.vhd
vcom/csAlu_tb.vhd: vcom/csAlu.vhd
vcom/csAlu.vhd: vcom/alu_1bit.vhd

.PHONY:
clean:
	rm -f waveform.vcd
