#RUN := -all
#LIB := work
#VSIM := vsim -novopt
#VCOM := vcom
VMAP := RorgPrSimLib:/afs/tu-berlin.de/units/Fak_IV/aes/tools/rorg-pr/ROrgPrSimLib

-include modelsim.mk

all: signExtend_tb leftShifter_tb mult32x32_tb

leftShifter_tb: vsim/leftShifter_tb
	ln -fs vsim/leftShifter_tb.vcd waveform.vcd
vsim/leftShifter_tb: vcom/leftShifter_tb.vhd
vcom/leftShifter_tb.vhd: vcom/leftShifter.vhd

mult32x32_tb: vsim/mult32x32_tb
	ln -fs vsim/mult32x32_tb.vcd waveform.vcd
vsim/mult32x32_tb: vcom/mult32x32_tb.vhd
vcom/mult32x32_tb.vhd: vcom/mult32x32.vhd
vcom/mult32x32.vhd: vcom/mult16x16SignedSigned.vhd
vcom/mult32x32.vhd: vcom/mult16x16UnsignedSigned.vhd
vcom/mult32x32.vhd: vcom/mult16x16UnsignedUnsigned.vhd

signExtend_tb: vsim/signExtend_tb
	ln -fs vsim/signExtend_tb.vcd waveform.vcd
vsim/signExtend_tb: vcom/signExtend_tb.vhd
vcom/signExtend_tb.vhd: vcom/signExtend.vhd

.PHONY:
clean:
	rm -f waveform.vcd
