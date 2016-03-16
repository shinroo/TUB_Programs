#RUN := -all
#LIB := work
#VSIM := vsim -novopt
#VCOM := vcom
VMAP := RorgPrSimLib:/afs/tu-berlin.de/units/Fak_IV/aes/tools/rorg-pr/ROrgPrSimLib

-include modelsim.mk

all: addrDecoder_tb regFile_tb

addrDecoder_tb: vsim/addrDecoder_tb
	ln -fs vsim/addrDecoder_tb.vcd waveform.vcd
vsim/addrDecoder_tb: vcom/addrDecoder_tb.vhd
vcom/addrDecoder_tb.vhd: vcom/addrDecoder.vhd

regFile_tb: vsim/regFile_tb
	ln -fs vsim/regFile_tb.vcd waveform.vcd
vsim/regFile_tb: vcom/regFile_tb.vhd
vcom/regFile_tb.vhd: vcom/regFile.vhd
vcom/regFile.vhd: vcom/reg.vhd
vcom/regFile.vhd: vcom/addrDecoder.vhd
vcom/reg.vhd: $(wildcard reg.vhd)


.PHONY:
clean:
	rm -f waveform.vcd
