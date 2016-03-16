#RUN := -all
#LIB := work
#VSIM := vsim -novopt
#VCOM := vcom
VMAP := RorgPrSimLib:/afs/tu-berlin.de/units/Fak_IV/aes/tools/rorg-pr/ROrgPrSimLib

export PATH := /afs/tu-berlin.de/units/Fak_IV/aes/tools/gtkwave/binaries/ubuntu$(shell grep RELEASE /etc/lsb-release | tr -c -d '[:digit:]' ):$(PATH)

-include /afs/tu-berlin.de/units/Fak_IV/aes/tools/gtkwave/modelsim.mk

all: mipsCtrl_tb mipsCpu_tb

mipsCtrl_tb: vsim/mipsCtrl_tb
	ln -fs vsim/mipsCtrl_tb.vcd.gz waveform.vcd.gz
	ln -fs vsim/mipsCtrl_tb     transcript

mipsCpu_tb: vsim/mipsCpu_tb
	ln -fs vsim/mipsCpu_tb.vcd.gz waveform.vcd.gz
	ln -fs vsim/mipsCpu_tb     transcript

vcom/dff.vhd:         $(wildcard dff.vhd)
vcom/reg.vhd:         $(wildcard reg.vhd)
vcom/addrDecoder.vhd: $(wildcard addrDecoder.vhd)
vcom/regFile.vhd:     $(wildcard regFile.vhd)
vcom/leftShifter.vhd: $(wildcard leftShifter.vhd)
vcom/signExtend.vhd:  $(wildcard signExtend.vhd)
vcom/aluCtrl.vhd:     $(wildcard aluCtrl.vhd)
vcom/alu_1bit.vhd:    $(wildcard alu_1bit.vhd)
vcom/csAlu.vhd:       $(wildcard csAlu.vhd)
vcom/mipsCtrl.vhd:    $(wildcard mipsCtrl.vhd)
vcom/bin2Char.vhd:    $(wildcard bin2Char.vhd)
vcom/csAlu.vhd:       $(wildcard csAlu.vhd)
vcom/alu_1bit.vhd:    $(wildcard alu_1bit.vhd)
vcom/adder_1bit.vhd:  $(wildcard adder_1bit.vhd)

vcom/regFile.vhd: vcom/reg.vhd
vcom/regFile.vhd: vcom/addrDecoder.vhd

vcom/reg.vhd: vcom/dff.vhd

vcom/csAlu.vhd: vcom/alu_1bit.vhd

vcom/alu_1bit.vhd: vcom/adder_1bit.vhd

vsim/mipsCpu_tb: vcom/mipsCpu_tb.vhd

vcom/mipsCpu_tb.vhd: vcom/mipsCpu.vhd

vcom/mipsCpu.vhd: vcom/reg.vhd
vcom/mipsCpu.vhd: vcom/leftShifter.vhd
vcom/mipsCpu.vhd: vcom/mipsCtrl.vhd
vcom/mipsCpu.vhd: vcom/regFile.vhd
vcom/mipsCpu.vhd: vcom/signExtend.vhd
vcom/mipsCpu.vhd: vcom/aluCtrl.vhd
vcom/mipsCpu.vhd: vcom/csAlu.vhd
vcom/mipsCpu.vhd: vcom/bin2Char.vhd

vsim/mipsCtrl_tb: vcom/mipsCtrl_tb.vhd

vcom/mipsCtrl_tb.vhd: vcom/mipsCtrl.vhd

.PHONY:
view_wave:
	gtkwave -mtranscript waveform.vcd.gz

.PHONY:
clean:
	rm -f waveform.vcd.gz
	rm -f transcript
