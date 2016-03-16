#RUN := -all
#LIB := work
#VSIM := vsim -novopt
#VCOM := vcom
VMAP := RorgPrSimLib:/afs/tu-berlin.de/units/Fak_IV/aes/tools/rorg-pr/ROrgPrSimLib

export PATH := /afs/tu-berlin.de/units/Fak_IV/aes/tools/gtkwave/binaries/ubuntu$(shell grep RELEASE /etc/lsb-release | tr -c -d '[:digit:]' ):$(PATH)

-include /afs/tu-berlin.de/units/Fak_IV/aes/tools/gtkwave/modelsim.mk

all: mipsCpu_mc_mem_tb mipsCpu_mc_alu_tb

mipsCpu_mc_alu_tb: .mipsCpu_mc_alu_tb
	ln -fs vsim/mipsCpu_mc_tb.vcd.gz waveform.vcd.gz
	ln -fs vsim/mipsCpu_mc_tb        transcript

mipsCpu_mc_mem_tb: .mipsCpu_mc_mem_tb
	ln -fs vsim/mipsCpu_mc_tb.vcd.gz waveform.vcd.gz
	ln -fs vsim/mipsCpu_mc_tb        transcript

vcom/dff.vhd:         $(wildcard dff.vhd)
vcom/reg.vhd:         $(wildcard reg.vhd)
vcom/addrDecoder.vhd: $(wildcard addrDecoder.vhd)
vcom/regFile.vhd:     $(wildcard regFile.vhd)
vcom/leftShifter.vhd: $(wildcard leftShifter.vhd)
vcom/signExtend.vhd:  $(wildcard signExtend.vhd)
vcom/aluCtrl.vhd:     $(wildcard aluCtrl.vhd)
vcom/alu_1bit.vhd:    $(wildcard alu_1bit.vhd)
vcom/csAlu.vhd:       $(wildcard csAlu.vhd)
vcom/mipsCtrlFsm.vhd: $(wildcard mipsCtrlFsm.vhd)
vcom/bin2Char.vhd:    $(wildcard bin2Char.vhd)
vcom/csAlu.vhd:       $(wildcard csAlu.vhd)
vcom/alu_1bit.vhd:    $(wildcard alu_1bit.vhd)
vcom/adder_1bit.vhd:  $(wildcard adder_1bit.vhd)

vcom/regFile.vhd: vcom/reg.vhd
vcom/regFile.vhd: vcom/addrDecoder.vhd

vcom/reg.vhd: vcom/dff.vhd

vcom/csAlu.vhd: vcom/alu_1bit.vhd

vcom/alu_1bit.vhd: vcom/adder_1bit.vhd

.mipsCpu_mc_mem_tb: VSIM=vsim -novopt -ginit_file_name="memcpy.mif"
.mipsCpu_mc_mem_tb: vsim/mipsCpu_mc_tb

.mipsCpu_mc_alu_tb: VSIM=vsim -novopt -ginit_file_name="clip.mif"
.mipsCpu_mc_alu_tb: vsim/mipsCpu_mc_tb

vsim/mipsCpu_mc_tb: vcom/mipsCpu_mc_tb.vhd
vcom/mipsCpu_mc_tb.vhd: vcom/mipsCpu_mc.vhd

vcom/mipsCpu_mc.vhd: vcom/reg.vhd
vcom/mipsCpu_mc.vhd: vcom/leftShifter.vhd
vcom/mipsCpu_mc.vhd: vcom/mipsCtrlFsm.vhd
vcom/mipsCpu_mc.vhd: vcom/regFile.vhd
vcom/mipsCpu_mc.vhd: vcom/signExtend.vhd
vcom/mipsCpu_mc.vhd: vcom/aluCtrl.vhd
vcom/mipsCpu_mc.vhd: vcom/csAlu.vhd
vcom/mipsCpu_mc.vhd: vcom/bin2Char.vhd

.PHONY:
view_wave:
	gtkwave -mtranscript waveform.vcd.gz

.PHONY:
clean:
	rm -f waveform.vcd.gz
	rm -f transcript
