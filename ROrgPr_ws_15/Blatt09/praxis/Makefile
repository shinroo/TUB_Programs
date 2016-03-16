#RUN := -all
#LIB := work
#VSIM := vsim -novopt
#VCOM := vcom
VMAP := RorgPrSimLib:/afs/tu-berlin.de/units/Fak_IV/aes/tools/rorg-pr/ROrgPrSimLib

export PATH := /afs/tu-berlin.de/units/Fak_IV/aes/tools/gtkwave/binaries/ubuntu$(shell grep RELEASE /etc/lsb-release | tr -c -d '[:digit:]' ):$(PATH)

-include /afs/tu-berlin.de/units/Fak_IV/aes/tools/gtkwave/modelsim.mk

all: mipsCtrlFsm_tb

mipsCtrlFsm_tb: vsim/mipsCtrlFsm_tb
	ln -fs vsim/mipsCtrlFsm_tb.vcd.gz waveform.vcd.gz
	ln -fs vsim/mipsCtrlFsm_tb        transcript

vsim/mipsCtrlFsm_tb: vcom/mipsCtrlFsm_tb.vhd

vcom/mipsCtrlFsm_tb.vhd: vcom/mipsCtrlFsm.vhd

.PHONY:
view_wave:
	gtkwave -mtranscript waveform.vcd.gz

.PHONY:
clean:
	rm -f waveform.vcd.gz
	rm -f transcript
