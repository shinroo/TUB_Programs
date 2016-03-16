export LM_LICENSE_FILE := 1650@bolero.cs.tu-berlin.de:2100@bolero.cs.tu-berlin.de:$(LM_LICENSE_FILE)
export PATH := $(PATH):/afs/tu-berlin.de/units/Fak_IV/aes/tools/mentor/modeltech/linux_x86_64

PRETTY_SIM_TIME := awk '/Time/{print $$0" "a; fflush(); next;} {a=$$0} $(VERBOSE)'
PRETTY_SIM_COLOR := awk '/Failure|Error/{print "\033[31m"$$0"\033[0m";fflush();next} /Warning/{print "\033[33m"$$0"\033[0m"; fflush(); next} 1'
PRETTY_COMP_COLOR := awk '/Error/{print "\033[31m"$$0"\033[0m";next} /Warning/{print "\033[33m"$$0"\033[0m";next} $(VERBOSE)'
PRETTY := awk '$(VERBOSE)'

ifndef TOPLEVEL
$(error TOPLEVEL is not defined!)
endif

ifndef SUFFIX
SUFFIX := vhd
endif

ifndef TOPLEVEL_ENTITY
TOPLEVEL_ENTITY := $(basename $(TOPLEVEL))
endif

ifndef VERBOSE
VERBOSE := 0
endif

ifndef LIB
LIB := work
endif

ifndef RUN
RUN := -all
endif

ifndef VCOM
VCOM := vcom
endif

ifndef VSIM
VSIM := vsim -novopt
endif

ifndef VCD
VCD := $(TOPLEVEL:%.$(SUFFIX)=%.vcd)
endif

ifndef TRANSCRIPT
TRANSCRIPT := $(TOPLEVEL:%.$(SUFFIX)=%.vsim)
endif

ifndef WLF
WLF := $(TOPLEVEL:%.$(SUFFIX)=%.wlf)
endif

help:
	@echo "################# usage ######################"
	@echo "make all        make compile sim."
	@echo "make compile    Compiles everything."
	@echo "make sim        Simulates TOPLEVEL."
	@echo "make clean      Removes everything generated."
	@echo "make clean_lib  Removes libs."
	@echo "make clean_comp Removes compiler outputs."
	@echo "make clean_sim  Removes simulation outputs."
	@echo "make log        Pretty prints simulation log."
	@echo "make options    Prints all setable options."
	@echo "##############################################"

options:
	@echo TOPLEVEL: $(TOPLEVEL)
	@echo TOPLEVEL_ENTITY: $(TOPLEVEL_ENTITY)
	@echo VERBOSE: $(VERBOSE)
	@echo LIB: $(LIB)
	@echo RUN: $(RUN)
	@echo VCOM: $(VCOM)
	@echo VSIM: $(VSIM)
	@echo VCD: $(VCD)
	@echo TRANSCRIPT: $(TRANSCRIPT)
	@echo WLF: $(WLF)

all: compile sim

compile: $(TOPLEVEL:.$(SUFFIX)=.vcom)

sim: $(TRANSCRIPT)

log: $(TRANSCRIPT)
	@cat $< | $(PRETTY_SIM)

$(TRANSCRIPT) $(WLF) $(VCD): $(TOPLEVEL:.$(SUFFIX)=.vcom)| $(LIB)
	@echo bash -c "$(VSIM) $(TOPLEVEL:.$(SUFFIX)=) -c <<< \"vcd file $(VCD); vcd add -r /$(TOPLEVEL_ENTITY)/*; run $(RUN);\" "
	@bash -c "$(VSIM) $(TOPLEVEL:.$(SUFFIX)=) -c <<< \"vcd file $(VCD); vcd add -r /$(TOPLEVEL_ENTITY)/*; run $(RUN);\" " | $(PRETTY_SIM_TIME) | $(PRETTY_SIM_COLOR) 
	@mv transcript $(TRANSCRIPT)
	@mv vsim.wlf $(WLF) 2> /dev/null || true

%.vcom:%.$(SUFFIX) | $(LIB)
	@rm -f $@.failed
	@echo "$(VCOM) $<"
	@$(VCOM) $< 2>&1 > $@;\
   error=$$?;\
	 $(PRETTY_COMP_COLOR) $@;\
   [ $$error -ne "0"  ] && mv $@ $@.failed;\
   exit $$error;

$(LIB):
	@echo "vlib $(LIB)"
	@vlib $(LIB) | $(PRETTY)
	@echo "vmap work $(LIB)"
	@vmap work $(LIB) | $(PRETTY)

.PHONY: clean
clean: clean_lib clean_comp clean_sim

.PHONY: clean_lib
clean_lib:
	rm -rf $(LIB)
	rm -f modelsim.ini

.PHONY: clean_comp
clean_comp:
	rm -f *.vcom
	rm -f *.vcom.failed

.PHONY: clean_sim
clean_sim:
	rm -f $(TRANSCRIPT)
	rm -f $(VCD)
	rm -f $(WLF)
