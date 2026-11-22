lint:
	../scripts/lint.sh

sim:
	../scripts/sim.sh

syn:
	../scripts/syn.sh

gui:
	gtkwave waveform.vcd

all: lint sim syn
	
clean:
	rm -rf *.log *.vcd *.v *.o
