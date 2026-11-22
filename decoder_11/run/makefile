all: lint sim syn

lint:
	../scripts/lint.sh

sim:
	../scripts/sim.sh

syn:
	../scripts/syn.sh

gui:
	gtkwave waveform.vcd

	
clean:
	rm -rf *.log *.vcd *.v *.o
