prj_scripts_dir = ../scripts

lint:
	${prj_scripts_dir}/lint.sh

sim:
	${prj_scripts_dir}/sim.sh

syn:
	${prj_scripts_dir}/syn.sh

gui:
	gtkwave waveform.vcd

all: lint sim syn
	
clean:
	rm -rf *.log *.vcd *.v *.o
