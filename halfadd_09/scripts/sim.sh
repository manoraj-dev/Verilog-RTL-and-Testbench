#! /usr/bin/env bash

xezim \
--compile \
-I ../src -I ../tb -s tb_top \
-l elab.log \
../tb/tb_top.v

xezim \
--simulate \
-I ../src -I ../tb \
-l sim.log \
--wave \
-s tb_top \
../tb/tb_top.v 
