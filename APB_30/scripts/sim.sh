#! /usr/bin/env bash

xezim \
--compile \
-I ../src -I ../tb -s tb_top_1 \
-l elab.log \
../tb/tb_top_1.v

xezim \
--simulate \
-I ../src -I ../tb \
-l sim.log \
--wave \
-s tb_top_1 \
../tb/tb_top_1.v 
