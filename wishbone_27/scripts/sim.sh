#! /usr/bin/env bash

xezim \
--compile \
-I ../src -I ../tb -s tb_top_3 \
-l elab.log \
../tb/tb_top_3.v

xezim \
--simulate \
-I ../src -I ../tb \
-l sim.log \
--wave \
-s tb_top_3 \
../tb/tb_top_3.v 
