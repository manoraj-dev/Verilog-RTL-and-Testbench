#! /usr/bin/env bash

xezim \
--compile \
-I ../src -I ../tb -s tb_top_2 \
-l elab.log \
../tb/tb_top_2.v

xezim \
--simulate \
-I ../src -I ../tb \
-l sim.log \
--wave \
-s tb_top_2 \
../tb/tb_top_2.v 
