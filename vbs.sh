#!/bin/bash

folder=/home/martin/Datasets/VBS/Test
output=/home/martin/Datasets/VBS/output
seq=$folder/arctic_kayak
scales=1

for f in $seq/*.png; do
    filename=$(basename "$f")
	extension="${filename##*.}"
	#filename="${filename%.*}"
    
    
    ./segment $f $output/$filename $scales 0 0.05
done
#  gdb --args 

