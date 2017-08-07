#!/bin/bash

scales=2
#true=1
#false=0
gpu=0 

dir=(/home/martin/Datasets/VBS/Train/*/)
for ((i = ${#dir[@]} - 1;i >= 0;i--)); do
	    d=${dir[i]}	
	    echo $d
	    
	    seq=$d
	    output=$d/output
	    mkdir -p $output
		
		for f in $seq/*.png; do
			filename=$(basename "$f")
			extension="${filename##*.}"
			#filename="${filename%.*}"
		
		
			#if the threshold is too small, too much noise from the edges shows up
			#if it is too big, finer levels of the pyramid miss the details
			build/segment $f $output/$filename $scales $gpu 0.05  
		done
		#  gdb --args 
	    
	    
	    
done


