#!/bin/bash

# This extracts RGB images from videos
for d in data/* ; do
	cd $d
    echo "Now inside $d"
	../../scripts/kinectImgsExtractor.sh
	echo "Done extracting images from $d"
	echo "Waiting for 2 seconds..."
	sleep 2
	cd ..
done