#!/bin/bash

if [ -d "./results" ]; then
	rm -rf results
fi
mkdir results
./build/vislab_dataset ./config/config.json
