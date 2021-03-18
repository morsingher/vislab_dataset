#!/bin/bash

if [ -d "./results" ]; then
	rm -rf results
fi
mkdir results
./build/multi_view_clustering ./config/config.json
