#!/bin/bash

OUT_FILE_NAME=$1;
ARFF_CACHE_PATH=$2;
for ARFF_FILE in `ls $ARFF_CACHE_PATH/*.arff`
do
	(cat $ARFF_FILE; echo '') >> $OUT_FILE_NAME;
done
