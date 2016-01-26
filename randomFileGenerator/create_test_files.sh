#!/bin/bash

i="1"

while [ $i -lt 17 ]
do
    ./randomFileGenerator 230 bins/test_group_$i.bin
    i=$[$i+1]
done
