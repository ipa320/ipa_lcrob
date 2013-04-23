#!/bin/bash

for f in `ls /usr/lib/libboost_*so`
do
echo $f
ln -s $f $f.1.50.0
done
