#!/bin/bash

FUNC=${1-none}
DIR=${2-/home/denis/LIBS/lib64/}

for ff in `ls -1 ${DIR}/*.so`; do echo "==================== $ff"; objdump -t $ff; done | grep -e InputArray -e "====" | grep -e $FUNC   -e "===="
