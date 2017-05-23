#!/bin/bash
# Runs RewardShape, the kick off class for the RSGridWorld domain
if [ "$#" -eq 1 ]; then
    if [ "$1" = "mistake" ]; then
        java -cp ../lib/burlap.jar:. MistakeDriven
    fi
else
    java -cp ../lib/burlap.jar:. -Xmx4096M RewardShape
fi
