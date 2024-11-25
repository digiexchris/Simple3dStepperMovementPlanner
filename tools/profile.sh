#!/bin/bash

export HEAPPROFILE=/tmp/heapprof
export CPUPROFILE=/tmp/prof.out

build/tests/StepTimingGenerator_Tests
# pprof --gv <path/to/binary> /tmp/heapprof.0045.heap
# pprof --gv <path/to/binary> /tmp/prof.out