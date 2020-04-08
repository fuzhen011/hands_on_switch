#!/bin/bash

if [ $# -ne 1 ]; then
    echo "Usage: $0 [directory of app.c and app.h]"
    exit 1
fi

ln -sf $1/app.c
ln -sf $1/app.h
