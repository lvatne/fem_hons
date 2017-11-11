#!/bin/sh
cd $ROBOHOME/doc
pydoc3 -w $ROBOHOME/src/platform/*.py $ROBOHOME/src/imaging/*.py $ROBOHOME/src/weedkiller/*.py
