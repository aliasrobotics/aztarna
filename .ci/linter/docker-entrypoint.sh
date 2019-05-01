#!/bin/bash

#Install aztarna dependencies
cd /root/aztarna
python3.7 -m pip install -r requirements.txt
pip3 install pylint
pylint linter/.pylintrc aztarna
