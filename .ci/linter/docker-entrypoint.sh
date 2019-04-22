#!/bin/bash

#Install aztarna dependencies
cd /root/aztarna
python3.7 -m pip install -r requirements.txt
python3.7 -m pip install pylint
python3.7 -m pylint linter/.pylintrc aztarna
