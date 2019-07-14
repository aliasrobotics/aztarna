#!/bin/bash

#Author: Lander Usategui <lander at aliasrobotics dot com>

export RESET="\e[0m"
export CYAN="\e[36m"
export RED="\e[31m"
export BOLD"=\e[1m"
export YELLOW="\e[93m"
WS_PATH="/home/root/aztarna"

function qaCode()
{
  echo -e "${CYAN}Linter checks for python code, using: pep8 ${BOLD}$(pep8 --version)${RESET}"
  pep8 ${WS_PATH}
  result=$?
  if [ $result -ne 0 ]; then
    echo -e "${RED}pep8 error/s found, please review it!${RESET}"
    exit 2
  else
    echo -e "${CYAN}No pep8 errors found!${RESET}"
  fi
}

qaCode
