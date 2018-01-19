#!/bin/bash

# read username for hs-karlsruhe.de proxy
read -p "HsKA Username: " username

# read password for hs-karlsruhe.de proxy
read -s -p "HsKA Password: " password
echo

# set user environment
git config --global http.proxy http://$username:$password@proxy.hs-karlsruhe.de:8888

# execute git command
git "$@"

# clear user environment
git config --global --unset http.proxy
