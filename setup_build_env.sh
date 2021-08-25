#!/bin/bash

#install miniconda for Linux x64 before launching this script.
#launch with bash -i OTHERWISE THIS WILL NOT WORK

# Install gcc and gcc-multilib
sudo apt install gcc
sudo apt install gcc-multilib

# Setup the developer environment (mainly for west). Versions in the yml file
conda env create -n dev_env -f z-wear_env.yml
conda activate dev_env

# West initialization with nrf sdk version 1.6.0
west init -m https://github.com/nrfconnect/sdk-nrf --mr v1.6.0
west update
west zephyr-export

# Change toolchain variables to use gnuarmemb. In bashrc to make it permanent.
echo 'export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb' >> ~/.bashrc
echo 'export GNUARMEMB_TOOLCHAIN_PATH=~/miniconda3/envs/cobrax_env' >> ~/.bashrc

# Restart terminal after launching this script.
