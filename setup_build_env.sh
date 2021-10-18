#!/bin/bash

#install miniconda for Linux x64 before launching this script.
#launch with bash -i OTHERWISE THIS WILL NOT WORK

# Install gcc and gcc-multilib
sudo apt install gcc
sudo apt install gcc-multilib

# Install nRF Command Line Tools. Version 10.13.0
wget https://www.nordicsemi.com/-/media/Software-and-other-downloads/Desktop-software/nRF-command-line-tools/sw/Versions-10-x-x/10-13-0/nRF-Command-Line-Tools_10_13_0_Linux64.zip
sudo apt-get install unzip
unzip nRF-Command-Line-Tools_10_13_0_Linux64.zip
tar -xf nRF-Command-Line-Tools_10_13_0_Linux64/nRF-Command-Line-Tools_10_13_0_Linux-amd64.tar.gz
chmod +x JLink_Linux_V750a_x86_64.deb
chmod +x nRF-Command-Line-Tools_10_13_0_Linux-amd64.deb
sudo dpkg -i JLink_Linux_V750a_x86_64.deb
sudo dpkg -i nRF-Command-Line-Tools_10_13_0_Linux-amd64.deb
rm -R nRF-Command-Line-Tools_10_13_0_Linux64
rm JLink_Linux_V750a_x86_64.*
rm nRF-Command-Line-Tools_10_13_0.*
rm nRF-Command-Line-Tools_10_13_0_Linux64.*
rm nRF-Command-Line-Tools_10_13_0_Linux-amd64.deb
rm README.txt

# Setup the developer environment (mainly for west). Versions in the yml file
conda env create -n dev_env -f z-wear_env.yml
conda activate dev_env

# West initialization with nrf sdk version 1.6.0
west init -m https://github.com/nrfconnect/sdk-nrf --mr v1.7.0
west update
west zephyr-export

# Change toolchain variables to use gnuarmemb. In bashrc to make it permanent.
echo 'export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb' >> ~/.bashrc
echo 'export GNUARMEMB_TOOLCHAIN_PATH=~/miniconda3/envs/dev_env' >> ~/.bashrc

# Restart terminal after launching this script.
