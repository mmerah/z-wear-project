# Z-Wear

Zephyr based wearable device project template. This is a framework for quick development of a Zephyr-based IoT device supporting BLE and geared towards wearables.

This will contain:
- Conda for managing build environments
- Github actions for building, tracking code size deltas, and automated testing. Automatic versioning.
- Application template code supporting BLE, quick service definitions and easy sensor interfacing.
- Drivers for some interesting sensors for wearables.
- and more ...

# Upcoming

- Kicad files for a board ?
- Build environment management: make a conda package work for nRF-Command-Lines-Tools. Make the bash script more compatible, right now it would only work on Linux and if you do not change the default Miniconda3 installation path.
- GitHub Actions: Get a python script to calculate code sizes and deltas and incorporate it into a GitHub Action. Make action faster using caching. Incorporate automated unit testing and automated versionning.
- Template code: Code is not ideal right now, it is only a draft and was written quickly. Missing lots of features.
- Incorporate drivers for wearables-oriented sensors that are not in the zephyr repo.
