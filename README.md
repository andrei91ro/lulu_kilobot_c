# Kilobot robot controller modelled using a P/XP colony, written in C

This C application allows for the control of a Kilobot robot using P colonies.

This application has two distinct parts: (1) a PC simulation using the Kilombo simulator and (2) a firmware generation component that can be used to control real robots. Both components use the same C source code.

# Requirements
* Compilers:
  * GCC - for simulations on a computer
  * AVR-GCC - for building a firmware file (Intel hex) that can be used on real Kilobot robots
* Libraries
  * [Lulu\_pcol\_sim\_C](https://github.com/andrei91ro/lulu_pcol_sim_c) - includes the P/XP colony execution framework
  * [Kilolib](https://github.com/acornejo/kilolib) - for interfacing with the Kilobot robot
  * [Kilombo](https://github.com/JIC-CSB/kilombo) - for building a standalone simulation application for the entire Kilobot swarm, that uses the same C code as the real robots
* [Lulu\_C](https://github.com/andrei91ro/lulu_c) - Python 3 conversion script used to translate Lulu text input files to C source code
* [Make](https://www.gnu.org/software/make/) - for an automated build process

The build process was tested on an up-to-date Linux distribution (ArchLinux). The build should work on most unixes and on Windows using packages such as MinGW.

The [Kilombo](https://github.com/JIC-CSB/kilombo) library has other library dependencies that have to be resolved.

# Usage

## Build process

After installing all components and adjusting the corresponding paths (using Makefile parameters), simply execute the following list of commands, in the project folder:

`mkdir build build_hex

make`


It is important to ensure that both [Lulu\_pcol\_sim\_C](https://github.com/andrei91ro/lulu_pcol_sim_c) and [Lulu\_C](https://github.com/andrei91ro/lulu_c) use the same branch (i.e `master` or `initialize_progmem`) as Lulu_kilobot. Different branches are not compatible.

If no errors are reported during compilation, the build folders should contain, among others, the following files:

* build
  * lulu\_kilobot
* build\_hex
  * lulu\_kilobot.hex

`lulu_kilobot` can be executed as a PC application (Kilombo simulation) and will run the P colony based robot controller specified in the `LULU_INSTANCE_FILE` Makefile parameter, by default the dispersion from nearby neighbors.

The `build_hex` folder contains the Intel Hex file required for flashing real Kilobot robots.

The default example P colony, `input_files/test_disperse.lulu` is a dispersion algorithm where robots move randomly if there are neighbors closer than a pre-specified range.

## Using a different controller input file

When changing or updating the Lulu P colony input file, the build tree must be cleaned and then the controller has to be rebuilt:

`make clean && make`

# Configuration

## Debug level

This parameter controls the verbosity of the `Lulu_kilobot` controller. The accepted values are:

* 0 (DEBUG)
* 1 (INFO)
* 2 (RELEASE) - no messages are printed

# API Documentation

More detailed information can be found on the project [documentation page](https://andrei91ro.github.io/lulu_kilobot_c).

# Authors
Andrei George Florea, [Cătălin Buiu](http://catalin.buiu.net)

[Department of Automatic Control And Systems Engineering](http://acse.pub.ro),

Politehnica University of Bucharest

Bucharest, Romania.
