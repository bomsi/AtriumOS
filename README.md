# AtriumOS

Just a hobby operating system, nothing big and professional.

## System Requirements

Here's an overview of requirements to run AtriumOS. Currently it is tested in
a virtual machine with the following configuration:
* CPU: x86-64
* RAM: 4 MB
* Storage: Floppy drive

AtriumOS assumes that the following BIOS services are supported:
* disk service to interact with the floppy drive (INT 13h)
* video service to change video mode to 320x200 resolution
with 256 colors (INT 10h, mode 13h)
* general service to query system address map (INT 15h, AX=e820h)
* general service to enable the A20 gate (INT 15h, AX=2401h)

## Build

To build the OS image, just run `nasm AtriumOS.asm -fbin -o atrium.flp`
from the `src` directory. NASM version [2.15.05](https://www.nasm.us/pub/nasm/releasebuilds/2.15.05/)
is currently used.

