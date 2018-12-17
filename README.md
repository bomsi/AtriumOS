# AtriumOS

Just a hobby operating system, nothing big and professional.

## System Requirements

Here's an overview of requirements to run Atrium OS. Currently it is tested in
a virtual machine with the following configuration:
* RAM: 4 MB
* Storage: Floppy drive

Atrium OS assumes that the following BIOS services are supported:
* disk service to interact with the floppy drive (INT 13h)
* video service to change video mode to 320x200 resolution
with 256 colors (INT 10h, mode 13h)
* general service to query system address map (INT 15h, AX=e820h)
* general service to enable the A20 gate (INT 15h, AX=2401h)

