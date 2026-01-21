### Linux kernel for SBC2440-I single board computer.

Linux kernel 4.9.337 with patches to support the SBC2440-I single-board computer (my board is marked “SBC2440V4”). The patches are based on the kernel for the Mini2440 board.

The SBC2440-I board is mostly similar to the popular Mini2440 board; however, it includes a second Ethernet interface, an additional USB port, a different storage layout, different buttons, etc.

My board originally came with a modified Linux kernel 2.6.13, but I was unable to find its source code. Therefore, I modified the original Linux kernel 4.9.337 by incorporating changes from the Mini2440 2.6.32.2 kernel and adding additional tweaks for the SBC2440-I where it differs from the Mini2440.

Why kernel 4.9? Linux 4.9 is the last LTS kernel that works reliably with Samsung K9F1208 NAND Flash memory.
In newer kernels, support for older NAND chips is largely broken.

What does not work:
- 10 Mb Ethernet interface (CS8900A)
- Only one USB port works (reason unknown at the moment)

Not used / not tested / not of interest:
- Camera interface
- IDE interface



Board photo: [sbc2440v4.jpg](_board_info/sbc2440v4.jpg)  
Board specification: [SBC2440I.pdf](_board_info/SBC2440I.pdf)  
Schematic: [sbc2440Isch.pdf](_board_info/sbc2440Isch.pdf)
