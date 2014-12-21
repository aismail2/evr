EVR
===
Evr is an EPICS driver for controlling the VME-EVR230 event receiver over the nework.

Background
==========
All control servers at SESAME run on Linux/x86 platforms. While the VME-EVR230 is typically controlled over the VME bus, there are some advantages in controlling it over the network instead:
* Drops the dead weight: The VME crate, the VME CPU card, the RTOS that runs on the CPU card (along with any required licenses), and the debug terminal that connects to the CPU card are no longer needed.
* Lowers the cost of implementation: A direct consequence of the point above.
* Confines the required development skills to Linux/x86 platforms. Knowledge in VME-bus, VxWorks, or any other RTOS/OS is not required.
* Maintains coherency in the control infrastructure (applies to SESAME since the rest of the IOC's at SESAME run on Linux/x86 platforms).

Features
========

Installation
============
Clone the repository and integrate the driver with the EPICS/support framework. Look in the "evr" folder for examples of database and startup files.
