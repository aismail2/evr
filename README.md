Introduction
============
Evr is an EPICS driver for controlling the VME-EVR230 event receiver over the nework. This driver is currently deployed at SESAME (www.sesame.org.jo).

Background
==========
All control servers at SESAME run on Linux/x86 platforms. While the VME-EVR230 is typically controlled over the VME bus, there are some advantages in controlling it over the network instead:
* Drops the dead weight: The VME crate, the VME CPU card, the RTOS that runs on the CPU card (along with any required licenses), and the debug terminal that connects to the CPU card are no longer needed.
* Lowers the cost of implementation: A direct consequence of the point above.
* Confines the required development skills to Linux/x86 platforms. Knowledge in VME-bus, VxWorks, or any other RTOS/OS is not required.
* Maintains coherency in the control infrastructure (applies to SESAME since the rest of the IOC's at SESAME run on Linux/x86 platforms).

Features
========
The driver implements the following features:
* Enable/disable the device.
* Set events and actions via the event mapping RAM.
* Control the widths and delays of pulser outputs (OTP).
* Control the prescalers, widths, and delays of extended pulser outputs (PDP).
* Control the prescalers or prescaler outputs.
* Multiplex the front panel.
* Process external event and external event input.

The driver does not implement the following features:
* Trigger events.
* Event decoder events.
* Event FIFO and timestamp events.
* Distributed bus and data transmission.
* Level outputs.
* Front panel CML outputs (VME-EVR-230RF only).
* Interlocks.
* Interrupts.


Installation
============
Clone the repository and integrate the driver with your EPICS/support framework. Look in the "evr" folder for examples of database and startup files.
