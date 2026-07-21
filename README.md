# Remora-RT1052

> [!WARNING]
> This is an experimental fork of `scottalford75/Remora-RT1052-cpp`.
>
> The goal of this fork is to investigate and fix reliability issues, primarily for the EC500 V5 RT1052 controller.
>
> Development and code review are AI-assisted. Changes are not considered reliable until they have been manually reviewed, compiled, and tested on real hardware.
>
> **Current status:** Early development. No fixes have yet been fully validated for machine use.

## Planned work

* Fix identified firmware and LinuxCNC driver bugs
* Improve Ethernet communication and fault handling
* Add repeatable software and hardware tests
* Validate STEP/DIR output using independent measurement equipment

---

Remora-RT1052 is a port of Remora firmware for the Digital Dream Ethernet CNC controllers. This firmware allows these Mach3 boards to be used with LinuxCNC.
