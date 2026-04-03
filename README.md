# Pultrusion-Gradient-Sensor-for-PET-waste-management

> An open-source, Arduino-based volumetric gradient sensor that actively corrects filament inconsistencies in PET bottle pultrusion systems — making recycled 3D printing filament reliable enough to actually use.

![ISEF 2024](ISEF%202024%20Banner.png)

---

## The Problem

Single-use PET bottles degrade unevenly in landfills. Temperature, pressure, and time cause the plastic to thin, warp, and fragment at different rates across the bottle's surface. When those bottles are cut into strips and fed through a pultrusion system to make 3D printing filament, those thickness variations carry through to the final product — producing filament with volumetric gradients that cause under-extrusion, clogs, stringing, and failed prints.

Standard pultrusion systems have no way to detect or respond to these inconsistencies. The strip goes in, filament comes out, and whatever gradients existed in the strip exist in the filament too.

---

## The Solution

This project introduces a **prototypic volumetric gradient sensor** that slots into any standard open-source PET pultrusion rig and actively corrects for strip inconsistencies in real time.

The sensor works by mechanically amplifying the strip's surface profile through a two-lever system. As the strip passes under **Lever 1** (a heated 60°C ball bearing that flattens surface artefacts and ensures flat contact), **Lever 2** — connected at a 1:3 arm ratio — amplifies that motion and moves a toroidal magnet. A **DRV5055A4 Hall effect sensor** reads the resulting magnetic field change. That reading goes to an **Arduino UNO R3**, which calculates the deviation from the nominal strip thickness and adjusts the **Nema 17 stepper motor's pull rate** accordingly.

- **Thick region detected** → pull rate increases → strip is stretched → volume reduced toward nominal
- **Thin region detected** → pull rate decreases → strip is compressed → volume increased toward nominal

The correction happens before that segment of strip reaches the nozzle, so the filament that exits is volumetrically consistent even when the strip feeding in is not.

---

## Results

Testing was conducted across 20 prints using the corrected filament versus uncorrected strips.

| Metric | Without Sensor | With Sensor |
|---|---|---|
| Volumetric gradient reduction | — | **>85%** |
| Failed prints (per 20) | 9–11 | **2–3** |
| Max deviation in filament diameter | baseline | **−85.92%** |
| Mean filament diameter | irregular | **~2 mm** |

**Best operating configuration found through experimentation:**

| Parameter | Value |
|---|---|
| Hall effect sensor frequency | 200 Hz |
| Nozzle temperature | 235 °C |
| Pull rate (nominal) | 0.38 cm/s |

The filament cost for 2 kg of PET filament produced by this system was estimated at **$6** — roughly 4× cheaper than equivalent commercial filament (ABS, PLA, PETG).

---

## Hardware

| Component | Purpose |
|---|---|
| Arduino UNO R3 | Main controller |
| DRV5055A4 Hall effect sensor | Strip thickness measurement (12.5 mV/mT sensitivity) |
| Nema 17 stepper motor | Puller mechanism drive |
| A4988 / DRV8825 stepper driver | Motor control |
| N52 neodymium magnet (20×5×5 mm) | Lever 2 displacement measurement |
| 608zz ball bearing (22 mm OD) | Lever 1 roller |
| 685zz ball bearing (5 mm OD) | Lever 2 tip |
| NTC thermistor (optional) | Ambient temperature correction for STC compensation |
| Balsa wood / aluminium base | Structural frame for the sensor |
