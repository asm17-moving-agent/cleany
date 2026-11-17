# Hansaem Digitec fabrication profile

This project targets **Hansaem Digitec (한샘디지텍), two copper layers, nominal 1.6T FR-4 and 1 oz base copper**. KiCad loads `integrated_controller.kicad_dru` beside the project. General constraints and drawing defaults are in `integrated_controller.kicad_pro`; solder-mask settings are in `integrated_controller.kicad_pcb`.

## Limits

Dimensions are in mm. Manufacturer limits follow [Hansaem Digitec's fabrication specifications](https://www.hsdgt.com/order/information.html), checked on 2026-11-11. Retained project margins are identified separately; these are not additional manufacturer requirements.

| Feature | Applied value |
|---|---:|
| Track width and general copper clearance | Manufacturer 0.125 minimum (1 oz base copper + plating) |
| Via drill, copper diameter and annular ring | Manufacturer 0.20 drill and 0.15 ring; derived 0.50 diameter minimum |
| PTH annular ring | Manufacturer 0.20 minimum |
| Mechanical drill | Manufacturer 0.20–6.00; larger holes require router processing |
| Plated slot width | Manufacturer 0.50 minimum; normal process 0.70+ |
| Copper to routed board edge | Manufacturer 0.30; project 0.50 minimum |
| Solder-mask web | Manufacturer 0.10 for green/red/yellow/blue, 0.12 for black/white; project 0.13 |
| Mask opening to neighboring copper | Manufacturer 0.05; project 0.09 minimum |
| Silk text height, stroke, exposed-pad clearance | Manufacturer 0.80, 0.125, 0.10; project 1.00, 0.15, 0.15 minimum |
| Concave outline corner radius | Manufacturer normal router R0.90; mounting-ear roots use R0.90 |
| SMD copper pad dimensions | Project 0.25 × 0.25 minimum |
| Different-net SMD pad / zone clearance | Project 0.15 / 0.25 minimum |
| NPTH hole / slot width | Project 0.50 / 1.00 minimum |
| Plated slot length / width | Project ratio at least 2; manufacturer recommends length ≥ 2 × width + 0.20 |
| Via-hole / pad-hole / mixed drill spacing | Project 0.20 / 0.45 / 0.45 minimum, regardless of net |
| Copper to drill / PTH drill to track | Project 0.20 / 0.28 minimum |

NPTH locating pegs do not require copper rings. Mask expansion is zero, following Hansaem's recommendation to submit pad-sized openings for their CAM expansion. Individual component land and stencil requirements remain part-specific. The outline is routed, with no V-cut; the 0.50 mm copper-edge margin also covers Hansaem's V-cut copper clearance but does not define a panel design.

The saved KiCad stackup remains a nominal model: two 0.035 mm copper layers, a 1.51 mm FR-4 core and two 0.01 mm mask layers, totaling 1.60 mm. It is not a supplier-confirmed finished stackup. Hansaem defines the selected copper weight as **base copper before plating** and lists approximately **1.67 mm average finished thickness for 1.6T, 1 oz two-layer boards**, with ±0.13 mm CCL tolerance including copper. Use `2 Layer / FR-4 / 1.6T / Base Copper 1oz` on the order. If assembly requires a finished 1.60 mm limit, specify that separately with the supplier. Solder-mask color and surface finish are not selected by this change; KiCad's `copper_finish "None"` is an unset design field, not an instruction to fabricate bare copper.

The ordinary process uses through vias. Smaller SMD/BGA lands and special drilling processes require an explicitly reviewed profile rather than silently relaxing these limits.

Default routing starts at 0.20 mm tracks and 0.60/0.30 mm vias. Crowded motor and I²C signal segments use 0.15 mm tracks to meet the 0.125 mm spacing rule. Power track widths are retained; local power-route vertices and signal-via fanouts move to provide clearance. Fabrication minima do not determine power-rail current capacity or USB impedance; select those geometries for the circuit and actual stackup.

## USB routing

The USB pairs `USB_D_P/N` and `/USB_CONN_P/N` retain native KiCad differential routing and skew tuning. The custom rules specify **0.35 mm width, 0.15 mm pair edge gap, and 0.25 mm clearance to GND**. They retain the maximum 0.50 mm skew within each pair as a project target, not a USB compliance limit. Route the pairs on F.Cu above uninterrupted B.Cu GND and maintain GND copper along both outer edges of the coupled sections. The clearance rule sets a minimum distance; it cannot ensure that adjacent ground exists.

The **90 Ω differential target** follows [Espressif's USB layout guidance](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/pcb-layout-design.html#usb). The existing 0.35/0.15/0.25 mm width/pair-gap/GND-gap geometry is retained. The previous 89.522 Ω estimate used JLCPCB's JLC0216A stackup and is not a Hansaem impedance result. Hansaem's production dielectric thickness, permittivity, plated copper and mask must be confirmed before recalculating this geometry or claiming controlled impedance. Connector antipads, pad escapes, bends, tuning and USBLC6 remain discontinuities outside a uniform cross-section calculation. USBLC6 pin assignments and its 100 nF bypass follow the [ST datasheet](https://www.st.com/resource/en/datasheet/usblc6-2.pdf).

## Source and corrections

The original rule structure was adapted from [Cimos/kicad-druid v1.2.0](https://github.com/Cimos/kicad-druid/blob/8af046d6a1839320a360a4b6790ac11eb27a78ae/JLCPCB/JLCPCB-2L-1oz.kicad_dru), commit `8af046d6a1839320a360a4b6790ac11eb27a78ae`. Its MIT license is preserved in [libraries/LICENSE.kicad-druid.md](libraries/LICENSE.kicad-druid.md).

The local profile corrects three reproduced gaps: a cost rule overriding PTH annular minima, track-only board-edge coverage, and reduced same-net drill spacing. General copper clearance also covers objects beyond track pairs.

Hansaem lists a surcharge for 0.20/0.25 mm via drills in its [additional sample pricing](https://www.hsdgt.com/order/sample.html). The profile warns on via drills below 0.30 mm. These assertions do not override fabrication dimensions; drills below 0.20 mm remain errors. Graphical silk strokes below 0.15 mm are warnings on silkscreen layers only.

The 0.25 mm SMD land floor is a retained project constraint. NPTH holes have no annulus requirement. The USB geometry and its pending stackup confirmation are described above. More specific KiCad constraints are evaluated last; later rules must not accidentally weaken manufacturing limits.

## Project footprint overrides

`fp-lib-table` resolves six standard KiCad library names to the project copies in `libraries/footprints/overrides/`. These copies preserve footprint identifiers while incorporating the 0.15 mm silkscreen strokes and shortened outlines needed by this fabrication profile. Update these copies when changing the corresponding outlines; replacing them with unmodified system footprints can restore mask and edge conflicts. The upstream license is retained in [libraries/LICENSE.KiCad-footprints.md](libraries/LICENSE.KiCad-footprints.md). Board-specific 3D model assignments remain on the PCB.

U1 retains the original nine 1 × 1 mm solder lands, their paste apertures, and all twelve central hole locations. The selected holes are Ø0.40 mm with Ø1.00 mm copper, giving a 0.30 mm annulus. The nearest hole centers are 1.0607 mm apart, leaving 0.6607 mm between drilled edges against the 0.45 mm rule. The pad numbers, GND net, outer module pads, antenna keepout and 3D placement are retained. This hole enlargement is a project-specific manual assembly adaptation, not an Espressif stencil recommendation.

The four LED converter output pours are mirrored on B.Cu and joined by Ø0.60/1.20 mm vias. The switching-node pours remain on F.Cu. The 0.25 mm zone clearance is an explicit custom rule because the general 0.125 mm copper rule otherwise takes precedence over individual zone clearance settings.

## Sensor and battery-sense layout

U8 has a native F.Cu rule area inside its pad ring. It excludes copper pours, tracks and vias from the package center while retaining the pads and B.Cu ground. This is the project implementation of the copper-underfill precaution in [Bosch BNO055 handling and mounting instructions, page 20](https://www.bosch-sensortec.com/media/boschsensortec/downloads/handling_soldering_mounting_instructions/bst-bno055-hs000.pdf); that page does not specify an all-layer copper void. The board-level rule area must move with U8 if the sensor is moved.

Battery sensing uses GPIO3/ADC1_CH2. Keep C45 next to the MCU input and R22/R23 close to that filtered node. RAW branches before D1 and remains connected through R22 when the logic supply is off. Keep its route away from the USB pair reference area. The added vias use 0.60 mm copper and 0.30 mm drills and remain separate from component lands.

## Mounting ears

H1–H4 are Ø3.4 mm non-plated clearance holes for M3 screws passing through the PCB into the enclosure. The ears have 4.5 mm outer radii, leaving 2.8 mm between the drill and the board edge. Each footprint reserves a Ø7 mm screw-head envelope, a Ø7.5 mm courtyard on both sides, and a Ø8 mm copper keepout. They are PCB-only mechanical features excluded from the component BOM and placement files.

The four ear-to-body concave fillets use R0.90 for Hansaem's normal Ø1.8 mm router. The four ears use the same profile and form a rectangular mounting pattern with 109.95 × 86.625 mm hole pitch. The overall PCB bounds are 118.95 × 95.625 mm. Coordinates below use the upper-left of those bounds, with X right and Y down; the corresponding KiCad origin is X=63.85, Y=56.90 mm.

| Hole | X (mm) | Y (mm) |
|---|---:|---:|
| H1, upper left | 4.50 | 4.50 |
| H2, upper right | 114.45 | 4.50 |
| H3, lower right | 114.45 | 91.125 |
| H4, lower left | 4.50 | 91.125 |

All four ears extend sideways and remain between the original top and bottom board edges. The USB, motor and ToF insertion corridors remain clear with 0.5 mm lateral allowance. The Ø7 mm screw-head envelopes clear the screw-terminal bodies by more than 2 mm in plan view; the terminal wire-entry height and selected screw-head height require an assembly check.

The upper-right ear and screw head remain outside the existing ESP32 antenna keepout. The Ø7 mm head envelope is approximately 6.83 mm from the antenna area, so this symmetric placement does not provide the 15 mm enclosure clearance recommended in [Espressif's module placement guidance](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/pcb-layout-design.html#general-principles-of-pcb-layout-for-modules-positioning-a-module-on-a-base-board). RF performance with the final screw and enclosure requires physical verification.

## Validation

The tests require `kicad-cli` and the `pcbnew` module provided by KiCad. `tests/test_pad_via_clearance.py` checks all standalone vias against component copper lands, including same-net pairs that electrical clearance checks permit to overlap. Intentional plated holes within component footprints remain part of their pad geometry. The rule tests generate temporary geometry and check accepted and rejected boundary cases for copper spacing, rings, drills, NPTH pegs, via pricing and silkscreen scope.

From this directory:

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover \
  -s tests -p test_design_rules.py -v
```

Check the actual layout with:

```bash
kicad-cli pcb drc --severity-all --exit-code-violations --all-track-errors \
  --schematic-parity --format json -o /tmp/integrated-controller-drc.json \
  integrated_controller.kicad_pcb
```

The saved-board Hansaem verification, including input hashes, environment, DRC and test results, is in [validation/hansaem_20261111.json](validation/hansaem_20261111.json).

Resolve fabrication errors before release. Review silkscreen and cost warnings separately. Reopen the PCB Editor after external project-rule changes so it loads the saved configuration.
