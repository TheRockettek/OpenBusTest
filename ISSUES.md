# OpenBus Issues

- [x] Holding the horn should keep the horn sound looping until release.
- [x] Cockpit indicator and interior lights should be visible at startup.
- [ ] Changing FOV should not flash the cockpit bus texture.
- [ ] Ticketer time should use the authored texture/font and correct position.
- [ ] Clicking the top of a cockpit button should press it down; clicking the bottom should press it up.
- [ ] Some installed OMSI vehicle configurations contain ambiguous material records. The
  2026-10-28 game log reports empty `[matl_transmap]` records in
  `MAN_SD200/Model/model_SD84_AI_X10.cfg` (lines 588, 1677) and
  `MAN_SD202/Model/model_D87_druckerFF.cfg` (4137) and `model_D92.cfg` (4552).
  The SD202 files also contain `[matl_item]` after `[matl]`, without a
  preceding `[matl_change]` (D87 line 7588 and D92 line 8166, repeated on
  other LOD doors). Their intended texture or selection state cannot be inferred
  safely; diagnostics remain rather than inventing material behavior.
- [ ] The same log reports a missing `Spiegel.tga` for
  `Vehicles/X10 Zafira/model/Zafira_body.o3d`. No matching filename or stem
  was found in that vehicle's installed texture directories; restoring the
  authored asset may be necessary.
- [ ] Visually verify mirrored BRT Berlin roads and compare native OMSI
  behavior. OpenBus now reverses unidirectional lanes on mirrored roads along
  with their offsets; bidirectional and pedestrian paths are unchanged.
  `OPENBUS_MIRRORED_ROAD_DIRECTIONS=authored` restores the former behavior.
  A reproducible example is `tile_-3_3.map` spline `10713186`, which uses a
  `mirror` marker with `X6.5_2spurE_Mahlower.sli`. The correction and vehicle
  heading are covered by focused probes but have not been visually confirmed.
  A parallel one-way carriageway example is `tile_-13_12.map` splines
  `8299355` and `8299357`; their pedestrian paths are both bidirectional.
