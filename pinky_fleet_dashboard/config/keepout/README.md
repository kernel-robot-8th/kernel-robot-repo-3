# map-proto Keepout mask

`keepout_mask.pgm` is a separate Nav2 filter mask. The original occupancy map is unchanged.

The two rectangles were estimated from the user's yellow annotations in `map-proto` coordinates:

- Upper room band: `x=[-0.20, 1.80] m`, `y=[1.86, 2.19] m`
- Cross-room strip: `x=[-0.20, 1.36] m`, `y=[0.89, 1.00] m`

The mask is aligned to `/home/gigi/Documents/Pinky_backup/map-proto.yaml` (`resolution=0.05 m`, `origin=[-0.349, -0.295, 0]`, 44 by 51 cells). Dark cells are forbidden; white cells remain traversable. Regenerate it after editing the map or the rectangles with:

```bash
python3 scripts/create_keepout_mask.py
```

The dashboard enables this mask only when `map-proto.yaml` is selected. `my_map.yaml` and `pinklab.yaml` continue to use their existing navigation configuration because their alignment with these rectangles has not been confirmed.
