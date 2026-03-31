Example: Sidewalk centerline navigation on a LAS point cloud.



Usage

\-----

1\. Place your .las file inside the data/ directory.

2\. Update las\_file\_path, start\_point, and end\_point below.

3\. Run from the repo root:

&#x20;      python examples/run\_example.py



Output

\------

A CSV file is written to results/ with per-step terrain and navigation features.

See results/README.md for column descriptions.



Reference

\---------

Ahmadi et al. (2026). Characterizing Pedestrian Network from Segmented

3D Point Clouds for Accessibility Assessment: A Virtual Robotic Approach.

Sensors, 26. https://doi.org/10.3390/s26072172 

"""



import sys

import os

sys.path.insert(0, os.path.join(os.path.dirname(\_\_file\_\_), ".."))



from src.centerline\_navigation import NavigationConfig, CenterlineNavigator



\# ── Configuration ──────────────────────────────────────────────────────────────

config = NavigationConfig(

&#x20;   # Robot footprint

&#x20;   triangle\_side\_length=0.8,       # equilateral triangle side length (m) \~ wheelchair footprint

&#x20;   step\_size=0.3,                  # forward distance per step (m)

&#x20;   robot\_width=0.8,                # physical robot width (m)



&#x20;   # Navigation

&#x20;   sensor\_range=5.0,               # maximum ray-cast distance (m)

&#x20;   goal\_tolerance=0.2,             # distance threshold to declare goal reached (m)



&#x20;   # Navigable surface class codes (Label field in the LAS file)

&#x20;   navigable\_classes=\[

&#x20;       1111.00,  # Sidewalk

&#x20;       1112.00,  # Path

&#x20;       1113.00,  # Parking lot

&#x20;       1114.00,  # Bare surface

&#x20;       1115.00,  # Curb cut

&#x20;       1116.00,  # Crosswalk

&#x20;       1810.00,  # Additional navigable surface

&#x20;   ],



&#x20;   # Point cloud resolution

&#x20;   open\_space\_grid\_size=0.05,      # spatial resolution for obstacle detection (m)



&#x20;   # Width and clearance

&#x20;   min\_passage\_width=0.6,          # minimum usable corridor width (m) — ADA: 0.91 m

&#x20;   obstacle\_avoidance\_distance=0.5,

&#x20;   centerline\_samples=36,          # number of radial directions sampled around robot



&#x20;   # Lateral clearance targets

&#x20;   desired\_side\_distance=0.5,      # target clearance to each edge (m)

&#x20;   min\_side\_distance=0.45,         # hard minimum clearance (m)



&#x20;   # Heading smoothing (anti-zigzag)

&#x20;   heading\_smooth\_alpha=0.2,       # 0 = no smoothing, 1 = instant update

&#x20;   center\_error\_gain=0.1,

&#x20;   distance\_error\_gain=0.2,



&#x20;   # High-precision ray casting

&#x20;   ray\_coarse\_step=0.05,           # coarse step for ray marching (m)

&#x20;   ray\_refine\_iters=6,             # binary-search refinement iterations

&#x20;   width\_fan\_half\_angle\_deg=15.0,  # fan ±15° around the perpendicular direction

&#x20;   width\_fan\_rays=5,               # rays per side in the fan

)



\# ── Input / output paths ───────────────────────────────────────────────────────

las\_file\_path = "data/your\_point\_cloud.las"   # ← update this



\# Coordinates in the LAS file's projected CRS (x, y, z in meters)

\# Example uses the Victoriaville dataset from the paper (NAD1983 MTM Zone 7)

start\_point = \[193328.856628, 5102483.658905, 143.8510]

end\_point   = \[193311.138123, 5102503.181488, 144.1012]



output\_file = "results/run\_output.csv"        # ← update this for each run



\# ── Run ────────────────────────────────────────────────────────────────────────

os.makedirs("results", exist\_ok=True)



navigator = CenterlineNavigator(config)

navigator.load\_point\_cloud(las\_file\_path)

navigator.initialize\_robot(start\_point, end\_point)



success = navigator.navigate(max\_steps=2000)



navigator.save\_results(output\_file)



if success:

&#x20;   print("\\\\nNavigation successful — goal reached.")

else:

&#x20;   print("\\\\nNavigation ended without reaching the goal (max\_steps exceeded).")

