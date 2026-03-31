\# Data



This directory is intentionally empty in the public repository.

Large LiDAR point cloud files (`.las`, `.laz`) are not committed to Git.



\---



\## Dataset Description



The algorithm expects a classified 3D LiDAR point cloud in \*\*LAS format\*\*.



\### Source



The point cloud dataset used in the published paper was collected in the area of

\*\*Victoriaville, Quebec, Canada\*\* by \[Groupe Trifide](https://www.groupetrifide.com/).



\- \*\*Horizontal coordinate system:\*\* NAD1983 MTM Zone 7

\- \*\*Vertical coordinate system:\*\* NAD1983

\- \*\*Annotation tool:\*\* CloudCompare

\- \*\*Total semantic classes:\*\* 35 distinct classes



\### LAS File Requirements



Your `.las` file must contain:



| Attribute | Description |

|---|---|

| `x`, `y`, `z` | 3D coordinates in a projected coordinate system (meters) |

| `Label` | Semantic class label stored as a float field |



The field must be named \*\*`Label`\*\* exactly (case-sensitive), as accessed via `las\_file.Label`.



\### Navigable Class Codes



The following Label values are treated as navigable surfaces by default:



| Code | Surface Type |

|---|---|

| 1111.00 | Sidewalk |

| 1112.00 | Path |

| 1113.00 | Parking lot |

| 1114.00 | Bare surface |

| 1115.00 | Curb cut |

| 1116.00 | Crosswalk |

| 1810.00 | Additional navigable surface |



All other class codes are treated as obstacles (vegetation, buildings, roads, etc.).

Customize the list in `NavigationConfig.navigable\_classes`.



\### How to Prepare Your Own Dataset



1\. Collect or download a 3D LiDAR point cloud of a pedestrian area.

2\. Semantically annotate it using a tool such as \[CloudCompare](https://www.cloudcompare.org/).

3\. Assign class codes to a field named `Label`.

4\. Export as a `.las` file.

5\. Place the file in this `data/` directory.

6\. Update the path in `examples/run\_example.py`.



\### Recommended Parameter Settings



| Scenario | `open\_space\_grid\_size` | `step\_size` | Notes |

|---|---|---|---|

| Narrow passages / building entrances | 0.05–0.10 m | 0.20–0.30 m | Captures small obstacles and terrain transitions |

| Open sidewalks | 0.05–0.20 m | 0.30–0.50 m | Balances accuracy and speed |

| Ramp areas | 0.05–0.10 m | 0.20–0.30 m | Captures rapid slope changes |



\---



\## Contact for Dataset Access



For access to the Victoriaville dataset used in the paper, contact:

\*\*ali.ahmadi.2@ulaval.ca\*\*

