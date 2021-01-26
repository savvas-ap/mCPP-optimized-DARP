## Cooperative Multi-UAV Coverage Mission Planning Platform for Remote Sensing Applications - Path Planning back-end


![Mission examples](cover.png)


### Description
The project in this repository is a back-end, multi-robot coverage path planning (mCPP) module, utilizing STC 
[ https://link.springer.com/article/10.1023/A:1016610507833 ] and DARP [ https://kapoutsis.info/wp-content/uploads/2017/02/j3.pdf ].
The implemented algorithm is optimized in order to efficiently cope with real-life multi-UAV coverage missions. The
most important contribution of this work is a grid placement optimization scheme, utilizing simulated annealing algorithm
[ https://www.researchgate.net/publication/6026283_Optimization_by_Simulated_Annealing ]. This optimization procedure can
help to efficiently apply any grid-based path planning methodology in complex-shaped polygon regions, overcoming discretization
issues and making them directly applicable to real-world problems. The implementation of this optimization procedure has
managed to achieve state-of-the-art performance for the mCPP algorithm (see also [ https://github.com/savvas-ap/cpp-simulated-evaluations ]).
In addition, a turn reduction methodology is applied in order to make the generated paths more efficient. 

A journal has been submitted, presenting a "mutli-UAV coverage mission planning platform for remote sensing applications"
that is currently under review (S. D. Apostolidis, P. Ch. Kapoutsis, A. Ch. Kapoutsis, E. B. Kosmatoupoulos, “Cooperative
Multi-UAV Coverage Mission Planning Platform for Remote Sensing Applications”, “Autonomous Robots”, Under Review). In 
this work, the mCPP algorithm included in this repository is presented, thoroughly explained and evaluated.  Please,
in case you use this code keep in mind to cite this work when published.

The first figure above, shows four examples of missions generated by this mCPP algorithm and visualized trough the mission
planning platform mentioned above.

#### Features of the CPP methodology:
- Support of multiple robots/vehicles
- Support of convex and non-convex, very complex-shaped polygon regions
- Support of multiple obstacles inside the operational area
- Fair and proportional area allocation of the overall region to each robot/vehicle
- Utilization of the initial positions of the vehicles for both the area allocation and the path planning, in order to make
the methodology more efficient
- Energy aware features (paths' length reduction, turns reduction, avoidance of redundant movements that do not contribute in the coverage, etc.)

#### mCPP method's architecture:
The figure below shows the data pipeline of the back-end mCPP module.
![Back-end architecture](backend.jpg)

The algorithm receives as input the following:
- A polygon Region of Interest (ROI), formatted in WGS84 coordinate system
- A set of obstacles (polygons formatted in WGS84 coordinate system) inside the ROI (optional)
- The desired scanning density (scanning density corresponds to the desired distance between two sequential trajectories in meters)
- The number of robots/vehicles
- The initial positions of the vehicles (optional - if not provided, random will be used instead)
- The desired percentages for proportional area allocation (optional - if not provided, equal will be used instead)
- A boolean variable named pathsStrictlyInPoly (default true), to select mode between (paths strictly in poly/better coverage)

After that, the algorithm performs the following tasks, as shown in the figure above:
- Transformation of all WGS84 coordinates to a local NED system with a common reference point
- Calculation of the optimal grid's placement, given the polygon ROI and the asked scanning density
- Representation of the ROI on grid (every cell of the grid gets a status/tag of (i) robot, (ii) obstacle or (iii) free
  space, that will be used for the area allocation and the path planning procedures)
- Given the optimal ROI's representation on grid, DARP algorithm performs the area allocation. After this step each robot/vehicle
gets assigned with an exclusive sub-region (part of the overall area) to cover
- For each vehicle is created a coverage path inside its exclusive sub-region, utilizing STC (as a first step a MST is
  generated and after that, a path that circumnavigates this MST is created / at this step is also performed the turns'
  reduction procedure)
- From the generated paths are selected only the turning points as waypoints for the robots/vehicles. These waypoints get
  transformed back to WGS84 coordinates
  
And this way, the algorithm provides as output a set of waypoints (path), for each vehicle involved in the mission, in order
to cooperatively completely cover the ROI.

#### How to run the code:
Poly2Waypoints.java contains an example mission for a specific polygon ROI that produces coverage paths for three vehicles.
In the code there are some lines commented, that can be used to skip some information such as the:
- Obstacles
- Specific initial positions
- User-defined percentages for the area allocation

"missionWaypoints" variable contains the coverage paths that are generated for all three vehicles, for the given ROI and
mission's specifications. In the resources folder you can also find a map where you can create polygons over a map and copy the
WGS84 coordinates and a Matlab code that can be used to visualize ROI's with obstacles and the generated paths. These two
simple tools can be used for experimentation with the code.

Finally, for everyone who wants to intervene with this code, the src/main/java/pathPlanning/darp/DARPinPoly.java file, along
with the method's architecture figure, are a good starting point.