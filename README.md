# Cooperative Multi-UAV Coverage Mission Planning Platform for Remote Sensing Applications - Path Planning back-end

## Description
The project in this repository is a multi-robot coverage path planning (mCPP) module, utilizing [STC](https://link.springer.com/article/10.1023/A:1016610507833) 
and [DARP](https://github.com/athakapo/DARP).
The implemented algorithm is optimized in order to efficiently cope with real-life multi-UAV coverage missions, utilizing a novel 
optimization scheme based on [simulated annealing](https://www.researchgate.net/publication/6026283_Optimization_by_Simulated_Annealing) algorithm.
The overall methodology achieves state-of-the-art performance in mCPP problem (see comparison with simulated evaluation [here](https://github.com/savvas-ap/cpp-simulated-evaluations)).

An on-line instance of an end-to-end mission planner utilizing this mCPP module can be found here: http://choosepath.ddns.net:9001/

In addition, a demonstrative video of the platform can be found [here](https://www.youtube.com/watch?v=JQrqt1dS4A8), including examples of missions'
creation, management and execution, real-life operations, and indicative results that can be acquired.

![Mission examples](cover.png)

The figure above, shows four examples of missions generated by this mCPP algorithm and visualized through the mission
planning platform mentioned above.

### Highlights:
- Support of multiple robots/vehicles
- Support of convex and non-convex, very complex-shaped polygon regions
- Support of multiple obstacles inside the operational area
- Fair and proportional area allocation of the overall region to each robot/vehicle
- Utilization of the initial positions of the vehicles for both the area allocation and the path planning, in order to make
the methodology more efficient
- Energy aware features (paths' length reduction, turns reduction, avoidance of redundant movements that do not contribute in the coverage, etc.)


## Input/Output:
The algorithm receives as input the following:
- The number of robots/vehicles
- The desired scanning density (scanning density corresponds to the desired distance between two sequential trajectories in meters)
- A polygon Region of Interest (ROI), formatted in WGS84 coordinate system
- A set of obstacles (polygons formatted in WGS84 coordinate system) inside the ROI (optional)
- A boolean variable named pathsStrictlyInPoly, to select mode between (paths strictly in poly/better coverage)
- The initial positions of the vehicles (optional - if not provided, random will be used instead | Note that the number 
  of the initial positions should always be the same as the number of robots/vehicles)
- The desired percentages for proportional area allocation (optional - if not provided, equal will be used instead | Note
  that the number of the percentages should always be the same as the number of robots/vehicles and their sum should be 1)

As an output, the algorithm provides set of waypoints (path), for each vehicle involved in the mission, in order
to cooperatively completely cover the ROI.

## Run the project:
In the src/main/resources folder is included a JSON file, containing input parameters for an example mission with 3 vehicles.
The input variables are included in the JSON with the same order as described above.

### 1st way: run the jar file


In the out/artifacts/mCPP_optimized_DARP_jar you can find the project packed in a jar file. The jar expects as an input the path
for a JSON file as the one included in the resources. To run the jar from its current location with the json in the resources folder
run:

```bash
java -jar mCPP-optimized-DARP.jar "../../../src/main/resources/inputVariables.json"
```

### 2nd way: run the Main.java

The Main class of the project expects as argument the path for such a JSON file as well. To run the main with the JSON file provided
in the resources folder run:

```
"src/main/resources/inputVariables.json"
```

### 3rd way: run the Poly2Waypoints.java
In the src/test.java folder you can find the Poly2Waypoints class. In this class you can find the definition of some example
input variables to run the project, with the same order as described above.

### Additional tools
A simple map tool where you can create polygons over a map and copy the coordinates, and a simple matlab code to visualize
the output of this project are also included in the "src/main/resources/" folder.


## Cite as:

```
S. D. Apostolidis, P. Ch. Kapoutsis, A. Ch. Kapoutsis, E. B. Kosmatoupoulos,
“Cooperative Multi-UAV Coverage Mission Planning Platform for Remote Sensing Applications”,
“Autonomous Robots”, Under Review
```


