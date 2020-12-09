package pathPlanning.darp;

import pathPlanning.handleGeo.*;
import pathPlanning.nodesPlacementOptimization.*;

import java.awt.*;
import java.util.ArrayList;
import java.util.Random;
import java.util.Vector;


public class DARPinPoly {

    private ArrayList<ArrayList<double[]>> missionWaypointsNED = new ArrayList<>();
    private ArrayList<ArrayList<double[]>> missionWaypointsWGS84 = new ArrayList<>();
    private int droneNo;
    private boolean pathsStrictlyInPoly;
    private int[][] DARPgrid;
    private double[][] geo;


    public void waypointsDARP(int droneNo, int scanDist, double[][] geo, double[][][] obstacles, boolean pathsStrictlyInPoly) {
        common(droneNo, scanDist, geo, obstacles, pathsStrictlyInPoly, new double[][]{}, true, new double[]{}, false);
    }

    public void waypointsDARP(int droneNo, int scanDist, double[][] geo, double[][][] obstacles, boolean pathsStrictlyInPoly, double[][] initialPos) {
        common(droneNo, scanDist, geo, obstacles, pathsStrictlyInPoly, initialPos, false, new double[]{}, false);
    }

    public void waypointsDARP(int droneNo, int scanDist, double[][] geo, double[][][] obstacles, boolean pathsStrictlyInPoly, double[] Rportions) {
        common(droneNo, scanDist, geo, obstacles, pathsStrictlyInPoly, new double[][]{}, true, Rportions, true);
    }

    public void waypointsDARP(int droneNo, int scanDist, double[][] geo, double[][][] obstacles, boolean pathsStrictlyInPoly, double[][] initialPos, double[] Rportions) {
        common(droneNo, scanDist, geo, obstacles, pathsStrictlyInPoly, initialPos, false, Rportions, true);
    }

    private void common(int droneNo, int scanDist, double[][] geo, double[][][] obstacles, boolean pathsStrictlyInPoly, double[][] initialPos, boolean randomInitPos, double[] Rportions, boolean notEqualPortions) {

        this.droneNo = droneNo;
        this.pathsStrictlyInPoly = pathsStrictlyInPoly;
        this.geo = geo;

        // Convert WGS84 to NED coordinates
        ConvCoords missionDARP = new ConvCoords();
        missionDARP.polygonWGS84ToNED(geo); // Convert geographical to local cartesian coordinates
        double[][] cartUnrotated = missionDARP.getPolygonNED();
        double[][][] obstNED = new double[][][]{};
        if (obstacles.length > 0) {
            missionDARP.obstaclesToNED(obstacles);
            obstNED = missionDARP.getObstaclesNED();
        }

        // Rotation and shift optimization
        long start = System.nanoTime();
        SimulatedAnnealing optimalParameters = new SimulatedAnnealing();
        optimalParameters.run(cartUnrotated, obstNED, scanDist);
        Rotate rotate = new Rotate();
        float theta = optimalParameters.getOptimalTheta();
        float shiftX = optimalParameters.getOptimalShiftX();
        float shiftY = optimalParameters.getOptimalShiftY();
        rotate.setTheta(theta);
        double[][] cart = rotate.rotatePolygon(cartUnrotated);
        double[][][] cartObst = new double[obstNED.length][][];
        for (int i=0; i<obstNED.length; i++){
            cartObst[i] = rotate.rotatePolygon(obstNED[i]);
        }
        System.out.println("Time needed to find the optimal solution: " + (System.nanoTime() - start) / 1000000000.0f + " seconds");
        System.out.println(" - Optimal theta: " + theta + "\n - Optimal shift in X axis: " + shiftX +
                "\n - Optimal shift in Y axis: " + shiftY + "\n");

        // Build grid for paths
        NodesInPoly mission = new NodesInPoly(cart, cartObst, scanDist, pathsStrictlyInPoly, false, shiftX, shiftY);
        System.out.println("User's defined polygon area: " + mission.getPolygonArea() + " square meters\n");

        // Cases that cannot run
        int megaNodesIn = mission.getMegaNodesInCount();
        if (megaNodesIn < 1) {
            System.out.println("\n\n !!! The defined area is too small to fit path for the asked scanning density!" +
                    " Try again a larger area or a smaller scanning density !!! \n\n");

            return;
        } else if (megaNodesIn < droneNo) {
            System.out.println("\n\n !!! Not enough space to have at least minimum length paths for every drone !!!\n" +
                    "          With this configuration you can deploy " + megaNodesIn + " drones at most\n" +
                    "    Number of drones automatically changed to " + megaNodesIn + " in order to have a solution\n" +
                    "\n   If you are not satisfied with the solution you could try to rerun for:\n   - Larger area\n   - Smaller scanning distance\n\n");

            droneNo = megaNodesIn;
            this.droneNo = megaNodesIn;
        }

        double[][][] megaNodes = mission.getMegaNodes();
        double[][][] subNodes = mission.getSubNodes();

        // DARP parameters
        int l = megaNodes.length;
        int m = megaNodes[0].length;            //  In DARPgrid 0 stands for free space
        DARPgrid = new int[l][m];               //  1 stands for Obstacle
                                                //  2 stands for Drone
//        System.out.println("\nPolygon ROI represented on grid:");
        for (int i = 0; i < l; i++) {
            for (int j = 0; j < m; j++) {
                DARPgrid[i][j] = (int) megaNodes[i][j][2];
//                System.out.print(DARPgrid[i][j]+" ");   // Uncomment to see grid
            }
//            System.out.println();   // Uncomment to see grid
        }

        // Check for grid connectivity
        ConnectComponent G2G = new ConnectComponent();
        int[][] connectivityTest = new int[l][m];
        for (int i = 0; i < l; i++) {
            for (int j = 0; j < m; j++) {
                connectivityTest[i][j] = Math.abs(DARPgrid[i][j] - 1);
            }
        }
        G2G.compactLabeling(connectivityTest, new Dimension(m, l), true);
        if (G2G.getMaxLabel() > 1) {
            System.out.println("\n\n !!! The environment grid MUST not have unreachable and/or closed shape regions !!! \n\n");
            return;
        }

        // Put drones in initial positions (close to physical or random)
        initializeDARPGrid(randomInitPos, l, m, initialPos, missionDARP, megaNodes, theta, shiftX, shiftY);

        // Parameters to run DARP
        int MaxIter = 80000;
        double CCvariation = 0.01;
        double randomLevel = 0.0001;
        int dcells = 2;
        boolean importance = false;

        // If user has not defined custom portions divide area equally for all drones
        if (!notEqualPortions) {
            Rportions = new double[droneNo];
            for (int i = 0; i < droneNo; i++) {
                Rportions[i] = 1.0/droneNo;
            }
        }

        System.out.println("Portion of the total area assigned to each drone:");
        for (int i = 0; i < droneNo; i++) {
            System.out.println(" - "+Rportions[i]*100+" %");
        }
        System.out.println();

        // Perform operational area division (run DARP)
        DARP problem = new DARP(l, m, DARPgrid, MaxIter, CCvariation, randomLevel, dcells, importance, Rportions);

        // Warn when no drone is defined
        System.out.println("Number of drones: " + problem.getNr());
        if (problem.getNr() <= 0) {
            System.out.println("\n\n !!! You should use at least one drone !!!\n");
            return;
        }

        // Check if DARP could find a solution - if not --> rerun for random initial positions
        int[][] DARPAssignmentMatrix = new int[0][];
        problem.constructAssignmentM();
        if (problem.getSuccess()) {
            DARPAssignmentMatrix = problem.getAssignmentMatrix();
        } else {
            int count = 0;
            while (!problem.getSuccess() && count < 5) {
                count++;
                System.out.println("\n\n !!! DARP will rerun for random initial positions !!! \n");

                DARPgrid = new int[l][m];
                for (int i = 0; i < l; i++) {
                    for (int j = 0; j < m; j++) {
                        DARPgrid[i][j] = (int) megaNodes[i][j][2];
                    }
                }
                initializeDARPGrid(true, l, m, initialPos, missionDARP, megaNodes, theta, shiftX, shiftY);

                problem = new DARP(l, m, DARPgrid, MaxIter, CCvariation, randomLevel, dcells, importance, Rportions);
                problem.constructAssignmentM();
                DARPAssignmentMatrix = problem.getAssignmentMatrix();
            }
            if (count == 5) {
                System.out.println("\nDARP did not manage to find a solution for the given configuration!\n Try to alter one or more of:" +
                        "\n  - Scanning distance\n  - Number of drones \n  - Given area\n\n");
                return;
            }
        }

        // Calculate paths for all drones, for all modes (see below) and keep the paths with the minimum turns
        ArrayList<ArrayList<double[]>>[] allDirectionsWaypoints = new ArrayList[droneNo];
        for (int k = 0; k < droneNo; k++) {
            allDirectionsWaypoints[k] = new ArrayList<>();
        }
        for (int mode = 0; mode < 4; mode++) {

            // Calculate MSTs for all drones
            // ------------------------------------------------------------------------------------------ //
            // mode is a variable that configures the side that the branches of the MST will be connected //
            //            this variable is used to minimize the turn points for every drone               //
            //  0 - connection on top                                                                     //
            //  1 - connection on bottom                                                                  //
            //  2 - connection on right                                                                   //
            //  3 - connection on left                                                                    //
            // ------------------------------------------------------------------------------------------ //
            ArrayList<Vector> MSTs = calculateMSTs(problem.getBinrayRobotRegions(), droneNo, l, m, mode);

            // Find paths around MSTs (circumnavigation)
            ArrayList<Integer[]> InitRobots = problem.getRobotsInit();  // Initial positions of drones
            ArrayList<ArrayList<Integer[]>> AllRealPaths = new ArrayList<>();
            for (int r = 0; r < droneNo; r++) {
                CalculateTrajectories ct = new CalculateTrajectories(l, m, MSTs.get(r)); //Send MSTs
                ct.initializeGraph(CalcRealBinaryReg(problem.getBinrayRobotRegions().get(r), l, m), true); //Send [x2 x2] Binary Robot Region
                ct.RemoveTheAppropriateEdges();
                ct.CalculatePathsSequence(4 * InitRobots.get(r)[0] * m + 2 * InitRobots.get(r)[1]);
                AllRealPaths.add(ct.getPathSequence());
            }

            // ---------------------------------------------------------------------------------------------------------
            // Type of lines is a 2*l x 2*m x 2 matrix - [2*l][2*m][0] contains the clockwise path
            //                                           [2*l][2*m][1] contains the counterclockwise path
            // ---------------------------------------------------------------------------------------------------------
            // In both matrices: 0 stands for obstacle or starting ending points
            //                   1 stands for up movement
            //                   2 stands for left movement
            //                   3 stands for right movement
            //                   4 stands for down movement
            // ---------------------------------------------------------------------------------------------------------
            int[][][] TypesOfLines = new int[l * 2][m * 2][2];
            int indxadd1;
            int indxadd2;

            for (int r = 0; r < droneNo; r++) {
                boolean flag = false;   // This variable is used to fix a bug that came along with darp
                for (Integer[] connection : AllRealPaths.get(r)) {
                    if (flag) {
                        if (TypesOfLines[connection[0]][connection[1]][0] == 0) {
                            indxadd1 = 0;
                        } else {
                            indxadd1 = 1;
                        }
                        if (TypesOfLines[connection[2]][connection[3]][0] == 0 && flag) {
                            indxadd2 = 0;
                        } else {
                            indxadd2 = 1;
                        }
                    } else {
                        if (!(TypesOfLines[connection[0]][connection[1]][0] == 0)) {
                            indxadd1 = 0;
                        } else {
                            indxadd1 = 1;
                        }
                        if (!(TypesOfLines[connection[2]][connection[3]][0] == 0 && flag)) {
                            indxadd2 = 0;
                        } else {
                            indxadd2 = 1;
                        }
                    }

                    flag = true;

                    if (connection[0].equals(connection[2])) { // Horizontal connection (Line types: 2,3)
                        if (connection[1] > connection[3]) {
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 2;
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 3;
                        } else {
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 3;
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 2;
                        }
                    } else { // Vertical connection (Line types: 1,4)
                        if (connection[0] > connection[2]) {
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 1;
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 4;
                        } else {
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 4;
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 1;
                        }
                    }
                }
            }

            // Clockwise path
//            System.out.println();
            for (int i = 0; i < TypesOfLines.length; i++) {
                for (int j = 0; j < TypesOfLines[0].length; j++) {
                    subNodes[i][j][2] = TypesOfLines[i][j][0];
//                System.out.print((int)subNodes[i][j][2]+" ");
                }
//            System.out.println();
            }

//            // Counterclockwise path
//            System.out.println();
//            for (int i = 0; i < TypesOfLines.length; i++) {
//                for (int j = 0; j < TypesOfLines[0].length; j++) {
//                    subNodes[i][j][2] = TypesOfLines[i][j][1];
//                    System.out.print((int)subNodes[i][j][2]+" ");
//                }
//                System.out.println();
//            }
            // ------------------------------------------------------------------------------------------------------------

            // From TypesOfLines to lists of waypoints
            int xInit;  // Coordinates for initial position
            int yInit;  // of drones in the sub-cells

            int[][] subCellsAssignment = new int[l * 2][m * 2];
            for (int i = 0; i < l; i++) {
                for (int j = 0; j < m; j++) {
                    subCellsAssignment[2 * i][2 * j] = DARPAssignmentMatrix[i][j];
                    subCellsAssignment[2 * i + 1][2 * j] = DARPAssignmentMatrix[i][j];
                    subCellsAssignment[2 * i][2 * j + 1] = DARPAssignmentMatrix[i][j];
                    subCellsAssignment[2 * i + 1][2 * j + 1] = DARPAssignmentMatrix[i][j];
                }
            }

            ArrayList<double[]> iWaypoints;
            double prevState;
            int i;
            int j;

            for (int k = 0; k < droneNo; k++) {
                iWaypoints = new ArrayList<>();
                // Compute waypoints for every drone
                xInit = 2 * InitRobots.get(k)[0];
                yInit = 2 * InitRobots.get(k)[1];
//                System.out.println("Initial positions: "+xInit+","+ yInit);

                i = xInit;
                j = yInit;

                iWaypoints.add(new double[]{subNodes[xInit][yInit][0], subNodes[xInit][yInit][1]});
                do {

                    prevState = subNodes[i][j][2];

                    if (subNodes[i][j][2] == 1.0) {
                        i--;
                    } else if (subNodes[i][j][2] == 2.0) {
                        j--;
                    } else if (subNodes[i][j][2] == 3.0) {
                        j++;
                    } else if (subNodes[i][j][2] == 4.0) {
                        i++;
                    }

                    if (prevState != subNodes[i][j][2] || (subNodes[i][j][0] == iWaypoints.get(0)[0] && subNodes[i][j][1] == iWaypoints.get(0)[1])) {
                        iWaypoints.add(new double[]{subNodes[i][j][0], subNodes[i][j][1]});
                    }

                } while (!(i == xInit && j == yInit));
                ArrayList<double[]> WP = rotate.rotateBackWaypoints(iWaypoints);
                allDirectionsWaypoints[k].add(WP);
            }
        }

        // Keep orientation with less turns
        int ind;
        int min;
        for (int i = 0; i < droneNo; i++) {
            ind = 0;
            min = allDirectionsWaypoints[i].get(0).size();
            for (int j = 1; j < 4; j++) {
                if (allDirectionsWaypoints[i].get(j).size() < min) {
                    min = allDirectionsWaypoints[i].get(j).size();
                    ind = j;
                }
            }
            missionWaypointsNED.add(allDirectionsWaypoints[i].get(ind));
        }
        missionDARP.NEDToWGS84(missionWaypointsNED);
        missionWaypointsWGS84 = missionDARP.getWaypointsWGS84();

        printWaypointsNum(missionWaypointsWGS84);
//        printWaypointsNED();
//        printWaypointsWGS84();
    }

    private void initializeDARPGrid(boolean randomInitPos, int l, int m, double[][] initialPos, ConvCoords missionDARP, double[][][] megaNodes, float theta, float shiftX, float shiftY) {
        if (randomInitPos) {

            // Add drones in random initial positions
            int i1;
            int i2;
            int c = 0;
            while (c < droneNo) {
                // Initial positions of drones
                Random ind1 = new Random();
                i1 = ind1.nextInt(l);
                Random ind2 = new Random();
                i2 = ind2.nextInt(m);
                if (DARPgrid[i1][i2] == 0) {
                    DARPgrid[i1][i2] = 2;
                    c++;
                }
            }

        } else {

            // Add drones in the closest mega-node
            int i1 = 0;
            int i2 = 0;

            // Convert initial positions from WGS84 to NED
            double[][] initPosNED = missionDARP.convWGS84ToNED(initialPos);

            // Rotate and shift intial positions
            Rotate rotate = new Rotate();
            rotate.setTheta(theta);
            double[][] initialPosNED = rotate.rotatePolygon(initPosNED);
            for (int i=0; i<initPosNED.length; i++){
                initialPosNED[i][0] += shiftX;
                initialPosNED[i][1] += shiftY;
            }

            for (int i = 0; i < initialPosNED.length; i++) {
                double minDist = Double.MAX_VALUE;
                for (int j = 0; j < l; j++) {
                    for (int k = 0; k < m; k++) {
                        double distance = Dist.euclidean(initialPosNED[i], new double[]{megaNodes[j][k][0], megaNodes[j][k][1]});
                        if (distance < minDist && megaNodes[j][k][2] == 0) {
                            minDist = distance;
                            i1 = j;
                            i2 = k;
                        }
                    }
                }
                DARPgrid[i1][i2] = 2;
                megaNodes[i1][i2][2] = -1;
            }

        }
    }

    private ArrayList<Vector> calculateMSTs(ArrayList<boolean[][]> BinrayRobotRegions, int nr, int rows, int cols, int mode) {
        ArrayList<Vector> MSTs = new ArrayList<>();
        for (int r = 0; r < nr; r++) {
            Kruskal k = new Kruskal(rows * cols);
            k.initializeGraph(BinrayRobotRegions.get(r), true, mode);
            k.performKruskal();
            MSTs.add(k.getAllNewEdges());
        }

        return MSTs;
    }

    private boolean[][] CalcRealBinaryReg(boolean[][] BinrayRobotRegion, int rows, int cols) {
        boolean[][] RealBinrayRobotRegion = new boolean[2 * rows][2 * cols];
        for (int i = 0; i < 2 * rows; i++) {
            for (int j = 0; j < 2 * cols; j++) {
                RealBinrayRobotRegion[i][j] = BinrayRobotRegion[i / 2][j / 2];
            }
        }
        return RealBinrayRobotRegion;
    }

    private void print(ArrayList<ArrayList<double[]>> waypoints) {
        for (int k = 0; k < droneNo; k++) {
            System.out.println("\n ~ Number of Waypoints: " + waypoints.get(k).size() + " ~ ");
            for (int jj = 0; jj < waypoints.get(k).size(); jj++) {
                System.out.println(waypoints.get(k).get(jj)[0] + ", " + waypoints.get(k).get(jj)[1] + ";");
            }
        }
    }

    private void printWaypointsNum(ArrayList<ArrayList<double[]>> waypoints) {
        for (int k = 0; k < droneNo; k++) {
            System.out.println("- Number of Waypoints for drone " + (k + 1) + ": " + waypoints.get(k).size());
        }
        System.out.println();
    }


    public ArrayList<ArrayList<double[]>> getMissionWaypointsNED() {
        return missionWaypointsNED;
    }

    public ArrayList<ArrayList<double[]>> getMissionWaypointsWGS84() {
        return missionWaypointsWGS84;
    }

    public void printWaypointsNED() {
        print(missionWaypointsNED);
    }

    public void printWaypointsWGS84() {
        print(missionWaypointsWGS84);
    }

}
