package pathPlanning.handleGeo;

import pathPlanning.darp.ConnectComponent;
import pathPlanning.helpers.MinMax;

import java.awt.*;

//import java.util.concurrent.Executor;
//import java.util.concurrent.Executors;
//import java.util.concurrent.ScheduledExecutorService;

public class NodesInPoly {

    private double[][] polygonCoordinates;
    private double[][][] cartObst;

    private double shiftX;
    private double shiftY;

    private double xRngMeters;
    private double yRngMeters;

    private int xNodes;
    private int yNodes;
    private int megaNodesCount;

    private double xMax;
    private double xMin;
    private double yMax;
    private double yMin;

    private double[][][] megaNodes;
    private double[][][] subNodes;

    private int nodeDistance;
    private float nodeIntervalOffset;

//    ScheduledExecutorService executor = Executors.newScheduledThreadPool(10);

    public NodesInPoly(double[][] cartCoords, double[][][] cartObst, int scanDist, boolean pathsStrictlyInPoly, boolean hideInfo, double shiftX, double shiftY) {

        this.cartObst = cartObst;
        this.shiftX = shiftX;
        this.shiftY = shiftY;
        nodeDistance = 2 * scanDist;
        nodeIntervalOffset = scanDist / 2.0f;
        polygonCoordinates = cartCoords;

        xMax = MinMax.xMax(polygonCoordinates) + nodeDistance;
        xMin = MinMax.xMin(polygonCoordinates) - nodeDistance;
        yMax = MinMax.yMax(polygonCoordinates) + nodeDistance;
        yMin = MinMax.yMin(polygonCoordinates) - nodeDistance;

        xRngMeters = xMax - xMin;
        yRngMeters = yMax - yMin;

        xNodes = (int) (xRngMeters / nodeDistance);
        yNodes = (int) (yRngMeters / nodeDistance);

        if (!hideInfo) {
            System.out.println("Bounding box: " + xRngMeters + " x " + yRngMeters + " meters");
            System.out.println("xNodes: " + xNodes);
            System.out.println("yNodes: " + yNodes);
            System.out.println("Total number of mega-nodes: " + xNodes * yNodes);
        }

        double xInter = xMax - xMin - (xNodes - 1) * nodeDistance;
        double yInter = yMax - yMin - (yNodes - 1) * nodeDistance;

        if (pathsStrictlyInPoly) {
            strictlyInPoly(xMin, yMin, nodeDistance, hideInfo);
        } else {
            betterCoverage(xMin, yMin, nodeDistance, hideInfo);
        }

//        removeObstacles();

    }

    private void strictlyInPoly(double xMin, double yMin, int nodeDistance, boolean hideInfo) {

        megaNodesCount = 0;
        megaNodes = new double[xNodes][yNodes][3];
        for (int i = 0; i < xNodes; i++) {
            for (int j = 0; j < yNodes; j++) {
                megaNodes[i][j][0] = xMin + i * nodeDistance + shiftX;
                megaNodes[i][j][1] = yMin + j * nodeDistance + shiftY;
//                // Uncomment to print the mega-megaNodes
//                if (!hideInfo){
//                    System.out.println(megaNodes[i][j][0]+", "+megaNodes[i][j][1]+" ---> "+megaNodes[i][j][2]);
//                }
            }
        }

        subNodes = new double[2 * xNodes][2 * yNodes][3];
        for (int i = 0; i < xNodes; i++) {
            for (int j = 0; j < yNodes; j++) {

                subNodes[2 * i][2 * j + 1][0] = megaNodes[i][j][0] - nodeIntervalOffset;
                subNodes[2 * i][2 * j + 1][1] = megaNodes[i][j][1] + nodeIntervalOffset;

                subNodes[2 * i + 1][2 * j + 1][0] = megaNodes[i][j][0] + nodeIntervalOffset;
                subNodes[2 * i + 1][2 * j + 1][1] = megaNodes[i][j][1] + nodeIntervalOffset;

                subNodes[2 * i][2 * j][0] = megaNodes[i][j][0] - nodeIntervalOffset;
                subNodes[2 * i][2 * j][1] = megaNodes[i][j][1] - nodeIntervalOffset;

                subNodes[2 * i + 1][2 * j][0] = megaNodes[i][j][0] + nodeIntervalOffset;
                subNodes[2 * i + 1][2 * j][1] = megaNodes[i][j][1] - nodeIntervalOffset;

                if (InPolygon.check(new double[]{subNodes[2 * i][2 * j + 1][0], subNodes[2 * i][2 * j + 1][1]}, polygonCoordinates) &&
                        InPolygon.check(new double[]{subNodes[2 * i + 1][2 * j + 1][0], subNodes[2 * i + 1][2 * j + 1][1]}, polygonCoordinates) &&
                        InPolygon.check(new double[]{subNodes[2 * i][2 * j][0], subNodes[2 * i][2 * j][1]}, polygonCoordinates) &&
                        InPolygon.check(new double[]{subNodes[2 * i + 1][2 * j][0], subNodes[2 * i + 1][2 * j][1]}, polygonCoordinates)) {

                    megaNodes[i][j][2] = 0;
                    megaNodesCount++;
                    checkInPolyAndObstacle(i, j);

                } else {
                    megaNodes[i][j][2] = 1;
                }
                subNodes[2 * i][2 * j + 1][2] = megaNodes[i][j][2];
                subNodes[2 * i + 1][2 * j + 1][2] = megaNodes[i][j][2];
                subNodes[2 * i][2 * j][2] = megaNodes[i][j][2];
                subNodes[2 * i + 1][2 * j][2] = megaNodes[i][j][2];

//                // Uncomment to print all the sub-nodes
//                System.out.println(subNodes[2*i+1][2*j][0]+", "+subNodes[2*i][2*j][1]+" ---> "+subNodes[2*i][2*j][2]);
//                System.out.println(subNodes[2*i+1][2*j][0]+", "+subNodes[2*i+1][2*j][1]+" ---> "+subNodes[2*i+1][2*j][2]);
//                System.out.println(subNodes[2*i][2*j+1][0]+", "+subNodes[2*i][2*j+1][1]+" ---> "+subNodes[2*i][2*j+1][2]);
//                System.out.println(subNodes[2*i+1][2*j+1][0]+", "+subNodes[2*i+1][2*j+1][1]+" ---> "+subNodes[2*i+1][2*j+1][2]);
//
//                // Uncomment to print the sub-nodes that will be used for trajectories
//                if (megaNodes[i][j][2]!=1){
//                    System.out.println(subNodes[2*i][2*j][0]+", "+subNodes[2*i][2*j][1]);
//                    System.out.println(subNodes[2*i+1][2*j][0]+", "+subNodes[2*i+1][2*j][1]);
//                    System.out.println(subNodes[2*i][2*j+1][0]+", "+subNodes[2*i][2*j+1][1]);
//                    System.out.println(subNodes[2*i+1][2*j+1][0]+", "+subNodes[2*i+1][2*j+1][1]);
//                }

            }

        }

        if (!hideInfo) {
            System.out.println("Number of mega-nodes that are used for STC: " + megaNodesCount);
            System.out.println("Number of sub-nodes that will be used for trajectories: " + 4.0 * megaNodesCount);
        }

    }

    private void betterCoverage(double xMin, double yMin, int nodeDistance, boolean hideInfo) {

        megaNodesCount = 0;
        megaNodes = new double[xNodes][yNodes][3];
        for (int i = 0; i < xNodes; i++) {
            for (int j = 0; j < yNodes; j++) {
                megaNodes[i][j][0] = xMin + i * nodeDistance + shiftX;
                megaNodes[i][j][1] = yMin + j * nodeDistance + shiftY;
                if (InPolygon.check(new double[]{megaNodes[i][j][0], megaNodes[i][j][1]}, polygonCoordinates)) {
                    megaNodes[i][j][2] = 0;
                    megaNodesCount++;
                } else {
                    megaNodes[i][j][2] = 1;
                }
//                // Uncomment to print the mega-nodes
//                if (!hideInfo){
//                    System.out.println(megaNodes[i][j][0]+", "+megaNodes[i][j][1]+" ---> "+megaNodes[i][j][2]);
//                }
            }
        }
        if (!hideInfo) {
            System.out.println("Number of mega-nodes inside polygon: " + megaNodesCount);
            System.out.println("Number of sub-nodes that will be used for trajectories: " + 4.0 * megaNodesCount);
        }

        subNodes = new double[2 * xNodes][2 * yNodes][3];
        for (int i = 0; i < xNodes; i++) {
            for (int j = 0; j < yNodes; j++) {

                subNodes[2 * i][2 * j + 1][0] = megaNodes[i][j][0] - nodeIntervalOffset;
                subNodes[2 * i][2 * j + 1][1] = megaNodes[i][j][1] + nodeIntervalOffset;

                subNodes[2 * i + 1][2 * j + 1][0] = megaNodes[i][j][0] + nodeIntervalOffset;
                subNodes[2 * i + 1][2 * j + 1][1] = megaNodes[i][j][1] + nodeIntervalOffset;

                subNodes[2 * i][2 * j][0] = megaNodes[i][j][0] - nodeIntervalOffset;
                subNodes[2 * i][2 * j][1] = megaNodes[i][j][1] - nodeIntervalOffset;

                subNodes[2 * i + 1][2 * j][0] = megaNodes[i][j][0] + nodeIntervalOffset;
                subNodes[2 * i + 1][2 * j][1] = megaNodes[i][j][1] - nodeIntervalOffset;

                checkInPolyAndObstacle(i, j);

                subNodes[2 * i][2 * j + 1][2] = megaNodes[i][j][2];
                subNodes[2 * i + 1][2 * j + 1][2] = megaNodes[i][j][2];
                subNodes[2 * i][2 * j][2] = megaNodes[i][j][2];
                subNodes[2 * i + 1][2 * j][2] = megaNodes[i][j][2];

//                // Uncomment to print all the sub-nodes
//                System.out.println(subNodes[2*i+1][2*j][0]+", "+subNodes[2*i][2*j][1]+" ---> "+subNodes[2*i][2*j][2]);
//                System.out.println(subNodes[2*i+1][2*j][0]+", "+subNodes[2*i+1][2*j][1]+" ---> "+subNodes[2*i+1][2*j][2]);
//                System.out.println(subNodes[2*i][2*j+1][0]+", "+subNodes[2*i][2*j+1][1]+" ---> "+subNodes[2*i][2*j+1][2]);
//                System.out.println(subNodes[2*i+1][2*j+1][0]+", "+subNodes[2*i+1][2*j+1][1]+" ---> "+subNodes[2*i+1][2*j+1][2]);
//
//                // Uncomment to print the sub-nodes that will be used for trajectories
//                if (nodes[i][j][2]!=1){
//                    System.out.println(subNodes[2*i][2*j][0]+", "+subNodes[2*i][2*j][1]);
//                    System.out.println(subNodes[2*i+1][2*j][0]+", "+subNodes[2*i+1][2*j][1]);
//                    System.out.println(subNodes[2*i][2*j+1][0]+", "+subNodes[2*i][2*j+1][1]);
//                    System.out.println(subNodes[2*i+1][2*j+1][0]+", "+subNodes[2*i+1][2*j+1][1]);
//                }
            }

        }

    }

    private void checkInPolyAndObstacle(int i, int j) {
        if (cartObst.length > 0){
            for (int k=0; k<cartObst.length; k++){
                if (InPolygon.check(new double[]{subNodes[2 * i][2 * j + 1][0], subNodes[2 * i][2 * j + 1][1]}, cartObst[k]) ||
                        InPolygon.check(new double[]{subNodes[2 * i + 1][2 * j + 1][0], subNodes[2 * i + 1][2 * j + 1][1]}, cartObst[k]) ||
                        InPolygon.check(new double[]{subNodes[2 * i][2 * j][0], subNodes[2 * i][2 * j][1]}, cartObst[k]) ||
                        InPolygon.check(new double[]{subNodes[2 * i + 1][2 * j][0], subNodes[2 * i + 1][2 * j][1]}, cartObst[k])) {
                    megaNodes[i][j][2] = 1;
                    megaNodesCount--;
                }
            }
        }
    }

    public double getOptimizationIndex(){
        double optimizationIndex;
        float a = .68f;
        float b = .32f;
        float c = .25f;

//        a = 0;
//        b = 0;
//        c = 0;

        double polygonArea = getPolygonArea();


        double nodesInTerm = (megaNodesCount * Math.pow(nodeDistance,2))/polygonArea;
        double minBBAreaTerm = polygonArea/getBoundingBoxArea();
        double equalMarginsTerm;
        if (megaNodesCount>0)
            equalMarginsTerm = marginNormSSI();
        else
            equalMarginsTerm = 1;

        optimizationIndex = a*nodesInTerm + b*minBBAreaTerm - c*equalMarginsTerm;

        ConnectComponent G2G = new ConnectComponent();
        int[][] connectivityTest = new int[xNodes][yNodes];
        for (int i = 0; i < xNodes; i++) {
            for (int j = 0; j < yNodes; j++) {
                connectivityTest[i][j] = (int) Math.abs(megaNodes[i][j][2] - 1);
            }
        }
        G2G.compactLabeling(connectivityTest, new Dimension(yNodes, xNodes), true);
        if (G2G.getMaxLabel() > 1) {
            optimizationIndex -= .5;
        }

        return optimizationIndex;
    }

    private double marginNormSSI(){

        double SSI;

        double[][] coords = new double[4*megaNodesCount][2];

        int c = 0;
        for (int i = 0; i < 2*xNodes; i++) {
            for (int j = 0; j < 2*yNodes; j++) {
                if (subNodes[i][j][2] != 1){
                    coords[c][0] =  subNodes[i][j][0];
                    coords[c][1] =  subNodes[i][j][1];
                    c++;
                }
            }
        }

        double xBoxMax = MinMax.xMax(coords);
        double xBoxMin = MinMax.xMin(coords);
        double yBoxMax = MinMax.yMax(coords);
        double yBoxMin = MinMax.yMin(coords);

        SSI = Math.abs(Math.abs(xBoxMax - xMax) - Math.abs(xMin - xBoxMin))/(2*Math.abs(xBoxMax - xBoxMin)) + Math.abs(Math.abs(yBoxMax - yMax) - Math.abs(yMin - yBoxMin))/(2*Math.abs(yBoxMax - yBoxMin));

        return SSI;
    }

    public double[][][] getMegaNodes() {
        return megaNodes;
    }

    public double[][][] getSubNodes() {
        return subNodes;
    }

    public int getMegaNodesInCount() {
        return megaNodesCount;
    }

    public double getBoundingBoxArea() {
        return (xRngMeters * yRngMeters);
    }

    public double getPolygonArea(){

        double sum = 0;
        for (int i = 0; i < polygonCoordinates.length ; i++)
        {
            if (i == 0)
            {
                sum += polygonCoordinates[i][0] * (polygonCoordinates[i + 1][1] - polygonCoordinates[polygonCoordinates.length - 1][1]);
            }
            else if (i == polygonCoordinates.length - 1)
            {
                sum += polygonCoordinates[i][0] * (polygonCoordinates[0][1] - polygonCoordinates[i - 1][1]);
            }
            else
            {
                sum += polygonCoordinates[i][0] * (polygonCoordinates[i + 1][1] - polygonCoordinates[i - 1][1]);
            }
        }

        double area = 0.5 * Math.abs(sum);
        return area;

    }

}