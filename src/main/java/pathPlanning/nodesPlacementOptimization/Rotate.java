package pathPlanning.nodesPlacementOptimization;

import pathPlanning.handleGeo.NodesInPoly;
import pathPlanning.helpers.MinMax;

import java.util.ArrayList;

public class Rotate {

    private float optimalTheta = 0.0f;
    private int range = 90;

    public void setParameters(float optimalTheta, int range) {
        this.optimalTheta = optimalTheta;
        this.range = range;
    }

    public void setTheta(float theta){
        optimalTheta =  theta;
    }

    public void setRange(int range){
        this.range = range;
    }

    public void findOptimalTheta(int scanDist, double[][] cartUnrotated, double[][][] cartObstUnrotated, float stepSize, double shiftX,  double shiftY){
        int testRange = (int) (range/stepSize);

        float theta;
        int[] nodesIn = new int[testRange];
        double[] area = new double[testRange];

        optimalTheta = 0.0f;
        for (int i=0; i<testRange; i++){
            double[][][] cartObstRotated = new double[cartObstUnrotated.length][][];
            for (int j=0; j<cartObstUnrotated.length; j++){
                cartObstRotated[j] = rotatePolygon(cartObstUnrotated[j]);
            }
            NodesInPoly testTheta = new NodesInPoly(rotatePolygon(cartUnrotated), cartObstRotated, scanDist, true, true, shiftX, shiftY);
            nodesIn[i] = testTheta.getMegaNodesInCount();
            area[i] = testTheta.getBoundingBoxArea();
            optimalTheta = optimalTheta + stepSize;
        }
//        theta = ((float)MinMax.indMin(area))*stepSize;  // Minimize area of bounding box
//        theta = ((float)MinMax.indMax(nodesIn))*stepSize; // Maximize number of mega-cells in polygon
        theta = (float) MinMax.indMaxNodeMinArea(nodesIn, area)*stepSize;    // Minimum area of bounding box for maximum number of mega-cells in polygon
        System.out.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n" +
                "        Optimal Rotation Angle for Paths: " + theta + " degrees\n" +
                "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

        optimalTheta = theta;
    }

    public double[][] rotatePolygon(double[][] cart){
        int l = cart.length;
        int m = cart[0].length;
        double[][] rotated = new double[l][m];

        for (int i=0; i<l; i++) {
            rotated[i][0] = cart[i][0]* Math.cos(Math.toRadians(optimalTheta)) - cart[i][1]* Math.sin(Math.toRadians(optimalTheta));
            rotated[i][1] = cart[i][0]* Math.sin(Math.toRadians(optimalTheta)) + cart[i][1]* Math.cos(Math.toRadians(optimalTheta));
        }

        return rotated;
    }

    public ArrayList<double[]> rotateBackWaypoints(ArrayList<double[]> iWaypoints){

        float minusTheta = -optimalTheta;

        int l = iWaypoints.size();
        ArrayList<double[]> waypoints = new ArrayList<>();

        for (int i=0; i<l; i++) {
            double a = iWaypoints.get(i)[0]* Math.cos(Math.toRadians(minusTheta)) - iWaypoints.get(i)[1]* Math.sin(Math.toRadians(minusTheta));
            double b = iWaypoints.get(i)[0]* Math.sin(Math.toRadians(minusTheta)) + iWaypoints.get(i)[1]* Math.cos(Math.toRadians(minusTheta));
            waypoints.add(new double[] {a, b});
        }

        return waypoints;
    }

    public float getOptimalTheta() {
        return optimalTheta;
    }

}