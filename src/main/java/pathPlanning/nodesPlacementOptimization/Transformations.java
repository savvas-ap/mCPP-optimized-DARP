package pathPlanning.nodesPlacementOptimization;

import pathPlanning.handleGeo.NodesInPoly;

public class Transformations {

    float theta;
    float shiftX;
    float shiftY;

    public float getTheta() {
        return theta;
    }
    public float getShiftX() {
        return shiftX;
    }
    public float getShiftY() {
        return shiftY;
    }

    public void setTheta(float theta) {
        this.theta = theta;
    }
    public void setShiftX(float shiftX) {
        this.shiftX = shiftX;
    }
    public void setShiftY(float shiftY) {
        this.shiftY = shiftY;
    }

    public double rotateAndShift(double[][] cart, double[][][] cartObst, int scanDist){

        double[][] rotated;
        double[][][] rotatedObst = new double[][][]{};

        Rotate rotate = new Rotate();
        rotate.setTheta(theta);
        rotated = rotate.rotatePolygon(cart);

        if (cartObst.length > 0) {
            rotatedObst = new double[cartObst.length][][];
            for (int i=0; i<cartObst.length; i++){
                rotatedObst[i] = rotate.rotatePolygon(cartObst[i]);
            }
        }

        NodesInPoly optimize = new NodesInPoly(rotated, rotatedObst, scanDist, true, true, shiftX, shiftY);
        double optimizationIndex = optimize.getOptimizationIndex();

        return optimizationIndex;
    }

    public int shift(double[][] cart, double[][][] cartObst, int scanDist){
        int megaNodesInCount;

        NodesInPoly optimize = new NodesInPoly(cart, cartObst, scanDist, true, true, shiftX, shiftY);
        megaNodesInCount = optimize.getMegaNodesInCount();

        return megaNodesInCount;
    }

}
