package pathPlanning.helpers;

public class MinMax {

    public static int indMaxNodeMinArea(int[] nodesIn, double[] area) {
        if (nodesIn.length != area.length || nodesIn.length == 0 || area.length == 0)
            return -1; // null/empty or unequal length of matrices

        int returnIND = 0;
        int largest = -Integer.MAX_VALUE;
        double bestArea = Double.MAX_VALUE;
        for (int i = 0; i < nodesIn.length; i++) {
            if (nodesIn[i] > largest) {
                largest = nodesIn[i];
                bestArea = area[i];
                returnIND = i;
            } else if (nodesIn[i] == largest && area[i] < bestArea) {
                bestArea = area[i];
                returnIND = i;
            }
        }

        return returnIND;
    }

    public static int indMax(int[] array) {

        if (array == null || array.length == 0) return -1; // null or empty

        int largest = 0;
        for (int i = 1; i < array.length; i++) {
            if (array[i] > array[largest]) largest = i;
        }

        return largest;
    }

    public static int indMin(double[] array) {

        if (array == null || array.length == 0) return -1; // null or empty

        int smallest = 0;
        for (int i = 1; i < array.length; i++) {
            if (array[i] < array[smallest]) {
                smallest = i;
            }
        }

        return smallest;
    }

    public static double xMax(double[][] polygonCoordinates) {
        int l = polygonCoordinates.length;
        double xMax = -Double.MAX_VALUE;

        for (int i = 0; i < l; i++) {
            if (polygonCoordinates[i][0] > xMax) {
                xMax = polygonCoordinates[i][0];
            }
        }

        return xMax;
    }

    public static double yMax(double[][] polygonCoordinates) {
        int l = polygonCoordinates.length;
        double yMax = -Double.MAX_VALUE;

        for (int i = 0; i < l; i++) {
            if (polygonCoordinates[i][1] > yMax) {
                yMax = polygonCoordinates[i][1];
            }
        }

        return yMax;
    }

    public static double xMin(double[][] polygonCoordinates) {
        int l = polygonCoordinates.length;
        double xMin = Double.MAX_VALUE;

        for (int i = 0; i < l; i++) {
            if (polygonCoordinates[i][0] < xMin) {
                xMin = polygonCoordinates[i][0];
            }
        }

        return xMin;
    }

    public static double yMin(double[][] polygonCoordinates) {
        int l = polygonCoordinates.length;
        double yMin = Double.MAX_VALUE;

        for (int i = 0; i < l; i++) {
            if (polygonCoordinates[i][1] < yMin) {
                yMin = polygonCoordinates[i][1];
            }
        }

        return yMin;
    }

}
