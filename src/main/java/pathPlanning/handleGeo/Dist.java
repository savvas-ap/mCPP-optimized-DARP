package pathPlanning.handleGeo;

import java.util.ArrayList;

public class Dist {

    public static double geo(double[] p1, double[] p2){
        double distance = 0;
        final int R = 6371; // Radius of the earth

        Double latDistance = Math.toRadians(p2[0] - p1[0]);
        Double lonDistance = Math.toRadians(p2[1] - p1[1]);
        Double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2)
                + Math.cos(Math.toRadians(p1[0])) * Math.cos(Math.toRadians(p2[0]))
                * Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);
        Double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        distance = R * c * 1000; // convert to meters

        return distance;
    }

    public static double euclidean(double[] x1, double[] x2){
        double distance = -1;
        distance =  Math.sqrt((x2[1] - x1[1])*(x2[1] - x1[1]) + (x2[0] - x1[0])*(x2[0] - x1[0]));

        return distance;
    }

    public static double getLength(ArrayList<double[]> WGS84, boolean printLength){

        double dist = 0;
        for (int i=0; i<WGS84.size() - 1; i++){
            dist += Dist.geo(WGS84.get(i), WGS84.get(i+1));
        }
        if (printLength)
            System.out.println("Path length: "+dist+" meters\n");

        return dist;
    }

    public static double getEstimatedTime(ArrayList<double[]> WGS84, float speed){
        double dist = 0;
        for (int i=0; i<WGS84.size() - 1; i++){
            dist += Dist.geo(WGS84.get(i), WGS84.get(i+1));
        }
        float turnDelay=speed/20*(1+Math.abs(speed));
        return (dist / speed + WGS84.size()*turnDelay) / 60.0;
    }
}
