package pathPlanning.handleGeo;
import pathPlanning.handleGeo.coordinates.*;

import java.util.ArrayList;

public class ConvCoords {

    private WGS84 reference = new WGS84();

    private double[][] polygonWGS84;
    private double[][] polygonNED;
    private double[][][] obstaclesWGS84;
    private double[][][] obstaclesNED;
    private ArrayList<ArrayList<double[]>> waypointsWGS84;
    private ArrayList<ArrayList<double[]>> waypointsNED;

    public void polygonWGS84ToNED(double[][] geoCoords){
        polygonWGS84 = geoCoords;
        reference = new WGS84(Math.toRadians(geoCoords[0][0]), Math.toRadians(geoCoords[0][1]), 0);

        polygonNED = WGS84toNED(geoCoords);
    }

    public double[][] convWGS84ToNED(double[][] geoCoords){
        double[][] cartCoords = new double[geoCoords.length][geoCoords[0].length];
        cartCoords = WGS84toNED(geoCoords);

        return cartCoords;
    }

    public void obstaclesToNED(double[][][] geoObstacles){
        obstaclesWGS84 = geoObstacles;
        obstaclesNED = new double[geoObstacles.length][][];
        for (int i=0; i<geoObstacles.length; i++){
            obstaclesNED[i] = WGS84toNED(geoObstacles[i]);
        }
    }

    private double[][] WGS84toNED(double[][] geoCoords){
        WGS84 wgs84;
        NED ned;
        double[][] cartCoords = new double[geoCoords.length][geoCoords[0].length];

        for (int i=0; i<geoCoords.length; i++){
            wgs84 = new WGS84(Math.toRadians(geoCoords[i][0]), Math.toRadians(geoCoords[i][1]), 0);
            ned = WGS84.displacement(reference, wgs84);
            cartCoords[i][0] = ned.north;
            cartCoords[i][1] = ned.east;
        }

        return cartCoords;
    }

    public void NEDToWGS84(ArrayList<ArrayList<double[]>> cartCoords){
        WGS84 wgs84;
        NED ned;
        ArrayList<ArrayList<double[]>> local = new ArrayList<>();

        waypointsNED = cartCoords;
        for (int j=0; j<cartCoords.size(); j++){
            ArrayList<double[]> geoCoords = new ArrayList<>();
            for (int i=0; i<cartCoords.get(j).size(); i++) {
                ned = new NED(cartCoords.get(j).get(i)[0], cartCoords.get(j).get(i)[1], 0);
                wgs84 = WGS84.displace(reference, ned);
                geoCoords.add(new double[]{Math.toDegrees(wgs84.latitude), Math.toDegrees(wgs84.longitude)});
            }
            local.add(geoCoords);
        }
        waypointsWGS84 = local;
    }

    public double[][][] getObstaclesWGS84() {
        return obstaclesWGS84;
    }

    public double[][][] getObstaclesNED() {
        return obstaclesNED;
    }

    public double[][] getPolygonWGS84(){
        return polygonWGS84;
    }

    public double[][] getPolygonNED(){
        return polygonNED;
    }

    public ArrayList<ArrayList<double[]>> getWaypointsWGS84(){
        return waypointsWGS84;
    }

    public ArrayList<ArrayList<double[]>> getWaypointsNED(){
        return waypointsNED;
    }

}