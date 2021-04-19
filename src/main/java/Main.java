import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import pathPlanning.darp.DARPinPoly;

import java.io.FileReader;
import java.util.ArrayList;

public class Main {

    private static int droneNo;
    private static int scanningDensity;
    private static double[][] polygon;
    private static double[][][] obstacles;
    private static boolean pathsStrictlyInPoly;
    private static double[][] initialPos;
    private static double[] rPortions;

    private static String pathJSON;
    private static boolean pathDefined = false;

    public static void main (String[] args) {

        //--------------------------------------------------------------------------------------------------------------
        //                                  Parse JSON with mission parameters
        //--------------------------------------------------------------------------------------------------------------
        try {
            pathJSON = args[0];
            pathDefined = true;
        }
        catch (ArrayIndexOutOfBoundsException e){
            System.out.println("No path for JSON file defined");
        }

        if (pathDefined) {                                 // provide input variables with .txt file in resources
            JSONParser parser = new JSONParser();
            try {
                Object obj = parser.parse(new FileReader(pathJSON));

                // A JSON object. Key value pairs are unordered. JSONObject supports java.util.Map interface.
                JSONObject jsonObject = (JSONObject) obj;

                droneNo = ((Long) jsonObject.get("droneNo")).intValue();

                scanningDensity = ((Long) jsonObject.get("scanningDensity")).intValue();

                JSONArray temp = (JSONArray) jsonObject.get("polygon");
                polygon = new double[temp.size()][2];
                JSONObject vertex;
                for (int i=0; i<temp.size(); i++) {
                    vertex = (JSONObject) temp.get(i);
                    polygon [i][0] = (double) vertex.get("lat");
                    polygon [i][1] = (double) vertex.get("long");
                }

                JSONArray obst = (JSONArray) jsonObject.get("obstacles");
                JSONArray ob;
                if (obst.size()>0) {
                    obstacles = new double[obst.size()][][];
                    for (int i=0; i<obst.size(); i++) {
                        ob = (JSONArray) obst.get(i);
                        double[][] obMat = new double[ob.size()][2];
                        for (int j=0; j<ob.size(); j++){
                            vertex = (JSONObject) ob.get(j);
                            double[] vert = new double[2];
                            obMat[j][0] = (double) vertex.get("lat");
                            obMat[j][1] = (double) vertex.get("long");
                        }
                        obstacles[i] = obMat;
                    }
                }else{
                    obstacles = new double[][][]{};
                }

                pathsStrictlyInPoly = (boolean) jsonObject.get("pathsStrictlyInPoly");

                temp = (JSONArray) jsonObject.get("initialPos");
                if (temp.size()>0){
                    initialPos = new double[temp.size()][2];
                    for (int i=0; i<temp.size(); i++) {
                        vertex = (JSONObject) temp.get(i);
                        initialPos [i][0] = (double) vertex.get("lat");
                        initialPos [i][1] = (double) vertex.get("long");
                    }
                }else{
                    initialPos = new double[][]{};
                }

                temp = (JSONArray) jsonObject.get("rPortions");
                if (temp.size()>0){
                    rPortions = new double[temp.size()];
                    for (int i=0; i< temp.size(); i++){
                        rPortions[i] = (double) temp.get(i);
                    }
                }else{
                    rPortions = new double[]{};
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        //--------------------------------------------------------------------------------------------------------------



        //--------------------------------------------------------------------------------------------------------------
        //                                     Run optimized DARP mCPP algorithm
        //--------------------------------------------------------------------------------------------------------------
        DARPinPoly mission = new DARPinPoly();
        if(initialPos.length > 0){
            if (rPortions.length > 0){
                mission.waypointsDARP(droneNo, scanningDensity, polygon, obstacles, pathsStrictlyInPoly, initialPos, rPortions);
            } else {
                mission.waypointsDARP(droneNo, scanningDensity, polygon, obstacles, pathsStrictlyInPoly, initialPos);
            }
        } else {
            if (rPortions.length > 0){
                mission.waypointsDARP(droneNo, scanningDensity, polygon, obstacles, pathsStrictlyInPoly, rPortions);
            } else {
                mission.waypointsDARP(droneNo, scanningDensity, polygon, obstacles, pathsStrictlyInPoly);
            }
        }
        mission.printWaypointsWGS84();

        ArrayList<ArrayList<double[]>> missionWaypoints = mission.getMissionWaypointsWGS84();
        //--------------------------------------------------------------------------------------------------------------

    }

}
