import pathPlanning.darp.*;

import java.util.ArrayList;

public class Poly2Waypoints {

    public static void main (String[] args) {

        //--------------------------------------------------------------------------------------------------------------
        //                         Parameters to run optimized DARP mCPP algorithm
        //--------------------------------------------------------------------------------------------------------------
        int droneNo = 3;                                // #vehicles to scan the ROI
        int scanningDensity = 5;                        // scanning density in meters
        double[][] polygon = new double[][]             // polygon ROI (WGS84 coordinates)
                {{40.645510547499356, 22.97921244352735}, {40.64467536747447, 22.981182615433926},
                {40.64488996973772, 22.98113433567166}, {40.64588741975926, 22.98156885353207},
                {40.646108969111445, 22.981422315518348}, {40.64622444446692, 22.981429479252274},
                {40.64634567578807, 22.981385681014917}, {40.64630861244292, 22.98123191220795},
                {40.64620247498102, 22.981123740954235}, {40.64623195934348, 22.980880576356657},
                {40.64668739910403, 22.980693738148393}, {40.64662359504903, 22.980405221868086},
                {40.64657742426968, 22.980325046170964}, {40.646499594629674, 22.980295832445126},
                {40.64641588710689, 22.980293731382762}, {40.64625134983302, 22.97957740276458},
                {40.64624510712574, 22.979381422695287}};

        double[][][] obstacles = new double[][][]       // obstacles inside the polygon ROI (WGS84 coordinates)
                {{{40.645510547499356, 22.97921244352735}, {40.64467536747447, 22.981182615433926},
                {40.64467536747447, 22.981182615433926}, {40.646064982978004, 22.980222408074145},
                {40.6459444626512, 22.980252881878744}, {40.64589139766221, 22.98024397889991},
                {40.64584408883981, 22.980208253830927}, {40.64580703323685, 22.98015826136508},
                {40.64578724625597, 22.980106927794726}, {40.64580199655134, 22.98005559422437},
                {40.645879885252214, 22.979993717617493}, {40.64592323666065, 22.979915747756525},
                {40.64591208401881, 22.97978126570671}},
                {{40.645126368293454, 22.98090104813877}, {40.645178174707546, 22.981107868816473},
                {40.64542856124022, 22.98103544917307}, {40.64539114564109, 22.980828628495367}}};
//        double[][][] obstacles = new double[][][]{};

        boolean pathsStrictlyInPoly = true;             // true --> paths strictly in polygon mode/false --> better coverage mode

        double[][] initialPos = new double[][]          // initial positions of the vehicles (WGS84 coordinates)
                {{40.93267,24.40603}, {40.93131,24.41342}, {40.93534,24.41271}};
//        double[][] initialPos = new double[][]{};

        double[] rPortions = new double[]{.3, .6, .1 }; // percentage of the region that each vehicle should undertake (sum should always be 1)
//        double[] rPortions = new double[]{};
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
