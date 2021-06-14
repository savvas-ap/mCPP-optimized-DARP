import pathPlanning.darp.ConnectComponent;
import pathPlanning.darp.DARP;

import java.awt.*;


public class DARPTester {

    public static void main (String[] args) {
        // -------------------------------------------------------------------------------------------------------------
        //                                                 DARP parameters
        // -------------------------------------------------------------------------------------------------------------
        int droneNo = 3;                        // #UAVs
        boolean notEqualPortions = false;       // # For proportional area allocation --> true
        double[] Rportions = new double[]{};    // When proportional area allocation is asked, provide portions


        // Grid size
        int l = 50;
        int m = 50;


        // Grid that will be given as input to DARP
        int[][] DARPgrid;

        //----------------------------------------------
        //   Initianize GRID (or give grid as input)
        //----------------------------------------------
        //  In DARPgrid 0 stands for free space
        //  1 stands for Obstacle
        //  2 stands for Drone
        DARPgrid = new int[l][m];
        // Grid representation
        for (int i = 0; i < l; i++) {
            for (int j = 0; j < m; j++) {
                DARPgrid[i][j] = 0;
            }
        }
        //----------------------------------------------


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
            System.out.println("\n\n !!! The environment grid MUST not have" +
                    " unreachable and/or closed shape regions !!! \n\n");
            return;
        }


        //initial positions of drone(s)
        DARPgrid[0][0] = 2;
        DARPgrid[40][20] = 2;
        DARPgrid[0][49] = 2;


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

        // Run DARP
        int[][] DARPAssignmentMatrix = new int[0][];
        problem.constructAssignmentM();
        if (problem.getSuccess()) {
            DARPAssignmentMatrix = problem.getAssignmentMatrix();
        } else {
            System.out.println("\nDARP did not manage to find a solution for the given configuration!");
            return;
        }

        System.out.println("Assignment matrix: \n");
        for (int i=0; i<l; i++){
            for (int j=0; j<m; j++){
                System.out.print(DARPAssignmentMatrix[i][j]+" ");
            }
            System.out.println();
        }
    }

}
