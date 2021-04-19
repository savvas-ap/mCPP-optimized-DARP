package pathPlanning.darp;

import java.util.HashSet;
import java.util.TreeSet;
import java.util.Vector;

/**
 * Created by atkap on 5/20/2016.
 */

public class Kruskal {
    private HashSet nodes[];               // Array of connected components
    private TreeSet allEdges;              // Priority queue of Edge objects
    private Vector allNewEdges;            // Edges in Minimal-Spanning Tree
    private int MAX_NODES;

    Kruskal(int maxN) {
        // Constructor
        this.MAX_NODES = maxN;
        nodes = new HashSet[MAX_NODES];      // Create array for components
        allEdges = new TreeSet(new Edge());  // Create empty priority queue
        allNewEdges = new Vector(MAX_NODES); // Create vector for MST edges
    }

    public void initializeGraph(boolean [][] A, boolean connect4, int mode) {
        int rows = A.length;
        int cols = A[0].length;
        int cost1 = 1;
        int cost2 = 1;

        for (int i=0;i<rows;i++){
            for (int j=0;j<cols;j++){
                if (A[i][j]){

                    if (mode == 0) {
                        cost2 = rows - i;
                    } else if (mode == 1) {
                        cost2 = i;
                    } else if (mode == 2) {
                        cost1 = cols - j;
                    } else if (mode == 3) {
                        cost1 = j;
                    }

                    if (i>0 && A[i-1][j]) {AddToAllEdges(i*cols+j, (i-1)*cols+j, cost1);}
                    if (i<rows-1 && A[i+1][j]) {AddToAllEdges(i*cols+j, (i+1)*cols+j, cost1);}
                    if (j>0 && A[i][j-1]) {AddToAllEdges(i*cols+j, i*cols+j-1, cost2);}
                    if (j<cols-1 && A[i][j+1]) {AddToAllEdges(i*cols+j, i*cols+j+1, cost2);}

                    if (!connect4){
                        if (i>0 && j>0 && A[i-1][j-1]) {AddToAllEdges(i*cols+j, (i-1)*cols+j-1, 1);}
                        if (i<rows-1 && j<cols-1 && A[i+1][j+1]) {AddToAllEdges(i*cols+j, (i+1)*cols+j+1, 1);}
                        if (i>rows-1 && j>0 && A[i+1][j-1]) {AddToAllEdges(i*cols+j, (i+1)*cols+j-1, 1);}
                        if (i>0 && j<cols-1 && A[i-1][j+1]) {AddToAllEdges(i*cols+j, (i-1)*cols+j+1, 1);}
                    }
                }
            }
        }

    }

    private void AddToAllEdges(int from, int to, int cost){
        allEdges.add(new Edge(from, to, cost));  // Update priority queue
        if (nodes[from] == null) {
            // Create set of connect components [singleton] for this node
            nodes[from] = new HashSet(2*MAX_NODES);
            nodes[from].add(new Integer(from));
        }

        if (nodes[to] == null) {
            // Create set of connect components [singleton] for this node
            nodes[to] = new HashSet(2*MAX_NODES);
            nodes[to].add(new Integer(to));
        }
    }



    public void performKruskal() {
        int size = allEdges.size();
        for (int i=0; i<size; i++) {
            Edge curEdge = (Edge) allEdges.first();
            if (allEdges.remove(curEdge)) {
                // successful removal from priority queue: allEdges
                if (nodesAreInDifferentSets(curEdge.from, curEdge.to)) {
                    // System.out.println("Nodes are in different sets ...");
                    HashSet src, dst;
                    int dstHashSetIndex;

                    if (nodes[curEdge.from].size() > nodes[curEdge.to].size()) {
                        // have to transfer all nodes including curEdge.to
                        src = nodes[curEdge.to];
                        dst = nodes[dstHashSetIndex = curEdge.from];
                    } else {
                        // have to transfer all nodes including curEdge.from
                        src = nodes[curEdge.from];
                        dst = nodes[dstHashSetIndex = curEdge.to];
                    }

                    Object srcArray[] = src.toArray();
                    int transferSize = srcArray.length;
                    for (int j=0; j<transferSize; j++) {
                        // move each node from set: src into set: dst
                        // and update appropriate index in array: nodes
                        if (src.remove(srcArray[j])) {
                            dst.add(srcArray[j]);
                            nodes[((Integer) srcArray[j]).intValue()] = nodes[dstHashSetIndex];
                        } else {
                            // This is a serious problem
                            System.out.println("Something wrong: set union");
                            System.exit(1);
                        }
                    }

                    // add new edge to MST edge vector
                    allNewEdges.add(curEdge);
                }
            } else {
                // This is a serious problem
                System.out.println("TreeSet should have contained this element!!");
                System.exit(1);
            }
        }
    }

    private boolean nodesAreInDifferentSets(int a, int b) {
        // returns true if graph nodes (a,b) are in different
        // connected components, ie the set for 'a' is different
        // from that for 'b'
        return(!nodes[a].equals(nodes[b]));
    }

    private void printFinalEdges() {
        System.out.println("The minimal spanning tree generated by "+
                "\nKruskal's algorithm is: ");
        while (!allNewEdges.isEmpty()) {
            // for each edge in Vector of MST edges
            Edge e = (Edge) allNewEdges.firstElement();
            System.out.println("Nodes: (" + e.from + ", " + e.to +
                    ") with cost: " + e.cost);
            allNewEdges.remove(e);
        }
    }


    public Vector getAllNewEdges(){return allNewEdges;}
    public TreeSet getAllEdges(){return allEdges;}

}
