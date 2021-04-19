package pathPlanning.nodesPlacementOptimization;

public class SimulatedAnnealing {

    public float getOptimalTheta() {
        return optimalTheta;
    }

    public float getOptimalShiftX() {
        return optimalShiftX;
    }

    public float getOptimalShiftY() {
        return optimalShiftY;
    }

    private float optimalTheta;
    private float optimalShiftX;
    private float optimalShiftY;

    private double[][] cart;
    private double[][][] cartObst;
    private int scanDist;

    private double optimizationIndexMax = 0;
    private double optimizationIndexCurrent = 0;

    public void run(double[][] cart, double[][][] cartObst, int scanDist){
        this.cart = cart;
        this.cartObst = cartObst;
        this.scanDist = scanDist;

        // Initial and final temperature
        double T = 1000;
        // Temperature at which iteration terminates
        final double Tmin = 5;
        // Decrease in temperature
        final double alpha = 0.9;
        // Number of iterations of annealing before decreasing temperature
        final int numIterations = 500;

        // Random initial solution
        Solution currentSol = new Solution();
        currentSol.Initialization();
        currentSol.CreateNew();

        // Continues annealing until reaching minimum temperature
        while (T > Tmin) {

            for (int i=0; i<numIterations; i++){

                optimizationIndexCurrent = currentSol.getOptimizationIndex();
                // Reassigns global minimum accordingly
                if (optimizationIndexCurrent > optimizationIndexMax){
                    optimizationIndexMax = optimizationIndexCurrent;
                    optimalTheta = currentSol.getTheta();
                    optimalShiftX = currentSol.getShiftX();
                    optimalShiftY = currentSol.getShiftY();
                }

                Solution newSol = new Solution();
                newSol.Random();
//                if (i == numIterations-1){
//                    newSol.Random();
//                } else {
//                    newSol.Neighbor();
//                }
                newSol.CreateNew();
                double ap = Math.pow(Math.E, (optimizationIndexCurrent - newSol.getOptimizationIndex())/T);
                if (ap > Math.random())
                    currentSol = newSol;
            }

            T *= alpha; // Decreases T, cooling phase
        }
        System.out.println("\n ~~~ Final value of optimization index: "+optimizationIndexMax+" ~~~ ");

    }

    private class Solution{

        private float theta;
        private float shiftX;
        private float shiftY;
        private double optimizationIndex;

        public void Initialization(){
            theta = 0;
            shiftX = scanDist;
            shiftY = scanDist;
        }

        public void Random(){
            theta = (float) Math.random() * 90;
            shiftX = (float) Math.random() * 2 * scanDist;
            shiftY = (float) Math.random() * 2 * scanDist;
        }

        public void Neighbor(){

            int a;
            int b;
            int c;

            if (Math.random() > .5) a = 45;
            else a = -45;
            if (Math.random() > .5) b = scanDist/4;
            else b = -scanDist/4;
            if (Math.random() > .5) c = scanDist/4;
            else c = -scanDist/4;

            theta = theta + a*((float) Math.random());
            shiftX = shiftX + b*((float) Math.random());
            shiftY = shiftY + c*((float) Math.random());

        }

        public void CreateNew(){

            Transformations randomSol = new Transformations();
            randomSol.setTheta(theta);
            randomSol.setShiftX(shiftX);
            randomSol.setShiftY(shiftY);
            optimizationIndex = randomSol.rotateAndShift(cart, cartObst, scanDist);

        }

        public float getTheta() {
            return theta;
        }

        public float getShiftX() {
            return shiftX;
        }

        public float getShiftY() {
            return shiftY;
        }

        public double getOptimizationIndex() {
            return optimizationIndex;
        }

    }


}
