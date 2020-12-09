package pathPlanning.handleGeo.coordinates;

/**
 * Representation of a point in the North/East/Down (NED) geographical coordinate system.
 */
public class NED {
    /** North coordinate in meter. */
    public double north;
    /** East coordinate in meter. */
    public double east;
    /** Down coordinate in meter. */
    public double down;

    /**
     * Constructs a point at the origin of the NED coordinate system.
     */
    public NED() {
        this.north = 0;
        this.east = 0;
        this.down = 0;
    }

    /**
     * Constructs a point at a given ECEF location.
     *
     * @param north North coordinate in meter.
     * @param east East coordinate in meter.
     * @param down Down coordinate in meter.
     */
    public NED(double north, double east, double down) {
        this.north = north;
        this.east = east;
        this.down = down;
    }
}