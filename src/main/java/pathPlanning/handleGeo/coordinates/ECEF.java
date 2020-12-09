package pathPlanning.handleGeo.coordinates;

/**
 * Representation of a point in the Earth-Centered, Earth-Fixed (ECEF) geographic coordinate
 * system.
 * <p>
 * ECEF uses three-dimensional XYZ coordinates (in meters) to describe a location. The term
 * "Earth-Centered" comes from the fact that the origin of the axis (0,0,0) is located at the mass
 * center of gravity. The term "Earth-Fixed" implies that the axes are fixed with respect to the
 * earth. The Z-axis pierces the North Pole, and the XY-axis defines the equatorial plane.
 */
public class ECEF {
    /** X coordinate in meter. */
    public double x;
    /** Y coordinate in meter. */
    public double y;
    /** Z coordinate in meter. */
    public double z;

    /**
     * Constructs a point at the origin of the ECEF coordinate system.
     */
    public ECEF() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    /**
     * Constructs a point at a given ECEF location.
     *
     * @param x X coordinate in meter.
     * @param y Y coordinate in meter.
     * @param z Z coordinate in meter.
     */
    public ECEF(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
}