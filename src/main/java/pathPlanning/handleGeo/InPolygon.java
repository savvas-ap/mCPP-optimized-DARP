package pathPlanning.handleGeo;

public class InPolygon {

    // Returns true if the point p lies inside the polygon[] with n vertices
    public static boolean check(double[] point, double[][] coords) {

        int n=coords.length;
        Point[] polygon = new Point[n];
        for (int i=0; i<n; i++){
            polygon[i] = new Point(coords[i][0],coords[i][1]);
        }
        Point p = new Point(point[0], point[1]);

        // Create a point for line segment from p to infinite
        Point extreme = new Point(Double.MAX_VALUE, p.Lon);

        // Count intersections of the above line with sides of polygon
        int count = 0, i = 0;
        do {
            int next = (i + 1) % n;

            // Check if the line segment from 'p' to 'extreme' intersects
            // with the line segment from 'polygon[i]' to 'polygon[next]'
            if (doIntersect(polygon[i], polygon[next], p, extreme)) {
                // If the point 'p' is colinear with line segment 'i-next',
                // then check if it lies on segment. If it lies, return true,
                // otherwise false
                if (orientation(polygon[i], p, polygon[next]) == 0)
                    return onSegment(polygon[i], p, polygon[next]);

                count++;
            }
            i = next;
        } while (i != 0);

        // Return true if count is odd, false otherwise
        return count % 2 == 1;  // Same as (count%2 == 1)
    }

    // The function that returns true if line segment 'p1q1' and 'p2q2' intersect.
    static boolean doIntersect(Point p1, Point q1, Point p2, Point q2) {
        // Find the four orientations needed for general and special cases
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // p1, q1 and p2 are colinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;

        // p1, q1 and p2 are colinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;

        // p2, q2 and p1 are colinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;

        // p2, q2 and q1 are colinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false; // Doesn't fall in any of the above cases
    }public

    // To find orientation of ordered triplet (p, q, r). The function returns following values:
    // 0 --> p, q and r are colinear
    // 1 --> Clockwise
    // 2 --> Counterclockwise
    static int orientation(Point p, Point q, Point r) {
        double val = (q.Lon - p.Lon) * (r.Lat - q.Lat) - (q.Lat - p.Lat) * (r.Lon - q.Lon);
        if (val == 0.0) return 0;  // colinear
        return (val > 0.0) ? 1 : 2; // clock or counterclock wise
    }

    // Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr'
    static boolean onSegment(Point p, Point q, Point r) {
        if (q.Lat <= Double.max(p.Lat, r.Lat) && q.Lat >= Double.min(p.Lat, r.Lat) &&
                q.Lon <= Double.max(p.Lon, r.Lon) && q.Lon >= Double.min(p.Lon, r.Lon)) {
            return true;
        }
        return false;
    }

    public static class Point {
        public double Lat;
        public double Lon;

        public Point(double Lat, double Lon) {
            this.Lat = Lat;
            this.Lon = Lon;
        }
    }

}