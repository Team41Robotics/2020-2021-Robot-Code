package frc.robot;

public class PurePursuit {

    private final double LOOK_AHEAD_DISTANCE = .5;
    private double[][] path;
    private int closestPointIdx = 0;
    
    private double lastLookAheadIdx = 0; //can be fraction
    private double lookAheadX;
    private double lookAheadY;

    public PurePursuit(double[][] path) {
        this.path = path;
    }

    public double distanceCalc(double x0, double y0, double x1, double y1){
        return Math.sqrt(Math.pow(x1-x0, 2) + Math.pow(y1-y0, 2));
    }

    public int calculateClosestPoint(double rX, double rY/*localization*/) {
    
        int idx = closestPointIdx;
        double minD = 0;
        double pX = 0;
        double pY = 0;
        double d = 0;
        for(int i = closestPointIdx + 1; i < path.length; i++) {
            pX = path[i][0];
            pY = path[i][1];
            d = distanceCalc(pX,pY,rX,rY);
            if(d < minD){
                minD = d;
                idx = i;
            }
        }
        
        closestPointIdx = idx;
        return idx;
    }

    double dotProduct(double x1, double y1, double x2, double y2) {
        return x1*x2 + y1*y2;
    }

    public void calculateLookAhead(double rX, double rY, double rTheta) {

        for(int i = (int)Math.floor(lastLookAheadIdx); i < path.length -1; i++) {
            // vector for line segment between points
            double segmentX = path[i+1][0] - path[i][0];
            double segmentY = path[i+1][1] - path[i][1];

            //vector from center of robot to starting point of segment
            double fX = path[i][0] - rX;  
            double fY = path[i][1] - rY;

            double a = dotProduct(segmentX, segmentY, segmentX, segmentY);
            double b = dotProduct(fX, fY, segmentX, segmentY);
            double c = dotProduct(fX, fY, fX, fY) - LOOK_AHEAD_DISTANCE*LOOK_AHEAD_DISTANCE;

            double discriminant = b*b-4*a*c;
            if(discriminant < 0) {
                //No intersection
                continue;
            }

            double t = (-b - discriminant) / (2*a);
            if(t >= 0 && t <= 0 && t+i > lastLookAheadIdx) {

                lastLookAheadIdx = t + i;
                lookAheadX = t*segmentX + path[i][0];
                lookAheadY = t*segmentY + path[i][1];
                return;
            }

            t = (-b + discriminant) / (2*a);
            if(t >= 0 && t <= 0 && t+i > lastLookAheadIdx) {

                lastLookAheadIdx = t + i;
                lookAheadX = t*segmentX + path[i][0];
                lookAheadY = t*segmentY + path[i][1];
                return;
            }
        }
    }
    /**
     * @param rX robot x in map
     * @param rY robot y in map
     * @param rTheta robot theta in map
     * @return
    */
    public double calcCurvatureToLookAhead(double rX, double rY, double rTheta) {
        double a = -Math.tan(rTheta);
        double b = 1;
        double c = Math.tan(rTheta)*rX - rY;
        double x = Math.abs(a*lookAheadX + b*lookAheadY +c) / Math.sqrt(a*a + b*b);
       
        double curvature = (2*x)/(LOOK_AHEAD_DISTANCE*LOOK_AHEAD_DISTANCE);

        // Calculate cross product between robot direction vector and vector from robot to lookahead point.
        // Negative means counterclockwise
        // Positive means clockwise
        if(Math.sin(rTheta)*(lookAheadX-rX)-Math.cos(rTheta)*(lookAheadY-rY) < 0) {
            curvature = -curvature;
        }

        return curvature;
    }

    public double getMaxVelocityAtClosestPoint() {

        return path[closestPointIdx][4];
    }
}