package org.firstinspires.ftc.teamcode.drive.path;


import static java.lang.Math.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;

public class PurePursuitUtil {

    private static int segment = 1;

    private static int touchCounter=0;

    private static boolean ending = false;

    private static boolean closeEnough = false;

    public static int getSegment(){
        return segment;
    }
    public static void updateSegment(int n){
        segment = n;
    }
    public static ArrayList<Pose2d> lineCircleIntersection(Pose2d circleCenter, double radius, Pose2d linePoint1, Pose2d linePoint2) {
        double discTolerance = 0.01;
        double m1;
        ArrayList<Pose2d> allPoints = new ArrayList<>();

        double vertShift = 0;

        // no verticals
        if (linePoint2.getX() == linePoint1.getX()) {
            vertShift = 0.001;
        }

        linePoint1 = new Pose2d(linePoint1.getX() + vertShift, linePoint1.getY());

        // Handle vertical line
        if (linePoint2.getX() - linePoint1.getX() != 0) {
            m1 = (linePoint2.getY() - linePoint1.getY()) / (linePoint2.getX() - linePoint1.getX());
        } else {
            if(linePoint2.getY()>linePoint1.getY()){
                allPoints.add(new Pose2d(circleCenter.getX(), circleCenter.getY()+radius, angleWrap(PI/2, circleCenter.getHeading())));
            }else {
                allPoints.add(new Pose2d(circleCenter.getX(), circleCenter.getY()-radius, angleWrap(-PI/2, circleCenter.getHeading())));
            }
            return allPoints;
        }

        double x1 = linePoint1.getX() - circleCenter.getX();
        double y1 = linePoint1.getY() - circleCenter.getY();

        // Quadratic coefficients of the solution to the intersection of the line segment and circle
        double A = 1 + Math.pow(m1, 2);
        double B = m1 * y1 - Math.pow(m1, 2) * x1;
        double C = Math.pow(y1, 2) - 2 * m1 * y1 * x1 + Math.pow(m1, 2) * Math.pow(x1, 2) - Math.pow(radius, 2);

        double disc = Math.sqrt(Math.pow(B, 2) -  A * C);


        try {
            if (disc > discTolerance) {
                // First root
                double xroot1 = (-B - disc) / ( A);
                double yroot1 = yCalculator(x1, y1, m1, xroot1);
                xroot1 += circleCenter.getX();
                yroot1 += circleCenter.getY();

                // Second root
                double xroot2 = (-B + disc) / (A);
                double yroot2 = yCalculator(x1, y1, m1, xroot2);
                xroot2 += circleCenter.getX();
                yroot2 += circleCenter.getY();

                if (withinSegment(linePoint1.getX(), linePoint2.getX(), xroot1) ) {
                    allPoints.add(new Pose2d(xroot1, yroot1, heading(circleCenter, xroot1, yroot1)));
                }
                if (withinSegment(linePoint1.getX(), linePoint2.getX(), xroot2) ) {
                    allPoints.add(new Pose2d(xroot2, yroot2,heading(circleCenter, xroot2, yroot2)));
                }
            } else if (disc >= 0 && disc <= discTolerance) {
                double xroot = -B / A;
                double yroot = yCalculator(x1, y1, m1, xroot);
                xroot += circleCenter.getX();
                yroot += circleCenter.getY();
                allPoints.add(new Pose2d(xroot, yroot, heading(circleCenter, xroot, yroot)));
            }else{
                Pose2d pt = recoveryPt(linePoint1, linePoint2, circleCenter);
                //allPoints.add(pt);
            }
        } catch (Exception e) {
            // Handle exceptions
        }

        return allPoints;
    }
    //checks if a point is on a line segment
    public static boolean withinSegment(double start, double end, double value){
        return value>min(start, end)&&value<max(start, end);
    }

    public static boolean isThereAny(Pose2d w1, Pose2d w2, Pose2d rb){
        ArrayList<Pose2d> intersections = lineCircleIntersection(rb, 15, w1, w2);
        if(intersections.size()!=0){
            return true;
        }else{
            return false;
        }
    }

    public static Pose2d followMe(ArrayList<Pose2d> path, Pose2d robotLocation, double followRadius, Pose2d lastPoint, boolean heading ){



        // stores all intersections, valid or not, for the entire path
        ArrayList<Pose2d> allIntersections = new ArrayList<>();
        //segment based view
        if(segment == path.size()-1){
            Pose2d wayPt1 = path.get(segment-1);
            Pose2d wayPt2 = path.get(segment);

            ArrayList<Pose2d> intersections = lineCircleIntersection(robotLocation, followRadius, wayPt1, wayPt2);
            if(ending){
                allIntersections.add(wayPt2);
            }
            else if ((Math.hypot(wayPt2.getX() - robotLocation.getX(), wayPt2.getY() - robotLocation.getY()) < followRadius)) {
                allIntersections.add(wayPt2);
                ending=true;
            }
            else if (intersections.size() == 2) {
                if (Math.hypot(wayPt2.getX() - intersections.get(0).getX(), wayPt2.getY() - intersections.get(0).getY()) > Math.hypot(wayPt2.getX() - intersections.get(1).getX(), wayPt2.getY() - intersections.get(1).getY())) {
                    allIntersections.add(intersections.get(1));
                } else {
                    allIntersections.add(intersections.get(0));
                }
            }

            // if there's only one intersection then save that
            else if (intersections.size() == 1) {
                allIntersections.add(intersections.get(0));
            }

        }else{
            Pose2d wayPt1 = path.get(segment-1);
            Pose2d wayPt2 = path.get(segment);
            Pose2d wayPt3 = path.get(segment +1);

            ArrayList<Pose2d> intersections = lineCircleIntersection(robotLocation, followRadius, wayPt1, wayPt2);
            ArrayList<Pose2d> fintersections = lineCircleIntersection(robotLocation, followRadius, wayPt2, wayPt3);

            if (fintersections.size() != 0) {
                if (fintersections.size() == 1) {
                    allIntersections.add(fintersections.get(0));
                } else if (fintersections.size() == 2) {
                    if (Math.hypot(wayPt3.getX() - fintersections.get(0).getX(), wayPt3.getY() - fintersections.get(0).getY()) > Math.hypot(wayPt3.getX() - fintersections.get(1).getX(), wayPt3.getY() - fintersections.get(1).getY())) {
                        allIntersections.add(fintersections.get(1));
                    } else {
                        allIntersections.add(fintersections.get(0));
                    }
                 
                }
                touchCounter++;
                if(touchCounter>=3){
                    segment++;
                    touchCounter=0;
                }

            } else {
                // if 2 valid intersections, then save the one that's more along the line
                if (intersections.size() == 2) {
                    if (Math.hypot(wayPt2.getX() - intersections.get(0).getX(), wayPt2.getY() - intersections.get(0).getY()) > Math.hypot(wayPt2.getX() - intersections.get(1).getX(), wayPt2.getY() - intersections.get(1).getY())) {
                        allIntersections.add(intersections.get(1));
                    } else {
                        allIntersections.add(intersections.get(0));
                    }
                }

                // if there's only one intersection then save that
                else if (intersections.size() == 1) {
                    allIntersections.add(intersections.get(0));
                }

                // if you're closer to the last point than follow radius then add it to the list, unless you're extending
                else if ((Math.hypot(wayPt2.getX() - robotLocation.getX(), wayPt2.getY() - robotLocation.getY()) < followRadius)) {
                    allIntersections.add(wayPt2);
                }

            }


        }


        // if we didn't find anything return robot location
        if(allIntersections.size() == 0) { return lastPoint; }

        return allIntersections.get(allIntersections.size() - 1);


    }

    public static boolean passedWayPt(Pose2d robotLocation, Pose2d wayPt, Double radius){
        return hypot((robotLocation.getY()- wayPt.getY()),(robotLocation.getX()- wayPt.getX())) <= radius;
    }
    public static Pose2d recoveryPt(Pose2d waypt1, Pose2d waypt2, Pose2d robotLocation){
        if(robotLocation.getX()<min(waypt1.getX(), waypt2.getX())){
            Pose2d newPt = min(waypt1.getX(), waypt2.getX()) == waypt1.getX() ? waypt1:waypt2;
            return new Pose2d(newPt.getX(), newPt.getY(), heading(robotLocation,newPt.getX(),newPt.getY()));
        }else if(robotLocation.getX()>max(waypt1.getX(), waypt2.getX())){
            Pose2d newPt = max(waypt1.getX(), waypt2.getX()) == waypt1.getX() ? waypt1:waypt2;
            return new Pose2d(newPt.getX(), newPt.getY(), heading(robotLocation,newPt.getX(),newPt.getY()));
        }else if(robotLocation.getX()<=max(waypt1.getX(), waypt2.getX())&&robotLocation.getX()>=min(waypt1.getX(), waypt2.getX())){
            double m;
            if (waypt2.getX() - waypt1.getX() != 0) {
                m = (waypt2.getY() - waypt1.getY()) / (waypt2.getX() - waypt1.getX());
                double b = (waypt2.getY()-waypt1.getY()) - m*(waypt2.getX()-waypt1.getX());
                double x1 = robotLocation.getX();
                double y1 = robotLocation.getY() - b;
                if(m==0){
                    b = waypt1.getY();
                    return new Pose2d(robotLocation.getX(), b,heading(robotLocation, robotLocation.getX(), b));
                }else{
                    double rootx = (m*(y1+x1*(1/m)))/(pow(m,2)+1);
                    double rooty = m*rootx + b;
                    return new Pose2d(rootx, rooty, heading(robotLocation, rootx, rooty));
                }
            } else {
                return new Pose2d(waypt1.getX(), robotLocation.getY(), heading(robotLocation, waypt1.getX(), robotLocation.getY()));
            }
        }else{
            Pose2d pt = hypot(robotLocation.getX()-waypt1.getX(), robotLocation.getY()-waypt1.getY()) < hypot(robotLocation.getX()-waypt2.getX(), robotLocation.getY()-waypt2.getY()) ? waypt1 : waypt2;
            return new Pose2d(pt.getX(), pt.getY(), heading(robotLocation, pt.getX(), pt.getY()));
        }
    }
    public static double yCalculator(double x1, double y1, double m, double rootx) {
        return m * (rootx - x1) + y1;
    }
    public static double heading(Pose2d robotLocation, double targetX, double targetY){
        double vectorX = targetX - robotLocation.getX();
        double vectorY = targetY - robotLocation.getY();

        double angle = atan2(vectorY, vectorX);

        return angleWrap(angle, robotLocation.getHeading());
    }
    public static double angleWrap(double targetAngle, double currentAngle){
        if(targetAngle-currentAngle>Math.PI){
            targetAngle -= Math.PI;
            return targetAngle;
        }else if(targetAngle-currentAngle<-Math.PI){
            targetAngle += Math.PI;
            return targetAngle;
        }else{
            return targetAngle;
        }
    }
    public static double distanceTo(Pose2d robot, Pose2d target){
        return hypot(robot.getX()-target.getX(), robot.getY()-target.getY());
    }

    public static boolean getEnding(){
        return ending;
    }

    public static void updateEnding(boolean a){
        ending=a;
    }
}