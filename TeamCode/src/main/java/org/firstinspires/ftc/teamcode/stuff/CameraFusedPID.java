package org.firstinspires.ftc.teamcode.stuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.vision.pipeline.AprilTagDetectionPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@TeleOp(name = "Cone Tracker")
@Config
public class CameraFusedPID extends LinearOpMode {
    public static double x1l=1;
    public static double y1l=1.3;
    public static double x2l=0.9;
    public static double y2l=3.5;
    double integralSum = 0;


    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private BNO055IMU imu;
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    /** MAKE SURE TO CHANGE THE FOV AND THE RESOLUTIONS ACCORDINGLY **/
    private String ObjectDirection;
    private int randomization = 99;
    private double thresh = CameraPipeline.perThreshold;
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    private static final double FOV = 40;

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        CameraPipeline aprilTagDetectionPipeline = new CameraPipeline(telemetry);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(864, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        waitForStart();
        while (opModeIsActive()){

            ObjectDirection = CameraPipeline.randomization(thresh);
            randomization = CameraPipeline.PosToNum(ObjectDirection);
            telemetry.addLine(ObjectDirection);
            telemetry.update();
        }
    }
    @Config
    public static class CameraPipeline extends OpenCvPipeline
    {
        private static double rx1l=1;
        private static double ry1l=1.3;
        private static double rx2l=0.9;
        private static double ry2l=3.5;
        private static double rx1r=4;
        private static double ry1r=4.5;
        private static double rx2r=1.2;
        private static double ry2r=2;
        private static double bx1l=0.4;
        private static double by1l=1.4;
        private static double bx2l=2.2;
        private static double by2l=4.5;
        private static double bx1r=2.7;
        private static double by1r=1.2;
        private static double bx2r=4.7;
        private static double by2r=3.5;

        public static String color = "BLUE";
        public static double perThreshold = 15;
        Telemetry telemetry;
        static Rect LEFT_ROI = null;
        static Rect RIGHT_ROI = null;
        public static String ObjectDirection;
        Mat mat = new Mat();

        public CameraPipeline(Telemetry t){
            telemetry = t;
        }

        public static double leftPer;
        public static double rightPer;
        public static double midPer;

        boolean objLeft;
        boolean objRight;

        public Mat processFrame(Mat input)
        {

            Size s = input.size();
            double height = s.height;
            double width = s.width;

            if(Objects.equals(color, "BLUE")){ //remind me to adjust
                LEFT_ROI = new Rect(
                        new Point(bx1l/8 * width, by1l/8 * height),
                        new Point(bx2l/8 * width, by2l/8 * height));

                RIGHT_ROI = new Rect(
                        new Point(bx1r/8 * width, by1r/8 * height),
                        new Point(bx2r/8 * width, by2r/8 * height));
            }
            else if (Objects.equals(color, "RED")) {
                LEFT_ROI = new Rect(
                        new Point(rx1l/8 * width, ry1l/8 * height),
                        new Point(rx2l * width/2, ry2l/8 * height));

                RIGHT_ROI = new Rect(
                        new Point(rx1r/8 * width, ry1r/8 * height),
                        new Point(rx2r * width/2, ry2r/8 * height));
            }


            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); //Uses HSV Colors

            Scalar lowHSVRed = new Scalar(0,150,20); // lower bound HSV for red 0 100 20
            Scalar highHSVRed = new Scalar(25, 255, 255); // higher bound HSV for red 10 255 255

            Scalar lowHSVBlue = new Scalar(100, 100, 20); // lower bound HSV for blue 110 100 20
            Scalar highHSVBlue = new Scalar(130, 255, 255); // higher bound HSV for blue 130 255 255

            Mat thresh = new Mat();

            Mat left = mat.submat(LEFT_ROI);
            Mat right = mat.submat(RIGHT_ROI);


            if(Objects.equals(color, "RED")){
                Core.inRange(mat, lowHSVRed, highHSVRed, thresh);
            }
            else if (Objects.equals(color, "BLUE")) {
                Core.inRange(mat, lowHSVBlue, highHSVBlue, thresh);
            }

            Mat leftT = thresh.submat(LEFT_ROI);
            Mat rightT = thresh.submat(RIGHT_ROI);

            double leftValThr = Core.sumElems(leftT).val[0] / LEFT_ROI.area() / 255;
            double rightValThr = Core.sumElems(rightT).val[0] / RIGHT_ROI.area() / 255;

            leftPer = Math.round(leftValThr * 100);
            rightPer = Math.round(rightValThr * 100);
            midPer = 0;

            //double midValue = Core.sumElems(mid).val[0] / RIGHT_ROI.area() / 255;

            left.release();
            right.release();

            leftT.release();
            rightT.release();

            //mid.release();

            objLeft = leftPer > perThreshold;
            objRight = rightPer > perThreshold;

            if(objLeft && objRight){
                if(leftPer > rightPer){
                    objLeft = true;
                    objRight = false;
                }
                else{
                    objRight = true;
                    objLeft = false;
                }
            }

            if(objLeft){
                if (Objects.equals(color, "RED")) {
                    ObjectDirection = "MIDDLE";
                }
                if (Objects.equals(color, "BLUE")) {
                    ObjectDirection = "MIDDLE";
                }
            }
            else if(objRight){
                if (color == "BLUE") {
                    ObjectDirection = "RIGHT";
                }
                if (color == "RED") {
                    ObjectDirection = "RIGHT";
                }
            }
            else{
                if(Objects.equals(color, "BLUE")){
                    ObjectDirection = "LEFT";
                }else if(Objects.equals(color, "RED")){
                    ObjectDirection = "LEFT";
                }
            }

            Imgproc.rectangle(
                    thresh,
                    LEFT_ROI,
                    new Scalar(255, 255, 255), 4);
            Imgproc.rectangle(
                    thresh,
                    RIGHT_ROI,
                    new Scalar(255, 255, 255), 4);



//        Imgproc.rectangle(
//                thresh,
//                MID_ROI,
//                new Scalar(255, 255, 255), 4);

            // telemetry.addData("Location: ", ObjectDirection);
            //telemetry.update();



            return thresh; //input

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

        }
        public static void setColor(String color){
            CameraPipeline.color = color;
        }
        /*public static OpenCvWebcam initPipeline(HardwareMap hardwareMap, Telemetry telemetry) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

            CameraPipeline s = new CameraPipeline(telemetry);
            webcam.setPipeline(s);

            webcam.setMillisecondsPermissionTimeout(5000);

            return webcam;
        }*/
        public static String randomization(double thresh){
            String ObjectDirection = "";
            if(leftPer > thresh || rightPer > thresh || midPer > thresh){
                if(leftPer > rightPer && leftPer > midPer){ //mid
                    if(color.equals("RED")){
                        ObjectDirection = "MIDDLE";
                    }
                    else if(color.equals("BLUE")){
                        ObjectDirection = "LEFT";
                    }
                }
                else if(rightPer > leftPer && rightPer > midPer){ //right
                    if(color.equals("RED")){
                        ObjectDirection = "RIGHT";
                    }
                    else if(color.equals("BLUE")){
                        ObjectDirection = "MIDDLE";
                    }
                }
            }
            else{
                if(color.equals("RED")){
                    ObjectDirection = "LEFT";
                }
                else if(color.equals("BLUE")){
                    ObjectDirection = "RIGHT";
                }
            }
            return ObjectDirection;
        }
        public static int PosToNum(String ObjectDirection){
            int randomization = 99;
            switch (ObjectDirection) {
                case "LEFT":
                    randomization = 0;
                    break;
                case "RIGHT":
                    randomization = 2;
                    break;
                case "MIDDLE":
                    randomization = 1;
                    break;
            }
            return randomization;
        }

        public static boolean isBlue(){
            return color.equals("BLUE");
        }
        public static boolean isRed(){
            return color.equals("RED");
        }
    }


    /*class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }*/
    private static double getAngleTarget(double objMidpoint){
        double midpoint = -((objMidpoint - (CAMERA_WIDTH/2))*FOV)/CAMERA_WIDTH;
        return midpoint;
    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }

    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }


}