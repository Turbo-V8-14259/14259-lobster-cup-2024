//package com.example.drivesim;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//public class DriveSim {
//    public static void main(String[] args) {
//        MeepMeep meepMeep = new MeepMeep(800);
////        Pose2d initialPosition = new Pose2d(59, -38, Math.toRadians(0));
////        Vector2d secondPosition = new Vector2d(12,-38);
////        Pose2d thirdPosition = new Pose2d(35,-32,Math.toRadians(90));
////        Pose2d fourthPosition = new Pose2d(12,-50,Math.toRadians(-90));
////        Pose2d fifthPosition = new Pose2d(12,-55, Math.toRadians(-90));
////        Pose2d sixthPosition = new Pose2d(12,0, Math.toRadians(-90));
////        Pose2d seventhPosition = new Pose2d(15,11, Math.toRadians(-120));
//        Pose2d initialPosition = new Pose2d(59, -38, Math.toRadians(0));
//        Vector2d secondPosition = new Vector2d(12,-38);
//        Pose2d thirdPosition = new Pose2d(35,-32,Math.toRadians(90));
//        Pose2d fourthPosition = new Pose2d(12,-50,Math.toRadians(-90));
//        Pose2d fifthPosition = new Pose2d(12,-55, Math.toRadians(-90));
//        Pose2d sixthPosition = new Pose2d(12,0, Math.toRadians(-90));
//        Pose2d seventhPosition = new Pose2d(15,11, Math.toRadians(-120));
//
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(52.48291908330528, 70, Math.toRadians(180), Math.toRadians(180), 14.35)
//                .followTrajectorySequence(drive ->
//                                drive.trajectorySequenceBuilder(initialPosition)
////                                .splineToSplineHeading(new Pose2d(-12.5, -60, Math.toRadians(-180)), Math.toRadians(180)) set x to 0
//////                                .splineToConstantHeading(new Vector2d(-12.5, -11.5), Math.toRadians(100))
//////                                .splineToConstantHeading(new Vector2d(-24.5, -11.5), Math.toRadians(70)) weird substation path
//////                                  .splineToConstantHeading(new Vector2d(-35.5, -35.5), Math.toRadians(170))
////////                                .splineTo(new Vector2d(-23.5, -12), Math.toRadians(180))
//////                                .lineToLinearHeading(new Pose2d(36, -2, Math.toRadians(-90))) right auton here
//////                                .lineToLinearHeading(new Pose2d(38, -10, Math.toRadians(0)))
//////                                .lineToLinearHeading(new Pose2d(11.5, -12, Math.toRadians(0))) park 1
//////                                .lineToLinearHeading(new Pose2d(36, -12.1, Math.toRadians(-90))) park 2
//////                                .lineToLinearHeading(new Pose2d(59, -12, Math.toRadians(0))) park 3
//////                                .lineToLinearHeading(new Pose2d(36, -18, Math.toRadians(-90)))
//////                                .splineToSplineHeading(new Pose2d(20, -14, Math.toRadians(0)), Math.toRadians(0)) goofy spline shit for right side
//////                                .splineTo(new Vector2d(24, -11), Math.toRadians(0))
//                                        .lineTo(secondPosition)
//                                        .splineToSplineHeading(thirdPosition, CalculateTangents.calculateTangent(secondPosition,thirdPosition)) //or y -47 and -45degree for down randomization and x 24 for straight
////                                .lineToLinearHeading(fourthPosition)
//                                        .splineToSplineHeading(fourthPosition, CalculateTangents.calculateTangent(thirdPosition,fourthPosition))
//                                        .lineTo(new Vector2d(fourthPosition.position.x,fourthPosition.position.y-10))
//                                        .lineToLinearHeading(sixthPosition)
//                                        .splineToSplineHeading(seventhPosition, CalculateTangents.calculateTangent(sixthPosition,seventhPosition))
//                                        .build()
//                );
//
//        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();
//    }
//
//}

package com.example.drivesim;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class DriveSim {
    int x =0, y=0;//arbitrary adjustment values
    int posNum = 0;//0, 1, 2, 3 clockwise from bottom left
    int mult[][] = {{1,1,1}, {1, -1, -1}, {-1, -1}, {-1, 1}};
    Pose2d initialPosition = new Pose2d(-35*mult[posNum][0]+x, -65*mult[posNum][1]+y, Math.toRadians(-90*mult[posNum][2]));
    Vector2d secondPosition = new Vector2d(-35*mult[posNum][0]+x,-11.6*mult[posNum][1]+y);
    Pose2d thirdPositionR = new Pose2d(-35*mult[posNum][0]+x,-29*mult[posNum][1]+y,Math.toRadians(0));//or -180
    Pose2d thirdPositionL = new Pose2d(-35*mult[posNum][0]+x, -29*mult[posNum][1]+y, Math.toRadians(-180*mult[posNum][2]));
    Vector2d thirdPositionF = new Vector2d(-35*mult[posNum][0]+x, -14*mult[posNum][1]+y);
    Pose2d fourthPosition = new Pose2d(-35*mult[posNum][0]+x,-11.6*mult[posNum][1]+y,Math.toRadians(-90*mult[posNum][2]));
    Pose2d fifthPosition = new Pose2d(-55*mult[posNum][0]+x,-11.6*mult[posNum][1]+y, Math.toRadians(-180*mult[posNum][2]));
    Pose2d sixthPosition = new Pose2d(5*mult[posNum][0]+x, -11.6*mult[posNum][1]+y, Math.toRadians(-180*mult[posNum][2]));
    Pose2d seventhPosition = new Pose2d(14*mult[posNum][0]+x, -16*mult[posNum][1]+y, Math.toRadians(-200*mult[posNum][2]));




    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        Pose2d startBPose = new Pose2d(-35, -65, Math.toRadians(-90)); //default position
        Pose2d startFPose = new Pose2d(11, -65, Math.toRadians(-90));

        //props
        Vector2d leftBProp = new Vector2d(-35, -32);
        Vector2d leftBPropIntermediate = new Vector2d(-37, -32);
        Vector2d leftFProp = new Vector2d(10, -31);
        Vector2d leftFPropIntermediate = new Vector2d(14,-31);

        Vector2d rightBProp = new Vector2d(-35, -32); //rn
        Vector2d rightBPropIntermediate = new Vector2d(-32, -32); //rn
        Vector2d rightFProp = new Vector2d(31, -32);
        Vector2d rightFPropIntermediate = new Vector2d(38, -32);

        Vector2d middleBProp = new Vector2d(-35, -12);
        Vector2d middleBPropIntermediate = new Vector2d(-35,-14);
        Vector2d middleFProp = new Vector2d(20, -24);

        //turns
        double nninetyTurn = Math.toRadians(-90);
        double halfTurn = Math.toRadians(180);
        double pninetyTurn = Math.toRadians(90);
        double TurnL = Math.toRadians(-20);
        double TurnR = Math.toRadians(-40);
        double TurnM = Math.toRadians(-30);
        double unTurnL = Math.toRadians(20);
        double unTurnM = Math.toRadians(30);
        double unTurnR = Math.toRadians(40);

        //stack
        Vector2d beforeStack = new Vector2d(-37, -12);
        Vector2d stackPos = new Vector2d(-54, -12);

        Vector2d FStackI2 = new Vector2d(35, -60);
        Vector2d FStackI1 = new Vector2d(-37, -60);
        Vector2d FStack = new Vector2d(-54, -36);
        //deposit
        Vector2d dropOff = new Vector2d(32, -12);

        Vector2d deposit = new Vector2d(35, -16);
        Vector2d afterDepo = new Vector2d(35, -12);

        Vector2d FFirstR = new Vector2d(45, -42);
        Vector2d FFirstM = new Vector2d(45, -35);
        Vector2d FFirstL = new Vector2d(45, -28);
        Vector2d runToBoardPos = new Vector2d(55, -12);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0,0))
                                .lineTo(new Vector2d(10,50))
                                .lineTo(new Vector2d(57,50))
                                .lineTo(new Vector2d(57,0))
                                .lineTo(new Vector2d(0,0))

//                                .lineTo(leftBProp)
//                                .turn(nninetyTurn)
//                                .lineTo(leftBPropIntermediate)
//                                .lineTo(leftBProp)
//                                .lineTo(beforeStack)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnL)
//                                .lineTo(afterDepo)
//                                .turn(unTurnL)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnL)
//                                .lineTo(afterDepo)
//                                .turn(unTurnL)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnL)
//                                .lineTo(afterDepo)
//                                .turn(unTurnL)
//                                .lineTo(runToBoardPos)
//                                <--- left back
//                                .lineTo(middleBProp)
//                                .lineTo(middleBPropIntermediate)
//                                .lineTo(middleBProp)
//                                .turn(nninetyTurn)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnM)
//                                .lineTo(afterDepo)
//                                .turn(unTurnM)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnM)
//                                .lineTo(afterDepo)
//                                .turn(unTurnM)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnM)
//                                .lineTo(afterDepo)
//                                .turn(unTurnM)
//                                .lineTo(runToBoardPos)
//                                 <--- middle back
//                                .lineTo(rightBProp)
//                                .turn(pninetyTurn)
//                                .lineTo(rightBPropIntermediate)
//                                .lineTo(rightBProp)
//                                .lineTo(beforeStack)
//                                .turn(halfTurn)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnR)
//                                .lineTo(afterDepo)
//                                .turn(unTurnR)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnR)
//                                .lineTo(afterDepo)
//                                .turn(unTurnR)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnR)
//                                .lineTo(afterDepo)
//                                .turn(unTurnR)
//                                .lineTo(runToBoardPos)
////                                <--- right back
//                                .lineTo(FFirstR)
//                                .turn(nninetyTurn)
//                                .lineTo(rightFPropIntermediate)
//                                .lineTo(rightFProp)
//                                .lineTo(dropOff)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnR)
//                                .lineTo(afterDepo)
//                                .turn(unTurnR)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnR)
//                                .lineTo(afterDepo)
//                                .turn(unTurnR)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnR)
//                                .lineTo(afterDepo)
//                                .turn(unTurnR)
//                                .lineTo(runToBoardPos)
                                // <--- right front
//                                .lineTo(FFirstL)
//                                .turn(nninetyTurn)
//                                .lineTo(leftFProp)
//                                .lineTo(leftFPropIntermediate)
//                                .lineTo(dropOff)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnL)
//                                .lineTo(afterDepo)
//                                .turn(unTurnL)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnL)
//                                .lineTo(afterDepo)
//                                .turn(unTurnL)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnL)
//                                .lineTo(afterDepo)
//                                .turn(unTurnL)
//                                .lineTo(runToBoardPos)
//                                <--- left front
//                                .lineTo(FFirstM)
//                                .turn(nninetyTurn)
//                                .lineTo(middleFProp)
//                                .lineTo(dropOff)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnL)
//                                .lineTo(afterDepo)
//                                .turn(unTurnL)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnL)
//                                .lineTo(afterDepo)
//                                .turn(unTurnL)
//                                .lineTo(stackPos)
//                                .lineTo(dropOff)
//                                .lineTo(deposit)
//                                .turn(TurnL)
//                                .lineTo(afterDepo)
//                                .turn(unTurnL)
//                                .lineTo(runToBoardPos)
//                                <--- front middle
//                                .lineTo(FFirstR)
//                                .turn(nninetyTurn)
//                                .lineTo(rightFPropIntermediate)
//                                .lineTo(rightFProp)
//                                .lineTo(FStackI2)
//                                .lineTo(FStackI1)
//                                .lineTo(FStack)
//                                .lineTo(FStackI1)
//                                .lineTo(FStackI2)
//                                .lineTo(FFirstR)
//                                .lineTo(FStackI2)
//                                .lineTo(FStackI1)
//                                .lineTo(FStack)
//                                .lineTo(FStackI1)
//                                .lineTo(FStackI2)
//                                .lineTo(FFirstR)
//                                .lineTo(FStackI2)
//                                .lineTo(FStackI1)
//                                .lineTo(FStack)
//                                .lineTo(FStackI1)
//                                .lineTo(FStackI2)
//                                .lineTo(FFirstR)
//                                  <-- front right bottom
//
//                                .lineTo(FFirstM)
//                                .turn(nninetyTurn)
//                                .lineTo(middleFProp)
//                                .lineTo(FStackI2)
//                                .lineTo(FStackI1)
//                                .lineTo(FStack)
//                                .lineTo(FStackI1)
//                                .lineTo(FStackI2)
//                                .lineTo(FFirstM)
//                                .lineTo(FStackI2)
//                                .lineTo(FStackI1)
//                                .lineTo(FStack)
//                                .lineTo(FStackI1)
//                                .lineTo(FStackI2)
//                                .lineTo(FFirstM)
//                                .lineTo(FStackI2)
//                                .lineTo(FStackI1)
//                                .lineTo(FStack)
//                                .lineTo(FStackI1)
//                                .lineTo(FStackI2)
//                                .lineTo(FFirstM)
//                                  <--- front middle bottom
//                                .lineTo(FFirstL)
//                                .turn(nninetyTurn)
//                                .lineTo(leftFProp)
//                                .lineTo(leftFPropIntermediate)
//                                .lineTo(FStackI2)
//                                .lineTo(FStackI1)
//                                .lineTo(FStack)
//                                .lineTo(FStackI1)
//                                .lineTo(FStackI2)
//                                .lineTo(FFirstL)
//                                .lineTo(FStackI2)
//                                .lineTo(FStackI1)
//                                .lineTo(FStack)
//                                .lineTo(FStackI1)
//                                .lineTo(FStackI2)
//                                .lineTo(FFirstL)
//                                  <--- front left bottom

                                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}