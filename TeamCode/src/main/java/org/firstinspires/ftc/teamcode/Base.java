package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Core.Angle;
import org.firstinspires.ftc.teamcode.Core.Motor;
import org.firstinspires.ftc.teamcode.Core.Point;
import org.firstinspires.ftc.teamcode.Modules.Indexer;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Shooter;
import org.firstinspires.ftc.teamcode.Modules.Turret;

import java.util.ArrayList;
import java.util.List;

public abstract class Base extends LinearOpMode{

    public List<LynxModule> allHubs;
    public Motor fLeft, bLeft, fRight, bRight;

    public Limelight3A limelight;



    public Motor frontSweeper, backSweeper;

    DcMotorEx shooterOne, shooterTwo;

    public RevColorSensorV3 slotOut, slotIntake, finalColor, lastColor;

    Servo rotOne, rotTwo;
    Servo kickerOne, kickerTwo, kickerThree;
    Servo hoodOne, hoodTwo;
    RevColorSensorV3 slotOne, slotTwo, slotThree;
    AnalogInput turretTrack;

    String oneSlot, twoSlot, threeSlot;



    double KICKER_ONE_DOWN = 0.953, KICKER_ONE_UP = 0.7, KICKER_TWO_DOWN = 1, KICKER_TWO_UP = 0.7,
            KICKER_THREE_DOWN = 0.057, KICKER_THREE_UP = 1;

    public static double kv = 0.00051;

    public static double kp = 0.004;
    public static double ki = 0;
    public static double kd = 0;
    public static double targetVelocity = 1400;

    int counter = 0;

    public final double X_POD_OFFSET = -1;
    public final double Y_POD_OFFSET = -7.5;





    double TPS;
    double error;

    GoBildaPinpointDriver odo;




    public void initHardware(HardwareMap hardwareMap){

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        //Drive Motors

        fLeft = new Motor(hardwareMap, "fLeft");
        fRight = new Motor(hardwareMap, "fRight");
        bLeft = new Motor(hardwareMap, "bLeft");
        bRight = new Motor(hardwareMap, "bRight");

        fLeft.noEncoder();
        fRight.noEncoder();
        bLeft.noEncoder();
        bRight.noEncoder();

        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Shooter Motors

        shooterOne = hardwareMap.get(DcMotorEx.class, "shooterOne");
        shooterTwo = hardwareMap.get(DcMotorEx.class, "shooterTwo");


        //Intake Motors

        frontSweeper = new Motor(hardwareMap, "intake");
        frontSweeper.noEncoder();
        backSweeper = new Motor(hardwareMap, "doubleIntake");
        backSweeper.noEncoder();

        //Turret Servos

        rotOne = hardwareMap.servo.get("rotOne");
        rotTwo = hardwareMap.servo.get("rotTwo");

        //Kicker Servos

        kickerOne = hardwareMap.servo.get("kickIntake");
        kickerTwo = hardwareMap.servo.get("kickOut");
        kickerThree = hardwareMap.servo.get("kickMiddle");


        //Color Sensor Slot Detection

         oneSlot = "";
         twoSlot = "";
         threeSlot = "";

         //Odometry Initialization

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo"); //2.75, 1.75

        odo.setOffsets(-1.75, 2.75, DistanceUnit.INCH);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.resetPosAndIMU();

        //Limelight Initialization
//
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        //Color Sensor Initialization

        slotIntake = hardwareMap.get(RevColorSensorV3.class, "colorIn");
        slotOut = hardwareMap.get(RevColorSensorV3.class, "colorOut");
        finalColor = hardwareMap.get(RevColorSensorV3.class, "colorLast");
        lastColor = hardwareMap.get(RevColorSensorV3.class, "bruhColor");




    }

    public void initHardware(HardwareMap hardwareMap, Boolean useCase){

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        //Drive Motors
        fLeft = new Motor(hardwareMap, "fLeft");
        fRight = new Motor(hardwareMap, "fRight");
        bLeft = new Motor(hardwareMap, "bLeft");
        bRight = new Motor(hardwareMap, "bRight");

        fLeft.noEncoder();
        fRight.noEncoder();
        bLeft.noEncoder();
        bRight.noEncoder();

        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);


//        shooterOne = new Motor(hardwareMap, "shooterOne");
//        shooterTwo = new Motor(hardwareMap, "shooterTwo");
//        shooterOne.noEncoder();
//        shooterTwo.noEncoder();






//        shooterOne.noEncoder();
//        shooterTwo.noEncoder();

//        shooterOne.noEncoder();
//        shooterTwo.noEncoder();

        frontSweeper = new Motor(hardwareMap, "intake");
        frontSweeper.noEncoder();
        backSweeper = new Motor(hardwareMap, "doubleIntake");
        backSweeper.noEncoder();
//        backSweeper = new Motor(hardwareMap, "backSweeper");

//        rotOne = hardwareMap.get(CRServo.class, "rotOne");
//        rotTwo = hardwareMap.get(CRServo.class, "rotTwo");
        rotOne = hardwareMap.servo.get("rotOne");
        rotTwo = hardwareMap.servo.get("rotTwo");


        kickerOne = hardwareMap.servo.get("kickIntake");
        kickerTwo = hardwareMap.servo.get("kickOut");
        kickerThree = hardwareMap.servo.get("kickMiddle");

        oneSlot = "";
        twoSlot = "";
        threeSlot = "";

//        hoodOne = hardwareMap.servo.get("hoodOne");
//        hoodTwo = hardwareMap.servo.get("hoodTwo");
//        turretTrack = hardwareMap.get(AnalogInput.class, "turretTrack");


        //Initialize Modules


        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo"); //2.75, 1.75

        odo.setOffsets(-1.75, 2.75, DistanceUnit.INCH);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.resetPosAndIMU();
//
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        slotIntake = hardwareMap.get(RevColorSensorV3.class, "colorIn");
        slotOut = hardwareMap.get(RevColorSensorV3.class, "colorOut");
        finalColor = hardwareMap.get(RevColorSensorV3.class, "colorLast");
        lastColor = hardwareMap.get(RevColorSensorV3.class, "bruhColor");




    }

    //Kicker Commands
    public void initializeKickers(){
        kickerOne.setPosition(KICKER_ONE_DOWN);
        kickerTwo.setPosition(KICKER_TWO_DOWN);
        kickerThree.setPosition(KICKER_THREE_DOWN);
    }

    public void kickerOneUp(){
        kickerOne.setPosition(KICKER_ONE_UP);
    }

    public void kickerOneDown(){
        kickerOne.setPosition(KICKER_ONE_DOWN);
    }

    public void kickerTwoUp(){
        kickerTwo.setPosition(KICKER_TWO_UP);
    }

    public void kickerTwoDown(){
        kickerTwo.setPosition(KICKER_TWO_DOWN);
    }

    public void kickerThreeUp(){
        kickerThree.setPosition(KICKER_THREE_UP);
    }

    public void kickerThreeDown(){
        kickerThree.setPosition(KICKER_THREE_DOWN);
    }


    //Position Getters

    public double getY(){
        Pose2D pos = odo.getPosition();

        return pos.getY(DistanceUnit.INCH);
    }

    public double getX(){
        Pose2D pos = odo.getPosition();

        return pos.getX(DistanceUnit.INCH);
    }

    public double getAngle(){
        odo.update();
        Pose2D pos = odo.getPosition();
        double angle = pos.getHeading(AngleUnit.DEGREES);
        return angle;
    }

    //Shooter Adjustment Handling
    //
    public double regressFlywheelVelocity(double dist){

        return ((-0.0000105231)*Math.pow(dist, 4)) + ((0.00335882)*Math.pow(dist, 3) -
                ((0.344996)*Math.pow(dist, 2)) + ((16.79623)*dist) + 1044.96012);

    }

    public double regressHoodAngle(double dist){
        return (((2.61303)*Math.pow(10, -7))*Math.pow(dist, 4)) - ((0.0000669)*Math.pow(dist, 3)) +
                ((0.00620573)*Math.pow(dist, 2)) - ((0.248683)*dist)+ 4.20828;
    }





    //MOVEMENT FUNCTIONS

    //Incorporates CarrotChase Movement Algorithm with constantly running Shooter PID
    //TODO: Add parameter for shooter power, instead of setting to fixed velocity
    public void ChaseTheCarrotWithShooter(
            ArrayList<Point> wp,
            int switchTolerance,
            int skip,
            boolean followSplineHeading,
            boolean invertSplineHeading,
            double heading,
            double error,
            double angleError,
            double normalMovementConstant,
            double finalMovementConstant,
            double turnConstant,
            double movementD,
            double turnD,
            double timeout, double powerCap, double shooterVelocity) {
        ElapsedTime time = new ElapsedTime();
        resetCache();
        odo.update();
        Pose2D pos = odo.getPosition();


        double xDiff = Integer.MAX_VALUE, yDiff = Integer.MAX_VALUE, angleDiff = Integer.MAX_VALUE, prevTime = 0, prevXDiff = 0, prevYDiff = 0, prevAngleDiff = 0, splineHeading = 0, maxSpeed = 1;
        double finalSplineHeading = Angle.normalize(Math.toDegrees(Math.atan2(wp.get(wp.size() - 1).yP, wp.get(wp.size() - 1).xP)));
        int pt = 0;
        time.reset();
        while ((pt < wp.size() - 1
                || (Math.abs(pos.getX(DistanceUnit.INCH) - wp.get(wp.size() - 1).xP) > error
                || Math.abs(pos.getY(DistanceUnit.INCH) - wp.get(wp.size() - 1).yP) > error
                || ( followSplineHeading ?  ( invertSplineHeading ? Math.abs(Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())))) > angleError :
                Math.abs(Angle.normalize(finalSplineHeading - getAngle())) > angleError)
                : (heading == Double.MAX_VALUE
                ? Math.abs(angleDiff) > 0
                : Math.abs(Angle.normalize(heading - getAngle())) > angleError) )))

                && time.milliseconds() < timeout && opModeIsActive()) {

            odo.update();
            pos = odo.getPosition();



            resetCache();
            runShooter(1350);


            double x = pos.getX(DistanceUnit.INCH);
            double y = pos.getY(DistanceUnit.INCH);
            double theta = pos.getHeading(AngleUnit.DEGREES);


            if (getRobotDistanceFromPoint(wp.get(pt)) <= switchTolerance && pt != wp.size() - 1) {
                odo.update();

                resetCache();
                pt = Math.min(wp.size()-1, pt+skip);
            }

            Point destPt = wp.get(pt);
            xDiff = destPt.xP - x;
            yDiff = destPt.yP - y;
            splineHeading = Angle.normalize(Math.toDegrees(Math.atan2(yDiff, xDiff)));
            maxSpeed = destPt.speed;

            if(followSplineHeading) {
                if(pt == wp.size() - 1){
                    if(invertSplineHeading){
                        angleDiff = Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())));
                    }else {
                        angleDiff = Angle.normalize(finalSplineHeading - theta);
                    }
                }else {
                    if(invertSplineHeading){
                        angleDiff = Angle.normalize(splineHeading - (-Angle.normalize(180 - getAngle())));
                    }else {
                        angleDiff = Angle.normalize(splineHeading - theta);
                    }
                }
            }else{
                if(heading == Double.MAX_VALUE){
                    if(wp.get(pt).invertSpline){
                        if(pt == wp.size() - 1){
                            angleDiff = Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())));
                        }else {
                            angleDiff = Angle.normalize(splineHeading - (-Angle.normalize(180 - getAngle())));
                        }
                    }else if(wp.get(pt).spline){
                        if(pt == wp.size() - 1){
                            angleDiff = Angle.normalize(finalSplineHeading - theta);
                        }else {
                            angleDiff = Angle.normalize(splineHeading - theta);
                        }
                    }else{
                        angleDiff = Angle.normalize(wp.get(wp.size() - 1).ang - theta);
                    }
                }else{
                    angleDiff = Angle.normalize(heading - theta);
                }
            }

            double xPow=0, yPow=0, turnPow=0;

            turnPow += angleDiff * turnConstant;
            if (pt == wp.size() - 1) {
                xPow += xDiff * finalMovementConstant;
                yPow += yDiff * finalMovementConstant;
                xPow += movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime);
                yPow += movementD * (yDiff - prevYDiff) / (time.seconds() - prevTime);
                turnPow += turnD * (angleDiff - prevAngleDiff) / (time.seconds() - prevTime);
                System.out.println((movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime)));
            } else {
                xPow += xDiff * normalMovementConstant;
                yPow += yDiff * normalMovementConstant;
            }

            turnPow = Range.clip(turnPow, -1, 1);
            xPow = Range.clip(xPow, -maxSpeed, maxSpeed);
            yPow = Range.clip(yPow, -maxSpeed, maxSpeed);
            System.out.println(maxSpeed);


            prevTime = time.seconds();
            prevXDiff = xDiff;
            prevYDiff = yDiff;
            prevAngleDiff = angleDiff;

            //driveFieldCentric(-yPow, -turnPow, xPow);
            telemetry.addData("target: ", destPt);
            telemetry.update();
            driveFieldCentric(-xPow, turnPow, yPow, powerCap);

        }
        stopDrive();
    }

    //Runs Classic CarrotChase Movement function with Limelight sensing to detect obelisk during auto
    public void ChaseTheCarrotWithLimelight(
            ArrayList<Point> wp,
            int switchTolerance,
            int skip,
            boolean followSplineHeading,
            boolean invertSplineHeading,
            double heading,
            double error,
            double angleError,
            double normalMovementConstant,
            double finalMovementConstant,
            double turnConstant,
            double movementD,
            double turnD,
            double timeout, double powerCap) {
        ElapsedTime time = new ElapsedTime();
        resetCache();
        odo.update();
        Pose2D pos = odo.getPosition();



        double xDiff = Integer.MAX_VALUE, yDiff = Integer.MAX_VALUE, angleDiff = Integer.MAX_VALUE, prevTime = 0, prevXDiff = 0, prevYDiff = 0, prevAngleDiff = 0, splineHeading = 0, maxSpeed = 1;
        double finalSplineHeading = Angle.normalize(Math.toDegrees(Math.atan2(wp.get(wp.size() - 1).yP, wp.get(wp.size() - 1).xP)));
        int pt = 0;
        time.reset();
        while ((pt < wp.size() - 1
                || (Math.abs(pos.getX(DistanceUnit.INCH) - wp.get(wp.size() - 1).xP) > error
                || Math.abs(pos.getY(DistanceUnit.INCH) - wp.get(wp.size() - 1).yP) > error
                || ( followSplineHeading ?  ( invertSplineHeading ? Math.abs(Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())))) > angleError :
                Math.abs(Angle.normalize(finalSplineHeading - getAngle())) > angleError)
                : (heading == Double.MAX_VALUE
                ? Math.abs(angleDiff) > 0
                : Math.abs(Angle.normalize(heading - getAngle())) > angleError) )))

                && time.milliseconds() < timeout && opModeIsActive()) {

            odo.update();
            pos = odo.getPosition();

            resetCache();
            runShooter(1300);
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                resetCache();
                if (result.isValid()) {

                    int tagIDR = result.getFiducialResults().get(0).getFiducialId();
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("ID", tagIDR);

                    if(tagIDR < 24 && tagIDR > 20){
                        counter = tagIDR;
                    }

                    telemetry.addData("Counter", counter);
                    telemetry.update();

                }else{


                    telemetry.addData("So", "Cooked");
                    telemetry.addData("Counter", counter);
                    telemetry.update();
                }
            }else{
                telemetry.addData("Result", "Bruh");
                telemetry.addData("Counter", counter);

                telemetry.update();
            }


            double x = pos.getX(DistanceUnit.INCH);
            double y = pos.getY(DistanceUnit.INCH);
            double theta = pos.getHeading(AngleUnit.DEGREES);


            if (getRobotDistanceFromPoint(wp.get(pt)) <= switchTolerance && pt != wp.size() - 1) {
                odo.update();

                resetCache();
                pt = Math.min(wp.size()-1, pt+skip);
            }

            Point destPt = wp.get(pt);
            xDiff = destPt.xP - x;
            yDiff = destPt.yP - y;
            splineHeading = Angle.normalize(Math.toDegrees(Math.atan2(yDiff, xDiff)));
            maxSpeed = destPt.speed;

            if(followSplineHeading) {
                if(pt == wp.size() - 1){
                    if(invertSplineHeading){
                        angleDiff = Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())));
                    }else {
                        angleDiff = Angle.normalize(finalSplineHeading - theta);
                    }
                }else {
                    if(invertSplineHeading){
                        angleDiff = Angle.normalize(splineHeading - (-Angle.normalize(180 - getAngle())));
                    }else {
                        angleDiff = Angle.normalize(splineHeading - theta);
                    }
                }
            }else{
                if(heading == Double.MAX_VALUE){
                    if(wp.get(pt).invertSpline){
                        if(pt == wp.size() - 1){
                            angleDiff = Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())));
                        }else {
                            angleDiff = Angle.normalize(splineHeading - (-Angle.normalize(180 - getAngle())));
                        }
                    }else if(wp.get(pt).spline){
                        if(pt == wp.size() - 1){
                            angleDiff = Angle.normalize(finalSplineHeading - theta);
                        }else {
                            angleDiff = Angle.normalize(splineHeading - theta);
                        }
                    }else{
                        angleDiff = Angle.normalize(wp.get(wp.size() - 1).ang - theta);
                    }
                }else{
                    angleDiff = Angle.normalize(heading - theta);
                }
            }

            double xPow=0, yPow=0, turnPow=0;

            turnPow += angleDiff * turnConstant;
            if (pt == wp.size() - 1) {
                xPow += xDiff * finalMovementConstant;
                yPow += yDiff * finalMovementConstant;
                xPow += movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime);
                yPow += movementD * (yDiff - prevYDiff) / (time.seconds() - prevTime);
                turnPow += turnD * (angleDiff - prevAngleDiff) / (time.seconds() - prevTime);
                System.out.println((movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime)));
            } else {
                xPow += xDiff * normalMovementConstant;
                yPow += yDiff * normalMovementConstant;
            }

            turnPow = Range.clip(turnPow, -1, 1);
            xPow = Range.clip(xPow, -maxSpeed, maxSpeed);
            yPow = Range.clip(yPow, -maxSpeed, maxSpeed);
            System.out.println(maxSpeed);


            prevTime = time.seconds();
            prevXDiff = xDiff;
            prevYDiff = yDiff;
            prevAngleDiff = angleDiff;

            //driveFieldCentric(-yPow, -turnPow, xPow);

            driveFieldCentric(-xPow, turnPow, yPow, powerCap);

        }
        stopDrive();
    }

    //Runs classic CarrotChase Movement with Color Sensor reading to determine ball placement in slots
    //TODO: Replace Color Sensors w/ Logitech Camera
    public void ChaseTheCarrotWithColor(
            ArrayList<Point> wp,
            int switchTolerance,
            int skip,
            boolean followSplineHeading,
            boolean invertSplineHeading,
            double heading,
            double error,
            double angleError,
            double normalMovementConstant,
            double finalMovementConstant,
            double turnConstant,
            double movementD,
            double turnD,
            double timeout, double powerCap) {
        ElapsedTime time = new ElapsedTime();
        resetCache();
        odo.update();
        Pose2D pos = odo.getPosition();



        double xDiff = Integer.MAX_VALUE, yDiff = Integer.MAX_VALUE, angleDiff = Integer.MAX_VALUE, prevTime = 0, prevXDiff = 0, prevYDiff = 0, prevAngleDiff = 0, splineHeading = 0, maxSpeed = 1;
        double finalSplineHeading = Angle.normalize(Math.toDegrees(Math.atan2(wp.get(wp.size() - 1).yP, wp.get(wp.size() - 1).xP)));
        int pt = 0;
        time.reset();
        while ((pt < wp.size() - 1
                || (Math.abs(pos.getX(DistanceUnit.INCH) - wp.get(wp.size() - 1).xP) > error
                || Math.abs(pos.getY(DistanceUnit.INCH) - wp.get(wp.size() - 1).yP) > error
                || ( followSplineHeading ?  ( invertSplineHeading ? Math.abs(Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())))) > angleError :
                Math.abs(Angle.normalize(finalSplineHeading - getAngle())) > angleError)
                : (heading == Double.MAX_VALUE
                ? Math.abs(angleDiff) > 0
                : Math.abs(Angle.normalize(heading - getAngle())) > angleError) )))

                && time.milliseconds() < timeout && opModeIsActive()) {

            odo.update();
            pos = odo.getPosition();

            resetCache();
            runShooter(1300);
            if(lastColor.getDistance(DistanceUnit.CM) < 2.2 || finalColor.getDistance(DistanceUnit.CM) < 2.2){
                if(lastColor.getDistance(DistanceUnit.CM)<finalColor.getDistance(DistanceUnit.CM)){
                    if(lastColor.green() > lastColor.blue()){
                        oneSlot = "green";
                    }else{
                        oneSlot = "purple";
                    }
                }else{
                    if(finalColor.green() > finalColor.green()){
                        oneSlot = "green";
                    }else{
                        oneSlot = "purple";
                    }
                }
            }else{
                oneSlot = "empty";
            }

            if(slotIntake.getDistance(DistanceUnit.CM) < 3.0 || slotOut.getDistance(DistanceUnit.CM) < 3.0){
                if(slotIntake.getDistance(DistanceUnit.CM)<slotOut.getDistance(DistanceUnit.CM)){
                    if(slotIntake.green() > slotIntake.blue()){
                        twoSlot = "green";
                    }else{
                        twoSlot = "purple";
                    }
                }else{
                    if(slotOut.green() > slotOut.blue()){
                        twoSlot = "green";
                    }else{
                        twoSlot = "purple";
                    }
                }
            }else{
                twoSlot = "empty";
            }

            if(oneSlot.equals("purple") && twoSlot.equals("purple")){
                threeSlot = ("green");
            }else{
                threeSlot = ("purple");
            }

            if(twoSlot.equals("empty") || oneSlot.equals("empty")){
                twoSlot = "purple";
                oneSlot = "purple";
            }



            double x = pos.getX(DistanceUnit.INCH);
            double y = pos.getY(DistanceUnit.INCH);
            double theta = pos.getHeading(AngleUnit.DEGREES);


            if (getRobotDistanceFromPoint(wp.get(pt)) <= switchTolerance && pt != wp.size() - 1) {
                odo.update();

                resetCache();
                pt = Math.min(wp.size()-1, pt+skip);
            }

            Point destPt = wp.get(pt);
            xDiff = destPt.xP - x;
            yDiff = destPt.yP - y;
            splineHeading = Angle.normalize(Math.toDegrees(Math.atan2(yDiff, xDiff)));
            maxSpeed = destPt.speed;

            if(followSplineHeading) {
                if(pt == wp.size() - 1){
                    if(invertSplineHeading){
                        angleDiff = Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())));
                    }else {
                        angleDiff = Angle.normalize(finalSplineHeading - theta);
                    }
                }else {
                    if(invertSplineHeading){
                        angleDiff = Angle.normalize(splineHeading - (-Angle.normalize(180 - getAngle())));
                    }else {
                        angleDiff = Angle.normalize(splineHeading - theta);
                    }
                }
            }else{
                if(heading == Double.MAX_VALUE){
                    if(wp.get(pt).invertSpline){
                        if(pt == wp.size() - 1){
                            angleDiff = Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())));
                        }else {
                            angleDiff = Angle.normalize(splineHeading - (-Angle.normalize(180 - getAngle())));
                        }
                    }else if(wp.get(pt).spline){
                        if(pt == wp.size() - 1){
                            angleDiff = Angle.normalize(finalSplineHeading - theta);
                        }else {
                            angleDiff = Angle.normalize(splineHeading - theta);
                        }
                    }else{
                        angleDiff = Angle.normalize(wp.get(wp.size() - 1).ang - theta);
                    }
                }else{
                    angleDiff = Angle.normalize(heading - theta);
                }
            }

            double xPow=0, yPow=0, turnPow=0;

            turnPow += angleDiff * turnConstant;
            if (pt == wp.size() - 1) {
                xPow += xDiff * finalMovementConstant;
                yPow += yDiff * finalMovementConstant;
                xPow += movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime);
                yPow += movementD * (yDiff - prevYDiff) / (time.seconds() - prevTime);
                turnPow += turnD * (angleDiff - prevAngleDiff) / (time.seconds() - prevTime);
                System.out.println((movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime)));
            } else {
                xPow += xDiff * normalMovementConstant;
                yPow += yDiff * normalMovementConstant;
            }

            turnPow = Range.clip(turnPow, -1, 1);
            xPow = Range.clip(xPow, -maxSpeed, maxSpeed);
            yPow = Range.clip(yPow, -maxSpeed, maxSpeed);
            System.out.println(maxSpeed);


            prevTime = time.seconds();
            prevXDiff = xDiff;
            prevYDiff = yDiff;
            prevAngleDiff = angleDiff;

            //driveFieldCentric(-yPow, -turnPow, xPow);
            telemetry.addData("Slot 1", oneSlot);
            telemetry.addData("Slot 2", twoSlot);
            telemetry.addData("Slot 3", threeSlot);
            telemetry.update();
            driveFieldCentric(-xPow, turnPow, yPow, powerCap);

        }
        stopDrive();
    }

    //Classic Chase the Carrot Movement Function

    public void ChaseTheCarrot(
            ArrayList<Point> wp,
            int switchTolerance,
            int skip,
            boolean followSplineHeading,
            boolean invertSplineHeading,
            double heading,
            double error,
            double angleError,
            double normalMovementConstant,
            double finalMovementConstant,
            double turnConstant,
            double movementD,
            double turnD,
            double timeout, double powerCap) {
        ElapsedTime time = new ElapsedTime();
        resetCache();
        odo.update();
        Pose2D pos = odo.getPosition();


        double xDiff = Integer.MAX_VALUE, yDiff = Integer.MAX_VALUE, angleDiff = Integer.MAX_VALUE, prevTime = 0, prevXDiff = 0, prevYDiff = 0, prevAngleDiff = 0, splineHeading = 0, maxSpeed = 1;
        double finalSplineHeading = Angle.normalize(Math.toDegrees(Math.atan2(wp.get(wp.size() - 1).yP, wp.get(wp.size() - 1).xP)));
        int pt = 0;
        time.reset();
        while ((pt < wp.size() - 1
                || (Math.abs(pos.getX(DistanceUnit.INCH) - wp.get(wp.size() - 1).xP) > error
                || Math.abs(pos.getY(DistanceUnit.INCH) - wp.get(wp.size() - 1).yP) > error
                || ( followSplineHeading ?  ( invertSplineHeading ? Math.abs(Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())))) > angleError :
                Math.abs(Angle.normalize(finalSplineHeading - getAngle())) > angleError)
                : (heading == Double.MAX_VALUE
                ? Math.abs(angleDiff) > 0
                : Math.abs(Angle.normalize(heading - getAngle())) > angleError) )))

                && time.milliseconds() < timeout && opModeIsActive()) {

            odo.update();
            pos = odo.getPosition();



            resetCache();


            double x = pos.getX(DistanceUnit.INCH);
            double y = pos.getY(DistanceUnit.INCH);
            double theta = pos.getHeading(AngleUnit.DEGREES);


            if (getRobotDistanceFromPoint(wp.get(pt)) <= switchTolerance && pt != wp.size() - 1) {
                odo.update();

                resetCache();
                pt = Math.min(wp.size()-1, pt+skip);
            }

            Point destPt = wp.get(pt);
            xDiff = destPt.xP - x;
            yDiff = destPt.yP - y;
            splineHeading = Angle.normalize(Math.toDegrees(Math.atan2(yDiff, xDiff)));
            maxSpeed = destPt.speed;

            if(followSplineHeading) {
                if(pt == wp.size() - 1){
                    if(invertSplineHeading){
                        angleDiff = Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())));
                    }else {
                        angleDiff = Angle.normalize(finalSplineHeading - theta);
                    }
                }else {
                    if(invertSplineHeading){
                        angleDiff = Angle.normalize(splineHeading - (-Angle.normalize(180 - getAngle())));
                    }else {
                        angleDiff = Angle.normalize(splineHeading - theta);
                    }
                }
            }else{
                if(heading == Double.MAX_VALUE){
                    if(wp.get(pt).invertSpline){
                        if(pt == wp.size() - 1){
                            angleDiff = Angle.normalize(finalSplineHeading - (-Angle.normalize(180 - getAngle())));
                        }else {
                            angleDiff = Angle.normalize(splineHeading - (-Angle.normalize(180 - getAngle())));
                        }
                    }else if(wp.get(pt).spline){
                        if(pt == wp.size() - 1){
                            angleDiff = Angle.normalize(finalSplineHeading - theta);
                        }else {
                            angleDiff = Angle.normalize(splineHeading - theta);
                        }
                    }else{
                        angleDiff = Angle.normalize(wp.get(wp.size() - 1).ang - theta);
                    }
                }else{
                    angleDiff = Angle.normalize(heading - theta);
                }
            }

            double xPow=0, yPow=0, turnPow=0;

            turnPow += angleDiff * turnConstant;
            if (pt == wp.size() - 1) {
                xPow += xDiff * finalMovementConstant;
                yPow += yDiff * finalMovementConstant;
                xPow += movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime);
                yPow += movementD * (yDiff - prevYDiff) / (time.seconds() - prevTime);
                turnPow += turnD * (angleDiff - prevAngleDiff) / (time.seconds() - prevTime);
                System.out.println((movementD * (xDiff - prevXDiff) / (time.seconds() - prevTime)));
            } else {
                xPow += xDiff * normalMovementConstant;
                yPow += yDiff * normalMovementConstant;
            }

            turnPow = Range.clip(turnPow, -1, 1);
            xPow = Range.clip(xPow, -maxSpeed, maxSpeed);
            yPow = Range.clip(yPow, -maxSpeed, maxSpeed);
            System.out.println(maxSpeed);


            prevTime = time.seconds();
            prevXDiff = xDiff;
            prevYDiff = yDiff;
            prevAngleDiff = angleDiff;

            //driveFieldCentric(-yPow, -turnPow, xPow);
            telemetry.addData("target: ", destPt);
            telemetry.update();
            driveFieldCentric(-xPow, turnPow, yPow, powerCap);

        }
        stopDrive();
    }




    public void shootGreen(){
        if(oneSlot.equals("green")){
            kickerOne.setPosition(KICKER_ONE_UP);
        }else if(twoSlot.equals("green")){
            kickerTwo.setPosition(KICKER_TWO_UP);
        }else{
            kickerThree.setPosition(KICKER_THREE_UP);
        }
    }



    public void driveFieldCentric(double drive, double turn, double strafe, double powerCap){
        Pose2D pos = odo.getPosition();

        double botHeading = pos.getHeading(AngleUnit.RADIANS);

        double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);



        rotX = rotX * 1.1;

        double denominator = Math.max( Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn) , 1);
        double fLeftPow = (rotY + rotX + turn) / denominator;
        double bLeftPow = (rotY - rotX + turn) / denominator;
        double fRightPow = (rotY - rotX - turn) / denominator;
        double bRightPow = (rotY + rotX - turn) / denominator;

        setDrivePowers(fLeftPow, fRightPow, bLeftPow, bRightPow, powerCap);
    }

    public void driveFieldCentricAuto(double drive, double strafe, double angle, double speedCap){
        Pose2D pos = odo.getPosition();

        double botHeading = pos.getHeading(AngleUnit.RADIANS);

        double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
        double angleError = normalizeAngle(angle - pos.getHeading(AngleUnit.DEGREES));
        double anglePow = -0.01 * angleError;

        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(anglePow), 1);
        double fLeftPow = (rotY + rotX + anglePow) / denominator;
        double bLeftPow = (rotY - rotX + anglePow) / denominator;
        double fRightPow = (rotY - rotX - anglePow) / denominator;
        double bRightPow = (rotY + rotX - anglePow) / denominator;

        setDrivePowers(fLeftPow, fRightPow, bLeftPow, bRightPow, speedCap);




    }

    public void setDrivePowers(double fLeftPow, double fRightPow, double bLeftPow, double bRightPow, double powerCap){
        fLeft.setPower(fLeftPow * powerCap);
        fRight.setPower(fRightPow * powerCap);
        bLeft.setPower(bLeftPow * powerCap);
        bRight.setPower(bRightPow * powerCap);
    }

    public void stopDrive(){
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    public void runShooter(double targetVelocity){
        TPS = shooterOne.getVelocity();

        error = Math.abs(targetVelocity) - Math.abs(shooterOne.getVelocity());

        double p = kp * error;
        double feedforward = kv * targetVelocity;

        double pow = p + feedforward;

        double finalPow = Range.clip(pow, -1, 1);

        shooterOne.setPower(-finalPow);
        shooterTwo.setPower(finalPow);
    }


    public double normalizeAngle(double rawAngle) {
        double scaledAngle = rawAngle % 360;
        if (scaledAngle < 0) {
            scaledAngle += 360;
        }

        if (scaledAngle > 180) {
            scaledAngle -= 360;
        }

        return scaledAngle;
    }

    public double getRobotDistanceFromPoint(Point p2) {
        return Math.sqrt((p2.yP - getY()) * (p2.yP - getY()) + (p2.xP - getX()) * (p2.xP - getX()));
    }

    //Overloaded Movement Methods

    public void ChaseTheCarrotConstantHeading(ArrayList<Point> wp,
                                              int switchTolerance,
                                              int skip,
                                              double heading,
                                              double error,
                                              double angleError,
                                              double normalMovementConstant,
                                              double finalMovementConstant,
                                              double turnConstant,
                                              double movementD,
                                              double turnD,
                                              double timeout, double powerCap) {
        ChaseTheCarrot(wp, switchTolerance, skip, false, false, heading, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout, powerCap);
    }

    public void ChaseTheCarrotConstantHeadingWithShooter(ArrayList<Point> wp,
                                              int switchTolerance,
                                              int skip,
                                              double heading,
                                              double error,
                                              double angleError,
                                              double normalMovementConstant,
                                              double finalMovementConstant,
                                              double turnConstant,
                                              double movementD,
                                              double turnD,
                                              double timeout, double powerCap) {
        ChaseTheCarrotWithShooter(wp, switchTolerance, skip, false, false, heading, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout, powerCap, 1020);
    }



    public void ChaseTheCarrotConstantHeadingWithLimeLight(ArrayList<Point> wp,
                                                         int switchTolerance,
                                                         int skip,
                                                         double heading,
                                                         double error,
                                                         double angleError,
                                                         double normalMovementConstant,
                                                         double finalMovementConstant,
                                                         double turnConstant,
                                                         double movementD,
                                                         double turnD,
                                                         double timeout) {
        ChaseTheCarrotWithLimelight(wp, switchTolerance, skip, false, false, heading, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout, 1);
    }

    public void   ChaseTheCarrotConstantHeadingWithColor(ArrayList<Point> wp,
                                                           int switchTolerance,
                                                           int skip,
                                                           double heading,
                                                           double error,
                                                           double angleError,
                                                           double normalMovementConstant,
                                                           double finalMovementConstant,
                                                           double turnConstant,
                                                           double movementD,
                                                           double turnD,
                                                           double timeout, double powerCap) {
        ChaseTheCarrotWithColor(wp, switchTolerance, skip, false, false, heading, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout, powerCap);
    }

    public void   ChaseTheCarrotConstantHeadingWithColor(ArrayList<Point> wp,
                                                         int switchTolerance,
                                                         int skip,
                                                         double heading,
                                                         double error,
                                                         double angleError,
                                                         double normalMovementConstant,
                                                         double finalMovementConstant,
                                                         double turnConstant,
                                                         double movementD,
                                                         double turnD,
                                                         double timeout) {
        ChaseTheCarrotWithColor(wp, switchTolerance, skip, false, false, heading, error, angleError, normalMovementConstant, finalMovementConstant, turnConstant, movementD, turnD, timeout, 1);
    }

    public void ChaseTheCarrotConstantHeading(ArrayList<Point> wp, double heading, double timeout, double powerCap){

        ChaseTheCarrot(wp, 9, 3, false, false, heading,3, 1,  0.05, 0.05, 0.03, 0.0005, 0, timeout, powerCap);
    }



    public void resetCache(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }






}
