package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

    public Motor shooterOne, shooterTwo, frontSweeper, backSweeper;
    Servo rotOne, rotTwo;
    Servo kickerOne, kickerTwo, kickerThree;
    Servo hoodOne, hoodTwo;
    RevColorSensorV3 slotOne, slotTwo, slotThree;
    AnalogInput turretTrack;

    public static double kv = 0.000543;

    public static double kp = 0.015;
    public static double ki = 0;
    public static double kd = 0;
    public static double targetVelocity = 1400;

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


        shooterOne = new Motor(hardwareMap, "shooterOne");
        shooterTwo = new Motor(hardwareMap, "shooterTwo");
        shooterOne.noEncoder();
        shooterTwo.noEncoder();






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

//        hoodOne = hardwareMap.servo.get("hoodOne");
//        hoodTwo = hardwareMap.servo.get("hoodTwo");
//        turretTrack = hardwareMap.get(AnalogInput.class, "turretTrack");


        //Initialize Modules


        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo"); //2.75, 1.75

        odo.setOffsets(-1.75, 2.75, DistanceUnit.INCH);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.resetPosAndIMU();




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


            runShooter(targetVelocity);
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



    public void moveToPosition(double targetX, double targetY, double targetAngle, double MOE, double angleMOE, double powerCap, double maxTime){
        ElapsedTime timer = new ElapsedTime();
        double xErr = 200, yErr = 200, angleErr = 200;
        resetCache();
        double previousX = 0, previousY = 0, previousTheta = 0;
        double previousTime = 0;
        while (timer.milliseconds() <= maxTime &&
                (Math.abs(xErr) >= MOE || Math.abs(yErr) >= MOE || Math.abs(angleErr) >= MOE)) {

            odo.update();
            Pose2D pos = odo.getPosition();
            resetCache();

            double kp = 0.055;
            double kd = 0.0005;
            double currentTime = timer.milliseconds();

            xErr = targetX - pos.getX(DistanceUnit.INCH);
            yErr = targetY - pos.getY(DistanceUnit.INCH);
            angleErr = targetAngle - pos.getHeading(AngleUnit.DEGREES);

            double anglePow = 0.01 * angleErr;


            double xP = kp * xErr;
            double yP = kp * yErr;
            double aP = 0.01 * angleErr;


            double xD = kd * (xErr - previousX) / (currentTime - previousTime);
            double yD =  kd * (yErr - previousY) / (currentTime - previousTime);
            double aD = kd * (angleErr - previousTheta) / (currentTime - previousTime);
            double aPow = Range.clip((angleErr * 0.03) + aD , -0.7, 0.7);

            double xPow = xP ;
            double yPow = yP ;


            driveFieldCentric(-xPow, aPow, yPow, powerCap);

            previousX = xErr;
            previousY = yErr;
            previousTheta = angleErr;
            previousTime = currentTime;
            telemetry.addData("Angle Error", angleErr);
            telemetry.addData("X Error", xErr);
            telemetry.addData("Y Error", yErr);
            telemetry.update();

        }
        setDrivePowers(0, 0, 0,0, 1);

    }

    public void turnTo(double targetAngle, long timeout, double powerCap, double minDifference){
        odo.update();
        Pose2D pos = odo.getPosition();
        double currAngle = pos.getHeading(AngleUnit.DEGREES);
        double prevError = 0;
        double currentTime = 0;
        double previousTime = 0;
        ElapsedTime time = new ElapsedTime();
        while (Math.abs(currAngle-targetAngle)>minDifference
                && time.milliseconds() < timeout){
            resetCache();
            odo.update();
            double kd = 0.0005;
            currAngle = odo.getPosition().getHeading(AngleUnit.DEGREES);
            currentTime = time.milliseconds();
            double angleDiff = targetAngle - currAngle;
            double aD = kd * (angleDiff - prevError) / (currentTime - previousTime);
            double power = Range.clip((angleDiff * 0.001) + aD , -powerCap, powerCap);

            prevError = angleDiff;
            previousTime = currentTime;

            driveFieldCentric(0, power, 0, 1);

            telemetry.addData("Angle Error", angleDiff);
            telemetry.addData("Power", power);
            telemetry.update();
        }
        currAngle = odo.getPosition().getHeading(AngleUnit.DEGREES);
        double angleDiff = normalizeAngle(targetAngle - currAngle);
        telemetry.addData("Angle Error", angleDiff);
        telemetry.update();

        stopDrive();
    }




    public double calculateD(double curr_error, double prev_error, double curr_time, double prev_time){

        double k_d = 0.01; //Tune this coefficient
        double error_diff = curr_error - prev_error;
        double time_diff = curr_time - prev_time;

        double errorByTime = error_diff / time_diff;
        double d = k_d * errorByTime;

        return d;
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
        TPS = shooterOne.retMotorEx().getVelocity();

        error = targetVelocity - shooterOne.retMotorEx().getVelocity();

        double p = kp * error;
        double feedforward = kv * targetVelocity;

        double pow = p + feedforward;

        double finalPow = Range.clip(pow, -1, 1);

        shooterOne.setPower(finalPow);
        shooterTwo.setPower(-finalPow);
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

    public void ChaseTheCarrotConstantHeading(ArrayList<Point> wp, double heading, double timeout, double powerCap){

        ChaseTheCarrot(wp, 9, 3, false, false, heading,3, 1,  0.05, 0.05, 0.03, 0.0005, 0, timeout, powerCap);
    }



    public void resetCache(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }






}
