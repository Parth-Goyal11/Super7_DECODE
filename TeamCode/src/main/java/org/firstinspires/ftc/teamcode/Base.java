package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Core.Motor;

import java.util.List;

public abstract class Base extends LinearOpMode{

    public List<LynxModule> allHubs;
    public Motor fLeft, bLeft, fRight, bRight;

    public Motor shooterOne, shooterTwo, sweeper;
    CRServo wheelOne, wheelTwo;
    Servo indexer, spin, kick, stop, angleLeft, angleRight;
    public final double X_POD_OFFSET = -1;
    public final double Y_POD_OFFSET = -7.5;

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



        //Module Motors
//        shooterOne = new Motor(hardwareMap, "shooterOne");
//        shooterTwo = new Motor(hardwareMap, "shooterTwo");
//        sweeper = new Motor(hardwareMap, "sweeper");
//
//        shooterOne.useEncoder();
//        shooterTwo.useEncoder();
//
//
//        wheelOne = hardwareMap.get(CRServo.class, "wheelOne");
//        wheelTwo = hardwareMap.get(CRServo.class, "wheelTwo");
//        indexer = hardwareMap.get(Servo.class, "indexer");
//        spin = hardwareMap.get(Servo.class, "spin");
//        kick = hardwareMap.get(Servo.class, "kicker");
//        stop = hardwareMap.get(Servo.class, "stop");
//
//        angleLeft = hardwareMap.get(Servo.class, "angleLeft");
//        angleRight = hardwareMap.get(Servo.class, "angleRight");
//        angleLeft.setPosition(0.7478);
//        angleRight.setPosition(0.2339);
//        indexer.setPosition(0);
////        spin.setPosition(0.097);//Lower
//        kick.setPosition(0.63);
//        stop.setPosition(0.453);
        //Instantiate Odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odo.setOffsets(3.5, 2.6, DistanceUnit.INCH);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.resetPosAndIMU();




    }

//    public void moveToPosition(double targetX, double targetY, double targetAngle, double MOE, double angleMOE, double maxTime){
//        ElapsedTime timer = new ElapsedTime();
//        double xErr = 200, yErr = 200, angleErr = 200;
//        resetCache();
//        double previousX = 0, previousY = 0, previousTheta = 0;
//        double previousTime = 0;
//        while (timer.milliseconds() <= maxTime &&
//                (Math.abs(xErr) >= MOE || Math.abs(yErr) >= MOE || Math.abs(angleErr) >= MOE)) {
//
//            odo.update();
//            Pose2D pos = odo.getPosition();
//            resetCache();
//
//            double kp = 0.055;
//            double kd = 0.03;
//            double currentTime = timer.milliseconds();
//
//            xErr = targetX - pos.getX(DistanceUnit.INCH);
//            yErr = targetY - pos.getY(DistanceUnit.INCH);
//            angleErr = targetAngle - pos.getHeading(AngleUnit.DEGREES);
//
//            double anglePow = 0.01 * angleErr;
//
//
//            double xP = kp * xErr;
//            double yP = kp * yErr;
//            double aP = 0.01 * angleErr;
//
//
//            double xD = kd * (xErr - previousX) / (currentTime - previousTime);
//            double yD =  kd * (yErr - previousY) / (currentTime - previousTime);
//            double aD = kd * (angleErr - previousTheta) / (currentTime - previousTime);
//
//            double xPow = xP ;
//            double yPow = yP ;
//            double aPow = aP;
//
//
//
//            driveFieldCentric(-xPow, aPow, yPow, 0.15);
//
//            previousX = xErr;
//            previousY = yErr;
//            previousTheta = angleErr;
//            previousTime = currentTime;
//            telemetry.addData("Angle Error", angleErr);
//            telemetry.addData("X Error", xErr);
//            telemetry.addData("Y Error", yErr);
//            telemetry.update();
//
//        }
//        setDrivePowers(0, 0, 0,0, 1);
//
//    }


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

    public void resetCache(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }






}
