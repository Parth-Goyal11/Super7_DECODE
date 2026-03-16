package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Core.Motor;

import java.util.List;

@Config
@TeleOp(name="States Blue Teleop")
public class StatesBlueTele extends LinearOpMode{
    public static double kv = 0.00051;

    public static double kp = 0.004;
    public static double ki = 0;
    public static double kd = 0;
    public static double targetVelocity = 1800; //1800 for far

    double targetHood = 0.5;

    boolean upLast = false, upCurr = false;
    boolean downLast = false, downCurr = false;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
//






        boolean triggerOneLast = false, triggerOneCurr = false, triggerTwoLast = false, triggerTwoCurr = false,
                triggerThreeLast = false, triggerThreeCurr = false, triggerFourCurr = false, triggerFourLast = false;


        double KICKER_ONE_DOWN = 0.953, KICKER_ONE_UP = 0.7, KICKER_TWO_DOWN = 1, KICKER_TWO_UP = 0.7,
                KICKER_THREE_DOWN = 0.057, KICKER_THREE_UP = 1;
        double powerCap = 1;
        ElapsedTime timerOne = new ElapsedTime();
        ElapsedTime timerTwo = new ElapsedTime();
        ElapsedTime timerThree = new ElapsedTime();
        ElapsedTime kickTimer = new ElapsedTime();
        ElapsedTime kickerTimerTwo = new ElapsedTime();
        ElapsedTime kickTimerThree = new ElapsedTime();

        ElapsedTime timerX = new ElapsedTime();
        ElapsedTime timerY = new ElapsedTime();
        ElapsedTime timerB = new ElapsedTime();

        //New Numerical Variables
        int counter = 0;
        double goalX  = 72, goalY = 72;
        double degreeToServo = ((0.4994 - 0.3598) / 98);

        //New Booleans
        boolean autoAim = true;

        //Boolean Button Variables
        boolean turrUpCurr = false, turrUpLast = false;
        boolean turrDownCurr = false, turrDownLast = false;

        boolean twoXCurr = false, twoXLast = false;
        boolean twoYCurr = false, twoYLast = false;
        boolean twoBCurr = false, twoBLast = false;
        boolean resetLast = false, resetCurr = false;

        //Booleans for Individual Kickers
        boolean kickX = false, kickY = false, kickB = false;

        double tShoot = 0.2;



        //Lynx Hub and Drive Motors

        List<LynxModule> allHubs;
        Motor fLeft, bLeft, fRight, bRight;



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

        DcMotorEx shooter, shooterTwo;

        shooter = hardwareMap.get(DcMotorEx.class, "shooterOne");
        shooterTwo = hardwareMap.get(DcMotorEx.class, "shooterTwo");



        //Kicker Initialization

        Servo kickerOne, kickerTwo, kickerThree;

        kickerOne = hardwareMap.servo.get("kickIntake");
        kickerTwo = hardwareMap.servo.get("kickOut");
        kickerThree = hardwareMap.servo.get("kickMiddle");

        //Sweeper Initialization

        Motor frontSweeper, backSweeper;

        frontSweeper = new Motor(hardwareMap, "intake");
        frontSweeper.noEncoder();
        backSweeper = new Motor(hardwareMap, "doubleIntake");
        backSweeper.noEncoder();

        //Turret Servo Initialization

        Servo rotOne, rotTwo;
        rotOne = hardwareMap.servo.get("rotOne");
        rotTwo = hardwareMap.servo.get("rotTwo");

        //Hood Servo
        Servo hood;
        hood = hardwareMap.servo.get("hoodOne");


        //Odometry Initialization
        GoBildaPinpointDriver odo;

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo"); //2.75, 1.75

        odo.setOffsets(-1.75, 2.75, DistanceUnit.INCH);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.resetPosAndIMU();

        //Color Sensors
//
        RevColorSensorV3 slotIntake = hardwareMap.get(RevColorSensorV3.class, "colorIn");
        RevColorSensorV3 slotOut = hardwareMap.get(RevColorSensorV3.class, "colorOut");
        RevColorSensorV3 finalColor = hardwareMap.get(RevColorSensorV3.class, "colorLast");
        RevColorSensorV3 lastColor = hardwareMap.get(RevColorSensorV3.class, "bruhColor");

        //Strings
        String slotOne = "";
        String slotTwo = "";


        boolean changeLast = false, changeCurr = false;
        boolean kickOne = false, kickTwo = false, kickThree = false;
        boolean bruhKick = false, bruhKickTwo = false, bruhKickThree = false;
        boolean incLast = false, incCurr = false;
        boolean shooterMode = false;


        double TPS = 0;
        double error = 0;
        double power = 0;

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        waitForStart();

        hood.setPosition(0.5);

        odo.setPosition(new Pose2D(DistanceUnit.INCH, 36.91, -22.85, AngleUnit.DEGREES, odo.getHeading(AngleUnit.DEGREES)));

        for (LynxModule hub : allHubs) {  //Reset Cache
            hub.clearBulkCache();
        }

        while(opModeIsActive()){

            for (LynxModule hub : allHubs) {  //Reset Cache
                hub.clearBulkCache();
            }

            odo.update();
            Pose2D pos = odo.getPosition();



            double xTrans = pos.getX(DistanceUnit.INCH) - 60;
            double yTrans = pos.getY(DistanceUnit.INCH) + 60;

            double xVelo = odo.getVelX(DistanceUnit.INCH);
            double yVelo = odo.getVelY(DistanceUnit.INCH);

            double xPred = xTrans;


            double total = Math.pow(Math.abs(xTrans), 2) + Math.pow(Math.abs(yTrans), 2);

            double totalDist = Math.sqrt(total);
            double addOn = counter * 0.0035;
            double theta = Math.toDegrees(Math.atan2(goalX - pos.getX(DistanceUnit.INCH), goalY + pos.getY(DistanceUnit.INCH))) - (pos.getHeading(AngleUnit.DEGREES));
            double turretPos =  0.486 + (theta*degreeToServo)  + addOn;




            if(autoAim) {

                rotOne.setPosition(turretPos);
                rotTwo.setPosition(turretPos);
            }



            turrUpLast = turrUpCurr;
            turrUpCurr = gamepad2.dpad_right;
            if(turrUpCurr && !turrUpLast){
                autoAim = false;
                rotOne.setPosition(rotOne.getPosition() - 0.0035);
                rotTwo.setPosition(rotTwo.getPosition() - 0.0035);
                counter --;
            }

            turrDownLast = turrDownCurr;
            turrDownCurr = gamepad2.dpad_left;
            if(turrDownCurr && !turrDownLast){
                autoAim = false;
                rotOne.setPosition(rotOne.getPosition() + 0.0035);
                rotTwo.setPosition(rotTwo.getPosition() + 0.0035);
                counter++;
            }

            if(gamepad2.dpad_up){
                hood.setPosition(hood.getPosition() + 0.01);
            }

            if(gamepad2.dpad_down){
                hood.setPosition(hood.getPosition() - 0.01);
            }



            if(!autoAim && (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1) ){
                autoAim = true;
            }

            odo.update();

            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double strafe = -gamepad1.left_stick_x;

            pos = odo.getPosition();

            double botHeading = pos.getHeading(AngleUnit.RADIANS);

            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);



            rotX = rotX * 1.1;

            double denominator = Math.max( Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn) , 1);
            double fLeftPow = (rotY + rotX + turn) / denominator;
            double bLeftPow = (rotY - rotX + turn) / denominator;
            double fRightPow = (rotY - rotX - turn) / denominator;
            double bRightPow = (rotY + rotX - turn) / denominator;

            fLeft.setPower(fLeftPow);
            fRight.setPower(fRightPow);
            bLeft.setPower(bLeftPow);
            bRight.setPower(bRightPow);





            if(gamepad2.left_bumper){
                shooterMode = true;
            }

            if(gamepad2.right_bumper){
                shooterMode = false;
            }

            if(shooterMode){
                TPS = shooter.getVelocity();

                error = Math.abs(targetVelocity) - Math.abs(shooter.getVelocity());

                double p = kp * error;
                double feedforward = kv * targetVelocity;

                double pow = p + feedforward;

                shooter.setPower(-pow);
                shooterTwo.setPower(pow);
                power = pow;

            }else{
                shooter.setPower(0);
                shooterTwo.setPower(0);
                power = 0;
            }

            downLast = downCurr;
            downCurr = gamepad1.dpad_down;
            if(downCurr && !downLast){
                targetVelocity -= 50;
            }

            upLast = upCurr;
            upCurr = gamepad1.dpad_up;
            if(upCurr && !upLast){
                targetVelocity += 50;
            }

            if (gamepad1.left_bumper) {
                frontSweeper.setPower(-1);
                backSweeper.setPower(0);
            } else if (gamepad1.right_bumper) {
                backSweeper.setPower(1);
                frontSweeper.setPower(0);
            } else {
                frontSweeper.setPower(0);
                backSweeper.setPower(0);

            }

            triggerOneLast = triggerOneCurr;
            triggerOneCurr = gamepad2.a;


            //Order - GPP

            //back slot 1 front slot 2 middle slot 3

            if (triggerOneCurr && !triggerOneLast) {


                kickerThree.setPosition(KICKER_THREE_UP);
                kickOne = true;
                timerOne.reset();
            }

            if (timerOne.milliseconds() > 200 && kickOne) {
                kickerThree.setPosition(KICKER_THREE_DOWN);
                kickOne = false;
                kickTimer.reset();
                bruhKick = true;
            }

            if (kickTimer.milliseconds() > 95  && bruhKick) {
                kickerTwo.setPosition(KICKER_TWO_UP);
                kickTwo = true;
                bruhKick = false;
                timerTwo.reset();
            }

            if (timerTwo.milliseconds() > 200 && kickTwo) {
                kickerTwo.setPosition(KICKER_TWO_DOWN);
                kickTwo = false;
                kickerTimerTwo.reset();
                bruhKickTwo = true;

            }

            if (kickerTimerTwo.milliseconds() > 95 && bruhKickTwo) {
                kickerOne.setPosition(KICKER_ONE_UP);
                kickThree = true;
                bruhKickTwo = false;
                timerThree.reset();
            }

            if (timerThree.milliseconds() > 300 && kickThree) {
                kickerOne.setPosition(KICKER_ONE_DOWN);
                kickThree = false;
                kickTimerThree.reset();
                bruhKickThree = true;
            }

            //Individual Triggers for Kickers

            twoYLast = twoYCurr;
            twoYCurr = gamepad2.y;

            if(twoYCurr && !twoYLast){
                timerY.reset();
                kickerThree.setPosition(KICKER_THREE_UP);
                kickY = true;
            }

            if(timerY.milliseconds() > 200 && kickY){
                kickerThree.setPosition(KICKER_THREE_DOWN);
                kickY = false;
            }

            twoBLast = twoBCurr;
            twoBCurr = gamepad2.b;

            if(twoBCurr && !twoBLast){
                timerB.reset();
                kickerTwo.setPosition(KICKER_TWO_UP);
                kickB = true;
            }

            if(timerB.milliseconds() > 200 && kickB){
                kickerTwo.setPosition(KICKER_TWO_DOWN);
                kickB = false;
            }

            twoXLast = twoXCurr;
            twoXCurr = gamepad2.x;

            if(twoXCurr && !twoXLast){
                timerX.reset();
                kickerOne.setPosition(KICKER_ONE_UP);
                kickX = true;
            }

            if(timerX.milliseconds() > 200 && kickX){
                kickerOne.setPosition(KICKER_ONE_DOWN);
                kickX = false;
            }

            resetLast = resetCurr;
            resetCurr = gamepad2.left_stick_button;

            if(resetCurr && !resetLast){
                odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, odo.getHeading(AngleUnit.DEGREES)));
            }


            //Regression for Velocity - Quartic -> R^2 = 0.99

            targetVelocity = ((-0.0000105231)*Math.pow(totalDist, 4)) + ((0.00335882)*Math.pow(totalDist, 3) - ((0.344996)*Math.pow(totalDist, 2)) + ((16.79623)*totalDist) + 1044.96012);


            //Regression for Hood - Quartic -> R^2 = 1

            targetHood = (((2.61303)*Math.pow(10, -7))*Math.pow(totalDist, 4)) - ((0.0000669)*Math.pow(totalDist, 3)) + ((0.00620573)*Math.pow(totalDist, 2)) - ((0.248683)*totalDist)+ 4.20828;

            //Restrictions on Regression
            //  -Hood locked to 0.5083 for far zone
            //  -Hood Bounded
            if(targetVelocity > 1600){
                targetHood = 0.5083;
            }
            if(targetHood < 0.45){
                targetHood = 0.45;
            }

            if(targetHood > 0.9094){
                targetHood = 0.9094;
            }

            hood.setPosition(targetHood);





            if(lastColor.getDistance(DistanceUnit.CM) < 2.2 || finalColor.getDistance(DistanceUnit.CM) < 2.2){
                if(lastColor.getDistance(DistanceUnit.CM)<finalColor.getDistance(DistanceUnit.CM)){
                    if(lastColor.green() > lastColor.blue()){
                        slotOne = "green";
                    }else{
                        slotOne = "purple";
                    }
                }else{
                    if(finalColor.green() > finalColor.green()){
                        slotOne = "green";
                    }else{
                        slotOne = "purple";
                    }
                }
            }else{
                slotOne = "empty";
            }

            if(slotIntake.getDistance(DistanceUnit.CM) < 3.0 || slotOut.getDistance(DistanceUnit.CM) < 3.0){
                if(slotIntake.getDistance(DistanceUnit.CM)<slotOut.getDistance(DistanceUnit.CM)){
                    if(slotIntake.green() > slotIntake.blue()){
                        slotTwo = "green";
                    }else{
                        slotTwo = "purple";
                    }
                }else{
                    if(slotOut.green() > slotOut.blue()){
                        slotTwo = "green";
                    }else{
                        slotTwo = "purple";
                    }
                }
            }else{
                slotTwo = "empty";
            }



            telemetry.addData("Shooter Velo", Math.abs(shooter.getVelocity()));
            telemetry.addData("Shooter Velo 2", Math.abs(shooterTwo.getVelocity()));
            telemetry.addData("Motor Current", shooter.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Pow", power);
            telemetry.addData("Shooter 2", shooterTwo.getPower());
            telemetry.addData("Hood", hood.getPosition());

            dashboardTelemetry.addData("Velocity", Math.abs(shooter.getVelocity()));
            dashboardTelemetry.addData("Target Velocity", targetVelocity);
            dashboardTelemetry.update();

            telemetry.addData("X", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Trans X", xTrans);
            telemetry.addData("Trans Y", yTrans);
            telemetry.addData("Distance", totalDist);

            telemetry.addData("Color 1", slotIntake.green());
            telemetry.addData("Color 1 Red", slotIntake.red());
            telemetry.addData("Color 1 Blue", slotIntake.blue());
            telemetry.addData("Color 1 Dist", slotIntake.getDistance(DistanceUnit.CM));
            telemetry.addData("Line", "Break");

            telemetry.addData("Color 2", slotOut.green());
            telemetry.addData("Color 2 Red", slotOut.red());
            telemetry.addData("Color 2 Blue", slotOut.blue());
            telemetry.addData("Color 2 Dist", slotOut.getDistance(DistanceUnit.CM));
            telemetry.addData("Line", "Break");





            telemetry.addData("Color 3", lastColor.green());

            telemetry.addData("Color 3 Blue", lastColor.blue());
            telemetry.addData("Color 3 Dist", lastColor.getDistance(DistanceUnit.CM));

            telemetry.addData("Line", "Break");

            telemetry.addData("Color 4", finalColor.green());

            telemetry.addData("Color 4 Blue", finalColor.blue());
            telemetry.addData("Color 4 Dist", finalColor.getDistance(DistanceUnit.CM));


            telemetry.addData("Highest Slot 2", Math.max(lastColor.green(), finalColor.green()));

            telemetry.addData("Slot One", slotOne);
            telemetry.addData("Slot Two", slotTwo);


            telemetry.update();












        }















    }
}

