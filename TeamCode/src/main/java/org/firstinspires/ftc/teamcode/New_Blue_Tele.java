package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Core.Motor;
import org.firstinspires.ftc.teamcode.Modules.Indexer;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

@Disabled
@TeleOp(name="League Champs Blue Tele")
public class New_Blue_Tele extends Base {




    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timerOne = new ElapsedTime();
        ElapsedTime timerTwo = new ElapsedTime();
        ElapsedTime timerThree = new ElapsedTime();
        ElapsedTime kickTimer = new ElapsedTime();
        ElapsedTime kickerTimerTwo = new ElapsedTime();
        ElapsedTime kickTimerThree = new ElapsedTime();
        String[] order = {"green", "purple", "purple"};
        String[] order2 = {"purple", "green", "purple"};
        String[] order3 = {"purple", "purple", "green"};
        ElapsedTime loopTime = new ElapsedTime();
        double kv = 0.0006;

        double kp = 0.018;
        double ki = 0;
        double kd = 0;
        double targetVelocity = 1500;





        resetCache();


        double fLeftPow = 0, fRightPow = 0, bLeftPow = 0, bRightPow = 0;
        String slot1 = "", slot2 = "", slot3="";









        hoodOne = hardwareMap.servo.get("hoodOne");
        hoodTwo = hardwareMap.servo.get("hoodTwo");


        int index = 0;

        boolean triggerOneLast = false, triggerOneCurr = false, triggerTwoLast = false, triggerTwoCurr = false,
                triggerThreeLast = false, triggerThreeCurr = false, triggerFourCurr = false, triggerFourLast = false;
        double KICKER_ONE_DOWN = 0.042, KICKER_ONE_UP = 1, KICKER_TWO_DOWN = 0.98, KICKER_TWO_UP = 0.18,
                KICKER_THREE_DOWN = 0.065, KICKER_THREE_UP = 1;

        boolean toggleVeloLast = false, toggleVeloCurr = false;
        boolean minusVeloLast = false, minusVeloCurr = false;
        boolean autoAim = true;
        boolean turrUpLast = false, turrUpCurr = false, turrDownLast = false, turrDownCurr = false;
        boolean changeUpLast = false, changeUpCurr = false;
        boolean changeDownLast = false, changeDownCurr = false;
        boolean pushCurr = false, pushLast = false;
        boolean feederOff = false, wheelsOff = false;
        boolean shooterLast = false, shooterCurr = false;
        boolean shoot = false;
        boolean intake = false;
        boolean canPush = false;
        boolean oneShot = false, twoShot = false, threeShot = false;
        boolean oneKick = false, twoKick = false, threeKick = false;
        ElapsedTime pushBall = new ElapsedTime();
        ElapsedTime reset = new ElapsedTime();
        boolean canReset = false;
        boolean indexerFlipped = false;
        double powerCap = 1;
        boolean shooterOn = false;
        boolean shooterMode = false;
        double TPS = 0;
        boolean doRumble = true;
        double error = 0;
        boolean kickOne = false, kickTwo = false, kickThree = false;
        boolean bruhKick = false, bruhKickTwo = false, bruhKickThree = false;
        boolean resetLast = false, resetCurr = false;
        int counter = 0;

        double goalX = 60, goalY = 60;

        String shootOneSlot = "", shootTwoSlot = "", shootThreeSlot = "";

        int KICKER_WAIT_TIME = 330; // in milliseconds
        Servo[] kickers = new Servo[3];
        Map<Servo, Double> UP_POSITIONS = new HashMap<>();
        Map<Servo, Double> DOWN_POSITIONS = new HashMap<>();

        UP_POSITIONS.put(kickerOne, KICKER_ONE_UP);
        UP_POSITIONS.put(kickerTwo, KICKER_TWO_UP);
        UP_POSITIONS.put(kickerThree, KICKER_THREE_UP);

        DOWN_POSITIONS.put(kickerOne, KICKER_ONE_DOWN);
        DOWN_POSITIONS.put(kickerTwo, KICKER_TWO_DOWN);
        DOWN_POSITIONS.put(kickerThree, KICKER_THREE_DOWN);

        double targetHood = 0.6;


//        double power = 1;
        double shooterPow = 1;

//        kickerOne.setPosition(KICKER_ONE_DOWN);
//        kickerTwo.setPosition(KICKER_TWO_DOWN);
//        kickerThree.setPosition(KICKER_THREE_DOWN);

//        hoodOne.setPosition(0.5);
//        hoodTwo.setPosition(0.5);

//        rotOne.setPosition(0.75);
//        rotTwo.setPosition(0.75);
//        rotOne.setPosition(0.5);
//        rotTwo.setPosition(0.5);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //double degreeToServo = ((0.5 - 0.12) /45);
        double degreeToServo = ((0.5 - 0.14) /45);




        waitForStart();

        resetCache();

        rotOne.setPosition(0.5);
        rotTwo.setPosition(0.5);

        hoodOne.setPosition(0.7);
        hoodTwo.setPosition(0.3);

        kickerOne.setPosition(KICKER_ONE_DOWN);
        kickerTwo.setPosition(KICKER_TWO_DOWN);
        kickerThree.setPosition(KICKER_THREE_DOWN);

        odo.setPosition(new Pose2D(DistanceUnit.INCH, 15.1, 34.3, AngleUnit.DEGREES, 0));



        while(opModeIsActive()) {
            resetCache();
//
//            rotOne.setPosition(0.75);
//            rotTwo.setPosition(0.75);
            double addOn = counter * 0.035;
            double theta = Math.toDegrees(Math.atan2(goalX + getX(), goalY - getY())) - (getAngle());

            double turretPos =  0.59 + (theta*degreeToServo) - 0.052 - 0.02 + addOn;




            if(autoAim) {

                rotOne.setPosition(turretPos);
                rotTwo.setPosition(turretPos);
            }



            turrUpLast = turrUpCurr;
            turrUpCurr = gamepad2.dpad_right;
            if(turrUpCurr && !turrUpLast){
                autoAim = false;
                rotOne.setPosition(rotOne.getPosition() - 0.035);
                rotTwo.setPosition(rotTwo.getPosition() - 0.035);
                counter --;
            }

            turrDownLast = turrDownCurr;
            turrDownCurr = gamepad2.dpad_left;
            if(turrDownCurr && !turrDownLast){
                autoAim = false;
                rotOne.setPosition(rotOne.getPosition() + 0.035);
                rotTwo.setPosition(rotTwo.getPosition() + 0.035);
                counter++;
            }



            if(!autoAim && (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1) ){
                autoAim = true;
            }

            odo.update();//7.5 in vertical to strafe
            //1 in horizontal to strafe



            double drive = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            fLeftPow = drive + turn + strafe;
            fRightPow = drive - turn - strafe;
            bLeftPow = drive + turn - strafe;
            bRightPow = drive - turn + strafe;

//            fLeft.setPower(fLeftPow);
//            fRight.setPower(fRightPow);
//            bLeft.setPower(bLeftPow);
//            bRight.setPower(bRightPow);


            driveFieldCentric(drive, turn, strafe, powerCap);

            resetLast = resetCurr;
            resetCurr = gamepad2.left_stick_button;

            if(resetCurr && !resetLast){
                odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, odo.getHeading(AngleUnit.DEGREES)));
            }

            if (gamepad2.left_trigger > 0.05) {
                shooterMode = true;
                doRumble = true;
            }

            if (Math.abs(shooterOne.getVelocity()) - targetVelocity < 25 && doRumble) {
                gamepad2.rumble(500);
                doRumble = false;
            }
            if (gamepad2.right_trigger > 0.05) {
                shooterMode = false;

            }


            if (shooterMode) {
                TPS = shooterOne.getVelocity();

                error = Math.abs(targetVelocity) - Math.abs(shooterOne.getVelocity());


                double p = (0.021) * error;
                double feedforward = (0.0004) * targetVelocity;

                double pow = p + feedforward;

                double finalPow = Range.clip(pow, -1, 1);


                shooterOne.setPower(-finalPow);
                shooterTwo.setPower(finalPow);
            } else {
                shooterOne.setPower(0);
                shooterTwo.setPower(0);
            }

            if (gamepad1.right_bumper) {
                frontSweeper.setPower(-1);
                backSweeper.setPower(0);
            } else if (gamepad1.left_bumper) {
//                if(fLeft.retMotorEx().getPower() > 0.2 || fRight.retMotorEx().getPower() > 0.2){
//                    backSweeper.setPower(-0.8);
//                    frontSweeper.setPower(0);
//                }else if(fLeft.retMotorEx().getPower() < -0.2 || bLeft.retMotorEx().getPower() < -0.2){
//                    frontSweeper.setPower(-0.8);
//                    backSweeper.setPower(0);
//                }else{
//                    frontSweeper.setPower(-0.8);
//                    backSweeper.setPower(-0.8);
//                }
                backSweeper.setPower(-1);
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

            if (timerOne.milliseconds() > 165 && kickOne) {
                kickerThree.setPosition(KICKER_THREE_DOWN);
                kickOne = false;
                kickTimer.reset();
                bruhKick = true;
            }

            if (kickTimer.milliseconds() > 75  && bruhKick) {
                kickerOne.setPosition(KICKER_ONE_UP);
                kickTwo = true;
                bruhKick = false;
                timerTwo.reset();
            }

            if (timerTwo.milliseconds() > 195 && kickTwo) {
                kickerOne.setPosition(KICKER_ONE_DOWN);
                kickTwo = false;
                kickerTimerTwo.reset();
                bruhKickTwo = true;

            }

            if (kickerTimerTwo.milliseconds() > 75 && bruhKickTwo) {
                kickerTwo.setPosition(KICKER_TWO_UP);
                kickThree = true;
                bruhKickTwo = false;
                timerThree.reset();
            }

            if (timerThree.milliseconds() > 165 && kickThree) {
                kickerTwo.setPosition(KICKER_TWO_DOWN);
                kickThree = false;
                kickTimerThree.reset();
                bruhKickThree = true;
            }

//            triggerTwoLast = triggerTwoCurr;
//            triggerTwoCurr = gamepad2.b;
//
//            String one = oneSlot, two = twoSlot, three = threeSlot;
//
//
//            //Order - GPP
//
//            //back slot 1 front slot 2 middle slot 3
//
//            if (triggerOneCurr && !triggerOneLast) {
//
//                if(order[0].equals(one)){
//                    kickerOne.setPosition(KICKER_ONE_UP);
//                    oneShot = true;
//                    oneKick = true;
//                }else if(order[0].equals(two)){
//                    kickerTwo.setPosition(KICKER_TWO_UP);
//                    twoShot = true;
//                    twoKick = true;
//                }else{
//                    kickerThree.setPosition(KICKER_THREE_UP);
//                    threeShot = true;
//                    threeKick = true;
//                }
//
//
//                kickOne = true;
//                timerOne.reset();
//            }
//
//            if (timerOne.milliseconds() > 190 && kickOne) {
//
//                if(oneKick){
//                    kickerOne.setPosition(KICKER_ONE_DOWN);
//                    oneKick = false;
//                }else if(twoKick){
//                    kickerTwo.setPosition(KICKER_TWO_DOWN);
//                    twoKick = false;
//                }else{
//                    kickerThree.setPosition(KICKER_THREE_DOWN);
//                    threeKick = false;
//                }
//
//                kickOne = false;
//                kickTimer.reset();
//                bruhKick = true;
//            }
//
//            if (kickTimer.milliseconds() > 73  && bruhKick) {
//
//                if(order[1].equals(one) && !oneShot){
//                    kickerOne.setPosition(KICKER_ONE_UP);
//                    oneShot = true;
//                    oneKick = true;
//                }else if(order[1].equals(two) && !twoShot){
//                    kickerTwo.setPosition(KICKER_TWO_UP);
//                    twoShot = true;
//                    twoKick = true;
//                }else{
//                    kickerThree.setPosition(KICKER_THREE_UP);
//                    threeShot = true;
//                    threeKick = true;
//                }
//                kickTwo = true;
//                bruhKick = false;
//                timerTwo.reset();
//            }
//
//            if (timerTwo.milliseconds() > 190 && kickTwo) {
//                if(oneKick){
//                    kickerOne.setPosition(KICKER_ONE_DOWN);
//                    oneKick = false;
//                }else if(twoKick){
//                    kickerTwo.setPosition(KICKER_TWO_DOWN);
//                    twoKick = false;
//                }else{
//                    kickerThree.setPosition(KICKER_THREE_DOWN);
//                    threeKick = false;
//                }
//                kickTwo = false;
//                kickerTimerTwo.reset();
//                bruhKickTwo = true;
//
//            }
//
//            if (kickerTimerTwo.milliseconds() > 73 && bruhKickTwo) {
//                if(oneShot && twoShot){
//                    kickerThree.setPosition(KICKER_THREE_UP);
//                    threeKick = true;
//                }else if(oneShot && threeShot){
//                    kickerTwo.setPosition(KICKER_TWO_UP);
//                    twoKick = true;
//                }else{
//                    kickerOne.setPosition(KICKER_ONE_UP);
//                    oneKick = true;
//                }
//                kickThree = true;
//                bruhKickTwo = false;
//                timerThree.reset();
//            }
//
//            if (timerThree.milliseconds() > 190 && kickThree) {
//                if(oneKick){
//                    kickerOne.setPosition(KICKER_ONE_DOWN);
//                    oneKick = false;
//                }else if(twoKick){
//                    kickerTwo.setPosition(KICKER_TWO_DOWN);
//                    twoKick = false;
//                }else{
//                    kickerThree.setPosition(KICKER_THREE_DOWN);
//                    threeKick = false;
//                }
//                kickThree = false;
//                kickTimerThree.reset();
//                bruhKickThree = true;
//            }











//            triggerTwoLast = triggerTwoCurr;
//            triggerTwoCurr = gamepad2.b;   //Shoot this one last as it gets kind of stuck
//            if(triggerTwoCurr && !triggerTwoLast){
//                kickerTwo.setPosition(KICKER_TWO_UP);
//                kickTwo = true;
//                timerTwo.reset();
//            }
//
//            if(timerTwo.milliseconds() > KICKER_WAIT_TIME && kickTwo){
//                kickerTwo.setPosition(KICKER_TWO_DOWN);
//                kickTwo = false;
//
//            }
//
//
//            triggerThreeLast = triggerThreeCurr;
//            triggerThreeCurr = gamepad2.x;
//            if(triggerThreeCurr && !triggerThreeLast){
//                kickerThree.setPosition(KICKER_THREE_UP);
//                kickThree = true;
//                timerThree.reset();
//            }
//
//            if(timerThree.milliseconds() > 290 && kickThree){
//                kickerThree.setPosition(KICKER_THREE_DOWN);
//                kickThree = false;
//            }


            toggleVeloLast = toggleVeloCurr;
            toggleVeloCurr = gamepad2.dpad_up;
            if(toggleVeloCurr && !toggleVeloLast){
                targetVelocity += 25;
            }

            minusVeloLast = minusVeloCurr;
            minusVeloCurr = gamepad2.dpad_down;
            if(minusVeloCurr && !minusVeloLast){
                targetVelocity -= 25;
            }

//            double turrDrive = gamepad2.left_stick_x;
//
//            rotOne.setPower(-turrDrive * 0.7);
//            rotTwo.setPower(-turrDrive * 0.7);


            if(gamepad2.right_bumper){
                hoodOne.setPosition(hoodOne.getPosition() + 0.004);
                hoodTwo.setPosition(hoodTwo.getPosition() - 0.004);
            }

            if(gamepad2.left_bumper){
                hoodOne.setPosition(hoodOne.getPosition() - 0.004);
                hoodTwo.setPosition(hoodTwo.getPosition() + 0.004);
//                targetVelocity = 1350;

            }












//            Pose2D pos = odo.getPosition();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));

            if(slotIntake.getDistance(DistanceUnit.CM) > 3 && finalColor.getDistance(DistanceUnit.CM) >3){
                twoSlot = "empty";
            }else{
                if(slotIntake.getDistance(DistanceUnit.CM) < finalColor.getDistance(DistanceUnit.CM)){
                    if(slotIntake.green() > 2080){
                        twoSlot = "green";
                    }else{
                        twoSlot = "purple";
                    }
                }else{
                    if(finalColor.green() > 340){
                        twoSlot = "green";
                    }else{
                        twoSlot = "purple";
                    }
                }
            }

            if(slotOut.getDistance(DistanceUnit.CM) > 3 && lastColor.getDistance(DistanceUnit.CM) >3){
                oneSlot = "empty";
            }else{
                if(slotOut.getDistance(DistanceUnit.CM) < lastColor.getDistance(DistanceUnit.CM)){
                    if(slotOut.green() > 1050){
                        oneSlot = "green";
                    }else{
                        oneSlot = "purple";
                    }
                }else{
                    if(lastColor.green() > 2500){
                        oneSlot = "green";
                    }else{
                        oneSlot = "purple";
                    }
                }
            }

            if(oneSlot.equals("purple") && twoSlot.equals("purple")){
                threeSlot = "green";
            }else{
                threeSlot = "purple";
            }

            if(oneSlot.equals("empty") || twoSlot.equals("empty")){
                oneSlot = ("purple");
                twoSlot = ("green");
            }






            Pose2D pos = odo.getPosition();

            double xTrans = pos.getX(DistanceUnit.INCH) + 60;
            double yTrans = pos.getY(DistanceUnit.INCH) - 60;


            double total = Math.pow(Math.abs(xTrans), 2) + Math.pow(Math.abs(yTrans), 2);
            double totalDist = Math.sqrt(total);
            ;
            targetVelocity = ((0.000827923)*Math.pow(totalDist, 3)) - ((0.137619)*Math.pow(totalDist, 2)) + ((12.04007)*totalDist) + 988.36408; //Cubic Regression for Shooter Power
            if(targetVelocity < 1420){
                targetVelocity -= 45;
            }
            targetHood =  ((-0.0000382087)*Math.pow(totalDist, 2)) + ((0.005844054)*totalDist) + 0.674147; //Cubic Regression for Hood Position
            if(targetHood > .87){
                targetHood = .92;
            }

            hoodOne.setPosition(targetHood);
            hoodTwo.setPosition(1 - targetHood);
            telemetry.addData("F Left Pow", fLeft.retMotorEx().getPower());
            telemetry.addData("X", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Trans X", xTrans);
            telemetry.addData("Trans Y", yTrans);
            telemetry.addData("Distance", totalDist);
            telemetry.addData("Heading", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Turret", rotOne.getPosition());
            telemetry.addData("Turret 2", rotTwo.getPosition());

            telemetry.addData("Slot 1", oneSlot);
            telemetry.addData("Slot 2", twoSlot);

            telemetry.addData("Conv", theta*degreeToServo);
            telemetry.addData("Degree To Servo", degreeToServo);
            telemetry.addData("Theta", theta);


//            telemetry.addData("Shooter Velocity", shooterOne.retMotorEx().getVelocity());
//            telemetry.addData("Shooter 2 Velocity", shooterTwo.retMotorEx().getVelocity());
//            telemetry.addData("Shooter 1 Current", shooterOne.retMotorEx().getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("Shooter 2 Current", shooterTwo.retMotorEx().getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("Shooter 1 Power", shooterOne.retMotorEx().getPower());
//            telemetry.addData("Shooter 2 Power", shooterTwo.retMotorEx().getPower());

            telemetry.addData("Set Speed", targetVelocity);

            telemetry.addData("Green Slot 1", slotIntake.green());
            telemetry.addData("Green Slot 2", slotOut.green());
            telemetry.addData("Green Slot 3", finalColor.green());
            telemetry.addData("Green Slot 4", lastColor.green());

            telemetry.addData("Distance 1", slotIntake.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance 2", slotOut.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance 3", finalColor.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance 4", lastColor.getDistance(DistanceUnit.CM));

            telemetry.addData("Hood 1", hoodOne.getPosition());
            telemetry.addData("Hood 2", hoodTwo.getPosition());

            telemetry.addData("First", order[0]);
            telemetry.addData("Second", order[1]);
            telemetry.addData("Third", order[2]);


            telemetry.update();









        }





    }
}

//
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorRangeSensor;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.Core.Motor;
//import org.firstinspires.ftc.teamcode.Modules.Indexer;
//
//import java.util.HashMap;
//import java.util.Locale;
//import java.util.Map;
//
//
//@TeleOp(name="Test #1")
//public class BasicDriveTest extends Base {
//
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initHardware(hardwareMap);
//        ElapsedTime timer = new ElapsedTime();
//        ElapsedTime timerOne = new ElapsedTime();
//        ElapsedTime timerTwo = new ElapsedTime();
//        ElapsedTime timerThree = new ElapsedTime();
//        ElapsedTime kickTimer = new ElapsedTime();
//        ElapsedTime kickerTimerTwo = new ElapsedTime();
//        ElapsedTime kickTimerThree = new ElapsedTime();
//        String[] order = {"green", "purple", "purple"};
//        String[] order2 = {"purple", "green", "purple"};
//        String[] order3 = {"purple", "purple", "green"};
//        ElapsedTime loopTime = new ElapsedTime();
//        double kv = 0.00057;
//
//        double kp = 0.018;
//        double ki = 0;
//        double kd = 0;
//        double targetVelocity = 1350;
//        resetCache();
//
//
//        double fLeftPow = 0, fRightPow = 0, bLeftPow = 0, bRightPow = 0;
//        String slot1 = "", slot2 = "", slot3="";
//
//
//
//
//        kickerOne = hardwareMap.servo.get("kickIntake");
//        kickerTwo = hardwareMap.servo.get("kickOut");
//        kickerThree = hardwareMap.servo.get("kickMiddle");
//
//
//        RevColorSensorV3 slotIntake = hardwareMap.get(RevColorSensorV3.class, "colorIn");
//        RevColorSensorV3 slotOut = hardwareMap.get(RevColorSensorV3.class, "colorOut");
//        RevColorSensorV3 finalColor = hardwareMap.get(RevColorSensorV3.class, "colorLast");
//
//
//        hoodOne = hardwareMap.servo.get("hoodOne");
//        hoodTwo = hardwareMap.servo.get("hoodTwo");
//
//
//        int index = 0;
//
//        boolean triggerOneLast = false, triggerOneCurr = false, triggerTwoLast = false, triggerTwoCurr = false,
//                triggerThreeLast = false, triggerThreeCurr = false;
//        double KICKER_ONE_DOWN = 0.15, KICKER_ONE_UP = 0.634, KICKER_TWO_DOWN = 0.6, KICKER_TWO_UP = 0.3,
//                KICKER_THREE_DOWN = 0.638, KICKER_THREE_UP = 0.91;
//
//        boolean toggleVeloLast = false, toggleVeloCurr = false;
//        boolean minusVeloLast = false, minusVeloCurr = false;
//        boolean changeUpLast = false, changeUpCurr = false;
//        boolean changeDownLast = false, changeDownCurr = false;
//        boolean pushCurr = false, pushLast = false;
//        boolean feederOff = false, wheelsOff = false;
//        boolean shooterLast = false, shooterCurr = false;
//        boolean shoot = false;
//        boolean intake = false;
//        boolean canPush = false;
//        ElapsedTime pushBall = new ElapsedTime();
//        ElapsedTime reset = new ElapsedTime();
//        boolean canReset = false;
//        boolean indexerFlipped = false;
//        double powerCap = 1;
//        boolean shooterOn = false;
//        boolean shooterMode = false;
//        double TPS = 0;
//        boolean doRumble = true;
//        double error = 0;
//        boolean kickOne = false, kickTwo = false, kickThree = false;
//        boolean bruhKick = false, bruhKickTwo = false, bruhKickThree = false;
//        String oneSlot = "", twoSlot = "", threeSlot ="";
//        String shootOneSlot = "", shootTwoSlot = "", shootThreeSlot = "";
//
//        int KICKER_WAIT_TIME = 330; // in milliseconds
//        Servo[] kickers = new Servo[3];
//        Map<Servo, Double> UP_POSITIONS = new HashMap<>();
//        Map<Servo, Double> DOWN_POSITIONS = new HashMap<>();
//
//        UP_POSITIONS.put(kickerOne, KICKER_ONE_UP);
//        UP_POSITIONS.put(kickerTwo, KICKER_TWO_UP);
//        UP_POSITIONS.put(kickerThree, KICKER_THREE_UP);
//
//        DOWN_POSITIONS.put(kickerOne, KICKER_ONE_DOWN);
//        DOWN_POSITIONS.put(kickerTwo, KICKER_TWO_DOWN);
//        DOWN_POSITIONS.put(kickerThree, KICKER_THREE_DOWN);
//
//
//
////        double power = 1;
//        double shooterPow = 1;
//
//        kickerOne.setPosition(KICKER_ONE_DOWN);
//        kickerTwo.setPosition(KICKER_TWO_DOWN);
//        kickerThree.setPosition(KICKER_THREE_DOWN);
//
//        hoodOne.setPosition(0.5);
//        hoodTwo.setPosition(0.5);
//
//        rotOne.setPosition(0.75);
//        rotTwo.setPosition(0.75);
//
//        odo.setHeading(179, AngleUnit.DEGREES);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//
//
//        waitForStart();
//
//
//        while(opModeIsActive()) {
//
//            resetCache();
//
//            odo.update();//7.5 in vertical to strafe
//            //1 in horizontal to strafe
//
//            double drive = gamepad1.left_stick_y;
//            double turn = -gamepad1.right_stick_x;
//            double strafe = -gamepad1.left_stick_x;
//
//            fLeftPow = drive + turn + strafe;
//            fRightPow = drive - turn - strafe;
//            bLeftPow = drive + turn - strafe;
//            bRightPow = drive - turn + strafe;
//
////            fLeft.setPower(fLeftPow);
////            fRight.setPower(fRightPow);
////            bLeft.setPower(bLeftPow);
////            bRight.setPower(bRightPow);
//
//
//            driveFieldCentric(drive, turn, strafe, powerCap);
//
//            if (gamepad2.left_trigger > 0.05) {
//                shooterMode = true;
//                doRumble = true;
//            }
//
//            if (Math.abs(shooterOne.retMotorEx().getVelocity()) - targetVelocity < 25 && doRumble) {
//                gamepad2.rumble(500);
//                doRumble = false;
//            }
//            if (gamepad2.right_trigger > 0.05) {
//                shooterMode = false;
//
//            }
//
//
//            if (shooterMode) {
//                TPS = shooterOne.retMotorEx().getVelocity();
//
//                error = targetVelocity - shooterOne.retMotorEx().getVelocity();
//
//                double p = kp * error;
//                double feedforward = kv * targetVelocity;
//
//                double pow = p + feedforward;
//
//                double finalPow = Range.clip(pow, -1, 1);
//
//
//                shooterOne.setPower(finalPow);
//                shooterTwo.setPower(-finalPow);
//            } else {
//                shooterOne.setPower(0);
//                shooterTwo.setPower(0);
//            }
//
//            if (gamepad1.left_bumper) {
//                frontSweeper.setPower(0.95);
//                backSweeper.setPower(0.95);
//            } else if (gamepad1.right_bumper) {
//                frontSweeper.setPower(-0.95);
//                backSweeper.setPower(-0.95);
//            } else {
//                frontSweeper.setPower(0);
//                backSweeper.setPower(0);
//
//            }
//
//
//            triggerOneLast = triggerOneCurr;
//            triggerOneCurr = gamepad2.a;
//
//
//            //Order - GPP
//
//            //back slot 1 front slot 2 middle slot 3
//
//            if (triggerOneCurr && !triggerOneLast) {
//
//
//                kickerThree.setPosition(KICKER_THREE_UP);
//                kickOne = true;
//                timerOne.reset();
//
//
//
//                if (timerOne.milliseconds() > 290 && kickOne) {
//                    kickerThree.setPosition(KICKER_THREE_DOWN);
//                    kickOne = false;
//                    kickTimer.reset();
//                    bruhKick = true;
//                }
//
//                if (kickTimer.milliseconds() > 200 && bruhKick) {
//                    kickerOne.setPosition(KICKER_ONE_UP);
//                    kickTwo = true;
//                    bruhKick = false;
//                    timerTwo.reset();
//                }
//
//                if (timerTwo.milliseconds() > 290 && kickTwo) {
//                    kickerOne.setPosition(KICKER_ONE_DOWN);
//                    kickTwo = false;
//                    kickerTimerTwo.reset();
//                    bruhKickTwo = true;
//
//                }
//
//                if (kickerTimerTwo.milliseconds() > 200 && bruhKickTwo) {
//                    kickerTwo.setPosition(KICKER_TWO_UP);
//                    kickThree = true;
//                    bruhKickTwo = false;
//                    timerThree.reset();
//                }
//
//                if (timerThree.milliseconds() > 290 && kickThree) {
//                    kickerTwo.setPosition(KICKER_TWO_DOWN);
//                    kickThree = false;
//                    kickTimerThree.reset();
//                    bruhKickThree = true;
//                }
//            }
//
//
//
//
//
//
////            triggerTwoLast = triggerTwoCurr;
////            triggerTwoCurr = gamepad2.b;   //Shoot this one last as it gets kind of stuck
////            if(triggerTwoCurr && !triggerTwoLast){
////                kickerTwo.setPosition(KICKER_TWO_UP);
////                kickTwo = true;
////                timerTwo.reset();
////            }
////
////            if(timerTwo.milliseconds() > KICKER_WAIT_TIME && kickTwo){
////                kickerTwo.setPosition(KICKER_TWO_DOWN);
////                kickTwo = false;
////
////            }
////
////
////            triggerThreeLast = triggerThreeCurr;
////            triggerThreeCurr = gamepad2.x;
////            if(triggerThreeCurr && !triggerThreeLast){
////                kickerThree.setPosition(KICKER_THREE_UP);
////                kickThree = true;
////                timerThree.reset();
////            }
////
////            if(timerThree.milliseconds() > 290 && kickThree){
////                kickerThree.setPosition(KICKER_THREE_DOWN);
////                kickThree = false;
////            }
//
//
//            toggleVeloLast = toggleVeloCurr;
//            toggleVeloCurr = gamepad2.dpad_up;
//            if(toggleVeloCurr && !toggleVeloLast){
//                targetVelocity += 25;
//            }
//
//            minusVeloLast = minusVeloCurr;
//            minusVeloCurr = gamepad2.dpad_down;
//            if(minusVeloCurr && !minusVeloLast){
//                targetVelocity -= 25;
//            }
//
////            double turrDrive = gamepad2.left_stick_x;
////
////            rotOne.setPower(-turrDrive * 0.7);
////            rotTwo.setPower(-turrDrive * 0.7);
//
//
//            if(gamepad2.right_bumper){
//                hoodOne.setPosition(0.25);
//                hoodTwo.setPosition(0.75);
//                targetVelocity = 1150;
//            }
//
//            if(gamepad2.left_bumper){
//                hoodOne.setPosition(0.4);
//                hoodTwo.setPosition(0.6);
//                targetVelocity = 1350;
//
//            }
//
//
//
//
//
//
//
//
////            Pose2D pos = odo.getPosition();
////            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
//            if(slotOut.getDistance(DistanceUnit.CM) > 2.5){
//                twoSlot = "empty";
//            }else{
//                if(slotOut.green() > 4000){
//                    twoSlot = "green";
//                }else{
//                    twoSlot = "purple";
//                }
//            }
//
//            if(slotIntake.getDistance(DistanceUnit.CM) > 2.5){
//                oneSlot = "empty";
//            }else{
//                if(slotIntake.green() > 1500){
//                    oneSlot = "green";
//                }else{
//                    oneSlot = "purple";
//                }
//            }
//
//            if(finalColor.getDistance(DistanceUnit.CM) > 2.5){
//                threeSlot = "empty";
//
//            }else{
//                if(finalColor.green() > 550){
//                    threeSlot = "green";
//                }else{
//                    threeSlot = "purple";
//                }
//            }
//
//
//
//
//            Pose2D pos = odo.getPosition();
//
//            telemetry.addData("X", pos.getX(DistanceUnit.INCH));
//            telemetry.addData("Y", pos.getY(DistanceUnit.INCH));
//            telemetry.addData("Heading", pos.getHeading(AngleUnit.DEGREES));
//
//            telemetry.addData("Slot 1", oneSlot);
//            telemetry.addData("Slot 2", twoSlot);
//            telemetry.addData("Slot 3", threeSlot);
//
//            telemetry.addData("Shooter Velocity", shooterOne.retMotorEx().getVelocity());
//            telemetry.addData("Shooter 2 Velocity", shooterTwo.retMotorEx().getVelocity());
//            telemetry.addData("Shooter 1 Current", shooterOne.retMotorEx().getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("Shooter 2 Current", shooterTwo.retMotorEx().getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("Shooter 1 Power", shooterOne.retMotorEx().getPower());
//            telemetry.addData("Shooter 2 Power", shooterTwo.retMotorEx().getPower());
//
//            telemetry.addData("Set Speed", targetVelocity);
//            telemetry.addData("Green Slot 1", slotIntake.green());
//            telemetry.addData("Green Slot 2", slotOut.green());
//            telemetry.addData("Green Slot 3", finalColor.green());
//            telemetry.addData("Distance 1", slotIntake.getDistance(DistanceUnit.CM));
//            telemetry.addData("Distance 2", slotOut.getDistance(DistanceUnit.CM));
//            telemetry.addData("Distance 3", finalColor.getDistance(DistanceUnit.CM));
//
//            telemetry.addData("Hood 1", hoodOne.getPosition());
//            telemetry.addData("Hood 2", hoodTwo.getPosition());
//
//
//            telemetry.update();
//
//
//
//
//
//
//
//
//
//        }
//
//
//
//
//
//    }
//}


