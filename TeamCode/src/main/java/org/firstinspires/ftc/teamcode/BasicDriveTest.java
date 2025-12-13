package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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


@TeleOp(name="Test #1")
public class BasicDriveTest extends Base {




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
            FtcDashboard dashboard;
           double kv = 0.00057;

            double kp = 0.018;
            double ki = 0;
            double kd = 0;
            double targetVelocity = 1350;
           resetCache();


           double fLeftPow = 0, fRightPow = 0, bLeftPow = 0, bRightPow = 0;
           String slot1 = "", slot2 = "", slot3="";




        kickerOne = hardwareMap.servo.get("kickIntake");
        kickerTwo = hardwareMap.servo.get("kickOut");
        kickerThree = hardwareMap.servo.get("kickMiddle");


        RevColorSensorV3 slotIntake = hardwareMap.get(RevColorSensorV3.class, "colorIn");
        RevColorSensorV3 slotOut = hardwareMap.get(RevColorSensorV3.class, "colorOut");
        RevColorSensorV3 finalColor = hardwareMap.get(RevColorSensorV3.class, "colorLast");


        hoodOne = hardwareMap.servo.get("hoodOne");
        hoodTwo = hardwareMap.servo.get("hoodTwo");


        int index = 0;

        boolean triggerOneLast = false, triggerOneCurr = false, triggerTwoLast = false, triggerTwoCurr = false,
                triggerThreeLast = false, triggerThreeCurr = false;
        double KICKER_ONE_DOWN = 0.15, KICKER_ONE_UP = 0.634, KICKER_TWO_DOWN = 0.6, KICKER_TWO_UP = 0.3,
                KICKER_THREE_DOWN = 0.638, KICKER_THREE_UP = 0.91;

        boolean toggleVeloLast = false, toggleVeloCurr = false;
        boolean minusVeloLast = false, minusVeloCurr = false;
        boolean changeUpLast = false, changeUpCurr = false;
        boolean changeDownLast = false, changeDownCurr = false;
        boolean pushCurr = false, pushLast = false;
        boolean feederOff = false, wheelsOff = false;
        boolean shooterLast = false, shooterCurr = false;
        boolean shoot = false;
        boolean intake = false;
        boolean canPush = false;
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
        String oneSlot = "", twoSlot = "", threeSlot ="";
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



//        double power = 1;
        double shooterPow = 1;

        kickerOne.setPosition(KICKER_ONE_DOWN);
        kickerTwo.setPosition(KICKER_TWO_DOWN);
        kickerThree.setPosition(KICKER_THREE_DOWN);

        hoodOne.setPosition(0.5);
        hoodTwo.setPosition(0.5);

        odo.setHeading(179, AngleUnit.DEGREES);

        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();


        while(opModeIsActive()) {

            resetCache();

            odo.update();//7.5 in vertical to strafe
            //1 in horizontal to strafe

            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double strafe = -gamepad1.left_stick_x;

            fLeftPow = drive + turn + strafe;
            fRightPow = drive - turn - strafe;
            bLeftPow = drive + turn - strafe;
            bRightPow = drive - turn + strafe;

//            fLeft.setPower(fLeftPow);
//            fRight.setPower(fRightPow);
//            bLeft.setPower(bLeftPow);
//            bRight.setPower(bRightPow);


            driveFieldCentric(drive, turn, strafe, powerCap);

            if (gamepad2.left_trigger > 0.05) {
                shooterMode = true;
                doRumble = true;
            }

            if (Math.abs(shooterOne.retMotorEx().getVelocity()) - targetVelocity < 25 && doRumble) {
                gamepad2.rumble(500);
                doRumble = false;
            }
            if (gamepad2.right_trigger > 0.05) {
                shooterMode = false;

            }


            if (shooterMode) {
                TPS = shooterOne.retMotorEx().getVelocity();

                error = targetVelocity - shooterOne.retMotorEx().getVelocity();

                double p = kp * error;
                double feedforward = kv * targetVelocity;

                double pow = p + feedforward;

                double finalPow = Range.clip(pow, -1, 1);


                shooterOne.setPower(finalPow);
                shooterTwo.setPower(-finalPow);
            } else {
                shooterOne.setPower(0);
                shooterTwo.setPower(0);
            }

            if (gamepad1.left_bumper) {
                frontSweeper.setPower(0.95);
                backSweeper.setPower(0.95);
            } else if (gamepad1.right_bumper) {
                frontSweeper.setPower(-0.95);
                backSweeper.setPower(-0.95);
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



            if (timerOne.milliseconds() > 290 && kickOne) {
                kickerThree.setPosition(KICKER_THREE_DOWN);
                kickOne = false;
                kickTimer.reset();
                bruhKick = true;
            }

            if (kickTimer.milliseconds() > 200 && bruhKick) {
                kickerOne.setPosition(KICKER_ONE_UP);
                kickTwo = true;
                bruhKick = false;
                timerTwo.reset();
            }

            if (timerTwo.milliseconds() > 290 && kickTwo) {
                kickerOne.setPosition(KICKER_ONE_DOWN);
                kickTwo = false;
                kickerTimerTwo.reset();
                bruhKickTwo = true;

            }

            if (kickerTimerTwo.milliseconds() > 200 && bruhKickTwo) {
                kickerTwo.setPosition(KICKER_TWO_UP);
                kickThree = true;
                bruhKickTwo = false;
                timerThree.reset();
            }

            if (timerThree.milliseconds() > 290 && kickThree) {
                kickers[2].setPosition(KICKER_TWO_DOWN);
                kickThree = false;
                kickTimerThree.reset();
                bruhKickThree = true;
            }
        }






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

            double turrDrive = gamepad2.left_stick_x;
            rotOne.setPower(-turrDrive * 0.7);
            rotTwo.setPower(-turrDrive * 0.7);


            if(gamepad2.right_bumper){
                hoodOne.setPosition(0.25);
                hoodTwo.setPosition(0.75);
                targetVelocity = 1150;
            }

            if(gamepad2.left_bumper){
                hoodOne.setPosition(0.4);
                hoodTwo.setPosition(0.6);
                targetVelocity = 1350;

            }








//            Pose2D pos = odo.getPosition();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            if(slotOut.getDistance(DistanceUnit.CM) > 2.5){
                twoSlot = "empty";
            }else{
                if(slotOut.green() > 4000){
                    twoSlot = "green";
                }else{
                    twoSlot = "purple";
                }
            }

            if(slotIntake.getDistance(DistanceUnit.CM) > 2.5){
                oneSlot = "empty";
            }else{
                if(slotIntake.green() > 1500){
                    oneSlot = "green";
                }else{
                    oneSlot = "purple";
                }
            }

            if(finalColor.getDistance(DistanceUnit.CM) > 2.5){
                threeSlot = "empty";

            }else{
                if(finalColor.green() > 550){
                    threeSlot = "green";
                }else{
                    threeSlot = "purple";
                }
            }




            Pose2D pos = odo.getPosition();

            telemetry.addData("X", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", pos.getHeading(AngleUnit.DEGREES));

            telemetry.addData("Slot 1", oneSlot);
            telemetry.addData("Slot 2", twoSlot);
            telemetry.addData("Slot 3", threeSlot);

            telemetry.addData("Shooter Velocity", shooterOne.retMotorEx().getVelocity());
            telemetry.addData("Shooter 2 Velocity", shooterTwo.retMotorEx().getVelocity());
            telemetry.addData("Shooter 1 Current", shooterOne.retMotorEx().getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Shooter 2 Current", shooterTwo.retMotorEx().getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Shooter 1 Power", shooterOne.retMotorEx().getPower());
            telemetry.addData("Shooter 2 Power", shooterTwo.retMotorEx().getPower());

            telemetry.addData("Set Speed", targetVelocity);
            telemetry.addData("Green Slot 1", slotIntake.green());
            telemetry.addData("Green Slot 2", slotOut.green());
            telemetry.addData("Green Slot 3", finalColor.green());
            telemetry.addData("Distance 1", slotIntake.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance 2", slotOut.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance 3", finalColor.getDistance(DistanceUnit.CM));

            telemetry.addData("Hood 1", hoodOne.getPosition());
            telemetry.addData("Hood 2", hoodTwo.getPosition());


            telemetry.update();









        }





    }
}

