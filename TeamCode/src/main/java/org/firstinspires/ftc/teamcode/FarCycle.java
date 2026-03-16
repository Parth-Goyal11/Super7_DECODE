package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Core.Motor;
import org.firstinspires.ftc.teamcode.Core.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

@Autonomous(name="Far Cycle")
public class FarCycle extends Base{

    ArrayList<Point> shot1, adjustShot, midIntake, sweepMid, firstIntake, gate, secondIntake, thirdIntake, park, bang1, bang2, shotBruh, sweepGate, moveGate, sweepGateTwo, moveGateTwo, intakeOne, finalIntake, intakeOneCheck, finalPark;
    int condition = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        boolean kickThree = true;
        boolean loop2 = false, loop3 = false, loop4 = false, loop5 = false, loop6 = false,
                loop7 = false, loop8 = false, loop9 = false, loop10 = false, loop11 = false;
        boolean oneShot = false, twoShot = false, threeShot = false;
        boolean oneKick = false, twoKick = false, threeKick = false;
        initHardware(hardwareMap);

        ElapsedTime timerOne =  new ElapsedTime();
        ElapsedTime checkCurrent = new ElapsedTime();


        hoodOne = hardwareMap.servo.get("hoodOne");




        RevColorSensorV3 slotIntake = hardwareMap.get(RevColorSensorV3.class, "colorIn");
        RevColorSensorV3 slotOut = hardwareMap.get(RevColorSensorV3.class, "colorOut");
        RevColorSensorV3 finalColor = hardwareMap.get(RevColorSensorV3.class, "colorLast");
        RevColorSensorV3 otherSlot = hardwareMap.get(RevColorSensorV3.class, "bruhColor");

        Servo[] kickers = new Servo[3];

        String[] order = new String[3];





        double KICKER_ONE_DOWN = 0.95, KICKER_ONE_UP = 0.6, KICKER_TWO_DOWN = 0.985, KICKER_TWO_UP = 0.7,
                KICKER_THREE_DOWN = 0.06, KICKER_THREE_UP = 1;

        kickerOne.setPosition(KICKER_ONE_DOWN);
        kickerTwo.setPosition(KICKER_TWO_DOWN);
        kickerThree.setPosition(KICKER_THREE_DOWN);

        hoodOne.setPosition(0.555);

        rotOne.setPosition(0.517);
        rotTwo.setPosition(0.517);



        Map<Servo, Double> UP_POSITIONS = new HashMap<>();
        Map<Servo, Double> DOWN_POSITIONS = new HashMap<>();

        UP_POSITIONS.put(kickerOne, KICKER_ONE_UP);
        UP_POSITIONS.put(kickerTwo, KICKER_TWO_UP);
        UP_POSITIONS.put(kickerThree, KICKER_THREE_UP);

        DOWN_POSITIONS.put(kickerOne, KICKER_ONE_DOWN);
        DOWN_POSITIONS.put(kickerTwo, KICKER_TWO_DOWN);
        DOWN_POSITIONS.put(kickerThree, KICKER_THREE_DOWN);

//        kickerOne.setPosition(KICKER_ONE_DOWN);
//        kickerTwo.setPosition(KICKER_TWO_DOWN);
//        kickerThree.setPosition(KICKER_THREE_DOWN);
//        rotOne.setPosition(0.165);
//        rotTwo.setPosition(0.165);

        shot1 = new ArrayList<>();
        shot1.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(35, 0, 0, 1)
                        )
                )
        );

        adjustShot = new ArrayList<>();
        adjustShot.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(49, 0, 0, 1)
                        )
                )
        );

        intakeOne = new ArrayList<>();
        intakeOne.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-16, 8, 0, 1)

                        )
                )
        );

        intakeOneCheck = new ArrayList<>();
        intakeOneCheck.addAll(
                new ArrayList<>(
                        Arrays.asList(

                                new Point(-44, 8, 0, 0.4)
                        )
                )
        );



        midIntake = new ArrayList<>();
        midIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-10, 29, 0, 1)

                        )
                )
        );

        sweepMid = new ArrayList<>();
        sweepMid.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-53, 29, 0, 0.4)
                        )
                )
        );

        sweepGate = new ArrayList<>();
        sweepGate.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-52, 26, 0, 0.65)
                        )
                )
        );

        moveGate = new ArrayList<>();
        moveGate.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-49.8, 32.5 , 0, 0.9)
                        )
                )
        );

        sweepGateTwo = new ArrayList<>();
        sweepGateTwo.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-52, 26, 0, 0.65)
                        )
                )
        );

        moveGateTwo = new ArrayList<>();
        moveGateTwo.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-47.8, 33.5, 0, 0.9)
                        )
                )
        );




        gate = new ArrayList<>();
        gate.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-33, 15, 0, 0.9),
                                new Point(-49.5, 20, 0, 0.9)
                        )
                )
        );

        finalIntake = new ArrayList<>();
        finalIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-10, 52, 0, 0.9),
                                new Point(-49, 52, 0, 0.4)
                        )
                )
        );




        shotBruh = new ArrayList<>();
        shotBruh.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-5, 0, 0, 0.6)
                        )
                )
        );

        park = new ArrayList<>();
        park.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-31, -16, 0, 0.8)
                        )
                )
        );
        firstIntake = new ArrayList<>();
        firstIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-20, -45.5, 0, 1),
                                new Point(15.5, -45.5, 0, 0.3)
                        )
                )
        );

        bang1 = new ArrayList<>();
        bang1.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(27, -70, 0, 0.7)
                        )
                )
        );

        bang2 = new ArrayList<>();
        bang2.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(25, -94, 0, 0.7)
                        )
                )
        );





        secondIntake = new ArrayList<>();
        secondIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-20, -68, 0, 1),
                                new Point(31, -68, 0, 0.3)
                        )
                )
        );



        thirdIntake = new ArrayList<>();
        thirdIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-17.5, -92.5, 0, 1),
                                new Point(16, -92.5, 0, 0.3)
                        )
                )
        );

        finalPark = new ArrayList<>();
        finalPark.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(25, 0, 0, 0.6)
                        )
                )
        );


        while(opModeInInit()){
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                resetCache();
                if (result.isValid()) {

                    int tagIDR = result.getFiducialResults().get(0).getFiducialId();
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("ID", tagIDR);
                    telemetry.update();
                    counter = tagIDR;

                }else{


                    telemetry.addData("So", "Cooked");

                    telemetry.update();
                }
            }else{
                telemetry.addData("Result", "Bruh");
                telemetry.update();
            }
        }















        waitForStart();





//        ChaseTheCarrotConstantHeadingWithShooter(adjustShot, 9, 3, -42, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1);
//        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        timerOne.reset();
        while(timerOne.milliseconds() < 1275 + 1000){

            resetCache();

            runShooter(1800);
            if(kickThree && timerOne.milliseconds() > 1000){

                kickerThree.setPosition(KICKER_THREE_UP);

                kickThree = false;
                loop2 = true;
            }

            if(timerOne.milliseconds() > 250+1000 && loop2){

                kickerThree.setPosition(KICKER_THREE_DOWN);
                loop2 = false;
                loop3 = true;
            }

            if(timerOne.milliseconds() > 430+1000 && loop3){
                kickerOne.setPosition(KICKER_ONE_UP);
                loop3 = false;
                loop4 = true;
            }

            if(timerOne.milliseconds() > 680 +1000&& loop4){
                kickerOne.setPosition(KICKER_ONE_DOWN);
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 860+1000 && loop5){
                kickerTwo.setPosition(KICKER_TWO_UP);
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 1150 +1000&& loop6){
                kickerTwo.setPosition(KICKER_TWO_DOWN);
                loop6 = false;
            }
        }
        kickThree = true;
        backSweeper.setPower(1);
        ChaseTheCarrotConstantHeadingWithShooter(finalPark, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1);

//        hoodOne.setPosition(0.57 );
//
//        shooterOne.setPower(0);
//        shooterTwo.setPower(0);
//        rotOne.setPosition(0.53);
//        rotTwo.setPosition(0.53);
//        backSweeper.setPower(1);
//
//        ChaseTheCarrotConstantHeadingWithLimeLight(intakeOne, 9, 3, 0, 3, 1, 0.04, 0.04, 0.03, 0.0005, 0, 2000);
//
//        ChaseTheCarrotConstantHeadingWithShooter(intakeOneCheck, 9, 3, 0, 3, 1, 0.04, 0.04, 0.03, 0.0005, 0, 2500, 1);
//        sleep(450);
//        if(counter == 21){
//            order = new String[]{"green", "purple", "purple"};
//            condition = 1;
//        }else if(counter == 22){
//            order = new String[]{"purple", "green", "purple"};
//            condition = 2;
//        }else{
//            order = new String[]{"purple", "purple", "green"};
//            condition = 3;
//        }
//
//        rotOne.setPosition(0.4195);
//        rotTwo.setPosition(0.4195);
//        backSweeper.setPower(0);
//        frontSweeper.setPower(0);
//        ChaseTheCarrotConstantHeadingWithShooter(gate, 9, 3, 0, 3, 1, 0.04, 0.04, 0.03, 0.0005, 0, 800, 1);
//
//
//        sleep(550);
//
//        ChaseTheCarrotConstantHeadingWithColor(shotBruh, 9, 3, 0, 3, 1, 0.04, 0.04, 0.03, 0.0005, 0, 2500, 1);
//
////        timerOne.reset();
////        while(timerOne.milliseconds() < 1175){
////
////            resetCache();
////
////            runShooter(1350);
////            if(kickThree){
////
////                kickerThree.setPosition(KICKER_THREE_UP);
////
////                kickThree = false;
////                loop2 = true;
////            }
////
////            if(timerOne.milliseconds() > 250 && loop2){
////
////                kickerThree.setPosition(KICKER_THREE_DOWN);
////                loop2 = false;
////                loop3 = true;
////            }
////
////            if(timerOne.milliseconds() > 430 && loop3){
////                kickerOne.setPosition(KICKER_ONE_UP);
////                loop3 = false;
////                loop4 = true;
////            }
////
////            if(timerOne.milliseconds() > 680 && loop4){
////                kickerOne.setPosition(KICKER_ONE_DOWN);
////                loop4 = false;
////                loop5 = true;
////            }
////
////            if(timerOne.milliseconds() > 860 && loop5){
////                kickerTwo.setPosition(KICKER_TWO_UP);
////                loop5 = false;
////                loop6 = true;
////            }
////
////            if(timerOne.milliseconds() > 1150 && loop6){
////                kickerTwo.setPosition(KICKER_TWO_DOWN);
////                loop6 = false;
////            }
////        }
////        kickThree = true;
//
//        timerOne.reset();
//        String one = oneSlot, two = twoSlot, three = threeSlot;
//        while(timerOne.milliseconds() < 1800){
//
//            resetCache();
//
//            runShooter(1350);
//            if(kickThree){
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
//
//                kickThree = false;
//                loop2 = true;
//            }
//
//            if(timerOne.milliseconds() > 310 && loop2){
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
//
//                loop2 = false;
//                loop3 = true;
//            }
//
//            if(timerOne.milliseconds() > 490 + 250 && loop3){
//
//                loop3 = false;
//                loop4 = true;
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
//            }
//
//            if(timerOne.milliseconds() > 780 + 250 && loop4){
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
//                loop4 = false;
//                loop5 = true;
//            }
//
//            if(timerOne.milliseconds() > 980 + 500 && loop5){
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
//                loop5 = false;
//                loop6 = true;
//            }
//
//            if(timerOne.milliseconds() > 1280 + 500 && loop6){
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
//                loop6 = false;
//            }
//        }
//        kickThree = true;
//
//        oneShot = false;
//        twoShot = false;
//        threeShot = false;
//
//        oneKick = false;
//        twoKick = false;
//        threeKick = false;
//
//        backSweeper.setPower(1);
//
//
//
//        ChaseTheCarrotConstantHeading(midIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0.0005, 2500, 1);
//
//        ChaseTheCarrotConstantHeading(sweepMid, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0.0005, 2500, 1);
//        ChaseTheCarrotConstantHeading(midIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0.0005, 2500, 0.7);
//
//        sleep(300);
////
////        gate = PathGenerator.interpSplinePath(gate, new Point(getX(), getY(), 0));
////        ChaseTheCarrotConstantHeadingWithShooter(gate, 9, 3, -90, 3, 1, 0.04, 0.04, 0.03, 0.0005, 0, 1000, 1);
////        sleep(380);
//        backSweeper.setPower(0);
//        frontSweeper.setPower(0);
//        rotOne.setPosition(0.464);
//        rotTwo.setPosition(0.464);
//
//        ChaseTheCarrotConstantHeadingWithColor(shotBruh, 9, 3, -27, 3, 1, 0.04, 0.04, 0.03, 0.0005, 0, 2500, 1);
//
//        timerOne.reset();
//        one = oneSlot;
//        two = twoSlot;
//        three = threeSlot;
//        while(timerOne.milliseconds() < 1800){
//
//            resetCache();
//
//            runShooter(1400);
//            if(kickThree && timerOne.milliseconds()>80){
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
//
//                kickThree = false;
//                loop2 = true;
//            }
//
//            if(timerOne.milliseconds() > 310 && loop2){
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
//
//                loop2 = false;
//                loop3 = true;
//            }
//
//            if(timerOne.milliseconds() > 490 + 250 && loop3){
//
//                loop3 = false;
//                loop4 = true;
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
//            }
//
//            if(timerOne.milliseconds() > 780 + 250 && loop4){
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
//                loop4 = false;
//                loop5 = true;
//            }
//
//            if(timerOne.milliseconds() > 980 + 500 && loop5){
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
//                loop5 = false;
//                loop6 = true;
//            }
//
//            if(timerOne.milliseconds() > 1280 + 500 && loop6){
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
//                loop6 = false;
//            }
//        }
//        kickThree = true;
//
//        oneShot = false;
//        twoShot = false;
//        threeShot = false;
//
//        oneKick = false;
//        twoKick = false;
//        threeKick = false;
//
//
//        backSweeper.setPower(1);
//
//
//        finalIntake = PathGenerator.interpSplinePath(finalIntake, new Point(getX(), getY(), 0));
//
//        ChaseTheCarrotConstantHeading(finalIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 4000, 1);
//        sleep(300);
//
//        frontSweeper.setPower(0);
//        rotOne.setPosition(0.48);
//        rotTwo.setPosition(0.48);
//
//        backSweeper.setPower(0);
//
//        ChaseTheCarrotConstantHeadingWithColor(shotBruh, 9, 3, -34, 3, 1, 0.04, 0.04, 0.03, 0.0005, 0, 2500, 1);
//
//        sleep(200);
//
//
//
//        timerOne.reset();
//        one = oneSlot;
//        two = twoSlot;
//        three = threeSlot;
//        while(timerOne.milliseconds() < 1800){
//
//            resetCache();
//
//            runShooter(1350);
//            if(kickThree){
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
//
//                kickThree = false;
//                loop2 = true;
//            }
//
//            if(timerOne.milliseconds() > 310 && loop2){
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
//
//                loop2 = false;
//                loop3 = true;
//            }
//
//            if(timerOne.milliseconds() > 490 + 250 && loop3){
//
//                loop3 = false;
//                loop4 = true;
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
//            }
//
//            if(timerOne.milliseconds() > 780 + 250 && loop4){
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
//                loop4 = false;
//                loop5 = true;
//            }
//
//            if(timerOne.milliseconds() > 980 + 500 && loop5){
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
//                loop5 = false;
//                loop6 = true;
//            }
//
//            if(timerOne.milliseconds() > 1280 + 500 && loop6){
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
//                loop6 = false;
//            }
//        }
//        kickThree = true;
//
//        oneShot = false;
//        twoShot = false;
//        threeShot = false;
//
//        oneKick = false;
//        twoKick = false;
//        threeKick = false;
//
//        ChaseTheCarrotConstantHeading(finalPark, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 4000, 1);
//
//        sleep(500);







//
//
//        ChaseTheCarrotConstantHeading(sweepGate, 9, 3, 30, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1);
//        backSweeper.setPower(1);
//        sleep(160);
//
//        ChaseTheCarrotConstantHeading(moveGate, 9, 3, 30, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 800, 1);
//        sleep(410);
//        backSweeper.setPower(0);
//
//        ChaseTheCarrotConstantHeadingWithShooter(shotBruh, 9, 3, -34, 3, 1, 0.04, 0.04, 0.03, 0.0005, 0, 2500, 1);
//        frontSweeper.setPower(0);
//        sleep(250);
//        timerOne.reset();
//        while(timerOne.milliseconds() < 1175){
//
//            resetCache();
//
//            runShooter(1350);
//            if(kickThree){
//
//                kickerThree.setPosition(KICKER_THREE_UP);
//
//                kickThree = false;
//                loop2 = true;
//            }
//
//            if(timerOne.milliseconds() > 250 && loop2){
//
//                kickerThree.setPosition(KICKER_THREE_DOWN);
//                loop2 = false;
//                loop3 = true;
//            }
//
//            if(timerOne.milliseconds() > 430 && loop3){
//                kickerOne.setPosition(KICKER_ONE_UP);
//                loop3 = false;
//                loop4 = true;
//            }
//
//            if(timerOne.milliseconds() > 680 && loop4){
//                kickerOne.setPosition(KICKER_ONE_DOWN);
//                loop4 = false;
//                loop5 = true;
//            }
//
//            if(timerOne.milliseconds() > 860 && loop5){
//                kickerTwo.setPosition(KICKER_TWO_UP);
//                loop5 = false;
//                loop6 = true;
//            }
//
//            if(timerOne.milliseconds() > 1150 && loop6){
//                kickerTwo.setPosition(KICKER_TWO_DOWN);
//                loop6 = false;
//            }
//        }
//        kickThree = true;
//
//
//        ChaseTheCarrotConstantHeading(sweepGateTwo, 9, 3, 30, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1);
//        backSweeper.setPower(1);
//        sleep(160);
//
//
//
//        ChaseTheCarrotConstantHeading(moveGateTwo, 9, 3, 30, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 800, 1);
//        checkCurrent.reset();
//
//        sleep(410);
//        backSweeper.setPower(0);
//
//        ChaseTheCarrotConstantHeadingWithShooter(shotBruh, 9, 3, -34, 3, 1, 0.04, 0.04, 0.03, 0.0005, 0, 2500, 1);
//        frontSweeper.setPower(0);
//        sleep(250);
//
//        timerOne.reset();
//        while(timerOne.milliseconds() < 1175){
//
//            resetCache();
//
//            runShooter(1350);
//            if(kickThree){
//
//                kickerThree.setPosition(KICKER_THREE_UP);
//
//                kickThree = false;
//                loop2 = true;
//            }
//
//            if(timerOne.milliseconds() > 250 && loop2){
//
//                kickerThree.setPosition(KICKER_THREE_DOWN);
//                loop2 = false;
//                loop3 = true;
//            }
//
//            if(timerOne.milliseconds() > 430 && loop3){
//                kickerOne.setPosition(KICKER_ONE_UP);
//                loop3 = false;
//                loop4 = true;
//            }
//
//            if(timerOne.milliseconds() > 680 && loop4){
//                kickerOne.setPosition(KICKER_ONE_DOWN);
//                loop4 = false;
//                loop5 = true;
//            }
//
//            if(timerOne.milliseconds() > 860 && loop5){
//                kickerTwo.setPosition(KICKER_TWO_UP);
//                loop5 = false;
//                loop6 = true;
//            }
//
//            if(timerOne.milliseconds() > 1150 && loop6){
//                kickerTwo.setPosition(KICKER_TWO_DOWN);
//                loop6 = false;
//            }
//        }
//        kickThree = true;
//THIS IS TRUE END TO GATE AUTO



////        kickerThree.setPosition(KICKER_THREE_UP);
////        sleep(290);
////        kickerThree.setPosition(KICKER_THREE_DOWN);
////        sleep(200);
////        kickerOne.setPosition(KICKER_ONE_UP);
////        sleep(290);
////        kickerOne.setPosition(KICKER_ONE_DOWN);
////        sleep(200);
////        kickerTwo.setPosition(KICKER_TWO_UP);
////        sleep(200);
////        kickerTwo.setPosition(KICKER_TWO_DOWN);
//
//        frontSweeper.setPower(-1);
//
//        shooterOne.setPower(0);
//        shooterTwo.setPower(0);
//
//        rotOne.setPosition(0.55);
//        rotTwo.setPosition(0.55);
//        sleep(400);
//
//
//
//        firstIntake = PathGenerator.interpSplinePath(firstIntake, new Point(getX(), getY(), 0));
//
//        ChaseTheCarrotConstantHeadingWithLimeLight(firstIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500);
//        if(counter == 21){
//            order = new String[]{"green", "purple", "purple"};
//            condition = 1;
//        }else if(counter == 22){
//            order = new String[]{"purple", "green", "purple"};
//            condition = 2;
//        }else{
//            order = new String[]{"purple", "purple", "green"};
//            condition = 3;
//        }
//
//        frontSweeper.setPower(0);
//        backSweeper.setPower(0);
//
//        gate = PathGenerator.interpSplinePath(gate, new Point(getX(), getY(), 0));
//        ChaseTheCarrotConstantHeadingWithShooter(gate, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 1400, 1);
//        sleep(550);
//
//        rotOne.setPosition(0.175);
//        rotTwo.setPosition(0.175);
//
//
//
//
//
//
//
//
//
//        ChaseTheCarrotConstantHeadingWithColor(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.55);
//
//        timerOne.reset();
//        while(timerOne.milliseconds() < 1350){
//
//            resetCache();
//
//            runShooter(1300);
//            if(kickThree){
//
//                kickerThree.setPosition(KICKER_THREE_UP);
//
//                kickThree = false;
//                loop2 = true;
//            }
//
//            if(timerOne.milliseconds() > 310 && loop2){
//
//                kickerThree.setPosition(KICKER_THREE_DOWN);
//                loop2 = false;
//                loop3 = true;
//            }
//
//            if(timerOne.milliseconds() > 510 && loop3){
//                kickerOne.setPosition(KICKER_ONE_UP);
//                loop3 = false;
//                loop4 = true;
//            }
//
//            if(timerOne.milliseconds() > 800 && loop4){
//                kickerOne.setPosition(KICKER_ONE_DOWN);
//                loop4 = false;
//                loop5 = true;
//            }
//
//            if(timerOne.milliseconds() > 1000 && loop5){
//                kickerTwo.setPosition(KICKER_TWO_UP);
//                loop5 = false;
//                loop6 = true;
//            }
//
//            if(timerOne.milliseconds() > 1300 && loop6){
//                kickerTwo.setPosition(KICKER_TWO_DOWN);
//                loop6 = false;
//            }
//        }
//        kickThree = true;








//
//        shooterOne.setPower(0);
//        shooterTwo.setPower(0);
//
//        frontSweeper.setPower(-1);
//
//
//        secondIntake = PathGenerator.interpSplinePath(secondIntake, new Point(getX(), getY(), 0));
//        ChaseTheCarrotConstantHeading(secondIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 3000, 1);
//
//
//        frontSweeper.setPower(0);
//        backSweeper.setPower(0);
//
//
//
////        gate = PathGenerator.interpSplinePath(gate, new Point(getX(), getY(), 0));
////        ChaseTheCarrotConstantHeading(gate, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2300, 0.9);
////        sleep(1000);
//
//
//
//        shotBruh = PathGenerator.interpSplinePath(shotBruh, new Point(getX(), getY(), 0));
//
//        ChaseTheCarrotConstantHeadingWithColor(shotBruh, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500);
//
//
//        timerOne.reset();
//        while(timerOne.milliseconds() < 1350){
//
//            resetCache();
//
//            runShooter(1300);
//            if(kickThree){
//
//                kickerThree.setPosition(KICKER_THREE_UP);
//
//                kickThree = false;
//                loop2 = true;
//            }
//
//            if(timerOne.milliseconds() > 310 && loop2){
//
//                kickerThree.setPosition(KICKER_THREE_DOWN);
//                loop2 = false;
//                loop3 = true;
//            }
//
//            if(timerOne.milliseconds() > 510 && loop3){
//                kickerOne.setPosition(KICKER_ONE_UP);
//                loop3 = false;
//                loop4 = true;
//            }
//
//            if(timerOne.milliseconds() > 800 && loop4){
//                kickerOne.setPosition(KICKER_ONE_DOWN);
//                loop4 = false;
//                loop5 = true;
//            }
//
//            if(timerOne.milliseconds() > 1000 && loop5){
//                kickerTwo.setPosition(KICKER_TWO_UP);
//                loop5 = false;
//                loop6 = true;
//            }
//
//            if(timerOne.milliseconds() > 1300 && loop6){
//                kickerTwo.setPosition(KICKER_TWO_DOWN);
//                loop6 = false;
//            }
//        }
//        kickThree = true;
//
//
//
//        shooterOne.setPower(0);
//        shooterTwo.setPower(0);
//
//
//
//
//        frontSweeper.setPower(-1);
//
//
//
//        thirdIntake = PathGenerator.interpSplinePath(thirdIntake, new Point(getX(), getY(), 0));
//        ChaseTheCarrotConstantHeading(thirdIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 7000, 1);
//
//
//        frontSweeper.setPower(0);
//        backSweeper.setPower(0);
//
////        ChaseTheCarrotConstantHeading(bang2, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 1250 , 1);
//
//
//
//
//
//        ChaseTheCarrotConstantHeadingWithColor(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500);
//
//
//        timerOne.reset();
//        while(timerOne.milliseconds() < 1350){
//
//            resetCache();
//
//            runShooter(1300);
//            if(kickThree){
//
//                kickerThree.setPosition(KICKER_THREE_UP);
//
//                kickThree = false;
//                loop2 = true;
//            }
//
//            if(timerOne.milliseconds() > 310 && loop2){
//
//                kickerThree.setPosition(KICKER_THREE_DOWN);
//                loop2 = false;
//                loop3 = true;
//            }
//
//            if(timerOne.milliseconds() > 510 && loop3){
//                kickerOne.setPosition(KICKER_ONE_UP);
//                loop3 = false;
//                loop4 = true;
//            }
//
//            if(timerOne.milliseconds() > 800 && loop4){
//                kickerOne.setPosition(KICKER_ONE_DOWN);
//                loop4 = false;
//                loop5 = true;
//            }
//
//            if(timerOne.milliseconds() > 1000 && loop5){
//                kickerTwo.setPosition(KICKER_TWO_UP);
//                loop5 = false;
//                loop6 = true;
//            }
//
//            if(timerOne.milliseconds() > 1300 && loop6){
//                kickerTwo.setPosition(KICKER_TWO_DOWN);
//                loop6 = false;
//            }
//        }
//        kickThree = true;
//
//
//        shooterOne.setPower(0);
//        shooterTwo.setPower(0);
//        ChaseTheCarrotConstantHeadingWithShooter(park, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1);
//        shooterOne.setPower(0);
//        shooterTwo.setPower(0);
//        sleep(500);





    }
}

