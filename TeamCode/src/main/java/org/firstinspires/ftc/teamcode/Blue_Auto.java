package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Core.Motor;
import org.firstinspires.ftc.teamcode.Core.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

@Autonomous(name="Blue Auto")
public class Blue_Auto extends Base {
    ArrayList<Point> shot1, firstIntake, gate, secondIntake, thirdIntake, park, baseShoot;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean kickThree = true;
        boolean loop2 = false, loop3 = false, loop4 = false, loop5 = false, loop6 = false,
                loop7 = false, loop8 = false, loop9 = false, loop10 = false, loop11 = false;
        initHardware(hardwareMap);


        ElapsedTime timerOne = new ElapsedTime();


        hoodOne = hardwareMap.servo.get("hoodOne");
        hoodTwo = hardwareMap.servo.get("hoodTwo");

        kickerOne = hardwareMap.servo.get("kickIntake");
        kickerTwo = hardwareMap.servo.get("kickOut");
        kickerThree = hardwareMap.servo.get("kickMiddle");

        RevColorSensorV3 slotIntake = hardwareMap.get(RevColorSensorV3.class, "colorIn");
        RevColorSensorV3 slotOut = hardwareMap.get(RevColorSensorV3.class, "colorOut");
        RevColorSensorV3 finalColor = hardwareMap.get(RevColorSensorV3.class, "colorLast");

        Servo[] kickers = new Servo[3];


        double KICKER_ONE_DOWN = 0.72, KICKER_ONE_UP = 0.16, KICKER_TWO_DOWN = 0.445, KICKER_TWO_UP = 1,
                KICKER_THREE_DOWN = 0.96, KICKER_THREE_UP = 0.38;


        Map<Servo, Double> UP_POSITIONS = new HashMap<>();
        Map<Servo, Double> DOWN_POSITIONS = new HashMap<>();

        UP_POSITIONS.put(kickerOne, KICKER_ONE_UP);
        UP_POSITIONS.put(kickerTwo, KICKER_TWO_UP);
        UP_POSITIONS.put(kickerThree, KICKER_THREE_UP);

        DOWN_POSITIONS.put(kickerOne, KICKER_ONE_DOWN);
        DOWN_POSITIONS.put(kickerTwo, KICKER_TWO_DOWN);
        DOWN_POSITIONS.put(kickerThree, KICKER_THREE_DOWN);

        kickerOne.setPosition(KICKER_ONE_DOWN);
        kickerTwo.setPosition(KICKER_TWO_DOWN);
        kickerThree.setPosition(KICKER_THREE_DOWN);
        rotOne.setPosition(0.5);
        rotTwo.setPosition(0.5);

        shot1 = new ArrayList<>();
        shot1.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(48, 0, 0, 0.9)
                        )
                )
        );

        park = new ArrayList<>();
        park.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-18, -30, 0, 0.8),
                                new Point(-60, -30, 0, 0.4)
                        )
                )
        );

        baseShoot = new ArrayList<>();
        baseShoot.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(0,-8, 0, 0.9)
                        )
                )
        );
        firstIntake = new ArrayList<>();
        firstIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-30, -14, 0, 0.8)

                        )
                )
        );

        gate = new ArrayList<>();
        gate.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-55, -30.5, 0, 0.65)

                        )
                )
        );

        secondIntake = new ArrayList<>();
        secondIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-30, -62, 0, 0.8)
                        )
                )
        );

        thirdIntake = new ArrayList<>();
        thirdIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(15, -94, 0, 0.8),
                                new Point(-17, -94, 0, 0.3)
                        )
                )
        );


        String oneSlot = "", twoSlot = "", threeSlot = "";


        waitForStart();


        hoodOne.setPosition(0.75);
        hoodTwo.setPosition(0.25);


        ChaseTheCarrotConstantHeadingWithShooter(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1 );

        sleep(250);
        timerOne.reset();
        while(timerOne.milliseconds() < 850){

            resetCache();


            runShooter(1020);

            if(kickThree){
                kickerThree.setPosition(KICKER_THREE_UP);
                kickThree = false;
                loop2 = true;
            }

            if(timerOne.milliseconds() > 230 && loop2){
                kickerThree.setPosition(KICKER_THREE_DOWN);
                loop2 = false;
                loop3 = true;
            }

            if(timerOne.milliseconds() > 300 && loop3){
                kickerOne.setPosition(KICKER_ONE_UP);
                loop3 = false;
                loop4 = true;
            }

            if(timerOne.milliseconds() > 530 && loop4){
                kickerOne.setPosition(KICKER_ONE_DOWN);
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 600 && loop5){
                kickerTwo.setPosition(KICKER_TWO_UP);
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 830 && loop6){
                kickerTwo.setPosition(KICKER_TWO_DOWN);
                loop6 = false;
            }
        }
        kickThree = true;

        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -45));
        odo.update();
        sleep(750);
        backSweeper.setPower(-0.8);
        park = PathGenerator.interpSplinePath(park, new Point(getX(), getY(), 0));

        ChaseTheCarrotConstantHeading(park, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 3000, 1 );
        sleep(500);
        ChaseTheCarrotConstantHeading(baseShoot, 9, 3, 15, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1 );
        sleep(250);
        timerOne.reset();
        while(timerOne.milliseconds() < 850){

            resetCache();


            runShooter(1020);

            if(kickThree){
                kickerThree.setPosition(KICKER_THREE_UP);
                kickThree = false;
                loop2 = true;
            }

            if(timerOne.milliseconds() > 230 && loop2){
                kickerThree.setPosition(KICKER_THREE_DOWN);
                loop2 = false;
                loop3 = true;
            }

            if(timerOne.milliseconds() > 300 && loop3){
                kickerOne.setPosition(KICKER_ONE_UP);
                loop3 = false;
                loop4 = true;
            }

            if(timerOne.milliseconds() > 530 && loop4){
                kickerOne.setPosition(KICKER_ONE_DOWN);
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 600 && loop5){
                kickerTwo.setPosition(KICKER_TWO_UP);
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 830 && loop6){
                kickerTwo.setPosition(KICKER_TWO_DOWN);
                loop6 = false;
            }
        }
        kickThree = true;
        backSweeper.setPower(-0.8);
        ChaseTheCarrotConstantHeading(gate, 9, 3, -39, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 5000, 1 );
        sleep(2000);
        backSweeper.setPower(0);
        ChaseTheCarrotConstantHeading(baseShoot, 9, 3, 15, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1 );
        sleep(600);
        backSweeper.setPower(-0.8);
        ChaseTheCarrotConstantHeading(gate, 9, 3, -39, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 5000, 1 );
        sleep(2000);
        backSweeper.setPower(0);
        ChaseTheCarrotConstantHeading(baseShoot, 9, 3, 15, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1 );
        sleep(600);
        backSweeper.setPower(-0.8);
        ChaseTheCarrotConstantHeading(firstIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1 );
        sleep(500);
        ChaseTheCarrotConstantHeading(baseShoot, 9, 3, 15, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1 );
        sleep(600);
        ChaseTheCarrotConstantHeading(secondIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1 );
        sleep(600);
        ChaseTheCarrotConstantHeading(baseShoot, 9, 3, 15, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1 );
        sleep(500);




//        timerOne.reset();
//        while(timerOne.milliseconds() < 1300){
//
//            resetCache();
//
//
//            runShooter(1020);
//            if(kickThree){
//                kickerThree.setPosition(KICKER_THREE_UP);
//                kickThree = false;
//                loop2 = true;
//            }
//
//            if(timerOne.milliseconds() > 310 && loop2){
//                kickerThree.setPosition(KICKER_THREE_DOWN);
//                loop2 = false;
//                loop3 = true;
//            }
//
//            if(timerOne.milliseconds() > 490 && loop3){
//                kickerOne.setPosition(KICKER_ONE_UP);
//                loop3 = false;
//                loop4 = true;
//            }
//
//            if(timerOne.milliseconds() > 780 && loop4){
//                kickerOne.setPosition(KICKER_ONE_DOWN);
//                loop4 = false;
//                loop5 = true;
//            }
//
//            if(timerOne.milliseconds() > 980 && loop5){
//                kickerTwo.setPosition(KICKER_TWO_UP);
//                loop5 = false;
//                loop6 = true;
//            }
//
//            if(timerOne.milliseconds() > 1280 && loop6){
//                kickerTwo.setPosition(KICKER_TWO_DOWN);
//                loop6 = false;
//            }
//        }
//        kickThree = true;
//
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
////        frontSweeper.setPower(-0.95);
//        backSweeper.setPower(-0.95);
//        shooterOne.setPower(0);
//        shooterTwo.setPower(0);
//
//
//
//        firstIntake = PathGenerator.interpSplinePath(firstIntake, new Point(getX(), getY(), 0));
//        ChaseTheCarrotConstantHeading(firstIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 3000, 0.9);
//        sleep(500);
//        frontSweeper.setPower(0);
//        backSweeper.setPower(0);
//
//        gate = PathGenerator.interpSplinePath(gate, new Point(getX(), getY(), 0));
//        ChaseTheCarrotConstantHeading(gate, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 1100, 0.9);
//        sleep(575);
//
//
//
//
//
//        ChaseTheCarrotConstantHeadingWithShooter(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.9);
//
//
//
//        timerOne.reset();
//        while(timerOne.milliseconds() < 1300){
//
//            resetCache();
//
//            runShooter(1020);
//            if(kickThree){
//                kickerThree.setPosition(KICKER_THREE_UP);
//                kickThree = false;
//                loop2 = true;
//            }
//
//            if(timerOne.milliseconds() > 310 && loop2){
//                kickerThree.setPosition(KICKER_THREE_DOWN);
//                loop2 = false;
//                loop3 = true;
//            }
//
//            if(timerOne.milliseconds() > 490 && loop3){
//                kickerOne.setPosition(KICKER_ONE_UP);
//                loop3 = false;
//                loop4 = true;
//            }
//
//            if(timerOne.milliseconds() > 780 && loop4){
//                kickerOne.setPosition(KICKER_ONE_DOWN);
//                loop4 = false;
//                loop5 = true;
//            }
//
//            if(timerOne.milliseconds() > 980 && loop5){
//                kickerTwo.setPosition(KICKER_TWO_UP);
//                loop5 = false;
//                loop6 = true;
//            }
//
//            if(timerOne.milliseconds() > 1280 && loop6){
//                kickerTwo.setPosition(KICKER_TWO_DOWN);
//                loop6 = false;
//            }
//        }
//        kickThree = true;
//
//
//        shooterOne.setPower(0);
//        shooterTwo.setPower(0);
//
////        frontSweeper.setPower(-0.95);
//        backSweeper.setPower(-0.95);
//
//        secondIntake = PathGenerator.interpSplinePath(secondIntake, new Point(getX(), getY(), 0));
//        ChaseTheCarrotConstantHeading(secondIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 3000, 0.9);
//        sleep(500);
//
//        frontSweeper.setPower(0);
//        backSweeper.setPower(0);
//
//
//
//
//        ChaseTheCarrotConstantHeadingWithShooter(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.8);
//
//        timerOne.reset();
//        while(timerOne.milliseconds() < 1300){
//
//            resetCache();
//
//            runShooter(1020);
//            if(kickThree){
//                kickerThree.setPosition(KICKER_THREE_UP);
//                kickThree = false;
//                loop2 = true;
//            }
//
//            if(timerOne.milliseconds() > 310 && loop2){
//                kickerThree.setPosition(KICKER_THREE_DOWN);
//                loop2 = false;
//                loop3 = true;
//            }
//
//            if(timerOne.milliseconds() > 490 && loop3){
//                kickerOne.setPosition(KICKER_ONE_UP);
//                loop3 = false;
//                loop4 = true;
//            }
//
//            if(timerOne.milliseconds() > 780 && loop4){
//                kickerOne.setPosition(KICKER_ONE_DOWN);
//                loop4 = false;
//                loop5 = true;
//            }
//
//            if(timerOne.milliseconds() > 980 && loop5){
//                kickerTwo.setPosition(KICKER_TWO_UP);
//                loop5 = false;
//                loop6 = true;
//            }
//
//            if(timerOne.milliseconds() > 1280 && loop6){
//                kickerTwo.setPosition(KICKER_TWO_DOWN);
//                loop6 = false;
//            }
//        }
//        kickThree = true;
//
//
//
//
////        frontSweeper.setPower(-0.95);
//        backSweeper.setPower(-0.95);
//
//
//        thirdIntake = PathGenerator.interpSplinePath(thirdIntake, new Point(getX(), getY(), 0));
//        ChaseTheCarrotConstantHeading(thirdIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 7000, 0.9);
//        sleep(500);
//
//        frontSweeper.setPower(0);
//        backSweeper.setPower(0);
//
//
//
//
//        ChaseTheCarrotConstantHeadingWithShooter(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.9);
//
//        timerOne.reset();
//        while(timerOne.milliseconds() < 1300){
//
//            resetCache();
//
//            runShooter(1020);
//            if(kickThree){
//                kickerThree.setPosition(KICKER_THREE_UP);
//                kickThree = false;
//                loop2 = true;
//            }
//
//            if(timerOne.milliseconds() > 310 && loop2){
//                kickerThree.setPosition(KICKER_THREE_DOWN);
//                loop2 = false;
//                loop3 = true;
//            }
//
//            if(timerOne.milliseconds() > 490 && loop3){
//                kickerOne.setPosition(KICKER_ONE_UP);
//                loop3 = false;
//                loop4 = true;
//            }
//
//            if(timerOne.milliseconds() > 780 && loop4){
//                kickerOne.setPosition(KICKER_ONE_DOWN);
//                loop4 = false;
//                loop5 = true;
//            }
//
//            if(timerOne.milliseconds() > 980 && loop5){
//                kickerTwo.setPosition(KICKER_TWO_UP);
//                loop5 = false;
//                loop6 = true;
//            }
//
//            if(timerOne.milliseconds() > 1280 && loop6){
//                kickerTwo.setPosition(KICKER_TWO_DOWN);
//                loop6 = false;
//            }
//        }
//        kickThree = true;
//
//        shooterOne.setPower(0);
//        shooterTwo.setPower(0);
//        ChaseTheCarrotConstantHeadingWithShooter(park, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.9);
//
//        sleep(500);
//
//
//
//
//
//    }
    }
}
