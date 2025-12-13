package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.Motor;
import org.firstinspires.ftc.teamcode.Core.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

@Autonomous(name="Red Auto")
public class T1_Auto_Red extends Base{
    ArrayList<Point> shot1, firstIntake, gate, secondIntake, thirdIntake;
    @Override
    public void runOpMode() throws InterruptedException {
        boolean kickThree = true;
        boolean loop2 = false, loop3 = false, loop4 = false, loop5 = false, loop6 = false,
                loop7 = false, loop8 = false, loop9 = false, loop10 = false, loop11 = false;
        initHardware(hardwareMap);


        ElapsedTime timerOne =  new ElapsedTime();


        hoodOne = hardwareMap.servo.get("hoodOne");
        hoodTwo = hardwareMap.servo.get("hoodTwo");

        kickerOne = hardwareMap.servo.get("kickIntake");
        kickerTwo = hardwareMap.servo.get("kickOut");
        kickerThree = hardwareMap.servo.get("kickMiddle");

        RevColorSensorV3 slotIntake = hardwareMap.get(RevColorSensorV3.class, "colorIn");
        RevColorSensorV3 slotOut = hardwareMap.get(RevColorSensorV3.class, "colorOut");
        RevColorSensorV3 finalColor = hardwareMap.get(RevColorSensorV3.class, "colorLast");

        Servo[] kickers = new Servo[3];



        double KICKER_ONE_DOWN = 0.15, KICKER_ONE_UP = 0.634, KICKER_TWO_DOWN = 0.6, KICKER_TWO_UP = 0.3,
                KICKER_THREE_DOWN = 0.638, KICKER_THREE_UP = 0.91;



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

        shot1 = new ArrayList<>();
        shot1.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-38, -32, 0, 0.8)
                        )
                )
        );

        firstIntake = new ArrayList<>();
        firstIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-20, -47.5, 0, 0.8),
                                new Point(12, -47.5, 0, 0.3)
                        )
                )
        );

        gate = new ArrayList<>();
        gate.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(2.5, -52, 0, 0.8),
                                new Point(16, -59, 0, 0.8)
                        )
                )
        );

        secondIntake = new ArrayList<>();
        secondIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-20, -71.5, 0, 0.8),
                                new Point(19, -71.5, 0, 0.3)
                        )
                )
        );

        thirdIntake = new ArrayList<>();
        thirdIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-15, -96, 0, 0.8),
                                new Point(14.5, -96, 0, 0.3)
                        )
                )
        );



        String oneSlot = "", twoSlot = "", threeSlot ="";



        waitForStart();



        hoodOne.setPosition(0.25);
        hoodTwo.setPosition(0.75);

        ;
        ChaseTheCarrotConstantHeadingWithShooter(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.8);

        timerOne.reset();
        while(timerOne.milliseconds() < 1250){

            resetCache();

            runShooter(1020);
            if(kickThree){
                kickerThree.setPosition(KICKER_THREE_UP);
                kickThree = false;
                loop2 = true;
            }

            if(timerOne.milliseconds() > 310 && loop2){
                kickerThree.setPosition(KICKER_THREE_DOWN);
                loop2 = false;
                loop3 = true;
            }

            if(timerOne.milliseconds() > 490 && loop3){
                kickerOne.setPosition(KICKER_ONE_UP);
                loop3 = false;
                loop4 = true;
            }

            if(timerOne.milliseconds() > 780 && loop4){
                kickerOne.setPosition(KICKER_ONE_DOWN);
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 980 && loop5){
                kickerTwo.setPosition(KICKER_TWO_UP);
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 1180 && loop6){
                kickerTwo.setPosition(KICKER_TWO_DOWN);
                loop6 = false;
            }
        }
        kickThree = true;

//        kickerThree.setPosition(KICKER_THREE_UP);
//        sleep(290);
//        kickerThree.setPosition(KICKER_THREE_DOWN);
//        sleep(200);
//        kickerOne.setPosition(KICKER_ONE_UP);
//        sleep(290);
//        kickerOne.setPosition(KICKER_ONE_DOWN);
//        sleep(200);
//        kickerTwo.setPosition(KICKER_TWO_UP);
//        sleep(200);
//        kickerTwo.setPosition(KICKER_TWO_DOWN);

        frontSweeper.setPower(-0.95);
//        backSweeper.setPower(-0.95);
        shooterOne.setPower(0);
        shooterTwo.setPower(0);



        firstIntake = PathGenerator.interpSplinePath(firstIntake, new Point(getX(), getY(), 0));
        ChaseTheCarrotConstantHeading(firstIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 3000, 0.9);
        sleep(500);
        frontSweeper.setPower(0);
        backSweeper.setPower(0);

        gate = PathGenerator.interpSplinePath(gate, new Point(getX(), getY(), 0));
        ChaseTheCarrotConstantHeading(gate, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 1100, 0.9);
        sleep(250);





        ChaseTheCarrotConstantHeadingWithShooter(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.9);



        timerOne.reset();
        while(timerOne.milliseconds() < 1290){

            resetCache();

            runShooter(1020);
            if(kickThree){
                kickerOne.setPosition(KICKER_ONE_UP);
                kickThree = false;
                loop2 = true;
            }

            if(timerOne.milliseconds() > 290 && loop2){
                kickerOne.setPosition(KICKER_ONE_DOWN);
                loop2 = false;
                loop3 = true;
            }

            if(timerOne.milliseconds() > 490 && loop3){
                kickerTwo.setPosition(KICKER_TWO_UP);
                loop3 = false;
                loop4 = true;
            }

            if(timerOne.milliseconds() > 780 && loop4){
                kickerTwo.setPosition(KICKER_TWO_DOWN);
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 980 && loop5){
                kickerThree.setPosition(KICKER_THREE_UP);
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 1250 && loop6){
                kickerThree.setPosition(KICKER_THREE_DOWN);
                loop6 = false;
            }
        }
        kickThree = true;


        shooterOne.setPower(0);
        shooterTwo.setPower(0);

        frontSweeper.setPower(-0.95);
        backSweeper.setPower(-0.95);

        secondIntake = PathGenerator.interpSplinePath(secondIntake, new Point(getX(), getY(), 0));
        ChaseTheCarrotConstantHeading(secondIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 3000, 0.9);
        sleep(500);

        frontSweeper.setPower(0);
        backSweeper.setPower(0);




        ChaseTheCarrotConstantHeadingWithShooter(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.8);

        timerOne.reset();
        while(timerOne.milliseconds() < 1290){

            resetCache();

            runShooter(1020);
            if(kickThree){
                kickerTwo.setPosition(KICKER_TWO_UP);
                kickThree = false;
                loop2 = true;
            }

            if(timerOne.milliseconds() > 290 && loop2){
                kickerTwo.setPosition(KICKER_TWO_DOWN);
                loop2 = false;
                loop3 = true;
            }

            if(timerOne.milliseconds() > 490 && loop3){
                kickerOne.setPosition(KICKER_ONE_UP);
                loop3 = false;
                loop4 = true;
            }

            if(timerOne.milliseconds() > 780 && loop4){
                kickerOne.setPosition(KICKER_ONE_DOWN);
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 980 && loop5){
                kickerThree.setPosition(KICKER_THREE_UP);
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 1250 && loop6){
                kickerThree.setPosition(KICKER_THREE_DOWN);
                loop6 = false;
            }
        }
        kickThree = true;




        frontSweeper.setPower(-0.95);


        thirdIntake = PathGenerator.interpSplinePath(thirdIntake, new Point(getX(), getY(), 0));
        ChaseTheCarrotConstantHeading(thirdIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 7000, 0.9);
        sleep(500);

        frontSweeper.setPower(0);




        ChaseTheCarrotConstantHeadingWithShooter(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.9);

        timerOne.reset();
        while(timerOne.milliseconds() < 1250){

            resetCache();

            runShooter(1020);
            if(kickThree){
                kickerThree.setPosition(KICKER_THREE_UP);
                kickThree = false;
                loop2 = true;
            }

            if(timerOne.milliseconds() > 290 && loop2){
                kickerThree.setPosition(KICKER_THREE_DOWN);
                loop2 = false;
                loop3 = true;
            }

            if(timerOne.milliseconds() > 490 && loop3){
                kickerOne.setPosition(KICKER_ONE_UP);
                loop3 = false;
                loop4 = true;
            }

            if(timerOne.milliseconds() > 780 && loop4){
                kickerOne.setPosition(KICKER_ONE_DOWN);
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 980 && loop5){
                kickerTwo.setPosition(KICKER_TWO_UP);
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 1180 && loop6){
                kickerTwo.setPosition(KICKER_TWO_DOWN);
                loop6 = false;
            }
        }
        kickThree = true;






    }
}
