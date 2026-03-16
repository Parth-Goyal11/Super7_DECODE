package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Core.Motor;
import org.firstinspires.ftc.teamcode.Core.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

@Autonomous(name="Blue T3 Auto")
public class T1_Auto_Blue extends Base{

    ArrayList<Point> shot1, firstIntake, gate, secondIntake, thirdIntake, park, bang1, bang2, shotBruh;
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


        hoodOne = hardwareMap.servo.get("hoodOne");
        hoodTwo = hardwareMap.servo.get("hoodTwo");



        RevColorSensorV3 slotIntake = hardwareMap.get(RevColorSensorV3.class, "colorIn");
        RevColorSensorV3 slotOut = hardwareMap.get(RevColorSensorV3.class, "colorOut");
        RevColorSensorV3 finalColor = hardwareMap.get(RevColorSensorV3.class, "colorLast");
        RevColorSensorV3 otherSlot = hardwareMap.get(RevColorSensorV3.class, "bruhColor");

        Servo[] kickers = new Servo[3];

        String[] order = new String[3];




        double KICKER_ONE_DOWN = 0.2, KICKER_ONE_UP = 0.9, KICKER_TWO_DOWN = 1, KICKER_TWO_UP = 0.3,
                KICKER_THREE_DOWN = 0.15, KICKER_THREE_UP = 1;



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
        rotOne.setPosition(0.865);
        rotTwo.setPosition(0.865);

        shot1 = new ArrayList<>();
        shot1.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(29, -36, 0, 1)
                        )
                )
        );

        shotBruh = new ArrayList<>();
        shotBruh.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(20, -50, 0, 1),
                                new Point(29, -36, 0, 1)
                        )
                )
        );

        park = new ArrayList<>();
        park.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(29, -20, 0, 0.8)
                        )
                )
        );
        firstIntake = new ArrayList<>();
        firstIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(20, -45.5, 0, 1),
                                new Point(-15.5, -45.5, 0, 0.3)
                        )
                )
        );

        bang1 = new ArrayList<>();
        bang1.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-27, -70, 0, 0.7)
                        )
                )
        );

        bang2 = new ArrayList<>();
        bang2.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-25, -94, 0, 0.7)
                        )
                )
        );



        gate = new ArrayList<>();
        gate.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(1, -50.5, 0, 0.8),
                                new Point(-20, -55.5, 0, 1)
                        )
                )
        );

        secondIntake = new ArrayList<>();
        secondIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(20, -67.4, 0, 1),
                                new Point(-31, -67.4, 0, 0.3)
                        )
                )
        );



        thirdIntake = new ArrayList<>();
        thirdIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(15, -91, 0, 1),
                                new Point(-17.5, -91, 0, 0.3)
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



        hoodOne.setPosition(0.68);
        hoodTwo.setPosition(0.32);


        ChaseTheCarrotConstantHeadingWithShooter(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1);
        sleep(250);
        timerOne.reset();
        while(timerOne.milliseconds() < 1300){

            resetCache();

            runShooter(1050);
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

            if(timerOne.milliseconds() > 1280 && loop6){
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

        backSweeper.setPower(-1);

        shooterOne.setPower(0);
        shooterTwo.setPower(0);

        rotOne.setPosition(0.37);
        rotTwo.setPosition(0.37);
        sleep(400);



        firstIntake = PathGenerator.interpSplinePath(firstIntake, new Point(getX(), getY(), 0));

        ChaseTheCarrotConstantHeadingWithLimeLight(firstIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500);
        if(counter == 21){
            order = new String[]{"green", "purple", "purple"};
            condition = 1;
        }else if(counter == 22){
            order = new String[]{"purple", "green", "purple"};
            condition = 2;
        }else{
            order = new String[]{"purple", "purple", "green"};
            condition = 3;
        }
        sleep(500);
        frontSweeper.setPower(0);
        backSweeper.setPower(0);

        rotOne.setPosition(0.877);
        rotTwo.setPosition(0.877);



        gate = PathGenerator.interpSplinePath(gate, new Point(getX(), getY(), 0));
        ChaseTheCarrotConstantHeading(gate, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 1100, 1);
        sleep(750);





        ChaseTheCarrotConstantHeadingWithColor(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500);
        sleep(250);


        timerOne.reset();
        String one = oneSlot, two = twoSlot, three = threeSlot;
        while(timerOne.milliseconds() < 1300){

            resetCache();

            runShooter(1050);
            if(kickThree){

                if(order[0].equals(one)){
                    kickerOne.setPosition(KICKER_ONE_UP);
                    oneShot = true;
                    oneKick = true;
                }else if(order[0].equals(two)){
                    kickerTwo.setPosition(KICKER_TWO_UP);
                    twoShot = true;
                    twoKick = true;
                }else{
                    kickerThree.setPosition(KICKER_THREE_UP);
                    threeShot = true;
                    threeKick = true;
                }



                kickThree = false;
                loop2 = true;
            }

            if(timerOne.milliseconds() > 310 && loop2){

                if(oneKick){
                    kickerOne.setPosition(KICKER_ONE_DOWN);
                    oneKick = false;
                }else if(twoKick){
                    kickerTwo.setPosition(KICKER_TWO_DOWN);
                    twoKick = false;
                }else{
                    kickerThree.setPosition(KICKER_THREE_DOWN);
                    threeKick = false;
                }


                loop2 = false;
                loop3 = true;
            }

            if(timerOne.milliseconds() > 490 && loop3){

                loop3 = false;
                loop4 = true;

                if(order[1].equals(one) && !oneShot){
                    kickerOne.setPosition(KICKER_ONE_UP);
                    oneShot = true;
                    oneKick = true;
                }else if(order[1].equals(two) && !twoShot){
                    kickerTwo.setPosition(KICKER_TWO_UP);
                    twoShot = true;
                    twoKick = true;
                }else{
                    kickerThree.setPosition(KICKER_THREE_UP);
                    threeShot = true;
                    threeKick = true;
                }
            }

            if(timerOne.milliseconds() > 780 && loop4){
                if(oneKick){
                    kickerOne.setPosition(KICKER_ONE_DOWN);
                    oneKick = false;
                }else if(twoKick){
                    kickerTwo.setPosition(KICKER_TWO_DOWN);
                    twoKick = false;
                }else{
                    kickerThree.setPosition(KICKER_THREE_DOWN);
                    threeKick = false;
                }
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 980 && loop5){
                if(oneShot && twoShot){
                    kickerThree.setPosition(KICKER_THREE_UP);
                    threeKick = true;
                }else if(oneShot && threeShot){
                    kickerTwo.setPosition(KICKER_TWO_UP);
                    twoKick = true;
                }else{
                    kickerOne.setPosition(KICKER_ONE_UP);
                    oneKick = true;
                }
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 1280 && loop6){
                if(oneKick){
                    kickerOne.setPosition(KICKER_ONE_DOWN);
                    oneKick = false;
                }else if(twoKick){
                    kickerTwo.setPosition(KICKER_TWO_DOWN);
                    twoKick = false;
                }else{
                    kickerThree.setPosition(KICKER_THREE_DOWN);
                    threeKick = false;
                }
                loop6 = false;
            }
        }
        kickThree = true;

        oneShot = false;
        twoShot = false;
        threeShot = false;

        oneKick = false;
        twoKick = false;
        threeKick = false;


        shooterOne.setPower(0);
        shooterTwo.setPower(0);

        backSweeper.setPower(-1);


        secondIntake = PathGenerator.interpSplinePath(secondIntake, new Point(getX(), getY(), 0));
        ChaseTheCarrotConstantHeading(secondIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 3000, 1);


        frontSweeper.setPower(0);
        backSweeper.setPower(0);

        ChaseTheCarrotConstantHeading(bang1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 600, 1);


//        gate = PathGenerator.interpSplinePath(gate, new Point(getX(), getY(), 0));
//        ChaseTheCarrotConstantHeading(gate, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2300, 0.9);
//        sleep(1000);



        shotBruh = PathGenerator.interpSplinePath(shotBruh, new Point(getX(), getY(), 0));

        ChaseTheCarrotConstantHeadingWithColor(shotBruh, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500);
        sleep(250);
        timerOne.reset();
        one = oneSlot;
        two = twoSlot;
        three = threeSlot;
        while(timerOne.milliseconds() < 1300){

            resetCache();

            runShooter(1050);
            if(kickThree){

                if(order[0].equals(one)){
                    kickerOne.setPosition(KICKER_ONE_UP);
                    oneShot = true;
                    oneKick = true;
                }else if(order[0].equals(two)){
                    kickerTwo.setPosition(KICKER_TWO_UP);
                    twoShot = true;
                    twoKick = true;
                }else{
                    kickerThree.setPosition(KICKER_THREE_UP);
                    threeShot = true;
                    threeKick = true;
                }



                kickThree = false;
                loop2 = true;
            }

            if(timerOne.milliseconds() > 310 && loop2){

                if(oneKick){
                    kickerOne.setPosition(KICKER_ONE_DOWN);
                    oneKick = false;
                }else if(twoKick){
                    kickerTwo.setPosition(KICKER_TWO_DOWN);
                    twoKick = false;
                }else{
                    kickerThree.setPosition(KICKER_THREE_DOWN);
                    threeKick = false;
                }


                loop2 = false;
                loop3 = true;
            }

            if(timerOne.milliseconds() > 490 && loop3){

                loop3 = false;
                loop4 = true;

                if(order[1].equals(one) && !oneShot){
                    kickerOne.setPosition(KICKER_ONE_UP);
                    oneShot = true;
                    oneKick = true;
                }else if(order[1].equals(two) && !twoShot){
                    kickerTwo.setPosition(KICKER_TWO_UP);
                    twoShot = true;
                    twoKick = true;
                }else{
                    kickerThree.setPosition(KICKER_THREE_UP);
                    threeShot = true;
                    threeKick = true;
                }
            }

            if(timerOne.milliseconds() > 780 && loop4){
                if(oneKick){
                    kickerOne.setPosition(KICKER_ONE_DOWN);
                    oneKick = false;
                }else if(twoKick){
                    kickerTwo.setPosition(KICKER_TWO_DOWN);
                    twoKick = false;
                }else{
                    kickerThree.setPosition(KICKER_THREE_DOWN);
                    threeKick = false;
                }
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 980 && loop5){
                if(oneShot && twoShot){
                    kickerThree.setPosition(KICKER_THREE_UP);
                    threeKick = true;
                }else if(oneShot && threeShot){
                    kickerTwo.setPosition(KICKER_TWO_UP);
                    twoKick = true;
                }else{
                    kickerOne.setPosition(KICKER_ONE_UP);
                    oneKick = true;
                }
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 1280 && loop6){
                if(oneKick){
                    kickerOne.setPosition(KICKER_ONE_DOWN);
                    oneKick = false;
                }else if(twoKick){
                    kickerTwo.setPosition(KICKER_TWO_DOWN);
                    twoKick = false;
                }else{
                    kickerThree.setPosition(KICKER_THREE_DOWN);
                    threeKick = false;
                }
                loop6 = false;
            }
        }
        kickThree = true;
        oneShot = false;
        twoShot = false;
        threeShot = false;

        oneKick = false;
        twoKick = false;
        threeKick = false;
        shooterOne.setPower(0);
        shooterTwo.setPower(0);




        backSweeper.setPower(-1);



        thirdIntake = PathGenerator.interpSplinePath(thirdIntake, new Point(getX(), getY(), 0));
        ChaseTheCarrotConstantHeading(thirdIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 7000, 1);


        frontSweeper.setPower(0);
        backSweeper.setPower(0);

        ChaseTheCarrotConstantHeading(bang2, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 1250 , 1);
        sleep(500);





        ChaseTheCarrotConstantHeadingWithColor(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500);

        timerOne.reset();
        one = oneSlot;
        two = twoSlot;
        three = threeSlot;
        while(timerOne.milliseconds() < 1300){

            resetCache();

            runShooter(1050);
            if(kickThree){

                if(order[0].equals(one)){
                    kickerOne.setPosition(KICKER_ONE_UP);
                    oneShot = true;
                    oneKick = true;
                }else if(order[0].equals(two)){
                    kickerTwo.setPosition(KICKER_TWO_UP);
                    twoShot = true;
                    twoKick = true;
                }else{
                    kickerThree.setPosition(KICKER_THREE_UP);
                    threeShot = true;
                    threeKick = true;
                }



                kickThree = false;
                loop2 = true;
            }

            if(timerOne.milliseconds() > 310 && loop2){

                if(oneKick){
                    kickerOne.setPosition(KICKER_ONE_DOWN);
                    oneKick = false;
                }else if(twoKick){
                    kickerTwo.setPosition(KICKER_TWO_DOWN);
                    twoKick = false;
                }else{
                    kickerThree.setPosition(KICKER_THREE_DOWN);
                    threeKick = false;
                }


                loop2 = false;
                loop3 = true;
            }

            if(timerOne.milliseconds() > 490 && loop3){

                loop3 = false;
                loop4 = true;

                if(order[1].equals(one) && !oneShot){
                    kickerOne.setPosition(KICKER_ONE_UP);
                    oneShot = true;
                    oneKick = true;
                }else if(order[1].equals(two) && !twoShot){
                    kickerTwo.setPosition(KICKER_TWO_UP);
                    twoShot = true;
                    twoKick = true;
                }else{
                    kickerThree.setPosition(KICKER_THREE_UP);
                    threeShot = true;
                    threeKick = true;
                }
            }

            if(timerOne.milliseconds() > 780 && loop4){
                if(oneKick){
                    kickerOne.setPosition(KICKER_ONE_DOWN);
                    oneKick = false;
                }else if(twoKick){
                    kickerTwo.setPosition(KICKER_TWO_DOWN);
                    twoKick = false;
                }else{
                    kickerThree.setPosition(KICKER_THREE_DOWN);
                    threeKick = false;
                }
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 980 && loop5){
                if(oneShot && twoShot){
                    kickerThree.setPosition(KICKER_THREE_UP);
                    threeKick = true;
                }else if(oneShot && threeShot){
                    kickerTwo.setPosition(KICKER_TWO_UP);
                    twoKick = true;
                }else{
                    kickerOne.setPosition(KICKER_ONE_UP);
                    oneKick = true;
                }
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 1280 && loop6){
                if(oneKick){
                    kickerOne.setPosition(KICKER_ONE_DOWN);
                    oneKick = false;
                }else if(twoKick){
                    kickerTwo.setPosition(KICKER_TWO_DOWN);
                    twoKick = false;
                }else{
                    kickerThree.setPosition(KICKER_THREE_DOWN);
                    threeKick = false;
                }
                loop6 = false;
            }
        }
        kickThree = true;

        oneShot = false;
        twoShot = false;
        threeShot = false;

        oneKick = false;
        twoKick = false;
        threeKick = false;

        shooterOne.setPower(0);
        shooterTwo.setPower(0);
        ChaseTheCarrotConstantHeadingWithShooter(park, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1);

        sleep(500);





    }
}

