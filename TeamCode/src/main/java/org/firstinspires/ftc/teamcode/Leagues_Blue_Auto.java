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

@Autonomous(name="Blue Leagues Solo Auto")
public class Leagues_Blue_Auto extends Base{

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





        double KICKER_ONE_DOWN = 0.042, KICKER_ONE_UP = 1, KICKER_TWO_DOWN = 0.98, KICKER_TWO_UP = 0.18,
                KICKER_THREE_DOWN = 0.065, KICKER_THREE_UP = 1;



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
        rotOne.setPosition(0.9);
        rotTwo.setPosition(0.9);

        shot1 = new ArrayList<>();
        shot1.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(29, -36, 0, 0.78)
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
                                new Point(31, -16, 0, 0.8)
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
                                new Point(1, -52.5, 0, 0.8),
                                new Point(-23, -55.5, 0, 1)
                        )
                )
        );

        secondIntake = new ArrayList<>();
        secondIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(20, -70, 0, 1),
                                new Point(-31, -70 , 0, 0.3)
                        )
                )
        );



        thirdIntake = new ArrayList<>();
        thirdIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(17.5, -92.5, 0, 1),
                                new Point(-16, -92.5, 0, 0.3)
                        )
                )
        );
//        while(opModeInInit()){
//            LLResult result = limelight.getLatestResult();
//            if (result != null) {
//                resetCache();
//                if (result.isValid()) {
//
//                    int tagIDR = result.getFiducialResults().get(0).getFiducialId();
//                    Pose3D botpose = result.getBotpose();
//                    telemetry.addData("ID", tagIDR);
//                    telemetry.update();
//                    counter = tagIDR;
//
//                }else{
//
//
//                    telemetry.addData("So", "Cooked");
//
//                    telemetry.update();
//                }
//            }else{
//                telemetry.addData("Result", "Bruh");
//                telemetry.update();
//            }
//        }















        waitForStart();



        hoodOne.setPosition(0.8911);
        hoodTwo.setPosition(1-0.8911);


        ChaseTheCarrotConstantHeadingWithShooter(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1);

        timerOne.reset();
        while(timerOne.milliseconds() < 1350){

            resetCache();

            runShooter(1300);
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

            if(timerOne.milliseconds() > 510 && loop3){
                kickerOne.setPosition(KICKER_ONE_UP);
                loop3 = false;
                loop4 = true;
            }

            if(timerOne.milliseconds() > 800 && loop4){
                kickerOne.setPosition(KICKER_ONE_DOWN);
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 1000 && loop5){
                kickerTwo.setPosition(KICKER_TWO_UP);
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 1300 && loop6){
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

        rotOne.setPosition(0.55);
        rotTwo.setPosition(0.55);
        sleep(400);



        firstIntake = PathGenerator.interpSplinePath(firstIntake, new Point(getX(), getY(), 0));

        ChaseTheCarrotConstantHeadingWithShooter(firstIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 1);
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

        frontSweeper.setPower(0);
        backSweeper.setPower(0);

        gate = PathGenerator.interpSplinePath(gate, new Point(getX(), getY(), 0));
        ChaseTheCarrotConstantHeadingWithShooter(gate, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 1400, 1);
        sleep(550);

        rotOne.setPosition(0.875 );
        rotTwo.setPosition(0.875);









        ChaseTheCarrotConstantHeadingWithShooter(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.55);




        String one = oneSlot, two = twoSlot, three = threeSlot;
        timerOne.reset();
        while(timerOne.milliseconds() < 1350){

            resetCache();

            runShooter(1300);
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

            if(timerOne.milliseconds() > 510 && loop3){
                kickerOne.setPosition(KICKER_ONE_UP);
                loop3 = false;
                loop4 = true;
            }

            if(timerOne.milliseconds() > 800 && loop4){
                kickerOne.setPosition(KICKER_ONE_DOWN);
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 1000 && loop5){
                kickerTwo.setPosition(KICKER_TWO_UP);
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 1300 && loop6){
                kickerTwo.setPosition(KICKER_TWO_DOWN);
                loop6 = false;
            }
        }
        kickThree = true;





        shooterOne.setPower(0);
        shooterTwo.setPower(0);

        backSweeper.setPower(-1);


        secondIntake = PathGenerator.interpSplinePath(secondIntake, new Point(getX(), getY(), 0));
        ChaseTheCarrotConstantHeading(secondIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 3000, 1);


        frontSweeper.setPower(0);
        backSweeper.setPower(0);



//        gate = PathGenerator.interpSplinePath(gate, new Point(getX(), getY(), 0));
//        ChaseTheCarrotConstantHeading(gate, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2300, 0.9);
//        sleep(1000);



        shotBruh = PathGenerator.interpSplinePath(shotBruh, new Point(getX(), getY(), 0));

        ChaseTheCarrotConstantHeadingWithColor(shotBruh, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500);


        one = oneSlot;
        two = twoSlot;
        three = threeSlot;
        timerOne.reset();
        while(timerOne.milliseconds() < 1350){

            resetCache();

            runShooter(1300);
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

            if(timerOne.milliseconds() > 510 && loop3){
                kickerOne.setPosition(KICKER_ONE_UP);
                loop3 = false;
                loop4 = true;
            }

            if(timerOne.milliseconds() > 800 && loop4){
                kickerOne.setPosition(KICKER_ONE_DOWN);
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 1000 && loop5){
                kickerTwo.setPosition(KICKER_TWO_UP);
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 1300 && loop6){
                kickerTwo.setPosition(KICKER_TWO_DOWN);
                loop6 = false;
            }
        }
        kickThree = true;


        shooterOne.setPower(0);
        shooterTwo.setPower(0);




        backSweeper.setPower(-1);



        thirdIntake = PathGenerator.interpSplinePath(thirdIntake, new Point(getX(), getY(), 0));
        ChaseTheCarrotConstantHeading(thirdIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 7000, 1);


        frontSweeper.setPower(0);
        backSweeper.setPower(0);

//        ChaseTheCarrotConstantHeading(bang2, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 1250 , 1);





        ChaseTheCarrotConstantHeadingWithColor(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500);


        one = oneSlot;
        two = twoSlot;
        three = threeSlot;
        timerOne.reset();
        while(timerOne.milliseconds() < 1350){

            resetCache();

            runShooter(1300);
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

            if(timerOne.milliseconds() > 510 && loop3){
                kickerOne.setPosition(KICKER_ONE_UP);
                loop3 = false;
                loop4 = true;
            }

            if(timerOne.milliseconds() > 800 && loop4){
                kickerOne.setPosition(KICKER_ONE_DOWN);
                loop4 = false;
                loop5 = true;
            }

            if(timerOne.milliseconds() > 1000 && loop5){
                kickerTwo.setPosition(KICKER_TWO_UP);
                loop5 = false;
                loop6 = true;
            }

            if(timerOne.milliseconds() > 1300 && loop6){
                kickerTwo.setPosition(KICKER_TWO_DOWN);
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
        shooterOne.setPower(0);
        shooterTwo.setPower(0);
        sleep(500);





    }
}

