package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Point;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name="Red Auto")
public class T1_Auto_Red extends Base{
    ArrayList<Point> shot1, firstIntake, secondIntake, thirdIntake;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(hardwareMap);

        shot1 = new ArrayList<>();
        shot1.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-15, -10, 0, 0.7)
                        )
                )
        );

        firstIntake = new ArrayList<>();
        firstIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-15, -45, 0, 0.8),
                                new Point(4, -45, 0, 0.6)
                        )
                )
        );

        secondIntake = new ArrayList<>();
        secondIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-15, -68, 0, 0.8),
                                new Point(5.5, -68, 0, 0.6)
                        )
                )
        );

        thirdIntake = new ArrayList<>();
        thirdIntake.addAll(
                new ArrayList<>(
                        Arrays.asList(
                                new Point(-15, -91, 0, 0.8),
                                new Point(5.5, -91, 0, 0.6)
                        )
                )
        );

        waitForStart();

        ChaseTheCarrotConstantHeading(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.8);
        sleep(500);
        firstIntake = PathGenerator.interpSplinePath(firstIntake, new Point(getX(), getY(), 0));


        ChaseTheCarrotConstantHeading(firstIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 3000, 0.9);
        sleep(500);
        ChaseTheCarrotConstantHeading(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.9);
        sleep(500);
        secondIntake = PathGenerator.interpSplinePath(secondIntake, new Point(getX(), getY(), 0));
        ChaseTheCarrotConstantHeading(secondIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 3000, 0.9);
        sleep(500);
        ChaseTheCarrotConstantHeading(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.8);
        sleep(500);

        thirdIntake = PathGenerator.interpSplinePath(thirdIntake, new Point(getX(), getY(), 0));
        ChaseTheCarrotConstantHeading(thirdIntake, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 3000, 0.9);

        sleep(500);
        ChaseTheCarrotConstantHeading(shot1, 9, 3, 0, 3, 1, 0.05, 0.05, 0.03, 0.0005, 0, 2500, 0.9);
        sleep(500);



    }
}
