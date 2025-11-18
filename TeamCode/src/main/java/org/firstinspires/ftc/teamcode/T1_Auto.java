package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="T1_Auto")
public class T1_Auto extends Base{
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(hardwareMap);
        waitForStart();
        moveToPosition(20, -10, -45, 3, 2, 0.7,  10000);
        sleep(500);
        stopDrive();
        moveToPosition(40, -10, -90, 3, 2, 0.6, 10000);
        sleep(500);
        moveToPosition(40, 15, -90, 3, 2, 0.55, 10000);
        sleep(500);
        moveToPosition(20, -10, -45, 3, 2, 0.7, 10000);
        sleep(500);







//        turnTo(-128, 10000, 0.1, 1);
//        stopDrive();
//        Pose2D newPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES,0);
//        odo.setPosition(newPos);
//        sleep(1200);
//        moveToPosition(-3, 16.5, 0, 3, 3, 3000);
//        sleep(500);
//
//        timer.reset();
//        while(timer.milliseconds() < 1500){
//            odo.update();
//            resetCache();
//            driveFieldCentric(-0.5, 0, 0, 0.1);
//        }
//        stopDrive();




    }
}
