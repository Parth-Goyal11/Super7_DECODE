package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Auto")
public class T1_Auto_Red extends Base{

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(hardwareMap);
        stop.setPosition(0.072);
        angleLeft.setPosition(0.721); //.721
        angleRight.setPosition(0.2607);
        ElapsedTime timer = new ElapsedTime();
        resetCache();
        sleep(1000);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        shooterOne.setPower(-0.8);
        shooterTwo.setPower(0.8);

        stopDrive();



        sleep(500);
        indexer.setPosition(0.99);
        sleep(270);
        kick.setPosition(0.2967);
        sleep(250);
        kick.setPosition(0.63);
        indexer.setPosition(0);
        sleep(400);

        wheelOne.setPower(1);  //Intake In
        wheelTwo.setPower(-1);
        sleep(1050);
        wheelOne.setPower(0);  //Intake In
        wheelTwo.setPower(0);
        sleep(500);
        indexer.setPosition(0.99);
        sleep(270);
        kick.setPosition(0.2967);
        sleep(250);
        kick.setPosition(0.63);
        indexer.setPosition(0);
        sleep(400);

        sweeper.setPower(-1);

        sleep(1400);
        sweeper.setPower(0);
        wheelOne.setPower(0);
        wheelTwo.setPower(0);
        sleep(500);
        timer.reset();

        fLeft.setPower(-0.4);
        fRight.setPower(0.4);
        bLeft.setPower(0.4);
        bRight.setPower(-0.4);
        sleep(350);
        stopDrive();
    }
}
