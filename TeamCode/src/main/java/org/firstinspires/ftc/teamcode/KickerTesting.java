package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(name="Kicker Positions")
public class KickerTesting extends LinearOpMode {

    Servo kickerOne, kickerTwo, kickerThree;

    @Override
    public void runOpMode() throws InterruptedException {
        kickerOne = hardwareMap.servo.get("kickIntake");
        kickerTwo = hardwareMap.servo.get("kickOut");
        kickerThree = hardwareMap.servo.get("kickMiddle");
        int index = 0;

        boolean triggerOneLast = false, triggerOneCurr = false, triggerTwoLast = false, triggerTwoCurr = false,
                triggerThreeLast = false, triggerThreeCurr = false;
        double KICKER_ONE_DOWN = 0.1372, KICKER_ONE_UP = 0.6583, KICKER_TWO_DOWN = 0.88, KICKER_TWO_UP = 0.6122,
                KICKER_THREE_DOWN = 0.625, KICKER_THREE_UP = 0.89;

        Servo[] kickers = {kickerOne, kickerTwo, kickerThree};
        waitForStart();
        while(opModeIsActive()){

            triggerOneLast = triggerOneCurr;
            triggerOneCurr = gamepad1.a;
            if(triggerOneCurr && !triggerOneLast){
                kickerOne.setPosition(KICKER_ONE_UP);
                sleep(500);
                kickerOne.setPosition(KICKER_ONE_DOWN);
            }

            triggerTwoLast = triggerTwoCurr;
            triggerTwoCurr = gamepad1.b;   //Shoot this one last as it gets kind of stuck
            if(triggerTwoCurr && !triggerTwoLast){
                kickerTwo.setPosition(KICKER_TWO_UP);
                sleep(500);
                kickerTwo.setPosition(KICKER_TWO_DOWN);
            }


            triggerThreeLast = triggerThreeCurr;
            triggerThreeCurr = gamepad1.x;
            if(triggerThreeCurr && !triggerThreeLast){
                kickerThree.setPosition(KICKER_THREE_UP);
                sleep(500);
                kickerThree.setPosition(KICKER_THREE_DOWN);
            }

            telemetry.addData("Kicker 1", kickerOne.getPosition());
            telemetry.addData("Kicker 2", kickerTwo.getPosition());
            telemetry.addData("Kicker 3", kickerThree.getPosition());
            telemetry.update();




        }
    }
}
