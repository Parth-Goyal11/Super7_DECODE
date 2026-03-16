package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Motor;
@Disabled
@TeleOp(name="Shooter Test(3 in wheel)")
public class ShooterTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Motor shooterOne = new Motor(hardwareMap, "shooterOne");
        Motor shooterTwo = new Motor(hardwareMap, "shooterTwo");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                shooterOne.setPower(0.9);
                shooterTwo.setPower(-0.9);
            }else if(gamepad1.dpad_down){
                shooterOne.setPower(-0.9);
                shooterTwo.setPower(0.9);
            }else{
                shooterOne.setPower(0);
                shooterTwo.setPower(0);
            }
        }
    }
}
