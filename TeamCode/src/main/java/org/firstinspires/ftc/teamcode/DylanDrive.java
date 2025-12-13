package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Core.Motor;

@TeleOp(name="Dylan Drive")
public class DylanDrive extends LinearOpMode {

    Motor fLeft, fRight, bRight, bLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        fLeft = new Motor(hardwareMap, "leftOne");
        fRight = new Motor(hardwareMap, "rightOne");
        bLeft = new Motor(hardwareMap, "leftTwo");
        bRight = new Motor(hardwareMap, "rightTwo");


        waitForStart();

        while(opModeIsActive()){
            double driveLeft = -gamepad1.left_stick_y;
            double driveRight = gamepad1.right_stick_y;

            fLeft.setPower(driveLeft);
            bLeft.setPower(driveLeft);

            fRight.setPower(driveRight);
            bRight.setPower(driveRight);
        }
    }
}
