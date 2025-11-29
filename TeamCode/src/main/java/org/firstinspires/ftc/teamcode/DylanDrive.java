package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Core.Motor;

@TeleOp(name="Dylan Drive")
public class DylanDrive extends LinearOpMode {
    Motor fLeft, fRight, bLeft, bRight;

    @Override
    public void runOpMode() throws InterruptedException {
        fLeft = new Motor(hardwareMap, "leftOne");
        bLeft = new Motor(hardwareMap, "leftTwo");
        fRight = new Motor(hardwareMap, "rightOne");
        bRight = new Motor(hardwareMap, "rightTwo");

        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");

        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            double powOne = -gamepad1.left_stick_y;
            double powTwo = -gamepad1.right_stick_y;

            fLeft.setPower(powOne);
            bLeft.setPower(powOne);

            fRight.setPower(powTwo);
            bRight.setPower(powTwo);

            colorSensor.getNormalizedColors();
        }
    }
}
