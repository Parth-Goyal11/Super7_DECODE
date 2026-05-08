package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Worlds Red Tele")
public class Worlds_Red_Tele extends Base{

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(hardwareMap);

        //Drive Vars
        double powerCap = 1;

        waitForStart();

        initializeKickers();

        while(opModeIsActive()){

            double drive = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            driveFieldCentric(drive, turn, strafe, 1);
        }

    }
}
