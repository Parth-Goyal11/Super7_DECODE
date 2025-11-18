package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.Motor;

@TeleOp(name="testYeAHWOWO")
public class EncoderDirectionTest extends LinearOpMode {
    Motor shooter1, shooter2;


    @Override
    public void runOpMode() throws InterruptedException {
        shooter1 = new Motor(hardwareMap, "shooterOne");
        shooter2 = new Motor(hardwareMap, "shooterTwo");
        shooter1.useEncoder();
        shooter2.useEncoder();

        boolean yuhLast = false, yuhCurr = false;
        boolean incLast = false, incCurr = false;

        waitForStart();

        double conversionConstant = (60.0)/(28.0);
        double power = 2400 / conversionConstant;
        double targetVelocity = 1200;


        while(opModeIsActive()){

            if(gamepad1.dpad_up){
                //2400 RPM
                shooter1.retMotorEx().setVelocity(targetVelocity);
                shooter2.retMotorEx().setVelocity(-targetVelocity);
            }else if(gamepad1.dpad_down){
                shooter1.retMotorEx().setVelocity(-targetVelocity);
                shooter2.retMotorEx().setVelocity(targetVelocity);
            }else{
                shooter1.setPower(0);
                shooter2.setPower(0);
            }

            yuhLast = yuhCurr;
            yuhCurr = gamepad1.a;
            if(yuhCurr && !yuhLast){
                targetVelocity += 100;
            }

            incLast = incCurr;
            incCurr = gamepad1.b;
            if(incCurr && !incLast){
                targetVelocity -= 100;
            }


            telemetry.addData("Encoder Reading", shooter1.encoderReading());
            telemetry.addData("Encoder 2", shooter2.encoderReading());
            telemetry.addData("Velocity", shooter1.retMotorEx().getVelocity());
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.update();

        }
    }
}
