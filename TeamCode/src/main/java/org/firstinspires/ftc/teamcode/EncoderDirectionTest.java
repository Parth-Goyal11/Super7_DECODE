package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Core.Motor;

@TeleOp(name="Color Detection")
public class EncoderDirectionTest extends LinearOpMode {
    CRServo rotOne, rotTwo;


    @Override
    public void runOpMode() throws InterruptedException {
         double kv = 0.000543;

        double kp = 0.015;
        double ki = 0;
        double kd = 0;
        double targetVelocity = 1350;

//        rotOne = hardwareMap.get(CRServo.class, "rotOne");
//        rotTwo = hardwareMap.get(CRServo.class, "rotTwo");
//
//        AnalogInput firstAnalog = hardwareMap.get(AnalogInput.class, "firstTurret");
//        boolean yuhLast = false, yuhCurr = false;
        boolean incLast = false, incCurr = false;
        Servo kickerOne = hardwareMap.servo.get("kickerOne");
        Servo kickerTwo = hardwareMap.servo.get("kickerTwo");

        CRServo rotOne = hardwareMap.get(CRServo.class, "rotOne");
        CRServo rotTwo = hardwareMap.get(CRServo.class, "rotTwo");

        Motor shooter = new Motor(hardwareMap, "shooterOne");
        Motor shooterTwo = new Motor(hardwareMap, "shooterTwo");
        boolean shooterMode = false;


        String status = "";
        kickerOne.setPosition(0.9);
        kickerTwo.setPosition(0);


        double conversionConstant = (60.0)/(28.0);
        double power = 2400 / conversionConstant;
        double TPS = 0;
        double error = 0;

        waitForStart();



        while(opModeIsActive()){

            if(gamepad1.a){
                kickerOne.setPosition(0.2);
                sleep(300);
                kickerOne.setPosition(0.9);
            }

            if(gamepad1.b){
                kickerTwo.setPosition(0.8);
                sleep(300);
                kickerTwo.setPosition(0);
            }



            if(gamepad1.left_trigger > 0.05){
                shooterMode = true;
            }

            if(gamepad1.right_trigger > 0.05){
                shooterMode = false;
            }

            if(shooterMode){
                TPS = shooter.retMotorEx().getVelocity();

                error = targetVelocity - shooter.retMotorEx().getVelocity();

                double p = kp * error;
                double feedforward = kv * targetVelocity;

                double pow = p + feedforward;

                shooter.setPower(pow);
                shooterTwo.setPower(-pow);
            }else{
                shooter.setPower(0);
                shooterTwo.setPower(0);
            }

            if(gamepad1.dpad_left){
                rotOne.setPower(1);
                rotTwo.setPower(1);
            }else if(gamepad1.dpad_right){
                rotOne.setPower(-1);
                rotTwo.setPower(-1);
            }else{
                rotOne.setPower(0);
                rotTwo.setPower(0);
            }








            telemetry.addData("Kicker One", kickerOne.getPosition());
            telemetry.addData("Kicker Two", kickerTwo.getPosition());
            telemetry.update();

        }
    }
}
