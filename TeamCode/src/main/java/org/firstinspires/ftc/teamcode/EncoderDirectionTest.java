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
    Servo kickerOne, kickerTwo, kickerThree;


    @Override
    public void runOpMode() throws InterruptedException {
         double kv = 0.000543;

        double kp = 0.015;
        double ki = 0;
        double kd = 0;
        double targetVelocity = 1350;

        kickerOne = hardwareMap.servo.get("kickIntake");
        kickerTwo = hardwareMap.servo.get("kickOut");
        kickerThree = hardwareMap.servo.get("kickMiddle");

//        rotOne = hardwareMap.get(CRServo.class, "rotOne");
//        rotTwo = hardwareMap.get(CRServo.class, "rotTwo");
//
//        AnalogInput firstAnalog = hardwareMap.get(AnalogInput.class, "firstTurret");
//        boolean yuhLast = false, yuhCurr = false;
        boolean incLast = false, incCurr = false;



        CRServo rotOne = hardwareMap.get(CRServo.class, "rotOne");
        CRServo rotTwo = hardwareMap.get(CRServo.class, "rotTwo");

        Motor shooter = new Motor(hardwareMap, "shooterOne");
        Motor shooterTwo = new Motor(hardwareMap, "shooterTwo");
        boolean shooterMode = false;

        boolean triggerOneLast = false, triggerOneCurr = false, triggerTwoLast = false, triggerTwoCurr = false,
                triggerThreeLast = false, triggerThreeCurr = false;
        double KICKER_ONE_DOWN = 0.1372, KICKER_ONE_UP = 0.6583, KICKER_TWO_DOWN = 0.875, KICKER_TWO_UP = 0.6122,
                KICKER_THREE_DOWN = 0.625, KICKER_THREE_UP = 0.89;


        String status = "";
        kickerOne.setPosition(KICKER_ONE_DOWN);
        kickerTwo.setPosition(KICKER_TWO_DOWN);
        kickerThree.setPosition(KICKER_THREE_DOWN);


        double conversionConstant = (60.0)/(28.0);
        double power = 2400 / conversionConstant;
        double TPS = 0;
        double error = 0;

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












            telemetry.update();

        }
    }
}
