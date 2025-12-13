package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Axon Test")
public class ServoTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo one = hardwareMap.servo.get("servoOne");
        Servo two = hardwareMap.servo.get("servoTwo");
        Servo three = hardwareMap.servo.get("servoThree");
        Servo four = hardwareMap.servo.get("servoFour");
        Servo five = hardwareMap.servo.get("servoFive");
        boolean bruLast = false, bruCurr = false;
        int index = 0;
        Servo[] servos = {one, two, three, four, five};
        one.setPosition(0);
        two.setPosition(0);
        three.setPosition(0);
        four.setPosition(0);
        five.setPosition(0);

        waitForStart();

        while(opModeIsActive()){
            bruLast = bruCurr;
            bruCurr = gamepad1.right_bumper;
            if(bruCurr && !bruLast){
                if(index == 4){
                    index = 0;
                }else{
                    index ++;
                }
            }

            if(gamepad1.a){
                servos[index].setPosition(0);
            }

            if(gamepad1.b){
                servos[index].setPosition(1);
            }
        }

    }
}
