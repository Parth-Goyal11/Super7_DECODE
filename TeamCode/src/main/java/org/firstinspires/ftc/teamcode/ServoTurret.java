package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

public class ServoTurret extends LinearOpMode {
    CRServo rot1, rot2;


    @Override
    public void runOpMode() throws InterruptedException {
        rot1 = hardwareMap.get(CRServo.class, "rotation1");
        rot2 = hardwareMap.get(CRServo.class, "rotation2");

        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "testInput");
        double position = analogInput.getVoltage() / 3.3 * 360;
        boolean notAligned = true;

        if(notAligned){
            align();
        }else{
            System.out.println("WOOWHOWOWOWHo0");
        }
    }

    public static void align(){

    }
}
