package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

public class Turret {
    CRServo rotateOne, rotateTwo;

    AnalogInput rotOneEncoder, rotTwoEncoder;

    Limelight3A cam;

    public Turret(CRServo rotateOne, CRServo rotateTwo, AnalogInput rotOneEncoder, AnalogInput rotTwoEncoder){
        this.rotateOne = rotateOne;
        this.rotateTwo = rotateTwo;

        this.rotOneEncoder = rotOneEncoder;
        this.rotTwoEncoder = rotTwoEncoder;



    }

    public Turret(CRServo rotateOne, CRServo rotateTwo){
        this.rotateOne = rotateOne;
        this.rotateTwo = rotateTwo;
    }






}
