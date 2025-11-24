package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.CRServo;

public class Turret {
    CRServo rotateOne, rotateTwo;

    public Turret(CRServo rotateOne, CRServo rotateTwo){
        this.rotateOne = rotateOne;
        this.rotateTwo = rotateTwo;
    }


}
