package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Core.Motor;

public abstract class Intake extends Base{
    public Motor sweeper;
    public Servo slotOne, slotTwo, slotThree;
    public RevColorSensorV3 colorOne, colorTwo, colorThree;

    public Intake(Motor sweeper, Servo slotOne, Servo slotTwo, Servo slotThree, RevColorSensorV3 colorOne, RevColorSensorV3 colorTwo, RevColorSensorV3 colorThree){
        this.sweeper = sweeper;
        this.slotOne = slotOne;
        this.slotTwo = slotTwo;
        this.slotThree = slotThree;
        this.colorOne = colorOne;
        this.colorTwo = colorTwo;
        this.colorThree = colorThree;
    }
    public void sweepIn(){
        sweeper.setPower(1);
    }

    public void reverseSweeper(){
        sweeper.setPower(-1);
    }

    public void shutOff(){
        sweeper.setPower(0);
    }

    boolean slotOneFull(){
        return(colorOne.getDistance(DistanceUnit.CM) < 5);
    }

    boolean slotTwoFull(){
        return(colorTwo.getDistance(DistanceUnit.CM) < 5);
    }

    boolean slotThreeFull(){
        return(colorThree.getDistance(DistanceUnit.CM) < 5);
    }



}
