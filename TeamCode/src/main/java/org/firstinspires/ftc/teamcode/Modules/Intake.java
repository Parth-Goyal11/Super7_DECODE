package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Core.Motor;

public class Intake {
    Motor frontSweep, backSweep;

    public Intake(Motor frontSweep, Motor backSweep){
        this.frontSweep = frontSweep;
        this.backSweep = backSweep;
    }

    public void intakeBoth(){
        frontSweep.setPower(1);
        backSweep.setPower(1);
    }

    public void intakeFront(){
        frontSweep.setPower(1);
    }

    public void intakeBack(){
        backSweep.setPower(1);
    }

    public void stopBoth(){
        frontSweep.setPower(0);
        backSweep.setPower(0);
    }

    public void stopFront(){
        frontSweep.setPower(0);
    }

    public void stopBack(){
        backSweep.setPower(0);
    }
}
