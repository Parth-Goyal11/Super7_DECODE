package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Core.Motor;

public class Shooter {
    public static double kv = 0.000543;

    public static double kp = 0.015;
    public static double ki = 0;
    public static double kd = 0;

    double TPS = 0;
    double error = 0;

    Motor shooterOne, shooterTwo;

    public Shooter(Motor shooterOne, Motor shooterTwo){
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
    }

    public void toSpeed(double targetVelocity){
        TPS = shooterOne.retMotorEx().getVelocity();

        error = targetVelocity - shooterOne.retMotorEx().getVelocity();

        double p = kp * error;
        double feedforward = kv * targetVelocity;

        double pow = p + feedforward;

        shooterOne.setPower(pow);
        shooterTwo.setPower(-pow);
    }

    public void stopShooter(){
        shooterOne.setPower(0);
        shooterTwo.setPower(0);
    }
}
