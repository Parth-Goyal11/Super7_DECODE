package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.Motor;

@Config
@TeleOp(name="Shooter Test")
public class DriveMotorConfig extends LinearOpMode {
    public static double kv = 0.000543;

    public static double kp = 0.015;
    public static double ki = 0;
    public static double kd = 0;
    public static double targetVelocity = 1350;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooter, shooterTwo;

        shooter = hardwareMap.get(DcMotorEx.class, "shooterOne");
        shooterTwo = hardwareMap.get(DcMotorEx.class, "shooterTwo");

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo angleLeft, angleRight;

        angleLeft = hardwareMap.servo.get( "angleLeft");
        angleRight = hardwareMap.servo.get("angleRight");

        angleLeft.setPosition(0.7478);
        angleRight.setPosition(0.2339);






       boolean changeLast = false, changeCurr = false;

       boolean incLast = false, incCurr = false;
       boolean shooterMode = false;
       double TPS = 0;
       double error = 0;

       dashboard = FtcDashboard.getInstance();
       Telemetry dashboardTelemetry = dashboard.getTelemetry();

       waitForStart();

       while(opModeIsActive()){

           if(gamepad1.a){
               shooterMode = true;
           }

           if(gamepad1.b){
               shooterMode = false;
           }

           if(shooterMode){
               TPS = shooter.getVelocity();

               error = targetVelocity - shooter.getVelocity();

               double p = kp * error;
               double feedforward = kv * targetVelocity;

               double pow = p + feedforward;

               shooter.setPower(pow);
               shooterTwo.setPower(-pow);
           }else{
               shooter.setPower(0);
               shooterTwo.setPower(0);
           }

           if(gamepad1.dpad_up){
               angleLeft.setPosition(angleLeft.getPosition() + 0.002);
               angleRight.setPosition(angleRight.getPosition() - 0.002);
           }else if(gamepad1.dpad_down){
               angleLeft.setPosition(angleLeft.getPosition() - 0.002);
               angleRight.setPosition(angleRight.getPosition() + 0.002);
           }


           dashboardTelemetry.addData("Shooter Velo", TPS);
           dashboardTelemetry.addData("Target Velocity", targetVelocity);
           dashboardTelemetry.addData("Error", error);
           dashboardTelemetry.update();

           telemetry.addData("Angle Left", angleLeft.getPosition());
           telemetry.addData("Angle Right", angleRight.getPosition());
           telemetry.update();








       }















    }
}
