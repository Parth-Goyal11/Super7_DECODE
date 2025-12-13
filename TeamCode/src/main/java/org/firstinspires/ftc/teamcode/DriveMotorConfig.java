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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Core.Motor;

@Config
@TeleOp(name="Test Shooting")
public class DriveMotorConfig extends LinearOpMode {
    public static double kv = 0.000543;

    public static double kp = 0.015;
    public static double ki = 0;
    public static double kd = 0;
    public static double targetVelocity = 1400;

    boolean upLast = false, upCurr = false;
    boolean downLast = false, downCurr = false;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooter, shooterTwo;



        boolean triggerOneLast = false, triggerOneCurr = false, triggerTwoLast = false, triggerTwoCurr = false,
                triggerThreeLast = false, triggerThreeCurr = false;
        double KICKER_ONE_DOWN = 0.1372, KICKER_ONE_UP = 0.6583, KICKER_TWO_DOWN = 0.88, KICKER_TWO_UP = 0.6122,
                KICKER_THREE_DOWN = 0.625, KICKER_THREE_UP = 0.89;

        shooter = hardwareMap.get(DcMotorEx.class, "shooterOne");
        shooterTwo = hardwareMap.get(DcMotorEx.class, "shooterTwo");

//        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        shooterTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        Servo angleLeft, angleRight;
//
//        angleLeft = hardwareMap.servo.get( "angleLeft");
//        angleRight = hardwareMap.servo.get("angleRight");
//
//        kickerOne.setPosition(KICKER_ONE_DOWN);
//        kickerTwo.setPosition(KICKER_TWO_DOWN);
//        kickerThree.setPosition(KICKER_THREE_DOWN);








       boolean changeLast = false, changeCurr = false;

       boolean incLast = false, incCurr = false;
       boolean shooterMode = false;
       double TPS = 0;
       double error = 0;
       double power = 0;

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
               power = pow;

           }else{
               shooter.setPower(0);
               shooterTwo.setPower(0);
               power = 0;
           }

           downLast = downCurr;
           downCurr = gamepad1.dpad_down;
           if(downCurr && !downLast){
               targetVelocity -= 50;
           }

           upLast = upCurr;
           upCurr = gamepad1.dpad_up;
           if(upCurr && !upLast){
               targetVelocity += 50;
           }






           dashboardTelemetry.addData("Shooter Velo", shooter.getVelocity());
           telemetry.addData("Motor Current", shooter.getCurrent(CurrentUnit.MILLIAMPS));
           dashboardTelemetry.addData("Target Velocity", targetVelocity);
           dashboardTelemetry.addData("Error", error);
           telemetry.addData("Pow", power);
           telemetry.update();
           dashboardTelemetry.update();











       }















    }
}
