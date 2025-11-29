package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Modules.Indexer;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Shooter;
import org.firstinspires.ftc.teamcode.Modules.Turret;

public class T2_Tele extends Base{
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(hardwareMap);

        //Robot Modules
        Shooter shooter = new Shooter(shooterOne, shooterTwo);
        Intake intake = new Intake(frontSweeper);
        Turret turret = new Turret(rotOne, rotTwo, turretTrack);
        Indexer indexer = new Indexer(kickerOne, kickerTwo);

        //Status Booleans
        boolean shooterOn = false;

        //Op-Mode Values
        double targetVelocity = 1350;
        double powerCap = 0.9;
        final double TRIGGER_THRESHOLD = 0.05;

        //Button Variables
        boolean shooterLast = false, shooterCurr = false;

        waitForStart();

        while(opModeIsActive()){

            //Field-Centric Driving
            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x ;
            double strafe = -gamepad1.left_stick_x;

            driveFieldCentric(drive, turn, strafe, powerCap);

            if(shooterOn){
                shooter.toSpeed(targetVelocity);
            }else{
                shooter.stopShooter();
            }

            //Manual Turret Control
            if(gamepad1.dpad_left){
                turret.rotate();
            }else if(gamepad1.dpad_right){
                turret.rotateReverse();
            }else{
                turret.stop();
            }

            //Intake Control
            if(gamepad1.right_trigger > TRIGGER_THRESHOLD){
                intake.intakeFront();
            }else if(gamepad1.left_trigger > TRIGGER_THRESHOLD){
                intake.intakeOutFront();
            }else{
                intake.stopFront();
            }

            //Update Shooter Status
            shooterLast = shooterCurr;
            shooterCurr = gamepad1.left_bumper;
            if(shooterCurr && !shooterLast){
                shooterOn = !shooterOn;
            }



            if(gamepad1.a){
                indexer.actuateKickerOne();
            }

            if(gamepad1.b){
                indexer.actuateKickerTwo();
            }


            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.addData("Turret Position", turret.getDegree());
            telemetry.update();

        }

    }
}
