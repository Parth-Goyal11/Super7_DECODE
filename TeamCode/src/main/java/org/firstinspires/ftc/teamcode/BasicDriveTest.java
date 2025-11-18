package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Core.Motor;

import java.util.Locale;


@TeleOp(name="Drive Test")
public class BasicDriveTest extends Base {




    @Override
    public void runOpMode() throws InterruptedException {
           initHardware(hardwareMap);
           resetCache();


           double fLeftPow = 0, fRightPow = 0, bLeftPow = 0, bRightPow = 0;





        boolean changeUpLast = false, changeUpCurr = false;
        boolean changeDownLast = false, changeDownCurr = false;
        boolean pushCurr = false, pushLast = false;
        boolean feederOff = false, wheelsOff = false;
        boolean shooterLast = false, shooterCurr = false;
        boolean shoot = false;
        boolean intake = false;
        boolean canPush = false;
        ElapsedTime pushBall = new ElapsedTime();
        ElapsedTime reset = new ElapsedTime();
        boolean canReset = false;
        boolean indexerFlipped = false;
        double powerCap = 0.5;
        boolean shooterOn = false;


//        double power = 1;
        double shooterPow = 1;



        waitForStart();


        while(opModeIsActive()){

            resetCache();

            odo.update();//7.5 in vertical to strafe
                         //1 in horizontal to strafe

            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x ;
            double strafe = -gamepad1.left_stick_x;

            fLeftPow = drive + turn + strafe;
            fRightPow = drive - turn - strafe;
            bLeftPow = drive + turn - strafe;
            bRightPow = drive - turn + strafe;

//            fLeft.setPower(fLeftPow * 0.2);
//            fRight.setPower(fRightPow * 0.2);
//            bLeft.setPower(bLeftPow * 0.2);
//            bRight.setPower(bRightPow * 0.2);

            driveFieldCentric(drive, turn, strafe, powerCap);




//            shooterLast = shooterCurr;
//            shooterCurr = gamepad2.right_bumper;
//            if(shooterCurr && !shooterLast){
//                shooterOn=!shooterOn;
//                if(shooterOn){
//                    shooterOne.setPower(-0.9);
//                    shooterTwo.setPower(0.9);
//                }else{
//                    shooterOne.setPower(0);
//                    shooterTwo.setPower(0);
//                }
//            }
//
//            if(gamepad2.dpad_up){
//                intake = true;
//                wheelOne.setPower(-1);  //Intake In
//                wheelTwo.setPower(1);
//
//            }else if(gamepad2.dpad_down){
//                intake = true;
//                wheelOne.setPower(1);
//                wheelTwo.setPower(-1);
//
//            }else{
//
//                intake = false;
//                if(wheelsOff){
//                    wheelOne.setPower(0);
//                    wheelTwo.setPower(0);
//                }
//
//
//
//
//
//            }
//
//            if(gamepad1.right_trigger > 0.05 || gamepad2.right_trigger > 0.05){
//                feederOff = false;
//                wheelsOff = false;
//                stop.setPosition(0.453);
//                sweeper.setPower(-1);
//
//                if(!indexerFlipped){
//                    wheelOne.setPower(-1);  //Intake In
//                    wheelTwo.setPower(1);
//                }else{
//                    wheelOne.setPower(1);
//                    wheelTwo.setPower(-1);
//                }
//
//
//            }else if(gamepad1.left_trigger > 0.05 || gamepad2.left_trigger > 0.05){
//                sweeper.setPower(0.5);
//                feederOff = true;
//                wheelsOff = true;
//            }else{
//                feederOff = true;
//                wheelsOff = true;
//                sweeper.setPower(0);
//
//
//            }
//
//            pushLast = pushCurr;
//            pushCurr = gamepad2.a;
//            if(pushCurr && !pushLast){
//                indexer.setPosition(0.99);
//                pushBall.reset();
//                canPush = true;
//            }
//
//            if(pushBall.milliseconds() > 270 && canPush){
//                canPush = false;
//                kick.setPosition(0.2967);
//                canReset = true;
//                reset.reset();
//                //Decrease
//            }
//
//            if(reset.milliseconds() > 250 && canReset){
//                kick.setPosition(0.63);
//                indexer.setPosition(0);
//                canReset = false;
//
//            }



//            if(gamepad2.dpad_left){
//                stop.setPosition(0.072);
//                sleep(300);
//                spin.setPosition(0.8);
//                indexerFlipped = true;
//            }else if(gamepad2.dpad_right){
//                stop.setPosition(0.072);
//                sleep(300);
//                spin.setPosition(0.097);
//                indexerFlipped = false;
//            }


//            if(gamepad2.y){
//                angleLeft.setPosition(0.8284);
//                angleRight.setPosition(0.1633);
//            }else if(gamepad2.x) {
//                angleLeft.setPosition(0.7578);
//                angleRight.setPosition(0.2339);
//            }
//
//




            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();







        }





    }
}

