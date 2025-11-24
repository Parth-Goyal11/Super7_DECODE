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
import org.firstinspires.ftc.teamcode.Modules.Indexer;

import java.util.Locale;


@TeleOp(name="Drive Test")
public class BasicDriveTest extends Base {




    @Override
    public void runOpMode() throws InterruptedException {
           initHardware(hardwareMap);
           ElapsedTime timer = new ElapsedTime();
           String[] order = {"Purple", "Green", "Purple"};
           Indexer indexer = new Indexer(slotOne, slotTwo, slotThree, kickerOne, kickerTwo, kickerThree, order, timer);
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
        double powerCap = 1;
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



            driveFieldCentric(drive, turn, strafe, powerCap);









            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();







        }





    }
}

