package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.Dependencies.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Dependencies.AutoCommands;

import java.time.chrono.MinguoChronology;

@Autonomous(name="Autonomous")
public class AutoControlled extends LinearOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight, shooter, intake;
    public VoltageSensor voltageSensor;
    public UGRectDetector vision;

    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = new Motor(hardwareMap, "fL");
        frontLeft = new Motor(hardwareMap, "fR");
        backRight = new Motor(hardwareMap, "bL");
        backLeft = new Motor(hardwareMap, "bR");
        shooter = new Motor(hardwareMap, "shooter");
        intake = new Motor(hardwareMap, "intake");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        vision = new UGRectDetector(hardwareMap, "camera");
        vision.init();
        vision.setTopRectangle(0.1, 0.22890625);
        vision.setBottomRectangle(0.20555555555555555, 0.2328125);
        vision.setRectangleSize(200, 42);

        AutoCommands robot = new AutoCommands(frontLeft, frontRight, backLeft, backRight, shooter, intake, voltageSensor);
        robot.initialize();

        waitForStart();

        /*robot.resetEncoders();
        robot.setTarget(500);
        robot.setTolerance(100);
        robot.navigate(0.3, false, false, false);

        robot.resetEncoders();
        robot.setTarget(-500);
        robot.setTolerance(100);
        robot.navigate(0.3, true, false, false);

        robot.resetEncoders();
        robot.setTarget(-500, 500, 500, -500);
        robot.setTolerance(100);
        robot.navigate(0.5, false, true, false);
        */

        long startTime = System.currentTimeMillis();
        int visionDec = 0;
        while((System.currentTimeMillis()-startTime)<4500) {
            telemetry.addData("Vision", vision.getStack());
            telemetry.update();
            visionDec = vision.getStack();
        }

        if (visionDec == 4){
            telemetry.addData("Path", 4);
            telemetry.update();
        }

        if (visionDec == 0){
            telemetry.addData("Path", 0);
            telemetry.update();
        }

        if (visionDec == 1){
            telemetry.addData("Path", 1);
            telemetry.update();
        }
//random comment
        //robot.resetDriveTrainEncoders();
        //robot.setTargetInches(24);
        //robot.setTolerance(0);
        //robot.navigate(0.3, false, false, false);

        //robot.resetDriveTrainEncoders();
        //robot.setTargetInches(-24, 24, 24, -24, true);
        //robot.setTolerance(0);
        //robot.navigate(0.1, false, true, false);

        //robot.resetDriveTrainEncoders();
        //robot.setTargetInches(-24);
        //robot.setTolerance(0);
        //robot.navigate(0.3, true, false, false);

        robot.resetDriveTrainEncoders();
        robot.setTargetInches(9*Math.PI, 9*-Math.PI, 9*Math.PI, 9*-Math.PI, false);
        robot.setTolerance(0);
        robot.navigate(0.3, false, false, false, false, true);


        //test rotate
        robot.stopMotors();

    }
}
