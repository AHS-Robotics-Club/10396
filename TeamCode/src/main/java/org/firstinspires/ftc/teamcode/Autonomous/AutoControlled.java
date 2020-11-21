package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.Dependencies.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Dependencies.AutoCommands;

import java.time.chrono.MinguoChronology;

@Autonomous(name="Autonomous")
public class AutoControlled extends LinearOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight, shooter, intake, grabberLift;
    private CRServo grabber;
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
        grabberLift = new Motor(hardwareMap, "grabberLift");
        grabber = new CRServo(hardwareMap, "flicker");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        vision = new UGRectDetector(hardwareMap, "camera");
        vision.init();
        vision.setTopRectangle(0.1, 0.22890625);
        vision.setBottomRectangle(0.20555555555555555, 0.2328125);
        vision.setRectangleSize(200, 42);

        AutoCommands robot = new AutoCommands(frontLeft, frontRight, backLeft, backRight, shooter, intake, grabberLift, grabber, voltageSensor);
        robot.initialize();

        waitForStart();

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
//positive = grab
            robot.resetDriveTrainEncoders();
            robot.setTargetInches((int)(9 * 12));
            robot.setTolerance(10);
            robot.navigate(0.3, false, false, false, false, false);

            robot.resetDriveTrainEncoders();
            robot.setTargetInches(12, -12, -12, 12, true);
            robot.setTolerance(10);
            robot.navigate(0.1, false, false, true, false, false);

            grabberLift.set((13/voltageSensor.getVoltage()) * -0.4);
            sleep(1000);
            grabberLift.set(0);

            grabber.set((13/voltageSensor.getVoltage()) * -1);

            grabberLift.set((13/voltageSensor.getVoltage()) * 0.4);
            sleep(1000);
            grabberLift.set(0);

            robot.resetDriveTrainEncoders();
            robot.setTargetInches(-12, 12, 12, -12, true);
            robot.setTolerance(10);
            robot.navigate(0.1, false, true, false, false, false);

            robot.resetDriveTrainEncoders();
            robot.setTargetInches((int)(-4 * 12));
            robot.setTolerance(10);
            robot.navigate(0.3, true, false, false, false, false);
        }

        if (visionDec == 0){
            telemetry.addData("Path", 0);
            telemetry.update();

            robot.resetDriveTrainEncoders();
            robot.setTargetInches((int)(6 * 12));
            robot.setTolerance(10);
            robot.navigate(0.3, false, false, false, false, false);

            robot.resetDriveTrainEncoders();
            robot.setTargetInches(12, -12, -12, 12, true);
            robot.setTolerance(10);
            robot.navigate(0.1, false, false, true, false, false);

            grabberLift.set((13/voltageSensor.getVoltage()) * -0.4);
            sleep(1000);
            grabberLift.set(0);

            grabber.set((13/voltageSensor.getVoltage()) * -1);

            grabberLift.set((13/voltageSensor.getVoltage()) * 0.4);
            sleep(1000);
            grabberLift.set(0);

            robot.resetDriveTrainEncoders();
            robot.setTargetInches(-12, 12, 12, -12, true);
            robot.setTolerance(10);
            robot.navigate(0.1, false, true, false, false, false);
        }

        if (visionDec == 1){
            telemetry.addData("Path", 1);
            telemetry.update();

            robot.resetDriveTrainEncoders();
            robot.setTargetInches((int)(8 * 12));
            robot.setTolerance(10);
            robot.navigate(0.3, false, false, false, false, false);

            robot.resetDriveTrainEncoders();
            robot.setTargetInches(-24, 24, 24, -24, true);
            robot.setTolerance(10);
            robot.navigate(0.1, false, true, false, false, false);

            grabberLift.set((13/voltageSensor.getVoltage()) * -0.4);
            sleep(1000);
            grabberLift.set(0);

            grabber.set((13/voltageSensor.getVoltage()) * -1);

            grabberLift.set((13/voltageSensor.getVoltage()) * 0.4);
            sleep(1000);
            grabberLift.set(0);

            robot.resetDriveTrainEncoders();
            robot.setTargetInches(24, -24, -24, 24, true);
            robot.setTolerance(10);
            robot.navigate(0.1, false, false, true, false, false);

            robot.resetDriveTrainEncoders();
            robot.setTargetInches((int)(-2 * 12));
            robot.setTolerance(10);
            robot.navigate(0.3, true, false, false, false, false);
        }

        robot.stopMotors();

    }
}
