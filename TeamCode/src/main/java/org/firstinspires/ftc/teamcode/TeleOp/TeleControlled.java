package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Dependencies.TeleBot;

@TeleOp(name = "TeleOp")
public class TeleControlled extends LinearOpMode {
    Motor frontLeft, frontRight, backLeft, backRight;
    Motor intake, shooter, grabberLift;
    RevIMU imu;
    MecanumDrive m_drive;
    VoltageSensor voltageSensor;
    TeleBot robot;
    CRServo flicker, grabber;

    double speed_multiplier = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean unlocked = true;

        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");
        intake = new Motor(hardwareMap, "intake");
        shooter = new Motor(hardwareMap, "shooter");
        flicker = new CRServo(hardwareMap, "flicker");
        grabberLift = new Motor(hardwareMap, "grabberLift");
        grabber = new CRServo(hardwareMap, "grabber");

        imu = new RevIMU(hardwareMap, "imu");

        m_drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        robot = new TeleBot(frontLeft, frontRight, backLeft, backRight, intake, shooter, grabberLift, grabber, imu, flicker);
        robot.initialize();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            m_drive.driveFieldCentric(
                    -gamepad1.left_stick_x * speed_multiplier,
                    -gamepad1.left_stick_y * speed_multiplier,
                    -gamepad1.right_stick_x * speed_multiplier,
                    Math.toRadians(imu.getHeading())
            );

            if (gamepad1.y){
                shooter.set((13/voltageSensor.getVoltage()) * 0.52);
            } else if (gamepad1.a){
                shooter.set((13/voltageSensor.getVoltage()) * 0.4);
            } else if (gamepad1.b) {
                shooter.set((13/voltageSensor.getVoltage()) * 0.5);
            } else {
                shooter.set(0);
            }

            if (gamepad1.dpad_up){
                speed_multiplier = 1.0;
            } else if (gamepad1.dpad_down){
                speed_multiplier = 0.5;
            }

            if (gamepad1.dpad_left){
                flicker.set(0);
                sleep(300);
                flicker.set(-1);
                sleep(300);
                flicker.set(0);
                sleep(300);
                flicker.set(1);
                sleep(300);
                flicker.set(0);
            }

            if (gamepad1.left_bumper){
                grabberLift.set((13/voltageSensor.getVoltage()) * -0.4);
                //sleep(1000);
                //grabberLift.set(0);
            }
            if (gamepad1.right_bumper){
                grabberLift.set((13/voltageSensor.getVoltage()) * 0.4);
                sleep(1000);
                grabberLift.set(0);
            }

            if (gamepad1.dpad_right){
                if (unlocked){
                    grabber.set(1);
                    unlocked = false;
                } else {
                    grabber.set((13/voltageSensor.getVoltage()) * -1);
                    sleep(1000);
                    grabber.set(0);
                    unlocked = true;
                }
            }
        }
        m_drive.stop();
    }


}
