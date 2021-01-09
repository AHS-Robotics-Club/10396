package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    CRServo flicker;
    SimpleServo grabber;

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
        shooter.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(0.6, 0.03, 0);
        shooter.setRunMode(Motor.RunMode.RawPower);
        flicker = new CRServo(hardwareMap, "flicker");
        grabberLift = new Motor(hardwareMap, "grabberLift");
        grabber = new SimpleServo(hardwareMap, "grabber");

        imu = new RevIMU(hardwareMap, "imu");

        m_drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        robot = new TeleBot(frontLeft, frontRight, backLeft, backRight, intake, shooter, grabberLift, grabber, imu, flicker);
        robot.initialize();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            m_drive.driveRobotCentric(
                    -gamepad1.left_stick_x * 1.75 * 0.75,
                    gamepad1.left_stick_y  *0.75,
                    -gamepad1.right_stick_x * speed_multiplier * 0.75
            );

            if (gamepad1.y) {
                double placeholder = Math.pow(Math.sqrt(13.5/voltageSensor.getVoltage()), 0.3);
                shooter.set(placeholder * 0.52);
            } else if(gamepad1.left_bumper){
                double placeholder = Math.pow(Math.sqrt(13.5/voltageSensor.getVoltage()), 0.3);
                shooter.set(placeholder * 0.53);
            }
            else {
                shooter.set(0);
            }

            if (gamepad1.dpad_up) {
                speed_multiplier = 1.0;
            } else if (gamepad1.dpad_down) {
                speed_multiplier = 0.5;
            }

            if (gamepad1.b) {
                flicker.set(0);
                sleep(350);
                flicker.set(-1);
                sleep(350);
                flicker.set(0);
                sleep(350);
                flicker.set(1);
                sleep(350);
                flicker.set(0);
            }

            if (gamepad1.dpad_left) {
                grabberLift.set(-(13 / voltageSensor.getVoltage()));
                sleep(400);
                grabberLift.set(0);
                sleep(500);
            } else if (gamepad1.dpad_right) {
                grabberLift.set((13 / voltageSensor.getVoltage()) * 0.4);
                sleep(400);
                grabberLift.set(0);
                sleep(500);
            } else {
                grabberLift.set(0);
            }

            if (gamepad1.a) {
                grabber.setPosition(0);
            }else if(gamepad1.x){
                grabber.setPosition(1);
            }
        }
        m_drive.stop();
    }
}
