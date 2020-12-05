package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Dependencies.TeleBot;

@TeleOp(name = "Tune PID Constants")
public class PIDTune extends LinearOpMode {
    Motor frontLeft, frontRight, backLeft, backRight;
    Motor intake, shooter, grabberLift;
    RevIMU imu;
    MecanumDrive m_drive;
    VoltageSensor voltageSensor;
    TeleBot robot;
    CRServo flicker, grabber;
    PIDController pid = new PIDController(0, 0, 0);
    GamepadEx gamepad = new GamepadEx(gamepad1);

    @Override
    public void runOpMode() throws InterruptedException {

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

        robot = new TeleBot(frontLeft, frontRight, backLeft, backRight, intake, shooter, grabberLift, grabber, imu, flicker);
        robot.initialize();

        waitForStart();
        double p = 0;
        double i = 0;
        double d = 0;
        double increment = 0.1;

        ButtonReader dpadup = new ButtonReader(
                gamepad, GamepadKeys.Button.DPAD_UP
        );
        ButtonReader dpaddown = new ButtonReader(
                gamepad, GamepadKeys.Button.DPAD_DOWN
        );
        ButtonReader dpadleft = new ButtonReader(
                gamepad, GamepadKeys.Button.DPAD_LEFT
        );
        ButtonReader dpadright = new ButtonReader(
                gamepad, GamepadKeys.Button.DPAD_RIGHT
        );
        ButtonReader leftbumper = new ButtonReader(
                gamepad, GamepadKeys.Button.LEFT_BUMPER
        );
        ButtonReader rightbumper = new ButtonReader(
                gamepad, GamepadKeys.Button.RIGHT_BUMPER
        );
        ButtonReader a = new ButtonReader(
                gamepad, GamepadKeys.Button.A
        );
        ButtonReader b = new ButtonReader(
                gamepad, GamepadKeys.Button.B
        );

        while (opModeIsActive() && !isStopRequested()) {
            pid.setSetPoint(10);
            shooter.resetEncoder();
            shooter.set(pid.calculate(shooter.getCurrentPosition()));

            telemetry.addData("Ticks Per Loop Run", shooter.getCurrentPosition());
            telemetry.addData("Increment Level", increment);
            telemetry.addData("P Constant", p);
            telemetry.addData("I Constant", i);
            telemetry.addData("D Constant", d);

            telemetry.update();

            if (dpadup.wasJustPressed()){
                p += increment;
            }
            if (dpaddown.wasJustPressed()){
                p -= increment;
            }
            if (dpadleft.wasJustPressed()){
                i -= increment;
            }
            if (dpadright.wasJustPressed()){
                i += increment;
            }
            if (leftbumper.wasJustPressed()){
                d -= increment;
            }
            if (rightbumper.wasJustPressed()){
                d += increment;
            }
            if (a.wasJustPressed()){
                increment = 0.01;
            }
            if (b.wasJustPressed()){
                increment = 0.1;
            }
        }
    }
}
