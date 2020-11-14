package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "TeleOpMain")
public class TeleControlled extends LinearOpMode {
    HardwareMap hmap;
    Motor frontLeft, frontRight, backLeft, backRight, intake, shooter;
    RevIMU imu;
    MecanumDrive m_drive;


    @Override
    public void runOpMode() throws InterruptedException {
        double sp_multiplier = 0.75;
        double shooter_speed, voltage;

        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");
        intake = new Motor(hardwareMap, "intake");
        shooter = new Motor(hardwareMap, "shooter");
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        m_drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        imu = new RevIMU(hardwareMap, "imu");

        //frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);
        shooter.set(0);
        intake.set(0);

        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();
        shooter.resetEncoder();
        intake.resetEncoder();

        imu.init();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            m_drive.driveFieldCentric(
                    -gamepad1.left_stick_x * sp_multiplier,
                    -gamepad1.left_stick_y * sp_multiplier,
                    -gamepad1.right_stick_x * sp_multiplier,
                    Math.toRadians(imu.getHeading())
            );

            if (gamepad1.y){
                voltage = voltageSensor.getVoltage();
                shooter_speed = (13/voltage) * 0.47;
                shooter.set(shooter_speed);
                shooter.resetEncoder();
                //shooter.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Offset (0.6)", shooter.getCurrentPosition());
                telemetry.addData("Voltage", voltage);
                telemetry.addData("Speed", shooter_speed);
                telemetry.update();
            } else if (gamepad1.a){
                voltage = voltageSensor.getVoltage();
                shooter_speed = (13/voltage) * 0;
                shooter.set(shooter_speed);
                shooter.resetEncoder();
                //shooter.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Offset (0.5)", shooter.getCurrentPosition());
                telemetry.addData("Voltage", voltage);
                telemetry.addData("Speed", shooter_speed);
                telemetry.update();
            } else if (gamepad1.b) {
                voltage = voltageSensor.getVoltage();
                shooter_speed = (13/voltage) * 0.44;
                shooter.set(shooter_speed);
                shooter.resetEncoder();
                //shooter.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shooter.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Offset (0.4)", shooter.getCurrentPosition());
                telemetry.addData("Voltage", voltage);
                telemetry.addData("Speed", shooter_speed);
                telemetry.update();
            } else if (gamepad1.x) {
                shooter.set(-0.25);
            } else {
                shooter.set(0);
            }

            intake.set(gamepad1.left_trigger);

            if (gamepad1.dpad_up){
                sp_multiplier = 1.0;
                telemetry.addData("Max Speed", sp_multiplier);
                telemetry.update();
            } else if (gamepad1.dpad_down){
                sp_multiplier = 0.5;
                telemetry.addData("Max Speed", sp_multiplier);
                telemetry.update();
            }
        }
        m_drive.stop();
    }


}