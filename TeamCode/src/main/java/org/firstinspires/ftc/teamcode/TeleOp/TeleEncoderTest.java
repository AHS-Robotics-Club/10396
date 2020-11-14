package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Encoder Test")

public class TeleEncoderTest extends LinearOpMode {
        HardwareMap hmap;
        Motor frontLeft, frontRight, backLeft, backRight, intake, shooter;
        RevIMU imu;

        @Override
        public void runOpMode() throws InterruptedException {
            frontLeft = new Motor(hardwareMap, "fL");
            frontRight = new Motor(hardwareMap, "fR");
            backLeft = new Motor(hardwareMap, "bL");
            backRight = new Motor(hardwareMap, "bR");
            intake = new Motor(hardwareMap, "intake");
            shooter = new Motor(hardwareMap, "shooter");

            imu = new RevIMU(hardwareMap, "imu");

            frontLeft.set(0);
            frontRight.set(0);
            backLeft.set(0);
            backRight.set(0);
            shooter.set(0);
            intake.set(0);

            frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            imu.init();

            waitForStart();

            while (opModeIsActive() && !isStopRequested()) {
                if (gamepad1.a){
                    frontLeft.set(0.5);
                    telemetry.addData("Front Left", frontLeft.getCurrentPosition());
                    telemetry.update();
                } else {
                    frontLeft.set(0);
                }

                if (gamepad1.b){
                    frontRight.set(0.5);
                    telemetry.addData("Front Right", frontRight.getCurrentPosition());
                    telemetry.update();
                } else {
                    frontRight.set(0);
                }

                if (gamepad1.x){
                    backLeft.set(0.5);
                    telemetry.addData("Back Left", backLeft.getCurrentPosition());
                    telemetry.update();
                } else {
                    backLeft.set(0);
                }

                if (gamepad1.y){
                    backRight.set(0.5);
                    telemetry.addData("Back Right", backRight.getCurrentPosition());
                    telemetry.update();
                } else {
                    backRight.set(0);
                }

                if (gamepad1.dpad_up){
                    frontLeft.setInverted(true);
                    telemetry.addData("Front Left INV", frontLeft.getCurrentPosition());
                    telemetry.update();
                    frontLeft.set(0.5);
                } else {
                    frontLeft.set(0);
                }

                if (gamepad1.dpad_down){
                    frontRight.encoder.setDirection(Motor.Direction.REVERSE);
                    telemetry.addData("Front Right ENC INV", frontRight.getCurrentPosition());
                    telemetry.update();
                    frontRight.set(0.5);
                } else {
                    frontRight.set(0);
                }

                if (gamepad1.dpad_left){
                    backLeft.setInverted(true);
                    backLeft.encoder.setDirection(Motor.Direction.REVERSE);
                    telemetry.addData("Back Left INV ENC INV", backLeft.getCurrentPosition());
                    telemetry.update();
                    backLeft.set(0.5);
                } else {
                    backLeft.set(0);
                }
            }
        }


    }