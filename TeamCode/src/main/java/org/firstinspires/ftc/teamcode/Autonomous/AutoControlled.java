package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AutonomousCommands.AutoCommands;

@Autonomous(name="AutoMain")
public class AutoControlled extends LinearOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight, shooter, intake;
    private AutoCommands autoBot;

    public static final double WHEEL_DIAMETER = 3.93701;
    public static final double TICKS_PER_REV = 145.6;
    public static final double HALF_WHEEL_CIRCUMFERENCE = Math.PI * (WHEEL_DIAMETER / 2);
    public static final double TICKS_PER_INCH = TICKS_PER_REV / HALF_WHEEL_CIRCUMFERENCE;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = new Motor(hardwareMap, "fL");
        frontRight = new Motor(hardwareMap, "fR");
        backLeft = new Motor(hardwareMap, "bL");
        backRight = new Motor(hardwareMap, "bR");
        shooter = new Motor(hardwareMap, "shooter");
        intake = new Motor(hardwareMap, "intake");

        frontLeft.setInverted(true);
        backLeft.setInverted(true);

        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(5000);
        backLeft.setTargetPosition(5000);
        frontRight.setTargetPosition(5000);
        backRight.setTargetPosition(5000);

        frontLeft.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        frontLeft.set(1.0);
        frontRight.set(1.0);
        backLeft.set(1.0);
        backRight.set(1.0);

        while (opModeIsActive() && Math.abs(5000 - frontLeft.getCurrentPosition()) < 200)   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left", frontLeft.getCurrentPosition());
            telemetry.addData("encoder-fwd-right", frontRight.getCurrentPosition());
            telemetry.update();
            idle();
        }

        frontLeft.set(0.0);
        frontRight.set(0.0);
        backLeft.set(0.0);
        backRight.set(0.0);

/*



        autoBot = new AutoCommands(frontLeft, frontRight, backLeft, backRight);

        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);
        shooter.set(0);
        intake.set(0);

        //frontLeft.encoder.setDirection(Motor.Direction.REVERSE);
        //backLeft.encoder.setDirection(Motor.Direction.REVERSE);

        autoBot.resetDTEncoders(frontLeft, frontRight, backLeft, backRight);

        frontLeft.setRunMode(Motor.RunMode.PositionControl);
        frontRight.setRunMode(Motor.RunMode.PositionControl);
        backLeft.setRunMode(Motor.RunMode.PositionControl);
        backRight.setRunMode(Motor.RunMode.PositionControl);

        frontLeft.setPositionCoefficient(0.05);
        frontRight.setPositionCoefficient(0.05);
        backLeft.setPositionCoefficient(0.05);
        backRight.setPositionCoefficient(0.05);

        waitForStart();

        //autoBot.forward(frontLeft, frontRight, backLeft, backRight, TICKS_PER_INCH, 12, 5, 0.3);
        //autoBot.backward(frontLeft, frontRight, backLeft, backRight, TICKS_PER_INCH, 12, 5, 0.3);
        //autoBot.strafeLeft(frontLeft, frontRight, backLeft, backRight, TICKS_PER_INCH, 24, 5, 0.5);
        //autoBot.strafeRight(frontLeft, frontRight, backLeft, backRight, TICKS_PER_INCH, 24, 5, 0.5);

        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setRunMode(Motor.RunMode.PositionControl);
        frontRight.setRunMode(Motor.RunMode.PositionControl);
        backLeft.setRunMode(Motor.RunMode.PositionControl);
        backRight.setRunMode(Motor.RunMode.PositionControl);

        frontLeft.setTargetPosition((int)(1000));
        frontRight.setTargetPosition((int)(1000));
        backLeft.setTargetPosition((int)(1000));
        backRight.setTargetPosition((int)(1000));

        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);

        frontLeft.setPositionTolerance(100);
        frontRight.setPositionTolerance(100);
        backLeft.setPositionTolerance(100);
        backRight.setPositionTolerance(100);

        telemetry.addData("1", opModeIsActive());

        while (opModeIsActive() && (!frontLeft.atTargetPosition() || !frontRight.atTargetPosition() || !backRight.atTargetPosition() || !backLeft.atTargetPosition())) {
            telemetry.addData("fL", frontLeft.getCurrentPosition());
            telemetry.addData("fR", frontRight.getCurrentPosition());
            telemetry.addData("bL", backLeft.getCurrentPosition());
            telemetry.addData("bR", backRight.getCurrentPosition());
            telemetry.update();
            if (!frontLeft.atTargetPosition()){
                frontLeft.set(0.05);
            } else {
                frontLeft.set(0);
            }
            if (!frontRight.atTargetPosition()){
                frontRight.set(0.05);
            } else {
                frontRight.set(0);
            }
            if (!backLeft.atTargetPosition()){
                backLeft.set(0.05);
            } else {
                backLeft.set(0);
            }
            if (!backRight.atTargetPosition()){
                backRight.set(0.05);
            } else {
                backRight.set(0);
            }
        }

        frontLeft.set(0);
        frontRight.set(0);
        backLeft.set(0);
        backRight.set(0);

        sleep(10000);
        */

    }
}