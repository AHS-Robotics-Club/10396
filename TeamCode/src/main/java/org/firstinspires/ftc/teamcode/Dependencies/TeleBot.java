package org.firstinspires.ftc.teamcode.Dependencies;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TeleBot {
    private Motor fL, fR, bL, bR, shooter, intake;
    private RevIMU imu;
    private CRServo flick;

    public TeleBot(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, Motor intakeParam, Motor shooterParam, RevIMU imuParam, CRServo flicker) {
        fL = frontLeft;
        fR = frontRight;
        bL = backLeft;
        bR = backRight;
        shooter = shooterParam;
        intake = intakeParam;
        imu = imuParam;
        flick = flicker;
    }

    public void initialize(){
        fL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        fL.set(0);
        fR.set(0);
        bL.set(0);
        bR.set(0);
        shooter.set(0);
        intake.set(0);

        flick.set(0);

        fL.resetEncoder();
        fR.resetEncoder();
        bL.resetEncoder();
        bR.resetEncoder();
        shooter.resetEncoder();
        intake.resetEncoder();

        imu.init();
    }
}