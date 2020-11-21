package org.firstinspires.ftc.teamcode.Dependencies;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Autonomous.AutoControlled;

public class AutoCommands extends AutoControlled {
    private Motor fL, fR, bL, bR;
    private Motor shooter, intake;
    private int tolerance;
    private VoltageSensor voltageSensor;

    public double ticksPerInch = 145.6/((96/25.4)*Math.PI);

    public AutoCommands(Motor fLM, Motor fRM, Motor bLM, Motor bRM, Motor shooterM, Motor intakeM, VoltageSensor volt){
        fL = fLM;
        fR = fRM;
        bL = bLM;
        bR = bRM;
        shooter = shooterM;
        intake = intakeM;
        tolerance = 0;
        voltageSensor = volt;
    }

    public void initialize(){
        fL.setInverted(true);
        bL.setInverted(true);
        fL.encoder.setDirection(Motor.Direction.FORWARD);
        bL.encoder.setDirection(Motor.Direction.FORWARD);

        fL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetDriveTrainEncoders(){
        fR.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetEncoders(){
        fR.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setTarget(int ticks){
        fR.motor.setTargetPosition(ticks);
        fL.motor.setTargetPosition(ticks);
        bR.motor.setTargetPosition(ticks);
        bL.motor.setTargetPosition(ticks);
    }

    public void setTarget(int ticksFL, int ticksFR, int ticksBL, int ticksBR){
        fR.motor.setTargetPosition(ticksFR);
        fL.motor.setTargetPosition(ticksFL);
        bR.motor.setTargetPosition(ticksBR);
        bL.motor.setTargetPosition(ticksBL);
    }

    public void setTargetInches(int inches){
        fR.motor.setTargetPosition((int)(inches * ticksPerInch * 1.5));
        fL.motor.setTargetPosition((int)(inches * ticksPerInch * 1.5));
        bR.motor.setTargetPosition((int)(inches * ticksPerInch * 1.5));
        bL.motor.setTargetPosition((int)(inches * ticksPerInch * 1.5));
    }

    public void setTargetInches(double inchesFL, double inchesFR, double inchesBL, double inchesBR){
        fR.motor.setTargetPosition((int) (inchesFR * ticksPerInch * 1.5));
        fL.motor.setTargetPosition((int) (inchesFL * ticksPerInch * 1.5));
        bR.motor.setTargetPosition((int) (inchesBR * ticksPerInch * 1.5));
        bL.motor.setTargetPosition((int) (inchesBL * ticksPerInch * 1.5));
    }

    public void setTargetInches(double inchesFL, double inchesFR, double inchesBL, double inchesBR, boolean strafe){
        if (!strafe) {
            fR.motor.setTargetPosition((int) (inchesFR * ticksPerInch * 1.5));
            fL.motor.setTargetPosition((int) (inchesFL * ticksPerInch * 1.5));
            bR.motor.setTargetPosition((int) (inchesBR * ticksPerInch * 1.5));
            bL.motor.setTargetPosition((int) (inchesBL * ticksPerInch * 1.5));
        } else {
            fR.motor.setTargetPosition((int) (inchesFR * ticksPerInch * 1.5 * 1.2972973));
            fL.motor.setTargetPosition((int) (inchesFL * ticksPerInch * 1.5 * 1.2972973));
            bR.motor.setTargetPosition((int) (inchesBR * ticksPerInch * 1.5 * 1.2972973));
            bL.motor.setTargetPosition((int) (inchesBL * ticksPerInch * 1.5 * 1.2972973));
        }
    }

    public void setTargetRotation(int degrees){
        if (degrees < 0) {
            fL.motor.setTargetPosition((int)(-(degrees/10)*Math.PI * ticksPerInch * 1.5));
            fR.motor.setTargetPosition((int)((degrees/10)*Math.PI * ticksPerInch * 1.5));
            bL.motor.setTargetPosition((int)(-(degrees/10)*Math.PI * ticksPerInch * 1.5));
            bR.motor.setTargetPosition((int)((degrees/10)*Math.PI * ticksPerInch * 1.5));
        }
        if (degrees > 0) {
            fL.motor.setTargetPosition((int)((degrees/10)*Math.PI * ticksPerInch * 1.5));
            fR.motor.setTargetPosition((int)(-(degrees/10)*Math.PI * ticksPerInch * 1.5));
            bL.motor.setTargetPosition((int)((degrees/10)*Math.PI * ticksPerInch * 1.5));
            bR.motor.setTargetPosition((int)(-(degrees/10)*Math.PI * ticksPerInch * 1.5));
        }

    }

    public void setTargetFeet(int feet){
        fR.motor.setTargetPosition((int)(feet * 12 * ticksPerInch * 1.5));
        fL.motor.setTargetPosition((int)(feet * 12 * ticksPerInch * 1.5));
        bR.motor.setTargetPosition((int)(feet * 12 * ticksPerInch * 1.5));
        bL.motor.setTargetPosition((int)(feet * 12 * ticksPerInch * 1.5));
    }

    public void setTargetFeet(int feetFL, int feetFR, int feetBL, int feetBR){
        fR.motor.setTargetPosition((int)(feetFR * 12 * ticksPerInch * 1.5));
        fL.motor.setTargetPosition((int)(feetFL * 12 * ticksPerInch * 1.5));
        bR.motor.setTargetPosition((int)(feetBR * 12 * ticksPerInch * 1.5));
        bL.motor.setTargetPosition((int)(feetBL * 12 * ticksPerInch * 1.5));
    }

    public void setTolerance(int toleranceParam){
        tolerance = toleranceParam;
    }

    public void navigate(double speed, boolean reverse, boolean strafeLeft, boolean strafeRight, boolean rotateLeft, boolean rotateRight){
        fR.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fL.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double adjustedSpeed = (13/voltageSensor.getVoltage()) * speed;

        if (reverse) {
            fR.set(-adjustedSpeed);
            bL.set(-adjustedSpeed);
            bR.set(-adjustedSpeed);
            sleep(70);
            fL.set(-adjustedSpeed);
        } else if (strafeLeft) {
            fR.set(adjustedSpeed);
            bL.set(adjustedSpeed);
            bR.set(-adjustedSpeed);
            sleep(70);
            fL.set(-adjustedSpeed);
        } else if (strafeRight) {
            fR.set(-adjustedSpeed);
            bL.set(-adjustedSpeed);
            bR.set(adjustedSpeed);
            sleep(70);
            fL.set(adjustedSpeed);
        } else if (rotateLeft) {
            fR.set(adjustedSpeed);
            bL.set(-adjustedSpeed);
            bR.set(adjustedSpeed);
            sleep(70);
            fL.set(-adjustedSpeed);
        } else if (rotateRight) {
            fR.set(-adjustedSpeed);
            bL.set(adjustedSpeed);
            bR.set(-adjustedSpeed);
            sleep(70);
            fL.set(adjustedSpeed);
        } else{
            fR.set(adjustedSpeed);
            bL.set(adjustedSpeed);
            bR.set(adjustedSpeed);
            sleep(70);
            fL.set(adjustedSpeed);
        }

        while (fL.motor.isBusy()){
            idle();
        }
        fL.set(0);
        fR.set(0);
        bL.set(0);
        bR.set(0);
        resetDriveTrainEncoders();
    }

    public void stopMotors() {
        fL.stopMotor();
        fR.stopMotor();
        bL.stopMotor();
        bR.stopMotor();
    }
}
