package org.firstinspires.ftc.teamcode.Dependencies;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Autonomous.AutoControlled;

public class AutoCommands extends AutoControlled {
    private Motor fL, fR, bL, bR;
    private Motor shooter, intake;
    private int tolerance;

    public double ticksPerInch = 145.6/((96/25.4)*Math.PI);
    public VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

    public AutoCommands(Motor fLM, Motor fRM, Motor bLM, Motor bRM, Motor shooterM, Motor intakeM){
        fL = fLM;
        fR = fRM;
        bL = bLM;
        bR = bRM;
        shooter = shooterM;
        intake = intakeM;
        tolerance = 0;
    }

    public void initialize(){
        fL.setInverted(true);
        bL.setInverted(true);
        fL.encoder.setDirection(Motor.Direction.FORWARD);
        bL.encoder.setDirection(Motor.Direction.FORWARD);
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
        fR.motor.setTargetPosition((int)(inches * ticksPerInch));
        fL.motor.setTargetPosition((int)(inches * ticksPerInch));
        bR.motor.setTargetPosition((int)(inches * ticksPerInch));
        bL.motor.setTargetPosition((int)(inches * ticksPerInch));
    }

    public void setTargetFeet(int feet){
        fR.motor.setTargetPosition((int)(feet * 12 * ticksPerInch));
        fL.motor.setTargetPosition((int)(feet * 12 * ticksPerInch));
        bR.motor.setTargetPosition((int)(feet * 12 * ticksPerInch));
        bL.motor.setTargetPosition((int)(feet * 12 * ticksPerInch));
    }

    public void setTargetFeet(int feetFL, int feetFR, int feetBL, int feetBR){
        fR.motor.setTargetPosition((int)(feetFR * 12 * ticksPerInch));
        fL.motor.setTargetPosition((int)(feetFL * 12 * ticksPerInch));
        bR.motor.setTargetPosition((int)(feetBR * 12 * ticksPerInch));
        bL.motor.setTargetPosition((int)(feetBL * 12 * ticksPerInch));
    }

    public void setTolerance(int toleranceParam){
        tolerance = toleranceParam;
    }

    public void navigate(double speed, boolean reverse, boolean strafeLeft, boolean strafeRight){
        fR.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fL.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double adjustedSpeed = (13/voltageSensor.getVoltage()) * speed;

        if (reverse) {
            fL.set(-adjustedSpeed);
            fR.set(-adjustedSpeed);
            bL.set(-adjustedSpeed);
            bR.set(-adjustedSpeed);
        } else if (strafeLeft) {
            fL.set(-adjustedSpeed);
            fR.set(adjustedSpeed);
            bL.set(adjustedSpeed);
            bR.set(-adjustedSpeed);
        } else if (strafeRight) {
            fL.set(adjustedSpeed);
            fR.set(-adjustedSpeed);
            bL.set(-adjustedSpeed);
            bR.set(adjustedSpeed);
        } else {
            fL.set(adjustedSpeed);
            fR.set(adjustedSpeed);
            bL.set(adjustedSpeed);
            bR.set(adjustedSpeed);
        }

        while (opModeIsActive() && ((Math.abs(Math.abs(fL.motor.getTargetPosition()) - Math.abs(fL.motor.getCurrentPosition())) < tolerance) || (Math.abs(Math.abs(fR.motor.getTargetPosition()) - Math.abs(fR.motor.getCurrentPosition())) < tolerance) || (Math.abs(Math.abs(bL.motor.getTargetPosition()) - Math.abs(bL.motor.getCurrentPosition())) < tolerance) || (Math.abs(Math.abs(bR.motor.getTargetPosition()) - Math.abs(bR.motor.getCurrentPosition())) < tolerance))) {
            if (Math.abs(fL.motor.getTargetPosition() - fL.motor.getCurrentPosition()) <= tolerance){
                fL.set(0);
            }
            if (Math.abs(fR.motor.getTargetPosition() - fR.motor.getCurrentPosition()) <= tolerance){
                fR.set(0);
            }
            if (Math.abs(bL.motor.getTargetPosition() - bL.motor.getCurrentPosition()) <= tolerance){
                bL.set(0);
            }
            if (Math.abs(bR.motor.getTargetPosition() - bR.motor.getCurrentPosition()) <= tolerance){
                bR.set(0);
            }
        }

        resetDriveTrainEncoders();
    }

    public void stopMotors() {
        fL.stopMotor();
        fR.stopMotor();
        bL.stopMotor();
        bR.stopMotor();
    }
}
