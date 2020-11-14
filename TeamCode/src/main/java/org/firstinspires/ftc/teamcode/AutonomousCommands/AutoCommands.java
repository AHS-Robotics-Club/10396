package org.firstinspires.ftc.teamcode.AutonomousCommands;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AutonomousCommands.AutoCommands;


import org.firstinspires.ftc.teamcode.Autonomous.AutoControlled;

public class AutoCommands extends AutoControlled {
    private Motor fL, fR, bL, bR;

    public AutoCommands(Motor fLM, Motor fRM, Motor bLM, Motor bRM){
        fL = fLM;
        fR = fRM;
        bL = bLM;
        bR = bRM;
    }

    public void resetDTEncoders(Motor fL, Motor fR, Motor bL, Motor bR){
        fL.resetEncoder();
        fR.resetEncoder();
        bL.resetEncoder();
        bR.resetEncoder();
    }

    public void stopMotors(Motor fL, Motor fR, Motor bL, Motor bR){
        fL.stopMotor();
        fR.stopMotor();
        bL.stopMotor();
        bR.stopMotor();
    }

    public void setPosTol(Motor fL, Motor fR, Motor bL, Motor bR, int posTol){
        fL.setPositionTolerance(posTol);
        fR.setPositionTolerance(posTol);
        bL.setPositionTolerance(posTol);
        bR.setPositionTolerance(posTol);
    }

    public void setPos(Motor fL, Motor fR, Motor bL, Motor bR, int inchFor, double tpi){
        fL.setTargetPosition((int)(inchFor * tpi));
        fR.setTargetPosition((int)(inchFor * tpi));
        bL.setTargetPosition((int)(inchFor * tpi));
        bR.setTargetPosition((int)(inchFor * tpi));
    }

    public void setPosStrafeLeft(Motor fL, Motor fR, Motor bL, Motor bR, int inchLeft, double tpi){
        fL.setTargetPosition(-(int)(inchLeft * tpi));
        fR.setTargetPosition((int)(inchLeft * tpi));
        bL.setTargetPosition((int)(inchLeft * tpi));
        bR.setTargetPosition(-(int)(inchLeft * tpi));
    }

    public void setPosStrafeRight(Motor fL, Motor fR, Motor bL, Motor bR, int inchRight, double tpi){
        fL.setTargetPosition((int)(inchRight * tpi));
        fR.setTargetPosition(-(int)(inchRight * tpi));
        bL.setTargetPosition(-(int)(inchRight * tpi));
        bR.setTargetPosition((int)(inchRight * tpi));
    }

    public void forward(Motor fL, Motor fR, Motor bL, Motor bR, double tpi, int inchForward, int posTol, double speed){
        resetDTEncoders(fL, fR, bL, bR);
        setPos(fL, fR, bL, bR, inchForward, tpi);
        setPosTol(fL, fR, bL, bR, posTol);
        telemetry.addData("Pos", "reset");
        telemetry.update();
        while (opModeIsActive() && (!fL.atTargetPosition() || !fR.atTargetPosition() || !bR.atTargetPosition() || !bL.atTargetPosition())) {
            telemetry.addData("Pos", "in loop");
            telemetry.update();
            if(!fL.atTargetPosition())
                fL.set(speed);
            if(!fR.atTargetPosition())
                fR.set(speed);
            if(!bL.atTargetPosition())
                bL.set(speed);
            if(!bR.atTargetPosition())
                bR.set(speed);
        }

        stopMotors(fL, fR, bL, bR);
        resetDTEncoders(fL, fR, bL, bR);
    }

    public void backward(Motor fL, Motor fR, Motor bL, Motor bR, double tpi, int inchBackward, int posTol, double speed){
        resetDTEncoders(fL, fR, bL, bR);
        setPos(fL, fR, bL, bR, -inchBackward, tpi);
        setPosTol(fL, fR, bL, bR, posTol);

        while (opModeIsActive() && (!fL.atTargetPosition() || !fR.atTargetPosition() || !bR.atTargetPosition() || !bL.atTargetPosition())) {
            if(!fL.atTargetPosition())
                fL.set(-speed);
            if(!fR.atTargetPosition())
                fR.set(-speed);
            if(!bL.atTargetPosition())
                bL.set(-speed);
            if(!bR.atTargetPosition())
                bR.set(-speed);
        }

        stopMotors(fL, fR, bL, bR);
        resetDTEncoders(fL, fR, bL, bR);
    }

    public void strafeLeft(Motor fL, Motor fR, Motor bL, Motor bR, double tpi, int inchLeft, int posTol, double speed){
        resetDTEncoders(fL, fR, bL, bR);
        setPosStrafeLeft(fL, fR, bL, bR, inchLeft, tpi);
        setPosTol(fL, fR, bL, bR, posTol);

        while (opModeIsActive() && (!fL.atTargetPosition() || !fR.atTargetPosition() || !bR.atTargetPosition() || !bL.atTargetPosition())) {
            if(!fL.atTargetPosition())
                fL.set(-speed);
            if(!fR.atTargetPosition())
                fR.set(speed);
            if(!bL.atTargetPosition())
                bL.set(speed);
            if(!bR.atTargetPosition())
                bR.set(-speed);
        }

        stopMotors(fL, fR, bL, bR);
        resetDTEncoders(fL, fR, bL, bR);
    }


    public void strafeRight(Motor fL, Motor fR, Motor bL, Motor bR, double tpi, int inchRight, int posTol, double speed){
        resetDTEncoders(fL, fR, bL, bR);
        setPosStrafeRight(fL, fR, bL, bR, inchRight, tpi);
        setPosTol(fL, fR, bL, bR, posTol);

        while (opModeIsActive() && (!fL.atTargetPosition() || !fR.atTargetPosition() || !bR.atTargetPosition() || !bL.atTargetPosition())) {
            if(!fL.atTargetPosition())
                fL.set(speed);
            if(!fR.atTargetPosition())
                fR.set(-speed);
            if(!bL.atTargetPosition())
                bL.set(-speed);
            if(!bR.atTargetPosition())
                bR.set(speed);
        }

        stopMotors(fL, fR, bL, bR);
        resetDTEncoders(fL, fR, bL, bR);
    }
}
