package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Testing")
public class Test extends LinearOpMode {
    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;
    private GamepadEx game;
    @Override
    public void runOpMode() throws InterruptedException {
        fL = new Motor(hardwareMap, "fL");
        game = new GamepadEx(gamepad1);
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        drive = new MecanumDrive(fL, fR, bL, bR);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            drive.driveRobotCentric(
                    -game.getLeftX(),
                    -game.getLeftY(),
                    -game.getRightX()
            );
        }

    }
}
