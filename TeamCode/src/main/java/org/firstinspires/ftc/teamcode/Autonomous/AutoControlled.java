package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.Dependencies.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Dependencies.AutoCommands;

@Autonomous(name="Autonomous")
public class AutoControlled extends LinearOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight, shooter, intake, grabberLift;
    private CRServo grabber, flicker;
    public VoltageSensor voltageSensor;
    public UGRectDetector vision;

    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = new Motor(hardwareMap, "fL");
        frontLeft = new Motor(hardwareMap, "fR");
        backRight = new Motor(hardwareMap, "bL");
        backLeft = new Motor(hardwareMap, "bR");
        shooter = new Motor(hardwareMap, "shooter");
        intake = new Motor(hardwareMap, "intake");
        grabberLift = new Motor(hardwareMap, "grabberLift");
        grabber = new CRServo(hardwareMap, "grabber");
        flicker = new CRServo(hardwareMap, "flicker");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        vision = new UGRectDetector(hardwareMap, "camera");
        vision.init();
        vision.setTopRectangle(0.1, 0.22890625);
        vision.setBottomRectangle(0.20555555555555555, 0.2328125);
        vision.setRectangleSize(200, 42);

        AutoCommands robot = new AutoCommands(frontLeft, frontRight, backLeft, backRight, shooter, intake, grabberLift, grabber, flicker, voltageSensor);
        robot.initialize();

        waitForStart();

        long startTime = System.currentTimeMillis();
        int visionDec = 0;
        while((System.currentTimeMillis()-startTime)<4500) {
            telemetry.addData("Vision", vision.getStack());
            telemetry.update();
            visionDec = vision.getStack();
        }

        if (visionDec == 4){
            //telemetry for vision
            telemetry.addData("Vision", 4);
            telemetry.update();

            //move forward 9 feet at 0.3 speed
            robot.setTargetInches((int)(9 * 12));
            robot.navigate(0.3);

            //strafe right 12 inches at 0.1 speed
            robot.setTargetInches(12, -12, -12, 12);
            robot.navigate(0.1);

            //drop the wobble goal
            robot.dropWobbleGoal();

            //strafe left 1 feet at 0.1 speed
            robot.setTargetInches(-12, 12, 12, -12);
            robot.navigate(0.1);

            //move backward 4 feet at 0.3 speed
            robot.setTargetInches((int)(-4 * 12));
            robot.navigate(0.3);
        }

        if (visionDec == 0){
            //telemetry for vision
            telemetry.addData("Vision", 0);
            telemetry.update();

            //move forward 6 feet at 0.3 speed
            robot.setTargetInches((int)(6 * 12));
            robot.navigate(0.3);

            //strafe right 12 inches at 0.1 speed
            robot.setTargetInches(12, -12, -12, 12);
            robot.navigate(0.1);

            //drop the wobble goal
            robot.dropWobbleGoal();

            //strafe left 1 feet at 0.1 speed
            robot.setTargetInches(-12, 12, 12, -12);
            robot.navigate(0.1);
        }

        if (visionDec == 1){
            //telemetry for vision
            telemetry.addData("Vision", 1);
            telemetry.update();

            //move forward 8 feet at 0.3 speed
            robot.setTargetInches((int)(8 * 12));
            robot.navigate(0.3);

            //strafe left 2 feet at 0.1 speed
            robot.setTargetInches(-24, 24, 24, -24);
            robot.navigate(0.1);

            //drop the wobble goal
            robot.dropWobbleGoal();

            //strafe right 2 feet at 0.1 speed
            robot.setTargetInches(24, -24, -24, 24);
            robot.navigate(0.1);

            //move backward 2 feet at 0.3 speed
            robot.setTargetInches((int)(-2 * 12));
            robot.navigate(0.3);
        }

        //stop motors
        robot.stopMotors();
    }
}
