package org.firstinspires.ftc.teamcode.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Units.Chassis;
import org.firstinspires.ftc.teamcode.Units.Grabber;
import org.firstinspires.ftc.teamcode.Units.SkyStoneDetector;

@Autonomous(name = "BlueSkyStoneFoundation", group = "Blue")
//@Disabled
public class BlueSkyStoneFoundation extends LinearOpMode {

    private Chassis chassis = new Chassis();
    private SkyStoneDetector detector = new SkyStoneDetector();
    private Grabber grabber = new Grabber();

    @Override
    public void runOpMode() throws InterruptedException {
        chassis.init(this);
        detector.init(this);
        grabber.init(this);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        SkyStoneDetector.Position position = detector.getSkystonePosition();
        telemetry.addData("Position", position);
        telemetry.update();
        detector.shutdown();

        chassis.encoderDrive(0.5, 750, Chassis.Direction.LEFT, 2.5);

        switch (position) {
            case LEFT:
                chassis.encoderDrive(0.5, 400, Chassis.Direction.BACKWARD, 2.5);
                break;
            case CENTER:
                chassis.encoderDrive(0.5, 200, Chassis.Direction.BACKWARD, 2);
        }

        grabber.grabBlue();

        chassis.encoderDrive(0.5, 100, Chassis.Direction.RIGHT, 1.5);
        chassis.distanceDrive(0.5, Chassis.Direction.BACKWARD, 50, 2);
        chassis.encoderDrive(0.5, 200, Chassis.Direction.LEFT, 2);

        grabber.throwBlockBlue();

        chassis.gyroTurn(0.8, -90);
        chassis.encoderDrive(0.5, 100, Chassis.Direction.FORWARD, 2);

        grabber.lockFundament();

        chassis.encoderDrive(0.5, 600, Chassis.Direction.BACKWARD, 2.5);
        chassis.gyroTurn(0.8, -90);

        chassis.encoderDrive(0.5, 370, Chassis.Direction.FORWARD, 2.5);

        grabber.unlockFundament();

        chassis.encoderDrive(0.5, 1000, Chassis.Direction.BACKWARD, 3.5);
    }
}
