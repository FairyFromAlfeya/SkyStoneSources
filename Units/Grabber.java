package org.firstinspires.ftc.teamcode.Units;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {

    private Servo found1;
    private Servo found2;
    private Servo hand1;
    private Servo hand2;
    private Servo take1;
    private Servo take2;

    private OpMode opMode;

    public void init(OpMode opMode) {
        this.opMode = opMode;

        found1 = opMode.hardwareMap.get(Servo.class, "found1");
        found2 = opMode.hardwareMap.get(Servo.class, "found2");
        hand1 = opMode.hardwareMap.get(Servo.class, "hand1");
        hand2 = opMode.hardwareMap.get(Servo.class, "hand2");
        take1 = opMode.hardwareMap.get(Servo.class, "take1");
        take2 = opMode.hardwareMap.get(Servo.class, "take2");

        found1.setPosition(0.25);
        found2.setPosition(0.75);

        hand1.setPosition(0.0);
        take1.setPosition(0.0);

        hand2.setPosition(1.0); //hand left
        take2.setPosition(0.6); //lift left
    }

    public void grabBlue() {
        LinearOpMode linearOpMode = (LinearOpMode)opMode;

        take2.setPosition(0.15);
        linearOpMode.sleep(400);
        hand2.setPosition(0.3);
        linearOpMode.sleep(600);
        take2.setPosition(0.8);
        linearOpMode.sleep(300);
    }

    public void throwBlockBlue() {
        LinearOpMode linearOpMode = (LinearOpMode)opMode;

        hand2.setPosition(1.0);
        linearOpMode.sleep(300);
        take2.setPosition(0.6);
        linearOpMode.sleep(300);
    }

    public void grabRed() {
        LinearOpMode linearOpMode = (LinearOpMode)opMode;

        take1.setPosition(0.15);
        linearOpMode.sleep(400);
        hand1.setPosition(0.3);
        linearOpMode.sleep(600);
        take1.setPosition(0.8);
        linearOpMode.sleep(300);
    }

    public void throwBlockRed() {
        LinearOpMode linearOpMode = (LinearOpMode)opMode;

        hand1.setPosition(1.0);
        linearOpMode.sleep(300);
        take1.setPosition(0.6);
        linearOpMode.sleep(300);
    }

    public void lockFundament() {
        LinearOpMode linearOpMode = (LinearOpMode)opMode;

        found1.setPosition(0.8);
        found2.setPosition(0.2);
        linearOpMode.sleep(300);
    }

    public void unlockFundament() {
        found1.setPosition(0);
        found2.setPosition(1);
    }
}
