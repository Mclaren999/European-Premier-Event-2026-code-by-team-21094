package org.firstinspires.ftc.teamcode.subsystems.Tests;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Stopper Test", group = "DECODE Tuning")
public class StopperTest extends LinearOpMode {

    public static double POSITION = 0.75;

    @Override
    public void runOpMode() {
        Servo stopper = hardwareMap.get(Servo.class, "opener");

        waitForStart();

        while (opModeIsActive()) {
            stopper.setPosition(POSITION);
        }
    }
}
