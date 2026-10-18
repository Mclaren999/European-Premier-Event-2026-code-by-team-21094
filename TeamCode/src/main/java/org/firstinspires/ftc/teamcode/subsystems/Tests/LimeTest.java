package org.firstinspires.ftc.teamcode.subsystems.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;

@TeleOp
public class LimeTest extends OpMode {
    private Limelight3A limelight;
    private ShooterSystem shooterSystem;
    private ShooterController shooterController;
    private Follower follower;

    public void init() {
        follower = Constants.createFollower(hardwareMap);
        Tuning.follower = follower;
        follower.setPose(new Pose(130, 85, Math.toRadians(0)));
        follower.startTeleOpDrive(true);
        follower.setMaxPower(1);
        shooterSystem = new ShooterSystem(hardwareMap);
        shooterController = new ShooterController(hardwareMap, follower, shooterSystem);
        shooterController.lime = true;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(2);
    }

    @Override
    public void loop() {
        shooterController.update(130, 143, shooterSystem.Vel);
    }
}
