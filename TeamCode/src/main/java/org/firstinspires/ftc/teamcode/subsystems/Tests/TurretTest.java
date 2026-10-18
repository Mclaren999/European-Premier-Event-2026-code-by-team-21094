package org.firstinspires.ftc.teamcode.subsystems.Tests;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad.GamepadSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;

@Config
@TeleOp(name = "Turret Test")
public class TurretTest extends OpMode {
    private Follower follower;
    private ShooterSystem shooter;
    private PIDController headingPID;
    private ShooterController shooterController;
    public static double HEADING_KP = 0.015;
    public static double HEADING_KD = 0.002;
    public static double TURN_SPEED_DEG_PER_SEC = 180;
    public static double HEADING_DEADBAND_DEG = 2;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double headingTargetDeg;
    private GamepadSubSystem DriveOP;
    private boolean xSnapLast = false;

    @Override
    public void init() {
        DriveOP = new GamepadSubSystem(gamepad1);
        shooter = new ShooterSystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        Tuning.follower = follower;
        follower.setPose(new Pose(72, 72, Math.toRadians(180)));
        follower.startTeleOpDrive(true);
        follower.setMaxPower(1);
        shooterController = new ShooterController(hardwareMap, follower, shooter);
        headingPID = new PIDController(HEADING_KP, 0, HEADING_KD);
        headingPID.setTolerance(HEADING_DEADBAND_DEG);
        headingTargetDeg = Math.toDegrees(follower.getPose().getHeading());
        shooterController.zero = false;
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        shooterController.update(24, 72, shooter.Vel);
        handleDrivingAndTurret(dt);
        telemetry.addData("Target Position", shooterController.turretTargetPosition);
        telemetry.addData("Angle", shooterController.Angle);
        telemetry.addData("Field Angle", shooterController.fieldAngle);
        follower.update();
    }

    private void handleDrivingAndTurret(double dt) {
        DriveOP.readButtons();
        double turnStick = DriveOP.getRightX(dt);
        double forward = DriveOP.getLeftY(dt);
        double strafe = DriveOP.getLeftX(dt);
        double currentHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        headingTargetDeg += turnStick * TURN_SPEED_DEG_PER_SEC * dt;
        if (gamepad1.x && !xSnapLast) headingTargetDeg += 180;
        xSnapLast = gamepad1.x;
        double headingError = DriveOP.angleErrorDeg(currentHeadingDeg, headingTargetDeg);
        if (abs(headingError) < HEADING_DEADBAND_DEG) headingError = 0;
        double turnPower = -headingPID.calculate(headingError, 0);
        follower.setTeleOpDrive(forward, strafe, turnPower, false);
        follower.update();
    }
}
