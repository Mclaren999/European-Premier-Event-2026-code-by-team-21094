package org.firstinspires.ftc.teamcode.TeleOPs.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;

@Autonomous(name = "AutoBlueDalnee", group = "Autonomous")
public class AutoBlueDalnee extends LinearOpMode {
    private Follower follower;
    private ShooterSystem shooterSystem;
    private ShooterController shooterController;
    private TransferSystem transferSystem;
    private VoltageSensor batteryVoltageSensor;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private static final double NOMINAL_VOLTAGE = 12.0;

    private final Pose startPose       = new Pose(55, 8.6, Math.toRadians(180));
    private final Pose scorePose       = new Pose(55, 13, Math.toRadians(180));
    private final Pose intakeShortPose = new Pose(8, 8.6, Math.toRadians(180));
    private final Pose intake2Pose     = new Pose(45, 35, Math.toRadians(180));
    private final Pose pickup2Pose     = new Pose(12, 35, Math.toRadians(180));
    private final Pose intakeLongPose  = new Pose(8, 35, Math.toRadians(180));
    private final Pose leavePose       = new Pose(35, 15, Math.toRadians(180));

    private PathChain pathScore1, pathScore2, pathScore3, pathScore4, pathScore5, pathScore6, pathScore7;
    private PathChain pathIntake1, pathIntake2, pathIntake3, pathIntake4, pathIntake5, pathIntake6;
    private PathChain pathLeave;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        shooterSystem = new ShooterSystem(hardwareMap);
        shooterController = new ShooterController(hardwareMap, follower, shooterSystem);
        transferSystem = new TransferSystem(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        shooterController.zero = false;
        shooterController.lime = true;
        shooterSystem.enabled = true;

        buildPaths();
        waitForStart();
        loopTimer.reset();

        if (isStopRequested()) return;

        follower.setMaxPower(0.9);

        followPath(pathScore1);
        waitMs(300);

        while (!shooterSystem.atSetPoint()) waitMs(50);
        shootThreeBalls();

        transferSystem.TurnOn();
        followPath(pathIntake1);
        transferSystem.TurnOn();
        followPath(pathScore2);
        waitMs(300);
        shootThreeBalls();

        transferSystem.TurnOn();
        followPath(pathIntake2);
        transferSystem.TurnOn();
        followPath(pathScore3);
        waitMs(300);
        shootThreeBalls();

        transferSystem.TurnOn();
        followPath(pathIntake3);
        transferSystem.TurnOn();
        followPath(pathScore4);
        waitMs(300);
        shootThreeBalls();

        transferSystem.TurnOn();
        followPath(pathIntake4);
        transferSystem.TurnOn();
        followPath(pathScore5);
        waitMs(300);
        shootThreeBalls();

        transferSystem.TurnOn();
        followPath(pathIntake5);
        transferSystem.TurnOn();
        followPath(pathScore6);
        waitMs(300);
        shootThreeBalls();

        transferSystem.TurnOn();
        followPath(pathIntake6);
        transferSystem.TurnOn();
        followPath(pathScore7);
        waitMs(300);
        shootThreeBalls();

        shooterController.zero = true;
        followPath(pathLeave);
    }

    private void buildPaths() {
        pathScore1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        pathScore2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeShortPose, scorePose))
                .setLinearHeadingInterpolation(intakeShortPose.getHeading(), scorePose.getHeading())
                .build();
        pathScore3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();
        pathScore4 = follower.pathBuilder()
                .addPath(new BezierLine(intakeLongPose, scorePose))
                .setLinearHeadingInterpolation(intakeLongPose.getHeading(), scorePose.getHeading())
                .build();
        pathScore5 = follower.pathBuilder()
                .addPath(new BezierLine(intakeShortPose, scorePose))
                .setLinearHeadingInterpolation(intakeShortPose.getHeading(), scorePose.getHeading())
                .build();
        pathScore6 = follower.pathBuilder()
                .addPath(new BezierLine(intakeLongPose, scorePose))
                .setLinearHeadingInterpolation(intakeLongPose.getHeading(), scorePose.getHeading())
                .build();
        pathScore7 = follower.pathBuilder()
                .addPath(new BezierLine(intakeShortPose, scorePose))
                .setLinearHeadingInterpolation(intakeShortPose.getHeading(), scorePose.getHeading())
                .build();
        pathIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeShortPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeShortPose.getHeading())
                .build();
        pathIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading())
                .addPath(new BezierLine(intake2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), pickup2Pose.getHeading())
                .build();
        pathIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeLongPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeLongPose.getHeading())
                .build();
        pathIntake4 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeShortPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeShortPose.getHeading())
                .build();
        pathIntake5 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeLongPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeLongPose.getHeading())
                .build();
        pathIntake6 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeShortPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeShortPose.getHeading())
                .build();
        pathLeave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }

    private void updateEverything() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        follower.update();
        shooterSystem.updateShooter(shooterController.Distance(8, 143));
        shooterController.update(8, 143, shooterSystem.Vel);
        shooterSystem.update(dt);
    }

    private void followPath(PathChain path) {
        follower.followPath(path, true);
        while (opModeIsActive() && follower.isBusy()) {
            updateEverything();
        }
    }

    private void waitMs(long ms) {
        long endTime = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < endTime) {
            updateEverything();
        }
    }

    private void shootThreeBalls() {
        shooterSystem.Shoot();
        transferSystem.TurnOn();
        double voltage = batteryVoltageSensor.getVoltage();
        long shootTime = (long) (1000 * (NOMINAL_VOLTAGE / voltage));
        waitMs(shootTime);
        transferSystem.TurnOFF();
        shooterSystem.Stop();
    }
}
