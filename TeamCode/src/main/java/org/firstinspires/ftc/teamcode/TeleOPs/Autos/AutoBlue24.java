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

@Autonomous
public class AutoBlue24 extends LinearOpMode {

    private Follower follower;
    private ShooterSystem shooterSystem;
    private ShooterController shooterController;
    private TransferSystem transferSystem;
    private VoltageSensor batteryVoltageSensor;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private static final double NOMINAL_VOLTAGE = 12.0;

    private final Pose StartPose = new Pose(19, 122, Math.toRadians(143));
    private final Pose Shoot     = new Pose(57, 85, Math.toRadians(143));
    private final Pose Shoot2    = new Pose(57, 85, Math.toRadians(220));
    private final Pose Intake2   = new Pose(20, 56, Math.toRadians(213));
    private final Pose Intake1   = new Pose(25, 85, Math.toRadians(180));
    private final Pose Cycle     = new Pose(10, 61, Math.toRadians(156));
    private final Pose TeleOp    = new Pose(16, 85, Math.toRadians(180));

    private PathChain GoShoot1, AfterIntake2, AfterIntake1, CycleAfter, GoPreIntake1, GoIntake1, GoShoot2, GoPreIntake2, GoIntake2, GoShoot3, GoCycle, GoShoot4, GoTele;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(StartPose);

        shooterSystem = new ShooterSystem(hardwareMap);
        shooterController = new ShooterController(hardwareMap, follower, shooterSystem);
        transferSystem = new TransferSystem(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        shooterController.auto();

        shooterController.zero = false;
        shooterController.lime = false;
        shooterSystem.enabled = true;

        buildPaths();
        waitForStart();
        if (isStopRequested()) return;

        loopTimer.reset();

        shooterSystem.Shoot();
        followPath(GoShoot1);
        startShoot();

        turnThenGo(Intake2);
        followPathWithIntake(AfterIntake2);
        shootThreeBalls();

        turnThenGo(Intake1);
        followPathWithIntake(AfterIntake1);
        shootThreeBalls();

        followPathWithIntake(GoCycle);
        waitWithIntake(2000);
        turnThenGo(Shoot2);
        shootThreeBalls();
        followPathWithIntake(GoCycle);
        waitWithIntake(2000);
        turnThenGo(Shoot2);
        shootThreeBalls();
        followPathWithIntake(GoCycle);
        waitWithIntake(2000);
        turnThenGo(Shoot2);
        shootThreeBalls();
        followPathWithIntake(GoCycle);
        waitWithIntake(2000);
        turnThenGo(Shoot2);
        shootThreeBalls();
        followPath(GoTele);
    }

    private void turnThenGo(Pose target) {
        Pose current = follower.getPose();

        PathChain turn = follower.pathBuilder()
                .addPath(new BezierLine(current, new Pose(current.getX() + 0.01, current.getY(), current.getHeading())))
                .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                .build();

        followPathWithIntake(turn);

        Pose afterTurn = follower.getPose();

        PathChain go = follower.pathBuilder()
                .addPath(new BezierLine(afterTurn, target))
                .setConstantHeadingInterpolation(target.getHeading())
                .build();

        followPathWithIntake(go);
        while (!isStopRequested() && follower.isBusy()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            follower.update();
            shooterSystem.updateShooter(shooterController.Distance(8, 143));
            shooterController.update(8, 143, shooterSystem.Vel);
            shooterSystem.update(dt);
            telemetryUpdate(shooterController.Distance(8, 143));
            idle();
        }
    }

    private void waitWithIntake(long baseMs) {
        double voltage = batteryVoltageSensor.getVoltage();
        long ms = (long) (baseMs * (NOMINAL_VOLTAGE / voltage));
        transferSystem.TurnOn();
        long endTime = System.currentTimeMillis() + ms;
        while (!isStopRequested() && System.currentTimeMillis() < endTime) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            follower.update();
            shooterSystem.updateShooter(shooterController.Distance(8, 143));
            shooterController.update(8, 143, shooterSystem.Vel);
            shooterSystem.update(dt);
            idle();
        }
        transferSystem.TurnOFF();
    }

    private void followPathWithIntake(PathChain path) {
        follower.followPath(path, true);
        transferSystem.TurnOn();
        while (!isStopRequested() && follower.isBusy()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            follower.update();
            shooterSystem.updateShooter(shooterController.Distance(8, 143));
            shooterController.update(8, 143, shooterSystem.Vel);
            shooterSystem.update(dt);
            telemetryUpdate(shooterController.Distance(8, 143));
            idle();
        }
    }

    private void followPath(PathChain path) {
        follower.followPath(path, true);
        while (!isStopRequested() && follower.isBusy()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            follower.update();
            shooterSystem.updateShooter(shooterController.Distance(8, 143));
            shooterController.update(8, 143, shooterSystem.Vel);
            shooterSystem.update(dt);
            telemetryUpdate(shooterController.Distance(8, 143));
            idle();
        }
    }

    private void waitMs(long ms) {
        long endTime = System.currentTimeMillis() + ms;
        while (!isStopRequested() && System.currentTimeMillis() < endTime) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            follower.update();
            shooterSystem.updateShooter(shooterController.Distance(8, 143));
            shooterController.update(8, 143, shooterSystem.Vel);
            shooterSystem.update(dt);
            telemetryUpdate(shooterController.Distance(8, 143));
            idle();
        }
    }

    private void shootThreeBalls() {
        shooterSystem.Shoot();
        transferSystem.TurnOn();
        long shootMs = 600;
        long endTime = System.currentTimeMillis() + shootMs;
        while (!isStopRequested() && System.currentTimeMillis() < endTime) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            follower.update();
            shooterSystem.updateShooter(shooterController.Distance(8, 143));
            shooterController.update(8, 143, shooterSystem.Vel);
            shooterSystem.update(dt);
            telemetryUpdate(shooterController.Distance(8, 143));
            idle();
        }
        transferSystem.TurnOFF();
        shooterSystem.Stop();
    }

    private void startShoot() {
        shooterSystem.Shoot();
        transferSystem.TurnOn();
        long shootMs = 450;
        long endTime = System.currentTimeMillis() + shootMs;
        while (!isStopRequested() && System.currentTimeMillis() < endTime) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            follower.update();
            shooterSystem.updateShooter(shooterController.Distance(8, 143));
            shooterController.update(8, 143, shooterSystem.Vel);
            shooterSystem.update(dt);
            telemetryUpdate(shooterController.Distance(8, 143));
            idle();
        }
        transferSystem.TurnOFF();
        shooterSystem.Stop();
    }

    private void buildPaths() {
        GoShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(StartPose, Shoot))
                .setLinearHeadingInterpolation(StartPose.getHeading(), Shoot.getHeading())
                .build();
        AfterIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(Intake2, Shoot))
                .setLinearHeadingInterpolation(Intake2.getHeading(), Shoot2.getHeading())
                .build();
        AfterIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(Intake1, Shoot))
                .setLinearHeadingInterpolation(Intake1.getHeading(), Shoot2.getHeading())
                .build();
        CycleAfter = follower.pathBuilder()
                .addPath(new BezierLine(Cycle, Shoot))
                .setLinearHeadingInterpolation(Cycle.getHeading(), Shoot2.getHeading())
                .build();
        GoCycle = follower.pathBuilder()
                .addPath(new BezierLine(Shoot2, Cycle))
                .setLinearHeadingInterpolation(Shoot2.getHeading(), Cycle.getHeading())
                .build();
        GoTele = follower.pathBuilder()
                .addPath(new BezierLine(Shoot2, TeleOp))
                .setLinearHeadingInterpolation(Shoot2.getHeading(), TeleOp.getHeading())
                .build();
    }

    private void telemetryUpdate(double distance) {
        telemetry.addData("Distance", distance);
        telemetry.addData("Filtered RPM Target", shooterSystem.TARGET_RPM);
        telemetry.addData("Actual RPM", shooterSystem.GetRPM());
        telemetry.addData("Shooter Vel", shooterSystem.Vel);
        telemetry.addData("Turret Target", shooterController.turretTargetPosition);
        telemetry.addData("Angle", shooterController.Angle);
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("dt", loopTimer);
        telemetry.update();
    }
}
