package org.firstinspires.ftc.teamcode.TeleOPs.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "AutoBlue21")
public class AutoBlue21 extends LinearOpMode {
    private Follower follower;
    private ShooterSystem shooterSystem;
    private ShooterController shooterController;
    private TransferSystem transferSystem;
    private VoltageSensor batteryVoltageSensor;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private static final double NOMINAL_VOLTAGE = 12.0;

    private PathChain GoShoot1, GoShoot2, GoShoot3, GoShoot4, GoShoot5;
    private PathChain GoIntake1, GoPreIntake2, GoIntake2, GoCycle, GoTele;

    //Constants
    private final Pose StartPose    = new Pose(25, 126, Math.toRadians(180));
    private final Pose ControlPoint = new Pose(43, 64, Math.toRadians(163));
    private final Pose Shoot1       = new Pose(55, 82, Math.toRadians(220));
    private final Pose Shoot2       = new Pose(55, 82, Math.toRadians(220));
    private final Pose PreIntake1   = new Pose(50, 82, Math.toRadians(180));
    private final Pose Intake1      = new Pose(16, 82, Math.toRadians(180));
    private final Pose PreIntake2   = new Pose(50, 60, Math.toRadians(180));
    private final Pose Intake2      = new Pose(16, 60, Math.toRadians(180));
    private final Pose Cycle        = new Pose(6.5, 59, Math.toRadians(155));
    private final Pose TeleOp       = new Pose(72, 90, Math.toRadians(180));

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

        shooterController.zero = false;
        shooterController.lime = false;
        shooterSystem.enabled = true;

        buildPaths();
        waitForStart();
        loopTimer.reset();
        if (isStopRequested()) return;

        follower.setMaxPower(1);
        shooterSystem.Shoot();
        followPath(GoShoot1);
        shootThreeBalls();
        followPath(GoPreIntake2);
        transferSystem.TurnOn();
        followPath(GoIntake2);
        transferSystem.TurnOn();
        followPath(GoShoot3);
        shootThreeBalls();

        transferSystem.TurnOn();
        followPathWithIntake(GoCycle);
        waitWithIntake(750);
        followPathWithIntake(GoShoot4);
        shootThreeBalls();

        followPathWithIntake(GoIntake1);
        followPath(GoShoot2);
        shootThreeBalls();

        followPathWithIntake(GoCycle);
        waitWithIntake(750);
        followPathWithIntake(GoShoot4);
        shootThreeBalls();

        followPathWithIntake(GoCycle);
        waitWithIntake(750);
        followPathWithIntake(GoShoot4);
        shootThreeBalls();

        followPathWithIntake(GoCycle);
        waitWithIntake(750);
        followPathWithIntake(GoShoot5);
        shootThreeBalls();

        shooterController.zero = true;
        followPath(GoTele);
    }
    private void buildPaths() {
        GoShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(StartPose, Shoot1))
                .setLinearHeadingInterpolation(StartPose.getHeading(), Shoot1.getHeading())
                .build();
        GoShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(Intake1, Shoot1))
                .setLinearHeadingInterpolation(Intake1.getHeading(), Shoot1.getHeading())
                .build();
        GoShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(Intake2, Shoot1))
                .setLinearHeadingInterpolation(Intake2.getHeading(), Shoot1.getHeading())
                .build();
        GoShoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(Cycle, ControlPoint, Shoot2))
                .setLinearHeadingInterpolation(Cycle.getHeading(), Shoot2.getHeading())
                .build();
        GoShoot5 = follower.pathBuilder()
                .addPath(new BezierCurve(Cycle, ControlPoint, Shoot2))
                .setLinearHeadingInterpolation(Cycle.getHeading(), Shoot2.getHeading())
                .build();
        GoIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(PreIntake1, Intake1))
                .setLinearHeadingInterpolation(PreIntake1.getHeading(), Intake1.getHeading())
                .build();
        GoPreIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot1, PreIntake2))
                .setLinearHeadingInterpolation(Shoot1.getHeading(), PreIntake2.getHeading())
                .build();
        GoIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(PreIntake2, Intake2))
                .setLinearHeadingInterpolation(PreIntake2.getHeading(), Intake2.getHeading())
                .build();
        GoCycle = follower.pathBuilder()
                .addPath(new BezierCurve(Shoot1, ControlPoint, Cycle))
                .setLinearHeadingInterpolation(Shoot1.getHeading(), Cycle.getHeading())
                .build();
        GoTele = follower.pathBuilder()
                .addPath(new BezierLine(Shoot1, TeleOp))
                .setLinearHeadingInterpolation(Shoot1.getHeading(), TeleOp.getHeading())
                .build();
    }

    private void waitWithIntake(long baseMs) {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        double voltage = batteryVoltageSensor.getVoltage();
        long ms = (long) (baseMs * (NOMINAL_VOLTAGE / voltage));

        transferSystem.TurnOn();

        long endTime = System.currentTimeMillis() + ms;
        while (!isStopRequested() && System.currentTimeMillis() < endTime) {
            loopTimer.reset();
            follower.update();
            shooterSystem.updateShooter(shooterController.Distance(8,143));
            shooterController.update(8,143,shooterSystem.Vel);
            shooterSystem.update(dt);
            idle();
        }

        transferSystem.TurnOFF();
    }

    private void followPathWithIntake(PathChain path) {
        follower.followPath(path, true);
        transferSystem.TurnOn();
        while (!isStopRequested() && follower.isBusy()) {
            transferSystem.TurnOn();
            double dt = loopTimer.seconds();
            loopTimer.reset();
            follower.update();
            shooterSystem.updateShooter(shooterController.Distance(4,143));
            shooterController.update(4,143,shooterSystem.Vel);
            shooterSystem.update(dt);

            telemetryUpdate(shooterController.Distance(4,143));
            idle();
        }
    }

    private void followPath(PathChain path) {
        follower.followPath(path, true);
        while (!isStopRequested() && follower.isBusy()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            follower.update();
            shooterSystem.updateShooter(shooterController.Distance(4,143));
            shooterController.update(4,143,shooterSystem.Vel);
            shooterSystem.update(dt);

            telemetryUpdate(shooterController.Distance(4,143));
            idle();
        }
    }
    private void waitMs(long ms) {
        long endTime = System.currentTimeMillis() + ms;
        while (!isStopRequested() && System.currentTimeMillis() < endTime) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            follower.update();
            shooterSystem.updateShooter(shooterController.Distance(4,143));
            shooterController.update(4,143,shooterSystem.Vel);
            shooterSystem.update(dt);

            telemetryUpdate(shooterController.Distance(4,143));
            idle();
        }
    }

    private void shootThreeBalls() {
        shooterSystem.Shoot();
        transferSystem.TurnOn();
        transferSystem.TurnOn();
        waitMs(200);
        long shootMs = 750;
        long endTime = System.currentTimeMillis() + shootMs;
        while (!isStopRequested() && System.currentTimeMillis() < endTime) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            follower.update();
            shooterSystem.updateShooter(shooterController.Distance(4,143));
            shooterController.update(4,143,shooterSystem.Vel);
            shooterSystem.update(dt);
            telemetryUpdate(shooterController.Distance(4,143));
            idle();
        }
        transferSystem.TurnOFF();
        shooterSystem.Stop();
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
        telemetry.addData("dt",loopTimer);
        telemetry.update();
    }
}
