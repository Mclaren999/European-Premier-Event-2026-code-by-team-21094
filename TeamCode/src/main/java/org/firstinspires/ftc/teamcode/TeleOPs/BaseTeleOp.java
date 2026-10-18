package org.firstinspires.ftc.teamcode.TeleOPs;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad.GamepadSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterController;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSystem;

@Config
public abstract class BaseTeleOp extends OpMode {

    protected abstract double getStartX();
    protected abstract double getStartY();
    protected abstract double getStartHeadingRad();
    protected abstract double getGoalX();
    protected abstract double getGoalY();
    protected abstract double getTagFieldX();
    protected abstract double getTagFieldY();
    protected abstract int getLimelightPipeline();
    protected abstract boolean isBlue();

    public static double goalIncX = 0, goalIncY = 0;

    public static double FAR_MULT = 1.02;

    public static double HEADING_KP = 0.01;
    public static double HEADING_KD = 0.002;
    public static double TURN_SPEED_DEG_PER_SEC = 360;
    public static double HEADING_DEADBAND_DEG = 4;

    private Limelight3A limelight;
    protected Follower follower;
    protected ShooterSystem shooter;
    private TransferSystem transferSystem;
    private FtcDashboard dashboard;
    private PIDController headingPID;
    protected ShooterController shooterController;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean r2Last = false;
    private boolean hangLast = false;
    private boolean isParking = false;
    private boolean xSnapLast = false;
    private double headingTargetDeg;
    private GamepadSubSystem driveOP;

    @Override
    public void init() {
        driveOP = new GamepadSubSystem(gamepad1);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(getLimelightPipeline());

        shooter = new ShooterSystem(hardwareMap);
        transferSystem = new TransferSystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        Tuning.follower = follower;
        follower.setPose(new Pose(getStartX(), getStartY(), getStartHeadingRad()));
        follower.startTeleOpDrive(true);
        follower.setMaxPower(1);
        shooterController = new ShooterController(hardwareMap, follower, shooter);

        headingPID = new PIDController(HEADING_KP, 0, HEADING_KD);
        headingPID.setTolerance(HEADING_DEADBAND_DEG);
        headingTargetDeg = Math.toDegrees(follower.getPose().getHeading());

        shooterController.calm();
        shooterController.zero = false;
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        driving(dt);

        shooterController.update(getGoalX() + goalIncX, getGoalY() + goalIncY, shooter.Vel);
        shooter.calculateVel(shooterController.Distance(getGoalX() + goalIncX, getGoalY() + goalIncY));

        LLResult vision = limelight.getLatestResult();
        double ty = 0;
        double disLime = 0;
        if (vision != null) {
            ty = vision.getTy();
            disLime = shooter.calcDisLime(ty);
            if (gamepad1.dpad_right) {
                shooterController.computeRobotPosition(disLime, vision.getTx(), getTagFieldX(), getTagFieldY());
                shooterController.zero = false;
                shooterController.lime = true;
            }
        }

        handleShooter();
        shooter.update(dt);
        handleTransfer();
        parking();

        if (follower.getPose().getY() < 40) {
            shooterController.zero = false;
            shooterController.lime = true;
        } else {
            shooterController.lime = false;
        }
        if (gamepad1.dpad_up) {
            shooterController.zero = true;
            shooterController.lime = false;
        }
        if (gamepad1.dpad_left) {
            shooterController.lime = true;
        }

        telemetry.addData("Goal X", getGoalX() + goalIncX);
        telemetry.addData("Goal Y", getGoalY() + goalIncY);
//        telemetry.addData("Pose", follower.getPose());
//        telemetry.addData("Vel (robot)", follower.getVelocity().getMagnitude());
//        telemetry.addData("Turret Target", shooterController.turretTargetPosition);
//        telemetry.addData("Turret Current", shooterController.getTurretPositionDeg());
//        telemetry.addData("Field Angle", shooterController.fieldAngle);
//        telemetry.addData("Angle", shooterController.Angle);
//        telemetry.addData("Shooter Vel", shooter.Vel);
//        telemetry.addData("Dis (lime)", disLime);
//        telemetry.addData("ty", ty);
//        telemetry.addData("RPM", shooter.GetRPM());
//        telemetry.addData("Target RPM", shooter.TARGET_RPM);
//        telemetry.addData("lime", shooterController.lime);
        telemetry.update();
    }

    private void driving(double dt) {
        driveOP.readButtons();
        double turnStick = driveOP.getRightX(dt);
        double forward = driveOP.getLeftY(dt);
        double strafe = driveOP.getLeftX(dt);
        double mult = gamepad1.right_bumper ? 0.2 : 1.0;

        if (isBlue()) {
            if (driveOP.wasJustPressed(GamepadKeys.Button.Y)) goalIncX--;
            if (driveOP.wasJustPressed(GamepadKeys.Button.B)) goalIncY++;
            if (driveOP.wasJustPressed(GamepadKeys.Button.A)) goalIncX++;
            if (driveOP.wasJustPressed(GamepadKeys.Button.X)) goalIncY--;
        } else {
            if (driveOP.wasJustPressed(GamepadKeys.Button.Y)) goalIncX++;
            if (driveOP.wasJustPressed(GamepadKeys.Button.B)) goalIncY--;
            if (driveOP.wasJustPressed(GamepadKeys.Button.A)) goalIncX--;
            if (driveOP.wasJustPressed(GamepadKeys.Button.X)) goalIncY++;
        }

        double currentHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        headingTargetDeg += turnStick * TURN_SPEED_DEG_PER_SEC * dt;

        double headingError = driveOP.angleErrorDeg(currentHeadingDeg, headingTargetDeg);
        if (abs(headingError) < HEADING_DEADBAND_DEG) headingError = 0;
        double turnPower = -headingPID.calculate(headingError, 0);

        double driveSign = isBlue() ? 1.0 : -1.0;
        follower.setTeleOpDrive(driveSign * forward * mult, driveSign * strafe * mult, turnPower * mult, false);
        follower.update();
    }

    private void handleShooter() {
        boolean r2 = gamepad1.dpad_down;
        if (r2 && !r2Last) shooter.toggle();
        r2Last = r2;
    }

    private void parking() {
        boolean press = gamepad1.options;
        if (press && !hangLast) {
            isParking = !isParking;
            if (isParking) shooterController.hang();
            else shooterController.calm();
        }
        hangLast = press;
    }

    private void handleTransfer() {
        if (gamepad1.right_trigger > 0.5) {
            shooter.Shoot();
            transferSystem.Shoot();
            return;
        }
        if (gamepad1.left_bumper) {
            shooter.Stop();
            transferSystem.Reverse();
            return;
        }
        if (gamepad1.left_trigger > 0.5) {
            shooter.Stop();
            transferSystem.TurnOn();
            return;
        }
        shooter.Stop();
        transferSystem.TurnOFF();
    }
}
