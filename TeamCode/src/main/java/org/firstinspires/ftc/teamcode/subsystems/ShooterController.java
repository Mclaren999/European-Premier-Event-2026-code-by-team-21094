package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class ShooterController {
    private final Limelight3A limelight;
    private final ShooterSystem shooterSystem;
    public boolean lime = false;
    public boolean zero = false;
    private final Follower follower;

    private RGBLight lightR;
    private RGBLight lightL;
    private Servo servoLiftL;
    private Servo servoLiftR;

    public static double TURRET_TICKS_TO_DEG = 0.0177039431;
    public static double TURRET_KPLOcal = 0.026;
    public static double TURRET_KDLocal = 0.001;
    public static double TURRET_KFLocal = 0.002;
    public static double TURRET_KPLime = 0.025;
    public static double TURRET_KDLime = 0.003;

    public static double openL = 1;
    public static double openR = 0;
    public static double closeR = 1;
    public static double closeL = 0;

    private final PDController turretController;
    private final DcMotorEx turretMotor;
    public double turretTargetPosition = 0;
    public double fieldAngle = 0;
    public double error;

    private double LimeOffset = 6.513;

    public double Angle;
    public double dx, dy;
    public double RobotX, RobotY;
    public boolean park = false;
    public static double KPD = 0.55;

    public static double LIME_VELOCITY_THRESHOLD = 8.0;

    public ShooterController(HardwareMap hardwareMap, Follower follower, ShooterSystem shooterSystem) {
        this.follower = follower;
        this.shooterSystem = shooterSystem;
        turretController = new PDController(TURRET_KPLOcal, TURRET_KDLocal);
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        lightR = new RGBLight(hardwareMap.get(Servo.class, "light1"));
        lightL = new RGBLight(hardwareMap.get(Servo.class, "light2"));
        servoLiftL = hardwareMap.get(Servo.class, "liftL");
        servoLiftR = hardwareMap.get(Servo.class, "liftR");
    }

    public void setFarMult(double farMult) {
        shooterSystem.setFAR_MULT(farMult);
    }

    public void ready() {
        lightR.blue();
        lightL.blue();
    }

    public void calm() {
        servoLiftL.setPosition(closeL);
        servoLiftR.setPosition(closeR);
        park = false;
        lightR.blue();
        lightL.blue();
    }

    public void auto() {
        lightR.violet();
        lightL.blue();
    }

    public void hang() {
        park = true;
        servoLiftL.setPosition(openL);
        servoLiftR.setPosition(openR);
    }

    private double wrap190(double angle) {
        while (angle <= -190) angle += 360;
        while (angle >= 190) angle -= 360;
        return angle;
    }

    public double getTurretPositionDeg() {
        return -TURRET_TICKS_TO_DEG * turretMotor.getCurrentPosition();
    }

    public double Distance(double goalX, double goalY) {
        if (follower == null) return 0;
        dx = goalX - follower.getPose().getX();
        dy = goalY - follower.getPose().getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    private void calculateTarget(double goalX, double goalY, double vel) {
        if (follower == null) return;
        RobotX = follower.getPose().getX();
        RobotY = follower.getPose().getY();
        dx = goalX - RobotX;
        dy = goalY - RobotY;
        fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        double dis = Distance(goalX, goalY);
        double compX = dx / dis;
        double compY = dy / dis;
        double xPerp = -compY;
        double yPerp = compX;
        double robotHeading = Math.toDegrees(follower.getHeading());
        double velPerp = xPerp * follower.getVelocity().getXComponent() + yPerp * follower.getVelocity().getYComponent();
        double denom = vel * KPD;
        if (denom < 1e-6 || Double.isNaN(denom)) {
            Angle = 0;
            shooterSystem.setTargetRPM(1000);
        } else {
            Angle = Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, velPerp / denom))));
        }
        error = (fieldAngle - Angle) - robotHeading;
        turretTargetPosition = wrap190(error);
    }

    public void computeRobotPosition(double dis, double tx, double tagFieldX, double tagFieldY) {
        double robotHeading = follower.getHeading();
        double turretAngle = Math.toRadians(getTurretPositionDeg());
        double camHeading = robotHeading + turretAngle;
        double cosC = Math.cos(camHeading);
        double sinC = Math.sin(camHeading);
        double tagFieldOffsetX = dis * cosC + tx * sinC;
        double tagFieldOffsetY = dis * sinC - tx * cosC;
        double camFieldX = tagFieldX - tagFieldOffsetX;
        double camFieldY = tagFieldY - tagFieldOffsetY;
        double robotX = camFieldX + LimeOffset * cosC;
        double robotY = camFieldY + LimeOffset * sinC;
        follower.setPose(new Pose(robotX, robotY, follower.getHeading()));
    }

    private void updateTurretTargetPosition(double tx, double goalX, double goalY, double vel) {
        double dis = Distance(goalX, goalY);
        double compX = dx / dis;
        double compY = dy / dis;
        double xPerp = -compY;
        double yPerp = compX;
        double robotHeading = Math.toDegrees(follower.getHeading());
        double velPerp = xPerp * follower.getVelocity().getXComponent() + yPerp * follower.getVelocity().getYComponent();
        double denom = vel * KPD;
        if (denom < 1e-6 || Double.isNaN(denom) || follower.getVelocity().getMagnitude() < 4.5) {
            Angle = 0;
        } else {
            Angle = Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, velPerp / denom))));
        }
        turretTargetPosition = getTurretPositionDeg() - tx - Angle;
        if (Math.abs(turretTargetPosition) >= 190) {
            turretTargetPosition = wrap190(turretTargetPosition);
        }
    }

    public void update(double goalX, double goalY, double vel) {
        turretController.setPID(TURRET_KPLOcal, 0, TURRET_KDLocal);

        if (park) {
            turretTargetPosition = 180;
        } else if (zero) {
            turretTargetPosition = 0;
        } else {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()
                    && follower.getVelocity().getMagnitude() < LIME_VELOCITY_THRESHOLD
                    && lime) {
                turretController.setPID(TURRET_KPLime, 0, TURRET_KDLime);
                updateTurretTargetPosition(result.getTx(), goalX, goalY, vel);
            } else {
                calculateTarget(goalX, goalY, vel);
            }
        }

        double rawPower = turretController.calculate(getTurretPositionDeg(), turretTargetPosition);
        double power = Math.abs(rawPower) >= 0.05 ? Range.clip(rawPower, -1, 1) : 0;
        turretMotor.setPower(power + TURRET_KFLocal * Math.signum(power));
    }
}
