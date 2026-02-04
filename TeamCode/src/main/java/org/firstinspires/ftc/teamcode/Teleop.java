package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//1

//left dpad slow mode
//y reset
//a robot
//x field
//b lockon

//2

//Right bumper: reset position
//a: Intake
//b: Stop launcher
@TeleOp(name = "Teleop")
//@Disabled
public class Teleop extends OpMode {
    final double FEED_TIME_SECONDS = 0.1;
    final double STOP_SPEED = 0.0;

    final double LAUNCHER_IDLE_VELOCITY = 650;
    final double LAUNCHER_MAX_VELOCITY = 1462.5;
    final double LAUNCHER_MIN_VELOCITY = 1125;
    final double LAUNCHER_SPINUP_VELOCITY = 900;
    final double LAUNCHER_REVERSE_VELOCITY = -375;
    final double FEEDER_INTAKE_VELOCITY = 3000;
    final double FEEDER_LAUNCH_VELOCITY = 3000;
    final double FEEDER_REVERSE_VELOCITY = 900;
    final double SLOW_MODE_MULTIPLIER = 0.5;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo stopper;
    private IMU imu = null;
    private DcMotorEx launcher = null;
    private DcMotorEx feeder = null;
    final double PGain = 1;
    final double DGain = 0.2;
    double xGoal = 144;
    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        SPIN_UP_FAR,
        LAUNCH,
        LAUNCH_FAR,
        LAUNCHING,
    }

    private LaunchState launchState;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    private boolean launcherIdle;
    private boolean fieldCentric;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        launcherIdle = true;
        fieldCentric = false;

        imu = (IMU) hardwareMap.get("imu");
        imu.resetYaw();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        stopper = hardwareMap.get(Servo.class, "gateServo");
        stopper.setPosition(1);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.FORWARD);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        feeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double leftStickY = gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;
        if (gamepad1.dpad_left) {
            leftStickY = gamepad1.left_stick_y * SLOW_MODE_MULTIPLIER;
            leftStickX = gamepad1.left_stick_x * SLOW_MODE_MULTIPLIER;
            rightStickX = gamepad1.right_stick_x * SLOW_MODE_MULTIPLIER;
        }
        mecuamnFieldDrive(-leftStickY, leftStickX, rightStickX);

        if (gamepad1.aWasPressed()) {
            fieldCentric = false;
        }

        if (gamepad1.xWasPressed()) {
            fieldCentric = true;
        }

        if (gamepad1.yWasPressed()) {
            imu.resetYaw();
        }

        if (gamepad2.a) {
            if (launcherIdle) {
                launcher.setDirection(DcMotor.Direction.REVERSE);
                telemetry.addData("Goal Ball Velocity", LAUNCHER_REVERSE_VELOCITY);
                launcher.setVelocity(LAUNCHER_REVERSE_VELOCITY);
                telemetry.addData("Shooter Speed", LAUNCHER_REVERSE_VELOCITY);
                stopper.setPosition(0.5);
            }
            feeder.setDirection(DcMotor.Direction.FORWARD);
            feeder.setVelocity(FEEDER_INTAKE_VELOCITY);
            if (launcher.getVelocity() > LAUNCHER_IDLE_VELOCITY) {
                feeder.setDirection(DcMotor.Direction.FORWARD);
                feeder.setVelocity(FEEDER_LAUNCH_VELOCITY);
            }
        }
        else if (gamepad2.x) {
            if (launcherIdle) {
                launcher.setDirection(DcMotor.Direction.REVERSE);
                telemetry.addData("Goal Ball Velocity", LAUNCHER_REVERSE_VELOCITY);
                launcher.setVelocity(LAUNCHER_REVERSE_VELOCITY);
                telemetry.addData("Shooter Speed", LAUNCHER_REVERSE_VELOCITY);
            }
            feeder.setDirection(DcMotor.Direction.REVERSE);
            feeder.setVelocity(FEEDER_REVERSE_VELOCITY);
        }
        else {
            if (launcherIdle) {
                launcher.setVelocity(LAUNCHER_IDLE_VELOCITY);
            }
        }

        if (gamepad2.left_bumper) {
            stopper.setPosition(1);
        }
        if (gamepad2.left_trigger >= 0.1) {
            stopper.setPosition(0.5);
        }

        if (gamepad2.b) {
            telemetry.addData("Goal Ball Velocity", LAUNCHER_IDLE_VELOCITY);
            launcher.setVelocity(LAUNCHER_IDLE_VELOCITY);
            telemetry.addData("Shooter Speed", LAUNCHER_IDLE_VELOCITY);
            //stopper.setPosition(1);
            launcherIdle = true;
        }
        if (gamepad2.y) {
            launcher.setVelocity(LAUNCHER_SPINUP_VELOCITY);
            launcherIdle = false;
        }
        if (gamepad2.dpad_up) {
            telemetry.addData("Goal Ball Velocity", "MAXIMUM");
            launcher.setVelocity(LAUNCHER_MAX_VELOCITY);
            telemetry.addData("Shooter Speed", "MAXIMUM");
            launcherIdle = false;
        }

        if (gamepad2.dpad_down) {
            telemetry.addData("Goal Ball Velocity", "MINIMUM");
            launcher.setVelocity(LAUNCHER_MIN_VELOCITY);
            telemetry.addData("Shooter Speed", "MINIMUM");
            launcherIdle = false;
        }
        else {
            feeder.setPower(STOP_SPEED);
        }
        if (gamepad2.right_trigger >= 0.1) {
            stopper.setPosition(1);
        }
        launch(gamepad2.right_trigger >= 0.1);

        telemetry.addData("State", launchState);
        telemetry.addData("motorSpeed", launcher.getVelocity());
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }

    void mecuamnFieldDrive(double forward, double strafe, double rotate) {
        if (fieldCentric) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double fieldStrafe = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double fieldForward = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
            mecanumDrive(fieldForward, fieldStrafe, rotate);
        }
        else {
            mecanumDrive(forward, strafe, rotate);
        }
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                else {
                    //stopper.setPosition(1);
                }
                break;
            case SPIN_UP:
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case SPIN_UP_FAR:
                if (launcher.getVelocity() > LAUNCHER_MAX_VELOCITY) {
                    launchState = LaunchState.LAUNCH_FAR;
                }
                break;
            case LAUNCH:
                //stopper.setPosition(0);
                feeder.setDirection(DcMotor.Direction.FORWARD);
                feeder.setVelocity(FEEDER_LAUNCH_VELOCITY);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCH_FAR:
                //stopper.setPosition(0);
                feeder.setDirection(DcMotor.Direction.FORWARD);
                feeder.setVelocity(FEEDER_LAUNCH_VELOCITY);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    feeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}