package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
//import com.acmerobotics.roadrunner.drive.DriveSignal;
//import com.acmerobotics.roadrunner.MecanumDrive;
//import com.acmerobotics.roadrunner.Localizer;
import com.acmerobotics.roadrunner.Pose2d;

@TeleOp(name="CPUplsCook", group="Linear OpMode")
@Config
@Disabled
public class CPUplsCook extends LinearOpMode {

    // --- Gamepad 1 drive motors ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- Wheel brake ---
    public static boolean wheelBreak = false;

    // --- Odometry encoders (no motors attached, just encoder readings) ---
    private DcMotor intake, intake2, shooter;
    private Servo shooterHinge;
    private CRServo intakeToShooter, intakeToShooter2;

    public static boolean intakeActive = false;
    public static boolean shooterActive = false;
    public static boolean shooterUp = false;

    public static double intake_speed = 0.5;
    public static double shooter_power = 0.5;
    public static double intakeToShooter_power = 0.5;

    // --- Odometry constants ---
    public static double TICKS_PER_INCH = 337.2;
    public static double TRACK_WIDTH = 13.5;
    public static double BACK_WHEEL_OFFSET = 8;

    private double xPos = 0, yPos = 0, heading = 0;
    private int prevLeft = 0, prevRight = 0, prevBack = 0;

    // --- Runtime ---
    private final ElapsedTime runtime = new ElapsedTime();

    // --- Dashboard ---
    private FtcDashboard dashboard;

    public static boolean slow_mode = false;
    public static boolean robot_centric = true;
    public static boolean field_centric = false;
    public static double kP = 0.01;
    public static double maxPower = 0.2;
    public static int maxError = 100;

    // --- PID for position hold ---
    public static double hold_kP = 0.05;
    public static double hold_maxPower = 0.3;
    public static double hold_targetX = 36;
    public static double hold_targetY = -36;

    // --- Road Runner drive ---
    private MecanumDrive rrDrive;

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) result = Math.min(result, voltage);
        }
        return result;
    }

    private void updateOdometry() {
        int leftPos = -1*(intake.getCurrentPosition());
        int rightPos = 1*(intake2.getCurrentPosition());
        int backPos = 1*(shooter.getCurrentPosition());

        int deltaLeft = leftPos - prevLeft;
        int deltaRight = rightPos - prevRight;
        int deltaBack = backPos - prevBack;

        prevLeft = leftPos;
        prevRight = rightPos;
        prevBack = backPos;

        double dLeft = deltaLeft / TICKS_PER_INCH;
        double dRight = deltaRight / TICKS_PER_INCH;
        double dBack = deltaBack / TICKS_PER_INCH;

        double dHeading = (dRight - dLeft) / TRACK_WIDTH;
        double dForward = (dLeft + dRight) / 2.0;
        double dSide = dBack - (dHeading * BACK_WHEEL_OFFSET);

        double sinHeading = Math.sin(heading);
        double cosHeading = Math.cos(heading);

        xPos += dForward * cosHeading - dSide * sinHeading;
        yPos += dForward * sinHeading + dSide * cosHeading;
        heading += dHeading;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backl");
        backRightDrive = hardwareMap.get(DcMotor.class, "backr");

        intake = hardwareMap.get(DcMotor.class, "i");
        intake2 = hardwareMap.get(DcMotor.class, "i2");
        shooter = hardwareMap.get(DcMotor.class, "s");
        shooterHinge = hardwareMap.get(Servo.class, "sH");
        intakeToShooter = hardwareMap.get(CRServo.class, "its");
        intakeToShooter2 = hardwareMap.get(CRServo.class, "its2");

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        prevLeft = intake.getCurrentPosition();
        prevRight = intake2.getCurrentPosition();
        prevBack = shooter.getCurrentPosition();

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        dashboard = FtcDashboard.getInstance();

//        rrDrive = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();
        runtime.reset();

        double nerf = 0.75;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        while (opModeIsActive()) {
            updateOdometry();
            double batteryVoltage = getBatteryVoltage();

            double Logdrive = -gamepad1.left_stick_y * nerf;
            double LATdrive = -gamepad1.left_stick_x * nerf;
            double Turndrive = -gamepad1.right_stick_x * nerf;

            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad1.a && robot_centric) {
                robot_centric = false;
                field_centric = true;
                sleep(200);
            } else if (gamepad1.y && field_centric) {
                field_centric = false;
                robot_centric = true;
                sleep(200);
            }

            dashboard.sendTelemetryPacket(packet);

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                wheelBreak = !wheelBreak;
                sleep(200);
            }

            if(!slow_mode && gamepad1.right_bumper){
                nerf = 0.1;
                slow_mode = true;
            } else if (slow_mode && gamepad1.right_bumper) {
                nerf = 0.75;
                slow_mode = false;
            }

            // --- PID-based Wheel Brake ---
            if (wheelBreak) {
                if (wheelBreak) {
                    // Set current odometry to RR localizer
//                    rrDrive.setPoseEstimate(new Pose2d(xPos, yPos, heading));
//
//                    // Define target pose to hold
//                    Pose2d targetPose = new Pose2d(hold_targetX, hold_targetY, 0);
//
//                    // Generate drive signal for target pose
//                    DriveSignal signal = rrDrive.getDriveSignalForPose(targetPose);
//
//                    // Send drive signal to motors
//                    rrDrive.setDriveSignal(signal);
//
//                    telemetry.addData("Wheel Brake Active", true);
//                    telemetry.addData("Target X", targetPose.getX());
//                    telemetry.addData("Target Y", targetPose.getY());
                    telemetry.addData("Current X", xPos);
                    telemetry.addData("Current Y", yPos);
                }
//                applyPositionHold();
//                telemetry.addData("WHEEL BRAKE ACTIVE", true);
//                telemetry.addData("Target X", hold_targetX);
//                telemetry.addData("Target Y", hold_targetY);
//                telemetry.addData("Current X", xPos);
//                telemetry.addData("Current Y", yPos);
            }
            else if (!wheelBreak && robot_centric) {
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                backLeftDrive.setPower(Logdrive + LATdrive - Turndrive);
                backRightDrive.setPower(Logdrive - LATdrive + Turndrive);
                frontLeftDrive.setPower(Logdrive - LATdrive - Turndrive);
                frontRightDrive.setPower(Logdrive + LATdrive + Turndrive);
            }
            else if (!wheelBreak && field_centric) {
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                double y = -gamepad1.left_stick_y * nerf;
                double x = gamepad1.left_stick_x * nerf;
                double rx = gamepad1.right_stick_x * nerf;

                if (gamepad1.start) imu.resetYaw();

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX = rotX * 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftDrive.setPower(frontLeftPower);
                backLeftDrive.setPower(backLeftPower);
                frontRightDrive.setPower(frontRightPower);
                backRightDrive.setPower(backRightPower);
            }

            handleIntake();
            handleShooter();
            handleShooterHinge();

            sendDashboardTelemetry(batteryVoltage);

            telemetry.update();
            sleep(20);
        }
    }

//    private void applyPositionHold() {
//        double errorX = hold_targetX - xPos;
//        double errorY = hold_targetY - yPos;
//
//        double powerX = Range.clip(hold_kP * errorX, -hold_maxPower, hold_maxPower);
//        double powerY = Range.clip(hold_kP * errorY, -hold_maxPower, hold_maxPower);
//
//        frontLeftDrive.setPower(powerY + powerX);
//        backLeftDrive.setPower(powerY - powerX);
//        frontRightDrive.setPower(powerY - powerX);
//        backRightDrive.setPower(powerY + powerX);
//    }

    private void handleIntake() {
        if (gamepad2.left_bumper && !intakeActive) {
            sleep(200);
            intake.setPower(intake_speed);
            intake2.setPower(intake_speed);
            intakeActive = true;
        } else if (gamepad2.left_bumper && intakeActive) {
            sleep(200);
            intake.setPower(0);
            intake2.setPower(0);
            intakeActive = false;
        }
    }

    private void handleShooter() {
        if (gamepad2.right_bumper) {
            sleep(200);
            shooterActive = !shooterActive;
            shooter.setPower(shooterActive ? shooter_power : 0);
            intakeToShooter.setPower(shooterActive ? intakeToShooter_power : 0);
            intakeToShooter2.setPower(shooterActive ? intakeToShooter_power : 0);
        }
    }

    private void handleShooterHinge() {
        if (gamepad2.a) {
            sleep(200);
            shooterUp = !shooterUp;
            shooterHinge.setPosition(shooterUp ? 1 : 0);
        }
    }

    private void sendDashboardTelemetry(double batteryVoltage) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        canvas.setStroke("#404040");
        for (int i = -72; i <= 72; i += 24) {
            canvas.strokeLine(i, -72, i, 72);
            canvas.strokeLine(-72, i, 72, i);
        }

        canvas.setStroke("#FFFFFF");
        canvas.strokeRect(-72, -72, 144, 144);

        canvas.setStroke("#FFFF00");
        canvas.strokeLine(-10, 0, 10, 0);
        canvas.strokeLine(0, -10, 0, 10);

        double robotSize = 18;
        canvas.setStroke("#3FBAFF");
        canvas.setFill("#3FBAFF");
        canvas.fillRect(xPos - robotSize / 2, yPos - robotSize / 2, robotSize, robotSize);

        double headingLineLength = 12;
        double headingX = xPos + headingLineLength * Math.cos(heading);
        double headingY = yPos + headingLineLength * Math.sin(heading);
        canvas.setStroke("#FF0000");
        canvas.setStrokeWidth(3);
        canvas.strokeLine(xPos, yPos, headingX, headingY);

        canvas.setStroke("#00FF00");
        canvas.fillCircle(xPos, yPos, 3);

        packet.put("Wheel Brake Active", wheelBreak);
        packet.put("Intake Active", intakeActive);
        packet.put("Shooter Active", shooterActive);
        packet.put("Shooter Hinge Position", shooterHinge.getPosition());
        packet.put("Robot X (in)", xPos);
        packet.put("Robot Y (in)", yPos);
        packet.put("Heading (rad)", heading);
        packet.put("Heading (deg)", Math.toDegrees(heading));
        packet.put("Odo Left Raw", -intake.getCurrentPosition());
        packet.put("Odo Right Raw", intake2.getCurrentPosition());
        packet.put("Odo Back Raw", shooter.getCurrentPosition());
        packet.put("Nerf Speed", 0.5);
        packet.put("Slow Mode", slow_mode);
        packet.put("Battery Voltage (V)", batteryVoltage);

        dashboard.sendTelemetryPacket(packet);
    }
}
