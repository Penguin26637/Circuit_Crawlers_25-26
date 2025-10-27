package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.canvas.Canvas;

@TeleOp(name="CPplsCookExpansive", group="Linear OpMode")
@Config
public class CPplsCookExpansive extends LinearOpMode {

    // --- Gamepad 1 drive motors ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // --- Wheel brake ---
    public static boolean wheelBreak = false;
    public static int wheelBreakTargetFL, wheelBreakTargetFR, wheelBreakTargetBL, wheelBreakTargetBR;
    public static double wheelBreak_kP = 0.01;
    public static double wheelBreak_maxPower = 0.2;
    public static int wheelBreak_maxError = 100;

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
    public static double TICKS_PER_INCH = 1377; // REV Odometry Pod 48mm wheel
    public static double TRACK_WIDTH = 12;      // distance between left/right wheels (inches)
    public static double BACK_WHEEL_OFFSET = 7; // distance from center (inches)

    private double xPos = 0, yPos = 0, heading = 0;
    private int prevLeft = 0, prevRight = 0, prevBack = 0;

    // --- Runtime ---
    private ElapsedTime runtime = new ElapsedTime();

    // --- Dashboard ---
    private FtcDashboard dashboard;

    public static boolean slow_mode = false;

    private void updateOdometry() {
        // --- Read encoder values ---
        int leftPos = intake.getCurrentPosition();
        int rightPos = intake2.getCurrentPosition();
        int backPos = shooter.getCurrentPosition();

        int deltaLeft = leftPos - prevLeft;
        int deltaRight = rightPos - prevRight;
        int deltaBack = backPos - prevBack;

        prevLeft = leftPos;
        prevRight = rightPos;
        prevBack = backPos;

        // --- Convert ticks to inches ---
        double dLeft = deltaLeft / TICKS_PER_INCH;
        double dRight = deltaRight / TICKS_PER_INCH;
        double dBack = deltaBack / TICKS_PER_INCH;

        // --- Odometry math ---
        double dHeading = (dRight - dLeft) / TRACK_WIDTH;
        double dForward = (dLeft + dRight) / 2.0;
        double dSide = dBack - (dHeading * BACK_WHEEL_OFFSET);

        // Update global position
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

        // --- Hardware Mapping ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backl");
        backRightDrive = hardwareMap.get(DcMotor.class, "backr");

        // Odometry pods (connected to motor ports but no motors attached)
        intake = hardwareMap.get(DcMotor.class, "i");
        intake2 = hardwareMap.get(DcMotor.class, "i2");
        shooter = hardwareMap.get(DcMotor.class, "s");
        shooterHinge = hardwareMap.get(Servo.class, "sH");
        intakeToShooter = hardwareMap.get(CRServo.class, "its");
        intakeToShooter2 = hardwareMap.get(CRServo.class, "its2");

        // --- Odometry encoder setup ---
        // Try reading before any mode changes
        telemetry.addData("Before Reset - Left", intake.getCurrentPosition());
        telemetry.addData("Before Reset - Right", intake2.getCurrentPosition());
        telemetry.addData("Before Reset - Back", shooter.getCurrentPosition());
        telemetry.update();
        sleep(1000);

        // Reset encoders
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(100); // Give time for reset

        // CRITICAL: Set to RUN_WITHOUT_ENCODER to read raw encoder values
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set to zero power (we're only reading encoders, not running motors)
        intake.setPower(0);
        intake2.setPower(0);
        shooter.setPower(0);

        // Initialize previous positions after reset
        prevLeft = intake.getCurrentPosition();
        prevRight = intake2.getCurrentPosition();
        prevBack = shooter.getCurrentPosition();

        telemetry.addData("After Reset - Left", prevLeft);
        telemetry.addData("After Reset - Right", prevRight);
        telemetry.addData("After Reset - Back", prevBack);
        telemetry.addData("Left Direction", intake.getDirection());
        telemetry.addData("Right Direction", intake2.getDirection());
        telemetry.addData("Back Direction", shooter.getDirection());
        telemetry.update();
        sleep(2000);

        // --- Motor directions ---
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        dashboard = FtcDashboard.getInstance();
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // --- Update odometry ---
            updateOdometry();

            // --- Driving control ---
            double nerf = 0.1;
            double Logdrive = -gamepad1.left_stick_y * nerf;
            double LATdrive = -gamepad1.left_stick_x * nerf;
            double Turndrive = -gamepad1.right_stick_x * 0.6;

            // Apply mecanum drive
            double flPower = Logdrive + LATdrive + Turndrive;
            double frPower = Logdrive - LATdrive - Turndrive;
            double blPower = Logdrive - LATdrive + Turndrive;
            double brPower = Logdrive + LATdrive - Turndrive;

            frontLeftDrive.setPower(flPower);
            frontRightDrive.setPower(frPower);
            backLeftDrive.setPower(blPower);
            backRightDrive.setPower(brPower);

            if (gamepad2.left_bumper && !intakeActive) {
                sleep(200);
                intake.setPower(intake_speed);
                intake2.setPower(intake_speed);

                telemetry.addData("intake 2", "power" + intake2.getPower());
                intakeActive = true;
            } else if (gamepad2.left_bumper && intakeActive) {
                sleep(200);
                intake.setPower(0);
                intake2.setPower(0);
                intakeActive = false;
            }

            if (gamepad2.right_bumper) {
                sleep(200);
                shooterActive = !shooterActive;
                shooter.setPower(shooterActive ? shooter_power : 0);
                intakeToShooter.setPower(shooterActive ? intakeToShooter_power : 0);
                intakeToShooter2.setPower(shooterActive ? intakeToShooter_power : 0);
            }

            // --- Shooter hinge ---
            if (gamepad2.a) {
                sleep(200);
                shooterUp = !shooterUp;
                shooterHinge.setPosition(shooterUp ? 1 : 0);
            }

            if (gamepad1.right_bumper && !slow_mode) {
                sleep(200);
                nerf = 0.1;
                slow_mode = true;
            }
            else if(gamepad1.right_bumper && slow_mode) {
                sleep(200);
                nerf = 0.5;
                slow_mode = false;
            }

            // --- Dashboard telemetry ---
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            // FTC Field is 144" x 144" (12 ft x 12 ft)
            // Dashboard field overlay: -72 to +72 inches from center

            // Always draw a reference grid to see if canvas is working
            canvas.setStroke("#404040");
            for (int i = -72; i <= 72; i += 24) {
                canvas.strokeLine(i, -72, i, 72);  // Vertical lines
                canvas.strokeLine(-72, i, 72, i);  // Horizontal lines
            }

            // Draw field border
            canvas.setStroke("#FFFFFF");
            canvas.strokeRect(-72, -72, 144, 144);

            // Draw origin marker
            canvas.setStroke("#FFFF00"); // Yellow
            canvas.strokeLine(-10, 0, 10, 0);
            canvas.strokeLine(0, -10, 0, 10);

            // Draw robot at current position
            double robotSize = 18; // inches (18x18 robot)

            canvas.setStroke("#3FBAFF"); // Blue outline
            canvas.setFill("#3FBAFF");
            canvas.fillRect(xPos - robotSize/2, yPos - robotSize/2, robotSize, robotSize);

            // Draw heading indicator (front of robot)
            double headingLineLength = 12; // inches
            double headingX = xPos + headingLineLength * Math.cos(heading);
            double headingY = yPos + headingLineLength * Math.sin(heading);

            canvas.setStroke("#FF0000"); // Red heading line
            canvas.setStrokeWidth(3);
            canvas.strokeLine(xPos, yPos, headingX, headingY);

            // Draw center point
            canvas.setStroke("#00FF00"); // Green center dot
            canvas.fillCircle(xPos, yPos, 3);

            packet.put("Wheel Brake Active", wheelBreak);
            packet.put("Intake Active", intakeActive);
            packet.put("Shooter Active", shooterActive);
            packet.put("Shooter Hinge Position", shooterHinge.getPosition());
            packet.put("Robot X (in)", xPos);
            packet.put("Robot Y (in)", yPos);
            packet.put("Heading (rad)", heading);
            packet.put("Heading (deg)", Math.toDegrees(heading));
            packet.put("Front Left Encoder", frontLeftDrive.getCurrentPosition());
            packet.put("Front Right Encoder", frontRightDrive.getCurrentPosition());
            packet.put("Back Left Encoder", backLeftDrive.getCurrentPosition());
            packet.put("Back Right Encoder", backRightDrive.getCurrentPosition());
            packet.put("Odo Left Raw", intake.getCurrentPosition());
            packet.put("Odo Right Raw", intake2.getCurrentPosition());
            packet.put("Odo Back Raw", shooter.getCurrentPosition());
            packet.put("Nerf Speed", nerf);
            packet.put("Slow Mode", slow_mode);
            packet.put("Wireless", "True");

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Wheel Brake Active", wheelBreak);
            telemetry.addData("Intake Active", intakeActive);
            telemetry.addData("Shooter Active", shooterActive);
            telemetry.addData("Shooter Hinge Position", shooterHinge.getPosition());
            telemetry.addData("X Pos (in)", xPos);
            telemetry.addData("Y Pos (in)", yPos);
            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Front Left Encoder", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front Right Encoder", frontRightDrive.getCurrentPosition());
            telemetry.addData("Back Left Encoder", backLeftDrive.getCurrentPosition());
            telemetry.addData("Back Right Encoder", backRightDrive.getCurrentPosition());
            telemetry.addData("Odo Left", intake.getCurrentPosition());
            telemetry.addData("Odo Right", intake2.getCurrentPosition());
            telemetry.addData("Odo Back", shooter.getCurrentPosition());
            telemetry.addData("Nerf Speed", nerf);
            telemetry.addData("Slow Mode", slow_mode);
            telemetry.addData("Wireless", "True");
            telemetry.update();

            sleep(20);
        }
    }
}