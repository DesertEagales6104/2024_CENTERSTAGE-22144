

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor rightElevator = hardwareMap.dcMotor.get("rightElevator");
        DcMotor leftElevator = hardwareMap.dcMotor.get("leftElevator");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        BNO055IMU imu = new BNO055IMU() {
            @Override
            public boolean initialize(@NonNull Parameters parameters) {
                return false;
            }

            @NonNull
            @Override
            public Parameters getParameters() {
                return null;
            }

            @Override
            public void close() {

            }

            @Override
            public Orientation getAngularOrientation() {
                return null;
            }

            @Override
            public Orientation getAngularOrientation(AxesReference reference, AxesOrder order, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit) {
                return null;
            }

            @Override
            public Acceleration getOverallAcceleration() {
                return null;
            }

            @Override
            public AngularVelocity getAngularVelocity() {
                return null;
            }

            @Override
            public Acceleration getLinearAcceleration() {
                return null;
            }

            @Override
            public Acceleration getGravity() {
                return null;
            }

            @Override
            public Temperature getTemperature() {
                return null;
            }

            @Override
            public MagneticFlux getMagneticFieldStrength() {
                return null;
            }

            @Override
            public Quaternion getQuaternionOrientation() {
                return null;
            }

            @Override
            public Position getPosition() {
                return null;
            }

            @Override
            public Velocity getVelocity() {
                return null;
            }

            @Override
            public Acceleration getAcceleration() {
                return null;
            }

            @Override
            public void startAccelerationIntegration(Position initialPosition, Velocity initialVelocity, int msPollInterval) {

            }

            @Override
            public void stopAccelerationIntegration() {

            }

            @Override
            public SystemStatus getSystemStatus() {
                return null;
            }

            @Override
            public SystemError getSystemError() {
                return null;
            }

            @Override
            public CalibrationStatus getCalibrationStatus() {
                return null;
            }

            @Override
            public boolean isSystemCalibrated() {
                return false;
            }

            @Override
            public boolean isGyroCalibrated() {
                return false;
            }

            @Override
            public boolean isAccelerometerCalibrated() {
                return false;
            }

            @Override
            public boolean isMagnetometerCalibrated() {
                return false;
            }

            @Override
            public CalibrationData readCalibrationData() {
                return null;
            }

            @Override
            public void writeCalibrationData(CalibrationData data) {

            }

            @Override
            public byte read8(Register register) {
                return 0;
            }

            @Override
            public byte[] read(Register register, int cb) {
                return new byte[0];
            }

            @Override
            public void write8(Register register, int bVal) {

            }

            @Override
            public void write(Register register, byte[] data) {

            }
        };

        // Servo intakeAngle = hardwareMap.Servo.get("intakeAngle");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightElevator.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {



            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double ry = gamepad1.right_stick_y;
            double i = gamepad1.right_trigger;
            //double ia gamepad1.right_bumper;


            double botHeading = -imu.getAngularOrientation().firstAngle /*- Math.PI*/;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double rightElevatorPower =ry;
            double leftElevatorPower=ry;
            double intakePower=i;

            if (gamepad1.circle){
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);

            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            intake.setPower(intakePower);
            if(ry>0.5){
                rightElevator.setPower(0.5);
                leftElevator.setPower(0.5);

            }
            else{
                rightElevator.setPower(rightElevatorPower);
                leftElevator.setPower(leftElevatorPower);
            }

        }
    }
}