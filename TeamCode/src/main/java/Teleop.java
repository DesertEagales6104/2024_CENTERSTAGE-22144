

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        IMU imu = hardwareMap.get(IMU.class, "imu");

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor rightElevator = hardwareMap.dcMotor.get("rightElevator");
        DcMotor leftElevator = hardwareMap.dcMotor.get("leftElevator");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo intakeServo = hardwareMap.servo.get("intakeServo");
        Servo rightElevatorServo = hardwareMap.servo.get("rightElevatorServo");
        Servo leftElevatorServo = hardwareMap.servo.get("leftElevatorServo");
        YawPitchRollAngles robotOrientation;
        


        // Servo intakeAngle = hardwareMap.Servo.get("intakeAngle");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo.setDirection(Servo.Direction.REVERSE);
        leftElevatorServo.setDirection(Servo.Direction.REVERSE);

        IMU.Parameters imuPar = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        );
        imu.initialize(imuPar);
        imu.resetYaw();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {



            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = - gamepad1.right_trigger + gamepad1.left_trigger;
            double ry = gamepad1.right_stick_y;
            double i = gamepad1.right_trigger;
            robotOrientation = imu.getRobotYawPitchRollAngles();

            double botHeading = -robotOrientation.getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
//            intakeServo.scaleRange(0.6,0.8);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if(gamepad1.a && intakeServo.getPosition()==1) {
                intakeServo.setPosition(0.645);
                intake.setPower(1);
            }
            else if(gamepad1.x&&intakeServo.getPosition()!= 1) {
                intakeServo.setPosition(1);
                intake.setPower(0);
            }


            if(gamepad1.dpad_up) {leftElevatorServo.setPosition(0); rightElevatorServo.setPosition(0);}

            if(gamepad1.touchpad) {imu.resetYaw();}

            if(gamepad1.dpad_down) {leftElevatorServo.setPosition(0.45); rightElevatorServo.setPosition(0.45);}

            if(Math.abs(ry) < 0.05) {
                rightElevator.setPower(0);
                leftElevator.setPower(0);
            } else if(Math.abs(ry) < 1) {
                if(!gamepad1.dpad_up && !gamepad1.dpad_down) {
                    rightElevatorServo.setPosition(0.55);
                    leftElevatorServo.setPosition(0.55);
                }
                rightElevator.setPower(ry);
                leftElevator.setPower(ry);
            } else {
                if(!gamepad1.dpad_up && !gamepad1.dpad_down) {
                    rightElevatorServo.setPosition(0.55);
                    leftElevatorServo.setPosition(0.55);
                }
                rightElevator.setPower(Math.copySign(1,ry));
                leftElevator.setPower(Math.copySign(1,ry));
            }


            telemetry.addLine("Servo Angle is: " + intakeServo.getPosition());
            telemetry.addLine("Right Elevator Angle is:" + rightElevatorServo.getPosition());
            telemetry.addLine("Left Elevator Angle is:" + leftElevatorServo.getPosition());
            telemetry.addLine("IMU is: " + robotOrientation.getYaw(AngleUnit.RADIANS));
            telemetry.update();
        }
    }
}