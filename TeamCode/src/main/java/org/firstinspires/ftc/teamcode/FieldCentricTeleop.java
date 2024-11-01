package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(67, 40); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        DcMotor testMotor = hardwareMap.dcMotor.get("testMotor");
        Servo testServo = hardwareMap.servo.get("testServo");
        testServo.setPosition(1);

        NormalizedColorSensor colorSensor
                = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        DigitalChannel touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        boolean touchSensorPreviouslyPressed = false;
        boolean touchSensorCurrentlyPressed = false;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            /******************  RESET GYRO / IMU ANGLE TO FIX TELEOP STARTS *******************/

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }
            /******************  RESET GYRO / IMU ANGLE TO FIX TELEOP ENDS *******************/


            /******************  TELEOP DRIVING CODE STARTS *******************/

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(0.5 * frontLeftPower);
            backLeftMotor.setPower(0.5 * backLeftPower);
            frontRightMotor.setPower(0.5 * frontRightPower);
            backRightMotor.setPower(0.5 * backRightPower);

            /******************  TELEOP DRIVING CODE ENDS *******************/


            /******************  TOUCH SENSOR BASED TEST SERVO OPERATION CODE STARTS *******************/

            telemetry.addData("Touch Sensor: ", touchSensor.getState());

            // button is PRESSED if value returned is LOW or false.
            if (touchSensor.getState()) {
                touchSensorCurrentlyPressed = false;
            } else {
                touchSensorCurrentlyPressed = true;
            }

            // If the button state is different than what it was, then act
            if (touchSensorCurrentlyPressed != touchSensorPreviouslyPressed) {
                // If the button is (now) down, then toggle the SERVO position
                if (touchSensorCurrentlyPressed) {
                    if (testServo.getPosition() > 0.9) {
                        testServo.setPosition(0);
                    } else if (testServo.getPosition() < 0.1) {
                        testServo.setPosition(1);
                    }
                }
            }
            touchSensorPreviouslyPressed = touchSensorCurrentlyPressed;

            /******************  TOUCH SENSOR BASED TEST SERVO OPERATION CODE ENDS *******************/


            /******************  GAMEPAD BASED TEST MOTOR OPERATION CODE STARTS *******************/

/* Enable this OR tbe next block only */
            if (gamepad1.right_trigger > 0.05) {
                testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                testMotor.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.05) {
                testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                testMotor.setPower(-gamepad1.left_trigger);
            } else {
                testMotor.setPower(0);
            }
/**/

/* Enable this OR tbe previous block only
            if (gamepad1.right_bumper) {
                testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                while (opModeIsActive() && (testMotor.getCurrentPosition() <= 2000)) {
                    testMotor.setPower(1);
                }
            } else if (gamepad1.left_bumper) {
                testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                testMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                testMotor.setPower(1);
                testMotor.setTargetPosition(-2000);
            }
 */

            telemetry.addData("Test Motor Pos: ", testMotor.getCurrentPosition());
            telemetry.addData("Test Servo Pos: ", testServo.getPosition());

            /******************  GAMEPAD BASED TEST MOTOR OPERATION CODE ENDS *******************/


            /******************  LOGGING TO CHECK PINPOINT TRACKING STARTS *******************/
            odo.update();
            telemetry.addData("X: ", odo.getPosX());
            telemetry.addData("Y: ", odo.getPosY());
            telemetry.addData("Heading Odo: ", Math.toDegrees(odo.getHeading()));
            telemetry.update();
            telemetry.update();

            /******************  LOGGING TO CHECK PINPOINT TRACKING ENDS *******************/
        }
    }
}