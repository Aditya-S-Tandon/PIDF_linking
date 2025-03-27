package org.firstinspires.ftc.teamcode.FTC_2024;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="encoder_test")
public class encoder_test extends LinearOpMode {
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotor LiftMotor = null;
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor IntakeMotor = null;
    Servo IntakeServo = null;
    IMU imu;
    Orientation angles;
    YawPitchRollAngles  robotorientation;
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    @Override
    public void runOpMode() {

        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");
        LiftMotor = hardwareMap.dcMotor.get("LM");
        IntakeMotor = hardwareMap.dcMotor.get("IM");
        IntakeServo = hardwareMap.servo.get("IS");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        double servo_bucket_intake = 0.8;
        double servo_bucket_output = 0.09;
        double intake_motor_power = 1.;
        double outtake_motor_power = 0.7;
        waitForStart();

        //First Sample Intake
/*
        IntakeServo.setPosition(servo_bucket_output);
        safeWaitSeconds(0.75);
        ///move forward to the net zone for sample 2
        FrontLeft.setPower(0.75);
        BackLeft.setPower(0.75);
        FrontRight.setPower(0.75);
        BackRight.setPower(0.75);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.19)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        FrontLeft.setPower(-0.);
        BackLeft.setPower(0.);
        FrontRight.setPower(0.);
        BackRight.setPower(-0.);

        IntakeMotor.setPower(-intake_motor_power);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        safeWaitSeconds(0.5);
        IntakeServo.setPosition(servo_bucket_intake);
        safeWaitSeconds(1.);
        IntakeMotor.setPower(outtake_motor_power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        IntakeMotor.setPower(0.);

 */






        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        // Initialize IMU directly
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );
        robotorientation = imu.getRobotYawPitchRollAngles();

        /*
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                90,
                                0,
                                -45,
                                0  // acquisitionTime, not used
                        )
                )
        );*/
// Initialize IMU using Parameters
        //imu.initialize(myIMUparameters);
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double init_yaw = angles.firstAngle;
        double init_pitch = angles.secondAngle;
        double init_roll = angles.thirdAngle;
            ///turn anticlockwise
           // FrontLeft.setPower(0.5);
            //BackLeft.setPower(0.5);
            //FrontRight.setPower(-0.5);
            //BackRight.setPower(-0.5);
            runtime.reset();

            while (opModeIsActive() && (runtime.seconds() < 1)) {
                angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Init Heading", init_yaw);
                telemetry.addData("Init Pitch", init_pitch);
                telemetry.addData("Init Roll", init_roll);
                telemetry.addData(" Heading", angles.firstAngle);
                telemetry.addData(" Pitch", angles.secondAngle);
                telemetry.addData(" Roll", angles.thirdAngle);
                telemetry.update();
            }
        double final_yaw = robotorientation.getYaw(AngleUnit.DEGREES);

            while (Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < Math.abs(init_yaw) && opModeIsActive()) {
                ///turn clockwise
                FrontLeft.setPower(0.2);
                BackLeft.setPower(0.2);
                FrontRight.setPower(-0.2);
                BackRight.setPower(-0.2);
                telemetry.addData(" Heading", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
            }


                while(Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)>Math.abs(init_yaw) && opModeIsActive()){
                    ///turn anticlockwise
                    FrontLeft.setPower(-0.2);
                    BackLeft.setPower(-0.2);
                    FrontRight.setPower(0.2);
                    BackRight.setPower(0.2);
                    telemetry.addData(" Heading", robotorientation.getYaw(AngleUnit.DEGREES));
                    telemetry.update();
                }



        /*
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );
        */

        FrontLeft.setPower(-0.);
        BackLeft.setPower(-0.);
        FrontRight.setPower(0.);
        BackRight.setPower(0.);
        //telemetry.addData("Heading", "Leg 3: %4.1f S Elapsed", angles.firstAngle);
        //telemetry.update();
        while (opModeIsActive()) {
            //if (FrontLeft.isBusy()) {
            telemetry.addData("Liftmotor motor position", LiftMotor.getCurrentPosition());
            telemetry.update();
            //}
        }
            //254


    }
}
