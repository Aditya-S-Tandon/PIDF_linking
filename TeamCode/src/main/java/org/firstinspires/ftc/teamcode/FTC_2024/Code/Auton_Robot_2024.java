package org.firstinspires.ftc.teamcode.FTC_2024;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name="Auton_robot2024")
public class Auton_Robot_2024 extends LinearOpMode {
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    Servo BigWrist = null;
    Servo SmallPivot = null;
    Servo SampleGrabber = null;
    DcMotor LiftMotor = null;
    IMU imu;
    Orientation angles;

    private ElapsedTime runtime = new ElapsedTime();
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
        BigWrist = hardwareMap.servo.get("BW");
      //  SmallPivot = hardwareMap.servo.get("SP");
      //  SampleGrabber = hardwareMap.servo.get("SG");
        LiftMotor = hardwareMap.dcMotor.get("LM");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Zero Power Behavior
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

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

        double big_wrist_position = -1;
        double bigout_wrist_position= 0.4;
        double small_pivot_position = 0.2  ;
        double sample_grabber_position = 0.6;
        String LiftMotorCurrentDirection = "up";
        int extention_target = 810;
        int retraction_target = 0;
        double lift_motor_power=0.7;
        double servoposition =1;
        waitForStart();
        BigWrist.setPosition(servoposition);
        /*
        ///turn anticlockwise
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
*/

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double init_yaw = angles.firstAngle;
        double init_pitch = angles.secondAngle;
        double init_roll = angles.thirdAngle;

        //Strafe left
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        ///move backward to the net zone for sample 1
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.4)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        ///turn anticlockwise
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        ///move backward to the net zone for sample 1
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Action to drop sample
            //Lift
        runtime.reset();
            LiftMotor.setPower(-lift_motor_power);
            LiftMotor.setTargetPosition(extention_target);
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotorCurrentDirection = "up";
            //Remove  Power from the lift Motor
        while (opModeIsActive() && LiftMotorCurrentDirection.equals("up") && (LiftMotor.getCurrentPosition() < extention_target) && (runtime.seconds() < 3)){
                //extention_power = 0;
                //ExtentionMotor.setPower(extenstion_power);
                //ExtentionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //
            if(LiftMotor.getCurrentPosition() >790){
                BigWrist.setPosition(big_wrist_position);
            }
            telemetry.addData("Lift motor position", LiftMotor.getCurrentPosition());
            telemetry.update();
            }
            idle();

        safeWaitSeconds(1.5);
        BigWrist.setPosition(servoposition);
        ///move backward to the net zone for sample 1
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

         ///turn clockwise
        FrontLeft.setPower(0.2);
        BackLeft.setPower(0.2);
        FrontRight.setPower(-0.2);
        BackRight.setPower(-0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.25)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        LiftMotor.setPower(-lift_motor_power);
        LiftMotor.setTargetPosition(retraction_target);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotorCurrentDirection = "down";

        while (opModeIsActive() && LiftMotorCurrentDirection.equals("down") && (LiftMotor.getCurrentPosition() > retraction_target)) {

            telemetry.addData("Lift motor position", LiftMotor.getCurrentPosition());
            telemetry.update();
        }
        idle();
        //Remove  Power from the lift Motor
        lift_motor_power = 0;
        LiftMotor.setPower(lift_motor_power);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < Math.abs(init_yaw) && opModeIsActive()) {
            ///turn anticlockwise
            FrontLeft.setPower(-0.5);
            BackLeft.setPower(-0.5);
            FrontRight.setPower(0.5);
            BackRight.setPower(0.5);

        }
        while(Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)>Math.abs(init_yaw) && opModeIsActive()){
            ///turn clockwise
            FrontLeft.setPower(0.5);
            BackLeft.setPower(0.5);
            FrontRight.setPower(-0.5);
            BackRight.setPower(-0.5);
        }
        FrontLeft.setPower(-0.);
        BackLeft.setPower(-0.);
        FrontRight.setPower(0.);
        BackRight.setPower(0.);
        ///move forward to the ascend zone for sample 2
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Strafe left for sample 2
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.2)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        ///move backward to the net zone for sample 2
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Strafe right for sample 2
        FrontLeft.setPower(0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.3)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Strafe left for sample 3
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.3)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        ///move backward to the net zone for sample 3
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Strafe right for sample 3
        FrontLeft.setPower(0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.2)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

//Sample 4
        //Strafe left for sample 4
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.3)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        ///move backward to the net zone for sample 4
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.15)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Strafe right for sample 4
        FrontLeft.setPower(0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

    /*
        //Strafe left to park
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
*/

        //Moving forward to park
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Strafe right
        FrontLeft.setPower(0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Moving forward to park
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.8)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

    }
}