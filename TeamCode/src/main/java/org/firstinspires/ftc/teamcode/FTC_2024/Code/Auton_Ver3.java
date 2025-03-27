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
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous(name="Auton_Ver3")
public class Auton_Ver3 extends LinearOpMode {
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    Servo BigWrist = null;
    Servo SmallPivot = null;
    Servo SampleGrabber = null;
    DcMotor LiftMotor = null;
    DcMotor IntakeMotor = null;
    Servo IntakeServo = null;
    DcMotor ExtentionMotor = null;
    IMU imu;
    Orientation angles;
    private PIDController controller;
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
        SmallPivot = hardwareMap.servo.get("SP");
        ExtentionMotor = hardwareMap.dcMotor.get("EM");
        LiftMotor = hardwareMap.dcMotor.get("LM");
        IntakeMotor = hardwareMap.dcMotor.get("IM");
        IntakeServo = hardwareMap.servo.get("IS");

        ExtentionMotor.setPower(-0.05);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Zero Power Behavior
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtentionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        //RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        //RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        // Initialize IMU directly
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                               // RevHubOrientationOnRobot.LogoFacingDirection.UP,
                               // RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );

        double big_wrist_position = -1;
        double bigout_wrist_position = 0.4;
        double small_pivot_position = 0.2;
        double sample_grabber_position = 0.6;
        String LiftMotorCurrentDirection = "up";
        int extention_target = 810;
        int retraction_target = 0;
        double lift_motor_power = 0.7;
        double servoposition = 1;

        double ticks_per_rev = 537.7;
        double diameter = 3.77;
        double sample_forward_initial = 23;
        double sample_forward_target = 15;
        double sample_left_target = 60;
        double sample_forward2_target = 32;
        double sample_left2_target = 10;
        double sample_left3_target =36;
        double sample_forward3_target = 55;
        double sample_backward_target = 3;
        double sample_backward_target_one = 26;
        double sample_forward_target_one = 24;
        double sample_forward_target_two = 27;
        double sample_backward_target_two = 27;
        double sample_forward_target_one_init = 14;

        double rotation_angle = 101;//65 old//80 picked
        double rotation_angle_sample2 = 100;//70 picked and old
        double rotation_angle2 = 50;//30 old
        double rotation_angle3 = 55;//37 old

        double intake_forward_power=0.5;
        double p = 0.004, i = 0, d = 0;
        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        int fl_target ;
        int fr_target;
        int bl_target ;
        int br_target;

        double servo_bucket_intake = 0.8;
        double servo_bucket_output = 0.09;
        //double servo_bucket_intake = 0.78; //old values for gobilda servos
        //double servo_bucket_output = 0.2;
        double intake_motor_power = 1;
        double intake_negative_power = -1;
        double outtake_motor_power = 1.;
        waitForStart();
        imu.resetYaw();
        //while (opModeIsActive()) {
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double init_yaw = angles.firstAngle;
        //double init_pitch = angles.secondAngle;
        //double init_roll = angles.thirdAngle;

        //Path test
        //Zero Power Behavior
        //while (opModeIsActive()) {

        //}
        //Strafe left
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while   (opModeIsActive() && (runtime.seconds() < 0.25)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Move forward
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl_target = (int) (FrontLeft.getCurrentPosition()-(int) sample_forward_initial*ticks_per_rev/(diameter*3.14));
        fr_target = (int) (FrontRight.getCurrentPosition()-(int) sample_forward_initial*ticks_per_rev/(diameter*3.14));
        bl_target = (int) (BackLeft.getCurrentPosition()-(int) sample_forward_initial*ticks_per_rev/(diameter*3.14));
        br_target = (int) (BackRight.getCurrentPosition()-(int) sample_forward_initial*ticks_per_rev/(diameter*3.14));

        FrontLeft.setTargetPosition(fl_target);
        FrontRight.setTargetPosition(fr_target);
        BackLeft.setTargetPosition(bl_target);
        BackRight.setTargetPosition(br_target);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Forward
        FrontLeft.setPower(1.);
        BackLeft.setPower(1.);
        FrontRight.setPower(1.);
        BackRight.setPower(1.);

        while   (opModeIsActive() && FrontLeft.isBusy()) {
            telemetry.addData("FL", FrontLeft.getCurrentPosition());
            telemetry.update();
        }

        while   (opModeIsActive() && FrontRight.isBusy()) {
            telemetry.addData("FR", FrontRight.getCurrentPosition());
            telemetry.update();
        }

        while   (opModeIsActive() && BackLeft.isBusy()) {
            telemetry.addData("BL", BackLeft.getCurrentPosition());
            telemetry.update();
        }

        while   (opModeIsActive() && BackRight.isBusy()) {
            telemetry.addData("BR", BackRight.getCurrentPosition());
            telemetry.update();
        }

         //FrontLeft.setPower(-0.);
         //BackLeft.setPower(0.);
         //FrontRight.setPower(0.);
         //BackRight.setPower(-0.);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        ///move backward to the net zone for sample 1
        FrontLeft.setPower(-0.95);
        BackLeft.setPower(-0.95);
        FrontRight.setPower(-0.95);
        BackRight.setPower(-0.95);
        runtime.reset();
        while   (runtime.seconds() < 0.45)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        */

        ///turn anticlockwise
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while   (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        ///move backward to the net zone for sample 1
      /*

       */
        //Move forward
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fl_target = (int) (FrontLeft.getCurrentPosition()-(int) sample_backward_target *ticks_per_rev/(diameter*3.14));
        fr_target = (int) (FrontRight.getCurrentPosition()-(int) sample_backward_target*ticks_per_rev/(diameter*3.14));
        bl_target = (int) (BackLeft.getCurrentPosition()-(int) sample_backward_target*ticks_per_rev/(diameter*3.14));
        br_target = (int) (BackRight.getCurrentPosition()-(int) sample_backward_target*ticks_per_rev/(diameter*3.14));

        FrontLeft.setTargetPosition(fl_target);
        FrontRight.setTargetPosition(fr_target);
        BackLeft.setTargetPosition(bl_target);
        BackRight.setTargetPosition(br_target);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Backward
        FrontLeft.setPower(1.);
        BackLeft.setPower(1.);
        FrontRight.setPower(1.);
        BackRight.setPower(1.);

        while   (opModeIsActive() && FrontLeft.isBusy()) {
            telemetry.addData("FL", FrontLeft.getCurrentPosition());
            telemetry.update();
        }

        while   (opModeIsActive() && FrontRight.isBusy()) {
            telemetry.addData("FR", FrontRight.getCurrentPosition());
            telemetry.update();
        }

        while   (opModeIsActive() &&BackLeft.isBusy()) {
            telemetry.addData("BL", BackLeft.getCurrentPosition());
            telemetry.update();
        }

        while   (opModeIsActive() && BackRight.isBusy()) {
            telemetry.addData("BR", BackRight.getCurrentPosition());
            telemetry.update();
        }

        //FrontLeft.setPower(-0.);
        //BackLeft.setPower(0.);
        //FrontRight.setPower(0.);
        //BackRight.setPower(-0.);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            FrontLeft.setPower(-0.5);
            BackLeft.setPower(-0.5);
            FrontRight.setPower(-0.5);
            BackRight.setPower(-0.5);
            runtime.reset();
            while   (opModeIsActive() && (runtime.seconds() < 0.1)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

        //Lift
        runtime.reset();
        LiftMotor.setPower(-lift_motor_power);
        LiftMotor.setTargetPosition(extention_target);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotorCurrentDirection = "up";
        //Remove  Power from the lift Motor
        while   (opModeIsActive() && LiftMotorCurrentDirection.equals("up") && (LiftMotor.getCurrentPosition() < extention_target) && (runtime.seconds() < 3)){
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
       // idle();

        safeWaitSeconds(1.5);
        BigWrist.setPosition(servoposition);
        safeWaitSeconds(0);
        /*
        ///move forward to the ascend zone for sample 1
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while   ((runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

         */

            //Move forward
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            fl_target = (int) (FrontLeft.getCurrentPosition()+(int) sample_forward_target_one_init*ticks_per_rev/(diameter*3.14));
            fr_target = (int) (FrontRight.getCurrentPosition()+(int) sample_forward_target_one_init*ticks_per_rev/(diameter*3.14));
            bl_target = (int) (BackLeft.getCurrentPosition()+(int) sample_forward_target_one_init*ticks_per_rev/(diameter*3.14));
            br_target = (int) (BackRight.getCurrentPosition()+(int) sample_forward_target_one_init*ticks_per_rev/(diameter*3.14));

            FrontLeft.setTargetPosition(fl_target);
            FrontRight.setTargetPosition(fr_target);
            BackLeft.setTargetPosition(bl_target);
            BackRight.setTargetPosition(br_target);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Backward
            FrontLeft.setPower(1.);
            BackLeft.setPower(1.);
            FrontRight.setPower(1.);
            BackRight.setPower(1.);

            while   (opModeIsActive() && FrontLeft.isBusy()) {
                telemetry.addData("FL", FrontLeft.getCurrentPosition());
                telemetry.update();
            }

            while   (opModeIsActive() && FrontRight.isBusy()) {
                telemetry.addData("FR", FrontRight.getCurrentPosition());
                telemetry.update();
            }

            while   (opModeIsActive() && BackLeft.isBusy()) {
                telemetry.addData("BL", BackLeft.getCurrentPosition());
                telemetry.update();
            }

            while   (opModeIsActive() && BackRight.isBusy()) {
                telemetry.addData("BR", BackRight.getCurrentPosition());
                telemetry.update();
            }
            //FrontLeft.setPower(-0.);
            //BackLeft.setPower(0.);
            //FrontRight.setPower(0.);
            //BackRight.setPower(-0.);

            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ///turn anticlockwise
        FrontLeft.setPower(-0.15);
        BackLeft.setPower(-0.15);
        FrontRight.setPower(0.15);
        BackRight.setPower(0.15);
        runtime.reset();
        while   (opModeIsActive() &&(runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        while (Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < Math.abs(rotation_angle) && opModeIsActive()) {
            ///turn anticlockwise
            FrontLeft.setPower(-0.5);
            BackLeft.setPower(-0.5);
            FrontRight.setPower(0.5);
            BackRight.setPower(0.5);

        }
        while(Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)>Math.abs(rotation_angle) && opModeIsActive()){
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
        IntakeServo.setPosition(servo_bucket_output);
        LiftMotor.setPower(-lift_motor_power);
        LiftMotor.setTargetPosition(retraction_target);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotorCurrentDirection = "down";

        while   (opModeIsActive() && LiftMotorCurrentDirection.equals("down") && (LiftMotor.getCurrentPosition() > retraction_target)) {

            telemetry.addData("Lift motor position", LiftMotor.getCurrentPosition());
            telemetry.update();
        }
//idle();
        //Remove  Power from the lift Motor
        lift_motor_power = 0;
        LiftMotor.setPower(lift_motor_power);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*
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

         */

        //First Sample Intake

        //safeWaitSeconds(1.);
        ///move forward to the net zone for sample 2
        /*
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while   ((runtime.seconds() < 0.35)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        FrontLeft.setPower(-0.);
        BackLeft.setPower(0.);
        FrontRight.setPower(0.);
        BackRight.setPower(-0.);

         */
            IntakeMotor.setPower(-intake_motor_power);
            //Move forward
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            fl_target = (int) (FrontLeft.getCurrentPosition()+(int) sample_forward_target_one*ticks_per_rev/(diameter*3.14));
            fr_target = (int) (FrontRight.getCurrentPosition()+(int) sample_forward_target_one*ticks_per_rev/(diameter*3.14));
            bl_target = (int) (BackLeft.getCurrentPosition()+(int) sample_forward_target_one*ticks_per_rev/(diameter*3.14));
            br_target = (int) (BackRight.getCurrentPosition()+(int) sample_forward_target_one*ticks_per_rev/(diameter*3.14));

            FrontLeft.setTargetPosition(fl_target);
            FrontRight.setTargetPosition(fr_target);
            BackLeft.setTargetPosition(bl_target);
            BackRight.setTargetPosition(br_target);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Backward
            FrontLeft.setPower(intake_forward_power);
            BackLeft.setPower(intake_forward_power);
            FrontRight.setPower(intake_forward_power);
            BackRight.setPower(intake_forward_power);

            while   (opModeIsActive() && FrontLeft.isBusy()) {
                telemetry.addData("FL", FrontLeft.getCurrentPosition());
                telemetry.update();
            }

            while   (opModeIsActive() && FrontRight.isBusy()) {
                telemetry.addData("FR", FrontRight.getCurrentPosition());
                telemetry.update();
            }

            while   (opModeIsActive() && BackLeft.isBusy()) {
                telemetry.addData("BL", BackLeft.getCurrentPosition());
                telemetry.update();
            }

            while   (opModeIsActive() && BackRight.isBusy()) {
                telemetry.addData("BR", BackRight.getCurrentPosition());
                telemetry.update();
            }
            FrontLeft.setPower(-0.);
            BackLeft.setPower(0.);
            FrontRight.setPower(0.);
            BackRight.setPower(-0.);

            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //runtime.reset();
        //while   ((runtime.seconds() < 1.5)) {
           // telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
          //  telemetry.update();
      //  }
        //safeWaitSeconds(0.5);


//Action to drop Sample
            //move back to drop second sample
            //Move forward
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            fl_target = (int) (FrontLeft.getCurrentPosition()-(int) sample_backward_target_one*ticks_per_rev/(diameter*3.14));
            fr_target = (int) (FrontRight.getCurrentPosition()-(int) sample_backward_target_one*ticks_per_rev/(diameter*3.14));
            bl_target = (int) (BackLeft.getCurrentPosition()-(int) sample_backward_target_one*ticks_per_rev/(diameter*3.14));
            br_target = (int) (BackRight.getCurrentPosition()-(int) sample_backward_target_one*ticks_per_rev/(diameter*3.14));

            FrontLeft.setTargetPosition(fl_target);
            FrontRight.setTargetPosition(fr_target);
            BackLeft.setTargetPosition(bl_target);
            BackRight.setTargetPosition(br_target);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Backward
            FrontLeft.setPower(1.);
            BackLeft.setPower(1.);
            FrontRight.setPower(1.);
            BackRight.setPower(1.);

            while   (opModeIsActive() && FrontLeft.isBusy()) {
                IntakeServo.setPosition(servo_bucket_intake);
                safeWaitSeconds(1.);
                IntakeMotor.setPower(outtake_motor_power);
                runtime.reset();
                while   (opModeIsActive() && (runtime.seconds() < 1.)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                IntakeMotor.setPower(0.);
                telemetry.addData("FL", FrontLeft.getCurrentPosition());
                telemetry.update();
            }

            while   (opModeIsActive() && FrontRight.isBusy()) {
                telemetry.addData("FR", FrontRight.getCurrentPosition());
                telemetry.update();
            }

            while   (opModeIsActive() && BackLeft.isBusy()) {
                telemetry.addData("BL", BackLeft.getCurrentPosition());
                telemetry.update();
            }

            while   (opModeIsActive() && BackRight.isBusy()) {
                telemetry.addData("BR", BackRight.getCurrentPosition());
                telemetry.update();
            }

            //FrontLeft.setPower(-0.);
            //BackLeft.setPower(0.);
            //FrontRight.setPower(0.);
            //BackRight.setPower(-0.);
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ///turn clockwise
        FrontLeft.setPower(0.15);
        BackLeft.setPower(0.15);
        FrontRight.setPower(-0.15);
        BackRight.setPower(-0.15);
        runtime.reset();
        /*
        while(opmodeactive){
            if(step == 1){
                if(runtime.seconds() < 0.2){
                    step1 code
                    slides = 200
                    armpose = 200
                }else{
                    step = 2
                }
            }
            if(step == 2){
                if(runtime.seconds() < 0.2){
                    step1 code
                }
            }
            arm code
            slides code
            
        }
         */
        while   (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        while (Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < Math.abs(rotation_angle2) && opModeIsActive()) {
            ///turn anticlockwise
            FrontLeft.setPower(-0.5);
            BackLeft.setPower(-0.5);
            FrontRight.setPower(0.5);
            BackRight.setPower(0.5);

        }
        while(Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)>Math.abs(rotation_angle2) && opModeIsActive()){
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


        ///move backward to the net zone for sample 1
        FrontLeft.setPower(-1.);
        BackLeft.setPower(-1.);
        FrontRight.setPower(-1.);
        BackRight.setPower(-1.);
        runtime.reset();
        while   (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        FrontLeft.setPower(-0.);
        BackLeft.setPower(-0.);
        FrontRight.setPower(-0.);
        BackRight.setPower(-0.);


        //Lift
        runtime.reset();
        lift_motor_power = 0.7;
        LiftMotor.setPower(-lift_motor_power);
        LiftMotor.setTargetPosition(extention_target);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotorCurrentDirection = "up";
        //Remove  Power from the lift Motor
        while( opModeIsActive() &&  LiftMotorCurrentDirection.equals("up") && (LiftMotor.getCurrentPosition() < extention_target) && (runtime.seconds() < 3)){
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
      //  idle();

        safeWaitSeconds(0.75);
        BigWrist.setPosition(servoposition);
        safeWaitSeconds(0);
        /*
        ///move forward to the net zone for sample 1
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while   ((runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

         */

        FrontLeft.setPower(-0.);
        BackLeft.setPower(-0.);
        FrontRight.setPower(0.);
        BackRight.setPower(0.);



        //Second sample pickup


        ///turn anticlockwise
        FrontLeft.setPower(-0.1);
        BackLeft.setPower(-0.1);
        FrontRight.setPower(0.1);
        BackRight.setPower(0.1);
            runtime.reset();
        while((runtime.seconds() < 0.15)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        while (Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < Math.abs(rotation_angle_sample2) && opModeIsActive()) {
            ///turn anticlockwise
            FrontLeft.setPower(-0.5);
            BackLeft.setPower(-0.5);
            FrontRight.setPower(0.5);
            BackRight.setPower(0.5);

        }
        while(Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)>Math.abs(rotation_angle_sample2) && opModeIsActive()){
            ///turn clockwise
            FrontLeft.setPower(0.5);
            BackLeft.setPower(0.5);
            FrontRight.setPower(-0.5);
            BackRight.setPower(-0.5);
        }
        IntakeServo.setPosition(servo_bucket_output);
        FrontLeft.setPower(-0.);
        BackLeft.setPower(-0.);
        FrontRight.setPower(0.);
        BackRight.setPower(0.);
        //Second Sample Intake

        LiftMotor.setPower(-lift_motor_power);
        LiftMotor.setTargetPosition(retraction_target);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotorCurrentDirection = "down";

        while( opModeIsActive() &&  LiftMotorCurrentDirection.equals("down") && (LiftMotor.getCurrentPosition() > retraction_target)) {

            telemetry.addData("Lift motor position", LiftMotor.getCurrentPosition());
            telemetry.update();
        }
  //      idle();
        //Remove  Power from the lift Motor from Sample 2
        lift_motor_power = 0;
        LiftMotor.setPower(lift_motor_power);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //safeWaitSeconds(0.5);
            /*
        ///move forward to the net zone for sample 2
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while   ((runtime.seconds() < 0.75)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        FrontLeft.setPower(-0.);
        BackLeft.setPower(0.);
        FrontRight.setPower(0.);
        BackRight.setPower(-0.);

             */

        IntakeMotor.setPower(-intake_motor_power);
            //Move forward
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            //Move forward
            fl_target = (int) (FrontLeft.getCurrentPosition()+(int) sample_forward_target_two*ticks_per_rev/(diameter*3.14));
            fr_target = (int) (FrontRight.getCurrentPosition()+(int) sample_forward_target_two*ticks_per_rev/(diameter*3.14));
            bl_target = (int) (BackLeft.getCurrentPosition()+(int) sample_forward_target_two*ticks_per_rev/(diameter*3.14));
            br_target = (int) (BackRight.getCurrentPosition()+(int) sample_forward_target_two*ticks_per_rev/(diameter*3.14));

            FrontLeft.setTargetPosition(fl_target);
            FrontRight.setTargetPosition(fr_target);
            BackLeft.setTargetPosition(bl_target);
            BackRight.setTargetPosition(br_target);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Forward
            FrontLeft.setPower(intake_forward_power);
            BackLeft.setPower(intake_forward_power);
            FrontRight.setPower(intake_forward_power);
            BackRight.setPower(intake_forward_power);
            while( opModeIsActive() &&  FrontLeft.isBusy()) {
                telemetry.addData("FL", FrontLeft.getCurrentPosition());
                telemetry.update();
            }

            while( opModeIsActive() &&  FrontRight.isBusy()) {
                telemetry.addData("FR", FrontRight.getCurrentPosition());
                telemetry.update();
            }

            while( opModeIsActive() &&  BackLeft.isBusy()) {
                telemetry.addData("BL", BackLeft.getCurrentPosition());
                telemetry.update();
            }

            while(  opModeIsActive() && BackRight.isBusy()) {
                telemetry.addData("BR", BackRight.getCurrentPosition());
                telemetry.update();
            }


            //idle();
            //FrontLeft.setPower(-0.);
            //BackLeft.setPower(0.);
            //FrontRight.setPower(0.);
            //BackRight.setPower(-0.);
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //move backward to the net zone for sample 1
            FrontLeft.setPower(-0.);
            BackLeft.setPower(-0.);
            FrontRight.setPower(-0.);
            BackRight.setPower(-0.);


            //runtime.reset();
        //while   ((runtime.seconds() < 1.5)) {
          //  telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
           // telemetry.update();
        //}


        //safeWaitSeconds(0.5);



//Action to drop Sample
/*
        ///move backward to the net zone for sample 1
        FrontLeft.setPower(-0.95);
        BackLeft.setPower(-0.95);
        FrontRight.setPower(-0.95);
        BackRight.setPower(-0.95);
        runtime.reset();
        while(   (runtime.seconds() < 0.35)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

 */

            //Move forward
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //set encoders to 0
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            //Move forward
            fl_target = (int) (FrontLeft.getCurrentPosition()-(int) sample_backward_target_two*ticks_per_rev/(diameter*3.14));
            fr_target = (int) (FrontRight.getCurrentPosition()-(int) sample_backward_target_two*ticks_per_rev/(diameter*3.14));
            bl_target = (int) (BackLeft.getCurrentPosition()-(int) sample_backward_target_two*ticks_per_rev/(diameter*3.14));
            br_target = (int) (BackRight.getCurrentPosition()-(int) sample_backward_target_two*ticks_per_rev/(diameter*3.14));

            FrontLeft.setTargetPosition(fl_target);
            FrontRight.setTargetPosition(fr_target);
            BackLeft.setTargetPosition(bl_target);
            BackRight.setTargetPosition(br_target);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Forward
            FrontLeft.setPower(0.95);
            BackLeft.setPower(0.95);
            FrontRight.setPower(0.95);
            BackRight.setPower(0.95);
            while( opModeIsActive() &&  FrontLeft.isBusy()) {
                IntakeServo.setPosition(servo_bucket_intake);
                safeWaitSeconds(1.);
                IntakeMotor.setPower(outtake_motor_power);
                runtime.reset();
                while( opModeIsActive() &&  (runtime.seconds() < 1.5)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                IntakeMotor.setPower(0.);
                telemetry.addData("FL", FrontLeft.getCurrentPosition());
                telemetry.update();
            }

            while( opModeIsActive() &&  FrontRight.isBusy()) {
                telemetry.addData("FR", FrontRight.getCurrentPosition());
                telemetry.update();
            }

            while( opModeIsActive() &&  BackLeft.isBusy()) {
                telemetry.addData("BL", BackLeft.getCurrentPosition());
                telemetry.update();
            }

            while(  opModeIsActive() && BackRight.isBusy()) {
                telemetry.addData("BR", BackRight.getCurrentPosition());
                telemetry.update();
            }


            //idle();
            //FrontLeft.setPower(-0.);
            //BackLeft.setPower(0.);
            //FrontRight.setPower(0.);
            //BackRight.setPower(-0.);
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //move backward to the net zone for sample 1
            //FrontLeft.setPower(-0.);
            //BackLeft.setPower(-0.);
           // FrontRight.setPower(-0.);
            //BackRight.setPower(-0.);

        ///turn anticlockwise
        FrontLeft.setPower(0.1);
        BackLeft.setPower(0.1);
        FrontRight.setPower(-0.1);
        BackRight.setPower(-0.1);
        runtime.reset();
        while   ((runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        while (Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < Math.abs(rotation_angle3) && opModeIsActive()) {
            ///turn anticlockwise
            FrontLeft.setPower(-0.5);
            BackLeft.setPower(-0.5);
            FrontRight.setPower(0.5);
            BackRight.setPower(0.5);

        }
        while(Math.abs(imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)>Math.abs(rotation_angle3) && opModeIsActive()){
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

        ///move backward to the net zone for sample 1
        FrontLeft.setPower(-0.75);
        BackLeft.setPower(-0.75);
        FrontRight.setPower(-0.75);
        BackRight.setPower(-0.75);
        runtime.reset();
        while   ((runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();


        }




        //Lift
        runtime.reset();
        lift_motor_power = 0.7;
        LiftMotor.setPower(-lift_motor_power);
        LiftMotor.setTargetPosition(extention_target);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotorCurrentDirection = "up";
        //Remove  Power from the lift Motor
        while(  opModeIsActive() && LiftMotorCurrentDirection.equals("up") && (LiftMotor.getCurrentPosition() < extention_target) && (runtime.seconds() < 3)){
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
    //    idle();

        safeWaitSeconds(0.75);
        BigWrist.setPosition(servoposition);
        safeWaitSeconds(0.5);
        /*
        ///move forward to the net zone for sample 1
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while(   (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

         */

        //Turn anticlockwise to Park
        FrontLeft.setPower(-0.85);
        BackLeft.setPower(-0.85);
        FrontRight.setPower(0.85);
        BackRight.setPower(0.85);
        runtime.reset();
        while   (opModeIsActive() &&(runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Move forward
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Move forward to Park
        fl_target = (int) (FrontLeft.getCurrentPosition()+(int) sample_forward3_target*ticks_per_rev/(diameter*3.14));
        fr_target = (int) (FrontRight.getCurrentPosition()+(int) sample_forward3_target*ticks_per_rev/(diameter*3.14));
        bl_target = (int) (BackLeft.getCurrentPosition()+(int) sample_forward3_target*ticks_per_rev/(diameter*3.14));
        br_target = (int) (BackRight.getCurrentPosition()+(int) sample_forward3_target*ticks_per_rev/(diameter*3.14));

        FrontLeft.setTargetPosition(fl_target);
        FrontRight.setTargetPosition(fr_target);
        BackLeft.setTargetPosition(bl_target);
        BackRight.setTargetPosition(br_target);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Forward
        FrontLeft.setPower(0.95);
        BackLeft.setPower(0.95);
        FrontRight.setPower(0.95);
        BackRight.setPower(0.95);
        while( opModeIsActive() &&  FrontLeft.isBusy()) {
            telemetry.addData("FL", FrontLeft.getCurrentPosition());
            telemetry.update();
            LiftMotor.setPower(-lift_motor_power);
            LiftMotor.setTargetPosition(retraction_target);
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotorCurrentDirection = "down";

            while( opModeIsActive() &&  LiftMotorCurrentDirection.equals("down") && (LiftMotor.getCurrentPosition() > retraction_target)) {

                telemetry.addData("Lift motor position", LiftMotor.getCurrentPosition());
                telemetry.update();
            }
            // idle();
            //Remove  Power from the lift Motor
            lift_motor_power = 0;
            LiftMotor.setPower(lift_motor_power);
            LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        while( opModeIsActive() &&  FrontRight.isBusy()) {
            telemetry.addData("FR", FrontRight.getCurrentPosition());
            telemetry.update();
        }

        while( opModeIsActive() &&  BackLeft.isBusy()) {
            telemetry.addData("BL", BackLeft.getCurrentPosition());
            telemetry.update();
        }

        while( opModeIsActive() &&  BackRight.isBusy()) {
            telemetry.addData("BR", BackRight.getCurrentPosition());
            telemetry.update();
        }


       // idle();
        FrontLeft.setPower(-0.);
        BackLeft.setPower(0.);
        FrontRight.setPower(0.);
        BackRight.setPower(-0.);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        //
        /*
        //Strafe left
        FrontLeft.setPower(-0.95);
        BackLeft.setPower(0.95);
        FrontRight.setPower(0.95);
        BackRight.setPower(-0.95);
        runtime.reset();
        while  (opModeIsActive() &&(runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        ///move forward to Park
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while  (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

         */

    }
}
