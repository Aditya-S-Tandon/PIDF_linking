package org.firstinspires.ftc.teamcode.FTC_2024;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

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

@Autonomous(name="Auton_encoder")
public class Auton_encoder extends LinearOpMode {
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    Servo BigWrist = null;
    Servo SmallPivot = null;
    Servo SampleGrabber = null;
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
        //BigWrist = hardwareMap.servo.get("BW");
        //SmallPivot = hardwareMap.servo.get("SP");
       // SampleGrabber = hardwareMap.servo.get("SG");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        double big_wrist_position = 0.4;
        double bigout_wrist_position= 0.1;
        double small_pivot_position = 0.7;
        double sample_grabber_position = 0.7;

        double sample_backward_target = 196;
        double sample_forward_target = 18;
        double sample_left_target = 2526;
        double sample2_left_target = 334;
        double sample3_left_target = 1685;
        double sample2_forward_target = 375;

        double ticks_per_rev = 537.7;
        double diameter = 3.77;

        int fl_target ;
        int fr_target ;
        int bl_target ;
        int br_target;


        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //set encoders to 0
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //set encoders to 0
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //set encoders to 0
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //set encoders to 0
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Zero Power Behavior
        /*FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //code before waitForStart is run when Init button is pressed

        //Zero Power Behavior
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //code before waitForStart is run when Init button is pressed

        //Zero Power Behavior
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //code before waitForStart is run when Init button is pressed

        //Zero Power Behavior
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         */





        //code before wai
        // tForStart is run when Init button is pressed

        while (!isStarted()) {
            //print encoder counts to telemetry while we manually move the ar
            telemetry.addData("Extension motor position", FrontLeft.getCurrentPosition());
            telemetry.update();
        }
        waitForStart();
        /*
        //Action to drop sample
        SmallPivot.setPosition(small_pivot_position);
        safeWaitSeconds(1);
        BigWrist.setPosition(big_wrist_position);
        safeWaitSeconds(0.5);
        SampleGrabber.setPosition(sample_grabber_position);
        //BigWrist.setPosition(bigout_wrist_position);
        //SmallPivot.setPosition(-small_pivot_position);
        //SampleGrabber.setPosition(-sample_grabber_position);
        */




        ///move forward
        fl_target = (int) (FrontLeft.getCurrentPosition()+(int) sample_forward_target*ticks_per_rev/(diameter*3.14));
        fr_target = (int) (FrontRight.getCurrentPosition()+(int) sample_forward_target*ticks_per_rev/(diameter*3.14));
        bl_target = (int) (BackLeft.getCurrentPosition()+(int) sample_forward_target*ticks_per_rev/(diameter*3.14));
        br_target = (int) (BackRight.getCurrentPosition()+(int) sample_forward_target*ticks_per_rev/(diameter*3.14));

        FrontLeft.setTargetPosition(fl_target);
        FrontRight.setTargetPosition(fr_target);
        BackLeft.setTargetPosition(bl_target);
        BackRight.setTargetPosition(br_target);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Forward
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);

        while (opModeIsActive() && FrontLeft.isBusy()) {
            telemetry.addData("FL", FrontLeft.getCurrentPosition());
            telemetry.update();
        }



        runtime.reset();
        while (opModeIsActive() && FrontRight.isBusy()) {
            telemetry.addData("FR", FrontRight.getCurrentPosition());
            telemetry.update();
        }



        runtime.reset();
        while (opModeIsActive() && BackLeft.isBusy()) {
            telemetry.addData("BL", BackLeft.getCurrentPosition());
            telemetry.update();
        }



        runtime.reset();
        while (opModeIsActive() && BackRight.isBusy()) {
            telemetry.addData("BR", BackRight.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("FL", FrontLeft.getCurrentPosition());
        idle();
        FrontLeft.setPower(-0.);
        BackLeft.setPower(0.);
        FrontRight.setPower(0.);
        BackRight.setPower(-0.);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         /*
        ///move forward to the ascend zone for sample 1
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

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


        fl_target = (int) (FrontLeft.getCurrentPosition()-(int) sample_forward_target*ticks_per_rev/(diameter*3.14));
        fr_target = (int) (FrontRight.getCurrentPosition()+(int) sample_forward_target*ticks_per_rev/(diameter*3.14));
        bl_target = (int) (BackLeft.getCurrentPosition()+(int) sample_forward_target*ticks_per_rev/(diameter*3.14));
        br_target = (int) (BackRight.getCurrentPosition()-(int) sample_forward_target*ticks_per_rev/(diameter*3.14));

        FrontLeft.setTargetPosition(fl_target);
        FrontRight.setTargetPosition(fr_target);
        BackLeft.setTargetPosition(bl_target);
        BackRight.setTargetPosition(br_target);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Forward
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        while (opModeIsActive() && FrontLeft.isBusy()) {
            telemetry.addData("FL", FrontLeft.getCurrentPosition());
            telemetry.update();
        }



        runtime.reset();
        while (opModeIsActive() && FrontRight.isBusy()) {
            telemetry.addData("FR", FrontRight.getCurrentPosition());
            telemetry.update();
        }



        runtime.reset();
        while (opModeIsActive() && BackLeft.isBusy()) {
            telemetry.addData("BL", BackLeft.getCurrentPosition());
            telemetry.update();
        }



        runtime.reset();
        while (opModeIsActive() && BackRight.isBusy()) {
            telemetry.addData("BR", BackRight.getCurrentPosition());
            telemetry.update();
        }


        idle();
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

        ///move forward to the ascend zone for sample 1
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        /*

        fl_target = (int) (FrontLeft.getCurrentPosition()+(int) sample_forward_target*ticks_per_rev/(diameter*3.14));
        fr_target = (int) (FrontRight.getCurrentPosition()-(int) sample_forward_target*ticks_per_rev/(diameter*3.14));
        bl_target = (int) (BackLeft.getCurrentPosition()-(int) sample_forward_target*ticks_per_rev/(diameter*3.14));
        br_target = (int) (BackRight.getCurrentPosition()+(int) sample_forward_target*ticks_per_rev/(diameter*3.14));

        FrontLeft.setTargetPosition(fl_target);
        FrontRight.setTargetPosition(fr_target);
        BackLeft.setTargetPosition(bl_target);
        BackRight.setTargetPosition(br_target);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Forward
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        while (opModeIsActive() && FrontLeft.isBusy()) {
            telemetry.addData("FL", FrontLeft.getCurrentPosition());
            telemetry.update();
        }



        runtime.reset();
        while (opModeIsActive() && FrontRight.isBusy()) {
            telemetry.addData("FR", FrontRight.getCurrentPosition());
            telemetry.update();
        }



        runtime.reset();
        while (opModeIsActive() && BackLeft.isBusy()) {
            telemetry.addData("BL", BackLeft.getCurrentPosition());
            telemetry.update();
        }



        runtime.reset();
        while (opModeIsActive() && BackRight.isBusy()) {
            telemetry.addData("BR", BackRight.getCurrentPosition());
            telemetry.update();
        }


        idle();
        FrontLeft.setPower(-0.);
        BackLeft.setPower(0.);
        FrontRight.setPower(0.);
        BackRight.setPower(-0.);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl_target = (int) (FrontLeft.getCurrentPosition()-(int) sample_forward_target*ticks_per_rev/(diameter*3.14));
        fr_target = (int) (FrontRight.getCurrentPosition()-(int) sample_forward_target*ticks_per_rev/(diameter*3.14));
        bl_target = (int) (BackLeft.getCurrentPosition()-(int) sample_forward_target*ticks_per_rev/(diameter*3.14));
        br_target = (int) (BackRight.getCurrentPosition()-(int) sample_forward_target*ticks_per_rev/(diameter*3.14));

        FrontLeft.setTargetPosition(fl_target);
        FrontRight.setTargetPosition(fr_target);
        BackLeft.setTargetPosition(bl_target);
        BackRight.setTargetPosition(br_target);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Forward
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        while (opModeIsActive() && FrontLeft.isBusy()) {
            telemetry.addData("FL", FrontLeft.getCurrentPosition());
            telemetry.update();
        }



        runtime.reset();
        while (opModeIsActive() && FrontRight.isBusy()) {
            telemetry.addData("FR", FrontRight.getCurrentPosition());
            telemetry.update();
        }



        runtime.reset();
        while (opModeIsActive() && BackLeft.isBusy()) {
            telemetry.addData("BL", BackLeft.getCurrentPosition());
            telemetry.update();
        }



        runtime.reset();
        while (opModeIsActive() && BackRight.isBusy()) {
            telemetry.addData("BR", BackRight.getCurrentPosition());
            telemetry.update();
        }


        idle();
        FrontLeft.setPower(-0.);
        BackLeft.setPower(0.);
        FrontRight.setPower(0.);
        BackRight.setPower(-0.);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /*
        //Action to drop sample
        SmallPivot.setPosition(small_pivot_position);
        safeWaitSeconds(1);
        BigWrist.setPosition(big_wrist_position);
        safeWaitSeconds(0.5);
        SampleGrabber.setPosition(sample_grabber_position);
        //BigWrist.setPosition(bigout_wrist_position);
        //SmallPivot.setPosition(-small_pivot_position);
        //SampleGrabber.setPosition(-sample_grabber_position);

         */
        /*
        ///move forward to the ascend zone for sample 2
        target = FrontLeft.getCurrentPosition()+sample_forward_target;
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        FrontLeft.setTargetPosition(target);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (FrontLeft.getCurrentPosition() < target) {
            telemetry.addData("Sample 1 Backward", FrontLeft.getCurrentPosition());
            telemetry.update();
        }
        //Strafe left for sample 2
        target = FrontLeft.getCurrentPosition()+sample_left_target;
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(-0.5);
        FrontLeft.setTargetPosition(target);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (FrontLeft.getCurrentPosition() < target) {
            telemetry.addData("Sample 1 Backward", FrontLeft.getCurrentPosition());
            telemetry.update();
        }
        ///move backward to the net zone for sample 2
        target = FrontLeft.getCurrentPosition()+sample2_backward_target;
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(-0.5);
        FrontLeft.setTargetPosition(target);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (FrontLeft.getCurrentPosition() < target) {
            telemetry.addData("Sample 1 Backward", FrontLeft.getCurrentPosition());
            telemetry.update();
        }
        //Strafe right for sample 2
        target = FrontLeft.getCurrentPosition()+sample_right_target;
        FrontLeft.setPower(0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(0.5);
        FrontLeft.setTargetPosition(target);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (FrontLeft.getCurrentPosition() < target) {
            telemetry.addData("Sample 1 Backward", FrontLeft.getCurrentPosition());
            telemetry.update();
        }

        //Strafe left for sample 3
        target = FrontLeft.getCurrentPosition()+sample_left_target;
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(-0.5);
        FrontLeft.setTargetPosition(target);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (FrontLeft.getCurrentPosition() < target) {
            telemetry.addData("Sample 1 Backward", FrontLeft.getCurrentPosition());
            telemetry.update();
        }

        ///move backward to the net zone for sample 3
        target = FrontLeft.getCurrentPosition()+sample3_backward_target;
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(-0.5);
        FrontLeft.setTargetPosition(target);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (FrontLeft.getCurrentPosition() < target) {
            telemetry.addData("Sample 1 Backward", FrontLeft.getCurrentPosition());
            telemetry.update();
        }
        //Strafe right for sample 3
        target = FrontLeft.getCurrentPosition()+sample_right_target;
        FrontLeft.setPower(0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(0.5);
        FrontLeft.setTargetPosition(target);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (FrontLeft.getCurrentPosition() < target) {
            telemetry.addData("Sample 1 Backward", FrontLeft.getCurrentPosition());
            telemetry.update();
        }

        //Strafe left to park
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


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
*/
    }
}
