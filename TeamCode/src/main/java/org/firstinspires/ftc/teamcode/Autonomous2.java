package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name = "A2 (Blocks to Java)")
public class Autonomous2 extends LinearOpMode {

    private CRServo intake;
    private DcMotor left_arm;
    private DcMotor right_front_drive;
    private DcMotor left_front_drive;

    int INTAKE_OFF;
    int armPosition;


    @Override
    public void runOpMode() {
        intake = hardwareMap.get(CRServo.class, "intake");
        left_arm = hardwareMap.get(DcMotor.class, "left_arm");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");

        // A function to create and set the target positions for different scoring positions.
        SetDefaultPositions();
        // A function to set the default settings for the drive and arm motors
        SetMotorSettings();
        // Make sure that the intake is off, and the wrist is folded in.
        waitForStart();
        intake.setPower(INTAKE_OFF);
        left_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Here we set the target velocity (speed) the motor runs at. This is in ticks-per-second.
        // This is a little under the maximum of 2800.
        ((DcMotorEx) left_arm).setVelocity(2100);
        // Check to see if the arm motor is over the current limit and report via telemetry.
        if (((DcMotorEx) left_arm).isOverCurrent()) {
            telemetry.addLine("MOTOR IS OVER CURRENT");
        }
        telemetry.addData("Arm Target Position:", armPosition);
        telemetry.addData("Arm Current Position:", left_arm.getCurrentPosition());
        telemetry.update();

        // 1. Move the bot FW
        // 2. Move the arm up
        // 3. Move the bot FW
        // 4. Activate the intake (to drop the sample in the basket)


        if (opModeIsActive()) {
            // 1. Move the bot FW
            right_front_drive.setPower(0.5);
            left_front_drive.setPower(-0.5);
            sleep(700);
            // GUESSING where the bot is really -> TODO: define how much to move
            // READ TELEMETRY DATA: to know where the bot is and act with that information instead
            right_front_drive.setPower(0);
            left_front_drive.setPower(0);

            // 2. Move the arm up
            left_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //       right_front_drive.setPower(-0.5);
            //       left_front_drive.setPower(0.5);
            //       sleep(1000);
            //      right_front_drive.setPower(0.5);
            //      left_front_drive.setPower(0.5);
            //      sleep(200);
            //      right_front_drive.setPower(0);
            //      left_front_drive.setPower(0);
            //      sleep(300);
            left_arm.setTargetPosition(4000);
            sleep(1050);
            intake.setPower(1);
            sleep(300);
        }
    }

    /**
     * This function defines the arm target positions, wrist target positions, and intake speeds.
     */
    private void SetDefaultPositions() {
        double ARM_TICKS_PER_DEGREE;
        int ARM_COLLAPSED_INTO_ROBOT;
        double ARM_COLLECT;
        double ARM_CLEAR_BARRIER;
        double ARM_SCORE_SPECIMEN;
        double ARM_SCORE_SAMPLE_IN_LOW;
        double ARM_ATTACH_HANGING_HOOK;
        double ARM_WINCH_ROBOT;
        double ARM_DOWN;
        int INTAKE_COLLECT;
        int INTAKE_DEPOSIT;
        double WRIST_FOLDED_OUT;
        double WRIST_FOLDED_IN;
        double FUDGE_FACTOR;

        // This constant is the number of encoder ticks for each degree of rotation of the arm.
        // To find this, we first need to consider the total gear reduction powering our arm.
        // First, we have an external 20t:100t (5:1) reduction created by two spur gears.
        //     But we also have an internal gear reduction in our motor.
        // The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
        //     reduction of ~50.9:1. (more precisely it is 250047/4913:1)
        // We can multiply these two ratios together to get our final reduction of ~254.47:1.
        // The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
        // counts per rotation of the arm. We divide that by 360 to get the counts per degree.
        ARM_TICKS_PER_DEGREE = 19.7924893140647;
        // Define Arm Setpoints in Degrees, multiplied by ARM_TICKS_PER_DEGREE
        // These constants hold the position that the arm is commanded to run to.
        // These are relative to where the arm was located when you start the OpMode. So make sure the
        //     arm is reset to collapsed inside the robot before you start the program.
        //
        // In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
        // This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
        // set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
        // 160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
        // If you'd like it to move further, increase that number. If you'd like it to not move
        //     as far from the starting position, decrease it.
        ARM_COLLAPSED_INTO_ROBOT = 0;
        ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
        ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
        ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
        ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
        ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
        ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;
        ARM_DOWN = 5 * ARM_TICKS_PER_DEGREE;
        // Variables to store the speed the intake servo should be set at to intake, and deposit game elements.
        INTAKE_COLLECT = -1;
        INTAKE_OFF = 0;
        INTAKE_DEPOSIT = 1;
        // Variables to store the positions that the wrist should be set to when folding in, or folding out.
        WRIST_FOLDED_OUT = 0.6;
        WRIST_FOLDED_IN = 0.5;
        // A number in degrees that the triggers can adjust the arm position by
        FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
        // Variables that are used to set the arm to a specific position
        armPosition = 0;
    }

    /**
     * Sets all the motor settings at once
     */
    private void SetMotorSettings() {
        // Most skid-steer/differential drive robots require reversing one motor to drive forward.
        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        // Setting zeroPowerBehavior to BRAKE enables a "brake mode".
        left_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) left_arm).setCurrentAlert(5, CurrentUnit.AMPS);
        // Set the arm motor to runToPosition mode
        left_arm.setTargetPosition(0);
        left_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

