package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "B2 (Blocks to Java)")
public class B2 extends LinearOpMode {

  private DcMotor left_front_drive;
  private DcMotor right_front_drive;
  private CRServo intake;
  private Servo wrist;
  private DcMotor left_arm;

  double ARM_COLLECT;
  double armPosition;
  double ARM_DOWN;
  double ARM_SCORE_SAMPLE_IN_LOW;
  double ARM_ATTACH_HANGING_HOOK;
  double ARM_WINCH_ROBOT;
  int INTAKE_COLLECT;
  int INTAKE_OFF;
  int INTAKE_DEPOSIT;
  double FUDGE_FACTOR;

  /**
   * * This OpMode is an example driver-controlled (TeleOp) mode for the goBILDA 2024-2025 FTC
   *  * Into The Deep Starter Robot
   *  * The code is structured as a LinearOpMode
   *  *
   * * This robot has a two-motor differential-steered (sometimes called tank or skid steer) drivetrain.
   *  * With a left and right drive motor.
   * * The drive on this robot is controlled in an "Arcade" style, with the left stick Y axis
   * * controlling the forward movement and the right stick X axis controlling rotation.
   *  * This allows easy transition to a standard "First Person" control of a
   *  * mecanum or omnidirectional chassis.
   *  *
   *  * The drive wheels are 96mm diameter traction (Rhino) or omni wheels.
   * * They are driven by 2x 5203-2402-0019 312RPM Yellow Jacket Planetary Gearmotors.
   *  *
   * * This robot's main scoring mechanism includes an arm powered by a motor, a "wrist" driven
   *  * by a servo, and an intake driven by a continuous rotation servo.
   *  *
   * * The arm is powered by a 5203-2402-0051 (50.9:1 Yellow Jacket Planetary Gearmotor) with an
   *  * external 5:1 reduction. This creates a total ~254.47:1 reduction.
   * * This OpMode uses the motor's encoder and the RunToPosition method to drive the arm to
   * * specific setpoints. These are defined as a number of degrees of rotation away from the arm's
   *  * starting position.
   *  *
   * * Make super sure that the arm is reset into the robot, and the wrist is folded in before
   * * you run start the OpMode. The motor's encoder is "relative" and will move the number of degrees
   * * you request it to based on the starting position. So if it starts too high, all the motor
   *  * setpoints will be wrong.
   *  *
   *  * The wrist is powered by a goBILDA Torque Servo (2000-0025-0002).
   *  *
   * * The intake wheels are powered by a goBILDA Speed Servo (2000-0025-0003) in Continuous Rotation mode.
   */
  @Override
  public void runOpMode() {
    double armPositionFudgeFactor;

    left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
    right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
    intake = hardwareMap.get(CRServo.class, "intake");
    wrist = hardwareMap.get(Servo.class, "wrist");
    left_arm = hardwareMap.get(DcMotor.class, "left_arm");

    // A function to create and set the target positions for different scoring positions.
    SetDefaultPositions();
    // A function to set the default settings for the drive and arm motors
    SetMotorSettings();
    // Make sure that the intake is off, and the wrist is folded in.
    waitForStart();
    armPosition = ARM_DOWN;
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // Set the drive and turn variables to follow the joysticks on the gamepad.
        // Here we "mix" the input channels together to find the power to apply to each motor.
        // The both motors need to be set to a mix of how much you're retesting the robot move
        // forward, and how much you're requesting the robot turn. When you ask the robot to rotate
        // the right and left motors need to move in opposite directions. So we will add rotate to
        // forward for the left motor, and subtract rotate from forward for the right motor.
        left_front_drive.setPower(-gamepad1.right_stick_x + gamepad1.left_stick_y);
        right_front_drive.setPower(-gamepad1.right_stick_x - gamepad1.left_stick_y);
        // Set the Intake speed based on which button on the gamepad was pressed
        if (gamepad1.left_bumper) {
          while (gamepad1.left_bumper) {
            intake.setPower(INTAKE_COLLECT);
          }
        } else {
          if (gamepad1.right_bumper) {
            while (gamepad1.right_bumper) {
              intake.setPower(INTAKE_DEPOSIT);
            }
          } else {
            intake.setPower(INTAKE_OFF);
          }
        }
        // Create Arm Fudge Factor, which lets us adjust the position of our deposit slightly with the triggers
        // Here we create a "fudge factor" for the arm position.
        // This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
        // We want the left trigger to move the arm up, and right trigger to move the arm down.
        // So we add the right trigger's variable to the inverse of the left trigger. If you pull
        // both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
        // than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
        // The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function.
        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + -gamepad1.left_trigger);
        // Set the arm position, wrist position, and intake state based on buttons pressed by the driver
        // Here we implement a set of if else loops to set our arm to different scoring positions.
        // We check to see if a specific button is pressed, and then move the arm (and sometimes
        // intake and wrist) to match. For example, if we click the right bumper we want the robot
        // to start collecting. So it moves the armPosition to the ARM_COLLECT position,
        // it folds out the wrist to make sure it is in the correct orientation to intake, and it
        // turns the intake on to the COLLECT mode.
        if (gamepad1.a) {
          // This is the intaking/collecting arm position
          armPosition = ARM_COLLECT;
        } else {
          if (gamepad1.y) {
            // This is the correct height to score the sample in the LOW BASKET
            armPosition = ARM_SCORE_SAMPLE_IN_LOW;
          } else {
            if (gamepad1.b) {
              armPosition = ARM_DOWN;
            } else {
              if (gamepad1.dpad_up) {
                // This sets the arm to vertical to hook onto the LOW RUNG for hanging
                armPosition = ARM_ATTACH_HANGING_HOOK;
              } else {
                if (gamepad1.dpad_down) {
                  armPosition = ARM_WINCH_ROBOT;
                } else {
                  if (gamepad1.dpad_left) {
                    wrist.setPosition(0.52);
                  } else {
                    if (gamepad1.dpad_right) {
                      wrist.setPosition(0.85);
                    }
                  }
                }
              }
            }
          }
        }
        // Set the target postiion of the arm based on the last variable selected by the driver
        left_arm.setTargetPosition((int) Math.round(armPosition + armPositionFudgeFactor));
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
      }
    }
  }

  /**
   * This function defines the arm target positions, wrist target positions, and intake speeds.
   */
  private void SetDefaultPositions() {
    double ARM_TICKS_PER_DEGREE;
    int ARM_COLLAPSED_INTO_ROBOT;
    double ARM_CLEAR_BARRIER;
    double ARM_SCORE_SPECIMEN;
    int WRIST_FOLDED_OUT;
    int WRIST_FOLDED_IN;

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
    WRIST_FOLDED_OUT = 1;
    WRIST_FOLDED_IN = -1;
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
