package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class VuforiaTester extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    VuforiaSensor vSensor = new VuforiaSensor();

    public VuforiaTester() {
        super();
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        vSensor.visionTargets.activate();
    }

    @Override
    public void loop() {
        driveRobot();
        vuforiaLoop();
    }

    @Override
    public void stop() {

    }

    public void leftDrive(double power) {
        motorFL.setPower(power);
        motorBL.setPower(power);
    }

    public void rightDrive(double power) {
        motorFR.setPower(power);
        motorBR.setPower(power);
    }

    public void driveRobot() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftDrive(leftPower);
        rightDrive(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("gpad1LX", gamepad1.left_stick_x);
        telemetry.addData("gpad1LY", gamepad1.left_stick_y);
    }

/////////////////////// VUFORIA ////////////////////////////////////////////////////////////////////

    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    private void vuforiaLoop() {
        // Ask the listener for the latest information on where the robot is
        OpenGLMatrix latestLocationBackPerimeter = vSensor.listenerBackPerimeter.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationBluePerimeter = vSensor.listenerBluePerimeter.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationRedPerimeter = vSensor.listenerRedPerimeter.getUpdatedRobotLocation();
        OpenGLMatrix latestLocationFrontPerimeter = vSensor.listenerFrontPerimeter.getUpdatedRobotLocation();

        // The listener will sometimes return null, so we check for that to prevent errors
        if (latestLocationBackPerimeter != null)
            vSensor.lastKnownLocationBackPerimeter = latestLocationBackPerimeter;
        if (latestLocationBluePerimeter != null)
            vSensor.lastKnownLocationBluePerimeter = latestLocationBluePerimeter;
        if (latestLocationRedPerimeter != null)
            vSensor.lastKnownLocationRedPerimeter = latestLocationRedPerimeter;
        if (latestLocationFrontPerimeter != null)
            vSensor.lastKnownLocationFrontPerimeter = latestLocationFrontPerimeter;

        // Send information about whether the target is visible, and where the robot is
        if (vSensor.listenerFrontPerimeter.isVisible())
            telemetry.addData("Tracking", vSensor.targetFrontPerimeter.getName());
        if (vSensor.listenerRedPerimeter.isVisible())
            telemetry.addData("Tracking", vSensor.targetRedPerimeter.getName());
        if (vSensor.listenerBluePerimeter.isVisible())
            telemetry.addData("Tracking", vSensor.targetBluePerimeter.getName());
        if (vSensor.listenerBackPerimeter.isVisible())
            telemetry.addData("Tracking", vSensor.targetBackPerimeter.getName());
    }

/////////////////////// VUFORIA ////////////////////////////////////////////////////////////////////

}
