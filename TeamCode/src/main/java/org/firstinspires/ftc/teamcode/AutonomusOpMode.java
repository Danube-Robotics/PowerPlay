package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "AutonomusOpModeQuantum" , group = "MechanumAutonomusOp")
@Disabled
public class AutonomusOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorDreaptaFata = null;
    private DcMotor motorStangaFata = null;
    private DcMotor motorDreaptaSpate = null;
    private DcMotor motorStangaSpate = null;

    //private DcMotor motorCarusel = null;
    private DcMotor motorBrat = null;
    //private DcMotor motorIntake = null;

    private final static int powerInit = 0;
    private double powerMiscareFata = 0.5;
    private double powerActiune = 0.8;

    TouchSensor             touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .50, correction;
    boolean                 aButton, bButton, touched;

    @Override
    public void runOpMode() {
        motorDreaptaFata = hardwareMap.dcMotor.get("motorDreaptaFata");
        motorStangaFata = hardwareMap.dcMotor.get("motorStangaFata");
        motorDreaptaSpate = hardwareMap.dcMotor.get("motorDreaptaSpate");
        motorStangaSpate = hardwareMap.dcMotor.get("motorStangaSpate");

        motorBrat = hardwareMap.dcMotor.get("motorBrat");
        //motorCarusel = hardwareMap.dcMotor.get("motorCarusel");
        //motorIntake = hardwareMap.dcMotor.get("motorIntake");

        motorDreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorCarusel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorDreaptaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaSpate.setDirection(DcMotorSimple.Direction.FORWARD);

        //motorCarusel.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBrat.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "gyroSensor");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        // wait for start button.
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Status", "waiting for start");
            telemetry.update();
        }

        /*
        motorCarusel.setPower(0.8);
        setPower(powerMiscareFata , -powerMiscareFata , -powerMiscareFata , powerMiscareFata);
        sleep(3000);
        setPower(powerInit , powerInit , powerInit , powerInit);
        sleep(4000);
        motorCarusel.setPower(powerInit);
        setPower(-powerMiscareFata, powerMiscareFata, powerMiscareFata, -powerMiscareFata);
        waitSec(3);
        setPower(powerInit , powerInit, powerInit, powerInit);
        */


        setPower(-powerMiscareFata, powerMiscareFata, powerMiscareFata, -powerMiscareFata);
        sleep(2000);

        return;
    }

    private void miscareMechanum(String directie, double ticks) {
        resetAngle();
        switch (directie) {

            case "Dreapta":
                while (runtime.seconds() < ticks && opModeIsActive()) {
                    setPower(powerMiscareFata, powerMiscareFata, powerMiscareFata, -powerMiscareFata);
                }
                setPower(powerInit, powerInit, powerInit, powerInit);
                break;

            case "Stanga":
                while (runtime.seconds() < ticks && opModeIsActive()) {

                    setPower(powerMiscareFata, -powerMiscareFata, powerMiscareFata, -powerMiscareFata);
                }
                setPower(powerInit, powerInit, powerInit, powerInit);
                break;

            case "Fata":
                runtime.reset();
                while (runtime.seconds() < ticks && opModeIsActive()) {

                    setPower(powerMiscareFata, powerMiscareFata, powerMiscareFata, powerMiscareFata);
                }
                setPower(powerInit, powerInit, powerInit, powerInit);
                break;

            case "Spate":
                runtime.reset();
                while (runtime.seconds() < ticks && opModeIsActive()) {

                    setPower(-powerMiscareFata , -powerMiscareFata, -powerMiscareFata, -powerMiscareFata);
                }
                setPower(powerInit, powerInit, powerInit, powerInit);
                break;

            default:
                setPower(powerInit, powerInit, powerInit, powerInit);
                break;
        }
    }

    private void setPower(double powerDreaptaFata, double powerStangaFata, double powerDreaptaSpate, double powerStangaSpate) {
        motorDreaptaFata.setPower(powerDreaptaFata);
        motorStangaFata.setPower(powerStangaFata);
        motorDreaptaSpate.setPower(powerDreaptaSpate);
        motorStangaSpate.setPower(powerStangaSpate);
    }

    public void waitSec(double seconds) {
        runtime.reset();
        while (runtime.seconds() < seconds && opModeIsActive()) ;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void rotate(int degrees)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        setPower(rightPower,leftPower,rightPower,leftPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        setPower(0,0,0,0);

        // wait for rotation to stop.

        // reset angle tracking on new heading.
        resetAngle();
    }
}
