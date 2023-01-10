/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Autonomous", group = "AutonomousMode")
//@Disabled
public class TensorFlowObjectDetectionWebcam extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "CustomModel.tflite";
    //private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "0 Plus",
            "1 Cerc",
            "2 Triunghi"
            // TODO junctiune si cele 4 imagini
    };

    boolean estePlus = false;
    boolean esteCerc = false;
    boolean esteTriunghi = false;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorDreaptaFata = null;
    private DcMotor motorStangaFata = null;
    private DcMotor motorDreaptaSpate = null;
    private DcMotor motorStangaSpate = null;
    private DcMotor motorBrat = null;
    private Servo Grabber = null;

    private final static int powerInit = 0;
    private double powerMiscareFata = 0.5;
    private double powerActiune = 0.8;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .50;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AYjRNx7/////AAABmTp7BF4SlE4Vq3lfkFG3MtU459ynZzi2xIyqOf8k9JXVlkbaKpBgVn0It4fpBjfZuBIhMdp3HjEFC/qoZykpnUnZsyiZbWKFesXC4yWtC5GkiVNjL/wMIX077+2SIURNGqKEsdKMs1VvGSqude9QIzRN3vzrpjKAoYGwvqELfdR8TqONI0nfTSYwmAvTIFVvBVhDf8bKPX3o7Fjpv6vTI4UP/5Weq469ateV0NYnGYHe4gs7KeigZKwT2ZOYws/I8g8qq9/2ZsfxKsnw0YMMLcbu68AzkNm6itcSep5/9wB32mT8rzIE0YWo/deKzja9mkFnwrIwXJiqlXyo1dt8RB4G1EzwvR5gjVr1jFSypvfi";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();



        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                            if (recognition.getLabel().equals("1 Bolt") && !isSignalFound()) {
                                estePlus = true;
                                esteCerc = false;
                                esteTriunghi = false;
                                telemetry.addData("Object detected", "0 Plus");
                                break;
                            } else if (recognition.getLabel().equals("2 Bulb") && !isSignalFound()) {
                                estePlus = false;
                                esteCerc = true;
                                esteTriunghi = false;
                                telemetry.addData("Object detected", "1 Cerc");
                                break;
                            } else if (recognition.getLabel().equals("3 Panel") && !isSignalFound()){
                                estePlus = false;
                                esteCerc = false;
                                esteTriunghi = true;
                                telemetry.addData("Object detected", "2 Triunghi");
                                break;
                            }
                        }
                        telemetry.update();
                    }
                    if(!isSignalFound()){
                        if(estePlus) MersSpreZona1();
                        else if(esteCerc) MersSpreZona2();
                        else if(esteTriunghi) MersSpreZona3();
                    }

                    // TODO cod pentru punerea conului pe junctiuni (pune-l intr-o functie recursiva)
                    updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null){
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                            // Daca obiectul recunoscut este o junctiune
                                // isJunction = true
                                // Iesi din loop (break)
                        }
                    }
                    // Daca isJunction este adevarat
                        // Aliniat robotul spre junctiune
                        // Pus conul pe junctiune
                    // Altfel
                        // Roteste robotul si cauta obiecte
                }
            }
        }
    }

    private void MersSpreZona1() {
        //  TODO Traseul spre zona 1
    }

    private void MersSpreZona2() {
        // TODO Traseul spre zona 2
    }

    private void MersSpreZona3(){
        // TODO Traseul spre zona 3
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    private void miscareMechanum(String directie, double ticks) {
        resetAngle();
        switch (directie) {

            case "Dreapta":
                while (runtime.seconds() < ticks && opModeIsActive()) {
                    setPower(-powerMiscareFata, powerMiscareFata, -powerMiscareFata, powerMiscareFata);
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

                    setPower(-powerMiscareFata, -powerMiscareFata, -powerMiscareFata, -powerMiscareFata);
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

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {
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

    private void rotate(int degrees) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = -power;
            rightPower = power;
        } else if (degrees > 0) {   // turn left.
            leftPower = power;
            rightPower = -power;
        } else return;

        // set power to rotate.
        setPower(rightPower, leftPower, rightPower, leftPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        setPower(0, 0, 0, 0);

        // wait for rotation to stop.

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void initMotors(){
        motorDreaptaFata = hardwareMap.dcMotor.get("motorDreaptaFata");
        motorStangaFata = hardwareMap.dcMotor.get("motorStangaFata");
        motorDreaptaSpate = hardwareMap.dcMotor.get("motorDreaptaSpate");
        motorStangaSpate = hardwareMap.dcMotor.get("motorStangaSpate");

        motorBrat = hardwareMap.dcMotor.get("motorBrat");

        motorDreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorDreaptaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaSpate.setDirection(DcMotorSimple.Direction.FORWARD);

        motorBrat.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private boolean isSignalFound(){
        return estePlus || esteCerc || esteTriunghi;
    }
}