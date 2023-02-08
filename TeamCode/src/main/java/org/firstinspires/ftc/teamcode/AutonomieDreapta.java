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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import androidx.annotation.Nullable;

@Autonomous(name = "AutonomieDreapta", group = "AutonomousMode")
//@Disabled
public class AutonomieDreapta extends LinearOpMode {

    // VARIABILE

    // Declararea variabilelor, se declara cu null toate, tipurile lor sunt DcMotor sau Servo sau ce mai puneti pe robot

    // Motoarele de la roti
    private DcMotor motorDreaptaFata = null;
    private DcMotor motorStangaFata = null;
    private DcMotor motorDreaptaSpate = null;
    private DcMotor motorStangaSpate = null;

    // Motoarele de la brat (cele care actioneaza glisierele din lateral)
    private DcMotor motorBrat1 = null;
    private DcMotor motorBrat2 = null;
    private DcMotor motorBratCentral = null;



    // Servo-ul care actioneaza clestele
    private Servo servoGrab1 = null;
    private Servo servoGrab2 = null;
    private Servo servoRotate=null;

    // Acestea sunt valorile minime si maxime care pot fii atribuite motoarelor si servo-urilor
    private double MIN_POSITION = 0, MAX_POSITION = 1;

    // Valoarea puterii cu care se initiaza motoarele (0, ca sa nu se miste)
    private final static int powerInit = 0;

    // Valorile puterilor cu care se poate deplasa robotul
    private final static double morePower = 0.8; // nu e in tlf
    private final static double lessPower = 0.6; // nu e in tlf

    // Variabila care contine valoarea puterii cu care se misca bratul
    private double powerBrat1 = powerInit;
    private double powerBrat2 = powerInit;


    private double positionGrab = 0.1;

    // Valoarea puterii maxime cu care poate fii actionat bratul
    private double powerActiune = 1;

    // Variabila care contine valoarea puterii cu care se deplaseaza robotul
    private double powerMiscareFata = lessPower;

    // Variabilele care contin valoarea puterilor cu care se vor misca motoarele de la roti
    private double powerDreaptaFata = powerInit;
    private double powerStangaFata = powerInit;
    private double powerDreaptaSpate = powerInit;
    private double powerStangaSpate = powerInit;



    // Variabila care contine numele fisierului de tip tflite care contine Modelul nostru de TensorFlow

    // Acesta se adauga in FtcRobotController/src/assets/

    // Modelul de TensorFlow NU se poate face pe Teachable Machine, doar pe FTC Machine Learning Toolchain,
    // deoarece SDK-ul de la FTC suporta strict modele de TensorFlow pentru Object Detection, in timp ce cu
    // Teachable Machine poti face modele doar pentru Image Detection (care functioneaza diferit)

    // Daca aceasta regula nu este respectata, atunci cand veti da run, aplicatia FTC Robot Controller din
    // telefon va da crash atunci cand dati Init din Driver Station
    private static final String TFOD_MODEL_ASSET = "CustomModel.tflite";

    // Acest string contine label-urile din TensorFlow (numele obiectelor)
    private static final String[] LABELS = {
            "Cerc",
            "Plus",
            "Triunghi"
            // TODO junctiune si cele 4 imagini
    };

    // Expresiile logice care contin informatii despre zona de semnal primita la randomizare (aflata pe signal cone)
    boolean estePlus = false;
    boolean esteCerc = false;
    boolean esteTriunghi = false;

    // Variabila care contine informatii despre Runtime (timpul de la Initializare)
    private ElapsedTime runtime = new ElapsedTime();

    // Aceasta este cheia de developer Vuforia care este necesara ca webcam-ul sa transmita informatii spre TensorFlow
    // O puteti obtine intrand pe site-ul lor si accesand Basic Plan-ul lor gratiut, care va va da o cheie
    // Aceasta este un string de 380 de caractere random care va va permite utilizarea platformei, in principiu, nelimitat
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

        initVuforia(); // Mereu aceasta intializare trebuie sa fie prima !!!
        initTfod();
        initHardware();



        // Se activeaza TensorFlow Object Detection (MEREU INAINTE DE WAIT FOR START)
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0 / 9.0);
        }

        // Se asteapta apasarea butonului de Start
        waitForStart();

        if (opModeIsActive()) {
                runtime.reset(); // Reseteaza runtime-ul, ca valoarea timpului sa fie de 0 secunde

                // Cat timp semnalul nu a fost gasit si nu au trecut mai mult de 5 secunde de la Start
                // se vor cauta zonele de semnal de pe signal sleeve
                while (tfod != null && !isSignalFound() && runtime.seconds() < 5) {
                    // Se creeaza o lista de tip Recognition, ce contin informatii despre obiectele recunoscute,
                    // cum ar fii Label (numele acestuia), Confidence (increderea ca acela este cu adevarat
                    // obiectul recunoscut) si informatii despre pozitionarea acestuia fata de Webcam

                    // getUpdatedRecognitions() va returna null daca nu gaseste nimic la ultima apelare a functiei
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

                            telemetry.update();

                            if (recognition.getLabel().equals("Plus")) {
                                estePlus = true;
                                telemetry.addData("Object detected", "Plus");
                                telemetry.update();
                            } else if (recognition.getLabel().equals("Cerc")) {
                                esteCerc = true;
                                telemetry.addData("Object detected", "Cerc");
                                telemetry.update();
                            } else if (recognition.getLabel().equals("Triunghi")){
                                esteTriunghi = true;
                                telemetry.addData("Object detected", "Triunghi");
                                telemetry.update();

                            }
                        }
                        telemetry.update();
                    }
                }

                MersSpreStack();
                //PuneConuri();
                sleep(1000);

                if(isSignalFound()){
                    if(estePlus) MersSpreZona3();
                    else if(esteCerc) MersSpreZona2();
                    else if(esteTriunghi) MersSpreZona1();
                }

                //return;
            }
        }

    private void PuneConuri() {

    }

    private void MersSpreStack() {
        Miscare("Spate", 0.15);
        sleep(500);
        Miscare("Stanga", 1);
        sleep(500);
        Rotatie("Dreapta",1.4);
        sleep(500);
        Miscare("Fata", 1);
        Miscare("Stanga", 0.1);
        Miscare("Fata", 0.25);
        sleep(500);
        Miscare("Stanga", 0.3);
        sleep(500);
        Rotatie("Dreapta", 0.6);
        sleep(500);
    }

    private void MersSpreZona1() {
        // Miscare care functioneaza fara functia de mers spre stack
        //Miscare("Spate", 0.75);
        //sleep(100);
        //Miscare("Dreapta", 1.05);
        return;
    }

    private void MersSpreZona2() {
        // Miscare care functioneaza fara functia de mers spre stack
        //Miscare("Spate", 1);
        return;
    }

    private void MersSpreZona3(){
        // Miscare care functioneaza fara functia de mers spre stack
        //Miscare("Spate", 0.75);
        //sleep(100);
        //Miscare("Stanga", 1);
        return;
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
        //tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    // Metoda care face robotul sa se miste intr-o directie data pentru un anumit interval de timp exprimat in secunde
    private void Miscare(String directie, double secunde) {
        switch (directie) {

            case "Dreapta":
                runtime.reset();
                while (runtime.seconds() < secunde && opModeIsActive()) {
                    setPowerRoti(powerMiscareFata, -powerMiscareFata, -powerMiscareFata, powerMiscareFata);
                }
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;

            case "Stanga":
                runtime.reset();
                while (runtime.seconds() < secunde && opModeIsActive()) {

                    setPowerRoti(-powerMiscareFata, powerMiscareFata, powerMiscareFata, -powerMiscareFata);
                }
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;

            case "Fata":
                runtime.reset();
                while (runtime.seconds() < secunde && opModeIsActive()) {

                    setPowerRoti(-powerMiscareFata, -powerMiscareFata, -powerMiscareFata, -powerMiscareFata);
                }
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;

            case "Spate":
                runtime.reset();
                while (runtime.seconds() < secunde && opModeIsActive()) {

                    setPowerRoti(powerMiscareFata, powerMiscareFata, powerMiscareFata, powerMiscareFata);
                }
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;

            default:
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;
        }
    }

    // Metoda care face robotul sa se roteasca intr-o directie data pentru un anumit interval de timp exprimat in secunde
    private void Rotatie(String directie, double secunde){
        switch (directie){

            case "Stanga":
                runtime.reset();
                while(runtime.seconds() < secunde && opModeIsActive()){
                    setPowerRoti(-powerMiscareFata, powerMiscareFata, -powerMiscareFata, powerMiscareFata);
                }
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;

            case "Dreapta":
                runtime.reset();
                while(runtime.seconds() < secunde && opModeIsActive()){
                    setPowerRoti(powerMiscareFata, -powerMiscareFata, powerMiscareFata, -powerMiscareFata);
                }
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;
        }
    }

    // Metoda care face robotul sa isi ridice/coboare glisierele mari pentru un anumit interval de timp exprimat in secunde
    private void RidicaGlisiereleMari(String directie, double secunde){
        switch (directie){
            case "Sus":
                runtime.reset();
                while(runtime.seconds() < secunde && opModeIsActive()){

                }
                break;

            case "Jos":
                runtime.reset();
                while(runtime.seconds() < secunde && opModeIsActive()) {

                }
                break;
        }
    }

    // Metoda care face robotul sa isi ridice/coboare glisiera centrala pentru un anumit interval de timp exprimat in secunde
    private void RidicaGlisieraCentrala(String directie, double secunde){
        switch (directie){
            case "Sus":
                runtime.reset();
                while(runtime.seconds() < secunde && opModeIsActive()){
                    setPowerGlisiere("Mari", powerActiune);
                }
                runtime.reset();
                break;

            case "Jos":
                runtime.reset();
                while(runtime.seconds() < secunde && opModeIsActive()){
                    setPowerGlisiere("Mari", powerActiune);
                }
                break;
        }
    }

    // Seteaza puterea rotilor
    private void setPowerRoti(double powerDreaptaFata, double powerStangaFata, double powerDreaptaSpate, double powerStangaSpate) {
        motorDreaptaFata.setPower(powerDreaptaFata);
        motorStangaFata.setPower(powerStangaFata);
        motorDreaptaSpate.setPower(powerDreaptaSpate);
        motorStangaSpate.setPower(powerStangaSpate);
    }

    // Seteaza puterea glisierelor mari, respectiv glisierei centrale
    private void setPowerGlisiere(String glisiera, double powerActiune){
        switch (glisiera) {
            case "Mari":
                motorBrat1.setPower(powerActiune);
                motorBrat2.setPower(powerActiune);
                break;
            case "Central":
                motorBratCentral.setPower(powerActiune);
                break;
        }
    }

    // Functia care initializeaza motoarele
    private void initMotor(@Nullable DcMotor motor, String string, boolean sens, double power, boolean mode, boolean... secMode) {
        motor = hardwareMap.dcMotor.get(string);
        if (secMode.length > 0) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        motor.setMode(mode ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setDirection(sens ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        if (motor != null) {
            motor.setPower(power);
        }
    }

    // Metoda care intializeaza motoarele, servo-urile si tot ce tine de Hardware pe robot (cu exceptia webcam-ului)
    private void initHardware(){

        //Metoda hardwareMap preia din expansion hub informatii despre motor/servo si nu mai este null, daca primiti eroarea NullPointerException s-ar putea sa fie din cauza ca nu ati initializat aici motorul si acesta a ramas null
        motorDreaptaFata = hardwareMap.dcMotor.get("motorDreaptaFata");
        motorDreaptaSpate = hardwareMap.dcMotor.get("motorDreaptaSpate");
        motorStangaFata = hardwareMap.dcMotor.get("motorStangaFata");
        motorStangaSpate = hardwareMap.dcMotor.get("motorStangaSpate");

        motorBrat1 = hardwareMap.dcMotor.get("motorBrat1");
        motorBrat2 = hardwareMap.dcMotor.get("motorBrat2");
        motorBratCentral = hardwareMap.dcMotor.get("motorBratCentral");

        servoGrab1 = hardwareMap.servo.get("servoGrab1");
        servoGrab2 = hardwareMap.servo.get("servoGrab2");
        servoRotate = hardwareMap.servo.get("servoRotate");


        //Apelarea metodei initMotor declarata mai jos(obligatorie)
        initMotor(motorDreaptaFata, "motorDreaptaFata", true, powerInit, true, true);
        initMotor(motorStangaFata, "motorStangaFata", false, powerInit, true, true);
        initMotor(motorDreaptaSpate, "motorDreaptaSpate", true, powerInit, true, true);
        initMotor(motorStangaSpate, "motorStangaSpate", false, powerInit, true, true);

        initMotor(motorBrat1, "motorBrat1", false, powerInit, true, true);
        initMotor(motorBrat2, "motorBrat2", false, powerInit, true, true);
        initMotor(motorBratCentral, "motorBratCentral", false, powerInit, true, true);


        //Motoatele pot rula cu encoder sau fara encoder, encoderul este un cablu care care masoara diferite chestii despre motor, daca nu folositi encoder trebuie sa setati modul asta
        motorDreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBrat1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBrat2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBratCentral.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Aici se seteaza diretia motorului, reverse sau forward, e simplu de inteles
        motorDreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorStangaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaSpate.setDirection(DcMotorSimple.Direction.FORWARD);

        motorBrat1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBrat2.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBratCentral.setDirection(DcMotorSimple.Direction.FORWARD);

        //servoGrab1.setDirection(Servo.Direction.REVERSE);
        servoGrab2.setDirection(Servo.Direction.REVERSE);

        /*servoRotate.setPosition(MIN_POSITION);
        servoGrab1.setPosition(MIN_POSITION);
        servoGrab2.setPosition(MIN_POSITION);*/

        setPowerRoti(powerDreaptaFata, powerStangaFata, powerDreaptaSpate, powerStangaSpate);
    }

    private boolean isSignalFound(){
        return estePlus || esteCerc || esteTriunghi;
    }
}
