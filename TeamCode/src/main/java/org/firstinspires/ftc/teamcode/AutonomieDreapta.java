package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Autonomous(name="Autonomie Dreapta", group="Autonomie")

public class AutonomieDreapta extends LinearOpMode{

    // VARIABILE

    // Declararea variabilelor, se declara cu null toate, tipurile lor sunt DcMotor sau Servo sau ce mai puneti pe robot

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     WHELL_CIRCUMFERENCE_CM    = 9.6;
    static final double INDICE = 13.76225;
    static final double CM_PER_DEG = 0.60599500129;

    // Motoarele de la roti
    private DcMotorEx motorDreaptaFata = null;
    private DcMotorEx motorStangaFata = null;
    private DcMotorEx motorDreaptaSpate = null;
    private DcMotorEx motorStangaSpate = null;

    // Motoarele de la brat (cele care actioneaza glisierele din lateral)
    private DcMotor motorBrat1 = null;
    private DcMotor motorBrat2 = null;

    // Servo-ul care actioneaza clestele
    private Servo servoGrab1 = null;
    private Servo servoGrab2 = null;

    // Acestea sunt valorile minime si maxime care pot fii atribuite motoarelor si servo-urilor
    private double MIN_POSITION = 0, MAX_POSITION = 1;

    // Valoarea puterii cu care se initiaza motoarele (0, ca sa nu se miste)
    private final static int powerInit = 0;

    // Variabila care contine valoarea puterii cu care se misca bratul
    private double powerBrat1 = powerInit;
    private double powerBrat2 = powerInit;

    // Variabila care contine valoarea pozitiei la care se inchide grabberul/clestele
    private double positionGrab = 0.065;

    // Valoarea puterii maxime cu care poate fii actionat bratul
    private double powerActiune = 1;

    // Variabila care contine valoarea puterii cu care se deplaseaza robotul
    private double powerMiscareRoti = 0.34;

    // Variabila care contine informatii despre Runtime (timpul de la Initializare)
    private ElapsedTime runtime = new ElapsedTime();

    // Variabila care contine instanta webcam-ului utilizat folosind libraria OpenCV
    OpenCvCamera camera;

    // Variabila care contine instanta Pipeline-ului folosit pentru detectarea April Tag-urilor
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Intrinsecii lentilei webcam-ului (cadrul)
    // Unitatea de masura este pixelul
    // Aceasta este calibrarea pentru webcam-ul Logitech C920 la 800x448
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // Unitatea de masura este metrul
    double tagsize = 0.166;

    // Variabila care contine informatii despre tag-ul pe care il detectam
    AprilTagDetection tagDetectat = null;

    boolean tagGasit = false;
//    int ticks = (int) ((cm / WHELL_CIRCUMFERENCE_CM) * COUNTS_PER_MOTOR_REV);

    @Override
    public void runOpMode() {
        initATD();
        initHardware();

        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
            while (!tagGasit && runtime.seconds() < 3) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if (currentDetections.size() != 0) {

                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == 0 || tag.id == 1 || tag.id == 2) {
                            tagDetectat = tag;
                            tagGasit = true;
                            break;
                        }
                    }

                    if (tagGasit) {
                        telemetry.addLine("Zona de semnal a fost gasita!\n\nTag data:");
                        tagToTelemetry(tagDetectat);
                    } else {
                        telemetry.addLine("Nu gasesc niciun tag");
                    }
                    telemetry.update();

                }
            }

            MersSpreStack();
            PuneConuri();

            if (tagDetectat != null) {
                switch (tagDetectat.id) {
                    case 0:
                        MersSpreZona1();
                        break;
                    case 1:
                        MersSpreZona2();
                        break;
                    case 2:
                        MersSpreZona3();
                        break;
                }
            } else {
                MersSpreZona2();
            }
        }
    }

    private void MersSpreZona1() {
        if(opModeIsActive()){
            RotatieDistanta("Dreapta", 155);
            sleep(500);
            MiscareRotiDistanta("Spate", 52);
            sleep(500);
            MiscareRotiDistanta("Dreapta", 25);
            sleep(500);
        }
    }

    private void MersSpreZona2() {
        if(opModeIsActive()){
            RotatieDistanta("Dreapta", 155);
            sleep(500);
        }
    }

    private void MersSpreZona3() {
        if(opModeIsActive()){
            RotatieDistanta("Dreapta", 155);
            sleep(500);
            MiscareRotiDistanta("Fata", 60);
            sleep(500);
            MiscareRotiDistanta("Dreapta", 25);
            sleep(500);
        }
    }

    private void PuneConuri() {
        if(opModeIsActive()) {
            MiscareRotiDistanta("Spate",60);
            sleep(200);
            RotatieDistanta("Stanga", 180);
            MiscareGlisiere("Sus", 0.4);
            sleep(200);
            MiscareRotiDistanta("Fata", 20);
            sleep(500);
            Grab("Deschis");
            sleep(500);
            MiscareRotiDistanta("Spate", 5);
            sleep(500);
            MiscareGlisiere("Jos", 1);
        }
    }

    private void MersSpreStack() {
        if(opModeIsActive()){
            /*MiscareGlisiere("Sus", 0.5);
            MiscareRotiDistanta("Fata", 70);
            sleep(100);
            RotatieDistanta("Stanga", 20);
            sleep(100);
            MiscareGlisiere("Sus", 0.4);
            sleep(100);
            MiscareRotiDistanta("Fata", 20);
            sleep(100);
            Grab("Deschis");
            MiscareRotiDistanta("Spate", 20);
            sleep(100);
            MiscareGlisiere("Jos", 0.4);
            sleep(100);
            RotatieDistanta("Dreapta", 30);
            sleep(100);
            MiscareRotiDistanta("Fata", 75);
            sleep(100);
            RotatieDistanta("Dreapta", 130);
            MiscareGlisiere("Jos", 0.2);
            sleep(100);
            MiscareRotiDistanta("Fata", 55);
            sleep(100);
            Grab("Inchis");
            sleep(100);
            MiscareGlisiere("Sus", 0.5);
            sleep(100);*/

            MiscareGlisiere("Sus", 0.5);
            MiscareRotiDistanta("Fata", 145);
            sleep(100);
            RotatieDistanta("Stanga", 5);
            sleep(100);
            MiscareGlisiere("Sus", 0.9);
            sleep(100);
            MiscareRotiDistanta("Fata", 15);
            sleep(300);
            Grab("Deschis");
            sleep(200);
            MiscareRotiDistanta("Spate", 10);
            sleep(200);
            RotatieDistanta("Dreapta", 145);
            MiscareGlisiere("Jos", 0.9);
            sleep(100);
            MiscareRotiDistanta("Fata", 75);
            sleep(200);
            Grab("Inchis");
            sleep(600);
            MiscareGlisiere("Sus", 0.8);
            sleep(500);

        }
    }

    private void MiscareRotiTimp(String directie, double secunde) {
        switch (directie) {
            case "Dreapta":
                runtime.reset();
                while (runtime.seconds() < secunde && opModeIsActive()) {
                    setPowerRoti(powerMiscareRoti, -powerMiscareRoti, -powerMiscareRoti, powerMiscareRoti);
                }
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;

            case "Stanga":
                runtime.reset();
                while (runtime.seconds() < secunde && opModeIsActive()) {

                    setPowerRoti(-powerMiscareRoti, powerMiscareRoti, powerMiscareRoti, -powerMiscareRoti);
                }
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;

            case "Fata":
                runtime.reset();
                while (runtime.seconds() < secunde && opModeIsActive()) {

                    setPowerRoti(-powerMiscareRoti, -powerMiscareRoti, -powerMiscareRoti, -powerMiscareRoti);
                }
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;

            case "Spate":
                runtime.reset();
                while (runtime.seconds() < secunde && opModeIsActive()) {

                    setPowerRoti(powerMiscareRoti, powerMiscareRoti, powerMiscareRoti, powerMiscareRoti);
                }
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;

            default:
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;
        }
    }

    private void RotatieDistanta(String directie, double deg) {
        // Encoder Ticks = (Distance in centimeters / Wheel Circumference in centimeters) * Encoder Counts per Revolution
        double cm = CM_PER_DEG * deg;
        int ticks = (int) (cm * INDICE);

        switch(directie) {
            case "Stanga":
                motorStangaFata.setTargetPosition(ticks);

                motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                setPowerRoti(-powerMiscareRoti, powerMiscareRoti, -powerMiscareRoti, powerMiscareRoti);

                while (motorStangaFata.isBusy()) {
                    telemetry.addData("Pozitie Stanga Fata:", motorStangaFata.getCurrentPosition());
                    telemetry.update();
                }
                setPowerRoti(0, 0, 0, 0);
                break;
            case "Dreapta":
                motorStangaFata.setTargetPosition(-ticks);

                motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                setPowerRoti(powerMiscareRoti, -powerMiscareRoti, powerMiscareRoti, -powerMiscareRoti);

                while (motorStangaFata.isBusy()) {
                    telemetry.addData("Pozitie Stanga Fata:", motorStangaFata.getCurrentPosition());
                    telemetry.update();
                }
                setPowerRoti(0, 0, 0, 0);
                break;
        }
        motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void MiscareRotiDistanta(String directie , double cm) {
        // Encoder Ticks = (Distance in centimeters / Wheel Circumference in centimeters) * Encoder Counts per Revolution
        int ticks = (int) (cm * INDICE);
        runtime.reset();

        switch(directie) {
            case "Fata":
                motorStangaFata.setTargetPosition(-ticks);

                motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                setPowerRoti(-powerMiscareRoti, -powerMiscareRoti, -powerMiscareRoti, -powerMiscareRoti);

                while (motorStangaFata.isBusy()) {
                    telemetry.addData("Pozitie Stanga Fata:", motorStangaFata.getCurrentPosition());
                    telemetry.update();
                }
                setPowerRoti(0, 0, 0, 0);
                break;
            case "Spate":
                motorStangaFata.setTargetPosition(ticks);

                motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                setPowerRoti(powerMiscareRoti, powerMiscareRoti, powerMiscareRoti, powerMiscareRoti);

                while (motorStangaFata.isBusy()) {
                    telemetry.addData("Pozitie Stanga Fata:", motorStangaFata.getCurrentPosition());
                    telemetry.update();
                }
                setPowerRoti(0, 0, 0, 0);
                break;
            case "Dreapta":
                motorStangaFata.setTargetPosition(-ticks);

                motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                setPowerRoti(powerMiscareRoti, -powerMiscareRoti, -powerMiscareRoti, powerMiscareRoti);

                while (motorStangaFata.isBusy()) {
                    telemetry.addData("Pozitie Stanga Fata:", motorStangaFata.getCurrentPosition());
                    telemetry.update();
                }
                setPowerRoti(0, 0, 0, 0);
                break;
            case "Stanga":
                motorStangaFata.setTargetPosition(ticks);

                motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                setPowerRoti(-powerMiscareRoti, powerMiscareRoti, powerMiscareRoti, -powerMiscareRoti);

                while (motorStangaFata.isBusy()) {
                    telemetry.addData("Pozitie Stanga Fata:", motorStangaFata.getCurrentPosition());
                    telemetry.update();
                }
                setPowerRoti(0, 0, 0, 0);
                break;
        }
        motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Metoda care face robotul sa se roteasca intr-o directie data pentru un anumit interval de timp exprimat in secunde
    private void RotatieTimp(String directie, double secunde){
        switch (directie){
            case "Stanga":
                runtime.reset();
                while(runtime.seconds() < secunde && opModeIsActive()){
                    setPowerRoti(-powerMiscareRoti, powerMiscareRoti, -powerMiscareRoti, powerMiscareRoti);
                }
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;

            case "Dreapta":
                runtime.reset();
                while(runtime.seconds() < secunde && opModeIsActive()){
                    setPowerRoti(powerMiscareRoti, -powerMiscareRoti, powerMiscareRoti, -powerMiscareRoti);
                }
                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
                break;
        }
    }

    private void MiscareGlisiere(String directie, double secunde){
        switch (directie){
            case "Sus":
                runtime.reset();
                while(runtime.seconds() < secunde && opModeIsActive()){
                    setPowerGlisiere(powerActiune);
                }
                setPowerGlisiere(0.1);
                break;

            case "Jos":
                runtime.reset();
                while(runtime.seconds() < secunde && opModeIsActive()) {
                    setPowerGlisiere(-powerActiune);
                }
                setPowerGlisiere(0.1);
                break;
        }
    }

    private void initATD(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
    }

    private void initHardware(){

        //Metoda hardwareMap preia din expansion hub informatii despre motor/servo si nu mai este null, daca primiti eroarea NullPointerException s-ar putea sa fie din cauza ca nu ati initializat aici motorul si acesta a ramas null
        motorDreaptaFata = hardwareMap.get(DcMotorEx.class, "motorDreaptaFata");
        motorDreaptaSpate = hardwareMap.get(DcMotorEx.class, "motorDreaptaSpate");
        motorStangaFata = hardwareMap.get(DcMotorEx.class, "motorStangaFata");
        motorStangaSpate = hardwareMap.get(DcMotorEx.class, "motorStangaSpate");

        motorBrat1 = hardwareMap.dcMotor.get("motorBrat1");
        motorBrat2 = hardwareMap.dcMotor.get("motorBrat2");

        servoGrab1 = hardwareMap.servo.get("servoGrab1");
        servoGrab2 = hardwareMap.servo.get("servoGrab2");

        //Motoatele pot rula cu encoder sau fara encoder, encoderul este un cablu care care masoara diferite chestii despre motor, daca nu folositi encoder trebuie sa setati modul asta
        motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorDreaptaFata.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaFata.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorDreaptaSpate.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaSpate.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motorBrat1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBrat2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Aici se seteaza diretia motorului, reverse sau forward, e simplu de inteles
        motorDreaptaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaSpate.setDirection(DcMotorSimple.Direction.FORWARD);

        motorBrat1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBrat2.setDirection(DcMotorSimple.Direction.REVERSE);

        servoGrab1.setDirection(Servo.Direction.FORWARD);
        servoGrab2.setDirection(Servo.Direction.REVERSE);

        setPowerGlisiere(0.1);
        setPowerRoti(0, 0, 0, 0);
        Grab("Inchis");
}

    private void Grab(String inchis){
        switch(inchis){
            case "Deschis":
                servoGrab1.setPosition(0.03);
                servoGrab2.setPosition(0.03);
                break;
            case "Inchis":
                servoGrab1.setPosition(positionGrab);
                servoGrab2.setPosition(positionGrab);
        }
    }

    // Seteaza puterea rotilor
    private void setPowerRoti(double powerDreaptaFata, double powerStangaFata, double powerDreaptaSpate, double powerStangaSpate) {
        motorDreaptaFata.setPower(powerDreaptaFata);
        motorStangaFata.setPower(powerStangaFata);
        motorDreaptaSpate.setPower(powerDreaptaSpate);
        motorStangaSpate.setPower(powerStangaSpate);
    }

    private void setPowerGlisiere(double powerActiune){
        motorBrat1.setPower(powerActiune);
        motorBrat2.setPower(powerActiune);
    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
