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
//@Disabled
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

    // Valorile puterilor cu care se poate deplasa robotul
    private final static double morePower = 0.6;
    private final static double lessPower = 0.4;

    // Variabila care contine valoarea puterii cu care se misca bratul
    private double powerBrat1 = powerInit;
    private double powerBrat2 = powerInit;


    private double positionGrab = 0.06;

    // Valoarea puterii maxime cu care poate fii actionat bratul
    private double powerActiune = 1;

    // Variabila care contine valoarea puterii cu care se deplaseaza robotul
    private double powerMiscareFata = lessPower;

    // Variabilele care contin valoarea puterilor cu care se vor misca motoarele de la roti
    private double powerDreaptaFata = powerInit;
    private double powerStangaFata = powerInit;
    private double powerDreaptaSpate = powerInit;
    private double powerStangaSpate = powerInit;

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

    // Expresia logica care
    boolean tagGasit = false;

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
                    case 1:
                        MersSpreZona2();
                    case 2:
                        MersSpreZona3();
                }
            } else {
                MersSpreZona2();
            }
        }
    }

    private void MersSpreZona1() {
        if(opModeIsActive()){
            // TODO cod pentru mers spre zona 1
        }
    }

    private void MersSpreZona2() {
        if(opModeIsActive()){
            // TODO cod pentru mers spre zona 2
        }
    }

    private void MersSpreZona3() {
        if(opModeIsActive()){
            // TODO cod pentru mers spre zona 3
        }
    }

    private void PuneConuri() {
        if(opModeIsActive()) {
            for (int i = 0; i < 5; i += 1) {
                // TODO cod sa puna conurile din stack pe junctiunea mare
            }
        }
    }

    private void MersSpreStack() {

    }

    private void MiscareRoti(String directie, double secunde) {
        switch (directie) {
            case "Dreapta":



//            case "Dreapta":
//                runtime.reset();
//                while (runtime.seconds() < secunde && opModeIsActive()) {
//                    setPowerRoti(powerMiscareFata, -powerMiscareFata, -powerMiscareFata, powerMiscareFata);
//                }
//                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
//                break;
//
//            case "Stanga":
//                runtime.reset();
//                while (runtime.seconds() < secunde && opModeIsActive()) {
//
//                    setPowerRoti(-powerMiscareFata, powerMiscareFata, powerMiscareFata, -powerMiscareFata);
//                }
//                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
//                break;
//
//            case "Fata":
//                runtime.reset();
//                while (runtime.seconds() < secunde && opModeIsActive()) {
//
//                    setPowerRoti(-powerMiscareFata, -powerMiscareFata, -powerMiscareFata, -powerMiscareFata);
//                }
//                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
//                break;
//
//            case "Spate":
//                runtime.reset();
//                while (runtime.seconds() < secunde && opModeIsActive()) {
//
//                    setPowerRoti(powerMiscareFata, powerMiscareFata, powerMiscareFata, powerMiscareFata);
//                }
//                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
//                break;
//
//            default:
//                setPowerRoti(powerInit, powerInit, powerInit, powerInit);
//                break;
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

    private void MiscareGlisiere(String directie, double secunde){
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
        motorDreaptaFata.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorStangaFata.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorDreaptaSpate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorStangaSpate.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorDreaptaFata.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorStangaFata.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorDreaptaSpate.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorStangaSpate.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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

        setPowerGlisiere(0.2);
        setPowerRoti(powerDreaptaFata, powerStangaFata, powerDreaptaSpate, powerStangaSpate);
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
