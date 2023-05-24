package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import androidx.annotation.Nullable;

@TeleOp(name = "TeleOp" , group = "MechanumTeleOp")
public class TeleOPMode extends OpMode {
    // VARIABILE

    // Declararea variabilelor, se declara cu null in cazul obiectelor (Servo, DcMotor, etc.)

    // Motoarele de la roti
    private DcMotor motorDreaptaFata = null;
    private DcMotor motorStangaFata = null;
    private DcMotor motorDreaptaSpate = null;
    private DcMotor motorStangaSpate = null;

    // Motoarele de la brat (cele care actioneaza glisierele din lateral)
    private DcMotor motorBrat1 = null;
    private DcMotor motorBrat2 = null;

    // Servo-urile care actioneaza clestele
    private Servo servoGrab1 = null;
    private Servo servoGrab2 = null;

    // Acestea sunt valorile minime si maxime care pot fii atribuite motoarelor si servo-urilor
    private double MIN_POSITION = 0, MAX_POSITION = 1;

    // Valoarea puterii cu care se initiaza motoarele (0, ca sa nu se miste)
    private final static int powerInit = 0;

    // Valorile puterilor cu care se poate deplasa robotul
    private final static double morePower = 0.75;
    private final static double lessPower = 0.6;

    // Variabila care contine valoarea puterii cu care se misca bratul
    private double powerBrat = powerInit;

    // Variabila care contine valoarea pozitiei la care se deschide (+positionGrab) si se inchide (-positionGrab) Grab-ul
    private double positionGrab = 0.065;

    // Valoarea puterii maxime cu care poate fii actionat bratul
    private double powerActiune = 1;

    // Variabila care contine valoarea puterii cu care se deplaseaza robotul
    private double powerMiscareFata = lessPower;

    // Variabilele care contin valoarea puterilor cu care se vor misca motoarele de la roti
    private double powerDreaptaFata = powerInit;
    private double powerStangaFata = powerInit;
    private double powerDreaptaSpate = powerInit;
    private double powerStangaSpate = powerInit;

    // Expresiile logice care verifica daca butonul X (gamepad 1) care schimba vitezele a fost apasat, respectiv daca puterea a fost
    // redusa (uita-te in metoda changePower)
    private boolean xRelease = true;
    private boolean putereRedusa = true;

    // Expresiile logice care verifica daca butonul Right Bumper (gamepad 2) care inchide sau deschide clestele a fost apasat, respectiv
    // daca clestele este inchis (uita-te in metoda Grabber)
    private boolean rbRelease = true;
    private boolean clesteInchis = false;

    // Expresiile logice care verifica daca butonul Left Bumper (gamepad 2) care roteste clestele la 0 si la 15 grade fata de pamant
    // a fost apasat, respectiv daca clestele a fost rotit (uita-te in metoda Grabber)
    private boolean lbRelease = true;
    private boolean bratRotit = false;

    @Override
    public void init() {
        // Se initializeaza in hardware map

        // Metoda hardwareMap preia din expansion hub informatii despre motor/servo si nu mai este null, daca primiti eroarea
        // NullPointerException s-ar putea sa fie din cauza ca nu ati initializat aici motorul si acesta a ramas null
        motorDreaptaFata = hardwareMap.dcMotor.get("motorDreaptaFata");
        motorDreaptaSpate = hardwareMap.dcMotor.get("motorDreaptaSpate");
        motorStangaFata = hardwareMap.dcMotor.get("motorStangaFata");
        motorStangaSpate = hardwareMap.dcMotor.get("motorStangaSpate");
        motorBrat1 = hardwareMap.dcMotor.get("motorBrat1");
        motorBrat2 = hardwareMap.dcMotor.get("motorBrat2");
        servoGrab1 = hardwareMap.servo.get("servoGrab1");
        servoGrab2 = hardwareMap.servo.get("servoGrab2");

        // Motoatele pot rula cu encoder sau fara encoder, encoderul este un cablu care care masoara diferite chestii despre motor,
        // daca nu folositi encoder trebuie sa setati modul asta
        motorDreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBrat1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBrat2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Aici se seteaza diretia motorului, reverse sau forward, e simplu de inteles
        motorDreaptaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaSpate.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBrat1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBrat2.setDirection(DcMotorSimple.Direction.FORWARD);
        servoGrab1.setDirection(Servo.Direction.FORWARD);
        servoGrab2.setDirection(Servo.Direction.REVERSE);

        SetPowerRoti(powerDreaptaFata, powerStangaFata, powerDreaptaSpate, powerStangaSpate);
    }

    // Metoda care merge pe tot parcursul OP Mode-ului, aici apelati toate metodele care fac robotul sa functioneze
    @Override
    public void loop() {
        ChangePower();
        MiscareGlisiere();
        Grabber();
        MiscareRoti();
    }

    // Metoda de setare a vitezei
    private void SetPowerRoti(double powerDreaptaFata, double powerStangaFata, double powerDreaptaSpate, double powerStangaSpate) {
        motorDreaptaFata.setPower(powerDreaptaFata);
        motorStangaFata.setPower(powerStangaFata);
        motorDreaptaSpate.setPower(powerDreaptaSpate);
        motorStangaSpate.setPower(powerStangaSpate);
    }

    // Metoda care modifica valoarea vitezei
    public void ChangePower() {
        if (gamepad1.square) {
            if (xRelease) {
                xRelease = false;
                if (!putereRedusa) {
                    powerMiscareFata = lessPower;
                    putereRedusa = true;
                } else {
                    powerMiscareFata = morePower;
                    putereRedusa = false;
                }
            }
        } else {
            xRelease = true;
        }
    }

    // Miscarea rotilor Strafer mechanum (miscarea robotului)
    private void MiscareRoti() {
        powerDreaptaFata = Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * 1.5), -powerMiscareFata, powerMiscareFata);
        powerStangaFata = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x * 1.5), -powerMiscareFata, powerMiscareFata);
        powerDreaptaSpate = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * 1.5), -powerMiscareFata, powerMiscareFata);
        powerStangaSpate = Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x * 1.5), -powerMiscareFata, powerMiscareFata);

        SetPowerRoti(powerDreaptaFata, powerStangaFata, powerDreaptaSpate, powerStangaSpate);
    }

    // Metoda care actioneaza clestele ; acesta se inchide/se deschide atunci cand butonul X de pe gamepad 2 este apasat
    private void Grabber() {
        if(gamepad2.right_bumper){
            if (rbRelease) {
                rbRelease = false;
                if (!clesteInchis) {
                    servoGrab1.setPosition(positionGrab);
                    servoGrab2.setPosition(positionGrab);
                    clesteInchis = true;
                } else {
                    servoGrab1.setPosition(0.03);
                    servoGrab2.setPosition(0.03);
                    clesteInchis = false;
                }
            }
        } else {
            rbRelease = true;
        }
    }

    // Metoda care ridica bratul
    private void MiscareGlisiere() {
        // Metoda range.clip seteaza puterea unui motor in functie de range-ul manetei de pe controller
        powerBrat = Range.clip(gamepad2.left_stick_y, -powerActiune, powerActiune) - 0.1;

        motorBrat1.setPower(powerBrat);
        motorBrat2.setPower(powerBrat);

    }

}