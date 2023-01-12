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

    // Declararea variabilelor, se declara cu null toate, tipurile lor sunt DcMotor sau Servo sau ce mai puneti pe robot

    // Motoarele de la roti
    private DcMotor motorDreaptaFata = null;
    private DcMotor motorStangaFata = null;
    private DcMotor motorDreaptaSpate = null;
    private DcMotor motorStangaSpate = null;

    // Motoarele de la brat (cele care actioneaza glisierele)
    private DcMotor motorBrat = null;
    private DcMotor motorBrat2 = null;

    // Servo-ul care actioneaza clestele
    private Servo servoGrab = null;

    // Acestea sunt valorile minime si maxime care pot fii atribuite motoarelor si servo-urilor
    private double MIN_POSITION = 0, MAX_POSITION = 1;

    // Valoarea puterii cu care se initiaza motoarele (0, ca sa nu se miste)
    private final static int powerInit = 0;

    // Valorile puterilor cu care se poate deplasa robotul
    private final static double morePower = 0.50;
    private final static double lessPower = 0.35;

    // Variabila care contine valoarea puterii cu care se misca bratul
    private double powerBrat = powerInit;

    // Valoarea puterii maxime cu care poate fii actionat bratul
    private double powerActiune = 0.9;

    // Variabila care contine valoarea puterii cu care se deplaseaza robotul
    private double powerMiscareFata = morePower;

    // Variabilele care contin valoarea puterilor cu care se vor misca motoarele de la roti
    private double powerDreaptaFata = powerInit;
    private double powerStangaFata = powerInit;
    private double powerDreaptaSpate = powerInit;
    private double powerStangaSpate = powerInit;

    // Expresiile logice care verifica daca butonul care actioneaza servo-ul a fost apasat, respectiv daca sistemul de Intake este pornit (uita-te in metoda Grabber)
    private boolean downRelease = true;
    private boolean isIntakeOn = false;

    // Expresiile logice care verifica daca butonul care schimba vitezele a fost apasat, respectiv daca puterea a fost sau nu redusa (uita-te in metoda changePower)
    private boolean xRelease = true;
    private boolean reducePower = false;


    @Override
    public void init() {
        // Se initializeaza in hardware map

        //Metoda hardwareMap preia din expansion hub informatii despre motor/servo si nu mai este null, daca primiti eroarea NullPointerException s-ar putea sa fie din cauza ca nu ati initializat aici motorul si acesta a ramas null
        motorDreaptaFata = hardwareMap.dcMotor.get("motorDreaptaFata");
        motorDreaptaSpate = hardwareMap.dcMotor.get("motorDreaptaSpate");
        motorStangaFata = hardwareMap.dcMotor.get("motorStangaFata");
        motorStangaSpate = hardwareMap.dcMotor.get("motorStangaSpate");

        motorBrat = hardwareMap.dcMotor.get("motorBrat");
        motorBrat2 = hardwareMap.dcMotor.get("motorBrat2");

        servoGrab = hardwareMap.servo.get("servoGrab");

        //Apelarea metodei initMotor declarata mai jos(obligatorie)
        initMotor(motorDreaptaFata, "motorDreaptaFata", true, powerInit, true, true);
        initMotor(motorStangaFata, "motorStangaFata", false, powerInit, true, true);
        initMotor(motorDreaptaSpate, "motorDreaptaSpate", true, powerInit, true, true);
        initMotor(motorStangaSpate, "motorStangaSpate", false, powerInit, true, true);

        initMotor(motorBrat, "motorBrat", false, powerInit, true, true);
        initMotor(motorBrat2, "motorBrat2", false, powerInit, true, true);


        //Motoatele pot rula cu encoder sau fara encoder, encoderul este un cablu care care masoara diferite chestii despre motor, daca nu folositi encoder trebuie sa setati modul asta
        motorDreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBrat2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Aici se seteaza diretia motorului, reverse sau forward, e simplu de inteles
        motorDreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorStangaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);
        motorStangaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBrat.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBrat2.setDirection(DcMotorSimple.Direction.FORWARD);

        setPower(powerDreaptaFata, powerStangaFata, powerDreaptaSpate, powerStangaSpate);
    }


    // Metoda care merge pe tot parcursul OP Mode-ului, aici apelati toate metodele care fac robotul sa se miste
    @Override
    public void loop() {
        miscareMechanum();
        changePower();
        ridicareBrat();
        Grabber();
    }


    // Metoda care initializeaza motoarele, nu se apaleaza in loop
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


    // Metoda de setare a vitezei
    private void setPower(double powerDreaptaFata, double powerStangaFata, double powerDreaptaSpate, double powerStangaSpate) {
        motorDreaptaFata.setPower(powerDreaptaFata);
        motorStangaFata.setPower(powerStangaFata);
        motorDreaptaSpate.setPower(powerDreaptaSpate);
        motorStangaSpate.setPower(powerStangaSpate);

    }

    // Metoda care modifica valoarea vitezei
    public void changePower() {
        if (gamepad1.x) {
            if (xRelease) {
                xRelease = false;
                if (!reducePower) {
                    powerMiscareFata = lessPower;
                    reducePower = true;
                } else {
                    powerMiscareFata = morePower;
                    reducePower = false;
                }
            }
        } else {
            xRelease = true;
        }
    }

    //Miscarea rotilor nachanum (Miscarea robotului fata spate stanga dreapta)
    private void miscareMechanum() {
        powerDreaptaFata = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -powerMiscareFata, powerMiscareFata);
        powerStangaFata = Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -powerMiscareFata, powerMiscareFata);
        powerDreaptaSpate = Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -powerMiscareFata, powerMiscareFata);
        powerStangaSpate = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -powerMiscareFata, powerMiscareFata);

        setPower(powerDreaptaFata, powerStangaFata, powerDreaptaSpate, powerStangaSpate);
    }

    // Metoda care se ocupa de sistemul de intake si de outtake ; apuca paharul atunci cand butonul X de pe controller (de PlayStation) este APASAT
    private void Grabber() {

        if (gamepad2.x) {
            if (downRelease == false) {
                downRelease = true;
                if (isIntakeOn) {
                    servoGrab.setPosition(MIN_POSITION);
                    isIntakeOn = false;
                } else {
                    servoGrab.setPosition(0.05);
                    isIntakeOn = true;
                }
            } else {
                downRelease = false;
            }
        }

        /*
        if(gamepad2.x) {
            servoGrab.setPosition(0.1);
        } else {
            servoGrab.setPosition(MIN_POSITION);
        }
        */
    }

    // Metoda care ridica bratul
    private void ridicareBrat() {
        //metoda range.clip seteaza puterea unui motor in functie de rangge-ul manetei de pe controller
        powerBrat = Range.clip(gamepad2.left_stick_y, -powerActiune, powerActiune);
        motorBrat.setPower(powerBrat);
        motorBrat2.setPower(powerBrat);
    }

}