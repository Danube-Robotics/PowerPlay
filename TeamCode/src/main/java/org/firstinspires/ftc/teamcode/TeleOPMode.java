package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import androidx.annotation.Nullable;

import static com.qualcomm.hardware.bosch.BNO055IMU.*;

@TeleOp(name = "TeleOp" , group = "MechanumTeleOp")
public class TeleOPMode extends OpMode {
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

    // Expresiile logice care verifica daca butonul X (gamepad 2) care actioneaza servo-ul a fost apasat, respectiv daca sistemul de Intake este pornit (uita-te in metoda Grabber)
    private boolean xRelease2 = true;
    private boolean isGrabbing = false;

    // Expresiile logice care verifica daca butonul X (gamepad 1) care schimba vitezele a fost apasat, respectiv daca puterea a fost sau nu redusa (uita-te in metoda changePower)
    private boolean xRelease = true;
    private boolean putereRedusa = true;

    // Expresiile logice care verifica daca butonul Y (gamepad 2) care ridica glisiera centrala a fost apasat, respectiv daca aceasta este ridicata sau nu (uita-te in metoda ridicareBrat)
    private boolean yRelease = true;
    private boolean esteRidicat = false;


    @Override
    public void init() {
        // Se initializeaza in hardware map

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

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
        motorDreaptaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaSpate.setDirection(DcMotorSimple.Direction.FORWARD);

        motorBrat1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBrat2.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBratCentral.setDirection(DcMotorSimple.Direction.FORWARD);

        //servoGrab1.setDirection(Servo.Direction.REVERSE);
        servoGrab2.setDirection(Servo.Direction.REVERSE);

        //servoGrab1.setPosition(0.3);
        //servoGrab2.setPosition(0.3);

        setPower(powerDreaptaFata, powerStangaFata, powerDreaptaSpate, powerStangaSpate);

    }


    // Metoda care merge pe tot parcursul OP Mode-ului, aici apelati toate metodele care fac robotul sa se miste
    @Override
    public void loop() {

        changePower();
        ridicareBrat();
        Grabber();
        miscareMechanum();

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

    //Miscarea rotilor nachanum (Miscarea robotului fata spate stanga dreapta)
    private void miscareMechanum() {
//
//        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = gamepad1.right_stick_x;
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio, but only when
//        // at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//
//        motorStangaFata.setPower(frontLeftPower);
//        motorStangaSpate.setPower(backLeftPower);
//        motorDreaptaFata.setPower(frontRightPower);
//        motorDreaptaSpate.setPower(backRightPower);



        powerDreaptaFata = Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -powerMiscareFata, powerMiscareFata);
        powerStangaFata = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -powerMiscareFata, powerMiscareFata);
        powerDreaptaSpate = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -powerMiscareFata, powerMiscareFata);
        powerStangaSpate = Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -powerMiscareFata, powerMiscareFata);

        setPower(powerDreaptaFata, powerStangaFata, powerDreaptaSpate, powerStangaSpate);
    }

    // Metoda care actioneaza clestele ; acesta se inchide/se deschide atunci cand butonul X de pe gamepad 2 este apasat
    private void Grabber() {
      /*  if(gamepad2.b){
            servoGrab.setPosition(0.3);
        }else {
            servoGrab.setPosition(Servo.MIN_POSITION);
        } */

        /*
        if (gamepad2.x) {
            if (xRelease2 == false) {
                xRelease2 = true;
                if (isGrabbing) {
                    servoGrab1.setPosition(MIN_POSITION);
                    servoGrab2.setPosition(MIN_POSITION);
                    isGrabbing = false;
                } else {
                    servoGrab1.setPosition(0.1);
                    servoGrab2.setPosition(0.1);
                    isGrabbing = true;
                }
            }
            else {
                xRelease2 = false;
            }
        }*/


        if(gamepad2.right_bumper){
            servoGrab1.setPosition(MIN_POSITION);
            servoGrab2.setPosition(MIN_POSITION);
        } else if(gamepad2.left_bumper) {
            servoGrab1.setPosition(0.04);
            servoGrab2.setPosition(0.04);
        }

//
//            } else {
//                xRelease2 = false;
//            }
//        }

        /*
        if (gamepad2.right_bumper){
            servoGrab1.setPosition(MIN_POSITION);
            servoGrab2.setPosition(MIN_POSITION);
        }
        else if (gamepad2.left_bumper){
            servoGrab1.setPosition(positionGrab);
            servoGrab2.setPosition(positionGrab);
        }*/


        /*
        if(gamepad2.x) {
            servoGrab.setPosition(0.1);
        } else {
            servoGrab.setPosition(MIN_POSITION);
        }
        */
    }

    /*private void ridicareGiliseraMijloc() {
        powerBrat2 = Range.clip(gamepad2.right_stick_y, -powerActiune, powerActiune);
        motorBratCentral.setPower(powerBrat2);
    }*/

    // Metoda care ridica bratul
    private void ridicareBrat() {
        // metoda range.clip seteaza puterea unui motor in functie de rangge-ul manetei de pe controller
        powerBrat1 = Range.clip(gamepad2.left_stick_y, -powerActiune, powerActiune);
        powerBrat2 = Range.clip(gamepad2.right_stick_y, -powerActiune, powerActiune);
        motorBrat1.setPower(powerBrat1);
        motorBrat2.setPower(powerBrat1);
        motorBratCentral.setPower(powerBrat2);

        // ridica glisiera centrala la apasarea butonului y

       // if (gamepad2.y) {
           // if (yRelease == false) {
             //   yRelease = true;
              //  if (esteRidicat) {
               //    servoBrat.setPosition(MIN_POSITION);
                //    esteRidicat = false;
               // } else {
                 //   servoBrat.setPosition(0.5);
                  //  esteRidicat = true;
               // }
           // } else {
            //   yRelease = false;
           // }
       // }
    }

}