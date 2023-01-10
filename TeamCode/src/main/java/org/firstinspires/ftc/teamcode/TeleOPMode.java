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

    //declararea variabilelor, se declara cu null toate, tipurile lor sunt DcMotor sau Servo sau ce mai puneti pe robot
    private DcMotor motorDreaptaFata = null;
    private DcMotor motorStangaFata = null;
    private DcMotor motorDreaptaSpate = null;
    private DcMotor motorStangaSpate = null;

   // private DcMotor motorCupa = null;
  // private DcMotor motorOutake1 = null;
   //private DcMotor motorOutake2 = null;
    //private DcMotor motorIntake1 = null;
    //private DcMotor motorIntake2 = null;

    private DcMotor motorBrat = null;

    private Servo servoGrab = null;
    //private Servo servoGheara1 = null;
    //private Servo servoGheara2 = null;

    

    private double MIN_POSITION = 0 , MAX_POSITION = 1;

    private final static int powerInit = 0;

    private double morePower = 0.50;
    private double lessPower = 0.35;

    /*private double powerIntake = 0.6;
    private double powerCaruse = 0.6;
    private double powerOutake = 0.9;*/

    private double powerBrat = powerInit;

    private double powerActiune = 0.9;

    private double powerServoBrat = powerInit;

    private double powerServoActiune = 0.6;



    private double powerMiscareFata = morePower;

    private double powerDreaptaFata = powerInit;
    private double powerStangaFata = powerInit;
    private double powerDreaptaSpate = powerInit;
    private double powerStangaSpate = powerInit;

    private boolean downRelease1 = true;
    //private boolean isMaturicaOn = false;

    private boolean buttonRelease = true;
    private boolean downRelease = true;
    private boolean upRelease = true;
    private boolean isIntakeOn = false;
    private boolean isOutakeOn = false;
    private boolean reducePower = false;
    private boolean xRelease = true;


    @Override
    public void init() {
        // Se initializeaza in hardware map
        //Metoda hardwareMap preia din expansion hub informatii despre motor si nu mai este null, daca primiti eroarea NullPointerException s-ar putea sa fie din cauza ca nu ati initializat aici motorul si acesta a ramas null
      motorDreaptaFata =  hardwareMap.dcMotor.get("motorDreaptaFata");
      motorDreaptaSpate = hardwareMap.dcMotor.get("motorDreaptaSpate");
       motorStangaFata = hardwareMap.dcMotor.get("motorStangaFata");
       motorStangaSpate = hardwareMap.dcMotor.get("motorStangaSpate");

       motorBrat = hardwareMap.dcMotor.get("motorBrat");
        /*motorOutake1= hardwareMap.dcMotor.get("motorOutake1");
        motorOutake2 = hardwareMap.dcMotor.get("motorOutake2");
       motorIntake1= hardwareMap.dcMotor.get("motorIntake1");
       motorIntake2=hardwareMap.dcMotor.get("motorIntake2");*/
       //servoGheara1 = hardwareMap.servo.get("servoGheara1");
       //servoGheara2 = hardwareMap.servo.get("servoGheara2");
       servoGrab = hardwareMap.servo.get("servoGrab");


       //Apelarea metodei initMotor declarata mai jos(obligatorie)
        initMotor(motorDreaptaFata, "motorDreaptaFata", true, powerInit, true, true);
        initMotor(motorStangaFata, "motorStangaFata", false, powerInit, true, true);
        initMotor(motorDreaptaSpate, "motorDreaptaSpate", true, powerInit, true, true);
        initMotor(motorStangaSpate, "motorStangaSpate", false, powerInit, true, true);


      //  initMotor(motorCupa, "motorCupa" , false , powerInit , true , true);
       /* initMotor(motorOutake1 , "motorOutake1" , false , powerInit , true , true);
        initMotor(motorOutake2 , "motorOutake2" , false , powerInit , true , true);
        initMotor(motorIntake1 , "motorIntake1" , false , powerInit , true , true);
        initMotor(motorIntake2 , "motorIntake2" , false , powerInit , true , true);*/





        initMotor(motorBrat , "motorBrat" , false, powerInit , true , true);


        //Motoatele pot rula cu encoder sau fara encoder, encoderul este un cablu care care masoara diferite chestii despre motor, daca nu folositi encoder trebuie sa setati modul asta
        motorDreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       /* motorOutake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorOutake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/


       // motorCupa.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorCupa.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //motorBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Aici se seteaza diretia motorului, reverse sau forward , e simplu de inteles
        motorDreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorStangaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);
        motorStangaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        /*motorIntake1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake2.setDirection(DcMotorSimple.Direction.REVERSE);
        motorOutake1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorOutake2.setDirection(DcMotorSimple.Direction.REVERSE);*/


        //motorCupa.setDirection(DcMotorSimple.Direction.FORWARD);
        //motorCupa.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBrat.setDirection(DcMotorSimple.Direction.FORWARD);
        setPower(powerDreaptaFata, powerStangaFata, powerDreaptaSpate, powerStangaSpate);
        //setPowerIntake(0);
    }


    //metoda care merge pe tot parcursul OP Mode-ului, aici apelati toate metodele care fac robotul sa ses miste
    @Override
    public void loop() {
        miscareMechanum();
        changePower();
        ridicareBrat();
        Grabber();
    }



    //Metoda care initializeaza motoarele, nu se apaleaza in loop
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


    //Metoda de setare a vitezei
    private void setPower(double powerDreaptaFata, double powerStangaFata, double powerDreaptaSpate, double powerStangaSpate) {
        motorDreaptaFata.setPower(powerDreaptaFata);
        motorStangaFata.setPower(powerStangaFata);
        motorDreaptaSpate.setPower(powerDreaptaSpate);
        motorStangaSpate.setPower(powerStangaSpate);

    }

    // Functia care modifica valoarea vitezei
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
        powerStangaSpate = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x +gamepad1.right_stick_x, -powerMiscareFata, powerMiscareFata);

        setPower(powerDreaptaFata, powerStangaFata, powerDreaptaSpate, powerStangaSpate);
    }

    // Metoda care se ocupa de sistemul de intake si de outtake ; apuca paharul atunci cand butonul X de pe controller (de PlayStation) este APASAT
    private void Grabber(){

        if(gamepad2.x) {
            if (downRelease == false) {
                downRelease = true;
                if(isIntakeOn) {
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
        powerBrat = Range.clip(gamepad2.left_stick_y, -powerActiune , powerActiune );
        motorBrat.setPower(powerBrat);
    }

    /*
    private void servoGheara() {
        if (gamepad2.dpad_left) {
                servoGheara1.setPosition(0.3);
//                servoGheara2.setPosition(0.3);
            } else {
                servoGheara1.setPosition(MIN_POSITION);
//                servoGheara2.setPosition(MIN_POSITION);
            }
        }
*/



        /*
            if(gamepad1.a) {
                if (downRelease == false) {
                    downRelease = true;
                    if(isIntakeOn) {
                        motorIntake1.setPower(powerInit);
                        motorIntake2.setPower(powerInit);
                        isIntakeOn = false;
                    } else {
                        motorIntake1.setPower(0.9);
                        motorIntake2.setPower(0.9);
                        isIntakeOn = true;
                    }
                } else {
                    downRelease = false;
                }
            }

            if(gamepad1.b) {
                if(downRelease1 == false) {
                    downRelease1 = true;
                    if (isOutakeOn) {
                        motorOutake1.setPower(powerInit);
                        motorOutake2.setPower(powerInit);
                        isOutakeOn = false;
                    }
                    else {
                        motorOutake1.setPower(0.9);
                        motorOutake2.setPower(0.9);
                        isOutakeOn = true;
                    }
                } else {
                    downRelease1 = false;
                }
            }
        }
*/


        //metoda care schimba puterea motoarelor, daca folositi un cod normal trebuie sa tineti apasat pe butonul respectiv, dar cu codul asat merge doar sa apasati o data pentru pornire si inca o data pentru oprire






    /*private void setPowerIntake(int sens) {
        if (sens == 1) {
            motorIntake1.setPower(powerIntake);
            motorIntake2.setPower(powerIntake);
        } else if (sens == -1) {
            motorIntake1.setPower((-powerIntake));
            motorIntake2.setPower((-powerIntake));

        } else if (sens == 0) {
            motorIntake1.setPower(powerInit);
            motorIntake2.setPower((-powerIntake));

        }
    }

    private void setPowerOutake(int sens) {
        if(sens==1){
            motorOutake1.setPower(powerOutake);
            motorOutake2.setPower(powerOutake);
        } else if(sens==-1){
            motorOutake1.setPower(-powerOutake);
            motorOutake2.setPower(-powerOutake);
        } else if (sens==0){
            motorOutake1.setPower(powerInit);
            motorOutake2.setPower(-powerOutake);
        }

    }*/


/*
    private void intake() {
        if (gamepad2.dpad_down) {
            if (downRelease == false) {
                downRelease = true;
                if (isIntakeOn) {
                    setPowerIntake(0);
                    isIntakeOn = false;
                } else {
                    setPowerIntake(1);
                    isIntakeOn = true;
                }
            }
        } else {
            downRelease = false;
        }
        if (gamepad2.dpad_up) {
            if (upRelease == false) {
                upRelease = true;
                if (isIntakeOn) {
                    setPowerIntake(0);
                    isIntakeOn = false;
                } else {
                    setPowerIntake(-1);
                    isIntakeOn = true;
                }
            }
        } else {
            upRelease = false;
        }
    }

 */


//    private void carusel() {
//        if(gamepad2.a) {
//            if (downRelease == false) {
//                downRelease = true;
//                if(isIntakeOn) {
//                    motorCarusel.setPower(powerInit);
//                    isIntakeOn = false;
//                } else {
//                    motorCarusel.setPower(0.3);
//                    isIntakeOn = true;
//                }
//            } else {
//                downRelease = false;
//            }
//        }
//    }
//    private void setPowerIntake(int sens) {
//        if (sens == 1) {
//            motorIntake.setPower(powerIntake);
//        } else if (sens == -1) {
//            motorIntake.setPower(-powerIntake);
//        } else if (sens == 0) {
//            motorIntake.setPower(powerInit);
//        }
//    }
//
//    private void maturica() {
//        if (gamepad2.dpad_down) {
//            if (downRelease1 == false) {
//                downRelease1 = true;
//                if (isIntakeOn) {
//                    setPowerIntake(0);
//                    isIntakeOn = false;
//                } else {
//                    setPowerIntake(1);
//                    isIntakeOn = true;
//                }
//            }
//        } else {
//            downRelease1 = false;
//        }
//        if (gamepad2.dpad_up) {
//            if (upRelease == false) {
//                upRelease = true;
//                if (isIntakeOn) {
//                    setPowerIntake(0);
//                    isIntakeOn = false;
//                } else {
//                    setPowerIntake(-1);
//                    isIntakeOn = true;
//                }
//            }
//        } else {
//            upRelease = false;
//        }
//    }

}



