package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

//import com.ctre.phoenix6.controls.Follower;
//import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ToolSubsystem {

    private final int LEFTSHOOTERID = 15;
    private final int RIGHTSHOOTERID = 14;
    private final int LEFTFEEDERID = 4;
    private final int RIGHTFEEDERID = 8;
    private final int BACKINTAKEID = 7;
    private final int FRONTINTAKEID = 9;
    public final XboxController MYCONTROLLER = new XboxController(0);
    public CANSparkMax frontIntake;
    public CANSparkMax backIntake;
    public CANSparkMax rightFeeder;
    public CANSparkMax leftFeeder;
    public TalonFX rightShooter;
    public TalonFX leftShooter;
    public static PneumaticHub m_pH = new PneumaticHub(21);
    DoubleSolenoid ampFlap = m_pH.makeDoubleSolenoid(7,6);
    public boolean shooter;
    
    public ToolSubsystem(){
    leftShooter = new TalonFX(LEFTSHOOTERID);
    rightShooter = new TalonFX(RIGHTSHOOTERID);
    rightShooter.setInverted(true);

    leftFeeder = new CANSparkMax(LEFTFEEDERID, MotorType.kBrushless);
    rightFeeder = new CANSparkMax(RIGHTFEEDERID, MotorType.kBrushless);
    backIntake = new CANSparkMax(BACKINTAKEID, MotorType.kBrushless);
    frontIntake = new CANSparkMax(FRONTINTAKEID, MotorType.kBrushless);
    //leftFeeder.follow(rightFeeder);
    //leftFeeder.setInverted(true);
    shooter = false;

    }
    
    //shooter
    public void shooterToggleStart(){
        leftShooter.set(TalonFXControlMode.PercentOutput, -1);
        rightShooter.set(TalonFXControlMode.PercentOutput, -1);
    }
    public void shooterToggleStop(){
        leftShooter.set(TalonFXControlMode.PercentOutput, 0);
        rightShooter.set(TalonFXControlMode.PercentOutput, 0);
    }
    public void shooterStop(){
        leftShooter.set(TalonFXControlMode.PercentOutput, 0);
        rightShooter.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void ampPoop(){
        leftShooter.set(TalonFXControlMode.PercentOutput, -0.2);
        rightShooter.set(TalonFXControlMode.PercentOutput, -0.2);
        leftFeeder.set(-.2);
        rightFeeder.set(.2);
    }

    //intake
    public void intakeStart(){
        leftFeeder.set(-.4);
        rightFeeder.set(.4);
        frontIntake.set(.6);
        backIntake.set(-.5);
    }
    public void intakeBackDown(){
        leftFeeder.set(0.2);
        leftFeeder.set(0.2);
    }
    public void intakeStop(){
        leftFeeder.set(0);
        rightFeeder.set(0);
        frontIntake.set(0);
        backIntake.set(0);
    }

    //ampFlap
    public void ampFlapOut(){
        ampFlap.set(DoubleSolenoid.Value.kForward);
    }
    public void ampFlapIn(){
        ampFlap.set(DoubleSolenoid.Value.kReverse);
    }
}
