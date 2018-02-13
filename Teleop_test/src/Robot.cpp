#include <iostream>
#include <string>
#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <XboxController.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Spark.h>
#include <Victor.h>
#include <AnalogInput.h>
#include <Timer.h>
#include <CameraServer.h>
#include <PowerDistributionPanel.h>
#include <Talon.h>
#include <BuiltInAccelerometer.h>
#include <DriverStation.h>

#define XBOX true

class Robot: public frc::SampleRobot {
public:
	Robot() {
		m_robotDrive.SetExpiration(0.1);
		Etime = new frc::Timer();
		PDP = new frc::PowerDistributionPanel(0);
		accel = new frc::BuiltInAccelerometer();

		m_open =  false;
		m_close = false;
		m_out = false;
		m_intake = false;
		m_Vertical = false;
		m_UpFront = false;
		m_DownFront = false;
		m_Behind = false;
		m_Wench = false;
		m_ElevatorUp = false;
		m_ElevatorDown = false;

		current = 0;
		gain = -0.00055;
		target = 830;
		drive = 0;
		error = 0;

	}

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		camera.SetResolution(640, 480);
	}

	void Autonomous() {
		std::string autoSelected = m_chooser.GetSelected();
		std::cout << "Auto selected: " << autoSelected << std::endl;
		m_robotDrive.SetSafetyEnabled(false);

		if (autoSelected == kAutoNameCustom) {
			std::cout << "Running custom Autonomous" << std::endl;
			m_robotDrive.ArcadeDrive(0.0, 0.5);
			frc::Wait(2.0);
			m_robotDrive.ArcadeDrive(0.0, 0.0);
		 } else {
			 std::cout << "Running default Autonomous" << std::endl;
			 m_robotDrive.ArcadeDrive(-0.5, 0.0);
			 frc::Wait(2.0);
			 m_robotDrive.ArcadeDrive(0.0, 0.0);
		 }
	}

	bool IntakeButton() {
		if (XBOX != true) {
			return m_stick.GetRawButton(4);
		} else {
			//left bumper
			return m_xbox.GetRawButton(5);
		}
	}

	bool OutButton() {
		if (XBOX != true) {
			return m_stick.GetRawButton(5);
		} else {
			//right bumper
			return m_xbox.GetRawButton(6);
		}
	}

	bool DownFront() {
		if (XBOX != true) {
			return m_stick.GetRawButton(6);
		} else {
			//button A
			return m_xbox.GetRawButton(1);
		}
	}

	bool UpFront() {
		if (XBOX != true) {
			return m_stick.GetRawButton(7);
		} else {
			return m_xbox.GetRawButton(3);
		}
	}

	bool Vertical() {
		if (XBOX != true) {
			return m_stick.GetRawButton(11);
		} else {
			return m_xbox.GetRawButton(4);
		}
	}

	bool Behind() {
		if (XBOX != true) {
			return m_stick.GetRawButton(10);
		} else {
			return m_xbox.GetRawButton(2);
		}
	}

	bool OpenJaw() {
		if (XBOX != true) {
			return m_stick.GetRawButton(9);
		} else {
			return m_xbox.GetRawButton(10);
		}
	}

	bool CloseJaw() {
		if (XBOX != true) {
			return m_stick.GetRawButton(8);
		} else {
			return m_xbox.GetRawButton(9);
		}
	}

	bool ElevatorUp(){
		if (XBOX != true) {
			return m_stick.GetRawButton(1);
		} else {
			return m_xbox.GetRawButton(6);
		}
	}

	bool ElevatorDown(){
		if (XBOX != true) {
			return m_stick.GetRawButton(3);
		} else {
			return m_xbox.GetRawButton(7);
		}
	}

	void DriveTrain() {
		if (XBOX != true) {
			m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetX());
		} else {
			m_robotDrive.ArcadeDrive(-m_xbox.GetRawAxis(1),
					m_xbox.GetRawAxis(4));
		}

	}

	void CurrentCheck(){
		if(m_leftlift.Get() != 0){
			if(PDP->GetCurrent(11) == 0){
				station.ReportError("Left Arm Motor Disconnected, Justin!");
			}
		}

		if(m_rightlift.Get() != 0){
			if(PDP->GetCurrent(12) == 0){
				station.ReportError("Right Arm Motor Disconnected, Justin!");
			}
		}

		if(m_leftintake.Get() != 0){
			if(PDP->GetCurrent(10) == 0){
				station.ReportError("Left Intake Motor Disconnected, Allison!");
			}
		}

		if(m_rightintake.Get() != 0){
			if(PDP->GetCurrent(13) == 0){
				station.ReportError("Right Intake Motor Disconnected, Allison!");
			}
		}

		if(m_Jaw.Get() != 0){
			if(PDP->GetCurrent(14) == 0){
				station.ReportError("Jaw Motor Disconnected, Justin & Allison!");
			}
		}
	}

	void OperatorControl() override {
		m_robotDrive.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {

			/*-----Dashboard Data Setting-----*/
			SmartDashboard::PutNumber("ArmPosition", ArmPosition.GetValue());
			SmartDashboard::PutNumber("Accelerometer", accel->GetY());
			SmartDashboard::PutNumber("JawCurrent", PDP->GetCurrent(9));
			SmartDashboard::PutNumber("ArmCurrent_Left", PDP->GetCurrent(4));
			SmartDashboard::PutNumber("ArmCurrent_Right", PDP->GetCurrent(5));
			SmartDashboard::PutNumber("Intake_Left", PDP->GetCurrent(2));
			SmartDashboard::PutNumber("Intake_Right", PDP->GetCurrent(3));

			/*-----DriveTrain Operation-----*/
			DriveTrain();
			frc::Wait(0.005);

			/*-----Current Checking-----*/
			CurrentCheck();

			/*-----JoystickButton Initialization-----*/
			m_intake = IntakeButton();
			m_out = OutButton();
			m_DownFront = DownFront();
			m_UpFront = UpFront();
			m_Vertical = Vertical();
			m_Behind = Behind();
			m_open = OpenJaw();
			m_close = CloseJaw();
			m_ElevatorUp = ElevatorUp();
			m_ElevatorDown = ElevatorDown();

			/*-----Elevator Operation-----*/
			if(m_ElevatorUp == 1){
				m_Elevator.Set(1.0);
			} else if (m_ElevatorDown == 1){
				m_Elevator.Set(-1.0);
			} else {
				m_Elevator.Set(0.0);
			}

			/*-----Jaw Operation-----*/
			if (m_close == true) {
				Etime->Reset();
				Etime->Start();
				while (Etime->Get() < 0.5) {
					m_Jaw.Set(0.75);
				}
				Etime->Stop();
				m_Jaw.Set(0.0);
			} else if (m_open == true) {
				Etime->Reset();
				Etime->Start();
				while (Etime->Get() < 0.5) {
					m_Jaw.Set(-0.75);
				}
				Etime->Stop();
				m_Jaw.Set(0.0);
			} else {
				m_Jaw.Set(0.0);
			}

			/*-----Intake Operation-----*/
			if (m_intake == 1) {
				m_leftintake.Set(1.0);
				m_rightintake.Set(1.0);
			} else if (m_out == 1) {
				m_leftintake.Set(-1.0);
				m_rightintake.Set(-1.0);
			} else {
				m_leftintake.Set(0.0);
				m_rightintake.Set(0.0);
			}

			/*-----Arm Operation-----*/
			current = ArmPosition.GetValue();
			if (m_DownFront == 1) {
				target = 2350;
			} else if (m_UpFront == 1) {
				target = 1820;
			} else if (m_Vertical == 1) {
				target = 830;
			} else if (m_Behind == 1) {
				target = 250;
			}

			//Caculate Arm Speed
			error = current - target;
			drive = gain * error;
			m_leftlift.Set(drive);
			m_rightlift.Set(drive);
		}
	}

	void Test() override {
	}

private:
	//Driving Motors
	frc::Spark m_leftMotor { 0 };
	frc::Spark m_rightMotor { 1 };
	frc::DifferentialDrive m_robotDrive { m_leftMotor, m_rightMotor };

	//Operation Motors
	frc::Spark m_leftintake { 2 };
	frc::Spark m_rightintake { 3 };
	frc::Spark m_leftlift { 4 };
	frc::Spark m_rightlift { 5 };
	frc::Spark m_leftWencch { 6 };
	frc::Spark m_rightWench { 7 };
	frc::Spark m_Elevator { 8 };
	frc::Talon m_Jaw { 9 };

	//Joystick Settings
	frc::Joystick m_stick { 1 };
	frc::XboxController m_xbox { 0 };
	bool m_intake, m_out, m_open, m_close;
	bool m_DownFront, m_UpFront, m_Vertical, m_Behind;
	bool m_Wench, m_ElevatorUp, m_ElevatorDown;

	//Values
	int current;
	int target;
	int error;
	double gain;
	double drive;

	//Roborio Sensors
	frc::Timer* Etime;
	frc::AnalogInput ArmPosition { 0 };
	frc::PowerDistributionPanel *PDP;
	BuiltInAccelerometer *accel;
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();

	//Driver Station Setting
	DriverStation station;
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
};

START_ROBOT_CLASS(Robot)
