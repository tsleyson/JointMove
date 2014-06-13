#include <iostream>
#include "JointMoveProto.h"
#include "tserial.h"
using namespace std;
using TLeyson_Robot::PI;
using TLeyson_Robot::joint;

int MoveRobot(Tserial* ComPort, char Joint);

int main(void)
{
	Tserial com;
	com.connect(L"com1", 9600, spEVEN);

	try
	{
		
		TLeyson_Robot::JointMove djoint('D', 5 * PI/12, -5 * PI/12, 
			"H:\\C++_Examples\\MoveClass\\Prototypes\\JointMoveProto\\resolutions.txt", &com);
		TLeyson_Robot::JointMove ejoint('E', PI/3, -PI/3, 
			"H:\\C++_Examples\\MoveClass\\Prototypes\\JointMoveProto\\resolutions.txt", &com);
		TLeyson_Robot::JointMove fjoint('F', PI/6, -PI/6, 
			"H:\\C++_Examples\\MoveClass\\Prototypes\\JointMoveProto\\resolutions.txt", &com);
		Sleep(1000);
		djoint.Move(-PI/8);
		djoint.Move(PI/12);
		fjoint.Move(PI/8);
		djoint.Move(-PI/12);
		fjoint.Move(-PI/12);
		ejoint.Move(-PI/8);

	}
	catch (TLeyson_Robot::FileNotFoundException e)
	{
		cout << "File not found." << endl;
		cout << "File with name " << e.fname << " could not be located." << endl;
	}
	catch (TLeyson_Robot::ValueNotFoundException f)
	{
		cout << "The value could not be found in " << f.fname << endl;
	}
	catch (TLeyson_Robot::BoundaryViolationException)
	{
		cout << "Boundary violated." << endl;
	}

	com.disconnect();
	return 0;
}

// This function was used to reposition the robot after movement; the Visual Basic program
// does a better job.

// int MoveRobot(Tserial* ComPort, char Direction, char Joint)
// {
	// char input;
	// char Up[] = {Joint, '+', '5', '0', 0x0A, 0x0D};
	// char Down[] = {Joint, '-', '5', '0', 0x0A, 0x0D};
	// while ( (input = cin.get()) == '+' || input == '-')
	// {
		// if (input == '+')
			// (*ComPort) << Up;
		// else if (input == '-')
			// (*ComPort) << Down;
	// }
	// return 0;
// }