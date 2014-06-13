#include <stdlib.h>
#include <new>
#include <iostream>
#include <string>
#include "JointMoveProto.h"
#include "GeneralExceptions.h"
#include "MoveExceptions.h"

namespace TLeyson_Robot
{
	bool JointMove::PortFree = true;

	/********************************************************************
	*                   JointMove::JointMove
	* Constructor for the JointMove class. Requires the following:
	*    - char JointToMove
	*        The letter of the motor which will be controlled by
	*        this instance of the class.
	*    - double UpperBound, LowerBound:
	*        The limits of motion (in radians). The Move command
	*        will check whether any angle it's given is outside
	*        this range.
	*    - char* ResolutionFile
	*        A C-string which contains the name of a file that
	*        has resolution values for the joints, in radians.
	*    - char* SwitchMaskFile
	*        Another C-string which contains the name of a file that
	*        has the masks needed to test whether a limit switch is
	*        open or closed.
	*    - Tserial* Port
	*        A pointer to an instance of Tserial that controls the
	*        serial port.
	*    - double HomePosition
	*        The angle defined as the joint's home position; the
	*        default value is 0/2pi radians or 0/360 degrees.
	* Precondition:
	*    - JointToMove is a character that represents a valid motor.
	*    - UpperBound and LowerBound are valid radian angles (they can be
	*      the same, though it won't get you anywhere).
	*    - ResolutionFile and SwitchMaskFile are names of existing files.
	*      See ReadFile for more information.
	*    - Port is a pointer to an instance of Tserial which has been
	*      connected to a com port on the robot.
	*    - HomePosition is a valid radian angle within the upper and lower
	*      bounds defined.
	* Postcondition:
	*    - An instance of JointMove will be created.
	* Throws:
	*    - FileNotFoundException and ValueNotFoundException, both from
	*      the ReadFile function. Instantiations of the class should be
	*      enclosed in a try-block.
	********************************************************************/
	JointMove::JointMove(joint Joint, double UpperBound, double LowerBound,
                         char* ResolutionFile, Tserial* Port, bool LimitSwitch, double HomePosition)
	{
		this->JointToMove  = toupper(Joint);
		this->UpperBound   = UpperBound;
		this->LowerBound   = LowerBound;
		this->ComPort      = Port;
		this->HomePosition = HomePosition;

		this->HomeDeviation = 0;
		this->CurrentPosition = HomePosition;

		// This function can throw errors, so the creation of an object
		// should take place inside a try/catch block.

		this->Resolution = atof( (this->ReadFile(ResolutionFile)).c_str() );

		this->SwitchMask = char(std::pow(2, float(this->JointToMove - 67)));

		JointMove::PortFree = true;
		if (LimitSwitch)
			this->Home();
	}

	/********************************************************************
	*                    JointMove::CheckSwitch
	* Checks the return value of the I command to determine if the limit
	* switch on the current motor is set. Returns false for closed and
	* true for open.
	* Precondition:  ComPort is a valid pointer to a Tserial instance;
	*                an XR series robot is connected to the computer.
	* Postcondition: A true or false value is returned. A true shows the
	*                switch is closed; a false shows it is open.
	*********************************************************************/
	char JointMove::CheckSwitch(void)
	{
		*(ComPort) << 'I';
		char SwitchCheck = ComPort->getChar();
		SwitchCheck -= 32;
		return SwitchCheck & this->SwitchMask;
	}

	/********************************************************************
	*                    JointMove::Round
	* Takes in the angular position in radians and returns a rounded
	* number of ticks to move. Other functions (Move) have the job of
	* ensuring that the AngularPosition is valid.
	* Precondition:  AngularPosition is a valid double number of radians.
	* Postcondition: AngularPosition will be converted into the number of
	*                ticks the robot needs to move, rounded to the nearest
	*                whole number.
	*********************************************************************/
	int JointMove::Round(double TickPosition)
	{
		int    TruncatedPosition = static_cast<int>(TickPosition);
		double DecimalPart       = TickPosition - TruncatedPosition;

		if (std::abs(DecimalPart) >= 0.5)
		{
			if (TickPosition > 0)
			{
				// If the angle is positive
				return ++TruncatedPosition;
			}
			else if (TickPosition < 0)
			{
				return --TruncatedPosition;
			}
		}
		// If the decimal part is less than 0.5
		return TruncatedPosition;
	}

	/********************************************************************
	*                    JointMove::ReadFile
	* Takes a file name, searches for the current motor letter in that
	* file, and reads out a number separated from the letter by
	* whitespace. The number must terminate with a newline. Returns a
	* string containing the number, which can be converted to a float 
	* or an int using atof or atoi.
	* Precondition:  Filename is a valid file which contains letters,
	*                representing motors, followed by a single tab or
	*                space, and a number.
	* Postcondition: The number will be read into a string and returned.
	********************************************************************/
	std::string JointMove::ReadFile(char* Filename)
	{
		std::ifstream infile;
		std::string   number;
		char CurrentPosition;

		infile.open(Filename);

		if (!infile.is_open())
		{
			throw TLeyson_Robot::FileNotFoundException(Filename);
		}

		while ( ( CurrentPosition = infile.get() ) != this->JointToMove )
		{
			if (infile.fail())
				throw TLeyson_Robot::ValueNotFoundException(Filename);
		}

		std::getline(infile, number);
		infile.close();
		return number;
	}

	/********************************************************************
	*                     JointMove::ConvertToTicks
	* Converts an angular position in radians to the ticks the robot needs
	* to move. Other functions (Move) check the validity of the number.
	* Precondition:  AngularPosition is a valid angle in radians.
	* Postcondition: A number of ticks, including a decimal portion, will
	*                be returned.
	********************************************************************/
	double JointMove::ConvertToTicks(double AngularPosition)
	{
		return AngularPosition / this->Resolution;
	}

	/********************************************************************
	*                    JointMove::DivideTicks
	* Divides the ticks into groups according to the GROUP_SIZE constant
	* in the class. 
	* Precondition:  NumberOfTicks is a rounded number of ticks.
	* Postcondition: A vector with the number of whole groups of size
	*                GROUP_SIZE and the final, non-uniform group will
	*                be returned to the calling process.
	*********************************************************************/
	std::vector<int> JointMove::DivideTicks(int NumberOfTicks)
	{
		// If the number of ticks is not divided evenly by the group size, we
		// have a final, less than full, group.
		unsigned int WholeGroups = NumberOfTicks / GROUP_SIZE;

		std::vector<int> TickGroups;  // The vector needs to hold the number of groups and the odd group.

		TickGroups.push_back(WholeGroups);

		if ( NumberOfTicks % GROUP_SIZE != 0 )
			TickGroups.push_back(NumberOfTicks % GROUP_SIZE);

		return TickGroups;
	}

	/*******************************************************************
	*                     JointMove::Move
	* Takes an angular position in radians and moves the arm to that
	* position. Throws a BoundaryViolationException if the given 
	* AngularPosition violates one of the defined boundaries of motion.
	* Precondition:  AngularPosition is a double number which represents
	*                a number of radians between the currently defined
	*                boundaries of motion.
	* Postcondition: The arm will have moved to the indicated position.
	********************************************************************/
	int JointMove::Move(double AngularPosition)
	{
		// First, if the position violates a boundary, throw an error.
		if ( !(AngularPosition > LowerBound && AngularPosition < UpperBound) )
			throw BoundaryViolationException();
		// If the current position and the desired position are the same, return.
		else if (this->CurrentPosition == AngularPosition)
			return 0;
		
		// Find out how far the desired position is from home (in ticks), then find out how
		// far that is from where you are
		int    DesiredPosition      = this->Round(this->ConvertToTicks(AngularPosition));
		int    TotalTicks           = DesiredPosition - this->HomeDeviation;

		// If AngularPosition is greater than the current position, we have to move
		// in the positive direction to reach it; otherwise, we have to move in the
		// negative direction. This means that the robot determines the positive and
		// negative directions of the coordinate system itself, based on its own 
		// inclination to move in a certain direction when presented with a choice.
		// This means that whatever we define HomePosition to be, everything greater
		// than it is always one way and everything smaller is always the other way.
		char MovementDirection = AngularPosition > this->CurrentPosition ? '+' : '-';
		char Newline[ ]  = {0x0A, 0x0D, '\0'};
		// A joint, a direction, 4 possible digits, and two newline characters plus a null = 9 spaces.
		char EvenCommand    [9] = {this->JointToMove, MovementDirection, '\0'};
		char UnevenCommand  [9] = {this->JointToMove, MovementDirection, '\0'};
		char TickString[4];  // With this length, we can do up to 9999 ticks, which is 6.666 * Pi radians
		std::vector<int> TickGroups = this->DivideTicks(abs(TotalTicks));

		// First send the uneven group, if there is one.
		if ( TickGroups.size() == 2 )
		{
			int OddGroup = TickGroups.back();
			TickGroups.pop_back();	
			// Convert the odd group to an integer and concatenate to the command string.
			_itoa_s(OddGroup, TickString, 4, 10);
			strcat_s(UnevenCommand, 9, TickString);
			(*ComPort) << UnevenCommand;
			(*ComPort) << Newline;
		}

		// Assemble a command string with the size of a normal group.
		_itoa_s(GROUP_SIZE, TickString, 4, 10);
		strcat_s(EvenCommand, 9, TickString);
		char QueryString[] = {this->JointToMove, '?', 0x0A, 0x0D, '\0'};
		char StopString [] = {this->JointToMove, 'X', ';', 0x0A, 0x0D, '\0'};

		// Now send the whole groups, using the string assembled above.
		for ( unsigned int k = TickGroups.front(); k > 0; k-- )
		{
			(*ComPort) << QueryString;
			char RegisterValue = abs(ComPort->getChar());
			RegisterValue -= 32;

		// Note: I'm a little worried that if the register weren't below
		// the replenish level, the program would just move on and skip a
		// group. However, the register seems to run down pretty fast, so it
		// might never be a problem.
		// Actually it was a problem, but it's been solved.
			while ( RegisterValue > REPLENISH )
			{
				Sleep(10);
				(*ComPort) << QueryString;
				RegisterValue = abs(ComPort->getChar());
				RegisterValue -= 32;
			}
			(*ComPort) << EvenCommand;
			(*ComPort) << Newline;
		}

		this->HomeDeviation = DesiredPosition;
		this->CurrentPosition = AngularPosition;

		return 0;
	}

	/********************************************************************
	*                     JointMove::Home
	* Tries to move the joint to the home position defined by the 
	* constructor. 
	* Precondition:  The joint represented by the instance must have a 
	*                limit switch on it; the joint must be positioned so
	*                that moving in the positive direction will lead it
	*                across the switch before making a complete revolution.
	* Postcondition: The joint will be repositioned to the limit switch.
	*********************************************************************/
	int JointMove::Home(void)
	{
		char Move[] = {this->JointToMove, '+', '2', '0', 0x0A, 0x0D, '\0'};
		char Stop[] = {this->JointToMove, 'X', 0x0A, 0x0D, '\0'};
		char SwitchStatus = this->CheckSwitch();

		while (!PortFree);

		PortFree = false;
		while (SwitchStatus)
		{
			//std::cout << Move << std::endl;
			*(this->ComPort) << Move;
			Sleep(300);
			SwitchStatus = this->CheckSwitch();
		}
		*(this->ComPort) << Stop;
		PortFree = true;
		return 0;
	}
} // End namespace TLeyson_Robot
