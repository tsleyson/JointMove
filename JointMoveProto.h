#ifndef JOINTMOVE_H
#define JOINTMOVE_H

#include <cmath>
#include <fstream>
#include <vector>
#include <string>
#include "tserial.h"
#include "GeneralExceptions.h"
#include "MoveExceptions.h"

/*************************************************************************************
* JointMove.h contains the following public members of class JointMove:
*
* - int Move(double AngularPosition):
*      Precondition:  AngularPosition is a valid angle in radians; the class has been
*                     instantiated with valid values for the joint, bounds, com port,
*                     and resolution file.
*      Postcondition: The joint will be moved the given angle.
*      Throws:        BoundaryViolationException, if the given angle violates one of 
*                     the boundaries.
* - int Home:
*      Precondition:  The joint has a limit switch and has been moved to the negative
*                     side of its switch.
*      Postcondition: The joint will have moved until it hits the switch. This is
*                     represented by the position passed into the constructor's 
*                     HomePosition argument,which is zero by default.
* It also contains the constant PI, which is calculated to 30 places, for use in 
* radian angles.
*************************************************************************************/
namespace TLeyson_Robot
{
	const double PI = 3.1415926535897932384626433832795;

	enum joint {A=65, B, C, D, E, F, G, H};

	class JointMove
	{
		public:
			JointMove(char Joint, double UpperBound, double LowerBound, char* ResolutionFile,
					  Tserial* Port, bool LimitSwitch = true, double HomePosition = 0);

			int  Move(double AngularPosition);
			int  Home(void);

			char   ViewJoint          (void) const { return this->JointToMove; }
			double ViewUpperBound     (void) const { return this->UpperBound; } 
			double ViewLowerBound     (void) const { return this->LowerBound; }
			double ViewCurrentPosition(void) const { return this->CurrentPosition; }
		private:
		// Attributes
			char   JointToMove;
			double UpperBound;
			double LowerBound;
			// The home position, in radians.
			double HomePosition;
			// The current angular distance from home position, in ticks.
			int HomeDeviation;
			// The current angular position, in radians.
			double CurrentPosition;
			// The number of radians in a single tick.
			double Resolution;
			// The Tserial instance that controls the serial port.
			Tserial* ComPort;
			// The mask to use when testing the limit switch.
			char SwitchMask;
			// The size of a single group of ticks sent to the robot at one time.
			const static unsigned int GROUP_SIZE = 50;
			// The number of ticks remaining when we send the next group in.
			const static unsigned int REPLENISH = 15;
			// The variable that lets different instances share the com port.
			static bool PortFree;

		// Private helper methods
			std::string                      ReadFile      (char*  Filename);
			double                           ConvertToTicks(double AngularPosition);
			char                             CheckSwitch   (void);
			int                              Round         (double TickPosition);

			std::vector<int>                 DivideTicks   (int NumberOfTicks);
	};
}
#endif