#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "getlcdbuttons.c"

typedef enum
{
	kAllianceBlue = 0,
  kAllianceRed
}
vexAlliance;

typedef enum
{
   kStartShort = 0, //driver loads side
   kStartLong// side with more cones
}
vexStartposition;

typedef enum
{
	KStartMobiles = 0,
	KStartStationary
}
vexFacingposition;

typedef enum
{
    kMenuStart    = 0,
    kMenuAlliance = 0,
    kMenuStartpos,
    KMenuFace,
    kMenuAutonSelect,
    kMenuMax,
}
vexLcdMenus;

static  vexAlliance         vAlliance = kAllianceBlue;
static  vexStartposition    vPosition = kStartShort;
static  vexFacingposition   VFace     = KStartMobiles;
static  short               vAuton = 0;

void
LcdAutonomousDisplay( vexLcdMenus menu )
{
    // Cleat the lcd
    clearLCDLine(0);
    clearLCDLine(1);

    // Display the selection arrows
    displayLCDString(1,  0, l_arr_str);
    displayLCDString(1, 13, r_arr_str);
    displayLCDString(1,  5, "CHANGE");

    // Show the autonomous names
    switch( menu )
    {
        case    kMenuAlliance:
            if( vAlliance == kAllianceBlue )
                displayLCDString(0, 0, "Alliance - BLUE");
            else
                displayLCDString(0, 0, "Alliance - RED");
            break;

            case    KMenuFace:
            if( VFace == KStartMobiles)
            	displayLCDString(0, 0, "Start-Mobiles");
            else
                displayLCDString(0, 0, "Start-Stationary");
            break;

            case    kMenuStartpos:
            if( vPosition == kStartShort)
                displayLCDString(0, 0, "Start-Driver Loads");
            else
                displayLCDString(0, 0, "Start - Long Side ");
            break;

        case    kMenuAutonSelect:
            switch( vAuton ) {
                case    0:
                    displayLCDString(0, 0, "Mobile+3+20zone");
                    break;
                case    1:
                    displayLCDString(0, 0, "Mobile+2+20zone");
                    break;
                case    2:
                    displayLCDString(0, 0, "Mobile+3+10zone");
                    break;
                case    3:
                    displayLCDString(0, 0, "Mobile+3+10zone");
                    break;
                default:
                    char    str[20];
                    sprintf(str,"Undefined %d", vAuton );
                    displayLCDString(0, 0, str);
                    break;
                }
            break;

        default:
            displayLCDString(0, 0, "Unknown");
            break;
        }
}

/*-----------------------------------------------------------------------------*/
/*  Rotate through a number of menus and use center button to select choices   */
/*-----------------------------------------------------------------------------*/

void
LcdAutonomousSelection()
{
    TControllerButtons  button;
    vexLcdMenus  menu = kMenuStart;

    // Turn on backlight
    bLCDBacklight = true;

    // diaplay default choice
    LcdAutonomousDisplay(0);

    while( bIfiRobotDisabled )
        {
        // this function blocks until button is pressed
        button = getLcdButtons();

        // Display and select the autonomous routine
        if( ( button == kButtonLeft ) || ( button == kButtonRight ) ) {
            // previous choice
            if( button == kButtonLeft )
                if( --menu < kMenuStart ) menu = kMenuMax-1;
            // next choice
            if( button == kButtonRight )
                if( ++menu >= kMenuMax ) menu = kMenuStart;
            }

        // Select this choice for the menu
        if( button == kButtonCenter )
            {
            switch( menu ) {
                case    kMenuAlliance:
                    // alliance color
                    vAlliance = (vAlliance == kAllianceBlue) ? kAllianceRed : kAllianceBlue;
                    break;
                case    kMenuStartpos:
                    // start position
                    vPosition = (vPosition == kStartShort) ? kStartLong : kStartShort;
                    break;
                case    kMenuAutonSelect:
                    // specific autonomous routine for this position
                    if( ++vAuton == 3 )
                        vAuton = 0;
                    break;
                }

            }

        // redisplay
        LcdAutonomousDisplay(menu);

        // Don't hog the cpu !
        wait1Msec(10);
        }
}

/*-----------------------------------------------------------------------------*/
/*  Display some status during Autonomous or driver controlled periods         */
/*-----------------------------------------------------------------------------*/

void
LcdDisplayStatus( long enabledTime )
{
    string str;

    sprintf(str,"VBatt %7.2f   ", nAvgBatteryLevel/1000.0 );
    displayLCDString(0, 0, str);
    if( bIfiAutonomousMode )
        sprintf(str, "Autonomous %.2f", (nPgmTime-enabledTime)/1000.0);
    else
        sprintf(str, "Driver     %.2f", (nPgmTime-enabledTime)/1000.0);
    displayLCDString(1, 0, str);
}

/*-----------------------------------------------------------------------------*/
/*  This task replaces the default competition control                         */
/*  It is started during pre_auton() and does not terminate                    */
/*  pre_auton also does not return and run the remaining code in the "main"    */
/*  task that is part of Vex_Competition_Includes.c                            */
/*                                                                             */
/*  This task does not stop other tasks when the robot is disable, that's up   */
/*  to the user to modify and add, using allTasksStop() is not recommended as  */
/*  it will also stop this task, allTasksStop stops everything except main     */
/*-----------------------------------------------------------------------------*/

task mainTask()
{
    long    enabledTime;

    while( true )
        {
        // When disabled run the autonomous selector
        if( bIfiRobotDisabled )
            LcdAutonomousSelection();
        else
            {
            // time we were enabled
            enabledTime = nPgmTime;

            // when enabled run either autonomous or usercontrol
            if (bIfiAutonomousMode)
                {
                StartTask(autonomous);

                // Waiting for autonomous phase to end
                while (bIfiAutonomousMode && !bIfiRobotDisabled) {
                    if (!bVEXNETActive) {
                        if (nVexRCReceiveState == vrNoXmiters) // the transmitters are powered off!!
                            allMotorsOff();
                        }

                    // Status display during autonomous
                    LcdDisplayStatus(enabledTime);

                    // Waiting for autonomous phase to end
                    wait1Msec(25);
                    }

                allMotorsOff();

                // Stop other taks here
                // if needed
                }
            else
                {
                StartTask(usercontrol);

                // Here we repeat loop waiting for user control to end and (optionally) start
                // of a new competition run
                while (!bIfiAutonomousMode && !bIfiRobotDisabled) {
                    if (nVexRCReceiveState == vrNoXmiters) // the transmitters are powered off!!
                        allMotorsOff();

                    // Status display during usercontrol
                    LcdDisplayStatus(enabledTime);

                    wait1Msec(25);
                    }

                allMotorsOff();

                // Stop other taks here
                // if needed
                }
            }
        }
}

/*-----------------------------------------------------------------------------*/
/* The pre_auton function does not return, "normal" competition control is     */
/* therefore disabled.                                                         */
/*-----------------------------------------------------------------------------*/

void pre_auton()
{
    // start our own main task
    StartTask( mainTask );

    // now block - use our own comp control
    while(1) wait1Msec(1000);
}

void
autonomousRed()
{
    // code is the same as the blue code
}

void
autonomousBlue()
{
    if( vPosition == kStartShort ) {
        switch( vAuton ) {
            case    0:
                // run some autonomous code
                break;
            case    1:
                // run some other autonomous code
                break;
            default:
                break;
        }
    }
    else { // middle zone
        switch( vAuton ) {
            case    0:
                // run some autonomous code
                break;
            case    1:
                // run some other autonomous code
                break;
            default:
                break;
        }
    }
}

task autonomous()
{
    if( vAlliance == kAllianceBlue )
        autonomousBlue();
    else
        autonomousRed();
}

task usercontrol()
{
    while (true) {
        wait1Msec(10);
        }
}
