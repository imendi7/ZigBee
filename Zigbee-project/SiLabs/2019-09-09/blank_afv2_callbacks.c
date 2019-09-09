// This callback file is created for your convenience. You may add application
// code to this file. If you regenerate this file over a previous version, the
// previous version will be overwritten and any code you have added will be
// lost.

#include "app/framework/include/af.h"

#include EMBER_AF_API_NETWORK_CREATOR
#include EMBER_AF_API_NETWORK_CREATOR_SECURITY

EmberEventControl networkCreationEventControl;
EmberEventControl networkOpeningEventControl;
EmberEventControl LEDEventControl;
EmberEventControl levelEventControl;

static uint8_t lastButton;

void networkCreationEventHandler(void)
{
	emberEventControlSetInactive(networkCreationEventControl);

	GPIO_PinModeSet(gpioPortF, 3, gpioModeInput, 1);

	EmberStatus creatorState, initState;

	initState = emberInit();

	creatorState = emberAfPluginNetworkCreatorStart(true);

	emberEventControlSetDelayMS(networkOpeningEventControl, 10000);

}


void networkOpeningEventHandler(void)
{
	emberEventControlSetInactive(networkOpeningEventControl);

	EmberStatus openState;

	openState = emberAfPluginNetworkCreatorSecurityOpenNetwork();

	emberAfCorePrintln("Open state: 0x%X", openState);

	emberEventControlSetDelayMS(LEDEventControl, 10000);

}

void LEDEventHandler(void)
{
	emberEventControlSetInactive(LEDEventControl);

	EmberStatus sendStatus;

	emberAfCorePrintln("LED");

	emberAfFillCommandOnOffClusterToggle();

	emberAfSetCommandEndpoints(1,11);

	sendStatus = emberAfSendCommandBroadcast(0xffff);

	emberAfCorePrintln("LED Status: 0x%X", sendStatus);

	emberEventControlSetDelayMS(LEDEventControl, 600000);
}

void levelEventHandler(void)
{
	emberEventControlSetInactive(levelEventControl);

	EmberStatus sendStatus;

	emberAfCorePrintln("Level");

	int nextLevel = 100;

	uint16_t transitionTime = 100;

	emberAfFillCommandLevelControlClusterMoveToLevel(nextLevel, transitionTime, 0, 0);

	emberAfSetCommandEndpoints(1,11);

	sendStatus = emberAfSendCommandBroadcast(0xffff);

	emberAfCorePrintln("level value: %d", nextLevel);

	emberAfCorePrintln("gpio: %f", GPIO_PinInGet(gpioPortF, 3));

	emberEventControlSetDelayMS(LEDEventControl, 3500);
}

/** @brief Hal Button Isr
 *
 * This callback is called by the framework whenever a button is pressed on the
 * device. This callback is called within ISR context.
 *
 * @param button The button which has changed state, either BUTTON0 or BUTTON1
 * as defined in the appropriate BOARD_HEADER.  Ver.: always
 * @param state The new state of the button referenced by the button parameter,
 * either ::BUTTON_PRESSED if the button has been pressed or ::BUTTON_RELEASED
 * if the button has been released.  Ver.: always
 */
void emberAfHalButtonIsrCallback(int8u button,
                                 int8u state)
{
}

/** @brief Main Init
 *
 * This function is called from the application's main function. It gives the
 * application a chance to do any initialization required at system startup. Any
 * code that you would normally put into the top of the application's main()
 * routine should be put into this function. This is called before the clusters,
 * plugins, and the network are initialized so some functionality is not yet
 * available.
        Note: No callback in the Application Framework is
 * associated with resource cleanup. If you are implementing your application on
 * a Unix host where resource cleanup is a consideration, we expect that you
 * will use the standard Posix system calls, including the use of atexit() and
 * handlers for signals such as SIGTERM, SIGINT, SIGCHLD, SIGPIPE and so on. If
 * you use the signal() function to register your signal handler, please mind
 * the returned value which may be an Application Framework function. If the
 * return value is non-null, please make sure that you call the returned
 * function from your handler to avoid negating the resource cleanup of the
 * Application Framework itself.
 *
 */
void emberAfMainInitCallback(void){

	emberEventControlSetActive(networkCreationEventControl);

}


