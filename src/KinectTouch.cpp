//============================================================================
// Name        : KinectTouch.cpp
// Author      : github.com/robbeofficial
// Version     : 0.something
// Description : recognizes touch points on arbitrary surfaces using kinect
// 				 and maps them to TUIO cursors
// 				 (turns any surface into a touchpad)
//============================================================================

/*
 * 1. point your kinect from a higher place down to your table
 * 2. start the program (keep your hands off the table for the beginning)
 * 3. use your table as a giant touchpad
 */

#include <iostream>
#include <vector>
#include <map>
#include <cstring>
using namespace std;

// openCV
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;

// openNI
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
using namespace xn;

extern bool gKinectAvailable;
#define CHECK_RC(rc, what)                                      \
	if (rc != XN_STATUS_OK)                                     \
	{                                                           \
		printf("%s failed: %s\n", what, xnGetStatusString(rc)); \
		gKinectAvailable = false;                               \
		return rc;                                              \
	}

// TUIO
#define TUIO_SUPPORT 1
#ifdef TUIO_SUPPORT
#include "TuioServer.h"
using namespace TUIO;
#endif

#define USE_XZ 1
// TODO smoothing using kalman filter

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------

// X11
#include <X11/Xlib.h>
#include <X11/extensions/XInput2.h>
#include <X11/extensions/XTest.h>

// #define HAVE_XINERAMA 1
#ifdef HAVE_XINERAMA
#include <X11/extensions/Xinerama.h>
#endif

// OpenNI
xn::Context xnContext;
xn::DepthGenerator xnDepthGenerator;
xn::ImageGenerator xnImgeGenertor;

bool mousePressed = false;
bool gKinectAvailable = false;
bool gMissingDeviceWarned = false;
const short kDefaultDepthValue = 0;

//---------------------------------------------------------------------------
// Functions
//---------------------------------------------------------------------------

void warnMissingDevice()
{
	if (!gMissingDeviceWarned)
	{
		cerr << "Warning: Kinect sensor not detected. Using default depth data." << endl;
		gMissingDeviceWarned = true;
	}
}

XnStatus initOpenNI(const XnChar *fname)
{
	XnStatus nRetVal = XN_STATUS_OK;
	ScriptNode scriptNode;

	// initialize context
	nRetVal = xnContext.InitFromXmlFile(fname, scriptNode);
	CHECK_RC(nRetVal, "InitFromXmlFile");

	// initialize depth generator
	nRetVal = xnContext.FindExistingNode(XN_NODE_TYPE_DEPTH, xnDepthGenerator);
	CHECK_RC(nRetVal, "FindExistingNode(XN_NODE_TYPE_DEPTH)");

	// initialize image generator
	nRetVal = xnContext.FindExistingNode(XN_NODE_TYPE_IMAGE, xnImgeGenertor);
	CHECK_RC(nRetVal, "FindExistingNode(XN_NODE_TYPE_IMAGE)");

	gKinectAvailable = true;
	return nRetVal;
}

bool updateDepthFrame(Mat1s &depth)
{
	if (!gKinectAvailable)
	{
		warnMissingDevice();
		depth.setTo(kDefaultDepthValue);
		return false;
	}

	XnStatus status = xnContext.WaitAndUpdateAll();
	if (status != XN_STATUS_OK)
	{
		cerr << "Failed to read from Kinect: " << xnGetStatusString(status) << endl;
		gKinectAvailable = false;
		depth.setTo(kDefaultDepthValue);
		return false;
	}

	const XnDepthPixel *depthMap = xnDepthGenerator.GetDepthMap();
	if (depthMap == nullptr)
	{
		cerr << "Depth map unavailable. Is the Kinect connected?" << endl;
		gKinectAvailable = false;
		depth.setTo(kDefaultDepthValue);
		return false;
	}

	std::memcpy(depth.data, depthMap, depth.total() * sizeof(XnDepthPixel));
	return true;
}

void average(vector<Mat1s> &frames, Mat1s &mean)
{
	Mat1d acc(mean.size());
	Mat1d frame(mean.size());

	for (unsigned int i = 0; i < frames.size(); i++)
	{
		frames[i].convertTo(frame, CV_64FC1);
		acc = acc + frame;
	}

	acc = acc / frames.size();

	acc.convertTo(mean, CV_16SC1);
}

class MouseController
{
public:
	static Display *display;
	static Screen *screen;
	static time_t lastActivityTime;
	static const int INACTIVITY_TIMEOUT = 15; // seconds
	static bool isMovingRandomly;
	static bool isPaused; // New pause state

	static void moveMouse(int x, int y)
	{
		Display *display = XOpenDisplay(NULL);
		if (display == NULL)
		{
			fprintf(stderr, "Unable to open X display\n");
			return;
		}

		XWarpPointer(display, None, DefaultRootWindow(display), 0, 0, 0, 0, x, y);
		XFlush(display);
		XCloseDisplay(display);
	}

	static void clickMouse(int button)
	{
		Display *display = XOpenDisplay(NULL);
		if (display == NULL)
		{
			fprintf(stderr, "Unable to open X display\n");
			return;
		}

		XTestFakeButtonEvent(display, button, True, CurrentTime);
		XTestFakeButtonEvent(display, button, False, CurrentTime);
		XFlush(display);
		XCloseDisplay(display);
	}

	static int getScreenCount()
	{
		int screenCount = ScreenCount(display);
#ifdef HAVE_XINERAMA
		if (XineramaIsActive(display))
		{
			XineramaScreenInfo *screens = XineramaQueryScreens(display, &screenCount);
			XFree(screens);
		}
#endif
		return screenCount;
	}

	static void togglePause()
	{
		isPaused = !isPaused;
		// Visual feedback could be added here
		std::cout << "Kinect control " << (isPaused ? "paused" : "resumed") << std::endl;
	}

	static bool checkRightClick()
	{
		XEvent event;
		while (XPending(display))
		{
			XNextEvent(display, &event);
			if (event.type == ButtonPress && event.xbutton.button == 3)
			{ // 3 is right mouse button
				togglePause();
				return true;
			}
		}
		return false;
	}

	static void updateActivityTimestamp()
	{
		if (!isPaused)
		{
			lastActivityTime = time(nullptr);
			isMovingRandomly = false;
		}
	}

	static bool shouldStartRandomMovement()
	{
		if (isPaused || isMovingRandomly)
			return false;
		time_t currentTime = time(nullptr);
		return (currentTime - lastActivityTime) >= INACTIVITY_TIMEOUT;
	}

	static void performRandomMovement()
	{
		if (!isMovingRandomly)
		{
			isMovingRandomly = true;
		}
		// Generate random coordinates within screen bounds
		int maxX = WidthOfScreen(screen);
		int maxY = HeightOfScreen(screen);
		int randomX = rand() % maxX;
		int randomY = rand() % maxY;

		XWarpPointer(display, None, RootWindowOfScreen(screen),
					 0, 0, 0, 0, randomX, randomY);
		XFlush(display);
	}
};

Display *MouseController::display = XOpenDisplay(NULL);
Screen *MouseController::screen = ScreenOfDisplay(MouseController::display, 0);
time_t MouseController::lastActivityTime = time(nullptr);
bool MouseController::isMovingRandomly = false;
bool MouseController::isPaused = false;

int main()
{

	const unsigned int nBackgroundTrain = 30;
	// const unsigned short touchDepthMin = 10;
	// const unsigned short touchDepthMax = 200;
	int touchDepthMin = 170;
	int touchDepthMax = 3000;

	// const unsigned int touchMinArea = 50;//50

	int touchMinArea = 100; // 800;//50

	const bool localClientMode = true; // connect to a local client

	const double debugFrameMaxDepth = 4000; // maximal distance (in millimeters) for 8 bit debug depth frame quantization
	const char *windowName = "Debug";
	const Scalar debugColor0(0, 0, 128);
	const Scalar debugColor1(255, 0, 0);	 // red
	const Scalar debugColor2(255, 255, 255); // white

	int xMin = 0;
	int xMax = 640;
	int yMin = 390;
	int yMax = 430;
	int zMin = 2736;
	int zMax = 3200;

	Mat1s depth(480, 640);	// 16 bit depth (in millimeters)
	Mat1b depth8(480, 640); // 8 bit depth
	Mat3b rgb(480, 640);	// 8 bit depth

	Mat3b debug(480, 640); // debug visualization

	Mat1s foreground(640, 480);
	Mat1b foreground8(640, 480);

	Mat1b touch(640, 480); // touch mask

	Mat1s background(480, 640);
	vector<Mat1s> buffer(nBackgroundTrain);

	XnStatus initStatus = initOpenNI("./niConfig.xml");
	if (initStatus != XN_STATUS_OK)
	{
		cerr << "OpenNI initialization failed: " << xnGetStatusString(initStatus)
			 << ". Make sure the Kinect sensor is connected." << endl;
		warnMissingDevice();
	}

	cout << "#Screens: " << MouseController::getScreenCount() << std::endl;
// TUIO server object
#ifdef TUIO_SUPPORT
	TuioServer *tuio;
	if (localClientMode)
	{
		tuio = new TuioServer();
	}
	else
	{
		tuio = new TuioServer("192.168.0.2", 3333, false);
	}
	TuioTime time;
#endif
	// create some sliders
	namedWindow(windowName);
	createTrackbar("xMin", windowName, &xMin, 640);
	createTrackbar("xMax", windowName, &xMax, 640);
	createTrackbar("yMin", windowName, &yMin, 480);
	createTrackbar("yMax", windowName, &yMax, 480);
	createTrackbar("touchDepthMin", windowName, &touchDepthMin, 4000);
	createTrackbar("touchDepthMax", windowName, &touchDepthMax, 4000);
	createTrackbar("zMin", windowName, &zMin, 4000);
	createTrackbar("zMax", windowName, &zMax, 4000);
	createTrackbar("touchMinArea", windowName, &touchMinArea, 4000);

	while ((char)waitKey(1) != (char)27)
	{
		// Check for right-click to toggle pause

		// read available data
		updateDepthFrame(depth);
		// xnImgeGenertor.GetGrayscale8ImageMap()
		imshow(windowName, debug);
	}

	// create background model (average depth)
	for (unsigned int i = 0; i < nBackgroundTrain; i++)
	{
		updateDepthFrame(depth);
		buffer[i] = depth;
	}
	average(buffer, background);

	while ((char)waitKey(1) != (char)27)
	{

		// MouseController::checkRightClick();
		if (waitKey(1) == 'q')
		{ // Right mouse button pressed
			MouseController::togglePause();
			continue; // Skip the rest of the loop if paused
		}
		// 			if(!MouseController::isPaused) // Reset pause state at the start of each loop
		// {
		// 	MouseController::togglePause();
		// }
		// read available data
		updateDepthFrame(depth);

		// xnImgeGenertor.GetGrayscale8ImageMap()

		// update rgb image
		// rgb.data = (uchar*) xnImgeGenertor.GetRGB24ImageMap(); // segmentation fault here
		// cvtColor(rgb, rgb, CV_RGB2BGR);

		// extract foreground by simple subtraction of very basic background model
		foreground = background - depth;

		// find touch mask by thresholding (points that are close to background = touch points)
		// extract ROI
		Rect roi(xMin, yMin, xMax - xMin, yMax - yMin);

#define TOUCH 1

#ifdef TOUCH
		touch = (foreground > touchDepthMin) & (foreground < touchDepthMax);
		Mat touchRoi = touch(roi);

#else
		// convert foreground to 8-bit single-channel image
		//		foreground.convertTo(foreground8, CV_8U, 255.0/xnGetDeviceMaxDepth());

		Mat touchRoi = foreground8(roi);
#endif

		// find touch points
		vector<vector<Point2i>> contours;
		vector<Point2f> touchPoints;
		vector<float> depth_of_touch_points;
		findContours(touchRoi, contours, RETR_LIST, CHAIN_APPROX_SIMPLE, Point2i(xMin, yMin));
		for (unsigned int i = 0; i < contours.size(); i++)
		{
			Mat contourMat(contours[i]);
			// find touch points by area thresholding
			if (contourArea(contourMat) > touchMinArea)
			{
				Scalar center = mean(contourMat);
				Point2i touchPoint(center[0], center[1]);
				touchPoints.push_back(touchPoint);
				depth_of_touch_points.push_back(depth.at<short>(touchPoint));
			}
			// Print out contour details
			//			cout << "Contour #" << i << ": Area = " << contourArea(contourMat) << ", Center = (" << center[0] << ", " << center[1] << ")" << endl;
		}

		// send TUIO cursors

		time = TuioTime::getSessionTime();
		tuio->initFrame(time);
		if (!MouseController::isPaused)
		{
			if (touchPoints.size() > 0 && MouseController::isPaused == false)
			{
				// Update activity timest amp when there are touch points
				MouseController::updateActivityTimestamp();

				for (unsigned int i = 0; i < touchPoints.size(); i++)
				{ // touch points
					float cursorX = 1 - (touchPoints[i].x - xMin) / (xMax - xMin);
#ifdef USE_XZ
					cout << "depth_of_touch_points[i]: " << depth_of_touch_points[i] << endl;
					float cursorY = (depth_of_touch_points[i] - zMin) / (zMax - zMin);
#else
					float cursorY = 1 - (touchPoints[i].y - yMin) / (yMax - yMin);
#endif

					TuioCursor *cursor = tuio->getClosestTuioCursor(cursorX, cursorY);
					if (cursor != NULL)
					{
						// cout << "cursor: " << cursor->getX() << ", " << cursor->getY() << endl;
						XWarpPointer(MouseController::display, None, RootWindowOfScreen(MouseController::screen), 0, 0, 0, 0, cursor->getX(), cursor->getY());
					}

					// TODO improve tracking (don't move cursors away, that might be closer to another touch point)
					if (cursor == NULL || cursor->getTuioTime() == time)
					{
						tuio->addTuioCursor(cursorX, cursorY);
						XTestFakeButtonEvent(MouseController::display, 1, True, CurrentTime);
						XFlush(MouseController::display);
					}
					else
					{
						tuio->updateTuioCursor(cursor, cursorX, cursorY);
					}
					// Convert TUIO cursor to screen coordinates and move the mouse
					constexpr int laptopWidth = 1920;
					int screenX = cursorX * (WidthOfScreen(MouseController::screen) - laptopWidth);
					int screenY = cursorY * HeightOfScreen(MouseController::screen);
					cout << "screenX: " << screenX << ", screenY: " << screenY << " W: " << WidthOfScreen(MouseController::screen) << "H: " << HeightOfScreen(MouseController::screen) << endl;
					XWarpPointer(MouseController::display, None, RootWindowOfScreen(MouseController::screen), 0, 0, 0, 0, screenX, screenY);
					XFlush(MouseController::display);

					//				MouseController::moveMouse(screenX, screenY);
				}
			}
			else if (MouseController::shouldStartRandomMovement())
			{
				MouseController::performRandomMovement();
			}
		}
		tuio->stopUntouchedMovingCursors();
		tuio->removeUntouchedStoppedCursors();
		tuio->commitFrame();

		// draw debug frame
		depth.convertTo(depth8, CV_8U, 255 / debugFrameMaxDepth); // render depth to debug frame
		cvtColor(depth8, debug, COLOR_GRAY2BGR);
		debug.setTo(debugColor0, touch);	   // touch mask
		rectangle(debug, roi, debugColor1, 2); // surface boundaries
		for (unsigned int i = 0; i < touchPoints.size(); i++)
		{ // touch points
			circle(debug, touchPoints[i], 5, debugColor2, FILLED);
		}

		// render debug frame (with sliders)
		imshow(windowName, debug);
		// imshow("image", rgb);
	}
	XCloseDisplay(MouseController::display);
	return 0;
}
