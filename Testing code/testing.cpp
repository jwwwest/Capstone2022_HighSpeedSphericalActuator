//============================================================================
// Name        : Dual.cpp
// Author      : Henry Cole & Jaiden Westover 
// Version     : 23.0
// Description : Epos motor control in in C++ for use with flapparatus 5.0
// Notes       : Henry's Latest Version (Fastest thus far at roughly 10 Hz)
//============================================================================
#include <iostream>
#include <string>
#include <algorithm>
#include <cmath>
#include "Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>

typedef void* HANDLE;
typedef int BOOL;

using namespace std;

/* Classes and their functions*/
class Node {
    public:
    string data;
    string type; // either num, op, param, or fun
    string sine;
    Node* next;
    Node* previ;

};
void append(Node** head_ref, string new_data, string new_type, string new_sine){
    // allocate node info
    Node* new_node = new Node();
    Node* last = *head_ref;

    new_node->data = new_data;
    new_node->type = new_type;
    new_node->sine = new_sine;

    // since appending to end of ll it points to null
    new_node->next = NULL;

    // If the ll is empty the we make this new node as the head
    if (*head_ref == NULL)
    {
        new_node->previ = NULL;
        *head_ref = new_node;
        return;
    }

    //else we will travesre the ll till the last node
    while (last->next != NULL)
        last = last->next;

    // changing the next of last node
    last->next = new_node;

    // make last node previous of new node
    new_node->previ =last;

    return;

}
void PrintLL(Node* node){
    // forward traversal
    Node* last;

    cout << "DLL :"<<endl;
    while(node != NULL){
        cout<<" "<<node->data<<" OF "<<node->type<<" " << " sine of: " << node->sine<<endl;
        last = node;
        node = node->next;
    }
}

class Node_float{
    public:
    long pos;
    float time;
    Node_float* next;
    Node_float* previ;

};
void append_f(Node_float** head_ref, long new_pos, float new_time){
    // allocate node info
    Node_float* new_node = new Node_float();
    Node_float* last = *head_ref;

    new_node->pos = new_pos;
    new_node->time = new_time;

    // since appending to end of ll it points to null
    new_node->next = NULL;

    // If the ll is empty the we make this new node as the head
    if (*head_ref == NULL)
    {
        new_node->previ = NULL;
        *head_ref = new_node;
        return;
    }

    //else we will travesre the ll till the last node
    while (last->next != NULL)
        last = last->next;

    // changing the next of last node
    last->next = new_node;

    // make last node previous of new node
    new_node->previ =last;

    return;

}
void Print_pos(Node_float* node){
    // forward traversal
    Node_float* last;

    cout << "results :"<<endl;
    while(node != NULL){
        cout<<" "<<node->pos<<" at "<<node->time<<" " << endl;
        last = node;
        node = node->next;
    }
}


// var itin

/* Global Variable declarations */
string pitch_eq = "1*sin(2*10*pi*t+0)";
string roll_eq = "1*sin(2*10*pi*t+0)";
float pi = 3.141592653; // needs to be of type float, and in the math lib its of type double
float time2run = 2.4;
float sample_time = .004;
Node* head_p = NULL;            // reference to head of pitch equation
Node* head_r = NULL;            // reference to head of roll equation
Node_float* head_pitch_ref = NULL;    // reference to the start of the reference signal, every node is a new sample, incrementing 1msec
Node_float* head_roll_ref = NULL;    // reference to the start of the reference signal, every node is a new sample, incrementing 1msec
int actualposition_p;
int actualposition_r;
int p_multip = 1433;
int r_multip = 682;

void* g_pKeyHandle = 0;
unsigned short g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;

void* g_pKeyHandle_r = 0;
unsigned short g_usNodeId_r = 1;
string g_deviceName_r;
string g_protocolStackName_r;
string g_interfaceName_r;
string g_portName_r;
int g_baudrate_r = 0;

const string g_programName = "Epos Control";

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif

/* Function declarations  */
void UserInput();                   // where the user enters pitch, roll, and time to run
void EnterPitch();                  // takes the pitch equation, string
void EnterRoll();                   // takes the roll equation, string
void EnterTime();                   // takes the time to run, float
void DoubleCheck();                 // this is where we check to make sure the user has entered everything correctly
string NormalizeString(string);     // whatever string is, we remove all spaces and all characters to lowercase
float RollCompensate(float, float); // This function takes the given pitch position and the given roll position and
                                    // compensates the roll position for the coupling seen by the actuator to produce
                                    // the position to tell the motor to achieve the desired position on actuator. Returns roll pos.
bool isOperator(string);            // operators are; +,-,(,),^,and *
bool isNumber(string);              // numbers are; 0,1,2,3,4,5,6,7,8,and 9
bool isFunction(string);            // Functions are; sin and cos and e. For now
bool isParam(string);               // Param is t, returns true is string is t
void CreateRoll();                  // creating the function for roll axis
void CreatePitch();                 // creating the function for pitch axis
void CreateSignalPitch();           // creating the pitch reference signal.
void CreateSignalRoll();            // creating the roll reference signal.
float RollEquation(float);          // this is the roll equation as an actual function, returns desired position for given time
float PitchEquation(float);         // this is the pitch equation as an actual function, returns desired position for given time
void ParseToDLL(Node**,string);      // parsing string equation into the DLL
float StringDecoder(string s,float t, string type);
float DLLDecoder(Node*,float);      // decode what is in the node equations to produce a float value, the desired actuator position
string GetNum(int, string);                             // returns number, used in parsing
string GetFun(int i, string s, Node** head_ref);
float sin_fun(string s, float t);     // sin function, only for sin(ft+phase)
float cos_fun(string s, float t);      // cos function, only for cos(ft+phase)
float multi_fun(string before, string before_type, string after, string after_type, float t);


// function definitions
void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(string message);
void  PrintUsage();
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();
int   ProfilePositionMode(HANDLE p_DeviceHandle, HANDLE p_DeviceHandle_r, unsigned short p_usNodeId, unsigned short p_usNodeId_r, unsigned int & p_rlErrorCode);
int   run(unsigned int* p_pErrorCode);
int   PreparePPM(unsigned int* p_pErrorCode);

void PrintUsage(){
	cout << "Usage: PITCH Control" << endl;
	cout << "\t-h   : this help" << endl;
	cout << "\t-n   : node id (default 1)" << endl;
	cout << "\t-d   : device name (EPOS2, EPOS4, default - EPOS4)"  << endl;
	cout << "\t-s   : protocol stack name (MAXON_RS232, CANopen, MAXON SERIAL V2, default - MAXON SERIAL V2)"  << endl;
	cout << "\t-i   : interface name (RS232, USB, CAN_ixx_usb 0, CAN_kvaser_usb 0,... default - USB)"  << endl;
	cout << "\t-p   : port name (COM1, USB0, CAN0,... default - USB0)" << endl;
	cout << "\t-b   : baudrate (115200, 1000000,... default - 1000000)" << endl;
	cout << "\t-l   : list available interfaces (valid device name and protocol stack required)" << endl;
	cout << "\t-r   : list supported protocols (valid device name required)" << endl;
	cout << "\t-v   : display device version" << endl;
}
void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode){
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}
void LogInfo(string message){
	cout << message << endl;
}
void SeparatorLine(){
	const int lineLength = 65;
	for(int i=0; i<lineLength; i++){
		cout << "-";
	}
	cout << endl;
}
void PrintSettings(){
	stringstream msg;

	cout << "default settings:" << endl;
	cout << "node id             = " << g_usNodeId << endl;
	cout << "device name         = '" << g_deviceName << "'" << endl;
	cout << "protocal stack name = '" << g_protocolStackName << "'" << endl;
	cout << "interface name      = '" << g_interfaceName << "'" << endl;
	cout << "port name           = '" << g_portName << "'"<< endl;
	cout << "baudrate            = " << g_baudrate << endl;

	SeparatorLine();

	//Roll
	//----------------------------------------------------------------------

	cout << "default settings:" << endl;
	cout << "node id             = " << g_usNodeId_r << endl;
	cout << "device name         = '" << g_deviceName_r << "'" << endl;
	cout << "protocal stack name = '" << g_protocolStackName_r << "'" << endl;
	cout << "interface name      = '" << g_interfaceName_r << "'" << endl;
	cout << "port name           = '" << g_portName_r << "'"<< endl;
	cout << "baudrate            = " << g_baudrate_r;

	LogInfo(msg.str());

	SeparatorLine();
}
void SetDefaultParameters(){
	//USB
	g_usNodeId = 1;
	g_deviceName = "EPOS4";
	g_protocolStackName = "MAXON SERIAL V2";
	g_interfaceName = "USB";
	g_portName = "USB0";
	g_baudrate = 1000000;

	//Roll
	//-----------------------------------------------------

	g_usNodeId_r = 1;
	g_deviceName_r = "EPOS4";
	g_protocolStackName_r = "MAXON SERIAL V2";
	g_interfaceName_r = "USB";
	g_portName_r = "USB1";
	g_baudrate_r = 1000000;

}
int OpenDevice(unsigned int* p_pErrorCode){
	int lResult_p = MMC_FAILED;
	int lResult = MMC_SUCCESS;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	LogInfo("Open device...");

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(g_pKeyHandle!=0 && *p_pErrorCode == 0){
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0){
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0){
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0){
					if(g_baudrate==(int)lBaudrate){
						lResult_p = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else{
		g_pKeyHandle = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

    //Roll
	//------------------------------------------------------------------------------------------------------------------

	int lResult_r = MMC_FAILED;

	char* pDeviceName_r = new char[255];
	char* pProtocolStackName_r = new char[255];
	char* pInterfaceName_r = new char[255];
	char* pPortName_r = new char[255];

	strcpy(pDeviceName_r, g_deviceName_r.c_str());
	strcpy(pProtocolStackName_r, g_protocolStackName_r.c_str());
	strcpy(pInterfaceName_r, g_interfaceName_r.c_str());
	strcpy(pPortName_r, g_portName_r.c_str());

	LogInfo("Open device...");

	g_pKeyHandle_r = VCS_OpenDevice(pDeviceName_r, pProtocolStackName_r, pInterfaceName_r, pPortName_r, p_pErrorCode);

	if(g_pKeyHandle!=0 && *p_pErrorCode == 0){
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle_r, &lBaudrate, &lTimeout, p_pErrorCode)!=0){
			if(VCS_SetProtocolStackSettings(g_pKeyHandle_r, g_baudrate_r, lTimeout, p_pErrorCode)!=0){
				if(VCS_GetProtocolStackSettings(g_pKeyHandle_r, &lBaudrate, &lTimeout, p_pErrorCode)!=0){
					if(g_baudrate_r==(int)lBaudrate){
						lResult_r = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else{
		g_pKeyHandle = 0;
	}

	delete []pDeviceName_r;
	delete []pProtocolStackName_r;
	delete []pInterfaceName_r;
	delete []pPortName_r;

    if(lResult_p == MMC_FAILED || lResult_r == MMC_FAILED){
        int lResult = MMC_FAILED;
    }

	return lResult;
}
int CloseDevice(unsigned int* p_pErrorCode){
	int lResult_p = MMC_FAILED;
	int lResult = MMC_SUCCESS;

	*p_pErrorCode = 0;

	LogInfo("Close device");

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0){
		lResult_p = MMC_SUCCESS;
	}

	//Roll
	//-----------------------------------------------------------------------------------

	int lResult_r = MMC_FAILED;

	*p_pErrorCode = 0;

	LogInfo("Close device");

	if(VCS_CloseDevice(g_pKeyHandle_r, p_pErrorCode)!=0 && *p_pErrorCode == 0){
		lResult_r = MMC_SUCCESS;
	}

	if(lResult_p == MMC_FAILED || lResult_r == MMC_FAILED){
        lResult = MMC_FAILED;
    }

	return lResult;
}
int PreparePPM(unsigned int* p_pErrorCode){
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0){
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0){
		if(oIsFault){
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0){
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0){
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0){
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0){
				if(!oIsEnabled){
					if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0){
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}

	//Roll
	//--------------------------------------------------------------------------------------------

	if(VCS_GetFaultState(g_pKeyHandle_r, g_usNodeId_r, &oIsFault, p_pErrorCode ) == 0){
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0){
		if(oIsFault){
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId_r << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId_r, p_pErrorCode) == 0){
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0){
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle_r, g_usNodeId_r, &oIsEnabled, p_pErrorCode) == 0){
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0){
				if(!oIsEnabled){
					if(VCS_SetEnableState(g_pKeyHandle_r, g_usNodeId_r, p_pErrorCode) == 0){
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}

	return lResult;
}
int ProfilePositionMode(HANDLE p_DeviceHandle, HANDLE p_DeviceHandle_r, unsigned short p_usNodeId, unsigned short p_usNodeId_r, unsigned int & p_rlErrorCode){
    Node_float* last_p;
    Node_float* last_r;
    long roll;
    long pitch;
    int lResult = MMC_SUCCESS;
    unsigned int pitchError;
    unsigned int rollError;
    stringstream msg;
	
	if((VCS_ActivatePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)||(VCS_ActivatePositionMode(p_DeviceHandle_r, p_usNodeId_r, &p_rlErrorCode) == 0)){
		LogError("VCS_ActivatePositionMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	
	else{

	    // while we are not at the end of the ref signal
	    while(head_roll_ref->next!= NULL || head_pitch_ref->next != NULL){
		pitch = head_pitch_ref->pos;
		roll = head_roll_ref->pos;
		
		if(VCS_SetPositionMust(p_DeviceHandle, p_usNodeId, pitch, &p_rlErrorCode) == 0){
		    LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
		    lResult = MMC_FAILED;
		    break;
		}
		VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &actualposition_p, &p_rlErrorCode);
		cout << "Desired Pitch Position: " << long(head_pitch_ref->pos) << endl;
		cout << "Actual Pitch Position: " << actualposition_p << endl;
		
		if(VCS_SetPositionMust(p_DeviceHandle_r, p_usNodeId_r, roll, &p_rlErrorCode) == 0){
		    LogError("VCS_SetPositionMust", lResult, p_rlErrorCode);
		    lResult = MMC_FAILED;
		    break;
		}
		//VCS_GetPositionIs(p_DeviceHandle_r, p_usNodeId_r, &actualposition_r, &p_rlErrorCode);
		//cout << "Desired Roll Position: " << long(head_roll_ref->pos) << endl;		
		//cout << "Actual Roll Position: " << actualposition_r << endl;

		// increase the position were looking at in the refrence signals in both the pitch and roll signals
		last_p = head_pitch_ref;
		head_pitch_ref = head_pitch_ref->next;
		last_r = head_roll_ref;
		head_roll_ref = head_roll_ref->next;

	    }
	    
	    	// for the pitch - if we get an error with the pitch then it logs the error and stops it
	    if(lResult == MMC_SUCCESS){
		LogInfo("halt pitch position movement");

		if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0){
		    LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
		    lResult = MMC_FAILED;
		}
	    }
		// for the roll - if we get an error with the roll then it logs the error and stops it
	    if(lResult == MMC_SUCCESS){
		LogInfo("halt roll position movement");

		if(VCS_HaltPositionMovement(p_DeviceHandle_r, p_usNodeId_r, &p_rlErrorCode) == 0){
		    LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
		    lResult = MMC_FAILED;
		}
	    }
	}
	return lResult;
}

int run(unsigned int* p_pErrorCode){
    int lResult = MMC_SUCCESS;
    unsigned int lErrorCode = 0;

    lResult = ProfilePositionMode(g_pKeyHandle, g_pKeyHandle_r, g_usNodeId, g_usNodeId_r, lErrorCode);
    
    if(lResult != MMC_SUCCESS){
	    LogError("ProfilePositionMode", lResult, lErrorCode);
    }
    else{
	if((VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)||(VCS_SetDisableState(g_pKeyHandle_r, g_usNodeId_r, &lErrorCode) == 0)){
		LogError("VCS_SetDisableState", lResult, lErrorCode);
		lResult = MMC_FAILED;
	}
    }

	return lResult;
}

// ejaiden code



void UserInput(){


    printf("For entering desired angular position equations...\n1. If one element is 0 then enter 0\n2. Enter equation in degrees\n3. NO division, only decimals");
    printf("Format: X(t) = A*f(w*t+p)\nExample: X(t) = 5*sin(15*t+90)+0.3*cos(10*t+0)-10*sin(2*t+0)+sin(0*t-90)\n ");

    printf("\nFor entering time to run...\n1. max time is 30sec, will auto stop at 30sec\n2. Run time is precise to the milisecond\n");
    printf("Format: t = t\nExample 1: X(t) = 5\nExample 2: X(t) = 1.234\n ");

    EnterPitch();
    sleep(1);
    EnterRoll();

    EnterTime();
    DoubleCheck();

    return;

}
string NormalizeString(string s){
    // removing all spaces from string
    s.erase(remove(s.begin(), s.end(), ' '), s.end());

    // converting the string to all lowercase
    for (int i = 0; i < s.length(); i++){
  		s[i] = tolower(s[i]);
  	}

    return s;
}
void EnterPitch(){
    cout << "\nEnter desired angular position equation for the pitch axis: X(t) = ";
    getline(cin,pitch_eq);

    string correct;
    cout << "You entered:\n X(t) = " << pitch_eq << "\nIs this correct? (Y/N): ";
    cin >> correct;
    correct = NormalizeString(correct);
    // if user entered y, yes it is correct, move on. if user entered n, no, then call enter_pitch again
    if (correct == "y"){
        pitch_eq = NormalizeString(pitch_eq);
        return;
    }
    else {
	cin.ignore();
        EnterPitch();
    }

    return;
}
void EnterRoll(){
    cout << "\nEnter desired angular position equation for the roll axis: X(t) = ";
    cin.ignore();
    getline(cin,roll_eq);

    string correct;
    cout << "You entered:\n X(t) = " << roll_eq << "\nIs this correct? (Y/N): ";
    cin >> correct;
    correct = NormalizeString(correct);
    // if user entered y, yes it is correct, move on. if user entered n, no, then call enter_pitch again
    if (correct == "y"){
        roll_eq = NormalizeString(roll_eq);
        return;
    }
    else {
        EnterRoll();
    }

    return;
}
void EnterTime(){
    string temptime;
    cout << "\nEnter desired time to run [sec] :";
    cin.ignore();
    getline(cin,temptime);

    string correct;
    cout << "You entered:\n t = " << temptime << "\nIs this correct? (y/n): ";
    cin >> correct;
    correct = NormalizeString(correct);
    // if user entered y, yes it is correct, move on. if user entered n, no, then call enter_roll again
    if (correct == "y"){
        time2run = stof(temptime); // converting string to float
        return;
    }
    else {
        EnterTime();
    }

    return;
}
void DoubleCheck(){
    cout << "\n\nFor the desired pitch equation you entered: " << pitch_eq;
    cout << "\nFor the desired roll equation you entered: " << roll_eq;
    cout << "\nFor the desired time to run you entered: " << time2run << " seconds";

    string correct;
    cout << "\nIs this correct? (y/n): ";
    cin >> correct;
    correct = NormalizeString(correct);
    // if user entered y, yes it is correct, move on. if user entered n, no, then call enter_roll again
    if (correct == "y"){
        return;
    }
    else {
        UserInput(); // starting over
    }

    return;
}
float RollCompensate(float x, float y){         // x is pitch position, y is roll desired given

    float roll_desired;

    // 5th order polynomial see final project report A.20
    roll_desired = (-.0008393) + (1.009*pow(10,-19)*x) + (.9952*y) +(6.319*pow(10, -70)*pow(x,2)) + (1.0807*pow(10,-17)*x*y) \
                   -(7.387*pow(10,-7)*pow(y,2)) - (8.825*pow(10,-20)*pow(x,3)) - (.0001228*pow(x,2)*y) +(2.516*pow(10,-21)*x*pow(y,2)) \
                   -(3.427*pow(10,-6)*pow(y,3)) + (5.232*pow(10,-11)*pow(x,4)) - (6.849*pow(10,-21)*pow(x,3)*y) \
                   -(4.232*pow(10,-10)*pow(x,2)*pow(y,2)) + (3.023*pow(10,-22)*x*pow(y,3)) + (3.698*pow(10,-10)*pow(y,4)) \
                   +(1.418*pow(10,-23)*pow(x,5)) - (6.228*pow(10,-9)*pow(x,4)*y) + (1.968*pow(10,-24)*pow(x,3)*pow(y,2)) \
                   +(1.585*pow(10,-8)*pow(x,2)*pow(y,3)) - (1.816*pow(10,-24)*x*pow(y,4)) + (1.564*pow(10,-9)*pow(y,5));

    return roll_desired;
}
void ParseToDLL(Node** head_ref, string s){
    string ch;
    string next;
    string n = "not null";
    for(int i=0; i < s.length();i++){     // looking at each charater
        ch = s.at(i);

        if( i == s.length()- 1){ // to avoid out of range error when assigning next
            n ="null";
        }
        else{
            next = s.at(i+1);
        }


        if (n == "null"){ // first checks if were already at the end of the string eq
            cout << ch << " is end of string eq";

            if(isNumber(ch)){
                string num;
                num = GetNum(i,s);
                append(head_ref,num, "num", "pos");

            }
            else if (isParam(ch)){
                append(head_ref,ch, "param", "na");
            }
            else{
                cout << "UnkNOwn....... in parse ddl";
            }

            return;
        }

        else if(isOperator(ch)){
            // then we are at start
            if(ch == "*" && next =="-"&& ch!="-"){ // ch!== - is for when the start thing is neg
                string next_next;
                next_next = s.at(i+2); // need to add check at end
                append(head_ref,ch,"op","na");
                if (isNumber(next_next)){
                    string num;
                    num = GetNum(i+2,s);
                    append(head_ref,"-"+num, "num", "neg");
                    i = i + num.length();
                }
                else if(isParam(next_next) || isFunction(next_next)){
                    append(head_ref,"-1", "num", "neg");
                    append(head_ref,"*", "op", "na");
                }

                i=i+1;
            }
            else if(i== 0 && ch == "-"){ //the start is negative thing
                if (isNumber(next)){
                    string num;
                    num = GetNum(i+1,s);
                    append(head_ref,"-"+num, "num", "neg");
                    i = i + num.length();
                }
                else if(isParam(next) || isFunction(next)){
                    append(head_ref,"-1", "num", "neg");
                    append(head_ref,"*", "op", "na");
                }
            }
            else{
                append(head_ref,ch,"op","na");
            }
        }

        else if(isNumber(ch)){
            string num;
            num = GetNum(i,s);
            i = i + num.length()-1;
            append(head_ref,num, "num", "pos");

        }

        // only works with function forms of sin(x*t+/-phase)
        else if(isFunction(ch)){
            string fun;
            fun = GetFun(i,s,head_ref);
            append(head_ref,fun, "fun", "na");
            while (ch!= ")"){
                i=i+1;
                ch = s.at(i);
            }
            fun = "";
        }
        else if(isParam(ch)){
             append(head_ref,ch, "param", "na");
        }
        else{
            cout << ch << " is invalid char" << endl;
        }

    }

    return;
}
bool isOperator(string s){                      // Operators are; +,-,(,),^,and *
    if (s == "+" || s == "-" || s == "^" || s == "(" || s == ")" || s == "*"){
        return true;
    }
    else{
        return false;
    }
}
bool isNumber(string s){                        // Numbers are 0,1,2,3,4,5,6,7,8,9, and e
    if (s == "0" || s == "1" || s == "2" || s == "3" || s == "4" || s == "5" || s == "6" || s == "7" || s == "8" || s == "9" || s =="p" ){
        return true;
    }
    else{
        return false;
    }
}
bool isFunction(string s){
    if (s == "e" || s == "s" || s == "c"){
        return true;
    }
    else{
        return false;
    }
}
bool isParam(string s){
    if (s == "t"){
        return true;
    }
    else{
        return false;
    }

}
float sin_fun(string s, float t){
    float ans;
    float t_mulit;
    float temp;
    float phase;
    string ch;
    string phase_str;

    // string looks like sin("number",t,"number ")
    int i = 4;
    string first_num = GetNum(i,s); // i is now at number
    t_mulit = stof(first_num);
    //------------ 4/18/2022------------------------------------- with sampling freq 200HZ--------
    t_mulit =2*t_mulit;
    //--------------------------------------------------------------------------------------------
    i = i + first_num.length();        // i is past the second , as t will never be negativer
    ch = s.at(i);
    while(ch != "," ){
        if( ch == "*"){
            // get the second numbre and multiply it with the first
            string temp;
            temp = GetNum(i+1,s); // i is now at next number
            i = i + temp.length()+1;
            if(temp == "pi"){
                t_mulit = t_mulit * pi;
            }
            else {
                t_mulit = t_mulit * stof(temp);
            }
            ch = s.at(i);
        }
    } // now looing to see if its been multiplied----------------------------------------------string phase_str;

    i= i+3;
    ch =s.at(i);
    phase_str = GetNum(i,s);
    if(phase_str =="pi"){
        phase = pi;
    }
    else {
        phase = stof(phase_str);
    }

    return sin((t_mulit*t) + phase); // rad

}
float cos_fun(string s, float t){
    float ans;
    float t_mulit;
    float temp;
    float phase;
    string ch;
    string phase_str;

    // string looks like sin("number",t,"number ")
    int i = 4;
    string first_num = GetNum(i,s); // i is now at number
    t_mulit = stof(first_num);
    //------------ 4/18/2022------------------------------------- with sampling freq 200HZ--------
    t_mulit =2*t_mulit;
    //--------------------------------------------------------------------------------------------
    i = i + first_num.length();        // i is past the second , as t will never be negativer
    ch = s.at(i);
    while(ch != "," ){
        if( ch == "*"){
            // get the second numbre and multiply it with the first
            string temp;
            temp = GetNum(i+1,s); // i is now at next number
            i = i + temp.length()+1;
            if(temp == "pi"){
                t_mulit = t_mulit * pi;
            }
            else {
                t_mulit = t_mulit * stof(temp);
            }
            ch = s.at(i);
        }
    } // now looing to see if its been multiplied-----

    i= i+3;
    ch =s.at(i);
    phase_str = GetNum(i,s);
    if(phase_str =="pi"){
        phase = pi;
    }
    else {
        phase = stof(phase_str);
    }

    return cos((t_mulit*t) + phase); // deg

}

float multi_fun(string before, string before_type, string after, string after_type, float t){
    float b = 0;
    float a = 0;

    b = StringDecoder(before,t, before_type);
    a = StringDecoder(after, t, after_type);

    return a*b;
}                         //
string GetNum(int i, string s){
    string number;
    string curr;
    string next;
    curr = s.at(i);
    if(curr == "-"){// if the number passed is negative
        i = i +1;
        curr = s.at(i);
        number = "-";
    }
    if (i != s.length()-1){
        next = s.at(i+1);

    }

    if (curr == "p"){
        return "pi";
    }
    if(curr == "0" && next !="." ){
        return curr;
    }
    while(isNumber(curr) || curr == "." ){
        if(isNumber(curr)){
            number = number +curr;
            i = i+1;
            if (i == s.length()){
                return number;
            }
            else{
                curr = s.at(i);
                if (i != s.length()-1){
                    next = s.at(i+1);
                }
            }
        }
        else  if(curr == "."){
            number = number+".";
            i = i+1;
            curr = s.at(i);
            if (i != s.length()-1){
                next = s.at(i+1);
            }
        }
    }
    return number;
}
string GetFun(int i, string s, Node** head_ref){ // need to fix this so we find all numbers in front then make one number,
    string fun = "";
    string ch;
    string next;
    ch = s.at(i);
    if(ch == "c" || ch =="s"){
        fun = ch + s.at(i+1) + s.at(i+2) + s.at(i+3); // ie sin(
        i = i+4;
        ch = s.at(i);
        next = s.at(i+1);
        string prev;
        if (i >= 2){
            prev = s.at(i-1);
        }
        while(ch !=")"){        // creating function in one block
            if (ch == "-"){ // if neg num
                if (prev == "t"){
                    fun = fun + ",";
                }
                fun = fun +"-";
                string num;
                num = GetNum(i+1,s);
                i = i + num.length() +1;
                ch = s.at(i);
                if (i == s.length()-1){
                    fun = fun +num;
                    break;
                }
                else{
                    next = s.at(i+1);
                }
                prev = s.at(i-1);
                fun = fun +num;
            }
            else if (prev == "(" && ch == "t" && isOperator(next)){ // if just t
                fun = fun +"1,t,";
                i = i +2;
                ch = s.at(i);
                prev = s.at(i-1);

                if (i == s.length()-1){
                    break;
                }
                else{
                    next = s.at(i+1);
                }

            }
            else if (isNumber(ch)){ // if pos num
                string num;
                num = GetNum(i,s);
                i = i + num.length();
                fun = fun +num;
                ch = s.at(i);
                prev = s.at(i-1);

                if (i == s.length()-1){
                    break;
                }
                else{
                    next = s.at(i+1);
                }
            }
            else if (isOperator(ch)){
                if (ch == "*" && isNumber(next) ){ // if two numbers are multiplied with eachother like 2*pi, then we put that in the same slot
                    fun = fun + "*";
                    string num;
                    num = GetNum(i+1,s);
                    i = i + num.length()+1;
                    fun = fun +num;
                    ch = s.at(i);
                    prev = s.at(i-1);

                    if (i == s.length()-1){
                        break;
                    }
                    else{
                        next = s.at(i+1);
                    }
                }
                else {
                    fun = fun +",";
                    i=i+1;
                    ch = s.at(i);
                    prev = s.at(i-1);
                    next = s.at(i+1);
                }

            }
            else if (isParam(ch)){ // if t
                fun = fun +"t";
                i=i+1;
                ch = s.at(i);
                prev = s.at(i-1);
                if (i == s.length()-1){
                    break;
                }
                else{
                    next = s.at(i+1);
                }
            }
        }
    }
    else if(ch == "e"){//---------------------------------------------fix this
        fun = ch + s.at(i+1) + s.at(i+2); // ie e^(
        i = i+3;
        ch = s.at(i);
        next = s.at(i+1);
        string prev;
        if (i >= 2){
            prev = s.at(i-1);
        }
        string insideE = "";
        while(ch !=")"){
            ch = s.at(i);
            prev = s.at(i-1);

            if (i == s.length()-1){
                break;
            }
            else{
                next = s.at(i+1);
            }
        }


    }
    fun = fun + ")";
    return fun;
}

void CreateRoll(){
    ParseToDLL(&head_r,roll_eq);
   // PrintLL(head_r); //debug purposes
    return;
}
void CreateSignalRoll(){                        // coupling for roll
    float curr_time = 0;
    float roll_pos;
    float pitch_pos;
    float roll_comp_pos;


    while (curr_time <= time2run){
        // add desired position to ref signal
        pitch_pos = DLLDecoder(head_p,curr_time); // did this correctly
        roll_pos = DLLDecoder(head_r,curr_time);
        roll_comp_pos = RollCompensate(pitch_pos,roll_pos);
	roll_comp_pos = roll_comp_pos*r_multip;
        append_f(&head_roll_ref,long(roll_comp_pos),curr_time);
        curr_time = curr_time + sample_time; // increment by 1 ms, 1000Hz
    }

    return;
}

void CreatePitch(){
    ParseToDLL(&head_p,pitch_eq);
    //PrintLL(head_p); // decodign purposes
    return;
}
void CreateSignalPitch(){                       // no coupling for pitch
    float curr_time = 0;
    float curr_pos;

    while (curr_time <= time2run){
        curr_pos = DLLDecoder(head_p,curr_time);
	curr_pos = curr_pos*p_multip;
        append_f(&head_pitch_ref,long(curr_pos),curr_time);
        curr_time = curr_time + sample_time; // increment by 1 ms, 1000Hz
    }

    return;
}

float StringDecoder(string s,float t, string type){
    string ch;
    ch = s.at(0);
    if (type == "num"){
        if (s =="pi"){
            return pi;
        }
        else {
            float num = stof(s);
            return num;
        }
    }
    else if (type == "param"){
        return t;
    }
    else if (type == "fun"){
        if (ch == "c"){ // cos
            float num = cos_fun(s,t);
            return num;
        }
        else if (ch == "s"){ // sin
            float num = sin_fun(s,t);
            return num;
        }
        // need to add e

    }
    else {
        cout << "Decodeing operatior.. not for this function , in string decoder " << s;
        return 0;
    }
    cout << "howd we get here";
    return 0;
}
float DLLDecoder(Node* head_ref,float t){   // t is the current time
    float fin = 0;
    Node* last;
    // will need to check if next is null tho
    string tmpdata = head_ref->data;
    string tmptype = head_ref->type;
    Node* tmpnext = head_ref->next;
    Node* tmpprevi = head_ref->previ;

    // if were at the last node
    while (tmpnext != NULL){

// multiplying shenanganiz ------------------------------------------------------
        if (tmpnext->data=="*"){
            Node* start = tmpnext->previ->previ; // was two previ
            float multi_fin_loop = 1;
            float multi_fin = 1;
            float loopcheck =0;
            if(tmpnext->next->next != NULL ){ // more then 2 multiplied in a row
               while(tmpnext != NULL && tmpnext->next->next != NULL && tmpnext->next->next->data =="*"){
                    loopcheck =1;
                    float n;
                    n = multi_fun(tmpdata,tmptype, tmpnext->next->data,tmpnext->next->type, t);
                    multi_fin_loop = multi_fin_loop *n;
                        // increase node position by j
                        for (int j =1; j<= 4; j++){
                            last = head_ref;
                            head_ref = head_ref->next;
                            tmpdata = head_ref->data;
                            tmptype = head_ref->type;
                            tmpprevi = head_ref->previ;
                            tmpnext = head_ref->next;
                        }
                }
            }
            if (loopcheck== 1){
                string restult;
                restult = to_string(multi_fin_loop);
                multi_fin =  multi_fun(restult,"num", tmpdata,tmptype, t);
                if(start == NULL){
                    fin = multi_fin;
                    break;
                }
                else if (start->data =="-"){
                    fin = fin - multi_fin;
                }
                else if (start->data =="+"){
                    fin = fin + multi_fin;
                }
                else{
                    fin = multi_fin;
                }

                if(tmpnext == NULL){
                    return fin;
                }
                // increase node position by j
                for (int j =1; j<= 2; j++){
                    last = head_ref;
                    head_ref = head_ref->next;
                    tmpdata = head_ref->data;
                    tmptype = head_ref->type;
                    tmpprevi = head_ref->previ;
                    tmpnext = head_ref->next;
                }
            }
            else{
                multi_fin =  multi_fun(tmpdata,tmptype, tmpnext->next->data,tmpnext->next->type, t);
                if(start == NULL){
                    fin = multi_fin;
                }
                else if (start->data =="-"){ // subtracting
                    fin = fin - multi_fin;
                }
                else if (start->data =="+"){ //  adding
                    fin = fin + multi_fin;
                }
                else{ // was at start
                    fin = multi_fin;
                }
                if (tmpnext->next->next !=NULL){
                    // increase node position by j
                    for (int j =1; j<= 3; j++){
                        last = head_ref;
                        head_ref = head_ref->next;
                        tmpdata = head_ref->data;
                        tmptype = head_ref->type;
                        tmpprevi = head_ref->previ;
                        tmpnext = head_ref->next;
                    }
                }
                else{
                    // increase node position by j
                    for (int j =1; j<= 2; j++){
                        last = head_ref;
                        head_ref = head_ref->next;
                        tmpdata = head_ref->data;
                        tmptype = head_ref->type;
                        tmpprevi = head_ref->previ;
                        tmpnext = head_ref->next;
                    }
                }
            }

        }
//-------------------------------------------------------------------------------
        else if(tmptype == "num"){ // current node is a number
            if(tmpprevi == NULL){ // this means were at the begining and the start was a pos number
                float num = StringDecoder(tmpdata,t,tmptype);
                fin = num;
                // increase node position by j
                for (int j =1; j<= 1; j++){
                    last = head_ref;
                    head_ref = head_ref->next;
                    tmpdata = head_ref->data;
                    tmptype = head_ref->type;
                    tmpprevi = head_ref->previ;
                    tmpnext = head_ref->next;
                }
            }
            else if(tmpnext->data =="*"){ // if the current number is being multiplied with stuffs
              break;
            }
            else if (tmpprevi->data == "-"){ // minus number
                float num = StringDecoder(tmpdata,t,tmptype);
                fin = fin - num;
                // increase node position by j
                for (int j =1; j<= 1; j++){
                    last = head_ref;
                    head_ref = head_ref->next;
                    tmpdata = head_ref->data;
                    tmptype = head_ref->type;
                    tmpprevi = head_ref->previ;
                    tmpnext = head_ref->next;
                }
            }
            else if (tmpprevi->data == "+"){ // negative number
                float num = StringDecoder(tmpdata,t,tmptype);
                fin = fin + num;
                // increase node position by j
                for (int j =1; j<= 1; j++){
                    last = head_ref;
                    head_ref = head_ref->next;
                    tmpdata = head_ref->data;
                    tmptype = head_ref->type;
                    tmpprevi = head_ref->previ;
                    tmpnext = head_ref->next;
                }
            }

        }
        else if (tmptype == "op"){ // current node is an operator
            if (tmpprevi == NULL){ // This means were at the beinging and the start was likly a negative number
                if(tmpnext->type=="num"){
                    if(tmpdata=="-"){
                        // increase node position by j
                        for (int j =1; j<= 1; j++){
                            last = head_ref;
                            head_ref = head_ref->next;
                            tmpdata = head_ref->data;
                            tmptype = head_ref->type;
                            tmpprevi = head_ref->previ;
                            tmpnext = head_ref->next;
                        }
                        float neg_beg;
                        neg_beg = DLLDecoder(head_ref,t);
                        return neg_beg;
                    }
                }
            }
            else if(tmpnext->next->data =="*"){
                // increase node position by j
                if(tmpnext->next!=NULL){
                    for (int j =1; j<= 1; j++){
                        last = head_ref;
                        head_ref = head_ref->next;
                        tmpdata = head_ref->data;
                        tmptype = head_ref->type;
                        tmpprevi = head_ref->previ;
                        tmpnext = head_ref->next;
                    } // we dont want to break out of the loop we just want to be done here so incremnet by one

                }
            }
            else if(tmpdata =="-"){
                float n;
                n =StringDecoder(tmpnext->data, t, tmpnext->type );
                fin = fin - n;
                // increase node position by j
                if(tmpnext->next!=NULL){
                    for (int j =1; j<= 2; j++){
                        last = head_ref;
                        head_ref = head_ref->next;
                        tmpdata = head_ref->data;
                        tmptype = head_ref->type;
                        tmpprevi = head_ref->previ;
                        tmpnext = head_ref->next;
                    }
                }
                else{
                    for (int j =1; j<= 1; j++){
                        last = head_ref;
                        head_ref = head_ref->next;
                        tmpdata = head_ref->data;
                        tmptype = head_ref->type;
                        tmpprevi = head_ref->previ;
                        tmpnext = head_ref->next;
                    }
                }
            }
            else if(tmpdata == "+"){
                fin = fin + StringDecoder(tmpnext->data, t, tmpnext->type );
                // increase node position by j
                if(tmpnext->next!=NULL){
                    for (int j =1; j<= 2; j++){
                        last = head_ref;
                        head_ref = head_ref->next;
                        tmpdata = head_ref->data;
                        tmptype = head_ref->type;
                        tmpprevi = head_ref->previ;
                        tmpnext = head_ref->next;
                    }
                }
                else{
                    for (int j =1; j<= 1; j++){
                        last = head_ref;
                        head_ref = head_ref->next;
                        tmpdata = head_ref->data;
                        tmptype = head_ref->type;
                        tmpprevi = head_ref->previ;
                        tmpnext = head_ref->next;
                    }
                }
            }
            else{
                cout << "you donked it, in decoder else if op not + or -"; // its getting "/ for some reason"
                exit(1);
            }

            // others


        }
        else if (tmptype == "fun"){ // curent node is a function
            if (tmpprevi==NULL){ // This means were at the beinging and the start was a functiton with no scaler
                fin = StringDecoder(tmpdata,t,tmptype);
            }
            // others
            else if(tmpprevi->data == "+"){
                fin = fin +StringDecoder(tmpdata,t,tmptype);
            }
            else if (tmpprevi->data == "-"){
                fin = fin - StringDecoder(tmpdata,t,tmptype);
            }

            for (int j =1; j<= 1; j++){
                last = head_ref;
                head_ref = head_ref->next;
                tmpdata = head_ref->data;
                tmptype = head_ref->type;
                tmpprevi = head_ref->previ;
                tmpnext = head_ref->next;
            }
        }
        else if (tmptype == "param"){ // curent node is a param, so t
            if (tmpprevi==NULL){ // This means were at the beinging and the start was t so a linear fun
                if(tmpdata=="t"){
                    fin = t;
                    // increase node position by j
                    for (int j =1; j<= 1; j++){
                        last = head_ref;
                        head_ref = head_ref->next;
                        tmpdata = head_ref->data;
                        tmptype = head_ref->type;
                        tmpprevi = head_ref->previ;
                        tmpnext = head_ref->next;
                    }
                }
            }
            else if (tmpprevi->type=="op"){ // means were either +,-, or * with t (thats all the ops coded for so far)
                if(tmpprevi->data == "+" && (tmpnext->data == "+" || tmpprevi->data=="-")){
                    fin = fin + t;
                    // increase node position by j
                    for (int j =1; j<= 2; j++){
                        last = head_ref;
                        head_ref = head_ref->next;
                        tmpdata = head_ref->data;
                        tmptype = head_ref->type;
                        tmpprevi = head_ref->previ;
                        tmpnext = head_ref->next;
                    }
                }
                else if(tmpprevi->data == "-" && (tmpnext->data == "+" || tmpprevi->data=="-")){
                    fin = fin - t;
                    //moving two node ahead, because we always check prev, one node ahead is the next prev
                    // increase node position by j
                    for (int j =1; j<= 2; j++){
                        last = head_ref;
                        head_ref = head_ref->next;
                        tmpdata = head_ref->data;
                        tmptype = head_ref->type;
                        tmpprevi = head_ref->previ;
                        tmpnext = head_ref->next;
                    }
                }
            }

            // other
        }
        else{
            cout << endl << "WARNING !!!!!!!!!!!!!!!!!!!!!!!!!!" << endl << "UNKNOWN: " << tmpdata;
            // increase node position by j
            for (int j =1; j<= 1; j++){
                last = head_ref;
                head_ref = head_ref->next;
                tmpdata = head_ref->data;
                tmptype = head_ref->type;
                tmpprevi = head_ref->previ;
                tmpnext = head_ref->next;
            }
        }
    }
    // do last node here
    if(tmpprevi == NULL){
        fin = StringDecoder(tmpdata,t,tmptype);
    }


    return fin;
}


int main(int argc, char** argv){
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	sleep(5);

	//UserInput();
	CreatePitch();
	CreateRoll();
	CreateSignalPitch();
	CreateSignalRoll();
	//cout << "printing roll positions:\n";
	//Print_pos(head_roll_ref);
	//cout << "printing pitch positions:\n";
	//Print_pos(head_pitch_ref);

	SetDefaultParameters();

	PrintSettings();

	if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS){
		LogError("OpenDevice", lResult, ulErrorCode);
		return lResult;
	}

	if((lResult = PreparePPM(&ulErrorCode))!=MMC_SUCCESS){
	    LogError("PreparePPM", lResult, ulErrorCode);
	    return lResult;
	}

	if((lResult = run(&ulErrorCode))!=MMC_SUCCESS){
	    LogError("run", lResult, ulErrorCode);
	    return lResult;
	}

	if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS){
	    LogError("CloseDevice", lResult, ulErrorCode);
	    return lResult;
	}

	return lResult;
}
