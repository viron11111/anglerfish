
/*******************************************************************************
Copyright (c) 1983-2012 Advantech Co., Ltd.
********************************************************************************
THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY INFORMATION
WHICH IS THE PROPERTY OF ADVANTECH CORP., ANY DISCLOSURE, USE, OR REPRODUCTION,
WITHOUT WRITTEN AUTHORIZATION FROM ADVANTECH CORP., IS STRICTLY PROHIBITED. 

================================================================================
REVISION HISTORY
--------------------------------------------------------------------------------
$Log:  $ 
--------------------------------------------------------------------------------
$NoKeywords:  $
*/
/******************************************************************************
*
* Windows Example:
*    AI_AsynOneBufferedAI_TDtp.cpp
*
* Example Category:
*    AI
*
* Description:
*    This example demonstrates how to use Asynchronous One Buffered AI with Trigger Delay
*    to Stop function.
*
* Instructions for Running:
*    1. Set the 'deviceDescription' for opening the device. 
*    2. Set the 'startChannel' as the first channel for scan analog samples  
*    3. Set the 'channelCount' to decide how many sequential channels to scan analog samples.
*    4. Set the 'sampleCount' to decide the capacity of buffer in kernel. 
*    5. Set 'trigger parameters' to decide trigger property. 
* I/O Connections Overview:
*    Please refer to your hardware reference manual.
*
******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "../inc/compatibility.h"
#include "../../../inc/bdaqctrl.h"
#include <fstream>
using namespace Automation::BDaq;
using namespace std;
//-----------------------------------------------------------------------------------
// Configure the following three parameters before running the sample
#define      deviceDescription  L"PCI-1714UL,BID#8"
int32        startChannel = 0;
const int32  channelCount = 1;
const int32  sampleCount =  20000; //2048   // for each channel, to decide the capacity of buffer in kernel.
// Recommend: sampleCount is an integer multiple of intervalCount, and equal to twice or greater

#define		 USER_BUFFER_SIZE    channelCount*sampleCount 
double       Data[USER_BUFFER_SIZE];

// Set trigger parameters
TriggerAction triggerAction = DelayToStop;
ActiveSignal  triggerEdge = FallingEdge;
int           triggerDelayCount = sampleCount/1.25;
double        triggerLevel = -0.09;
double        samplingFrequency = 5000000;  //in Hz

//for trigger1 parameters
TriggerAction trigger1Action = DelayToStop;
ActiveSignal  trigger1Edge = RisingEdge;
int           trigger1DelayCount = 1000;
double        trigger1Level = 2.0;

// set which trigger be used for this demo, trigger0(0) or trigger1(1).
int           triggerUsed = 0;

inline void waitAnyKey()
{
	do{SLEEP(1);} while(!kbhit());
} 

// This class is used to deal with 'DataReady' Event, we should overwrite the virtual function BfdAiEvent.
class StoppedHandler : public BfdAiEventListener
{
public:
	virtual void BDAQCALL BfdAiEvent(void * sender, BfdAiEventArgs * args)
	{
		BufferedAiCtrl * bufferedAiCtrl = (BufferedAiCtrl*)sender;
		printf("Buffered AI data ready !\n");

		int32 channelCountMax = bufferedAiCtrl->getFeatures()->getChannelCountMax();
		int32 chanStart = bufferedAiCtrl->getScanChannel()->getChannelStart();
		int32 chanCount = bufferedAiCtrl->getScanChannel()->getChannelCount();
		bufferedAiCtrl->GetData(args->Count, Data);
		std::cout << "channelCountMax: " << channelCountMax << std::endl;
		std::cout << "chanStart: " << chanStart << std::endl;
		std::cout << "chanCount: " << chanCount << std::endl;
		std::cout << "bufferedAiCtrl: " << bufferedAiCtrl << std::endl;

		int delayCount = 0;
      if (triggerUsed == 0 && bufferedAiCtrl->getFeatures()->getTriggerSupported())
      {
         delayCount = bufferedAiCtrl->getTrigger()->getDelayCount();
         std::cout << "delayCount: " << delayCount << std::endl;
      }else if (triggerUsed == 1 && bufferedAiCtrl->getFeatures()->getTrigger1Supported())
      {
         delayCount = bufferedAiCtrl->getTrigger1()->getDelayCount();
      }
      
		int triggerPointIndex = args->Count/chanCount - delayCount;
		std::cout << "triggerPointIndex: " << triggerPointIndex << std::endl;
		printf("The data count each channel:%d,  trigger point each channel: %d\n\n",args->Count/chanCount,triggerPointIndex );
		// in this example, we show only one sample of each channel's new data
		printf("the first data each channel are:\n");
		ofstream hdata;
		hdata.open ("Data/tap.csv");

		for(int32 i = 0; i < chanCount; ++i)
		{
			//printf("channel %d: %10.6f \n",(i + chanStart)%channelCountMax,Data[i]);   
			for(int z = 0; z < sampleCount; z++){
				hdata << Data[z] << endl; //"%10.6f \n" % Data[z];
				//printf("%10.6f \n" , Data[z]);
			}
			hdata.close();
		}
		printf("completed sample\n");
		// 
	}
};

int main(int argc, char* argv[])
{
	ErrorCode ret = Success;

	// Step 1: Create a 'BufferedAiCtrl' for buffered AI function.
	BufferedAiCtrl * bfdAiCtrl = AdxBufferedAiCtrlCreate();
	//bufferedAiCtrl->setSelectedDevice(devinfo)


	// Step 2: Set the notification event Handler by which we can known the state of operation effectively.
	StoppedHandler onStopped;
	bfdAiCtrl->addStoppedListener(onStopped);



	do
	{
		// Step 3: Select a device by device number or device description and specify the access mode.
		// in this example we use AccessWriteWithReset(default) mode so that we can 
		// fully control the device, including configuring, sampling, etc.
		DeviceInformation devInfo(deviceDescription);
		ret = bfdAiCtrl->setSelectedDevice(devInfo);
		CHK_RESULT(ret);
		ret = bfdAiCtrl->setStreaming(false);// specify the running mode: one-buffered.
		CHK_RESULT(ret);


		// Step 4: Set necessary parameters for Buffered AI operation, 
		// Note: some of operation of this step is optional(you can do these settings via "Device Configuration" dialog).
		ScanChannel* scanChannel = bfdAiCtrl->getScanChannel();
		ret = scanChannel->setChannelStart(startChannel);
		CHK_RESULT(ret);
		ret = scanChannel->setChannelCount(channelCount);
		CHK_RESULT(ret);
		ret = scanChannel->setSamples(sampleCount);
		CHK_RESULT(ret);
		
		ConvertClock * convertClock = bfdAiCtrl->getConvertClock();		
		ret = convertClock->setRate(samplingFrequency);
		CHK_RESULT(ret);

		//Step 5: Trigger parameters setting
      if (triggerUsed == 0)
      {
         Trigger* trigger = bfdAiCtrl->getTrigger();
         if (trigger != NULL)
         {
            ret = trigger->setAction(triggerAction);
            CHK_RESULT(ret);
            ICollection<SignalDrop>*  srcs = bfdAiCtrl->getFeatures()->getTriggerSources();
            ret = trigger->setSource(srcs->getItem(1));
            CHK_RESULT(ret);
            ret = trigger->setDelayCount(triggerDelayCount) ;
            CHK_RESULT(ret);
            ret = trigger->setEdge(triggerEdge);
            CHK_RESULT(ret);
            ret = trigger->setLevel(triggerLevel);
            CHK_RESULT(ret);
         }
         else
         {
            printf("The device can not support trigger function! \n any key to quit.");
            break;
         } 
      } 
      else if(triggerUsed == 1)
      {
         if( bfdAiCtrl->getFeatures()->getTrigger1Supported())
         {
            Trigger* trigger1 = bfdAiCtrl->getTrigger1();
            ret = trigger1->setAction(trigger1Action);
            CHK_RESULT(ret);
            ICollection<SignalDrop>*  srcs = bfdAiCtrl->getFeatures()->getTrigger1Sources(); 
            ret = trigger1->setSource(srcs->getItem(1));
            CHK_RESULT(ret);
            ret = trigger1->setDelayCount(trigger1DelayCount);
            CHK_RESULT(ret);
            ret = trigger1->setEdge(trigger1Edge);
            CHK_RESULT(ret);
            ret = trigger1->setLevel(trigger1Level);
            CHK_RESULT(ret);
         }else {
            printf("The trigger1 is not can not support by the device! \n any key to quit.");
            break;
         }
      }

		// Step 6: prepare the buffered AI. 
		ret = bfdAiCtrl->Prepare();
		CHK_RESULT(ret);

		// Step 7: start Asynchronous Buffered AI, 'Asynchronous' means the method returns immediately
		// after the acquisition has been started. The StoppedHandler's 'BfdAiEvent' method will be called
		// after the acquisition is completed.
		printf("Asynchronous finite acquisition is in progress.\n");
		printf("Please wait... any key to quit !\n\n");
		ret = bfdAiCtrl->Start();
		CHK_RESULT(ret);

		// Step 8: Do anything you are interesting while the device is acquiring data.
		do
		{
			// do something yourself !
			SLEEP(1);
		}while(!kbhit());

		// step 9: stop the operation if it is running.
		ret = bfdAiCtrl->Stop();  
		CHK_RESULT(ret);
	}
	while(false);

	// Step 10: close device, release any allocated resource before quit.
	bfdAiCtrl->Dispose();

	// If something wrong in this execution, print the error code on screen for tracking.
	if(BioFailed(ret))
	{
		printf("Some error occurred. And the last error code is Ox%X.\n", ret);
		waitAnyKey();// wait any key to quit!
	}

	return 0;   
}












