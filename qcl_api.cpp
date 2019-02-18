//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//// MIRcatSDK by Shihao Ran
//// STIM Lab copyright
//// 12/12/2016
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//#include <QtCore/QCoreApplication>
#include <stdio.h>
#include <Windows.h>
#include <MIRcatSDK.h>

#ifdef __cplusplus
extern "C"
{
#endif


#ifdef __cplusplus
}
#endif

int main(int argc, char * argv[]){

	//QCoreApplication a(argc, argv);
    uint16_t major, minor, patch;
    uint32_t ret;
	float minWW, maxWW, pulseRateMax, pulseWidthMax, pulseDutyCycleMax, qclTemp, minTemp, maxTemp, nomTemp;
		uint16_t maxCurPulsed, maxCurCw, tecCur;
    printf( "========================================================\n");
    printf( "Test : API Version\n");
    ret = MIRcatSDK_GetAPIVersion(&major, &minor, &patch);
    printf(" Test Result: ret = %d, major = %d, minor = %d, patch = %d\n", ret, major, minor, patch);
    printf( "========================================================\n");
    printf( "Test : API Initialize\n");
    ret = MIRcatSDK_Initialize();
    printf(" Test Result: ret = %d\n", ret);
    printf( "========================================================\n");
    if (  MIRcatSDK_RET_SUCCESS == ret  )
    {
		printf( "********************************************************\n");
		printf( " Getting some system information ... \n" );
		printf( "********************************************************\n");
		char stringData[24];
		ret = MIRcatSDK_GetModelNumber( stringData, 24 );
		printf( " Test Result: %d, Model Number: %s\n", ret, stringData );
		ret = MIRcatSDK_GetSerialNumber( stringData, 24 );
		printf( " Test Result: %d, Serial Number: %s\n", ret, stringData );

		uint8_t numQcls;
		ret = MIRcatSDK_GetNumInstalledQcls(&numQcls);
		printf( " Test Result: %d, Number of installed QCLs: %u\n", ret, numQcls );

		printf( "********************************************************\n");

		printf( "********************************************************\n");

        printf( "********************************************************\n");
        printf( " Starting Single tune test ... \n");
        printf( "********************************************************\n");
        //
        // Check to see if interlock is on
        // If yes, arm the laser, start a tune
        // operation
        // loop until tune is done
        //
        bool bIlockSet = false;
        bool bKeySwitchSet = false;
		bool IsArmed = false;
		bool atTemp = false;

        printf( "========================================================\n");
        printf( "Test : InterLocked Status\n");
        ret = MIRcatSDK_IsInterlockedStatusSet(&bIlockSet);
        printf(" Test Result: ret = %d bIslockSet = %d\n", ret, bIlockSet);
        printf( "========================================================\n");
        printf( "Test : Key Switch Status\n");
        ret = MIRcatSDK_IsKeySwitchStatusSet(&bKeySwitchSet);
        printf(" Test Result: ret = %d bKeySwitchSet = %d\n", ret, bKeySwitchSet);
        printf( "========================================================\n");
        printf( "Test : Arm Disarm\n");

		ret = MIRcatSDK_IsLaserArmed(&IsArmed);
		printf(" Test Result: ret = %d IsArmed = %d\n", ret, IsArmed);

		if (!IsArmed)
		{
			ret = MIRcatSDK_ArmDisarmLaser();
			printf(" Test Result: ret = %d\n", ret);
			printf( "========================================================\n");
			printf( "Test : Is Laser Armed\n");
		}
                
        while ( !IsArmed )
        {            
            ret = MIRcatSDK_IsLaserArmed(&IsArmed);
            printf(" Test Result: ret = %d IsArmed = %d\n", ret, IsArmed);
            ::Sleep(1000);
        }

		//Wait until at temperature to do any tune/scan
		printf( "========================================================\n");
		printf( "Test : TEC Temperature Status\n");
		ret = MIRcatSDK_AreTECsAtSetTemperature(&atTemp);
		printf("TECs at Temperature: ret = %d atTemp = %d\n", ret, atTemp );

		while ( !atTemp )
		{				
			for( uint8_t i = 1; i <= numQcls; i++ )
			{
				ret = MIRcatSDK_GetQCLTemperature( i, &qclTemp );
				printf(" Test Result: %d, QCL%u Temp: %.3f C\n", ret, i, qclTemp );
				ret = MIRcatSDK_GetTecCurrent( i, &tecCur );
				printf(" Test Result: %d, TEC%u Current: %u mA\n", ret, i, tecCur );
			}

			ret = MIRcatSDK_AreTECsAtSetTemperature(&atTemp);			
			printf("TECs at Temperature: ret = %d atTemp = %d\n", ret, atTemp );
			::Sleep(1000);
		}

        printf( "========================================================\n");
        printf( "Test : Tune to WW 7.50\n");
        ret = MIRcatSDK_TuneToWW(7.50, MIRcatSDK_UNITS_MICRONS, 1);
        printf(" Test Result: ret = %d\n", ret);
        printf( "========================================================\n");
        printf( "Test : isTuned\n");
        bool isTuned = false;
        while(!isTuned)
        {
            ret = MIRcatSDK_IsTuned(&isTuned);
            printf(" Test Result: ret = %d isTuned = %d\n", ret, isTuned);
            ::Sleep(500);
        }
	}
}