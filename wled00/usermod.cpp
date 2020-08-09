#include "wled.h"
#include <esp_task_wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*
 * This v1 usermod file allows you to add own functionality to WLED more easily
 * See: https://github.com/Aircoookie/WLED/wiki/Add-own-functionality
 * EEPROM bytes 2750+ are reserved for your custom use case. (if you extend #define EEPSIZE in const.h)
 * If you just need 8 bytes, use 2551-2559 (you do not need to increase EEPSIZE)
 * 
 * Consider the v2 usermod API if you need a more advanced feature set!
 */

// uncomment to print task core changes to serial
// #define CORE_MONITOR
// uncomment to print task stack usage to serial every 20 secs
#define STACK_MONITOR

// what pin is being used for the dallas temp probes?
#define ONE_WIRE_BUS 25
// name of the nextion waveform used for plotting air/water data
#define GRAPHNAME "maingraph"
// name of the nextion float control used to display the water temp
#define H20NAME "xWater"
// name of the nextion float control used to display the air temp
#define AIRNAME "xAir"
// name of the nextion float control used to display the delta between air/water temps
#define DELTANAME "xDelta"
// maximum temperature displayed by the graphs
#define MAX_TEMP 40
// minimum temperature displayed by the graphs
#define MIN_TEMP 20
// RGB value for nextion's red
#define NEXTION_RED "63488"
#define NEXTION_CYAN "21855"


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// hardware serial object for talking to nextion
HardwareSerial mySerial(2);

// sensor addresses and notes:
//
// All observations are to a single decimal place, 11bits resolution, rounded as "round(10 * temp) / 10"
// A: 286EEBE5391901FC baseline)
// B: 28ADC9D93919011B moves a bit slower than A.. with rounding, settles at 0.1 to 0.3 higher than A)
// C: 288D1EDE39190124 very close to A, perhaps 0.1 less than A
// D: 284D04F439190111 acts similar to C.. toying with 0.1 less than A
// E: 28D4DECC39190181 closer to B than anything else.. maybe 0.1 less than B depending on the resolution/rounding.
//
// Pairings:  A/C/D are fairly close.  C/D seemed the closest pair.  B/E are closer to each other than anything else.
//

// onewire addresses for thermometers -- used by SensorsTask/SensorsSetup
DeviceAddress airThermometer =   { 0x28, 0x8D, 0x1E, 0xDE, 0x39, 0x19, 0x01, 0x24 };  //C
DeviceAddress waterThermometer = { 0x28, 0x4D, 0x04, 0xF4, 0x39, 0x19, 0x01, 0x11 };  //D
DeviceAddress unknownThermometer = { 0,0,0,0,0,0,0,0 };

// used/populated by NextionReadData()
uint8_t NexRespBuf[100];
// the nextion componentID for the waveform control
uint8_t uGraphID = 0xFF;
// the height of the waveform control
uint8_t uGraphHeight = 0xFF;
// the width of the waveform control
uint8_t uGraphWidth = 0xFF;
// circular buffer for air temperature mappings
uint8_t *pAirGraphData = NULL;
// circular buffer for water temperature mappings
uint8_t *pH2OGraphData = NULL;
// index in pAirGraphData/pH2OGraphData for the next data point
uint8_t uGraphDataIdx = 0;

// task handle for the sensors task
TaskHandle_t hSensorsTask = NULL;
// task handle for the nextion task
TaskHandle_t hNextionTask = NULL;

// pretty-prints a OneWire device address
void printAddress(DeviceAddress deviceAddress);
// sends a string, optionally followed by another string or an optional number (int converted to base10 string), and then appends a trio of 0xFF bytes
bool NextionSendCmd(const char *pszCmd, const int nVal);
bool NextionSendCmd(const char *pszCmd, const char *pszCmd2 = NULL);
// if data is available, reads from the nextion serial device, putting the data in NexRespBuf.
// Stops reading when three 0xFF bytes are read, and returns the number of bytes read NOT 
// including the 3 0xFF bytes.
size_t    NextionReadData(void);
// reads/discards any data available to read from the nextion serial device
void      NextionRxFlush(void);
// queries the nextion device for the value passed as a parameter, returning the discovered
// value.  For example, passing "t0.id" will send "get t0.id" to the nextion device, and
// wait for a formatted numeric response, and return the value of that response.
uint32_t  NextionGetValue(const char* pszValToGet);
// sends a string, appends a trio of 0xFF bytes, and then waits for the specified response
bool      NextionSendAndWait(const char *pszToSend, const char *pszToWait, size_t nToWaitSize);

// starts (and finishes) the process of resending the nextion waveform plotting data
// from the pAirGraphData/pH2OGraphData circular queues.  This actually sends each queue
// twice (once as the main data, once as an offset.)  
void StartRefillGraph(void);
// helper function for StartRefillGraph(), this is called 4 times, once for each queue and
// once for each queue offset.
void RefillGraph(uint8_t *pGraphData, bool bOffset);

// called by SensorsTask one time, sets things up for reading the dallas temp sensors
void SensorSetup(void);
// task dedicated to reading the dallas temp sensors
void SensorsTask(void *p);
// called by NextionTask() once to set things up
void NextionSetup(void);
// task dedicated to communication with the nextion device
void NextionTask(void *);


void printAddress(DeviceAddress deviceAddress) 
{
    for (uint8_t i = 0; i < 8; i++)
    {
      if (deviceAddress[i] < 16) 
          Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
    }
}

bool NextionSendCmd(const char *pszCmd, const int nVal)
{
    if (mySerial.availableForWrite())
    {
        if (pszCmd && *pszCmd)
            mySerial.print(pszCmd); 
        mySerial.print(nVal, DEC);
        mySerial.write("\xFF\xFF\xFF");
        return true;
    }
    return false;
}

bool NextionSendCmd(const char *pszCmd, const char *pszCmd2 /* = NULL */)
{
    bool bRet = false;
    if (mySerial.availableForWrite())
    {
        if (pszCmd && *pszCmd)
            mySerial.print(pszCmd); 
        if (pszCmd2 && *pszCmd2)
            mySerial.print(pszCmd2); 
        mySerial.write("\xFF\xFF\xFF");
        return true;
    }
    return false;
}
    
void NextionRxFlush(void)
{
    while (mySerial.available())
        mySerial.read();
}

size_t NextionReadData(void)
{
    size_t nBytes = 0;
    size_t nFCnt = 0;
    char buf[10];
    if (mySerial.available())
    {
        while (true)
        {
            NexRespBuf[nBytes] = (uint8_t)mySerial.read();
            if ((0xFF == NexRespBuf[nBytes]) && (++nFCnt == 3))
            {        
                nBytes -= 2;
                break;
            }
            if (++nBytes >= 100)
            {
                Serial.println("BUFFER OVERFLOW READING RESPONSE DATA");
                nBytes = 0;
                break;
            }
        }
    }
#ifdef SPAMMY_DEBUG    
    if (nBytes)
    {
        Serial.print("NEXTION DATA: ");
        for (size_t i = 0; i < (nBytes+3); i++)
        {
            Serial.print(" 0x"); Serial.print(NexRespBuf[i], HEX);
        }
        Serial.println("");
    }
#endif // SPAMMY_DEBUG    
    return nBytes;
}

uint32_t NextionGetValue(const char* pszValToGet)
{
    uint32_t uRet = UINT32_MAX;
    NextionRxFlush();
    if (pszValToGet)
    {
        // send the query before waiting for a response.
        NextionSendCmd("get ", pszValToGet);
    }
    while (!NextionReadData()); // no-op while reading data returns 0 bytes

    if (NexRespBuf[0] == 0x71)
    {
        uRet = NexRespBuf[1] | (NexRespBuf[2] << 8) | (NexRespBuf[3] << 16) | (NexRespBuf[4] << 24);
#ifdef SPAMMY_DEBUG    
        Serial.print(pszValToGet); Serial.print(" == "); Serial.println(uRet, DEC);
#endif // SPAMMY_DEBUG        
    }
    else if (NexRespBuf[0] == 0x1A)
    {
        Serial.println("Nextion Err:  Invalid Variable");
    }
    return uRet;
}

bool NextionSendAndWait(const char *pszToSend, const char *pszToWait, size_t nToWaitSize)
{
    char buf[10];
    bool bRet = false;
    if (nToWaitSize <= 10)
    {
        if (pszToSend)
        {
            NextionRxFlush();
            NextionSendCmd(pszToSend);
        }
        // wait for the response
        mySerial.setTimeout(1000);
        if ((nToWaitSize == mySerial.readBytes(buf, nToWaitSize)) &&
          (0 == memcmp(pszToWait, buf, nToWaitSize)))
        {
            bRet = true;
        }
    }
    return bRet;
}

void userSetup()
{
    // disable the freeRTOS idle task watchdog timers on both CPUs
    TaskHandle_t hIdler;
    hIdler = xTaskGetIdleTaskHandleForCPU(1);
    if (hIdler) esp_task_wdt_delete(hIdler);
    hIdler = xTaskGetIdleTaskHandleForCPU(0);
    if (hIdler) esp_task_wdt_delete(hIdler);

    mySerial.begin(115200); //, SERIAL_8N1, 27, 26);

    TaskHandle_t hSelf = xTaskGetCurrentTaskHandle();

    // sensors is higher priority (5) while nextion is "idle" priority (0)
    // tskNO_AFFINITY -> let the scheduler use whatever CPU core it wants.
    // 2048 is how much stack space I'm giving each task.  Be careful as they
    // might need to be raised as functionality is added!
    //
    // At the time of these notes, with 2048 assigned to each, the SensorTask 
    // has 244 bytes spare, and the NextionTask has 568 bytes spare - but nextion 
    // seems to be using stack space oddly... watching it drop to 536..
    //
    // arduino "setup"/"loop" should have a priority of 1, so nextiontask at 0 will be lower priority.
    xTaskCreatePinnedToCore(SensorsTask, "SensorsTask", 2048, (void*)hSelf, 5, &hSensorsTask, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(NextionTask, "NextionTask", 2048, (void*)hSelf, 0, &hNextionTask, tskNO_AFFINITY);
    
}

void SensorSetup(void)
{
    int numberOfDevices = 0;
    DeviceAddress tempDeviceAddress;

    sensors.begin();
    sensors.setResolution(11);
    sensors.setWaitForConversion(false);
    numberOfDevices = sensors.getDeviceCount();

    // locate devices on the bus
    Serial.print("Locating devices...");
    Serial.print("Found ");
    Serial.print(numberOfDevices, DEC);
    Serial.println(" devices.");

    // 286EEBE5391901FC is "air" temperature
    // address of first temp sensor:  286EEBE5391901FC.  Second is 28ADC9D93919011B

    // Loop through each device, print out address
    for(int i=0;i<numberOfDevices; i++)
    {
        // Search the wire for address
        if(sensors.getAddress(tempDeviceAddress, i))
        {
            if (0 == memcmp(tempDeviceAddress, airThermometer, sizeof(DeviceAddress))) 
            {
                Serial.println("Found AIR thermometer!");
            } 
            else if (0 == memcmp(tempDeviceAddress, waterThermometer, sizeof(DeviceAddress))) 
            {
                Serial.println("Found WATER thermometer!");
            }
            else 
            {
                Serial.print("Found UNKNOWN device ");
                Serial.print(i, DEC);
                Serial.println(" with address: ");
                printAddress(tempDeviceAddress);
                memcpy(unknownThermometer, tempDeviceAddress, sizeof(DeviceAddress));
                Serial.println();
            }
        } 
        else 
        {
            Serial.print("Found ghost device at ");
            Serial.println(i, DEC);
        }
    }
}

void SensorsTask(void *p)
{
#ifdef CORE_MONITOR    
    uint32_t uLastCore = 0xFFFF;
    uint32_t uCore = xPortGetCoreID();
#endif CORE_MONITOR    
    // depending on compiler optimizations, declaring ALL the variables earlier will ease stack shifting in the middle of the task
    int32_t nAirTemp, nH2OTemp;
    TaskHandle_t hParentTask = (TaskHandle_t)p;
    TickType_t xLastWakeTime;
    float fAirTemp, fH2OTemp;

#ifdef CORE_MONITOR
    Serial.printf("SensorsTask starting on core %d\n", (int)uCore);
    uLastCore = uCore;
#endif CORE_MONITOR

    SensorSetup();
    sensors.requestTemperatures(); 

    xLastWakeTime = xTaskGetTickCount();
    while (true)
    {        
        // only execute every 1000 milliseconds
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));

#ifdef CORE_MONITOR
        uCore = xPortGetCoreID();
        if (uCore != uLastCore)
        {
          Serial.printf("*************** SensorsTask switched to core %d\n", (int)uCore);
          uLastCore = uCore;
        }
#endif CORE_MONITOR

        // if the last temp request is complete...
        if (sensors.isConversionComplete())        
        {
            fAirTemp = sensors.getTempC(airThermometer);
            Serial.printf("Air Temp: %2.2fC %2.2fF\n", round(100*fAirTemp)/100.0, DallasTemperature::toFahrenheit(fAirTemp));
            fH2OTemp = sensors.getTempC(waterThermometer);
            Serial.printf("H2O Temp: %2.2fC %2.2fF\n", round(100*fH2OTemp)/100.0, DallasTemperature::toFahrenheit(fH2OTemp));
            if (unknownThermometer[0] != 0)
            {
                float fUnkTemp = sensors.getTempC(unknownThermometer);
                Serial.printf("Unk Temp: %2.2fC %2.2fF\n", round(100*fUnkTemp)/100.0, DallasTemperature::toFahrenheit(fUnkTemp));
            }
            if (!mySerial.availableForWrite())
            {
                Serial.println("UNABLE TO WRITE TO mySerial");
            }
            // not using constrain with the assign because it's a macro that will run "round()" twice each time
            nAirTemp = (int)round(fAirTemp * 10.0);
            nH2OTemp = (int)round(fH2OTemp * 10.0);
            // make sure the values will fit in 16 bits
            // Actually, go ahead and constrain it for use by everything
            nAirTemp = constrain(nAirTemp, -9999, 9999);
            nH2OTemp = constrain(nH2OTemp, -9999, 9999);
            // bit pack the sensor readings and notify the userloop
            xTaskNotify(hParentTask, (nAirTemp<<16) | (nH2OTemp&0xFFFF), eSetValueWithOverwrite);

            // Only request temps inside the isConversionComplete() conditional. If the conversion wasn't
            // completed, we should wait until the previous "request" completes
            sensors.requestTemperatures(); 
        }
    } // while(true)
}

void NextionSetup(void)
{
    // wait 1000ms for the nextion to boot up, and then initialize it
    vTaskDelay(pdMS_TO_TICKS(1000));

    NextionReadData();
    NextionSendCmd("");
    NextionReadData();

    // loop on a page command to ensure we're talking

    // TODO:  this endless loop is bad.  Not sure what else to do, though.  There's no
    // way to physically reset the nextion device without also resetting the esp
    while (!NextionSendAndWait("page 0", "\x66\x00\xFF\xFF\xFF", 5))
        ; // endless loop waiting for page ack
    Serial.println("Getting variables from nextion");
    uGraphID = NextionGetValue(GRAPHNAME ".id");
    uGraphHeight = NextionGetValue(GRAPHNAME ".h");
    uGraphWidth = NextionGetValue(GRAPHNAME ".w");

    Serial.println("Forcing page switches at 1 minutes.");
    NextionSendCmd("page0.vHowLong.val=1");

    if (uGraphWidth)
    {
        pAirGraphData = new uint8_t[uGraphWidth];
        memset(pAirGraphData, 0, uGraphWidth);
        pH2OGraphData = new uint8_t[uGraphWidth];
        memset(pH2OGraphData, 0, uGraphWidth);

        uGraphDataIdx = 0;

        StartRefillGraph();
    }
}
void NextionTask(void *)
{
    int nNextH2OTemp = 0;
    int nNextAirTemp = 0;
    int nNextDeltaTemp = 0;

    int nAirTemp, nH2OTemp, nDeltaTemp, nAirMapped, nH2OMapped;
#ifdef CORE_MONITOR
    uint32_t uLastCore = 0xFFFF;
    uint32_t uCore = xPortGetCoreID();
#endif CORE_MONITOR
    char bufAddCmd[16] = { "add 00,0," };

#ifdef CORE_MONITOR
    Serial.printf("NextionTask starting on core %d\n", (int)uCore);
    uLastCore = uCore;    
#endif CORE_MONITOR    

    NextionSetup();
    // wait for notifications with a timeout.
    uint32_t uTaskNotifValue;

    while (true)
    {
        BaseType_t btWwaitResp = xTaskNotifyWait(0, 0, &uTaskNotifValue, pdMS_TO_TICKS(200)); // timeout after 200ms to check for nextion page changes
#ifdef CORE_MONITOR
        uCore = xPortGetCoreID();
        if (uCore != uLastCore)
        {
            Serial.printf("*************** NextionTask switched to core %d\n", (int)uCore);
            uLastCore = uCore;
        }
#endif CORE_MONITOR
        if (pdTRUE == btWwaitResp)
        {

            nAirTemp = (int) (int16_t) (uTaskNotifValue >> 16);  // casting first to int16 to preserve the sign
            nH2OTemp = (int) (int16_t) (uTaskNotifValue & 0xFFFF);  // casting first to int16 to preserve the sign
            if (!mySerial.availableForWrite())
            {
                Serial.println("UNABLE TO WRITE TO mySerial");
            }
            nDeltaTemp = nH2OTemp - nAirTemp;
            nDeltaTemp = constrain(nDeltaTemp, -999, 999);
 
            if (nH2OTemp != nNextH2OTemp) // if the water temperature changed since last iteration...
            {
                // if there wasn't a last H2O temp, make sure the colors update                 
                // change colors if we're going over 30.0 or under 30.0
                if ((nH2OTemp >= 340) && ((nNextH2OTemp < 340) || !nNextH2OTemp))
                {            
                    // change to red
                    NextionSendCmd(GRAPHNAME ".pco0=" NEXTION_RED); 
                    NextionSendCmd(GRAPHNAME ".pco1=" NEXTION_RED); 
                    NextionSendCmd(H20NAME ".pco=" NEXTION_RED);                 
                }
                else if ((nH2OTemp < 340) && ((nNextH2OTemp >= 340) || !nNextH2OTemp))
                {
                    // change to blue
                    NextionSendCmd(GRAPHNAME ".pco0=" NEXTION_CYAN); 
                    NextionSendCmd(GRAPHNAME ".pco1=" NEXTION_CYAN); 
                    NextionSendCmd(H20NAME ".pco=" NEXTION_CYAN); 
                }
                nNextH2OTemp = nH2OTemp;
                // update the dispaly
                NextionSendCmd(H20NAME ".val=", nH2OTemp);


            } // if the water temperature changed since last iteration.

            if (nAirTemp != nNextAirTemp) // if the air temperature changed since last iteration...
            {
                // no color changes for air temp.  Always green
                nNextAirTemp = nAirTemp;
                // update the display
                NextionSendCmd(AIRNAME ".val=", nAirTemp);
            } // if the air temperature changed since last iteration.

            if (nDeltaTemp != nNextDeltaTemp) // if the delta temperature changed since last iteration...
            {
                // change the BORDER for delta
                // change colors if we're going over 30.0 or under 30.0
                if ((nDeltaTemp >= 70) && ((nNextDeltaTemp < 70) || !nNextDeltaTemp))
                {            
                    // change border to red
                    NextionSendCmd(DELTANAME ".borderc=" NEXTION_RED);
                }
                else if ((nDeltaTemp < 70) && ((nNextDeltaTemp >= 70) || !nNextDeltaTemp))
                {
                    // change border to black (invisible)
                    NextionSendCmd(DELTANAME ".borderc=0");
                }
                nNextDeltaTemp = nDeltaTemp;
                // update the display
                NextionSendCmd(DELTANAME ".val=", nDeltaTemp);
            } // if the air temperature changed since last iteration.

            // Handle the graphs.  They'll need additional constrains on the temps.
            // the min mapping is "1" (not 0) so the offset can be drawn
            nH2OMapped = (int)map(constrain(nH2OTemp, MIN_TEMP, (MAX_TEMP*10)), (MIN_TEMP*10), (MAX_TEMP*10), 1, uGraphHeight );
            nAirMapped = (int)map(constrain(nAirTemp, MIN_TEMP, (MAX_TEMP*10)), (MIN_TEMP*10), (MAX_TEMP*10), 1, uGraphHeight );
          
            // keep a buffer a graph data in case we need to re-fill the graph
            // uGraphDataIdx points to the NEXT element to be used...
            pAirGraphData[uGraphDataIdx] = nAirMapped;
            pH2OGraphData[uGraphDataIdx] = nH2OMapped;

            if (++uGraphDataIdx >= uGraphWidth)
                uGraphDataIdx = 0; 

            // did the display page flip?
            if ((2 == NextionReadData()) && (0x66 == NexRespBuf[0]))
            {
                // the page just flipped.  The nextion scripts will reset colors, but still need to refill the graph
                StartRefillGraph();
            }
            else
            {
                // char bufAddCmd[16] = { "add 00,0," };
                bufAddCmd[4] = '0' + (char)(uGraphID / 10);
                bufAddCmd[5] = '0' + (char)(uGraphID % 10);

                bufAddCmd[7] = '0';
                NextionSendCmd(bufAddCmd, nH2OMapped);
                bufAddCmd[7] = '1';
                NextionSendCmd(bufAddCmd, nH2OMapped-1);
                bufAddCmd[7] = '2';
                NextionSendCmd(bufAddCmd, nAirMapped);
                bufAddCmd[7] = '3';
                NextionSendCmd(bufAddCmd, nAirMapped-1);
            }
        } 
        else if ((2 == NextionReadData()) && (0x66 == NexRespBuf[0]))
        {
          StartRefillGraph();
        }
    } // while (true)
}

void StartRefillGraph(void)
{
    // "addt 255,255,255" - 16 chars + nul
    char buf[17];

    // this loop can take a few milliseconds, so yield on occassion
    for (unsigned i = 0; i < 4; i++)
    {   
        taskYIELD();
        snprintf(buf, 17, "addt %d,%d,%d", (int)uGraphID, i, (int)uGraphWidth - 1);
        if (NextionSendAndWait(buf, "\xFE\xFF\xFF\xFF", 4))
        {
            RefillGraph(i < 2 ? pH2OGraphData : pAirGraphData, (0 == i % 2));
            if (!NextionSendAndWait(NULL, "\xFD\xFF\xFF\xFF", 4))
            {
                Serial.println("Missing ACK?");
            }
        }
    }
}


void RefillGraph(uint8_t *pGraphData, bool bOffset)
{
    // the last filled data.  uGraphDataIdx points to the "next" element that would have been used,
    // The following element (assigned to i) will be the oldest data point.
    unsigned i = uGraphDataIdx + 1;
    // circular buffer wraparound
    if (i >= uGraphWidth)
        i = 0;

    while (i != uGraphDataIdx) // until i reaches the next (unplotted) index...
    {
        // water data...
        uint8_t byteToSend = pGraphData[i];
        if ((0 != byteToSend) && bOffset)
            byteToSend -= 1;
        if (1 != mySerial.write(byteToSend))
        {
            Serial.println("(send failed?  why?)");
        }

        // increment i with wraparound if needed
        if (++i >= uGraphWidth)
            i = 0;
    }
}

//gets called every time WiFi is (re-)connected. Initialize own network interfaces here
void userConnected()
{
// ... really should display a little wifi symbol. ;)
}


void userLoop()
{    
    int nH2OTemp, nAirTemp, nDeltaTemp;

    // the userLoop should only deal with the LED's and to forward sensor data to nextion
#ifdef STACK_MONITOR    
    EVERY_N_MILLISECONDS_I(stackcheck, 20000)
    {
        printf("SENSOR TASK FREE STACK: %u\n", uxTaskGetStackHighWaterMark(hSensorsTask));
        printf("NEXTION TASK FREE STACK: %u\n", uxTaskGetStackHighWaterMark(hNextionTask));
    }
#endif STACK_MONITOR

    EVERY_N_MILLISECONDS_I(CalcTemp, 100)  // this can run every 100ms, but we'll only get a notification every 1000ms
    {
        uint32_t uTaskNotifValue = 0;
        if (pdTRUE == xTaskNotifyWait(0, 0, &uTaskNotifValue, 0))
        {
            if (hNextionTask)
            {
                xTaskNotify(hNextionTask, uTaskNotifValue, eSetValueWithOverwrite);
            }

            // check the water temperature to see if the effect color should be changed
            nH2OTemp = (int)(int16_t)(uTaskNotifValue & 0xFFFF); // casting first to int16 to preserve the sign
            nAirTemp = (int) (int16_t) (uTaskNotifValue >> 16);  // casting first to int16 to preserve the sign
            nDeltaTemp = nH2OTemp - nAirTemp;
            nDeltaTemp = constrain(nDeltaTemp, -999, 999);

            if (nDeltaTemp >= 70)
            {
                if (35 != effectPalette)
                {
                    // change to "lava"
                    effectPalette = 35;
                    colorUpdated(NOTIFIER_CALL_MODE_DIRECT_CHANGE);
                }
            }
            else if ((nDeltaTemp < 70) && (36 != effectPalette))
            {
                // change to "icefire"
                effectPalette = 36;
                colorUpdated(NOTIFIER_CALL_MODE_DIRECT_CHANGE);
            }
        }
    }
}
