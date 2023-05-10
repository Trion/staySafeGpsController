
/*
This code completely remade by @Trion
CopyRight By: TechmationMyanmar
26/11/2022
cloning for stay safe you can find orginal code in production folder 
@Trion
*/
#include <string.h>
#include <stdio.h>
#include <api_os.h>
#include <api_gps.h>
#include <api_event.h>
#include <api_hal_uart.h>
#include <api_debug.h>
#include "buffer.h"
#include "gps_parse.h"
#include "math.h"
#include "gps.h"
#include "api_hal_pm.h"
#include "time.h"
#include "api_info.h"
#include "assert.h"
#include "api_socket.h"
#include "api_network.h"
#include "api_hal_gpio.h"


#define SERVER_IP "137.184.127.105" // demo server ip , for production change here
#define SERVER_PORT 3002

#define MAIN_TASK_STACK_SIZE (2048 * 2)
#define MAIN_TASK_PRIORITY 0
#define MAIN_TASK_NAME "GPS Test Task"

static HANDLE gpsTaskHandle = NULL;
bool isGpsOn = true;
bool receiveEve = false; // enviromental sensor
bool networkFlag = false; // plese clean
uint32_t Eve_buffer[100];
uint8_t data[100];
uint8_t buffer[1024], buffer2[400];

#define SYSTEM_STATUS_LED   GPIO_PIN26
#define UPLOAD_DATA_LED     GPIO_PIN28
#define GPIO_PIN_SOS        GPIO_PIN27

Power_On_Cause_t powerOnCause = POWER_ON_CAUSE_MAX;

void EventDispatch(API_Event_t *pEvent)
{
    switch (pEvent->id)
    {
        case API_EVENT_ID_NO_SIMCARD:
        Trace(10, "!!NO SIM CARD%d!!!!", pEvent->param1);
        networkFlag = false;
        break;
        case API_EVENT_ID_NETWORK_REGISTER_SEARCHING:
        Trace(2, "network register searching");
        networkFlag = false;
        break;
        case API_EVENT_ID_NETWORK_REGISTER_DENIED:
        Trace(2, "network register denied");
        case API_EVENT_ID_NETWORK_REGISTER_NO:
        Trace(2, "network register no");
        break;
        case API_EVENT_ID_GPS_UART_RECEIVED:
        Trace(1,"received GPS data,length:%d, data:%s",pEvent->param1,pEvent->pParam1);
        GPS_Update(pEvent->pParam1, pEvent->param1);
        break;
        case API_EVENT_ID_NETWORK_REGISTERED_HOME:
        case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
        {
            uint8_t status;
            Trace(2, "network register success");
            bool ret = Network_GetAttachStatus(&status);
            if (!ret)
                Trace(1, "get attach staus fail");
            Trace(1, "attach status:%d", status);
            if (status == 0)
            {
                ret = Network_StartAttach();
                if (!ret)
                {
                    Trace(1, "network attach fail");
                }
            }
            else
            {
                Network_PDP_Context_t context = {
                    .apn = "cmnet",
                    .userName = "",
                    .userPasswd = ""};
                    Network_StartActive(context);
                }
                break;
            }
            case API_EVENT_ID_NETWORK_ATTACHED:
            Trace(2, "network attach success");
            Network_PDP_Context_t context = {
                .apn = "cmnet",
                .userName = "",
                .userPasswd = ""};
                Network_StartActive(context);
                break;

                case API_EVENT_ID_NETWORK_ACTIVATED:
                Trace(2, "network activate success");
                networkFlag = true;
                break;

                case API_EVENT_ID_UART_RECEIVED:
                if (pEvent->param1 == UART1)
                {
                    data[pEvent->param2 + 1];
                    data[pEvent->param2] = 0;
                    memcpy(data, pEvent->pParam1, pEvent->param2);
                    if (strcmp(data, "close") == 0)
                    {
                        Trace(1, "close gps");
                        GPS_Close();
                        isGpsOn = false;
                    }
                    else if (strcmp(data, "open") == 0)
                    {
                        Trace(1, "open gps");
                        GPS_Open(NULL);
                        isGpsOn = true;
                    }
                }
                break;
                default:
                break;
            }
        }
 

// http post with no header
        int Http_Post(const char *domain, int port, const char *path, uint8_t *body, uint16_t bodyLen, char *retBuffer, int bufferLen)
        {
            uint8_t ip[16];
            bool flag = false;
            uint16_t recvLen = 0;

    // connect server
            memset(ip, 0, sizeof(ip));
            if (DNS_GetHostByName2(domain, ip) != 0)
            {
                Trace(2, "get ip error");
                return -1;
            }
    // Trace(2,"get ip success:%s -> %s",domain,ip);
            char *servInetAddr = ip;
            char *temp = OS_Malloc(2048);
            if (!temp)
            {
                Trace(2, "malloc fail");
                return -1;
            }
            snprintf(temp, 2048, "POST %s HTTP/1.1\r\nHost:%s\r\nx-access-token:eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJfaWQiOiI2Mzk2ZmZiMWQ5NmZjNzc5MzdhODc0ZWYiLCJpYXQiOjE2NzQ1NTExNjd9.mRkG8ClIa058XoB5opcupGh3WD3Jy65-PpAPIbliC8I\r\n\r\n",
               path, domain);
            char *pData = temp;
            int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (fd < 0)
            {
                Trace(2, "socket fail");
                OS_Free(temp);
                return -1;
            }
    // Trace(2,"fd:%d",fd);
            struct sockaddr_in sockaddr;
            memset(&sockaddr, 0, sizeof(sockaddr));
            sockaddr.sin_family = AF_INET;
            sockaddr.sin_port = htons(port);
            inet_pton(AF_INET, servInetAddr, &sockaddr.sin_addr);

            int ret = connect(fd, (struct sockaddr *)&sockaddr, sizeof(struct sockaddr_in));
            if (ret < 0)
            {
                Trace(2, "socket connect fail");
                OS_Free(temp);
                close(fd);
                return -1;
            }
    // Trace(2,"socket connect success");
            Trace(2, "send request:%s", pData);
            ret = send(fd, pData, strlen(pData), 0);
            if (ret < 0)
            {
                Trace(2, "socket send fail");
                OS_Free(temp);
                close(fd);
                return -1;
            }
            ret = send(fd, body, bodyLen, 0);
            if (ret < 0)
            {
                Trace(2, "socket send fail");
                OS_Free(temp);
                close(fd);
                return -1;
            }
    // Trace(2,"socket send success");

            struct fd_set fds;
            struct timeval timeout = {12, 0};
            FD_ZERO(&fds);
            FD_SET(fd, &fds);
            while (!flag)
            {
                ret = select(fd + 1, &fds, NULL, NULL, &timeout);
        // Trace(2,"select return:%d",ret);
                switch (ret)
                {
                    case -1:
                    Trace(2, "select error");
                    flag = true;
                    break;
                    case 0:
                    Trace(2, "select timeout");
                    flag = true;
                    break;
                    default:
                    if (FD_ISSET(fd, &fds))
                    {
                        memset(retBuffer, 0, bufferLen);
                        ret = recv(fd, retBuffer, bufferLen, 0);
                        recvLen += ret;
                        if (ret < 0)
                        {
                            Trace(2, "recv error");
                            flag = true;
                            break;
                        }
                        else if (ret == 0)
                        {
                            Trace(2, "ret == 0");
                            break;
                        }
                        else if (ret < 1352)
                        {
                            GPS_DEBUG_I("recv len:%d,data:%s", recvLen, retBuffer);
                            close(fd);
                            OS_Free(temp);
                            return recvLen;
                        }
                    }
                    break;
                }
            }
            close(fd);
            OS_Free(temp);
            return -1;
        }

 
        void gps_testTask(void *pData)
        {
            GPS_Info_t *gpsInfo = Gps_GetInfo();
            GPIO_config_t btn_sos  = {
                .mode               = GPIO_MODE_INPUT,
                .pin                = GPIO_PIN_SOS,
                .defaultLevel       = GPIO_LEVEL_HIGH,
            };
            receiveEve = false;
            
            GPIO_Init(btn_sos);
            UART_Write(UART1, "Init now\r\n", strlen("Init now\r\n"));
            GPS_Init();
            GPS_Open(NULL);
            while (gpsInfo->rmc.latitude.value == 0)
                OS_Sleep(1000);
            for (uint8_t i = 0; i < 5; ++i)
            {
                bool ret = GPS_SetOutputInterval(10000);
                Trace(1, "set gps ret:%d", ret);
                if (ret)
                    break;
                OS_Sleep(1000);
            }

            if (!GPS_GetVersion(buffer, 150))
                Trace(1, "get gps firmware version fail");
            else
                Trace(1, "gps firmware version:%s", buffer);

            if (!GPS_SetLpMode(GPS_LP_MODE_SUPPER_LP))
                Trace(1, "set gps lp mode fail");

            if (!GPS_SetOutputInterval(1000))
                Trace(1, "set nmea output interval fail");

            Trace(1, "init ok");
            UART_Write(UART1, "Init ok\r\n", strlen("Init ok\r\n"));

            while (1)
            {
                if (isGpsOn)
                {
            // show fix info
                    uint8_t isFixed = gpsInfo->gsa[0].fix_type > gpsInfo->gsa[1].fix_type ? gpsInfo->gsa[0].fix_type : gpsInfo->gsa[1].fix_type;
                    char *isFixedStr;
                    if (isFixed == 2)
                        isFixedStr = "2D fix";
                    else if (isFixed == 3)
                    {
                        if (gpsInfo->gga.fix_quality == 1)
                            isFixedStr = "3D fix";
                        else if (gpsInfo->gga.fix_quality == 2)
                            isFixedStr = "3D/DGPS fix";
                    }
                    else
                        isFixedStr = "no fix";
                    // convert unit ddmm.mmmm to degree(°)
                    int temp = (int)(gpsInfo->rmc.latitude.value / gpsInfo->rmc.latitude.scale / 100);
                    double latitude = temp + (double)(gpsInfo->rmc.latitude.value - temp * gpsInfo->rmc.latitude.scale * 100) / gpsInfo->rmc.latitude.scale / 60.0;
                    temp = (int)(gpsInfo->rmc.longitude.value / gpsInfo->rmc.longitude.scale / 100);
                    double longitude = temp + (double)(gpsInfo->rmc.longitude.value - temp * gpsInfo->rmc.longitude.scale * 100) / gpsInfo->rmc.longitude.scale / 60.0;
                    snprintf(buffer, sizeof(buffer), "GPS fix mode:%d, BDS fix mode:%d, fix quality:%d, satellites tracked:%d, gps sates total:%d, is fixed:%s, coordinate:WGS84, Latitude:%f, Longitude:%f, unit:degree,altitude:%f", gpsInfo->gsa[0].fix_type, gpsInfo->gsa[1].fix_type,
                       gpsInfo->gga.fix_quality, gpsInfo->gga.satellites_tracked, gpsInfo->gsv[0].total_sats, isFixedStr, latitude, longitude, gpsInfo->gga.altitude);
                     // show in tracer
                    Trace(1, buffer);
                    // send to UART1
                    // UART_Write(UART1, buffer, strlen(buffer));
                    // UART_Write(UART1, "\r\n\r\n", 4);
                    char *requestPath = buffer2;
                    uint8_t percent;
                    uint16_t v = PM_Voltage(&percent);
                    Trace(1, "power:%d %d", v, percent);
                    if(percent <=50){
                       GPIO_Set(UPLOAD_DATA_LED, GPIO_LEVEL_HIGH);
                       OS_Sleep(500);
                       GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                   }
                   memset(buffer, 0, sizeof(buffer));
            if (!INFO_GetIMEI(buffer))
                    Assert(false, "NO IMEI");
                Trace(1, "device name:%s", buffer);
                // UART_Write(UART1, buffer, strlen(buffer));
                // Trace(1, "Uart main receive: %s", data);
                snprintf(requestPath, sizeof(buffer2), "/api/device-control/63b3edda77d59a0b3a6206ce/emergency?lat=%f&lon=%f",latitude, longitude);
                Trace(1, requestPath);
                uint8_t status;
                Network_GetActiveStatus(&status);
                GPIO_LEVEL level = 0;
                GPIO_GetLevel(btn_sos,&level);
                Trace(1,"GPIO STATE : %d",level);
                // btn pressed event
                if (level == GPIO_LEVEL_LOW) {
                    Trace(9,"EMERGENCY BUTTON PRESSED");
                    GPIO_Set(UPLOAD_DATA_LED, GPIO_LEVEL_HIGH);
                    PM_SleepMode(false);
                    receiveEve = true;
                } else if (level == GPIO_LEVEL_HIGH) {
                      GPIO_Set(UPLOAD_DATA_LED, GPIO_LEVEL_LOW);
                    Trace(9,"EMERGENCY BUTTON RELEASED");
                    Trace(9,"SLEEP MODE ");
                    PM_SleepMode(true);
                }

                if (status)
                {
                    GPIO_Set(GPIO_PIN25,GPIO_LEVEL_LOW);
                    Trace(9,"PRESEED TEST SEND");
                // please updat this point replace with push button pressed state
                    if (receiveEve)
                    {
                        GPIO_Set(UPLOAD_DATA_LED, GPIO_LEVEL_HIGH);
                        if (Http_Post(SERVER_IP, SERVER_PORT, requestPath, NULL, 0, buffer, sizeof(buffer)) < 0)
                            Trace(1, "send location to server fail");
                        else
                        {
                            Trace(1, "send location to server success");
                            Trace(1, "response:%s", buffer);
                        }
                        GPIO_Set(UPLOAD_DATA_LED, GPIO_LEVEL_LOW);
                        Trace(9,"Pressed sent completed data is closed");
                        receiveEve = false;
                    }
                }
                else
                {
                   
                    Trace(1, "no internet");
                }
            }
            PM_SetSysMinFreq(PM_SYS_FREQ_32K);
            OS_Sleep(500);
            PM_SetSysMinFreq(PM_SYS_FREQ_178M);
        }
    }

    void LED_Blink(void *param)
    {
        static int count = 0;
        if (++count == 5)
        {
            GPIO_Set(SYSTEM_STATUS_LED, GPIO_LEVEL_HIGH);
        }
        else if (count == 6)
        {
            GPIO_Set(SYSTEM_STATUS_LED, GPIO_LEVEL_LOW);
            count = 0;
        }
        OS_StartCallbackTimer(gpsTaskHandle, 1000, LED_Blink, NULL);
    }

    void gps_MainTask(void *pData)
    {
        API_Event_t *event = NULL;
        TIME_SetIsAutoUpdateRtcTime(true);
    // open UART1 to print NMEA infomation
    UART_Config_t config = {
            .baudRate = UART_BAUD_RATE_115200,
            .dataBits = UART_DATA_BITS_8,
            .stopBits = UART_STOP_BITS_1,
            .parity = UART_PARITY_NONE,
            .rxCallback = NULL,
            .useEvent = true};
    GPIO_config_t gpioLedBlue = {
                .mode = GPIO_MODE_OUTPUT,
                .pin = SYSTEM_STATUS_LED,
                .defaultLevel = GPIO_LEVEL_LOW};
    GPIO_config_t gpioLedUpload = {
                    .mode = GPIO_MODE_OUTPUT,
                    .pin = UPLOAD_DATA_LED,
                    .defaultLevel = GPIO_LEVEL_LOW};
                    PM_PowerEnable(POWER_TYPE_VPAD, true); // for all gpio power ouput open
                    GPIO_Init(gpioLedBlue);
                    GPIO_Init(gpioLedUpload);
                    //UART_Init(UART1, config);
    // Create UART1 send task and location print task
    OS_CreateTask(gps_testTask,
                      NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
                    OS_StartCallbackTimer(gpsTaskHandle, 1000, LED_Blink, NULL);
    // Wait event
    while (1)
                    {
                        if (OS_WaitEvent(gpsTaskHandle, (void **)&event, OS_TIME_OUT_WAIT_FOREVER))
                        {
                            EventDispatch(event);
                            OS_Free(event->pParam1);
                            OS_Free(event->pParam2);
                            OS_Free(event);
                        }
                    }
                }
                void gps_tracker_Main(void)
                {
                    gpsTaskHandle = OS_CreateTask(gps_MainTask,
                      NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
                    OS_SetUserMainHandle(&gpsTaskHandle);
                }
