
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
#include "api_lbs.h"
#include "api_hal_gpio.h"
#include "sdk_init.h"

#include "cJSON.h"
#include "tiny-json.h"
#include "assert.h"



/***
 *  Modified By Techmation Myanmar
 * @Trion (NYEIN CHAN KO)
 * cell center info gps sos emergency alarm device v1.1 upgrade version normal condition
 * 
 * in v1.4 
 * 1. add gprs restart btn 
 * 2. add gps time out state (time out state is counter 99)
 **/
#define SERVER_IP "137.184.127.105" // demo server ip , for production change here
#define SERVER_PORT 3002

#define CELL_TOWER_SERVER "api.mylnikov.org"
#define CELL_TOWER_PORT " "   

#define GPS_NMEA_LOG_FILE_PATH "/t/gps_nmea.log"



#define MAIN_TASK_STACK_SIZE    (2048 * 2)
#define MAIN_TASK_PRIORITY      0
#define MAIN_TASK_NAME          "GPS Test Task"

static HANDLE gpsTaskHandle = NULL;
bool isGpsOn = true;
bool networkFlag = false;
bool sos_btn_event = false; // v1.2
bool gps_time_out_event = false; // v1.4
HANDLE semGetCellInfo = NULL;

float latitudeLbs  = 0.0;
float longitudeLbs = 0.0;
bool btn_active   = false;
bool flag_lbs = false;

static int count = 0 ;  // v1.2
static int btn_press_time = 0;
uint8_t number;

uint8_t buffer[1024],buffer2[400];
char cell_tower_path_buffer[1024];
char buffer_tower[1000];
int buffer_tower_len = sizeof(buffer_tower);
int cell_tower_path_buffer_len = sizeof(cell_tower_path_buffer);

char cell_request_path[1024];



Network_Location_t* location;

uint8_t MCC[5],MNC[3],LAC[5],CELLID[6];
int lbs_count = 0;

#define SYSTEM_STATUS_LED GPIO_PIN26
#define UPLOAD_DATA_LED   GPIO_PIN28
#define GPIO_PIN_SOS      GPIO_PIN29


// const uint8_t nmea[]="$GNGGA,000021.263,2228.7216,N,11345.5625,E,0,0,,153.3,M,-3.3,M,,*4E\r\n$GPGSA,A,1,,,,,,,,,,,,,,,*1E\r\n$BDGSA,A,1,,,,,,,,,,,,,,,*0F\r\n$GPGSV,1,1,00*79\r\n$BDGSV,1,1,00*68\r\n$GNRMC,000021.263,V,2228.7216,N,11345.5625,E,0.000,0.00,060180,,,N*5D\r\n$GNVTG,0.00,T,,M,0.000,N,0.000,K,N*2C\r\n";


void EventDispatch(API_Event_t* pEvent)
{
    static uint8_t lbsCount = 0;
    switch(pEvent->id)
    {
        case API_EVENT_ID_NO_SIMCARD:
            Trace(10,"!!NO SIM CARD%d!!!!",pEvent->param1);
            networkFlag = false;
            break;
        case API_EVENT_ID_NETWORK_REGISTER_SEARCHING:
            Trace(2,"network register searching");
            networkFlag = false;
            break;
        case API_EVENT_ID_NETWORK_REGISTER_DENIED:
            Trace(2,"network register denied");
        case API_EVENT_ID_NETWORK_REGISTER_NO:
            Trace(2,"network register no");
            break;
        case API_EVENT_ID_GPS_UART_RECEIVED:
            // Trace(1,"received GPS data,length:%d, data:%s,flag:%d",pEvent->param1,pEvent->pParam1,flag);
            GPS_Update(pEvent->pParam1,pEvent->param1);
            break;
        case API_EVENT_ID_NETWORK_REGISTERED_HOME:
        case API_EVENT_ID_NETWORK_REGISTERED_ROAMING:
        {
            uint8_t status;
            Trace(2,"network register success");
            bool ret = Network_GetAttachStatus(&status);
            if(!ret)
                Trace(1,"get attach staus fail");
            Trace(1,"attach status:%d",status);
            if(status == 0)
            {
                ret = Network_StartAttach();
                if(!ret)
                {
                    Trace(1,"network attach fail");
                }
            }
            else
            {
                Network_PDP_Context_t context = {
                    .apn        ="cmnet",
                    .userName   = ""    ,
                    .userPasswd = ""
                };
                Network_StartActive(context);
            }
            break;
        }
        case API_EVENT_ID_NETWORK_ATTACHED:
            Trace(2,"network attach success");
            Network_PDP_Context_t context = {
                .apn        ="cmnet",
                .userName   = ""    ,
                .userPasswd = ""
            };
            Network_StartActive(context);
            break;

        case API_EVENT_ID_NETWORK_ACTIVATED:
            Trace(2,"network activate success");
            networkFlag = true;
            break;
        
        case API_EVENT_ID_UART_RECEIVED:
            if(pEvent->param1 == UART1)
            {
                uint8_t data[pEvent->param2+1];
                data[pEvent->param2] = 0;
                memcpy(data,pEvent->pParam1,pEvent->param2);
                Trace(1,"uart received data,length:%d,data:%s",pEvent->param2,data);
                if(strcmp(data,"close") == 0)
                {
                    Trace(1,"close gps");
                    GPS_Close();
                    isGpsOn = false;
                }
                else if(strcmp(data,"open") == 0)
                {
                    Trace(1,"open gps");
                    GPS_Open(NULL);
                    isGpsOn = true;
                }
            }
            break;
        case API_EVENT_ID_NETWORK_CELL_INFO:
        {
            number = pEvent->param1;
            location = (Network_Location_t*)pEvent->pParam1;
            Trace(2,"network cell infomation,serving cell number:1, neighbor cell number:%d",number-1);
            for(int i=0;i<number;++i)
            {
                Trace(2,"cell %d info:%d%d%d,%d%d%d,%d,%d,%d,%d,%d,%d",i,
                location[i].sMcc[0], location[i].sMcc[1], location[i].sMcc[2], 
                location[i].sMnc[0], location[i].sMnc[1], location[i].sMnc[2],
                location[i].sLac, location[i].sCellID, location[i].iBsic,
                location[i].iRxLev, location[i].iRxLevSub, location[i].nArfcn);
            }

              int latitudeLbs = 0;
              int longitudeLbs =0;
            if((latitudeLbs == 0) &&(longitudeLbs == 0))//not get location from server, try again
            {
                if(++lbsCount>=2)
                {
                    lbsCount = 0;
                    Trace(1,"try 6 times to get location from lbs but fail!! %d", lbsCount);
                    OS_ReleaseSemaphore(semGetCellInfo);
                    break;
                }
                if(!Network_GetCellInfoRequst())
                {
                    Trace(1,"network get cell info fail");
                    OS_ReleaseSemaphore(semGetCellInfo);
                }
                break;
            }
            OS_ReleaseSemaphore(semGetCellInfo);
            lbsCount = 0;
          
            break;
        }
        

        default:
            break;
    }
}


//http post with inteli server 
int Http_Post(const char* domain, int port,const char* path,uint8_t* body, uint16_t bodyLen, char* retBuffer, int bufferLen, bool check_lbs)
{
    uint8_t ip[16];
    bool flag = false;
    uint16_t recvLen = 0;
    int retBufferLen = bufferLen;
    //connect server
    memset(ip,0,sizeof(ip));
    if(DNS_GetHostByName2(domain,ip) != 0)
    {
        Trace(2,"get ip error");
        return -1;
    }
    // Trace(2,"get ip success:%s -> %s",domain,ip);
    char* servInetAddr = ip;
    char* temp = OS_Malloc(2048);
    if(!temp)
    {
        Trace(2,"malloc fail");
        return -1;
    }
    if (!check_lbs)
    {
     snprintf(temp, 2048, "POST %s HTTP/1.1\r\nHost:%s\r\nx-access-token:eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJfaWQiOiI2Mzk2ZmZiMWQ5NmZjNzc5MzdhODc0ZWYiLCJpYXQiOjE2ODQ5ODUyMjB9.U3qHiJ-HuJQDmCYrdew80k9M77nkS042pBNerafmqbs\r\n\r\n",
                   path, domain);
    } else {
         snprintf(temp, 2048, "POST %s HTTP/1.1\r\nContent-Type:application/json\r\nHost:%s\r\n\r\n",
                   path, domain);
    }

    char* pData = temp;
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(fd < 0){
        Trace(2,"socket fail");
        OS_Free(temp);
        return -1;
    }
    Trace(2,"fd:%d",fd);

    struct sockaddr_in sockaddr;
    memset(&sockaddr,0,sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(port);
    inet_pton(AF_INET,servInetAddr,&sockaddr.sin_addr);

    int ret = connect(fd, (struct sockaddr*)&sockaddr, sizeof(struct sockaddr_in));
    if(ret < 0){
        Trace(2,"socket connect fail");
        OS_Free(temp);
        close(fd);
        return -1;
    }
    Trace(2,"socket connect success");
    Trace(2,"send request:%s",pData);
    ret = send(fd, pData, strlen(pData), 0);
    if(ret < 0){
        Trace(2,"socket send fail");
        OS_Free(temp);
        close(fd);
        return -1;
    }
    ret = send(fd, body, bodyLen, 0);
    if(ret < 0){
        Trace(2,"socket send fail");
        OS_Free(temp);
        close(fd);
        return -1;
    }
    Trace(2,"socket send success");

    struct fd_set fds;
    struct timeval timeout={12,0};
    FD_ZERO(&fds);
    FD_SET(fd,&fds);
    while(!flag)
    {
        ret = select(fd+1,&fds,NULL,NULL,&timeout);
         Trace(2,"select return:%d",ret);
        switch(ret)
        {
            case -1:
                Trace(2,"select error");
                flag = true;
                break;
            case 0:
                Trace(2,"select timeout");
                flag = true;
                break;
            default:
                if(FD_ISSET(fd,&fds))
                {
                    // memset(retBuffer,0,bufferLen);
                    // ret = recv(fd,retBuffer,bufferLen,0);
                    Trace(1,"select return:%d",ret);
                    memset(retBuffer+recvLen,0,retBufferLen-recvLen);
                    ret = recv(fd,retBuffer+recvLen,retBufferLen-recvLen,0);
                    Trace(1,"ret:%d",ret);
                    recvLen += ret;
                    if(ret < 0)
                    {
                        Trace(2,"recv error");
                        flag = true;
                        break;
                    }
                    else if(ret == 0)
                    {
                        Trace(2,"ret == 0");
                        break;
                    }
                    else if(ret < 1352)
                    {
                        Trace(1,"recv len:%d,data:%s",recvLen,retBuffer);
                        GPS_DEBUG_I("recv len:%d,data:%s",recvLen,retBuffer);
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

bool callback_network_info(bool result){
    if(result){
          semGetCellInfo = OS_CreateSemaphore(0);
            if(!Network_GetCellInfoRequst())
                {
                        Trace(1,"network get cell info fail");
                }
            OS_WaitForSemaphore(semGetCellInfo,OS_TIME_OUT_WAIT_FOREVER);
            OS_DeleteSemaphore(semGetCellInfo);
        semGetCellInfo = NULL;
    } 
    
  
}

bool google_geolocation_get_route(int number){
    switch (number)
            {
                Trace(10, "NETWORK CELL LIST : %d",number);
                case 1 :
                        snprintf(cell_tower_path_buffer,cell_tower_path_buffer_len,
                        "/api/device-control/calculateLoc?mcc=%d%d%d&mnc=%d&radioType=%s&carrier=%s&considerIp=%d&cellTowers=%d;%d;%d", 
                        location[0].sMcc[0], location[0].sMcc[1], location[0].sMcc[2],
                        location[0].sMnc[1],"GSM","mpt",false,
                        location[0].sCellID,location[0].sLac,location[0].iRxLev);
                        Trace(10,cell_tower_path_buffer);
                        return true;
                break;
                case 2 : 
                        snprintf(cell_tower_path_buffer,cell_tower_path_buffer_len,
                        "/api/device-control/calculateLoc?mcc=%d%d%d&mnc=%d&radioType=%s&carrier=%s&considerIp=%d&cellTowers=%d;%d;%d,%d;%d;%d", 
                        location[0].sMcc[0], location[0].sMcc[1], location[0].sMcc[2],
                        location[0].sMnc[1],"GSM","mpt",false,
                        location[0].sCellID,location[0].sLac,location[0].iRxLev,
                        location[1].sCellID,location[1].sLac,location[1].iRxLev);
                        Trace(10,cell_tower_path_buffer);
                        return true;
                break;
                case 3 :
                        snprintf(cell_tower_path_buffer,cell_tower_path_buffer_len,
                        "/api/device-control/calculateLoc?mcc=%d%d%d&mnc=%d&radioType=%s&carrier=%s&considerIp=%d&cellTowers=%d;%d;%d,%d;%d;%d,%d;%d;%d", 
                        location[0].sMcc[0], location[0].sMcc[1], location[0].sMcc[2],
                        location[0].sMnc[1],"GSM","mpt",false,
                        location[0].sCellID,location[0].sLac,location[0].iRxLev,
                        location[1].sCellID,location[1].sLac,location[1].iRxLev,
                        location[2].sCellID,location[2].sLac,location[2].iRxLev);
                        Trace(10,cell_tower_path_buffer);
                        return true;
                break;
                case 4 :
                        snprintf(cell_tower_path_buffer,cell_tower_path_buffer_len,
                        "/api/device-control/calculateLoc?mcc=%d%d%d&mnc=%d&radioType=%s&carrier=%s&considerIp=%d&cellTowers=%d;%d;%d,%d;%d;%d,%d;%d;%d,%d;%d;%d", 
                        location[0].sMcc[0], location[0].sMcc[1], location[0].sMcc[2],
                        location[0].sMnc[1],"GSM","mpt",false,
                        location[0].sCellID,location[0].sLac,location[0].iRxLev,
                        location[1].sCellID,location[1].sLac,location[1].iRxLev,
                        location[2].sCellID,location[2].sLac,location[2].iRxLev,
                        location[3].sCellID,location[3].sLac,location[3].iRxLev);
                        Trace(10,cell_tower_path_buffer);
                        return true;

            }
}

void gps_testTask(void *pData)
{
    GPS_Info_t* gpsInfo = Gps_GetInfo();
    GPIO_config_t SOS_BTN =
    {
        .mode = GPIO_MODE_INPUT,
        .pin = GPIO_PIN_SOS,
        .defaultLevel = GPIO_LEVEL_HIGH,
    };
    GPIO_LEVEL BTN_STATE = 0; 
    GPIO_Init(SOS_BTN); 
    while(!networkFlag)
    {
        GPIO_GetLevel(SOS_BTN,&BTN_STATE); 
        Trace(10,"GPRS BTN STATE : %d",BTN_STATE);
        if(BTN_STATE == GPIO_LEVEL_LOW){
         Trace(10,"GPRS CONNECTION RESTART IN 4 SECONDS");
            GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_HIGH);
            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
            OS_Sleep(1000);
            GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_LOW);
            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
            OS_Sleep(1000);
            GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_HIGH);
            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
            OS_Sleep(1000);
            GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_LOW);
            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
            OS_Sleep(1000);
            PM_Restart();
        }
        Trace(1,"wait for gprs regiter complete");
        GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_HIGH);
        OS_Sleep(2000);
      
    }
   

    
    GPS_Init();
    GPS_SaveLog(true,GPS_NMEA_LOG_FILE_PATH);
    GPS_Open(NULL);
    while(gpsInfo->rmc.latitude.value == 0)
        OS_Sleep(1000);
    for(uint8_t i = 0;i<5;++i)
    {
        bool ret = GPS_SetOutputInterval(10000);
        Trace(1,"set gps ret:%d",ret);
        if(ret)
            break;
        OS_Sleep(1000);
    }
    if(!GPS_GetVersion(buffer,150))
        Trace(1,"get gps firmware version fail");
    else
        Trace(1,"gps firmware version:%s",buffer);
  

   //get location through LBS
    semGetCellInfo = OS_CreateSemaphore(0);
    if(!Network_GetCellInfoRequst())
    {
        Trace(1,"network get cell info fail");
    }
    OS_WaitForSemaphore(semGetCellInfo,OS_TIME_OUT_WAIT_FOREVER);
    OS_DeleteSemaphore(semGetCellInfo);
    semGetCellInfo = NULL;


    if(!GPS_SetOutputInterval(1000))
        Trace(1,"set nmea output interval fail");
    
    Trace(1,"init ok");

    GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_LOW);
    while(1)
    {
        if(isGpsOn)
        {
            uint8_t isFixed = gpsInfo->gsa[0].fix_type > gpsInfo->gsa[1].fix_type ?gpsInfo->gsa[0].fix_type:gpsInfo->gsa[1].fix_type;
            char* isFixedStr = NULL; 
            if(isFixed == 2)
                isFixedStr = "2D fix";
            else if(isFixed == 3)
            {
                if(gpsInfo->gga.fix_quality == 1)
                    isFixedStr = "3D fix";
                else if(gpsInfo->gga.fix_quality == 2)
                    isFixedStr = "3D/DGPS fix";
            }
            else
                isFixedStr = "no fix";
 
            int temp = (int)(gpsInfo->rmc.latitude.value/gpsInfo->rmc.latitude.scale/100);
            double latitude = temp+(double)(gpsInfo->rmc.latitude.value - temp*gpsInfo->rmc.latitude.scale*100)/gpsInfo->rmc.latitude.scale/60.0;
            temp = (int)(gpsInfo->rmc.longitude.value/gpsInfo->rmc.longitude.scale/100);
            double longitude = temp+(double)(gpsInfo->rmc.longitude.value - temp*gpsInfo->rmc.longitude.scale*100)/gpsInfo->rmc.longitude.scale/60.0;

            snprintf(buffer,sizeof(buffer),"GPS fix mode:%d, BDS fix mode:%d, fix quality:%d, satellites tracked:%d, gps sates total:%d, is fixed:%s, coordinate:WGS84, Latitude:%f, Longitude:%f, unit:degree,altitude:%f",gpsInfo->gsa[0].fix_type, gpsInfo->gsa[1].fix_type,
                                                                gpsInfo->gga.fix_quality,gpsInfo->gga.satellites_tracked, gpsInfo->gsv[0].total_sats, isFixedStr, latitude,longitude,gpsInfo->gga.altitude);
            Trace(1,buffer);

            //send to server
            char* requestPath = buffer2;
            uint8_t percent;
            uint16_t v = PM_Voltage(&percent);
            Trace(1,"power:%d %d",v,percent);
            if(percent<20){
                Trace(10,"!!LOW POWER PLEASE CHARGE!!");
                 GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_HIGH);
                 OS_Sleep(200);
                 GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_LOW);
            }
            snprintf(requestPath, sizeof(buffer2), "/api/device-control/645c55478a04726ce6575fa1/alert?lat=%f&lon=%f",latitude, longitude); // device id you can change here
         
            uint8_t status;
            GPIO_LEVEL BTN_STATE = 0; // BTN_STATE is SOS BUTTON STATE
            GPIO_GetLevel(SOS_BTN,&BTN_STATE);  //v1.4  add gprs restart connection state
            Trace(10,"SOS BTN STATE : %d",BTN_STATE);
            Network_GetActiveStatus(&status); // get active status for gprs network connection

        if(!status){
                GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
                OS_Sleep(200);
                GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                Trace(10,"no internet connection please check"); // add status led here
            Trace(10,"GPRS BTN STATE : %d",BTN_STATE);
            if(BTN_STATE == GPIO_LEVEL_LOW){
                OS_Sleep(2000);
                Trace(10,"GPRS CONNECTION RESTART IN 4 SECONDS");
                GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_HIGH);
                GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
                OS_Sleep(1000);
                GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_LOW);
                GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                OS_Sleep(1000);
                GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_HIGH);
                GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
                OS_Sleep(1000);
                GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_LOW);
                GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                OS_Sleep(1000);
                PM_Restart();
        }       
    } // check the internet connection
    
  
    
    if(BTN_STATE == GPIO_LEVEL_HIGH)
             btn_active = true;  
        if(BTN_STATE == GPIO_LEVEL_LOW){
           clock_t timeStart = clock();
           btn_press_time++;
           PM_SetSysMinFreq(PM_SYS_FREQ_312M); // high performance --> v1.4
           sos_btn_event = true;
        clock_t timeEnd   = clock();
        } // check the sos button event 


        // v 1.3 stand by mode when btn pressed , finding gps signal , gps signal is found send gps data to cloud.
        if(sos_btn_event == true){
            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
            callback_network_info(true);
            Trace(1,"CELL NUMBER SCHEDULER CLICK EVENT: %d",number);
            for(int i=0;i<number;++i)
                    {
                        Trace(2,"[GPS] cell %d info:%d%d%d,%d%d%d,%d,%d,%d,%d,%d,%d",i,
                        location[i].sMcc[0], location[i].sMcc[1], location[i].sMcc[2], 
                        location[i].sMnc[0], location[i].sMnc[1], location[i].sMnc[2],
                        location[i].sLac, location[i].sCellID, location[i].iBsic,
                        location[i].iRxLev, location[i].iRxLevSub, location[i].nArfcn);
                    }

    if(google_geolocation_get_route(number));
        Trace(1,"[GET GPS] recieve");

            if((latitude == 0.00 && longitude == 0.00)||(latitude == 90.0 && longitude == 0.00)){

            if(Http_Post(SERVER_IP,SERVER_PORT,cell_tower_path_buffer,NULL,0,buffer_tower,sizeof(buffer_tower) ,true) < 0)
                    Trace(1,"send location to server fail");
            else
                    {
                        Trace(1,"http get success,ret:%s",buffer_tower);
                        char* index0 = strstr(buffer_tower,"\r\n\r\n");
                        char temp = index0[4];
                        index0[4] = '\0';
                        Trace(1,"http response header:%s",buffer_tower);
                        index0[4] = temp;
                        Trace(1,"http response body:%s",index0+4); 
                        Trace(1,"show index : %s",index0+1);
                        Trace(1,"INDEX : %s",index0+3);
                        char geolocatoin[1024];
                        snprintf(geolocatoin,sizeof(geolocatoin),"%s",index0+4);
                        Trace(1,geolocatoin);
                       

                        json_t mem[32];
                        json_t const* json = json_create(geolocatoin,mem,sizeof(mem) / sizeof *mem);
                        if(!json) Trace(1,"error json create");
                        else Trace(1,"json create");

                        json_t const* locatoin_ = json_getProperty(json,"location");
                        if(locatoin_ == NULL) Trace(1,"locatoin retrieve error");
                        char const* lbs_latitude = json_getPropertyValue(locatoin_,"lat");
                        Trace(1,"Latitude : %s",lbs_latitude);
                        char const* lbs_longitude = json_getPropertyValue(locatoin_,"lng");
                        Trace(1,"Longitude : %s",lbs_longitude);
                        char const* accuracy_value = json_getPropertyValue(json,"accuracy");
                        Trace(1,"Accuaracy : %s",accuracy_value);
                   

                        if(status) {
                        GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
                        snprintf(cell_request_path, sizeof(buffer2), "/api/device-control/645c55478a04726ce6575fa1/alert?lat=%s&lon=%s",lbs_latitude,lbs_longitude);
                   

                      if(Http_Post(SERVER_IP,SERVER_PORT,cell_request_path,NULL,0,buffer,sizeof(buffer),false) < 0)
                        Trace(1,"send location to server fail");
                      else
                        {
                            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                            sos_btn_event = false;
                            Trace(1,"send location to server success");
                            Trace(1,"response:%s",buffer);
                            snprintf(buffer,sizeof(buffer),"close");
                            UART_Write(UART1,buffer,strlen(buffer));
                            UART_Write(UART1,"\r\n\r\n",4);
                        }
                    } else {
                        Trace(10,"no internet");
                    }
               
        } // lbs location 
            Trace(10,"GPS ERROR");
            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
            OS_Sleep(100);
            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
           } // lat and long  are zero 
            else  {
            if(isFixed >=2){
                 if(status)
                {   
                     count = 0; 
                     GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
                      if(Http_Post(SERVER_IP,SERVER_PORT,requestPath,NULL,0,buffer,sizeof(buffer),false) < 0)
                        Trace(1,"send location to server fail");
                      else
                        {
                            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                            sos_btn_event = false;
                            Trace(1,"send location to server success");
                            Trace(1,"response:%s",buffer);
                            snprintf(buffer,sizeof(buffer),"close");
                            UART_Write(UART1,buffer,strlen(buffer));
                            UART_Write(UART1,"\r\n\r\n",4);
                        }
                } else {
                        Trace(10,"[SOS] NO INTERNET");
                        Trace(10,"GPRS CONNECTION RESTART IN 4 SECONDS");
                        GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_HIGH);
                        GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
                        OS_Sleep(1000);
                        GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_LOW);
                        GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                        OS_Sleep(1000);
                        GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_HIGH);
                        GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
                        OS_Sleep(1000);
                        GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_LOW);
                        GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                        OS_Sleep(1000);
                        // restart for gprs connection in 1 second
                        PM_Restart();
                }
                PM_SetSysMinFreq(PM_SYS_FREQ_13M); 
           
        } else {
            count++;
            Trace(10,"COUNTER : %d",count); // add counter in v1.3 (stand by mode)
            Trace(10,"NO FIX");
            if(count > 99){ // v1.4 gps time out state;
                Trace(10,"GPS TIME OUT RESTART AGAIN !! EMEGENCY !!");
                 if(status)
                { 
                         GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
                if(Http_Post(SERVER_IP,SERVER_PORT,cell_tower_path_buffer,NULL,0,buffer_tower,sizeof(buffer_tower) ,true) < 0)
                    Trace(1,"send location to server fail");
            else
                    {
                        Trace(1,"http get success,ret:%s",buffer_tower);
                        char* index0 = strstr(buffer_tower,"\r\n\r\n");
                        char temp = index0[4];
                        index0[4] = '\0';
                        Trace(1,"http response header:%s",buffer_tower);
                        index0[4] = temp;
                        Trace(1,"http response body:%s",index0+4); 
                        Trace(1,"show index : %s",index0+1);
                        Trace(1,"INDEX : %s",index0+3);
                        char geolocatoin[1024];
                        snprintf(geolocatoin,sizeof(geolocatoin),"%s",index0+4);
                        Trace(1,geolocatoin);
                       

                        json_t mem[32];
                        json_t const* json = json_create(geolocatoin,mem,sizeof(mem) / sizeof *mem);
                        if(!json) Trace(1,"error json create");
                        else Trace(1,"json create");

                        json_t const* locatoin_ = json_getProperty(json,"location");
                        if(locatoin_ == NULL) Trace(1,"locatoin retrieve error");
                        char const* lbs_latitude = json_getPropertyValue(locatoin_,"lat");
                        Trace(1,"Latitude : %s",lbs_latitude);
                        char const* lbs_longitude = json_getPropertyValue(locatoin_,"lng");
                        Trace(1,"Longitude : %s",lbs_longitude);
                        char const* accuracy_value = json_getPropertyValue(json,"accuracy");
                        Trace(1,"Accuaracy : %s",accuracy_value);
                   

                        if(status) {
                        GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
                        snprintf(cell_request_path, sizeof(buffer2), "/api/device-control/645c55478a04726ce6575fa1/alert?lat=%s&lon=%s",lbs_latitude,lbs_longitude);
                      if(Http_Post(SERVER_IP,SERVER_PORT,cell_request_path,NULL,0,buffer,sizeof(buffer),false) < 0)
                        Trace(1,"send location to server fail");
                      else
                        {
                            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                            sos_btn_event = false;
                            Trace(1,"send location to server success");
                            Trace(1,"response:%s",buffer);
                            snprintf(buffer,sizeof(buffer),"close");
                            UART_Write(UART1,buffer,strlen(buffer));
                            UART_Write(UART1,"\r\n\r\n",4);
                        }
                    } else {
                        Trace(10,"no internet");
                    }
               
        }

        OS_Sleep(500);
        Trace(1,"DO NO FIX GPS SIGNAL : lat : %d, lng : %d",latitude,longitude);  

                    // v1.4 set gps time state to 0
                     GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
                    // gprs connection is pretty fine send data to cloud
                      if(Http_Post(SERVER_IP,SERVER_PORT,requestPath,NULL,0,buffer,sizeof(buffer) ,false) < 0)
                        Trace(1,"send location to server fail");
                      else
                        {
                            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                            sos_btn_event = false;
                            Trace(1,"send location to server success");
                            Trace(1,"response:%s",buffer);
                            count = 0;

                            Trace(10, "GPS TIME OUT STATE TO 0");
                        }
                } else {
                    // v1.4 restart gprs connection
                        Trace(10,"[SOS] NO INTERNET");
                        Trace(10,"GPRS CONNECTION RESTART IN 4 SECONDS");
                        GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_HIGH);
                        GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
                        OS_Sleep(1000);
                        GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_LOW);
                        GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                        OS_Sleep(1000);
                        GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_HIGH);
                        GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
                        OS_Sleep(1000);
                        GPIO_Set(SYSTEM_STATUS_LED,GPIO_LEVEL_LOW);
                        GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
                        OS_Sleep(1000);
                        // restart for gprs connection in 1 second
                        PM_Restart();
                }
                count = 0;
            }
            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_HIGH);
            OS_Sleep(400);
            GPIO_Set(UPLOAD_DATA_LED,GPIO_LEVEL_LOW);
        } // fix 2d fix end 
} // if(lat and long are zero)
    } // end btn true 
        } // end gps on 
    OS_Sleep(500);
    } // end while 
}




void LED_Blink(void* param)
{
    static int count_ = 0;
    if(++count_ == 5)
    {
  

    switch (number)
            {
                Trace(10, "NETWORK CELL LIST : %d",number);
                case 1 :
                        snprintf(cell_tower_path_buffer,cell_tower_path_buffer_len,
                        "/api/device-control/calculateLoc?mcc=%d%d%d&mnc=%d&radioType=%s&carrier=%s&considerIp=%d&cellTowers=%d;%d;%d", 
                        location[0].sMcc[0], location[0].sMcc[1], location[0].sMcc[2],
                        location[0].sMnc[1],"GSM","mpt",false,
                        location[0].sCellID,location[0].sLac,location[0].iRxLev);
                          Trace(10,cell_tower_path_buffer);
                break;
                case 2 : 
                        snprintf(cell_tower_path_buffer,cell_tower_path_buffer_len,
                        "/api/device-control/calculateLoc?mcc=%d%d%d&mnc=%d&radioType=%s&carrier=%s&considerIp=%d&cellTowers=%d;%d;%d,%d;%d;%d", 
                        location[0].sMcc[0], location[0].sMcc[1], location[0].sMcc[2],
                        location[0].sMnc[1],"GSM","mpt",false,
                        location[0].sCellID,location[0].sLac,location[0].iRxLev,
                        location[1].sCellID,location[1].sLac,location[1].iRxLev);
                          Trace(10,cell_tower_path_buffer);
                break;
                case 3 :
                        snprintf(cell_tower_path_buffer,cell_tower_path_buffer_len,
                        "/api/device-control/calculateLoc?mcc=%d%d%d&mnc=%d&radioType=%s&carrier=%s&considerIp=%d&cellTowers=%d;%d;%d,%d;%d;%d,%d;%d;%d", 
                        location[0].sMcc[0], location[0].sMcc[1], location[0].sMcc[2],
                        location[0].sMnc[1],"GSM","mpt",false,
                        location[0].sCellID,location[0].sLac,location[0].iRxLev,
                        location[1].sCellID,location[1].sLac,location[1].iRxLev,
                        location[2].sCellID,location[2].sLac,location[2].iRxLev);
                          Trace(10,cell_tower_path_buffer);
                break;
                case 4 :
                        snprintf(cell_tower_path_buffer,cell_tower_path_buffer_len,
                        "/api/device-control/calculateLoc?mcc=%d%d%d&mnc=%d&radioType=%s&carrier=%s&considerIp=%d&cellTowers=%d;%d;%d,%d;%d;%d,%d;%d;%d,%d;%d;%d", 
                        location[0].sMcc[0], location[0].sMcc[1], location[0].sMcc[2],
                        location[0].sMnc[1],"GSM","mpt",false,
                        location[0].sCellID,location[0].sLac,location[0].iRxLev,
                        location[1].sCellID,location[1].sLac,location[1].iRxLev,
                        location[2].sCellID,location[2].sLac,location[2].iRxLev,
                        location[3].sCellID,location[3].sLac,location[3].iRxLev);
                        Trace(10,cell_tower_path_buffer);

            }
    semGetCellInfo = NULL;
    }
    else if(count_ == 6)
    {
       
        count_ = 0;
    }
  //  OS_StartCallbackTimer(gpsTaskHandle,1000,LED_Blink,NULL);
}






void gps_MainTask(void *pData)
{
    API_Event_t* event=NULL;
    
    TIME_SetIsAutoUpdateRtcTime(true);
    
    //open UART1 to print NMEA infomation
    UART_Config_t config = {
        .baudRate = UART_BAUD_RATE_115200,
        .dataBits = UART_DATA_BITS_8,
        .stopBits = UART_STOP_BITS_1,
        .parity   = UART_PARITY_NONE,
        .rxCallback = NULL,
        .useEvent   = true
    };

    GPIO_config_t gpioLedBlue = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin          = SYSTEM_STATUS_LED,
        .defaultLevel = GPIO_LEVEL_LOW
    };

    GPIO_config_t gpioLedUpload = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin          = UPLOAD_DATA_LED,
        .defaultLevel = GPIO_LEVEL_LOW
    };
    //PM_PowerEnable(POWER_TYPE_VPAD,true);
    GPIO_Init(gpioLedBlue);
    GPIO_Init(gpioLedUpload);
    UART_Init(UART1,config);

    OS_CreateTask(gps_testTask,
            NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
  //  OS_StartCallbackTimer(gpsTaskHandle,1000,LED_Blink,NULL);
    //Wait event
    while(1)
    {
        if(OS_WaitEvent(gpsTaskHandle, (void**)&event, OS_TIME_OUT_WAIT_FOREVER))
        {
            EventDispatch(event);
            OS_Free(event->pParam1);
            OS_Free(event->pParam2);
            OS_Free(event);
        }
    }
}


void gps_agps_Main(void)
{
    gpsTaskHandle = OS_CreateTask(gps_MainTask,
        NULL, NULL, MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY, 0, 0, MAIN_TASK_NAME);
    OS_SetUserMainHandle(&gpsTaskHandle);
}