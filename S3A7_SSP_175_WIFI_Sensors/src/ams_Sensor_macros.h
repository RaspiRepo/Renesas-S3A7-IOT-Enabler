/*----------------------------------------------------------------------------
    ams_sensor_macros.h : AMS Demo application commaon macros

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/

#ifndef AMS_SENSOR_MACROS_H_
#define AMS_SENSOR_MACROS_H_




#define MAX_NUM_HEALTH_MESSGAES             3

#define TIME_SPLASH_SCREEN_MAX_WAIT_SECS  3
#define TIME_M1_UPDATE_INTERVAL_SECS      60
#define TICK_PER_SECS                     50


#define TIME_EVENT_TIMER_DISPLAY        (100)
#define TIME_EVENT_TIMER_SENSOR_TO_M1   (101)
#define TIME_EVENT_TIMER_BOARD_HEALTH   (102)
#define TIME_EVENT_TIMER_ETHERNET_CHECK (103)
#define TIME_EVENT_SWITCH_AMS_SCREEN    (104)
#define TIME_EVENT_TIMER_SPLASH_WAIT    (105)
#define TIME_EVENT_TIMER_SPLASH_COUNTER (106)
#define TIME_EVENT_TIMER_PROVISION       107

#define GX_EVENT_SWITCH_PROVISION_WINDOW (107)


#define GX_EVENT_ETH0_LINK_CHECK         (201)
#define GX_EVENT_ETH0_DHCP_START         (202)
#define GX_EVENT_ETH0_IP_RESOLVED        (203)
#define GX_EVENT_ETH0_CONNECTED          (204)
#define GX_EVENT_ETH0_DNS_CHECK          (205)
#define GX_EVENT_ETH0_DNS_CHECK          (205)
#define GX_EVENT_ETH0_DNS_CHECK_SUCCESS  (206)


#define GX_EVENT_WIFI_LINK_CHECK         (250)
#define GX_EVENT_WIFI_DHCP_START         (251)
#define GX_EVENT_WIFI_IP_RESOLVED        (252)
#define GX_EVENT_WIFI_CONNECTED          (253)
#define GX_EVENT_WIFI_DNS_CHECK          (254)
#define GX_EVENT_WIFI_DNS_CHECK_SUCCESS  (255)
#define GX_EVENT_WIFI_PROVISIONING       (256)
#define GX_EVENT_WIFI_PROVI_SUCCESS      (257)
#define GX_EVENT_SWITCH_TO_PROVISION_MODE (258)


#define GX_EVENT_SSID_DISPLAY               (1000)
#define GX_EVENT_PROVISION_END              (1001)
#define GX_EVENT_PROVISIONING               (1002)
#define GX_EVENT_DISPLAY_WIFI_ACCESS_INFO   (1003)

//#define _DNS_CHECK_ENABLE_                1
//#define _WIFI_M1_CLOUD_CONNECT_ENABLE_      0
//#define _ETH0_M1_CLOUD_CONNECT_ENABLE_      0

//M1 Cloud related events
#define GX_EVENT_M1_REFRESH_UPDATE           (300)
#define GX_EVENT_M1_CONNECT                  (301)
#define GX_EVENT_M1_CONNECT_SUCCESS          (302)
#define GX_EVENT_M1_CONNECT_FAILURE          (303)


#define GX_EVENT_ETH0_HTTP_SUCCESS           (400)
#define GX_EVENT_ETH0_HTTP_DOWNLOAD_SUCCESS  (401)
#define GX_EVENT_ETH0_HTTP_DOWNLOAD_FAILED   (402)


#define DHCP_STACK_SIZE                           (2  * 1024)
#define NETWORK_PACKETPOOL_BLOCK_SIZE                  (1568)
#define NETWORK_NUM_IP_PACKETS                         (10)

#define OPEN_MAP_WEATHER_URL                  "api.openweathermap.org"

#define ATT_M2X_CLOUD_API                      "api-m2x.att.com"

#define WIFI24GHZ_SSID       "WIFI_SSID"
#define WIFI24GHZ_SSID_PSW   "PSW"
#define DATA_FLASH_PROGRAMMING_UNIT (4)
#define DATA_FLASH_BLOCK_SIZE       (64)

/*
#define M1_APIKEY            "ULCNQZYRAPLQIPPIBSEQKBRQGVQWEY3BME3DAMRQGQ4TAMBQ"
#define M1_MQTT_USER_ID      "0mI-xGkTZAc"
#define M1_MQTT_PROJECT_ID   "HmMMmpO3lWA"
#define M1_PASSWORD          "IRCvNiIf"
*/
#define DHCP_SERVER_START_IP  IP_ADDRESS(192,168,3,100)
#define DHCP_SERVER_END_IP    IP_ADDRESS(192,168,3,110)
#define GATEWAY_ADDR          IP_ADDRESS(192, 168, 3, 1)

#define  TOKEN_DELIMITER    '&'

void post_event_gui (int event, void *target_widget);

#endif /* AMS_SENSOR_MACROS_H_ */
