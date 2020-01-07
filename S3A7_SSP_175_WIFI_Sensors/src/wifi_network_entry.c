#include "wifi_network.h"
#include <time.h>
#include <nx_api.h>
#include <nx_dns.h>
#include <nx_dhcp.h>

#include "ams_Sensor_macros.h"


NX_PACKET_POOL g_wifi_packet_pool;

NX_IP          g_ip_wifi;
NX_DHCP        g_dhcp_client_wifi;
NX_DNS         g_dns_wifi_client;

static CHAR mem_arp[1024] __attribute__ ((aligned(4)));
uint8_t g_wifi_ip_stack_memory[2048];

uint8_t g_packet_pool_wifi_pool_memory[(2 * 1568)];
//uint8_t g_packet_pool_dhcp_pool_memory[(10 * 1568)];
//uint8_t g_packet_pool_http_memory[(16 * 1568)];

sf_wifi_provisioning_t g_provision_info;
sf_wifi_provisioning_t g_active_provision_info;

//local variables
char wifi_ipstr[30];
bool wifi_provision_mode = false;
ULONG wifi_ip_address;


void write_display_console (int id, CHAR *msg_string)
/*----------------------------------------------------------------------------
    write_display_console: Dummy display function

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{

}



int8_t get_provisioning_from_flash ()
/*----------------------------------------------------------------------------
    get_provisioning_from_flash
                        : Get previously stored wifi/m1 clound connection
                          information from flash memeory

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    ssp_err_t ssp_err;
    UCHAR provisionConfigBuffer[300];
    int8_t num_tokens = 7;

    ssp_err = g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);
    ssp_err = g_flash0.p_api->read(g_flash0.p_ctrl, provisionConfigBuffer, 0x40100000, 199);

    ssp_err = g_flash0.p_api->close(g_flash0.p_ctrl);

    char *token_ptr = (char *)provisionConfigBuffer;
    char *str_ptr   = token_ptr;

    size_t key_str_len = strlen("ssid=");

    //Tokenization of provision string with &
    //first token from provision HTML page SSID
    token_ptr = strchr((char *)provisionConfigBuffer, TOKEN_DELIMITER);
    if (strncmp(str_ptr, "ssid=", key_str_len) == 0) {
        str_ptr = str_ptr + key_str_len;
        strncpy((char *)&g_provision_info.ssid[0], str_ptr, (token_ptr - str_ptr));
        --num_tokens;
    }

    //next token is Security
    str_ptr   = token_ptr + 1;
    token_ptr = strchr(str_ptr, TOKEN_DELIMITER);
    if (strncmp(str_ptr, "sec=wpa2", strlen("sec=wpa2")) == 0) {
        g_provision_info.security = SF_WIFI_SECURITY_TYPE_WPA2;
        --num_tokens;
    } else if (strncmp(str_ptr, "sec=wpa", strlen("sec=wpa2")) == 0) {
        g_provision_info.security = SF_WIFI_SECURITY_TYPE_WPA;
        --num_tokens;
    } else if (strncmp(str_ptr, "sec=open", strlen("sec=open")) == 0) {
        g_provision_info.security = SF_WIFI_SECURITY_TYPE_OPEN;
        --num_tokens;
    } else if (strncmp(str_ptr, "sec=wep", strlen("sec=wep")) == 0) {
        g_provision_info.security = SF_WIFI_SECURITY_TYPE_WEP;
    }


    str_ptr = token_ptr + 1;
    token_ptr = strchr(str_ptr, TOKEN_DELIMITER);
    key_str_len = strlen("key=");
    if (strncmp(str_ptr, "key=", key_str_len) == 0) {
        str_ptr = str_ptr + key_str_len;
        strncpy((char *)&g_provision_info.key[0], str_ptr, (token_ptr - str_ptr));
        --num_tokens;
    }

    key_str_len = strlen("mqttprojectid=");
    str_ptr = token_ptr + 1;
    token_ptr = strchr(str_ptr, TOKEN_DELIMITER);
    if (strncmp(str_ptr, "mqttprojectid=", key_str_len) == 0) {
        str_ptr = str_ptr + key_str_len;
//        strncpy((char *)&project_cred.proj_id[0], str_ptr, (token_ptr - str_ptr));
        --num_tokens;
    }

    key_str_len = strlen("mqttuserid=");
    str_ptr = token_ptr + 1;
    token_ptr = strchr(str_ptr, TOKEN_DELIMITER);
    if (strncmp(str_ptr, "mqttuserid=", key_str_len) == 0) {
        str_ptr = str_ptr + key_str_len;
//        strncpy((char *)&user_authn_cred.user_id[0], str_ptr, (token_ptr - str_ptr));
        --num_tokens;
    }

    key_str_len = strlen("apikey=");
    str_ptr = token_ptr + 1;
    token_ptr = strchr(str_ptr, TOKEN_DELIMITER);
    if (strncmp(str_ptr, "apikey=", key_str_len) == 0) {
        str_ptr = str_ptr + key_str_len;
//        strncpy((char *)&project_cred.apikey[0], str_ptr, (token_ptr - str_ptr));
        --num_tokens;
    }

    key_str_len = strlen("password=");
    str_ptr = token_ptr + 1;
    token_ptr = strchr(str_ptr, TOKEN_DELIMITER);
    if (strncmp(str_ptr, "password=", key_str_len) == 0) {
        str_ptr = str_ptr + key_str_len;
//        strcpy((char *)&user_authn_cred.password[0], str_ptr);
        --num_tokens;
    }

//    strcpy((char *)device.user_id, (char *)user_authn_cred.user_id);
//    strcpy((char *)device.password, (char *)user_authn_cred.password);

    return num_tokens;
}





void initilize_wifi_network ()
/*----------------------------------------------------------------------------
    initilize_wifi_network
                        : Wait for link to be enabled, Do the Wifi access
                          provisioning and start DHCP service to resolved
                          IP address.

    Written By          : RaspiRepo (mariya.k@gmail.com)
                          https://github.com/RaspiRepo

    Address             : Mountain View, CA 94040
    Date                : May 2017
------------------------------------------------------------------------------*/
{
    UINT ip_status = 0;
    ssp_err_t err_code = NX_SUCCESS;

    CHAR *ip_name       = NULL;
    ULONG actual_status = NX_IP_INITIALIZE_DONE;
    ULONG link_status   = NX_IP_INITIALIZE_DONE;

    snprintf(wifi_ipstr, sizeof(wifi_ipstr), "WiFi   : %s", "----------");
    write_display_console(0, wifi_ipstr);

    snprintf(wifi_ipstr, sizeof(wifi_ipstr), "IP     : Resolving...");
    write_display_console(1, wifi_ipstr);

    snprintf(wifi_ipstr, sizeof(wifi_ipstr), "Indoor:-----------------");
    write_display_console(2, wifi_ipstr);

    err_code = nx_packet_pool_create (&g_wifi_packet_pool, "g_packet_pool_wifi", 1568,
                                                    &g_packet_pool_wifi_pool_memory[0], (2 * 1568));

    //Create WiFi IP instance
    err_code = nx_ip_create (&g_ip_wifi, "g_wifi_ip Instance", IP_ADDRESS (0, 0, 0, 0),
                                    IP_ADDRESS (255, 255, 255, 0),
                                    &g_wifi_packet_pool, g_sf_el_nx0,
                                    &g_wifi_ip_stack_memory[0], 2048, 3);

    err_code = nx_ip_fragment_enable(&g_ip_wifi);
    err_code = nx_arp_enable(&g_ip_wifi, mem_arp, sizeof(mem_arp));

    err_code = nx_tcp_enable(&g_ip_wifi);
    err_code =  nx_udp_enable(&g_ip_wifi);
    err_code = nx_icmp_enable(&g_ip_wifi);

    //post_event_gui(GX_EVENT_WIFI_LINK_CHECK, 0);
    err_code = nx_ip_interface_status_check(&g_ip_wifi, 0, NX_IP_LINK_ENABLED, &ip_status, NX_WAIT_FOREVER);

    //post_event_gui(GX_EVENT_WIFI_PROVISIONING, 0);
    g_provision_info.mode     = SF_WIFI_INTERFACE_MODE_CLIENT;
    g_provision_info.channel  = (uint8_t)11;//(rand() % 9); //random channel

    g_provision_info.security = SF_WIFI_SECURITY_TYPE_WPA2;
    get_provisioning_from_flash();

    //uncomment below if you want to take default from global constant
    //strcpy(g_provision_info.ssid, WIFI24GHZ_SSID);
    //strcpy(g_provision_info.key, WIFI24GHZ_SSID_PSW);

    switch (g_provision_info.security) {
        case SF_WIFI_SECURITY_TYPE_WPA2:
            g_provision_info.encryption = SF_WIFI_ENCRYPTION_TYPE_CCMP;
            break;
        case SF_WIFI_SECURITY_TYPE_WPA:
            g_provision_info.encryption = SF_WIFI_ENCRYPTION_TYPE_TKIP;
            break;
        default:
            g_provision_info.encryption = SF_WIFI_ENCRYPTION_TYPE_AUTO;
    }
    ssp_err_t result = SSP_ERR_IN_USE;
    while (result != SSP_SUCCESS) {
        result = g_sf_wifi0.p_api->provisioningSet(g_sf_wifi0.p_ctrl, &g_provision_info);
    }
    //post_event_gui(GX_EVENT_WIFI_PROVI_SUCCESS, 0);

    //create DHCP client and  start to retrieve IP address
    err_code = nx_dhcp_create (&g_dhcp_client_wifi, &g_ip_wifi, "S3A7DHCP");
    err_code = nx_dhcp_start(&g_dhcp_client_wifi);

    //Check IP address resolved from router
    err_code = nx_ip_status_check(&g_ip_wifi, NX_IP_ADDRESS_RESOLVED, &ip_status, TX_WAIT_FOREVER);

    result = SSP_ERR_IN_USE;
    while (result != SSP_SUCCESS) {
        result = g_sf_wifi0.p_api->provisioningGet(g_sf_wifi0.p_ctrl, &g_active_provision_info);
    }

    //post_event_gui(GX_EVENT_WIFI_IP_RESOLVED, 0);

    // nx_ip_interface_address_get(NX_IP *ip_ptr, ULONG interface_index, ULONG *ip_address, ULONG *network_mask);
    while (wifi_ip_address == 0) {
        nx_ip_interface_info_get(&g_ip_wifi, 0, &ip_name, &wifi_ip_address, NULL, NULL, NULL, NULL);
    }
    //post_event_gui(GX_EVENT_WIFI_CONNECTED, 0);

//    //IP_ADDRESS(162,243,53,59)
    err_code = nx_dns_create (&g_dns_wifi_client, &g_ip_wifi, (UCHAR *) "g_wifidns_client");
    nx_dns_packet_pool_set(&g_dns_wifi_client, (NX_PACKET_POOL *)&g_wifi_packet_pool);

    //adding Google public DNS server IP address 8.8.8.8
    err_code = nx_dns_server_add(&g_dns_wifi_client, (8L << 24) + (8L << 16) + (8L << 8) + 8L);

    /* Send the message in the queue. Wait forever for space */
    /* to be available in the queue for the message.         */
    //tx_queue_send(&g_cdc_queue, wifi_ipstr, TX_WAIT_FOREVER);

    snprintf(wifi_ipstr, sizeof(wifi_ipstr), "WiFi   : %s", g_active_provision_info.ssid);
    write_display_console(0, wifi_ipstr);

    //construct IP address string
    snprintf(wifi_ipstr, sizeof(wifi_ipstr), "IP     : %lu.%lu.%lu.%lu",
                     (wifi_ip_address >> 24) & 0xff, (wifi_ip_address >> 16) & 0xff,
                     (wifi_ip_address >> 8) & 0xff, (wifi_ip_address >> 0) & 0xff);
    write_display_console(1, wifi_ipstr);
}

/* Networking entry function */
void wifi_network_entry(void)
{

    /* LED type structure */
    bsp_leds_t leds;
    /* LED state variable */
    ioport_level_t level = IOPORT_LEVEL_HIGH;

    /* Get LED information for this board */
    R_BSP_LedsGet(&leds);

    initilize_wifi_network();
    tx_thread_sleep (100);
    while (1) {
        if (0 < leds.led_count) {
            if(IOPORT_LEVEL_LOW == level) {
                level = IOPORT_LEVEL_HIGH;
            } else {
                level = IOPORT_LEVEL_LOW;
            }
            g_ioport.p_api->pinWrite(leds.p_leds[1], level);
        }
        tx_thread_sleep (10);
    }
}
