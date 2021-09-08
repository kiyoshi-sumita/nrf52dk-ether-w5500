#include "wizchip_ether.h"
#include "./wizchip/socket.h"
#include "./wizchip/dns/dns.h"
#include "./wizchip/wizchip_conf.h"

#define W5500_RESET             NRF_GPIO_PIN_MAP(0,30)
#define W5500_INT               NRF_GPIO_PIN_MAP(0,31)

#define OK_RESPONSE             true
#define ERROR_RESPONSE          false
#define NO_READ_DATA            false
#define TIMEOUT_RESPONSE        false
#define SOCKET_ERROR_RESPONSE   false

#define VERSIONR_W5500          0x04

#define SPI_BUFFER_LENGTH       4096
unsigned char rxBuffer[SPI_BUFFER_LENGTH];
unsigned char txBuffer[SPI_BUFFER_LENGTH];

#define DNS_DATA_BUF_SIZE   2048
uint8_t dns_data_buffer[DNS_DATA_BUF_SIZE];
uint8_t domain_IP[4]  = {0, }; 

bool connect_status = false;
int8_t socket_number = -1;
int32_t last_error_code = 0;

wiz_NetInfo _wizNetinfo = {
    .mac = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .ip = {192, 168, 1, 2},
    .sn = {255, 255, 255, 0},
    .gw = {192, 168, 1, 1},
    .dns = {8, 8, 8, 8},		// Google public DNS (8.8.8.8 , 8.8.4.4), OpenDNS (208.67.222.222 , 208.67.220.220)
    .dhcp = NETINFO_STATIC,
};

#define  TIMEOUT_MS     20
#define  TIMEOUT_UNIT   20
static int16_t timeout_counter = 0;

static void display_network_conf(void)
{
  #ifdef _WIZCHIP_DEBUG_
	uint8_t tmpstr[6] = {0,};
  #endif
	ctlnetwork(CN_GET_NETINFO, (void*) &_wizNetinfo);
  #ifdef _WIZCHIP_DEBUG_
	// Display Network Information
	ctlwizchip(CW_GET_ID,(void*)tmpstr);

	if (_wizNetinfo.dhcp == NETINFO_DHCP) { NRF_LOG_INFO("===== %s NET CONF : DHCP =====",(char*)tmpstr); }
	else { NRF_LOG_INFO("===== %s NET CONF : Static =====",(char*)tmpstr);}
	NRF_LOG_INFO(" MAC : %02X:%02X:%02X:%02X:%02X:%02X", _wizNetinfo.mac[0], _wizNetinfo.mac[1], _wizNetinfo.mac[2], _wizNetinfo.mac[3], _wizNetinfo.mac[4], _wizNetinfo.mac[5]);
	NRF_LOG_INFO(" IP : %d.%d.%d.%d", _wizNetinfo.ip[0], _wizNetinfo.ip[1], _wizNetinfo.ip[2], _wizNetinfo.ip[3]);
	NRF_LOG_INFO(" GW : %d.%d.%d.%d", _wizNetinfo.gw[0], _wizNetinfo.gw[1], _wizNetinfo.gw[2], _wizNetinfo.gw[3]);
	NRF_LOG_INFO(" SN : %d.%d.%d.%d", _wizNetinfo.sn[0], _wizNetinfo.sn[1], _wizNetinfo.sn[2], _wizNetinfo.sn[3]);
	NRF_LOG_INFO("=======================================");
  #endif
}

bool wizchip_is_connected_now(void) 
{
    uint8_t status;
	
    ctlwizchip(CW_GET_PHYLINK, (void*) &status);
    if (status == PHY_LINK_OFF) return false;
  #ifdef _WIZCHIP_DEBUG_
	NRF_LOG_INFO("PHY link detected = %02X", status);
  #endif
   return true;
}

int wizchip_connect(void)
{
    reg_wizchip_cris_cbfunc(NULL, NULL);
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
    reg_wizchip_spi_cbfunc(spi_read_byte, spi_write_byte);
    reg_wizchip_spiburst_cbfunc(spi_read_burst, spi_write_burst);

    setMR(MR_RST);
    nrf_delay_ms(100);

    uint8_t status;
  #ifdef _WIZCHIP_DEBUG_
    while(true) {
        status = getVERSIONR();
        if (status == VERSIONR_W5500) break;
        NRF_LOG_FLUSH();
        nrf_delay_ms(100);
    }
  #else
    status = getVERSIONR();
    if (status == VERSIONR_W5500) { NRF_LOG_INFO("phy_wizchip Version(%02X)", status); }
    else {  NRF_LOG_INFO("phy_wizchip Version not obtain(%02X)", status); }
  #endif

	uint8_t memsize[2][8] = { { 2, 2, 2, 2, 2, 2, 2, 2 }, { 2, 2, 2, 2, 2, 2, 2, 2 } };
	if (ctlwizchip(CW_INIT_WIZCHIP, (void*) memsize) == -1) { // wizchip initialize
     #ifdef _WIZCHIP_DEBUG_
		NRF_LOG_INFO("WIZCHIP Initialized fail.");
     #endif
	}

	ctlnetwork(CN_SET_NETINFO, (void*) &_wizNetinfo);   // wizchip netconf

    status = getMR();
  #ifdef _WIZCHIP_DEBUG_
    NRF_LOG_INFO("phy_wizchip Mode Regster = %02X", status);
  #endif

    connect_status = true;
    return true;
}

int wizchip_client_connect(network_client_t* client, const char *domain, const uint16_t port)
{
    NRF_LOG_INFO("wizchip_client_connect");

  #ifdef _WIZCHIP_DEBUG_
	uint8_t tmpInfo[6] = {0,};

	ctlwizchip(CW_GET_ID,(void*)tmpInfo);

    NRF_LOG_INFO("=======================================");
	NRF_LOG_INFO(" WIZnet %s  - DNS client", tmpInfo);
	NRF_LOG_INFO("=======================================");

	display_network_conf();
    
	NRF_LOG_INFO("===== DNS Servers =====");
	NRF_LOG_INFO("> DNS 1st : %d.%d.%d.%d", _wizNetinfo.dns[0], _wizNetinfo.dns[1], _wizNetinfo.dns[2], _wizNetinfo.dns[3]);
	NRF_LOG_INFO("=======================================");
	NRF_LOG_INFO("> Target Domain Name : %s", domain);
  #endif

  #ifndef WIZ_CONNECT_DNS
    DNS_init(port, dns_data_buffer);   // DNS client Initialization 

    int8_t ret;
    if ((ret = DNS_run(_wizNetinfo.dns, (uint8_t *)domain, domain_IP)) > 0) { // try to 1st DNS processing
     #ifdef _WIZCHIP_DEBUG_
       NRF_LOG_INFO("> 1st DNS Respond");
     #endif
    }
    else if (ret == -1) {
      #ifdef _WIZCHIP_DEBUG_
       NRF_LOG_INFO("> MAX_DOMAIN_NAME is too small. Should be redefined it.");
      #endif
    }
    else {
      #ifdef _WIZCHIP_DEBUG_
       NRF_LOG_INFO("> DNS Failed");
      #endif
    }

    if(ret > 0) {
      #ifdef _WIZCHIP_DEBUG_
       NRF_LOG_INFO("> Translated %s to [%d.%d.%d.%d]", domain, domain_IP[0], domain_IP[1], domain_IP[2], domain_IP[3]);
      #endif
    }
  #endif

    int8_t tempNumber = socket_number + 1;
    int8_t error_code = wiz_socket(tempNumber, Sn_MR_TCP, port, 0x00);
    if (error_code != tempNumber) return SOCKET_ERROR_RESPONSE;

    error_code = wiz_connect(tempNumber, &_wizNetinfo.ip[0], port);
    if (error_code != SOCK_OK) return SOCKET_ERROR_RESPONSE;

    uint8_t status;
    timeout_counter = TIMEOUT_MS;
    while((status = getSn_SR(tempNumber)) != SOCK_ESTABLISHED) {
        if (--timeout_counter <= 0) return TIMEOUT_RESPONSE;
        nrf_delay_ms(TIMEOUT_UNIT);
    }

    if (getSn_IR(tempNumber) & Sn_IR_CON) {
      #ifndef _WIZCHIP_DEBUG_
        uint8_t destip[4];
		getSn_DIPR(tempNumber, destip);
		destport = getSn_DPORT(tempNumber);
	    printf("%d:Connected - %d.%d.%d.%d : %d\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
      #endif
		setSn_IR(tempNumber, Sn_IR_CON);
    }
    client->socket = tempNumber;
    return OK_RESPONSE;
}

int wizchip_client_close(network_client_t client)
{
  NRF_LOG_INFO("wizchip_client_close");

     int8_t error_code = wiz_disconnect(client.socket);
    if (error_code != SOCK_OK) return SOCKET_ERROR_RESPONSE;

    error_code = wiz_close(client.socket);
    if (error_code != SOCK_OK) return SOCKET_ERROR_RESPONSE;

    return OK_RESPONSE;
}

int wizchip_client_read(network_client_t client, char *data, const uint16_t length)
{
    uint16_t size;

 	if ((size = getSn_RX_RSR(client.socket)) <= 0) return NO_READ_DATA;
	else if (size > length) return ERROR_RESPONSE;

    int32_t rcvresult = wiz_recv(client.socket, (uint8_t *)data, length);
    if (rcvresult <= SOCK_ERROR) {
        if (rcvresult == SOCK_ERROR) {
            // no data received
            return NO_READ_DATA;
        } 
        else {
            last_error_code = rcvresult;
            return ERROR_RESPONSE;
        }
    }
    return rcvresult;
}

int wizchip_client_write(network_client_t client, const char *data, const uint16_t length, int flgs)
{
    int32_t sendresult = wiz_send(client.socket, (uint8_t *)data, length);
    if (sendresult <= SOCK_ERROR) {
        if (sendresult == SOCK_ERROR) {
            // no data received
            return NO_READ_DATA;
        } 
        else {
            last_error_code = sendresult;
            return ERROR_RESPONSE;
        }
    }
    return sendresult;
}
