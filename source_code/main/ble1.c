#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <esp_log.h>
#include <esp_blufi_api.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "nvs_flash.h"
#include "sdkconfig.h"

#define BT_BD_ADDR_STR         "%02x:%02x:%02x:%02x:%02x:%02x"
#define BT_BD_ADDR_HEX(addr)   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]

static const char *bt_event_type_to_string(uint32_t eventType);
static void gap_callback_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static const char *btsig_gap_type(uint32_t gap_type);

#define tag "Scanning"





static uint16_t convertU8ToU16(uint8_t inByteA, uint8_t inByteB) {
	
	return inByteA<<8 | inByteB;

}



static double decode1809(uint8_t  *payload) {

	return ((float)(payload[4]<<16 | payload[3]<<8 | payload[2])) / 100.0;

}



static uint8_t decode180f(uint8_t  *payload) {

	return payload[2];

}



static void dump_16bituuid(uint8_t  *payload,uint8_t length) {
	
	uint16_t uuid =0;

	uuid = convertU8ToU16 (payload[1],payload[0]);
	ESP_LOGD(tag,"Dump16 UUID %04X, len %d", uuid,length);


	length -=2;

	
	switch(uuid)
	{
		
		case 0x1809: 
			if (length >=4) 
			{				
				ESP_LOGI(tag,"@ 0x1809 Temperature %f", decode1809(payload));
			}
			
			break;
		case 0x180f : 
			if (length >=1)
			{ 
				ESP_LOGI(tag,"@ 0x180F Battery %d %%", decode180f(payload));
			}
			break;

		default:
			ESP_LOGI(tag,"@ 16 BIT UUID 0x%04X - Packet decoding for thtis type not implemented",uuid); 
			break;
	}
}


static uint8_t dump_adv_payload(uint8_t *payload) 
{
	uint8_t length;
	uint8_t ad_type;
	uint8_t sizeConsumed = 0;
	int finished = 0;
	uint8_t total_length=0;
	

	while(!finished) {
		length = *payload;
		payload++;
		if (length != 0) {
			ad_type = *payload;
			


		
			switch(ad_type)
			{
				case 0x16: 
					dump_16bituuid(payload+1, length);
					break;

				case 0x09:  
					ESP_LOGI(tag, "Complete local name: %.*s", length, payload);			
					break;

				default:
					break;
			}

			
			int i;
			int size = length / sizeof(char);
			char *hex_str = (char*) calloc(3 * size,1);
			char *ascii_str = (char*) calloc(length+1,1); 
			char *buf_ptr1 = hex_str;
			char *buf_ptr2 = ascii_str;

			unsigned char *source = (unsigned char *)payload+1;	

			if ((hex_str) && (ascii_str)) 
			{
				for (i = 0; i < size; i++)
				{
					buf_ptr1 += sprintf(buf_ptr1, i < size - 1 ? "%02X:" : "%02X", source[i]);

					
					char ch = source[i];
					
					
					int ichar = ((int) ch) & 0xFF;
					if ((ichar<32) || (ichar>126))
					{
						ch = '.';  
					}
					
					buf_ptr2 += sprintf(buf_ptr2, "%c", ch);					
					
				}

				
				
				
				if (ad_type != 0x09){
					ESP_LOGI(tag,"%s: %s (%s)", btsig_gap_type(ad_type), hex_str, ascii_str);
				}

			}
			if (hex_str) free(hex_str);
			hex_str=0;

			if (ascii_str) free(ascii_str);
			ascii_str=0;

			payload += length;
			total_length += length+1;
		}	

		sizeConsumed = 1 + length;
		if (sizeConsumed >=31 || length == 0) {
			finished = 1;
		}
	} 
	return total_length;
} 


static void gap_callback_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	ESP_LOGD(tag, "Received a GAP event: %s", bt_event_type_to_string(event));
	esp_ble_gap_cb_param_t *p = (esp_ble_gap_cb_param_t *)param;

	esp_err_t status;	


	switch (event) 
	{
		case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
		{
			ESP_LOGD(tag, "status: %d", p->scan_param_cmpl.status);
		
			
			uint32_t duration = 30;
			status = esp_ble_gap_start_scanning(duration); 
			if (status != ESP_OK) 
			{
				ESP_LOGE(tag, "esp_ble_gap_start_scanning: rc=%d", status);
			}
		}
		break;

		case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
		{
			if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
			{
          	  ESP_LOGE(tag, "Scan start failed");
			}
		}
		break;
		
		case ESP_GAP_BLE_SCAN_RESULT_EVT:
		{	
			printf ("\n\n\n");
			ESP_LOGI(tag, "Device address: %02x:%02x:%02x:%02x:%02x:%02x", BT_BD_ADDR_HEX(p->scan_rst.bda));
			
			

			if ( p->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT)
			{
				
				uint32_t duration = 30;
				status = esp_ble_gap_start_scanning	(duration); 
				if (status != ESP_OK) 
				{
					ESP_LOGE(tag, "esp_ble_gap_start_scanning: rc=%d", status);
				}

				return;
			}

			
			if (p->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {

				dump_adv_payload(p->scan_rst.ble_adv);

			}
			
			
		}
		break;

		case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
		{
			if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
			{
				ESP_LOGE(tag, "Scan stop failed");
			}
			else 
			{
				ESP_LOGI(tag, "Stop scan successfully");
			}
		}

		case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		{
			if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
			{
				ESP_LOGE(tag, "Adv stop failed");
			}
			else 
			{
				ESP_LOGI(tag, "Stop adv successfully");
			}
		}
		break;
		
		
		default:
        break;
	}
} 




static const char *bt_event_type_to_string(uint32_t eventType) {
	switch(eventType) {
		case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
			return "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT";
		case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
			return "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT";
		case ESP_GAP_BLE_SCAN_RESULT_EVT:
			return "ESP_GAP_BLE_SCAN_RESULT_EVT";
		case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
			return "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT";
		default:
			return "Unknown event type";
	}
} 

static const char *btsig_gap_type(uint32_t gap_type) {
	switch (gap_type)
	{
		case 0x02: return "Incomplete List of 16-bit Service Class UUIDs";
		case 0x03: return "Complete List of 16-bit Service Class UUIDs";
		case 0x04: return "Incomplete List of 32-bit Service Class UUIDs";
		case 0x05: return "Complete List of 32-bit Service Class UUIDs";
		case 0x06: return "Incomplete List of 128-bit Service Class UUIDs";
		case 0x07: return "Complete List of 128-bit Service Class UUIDs";
		case 0x08: return "Shortened Local Name";
		case 0x09: return "Complete Local Name";
		case 0x14: return "List of 16-bit Service Solicitation UUIDs";
		case 0x1F: return "List of 32-bit Service Solicitation UUIDs";
		case 0x15: return "List of 128-bit Service Solicitation UUIDs";
		case 0x16: return "Service Data - 16-bit UUID";
		case 0x20: return "Service Data - 32-bit UUID";
		case 0x21: return "Service Data - 128-bit UUID";
		case 0x1B: return "LE Bluetooth Device Address";
		case 0xFF: return "Manufacturer Data";
		
		default: 
			return "Unknown type";
	}
}

	
esp_err_t register_ble_functionality(void)
{
	esp_err_t status;	
	
	ESP_LOGI(tag, "Register GAP callback");
	
	
	status = esp_ble_gap_register_callback(gap_callback_handler);
	if (status != ESP_OK) 
	{
		ESP_LOGE(tag, "esp_ble_gap_register_callback: rc=%d", status);
		return ESP_FAIL;
	}

	static esp_ble_scan_params_t ble_scan_params = 
	{	
		.scan_type              = BLE_SCAN_TYPE_ACTIVE,
		.own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
		.scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
		.scan_interval          = 0x50,
		.scan_window            = 0x30		
	};

	ESP_LOGI(tag, "Set GAP scan parameters");
	
	
	status = esp_ble_gap_set_scan_params(&ble_scan_params);
	if (status != ESP_OK) 
	{
		ESP_LOGE(tag, "esp_ble_gap_set_scan_params: rc=%d", status);
		return ESP_FAIL;
	}


	ESP_LOGD(tag, "We have registered what we need!");

	return ESP_OK ;
}


void bt_task(void *ignore)
{
	esp_err_t status;

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) 
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	ESP_LOGI(tag, "Enabling Bluetooth Controller");
	
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	if (esp_bt_controller_init(&bt_cfg) != ESP_OK) 
	{
		ESP_LOGE(tag, "Bluetooth controller initialize failed");
		goto end; 
	}

	if (esp_bt_controller_enable(ESP_BT_MODE_BTDM) != ESP_OK) 
	{
		ESP_LOGE(tag, "Bluetooth controller enable failed");
		goto end; 
	}

	ESP_LOGI(tag, "Bluetooth Controller Enabled");

	ESP_LOGI(tag, "Init Bluetooth stack");
	
	status = esp_bluedroid_init(); 
	if (status != ESP_OK)
	{ 
		ESP_LOGE(tag, "%s init bluetooth failed\n", __func__); 
		goto end; 
	} 
	
	status = esp_bluedroid_enable(); 
	if (status != ESP_OK) 
	{ 
		ESP_LOGE(tag, "%s enable bluetooth failed\n", __func__); 
		goto end;
	} 

	ESP_LOGI(tag, "Bluetooth stack initialized");

	ESP_LOGI(tag, "Register BLE functionality");
	status = register_ble_functionality();
	if (status != ESP_OK)
	{
		ESP_LOGE(tag, "Register BLE functionality failed");
		goto end;
	}

	while(1)
    {
        vTaskDelay(1000);
    }

end:
	ESP_LOGI(tag, "Terminating BT logging task");
	vTaskDelete(NULL);
	
	
} 
