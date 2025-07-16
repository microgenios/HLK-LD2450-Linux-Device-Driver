/*_____________________________________________________________________________
 * @file    LD2450.h
 * @author  Pabllo Lins
 * @date    20 de julho de 2024
 * @brief
 *
 * @misc
 * _____________________________________________________________________________
 * @attention
 *
 * 			COPYRIGHT 2024 _,  All Rights Reserved;
 *
 * Licensed under _ License Agreement, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * __________________________________________________________________________*/
/*___________________________________________________________________________*/
#ifndef	__LD2450_H__
#define	__LD2450_H__ 1

#ifdef __cplusplus
extern "C" {
#endif

/*___________________________________________________________________________*/
/*___________________________INFORMACOES ADICIONAIS__________________________*/


/*___________________________________________________________________________*/
/*_________________________HISTORICO DE REVISOES_____________________________*/



/*___________________________________________________________________________*/
/*_________________________________DEFINES___________________________________*/

#define STM32       1
#define RASPBERRY  	2
#define SYSTEM_PLATFORM      RASPBERRY


/*___________________________________________________________________________*/
/*_______________________________INCLUDES____________________________________*/
#if ( SYSTEM_PLATFORM == STM32 )
	#include <stdint.h>
  	#include "main.h"
  	#include "usart.h"
#elif( SYSTEM_PLATFORM == RASPBERRY)
	#define DEBUG_DRIVER 0
	#define PRINT_SERIAL_FIFO 0
	#define DEBUG_KERNEL_FIFO 0
	#define DEBUG_DRIVER_SERIAL_READ 0
#endif


#define READ_TRIES_NUM              10
#define POW_2_15					0x8000 //32768

#define TRACKING_BUFF_SIZE			0x20

#define LD2450_DEFAULT_BAUD_RATE    256000
#define PROTOCOL_START              0xFDFCFBFA
#define PROTOCOL_START_SIZE         4
#define PROT_START_BYTE_0           0xFD
#define PROT_START_BYTE_1           0xFC
#define PROT_START_BYTE_2           0xFB
#define PROT_START_BYTE_3           0xFA

#define PROTOCOL_END                0x04030201
#define PROTOCOL_END_SIZE         	4
#define PROT_END_BYTE_0             0x04
#define PROT_END_BYTE_1             0x03
#define PROT_END_BYTE_2             0x02
#define PROT_END_BYTE_3             0x01


#define PROT_DATA_OUT_START_BYTE_0             0xAA
#define PROT_DATA_OUT_START_BYTE_1             0xFF
#define PROT_DATA_OUT_START_BYTE_2             0x03
#define PROT_DATA_OUT_START_BYTE_3             0x00

#define PROT_DATA_OUT_END_BYTE_0             0x55
#define PROT_DATA_OUT_END_BYTE_1             0xCC


//Set Configurations
#define CMD_SET_ENABLE_CONFIG 				0x0400FF000100
#define SET_ENABLE_CONFIG_BYTE_0            0x04
#define SET_ENABLE_CONFIG_BYTE_1            0x00
#define SET_ENABLE_CONFIG_BYTE_2            0xFF
#define SET_ENABLE_CONFIG_BYTE_3            0x00
#define SET_ENABLE_CONFIG_BYTE_4            0x01
#define SET_ENABLE_CONFIG_BYTE_5            0x00
//Set Configurations ack
#define CMD_SET_ENABLE_CONFIG_ACK 			0x0800FF010001004000
#define SET_ENABLE_CONFIG_BYTE_0_ACK        0x08
#define SET_ENABLE_CONFIG_BYTE_1_ACK        0x00
#define SET_ENABLE_CONFIG_BYTE_2_ACK        0xFF
#define SET_ENABLE_CONFIG_BYTE_3_ACK        0x01
#define SET_ENABLE_CONFIG_BYTE_4_ACK        0x00
#define SET_ENABLE_CONFIG_BYTE_5_ACK        0x00
#define SET_ENABLE_CONFIG_BYTE_6_ACK        0x01
#define SET_ENABLE_CONFIG_BYTE_7_ACK        0x00
#define SET_ENABLE_CONFIG_BYTE_8_ACK        0x40
#define SET_ENABLE_CONFIG_BYTE_9_ACK        0x00

//Restart Disable config
#define CMD_SET_DISABLE_CONFIG 			    0x0200FE00
#define SET_DISABLE_CONFIG_BYTE_0            0x02
#define SET_DISABLE_CONFIG_BYTE_1            0x00
#define SET_DISABLE_CONFIG_BYTE_2            0xFE
#define SET_DISABLE_CONFIG_BYTE_3            0x00


//Restart module
#define CMD_SET_RESTART 					0x0200A300

//FW version
#define CMD_GET_FW_VERSION     				0x0200A000
#define GET_FW_VERSION_BYTE_0               0x02
#define GET_FW_VERSION_BYTE_1               0x00
#define GET_FW_VERSION_BYTE_2               0xA0
#define GET_FW_VERSION_BYTE_3               0x00
//FW version ack
#define CMD_GET_FW_VERSION_ACK 				0x0C00A001000000
#define GET_FW_VERSION_BYTE_0_ACK        	0x0C
#define GET_FW_VERSION_BYTE_1_ACK        	0x00
#define GET_FW_VERSION_BYTE_2_ACK        	0xA0
#define GET_FW_VERSION_BYTE_3_ACK        	0x01
#define GET_FW_VERSION_BYTE_4_ACK        	0x00
#define GET_FW_VERSION_BYTE_5_ACK        	0x00
#define GET_FW_VERSION_BYTE_6_ACK        	0x00


//SERIAL PORT BAUD RATES - 0x0400A100xx00
#define CMD_SET_BAUD_9600   				0x0400A1000100	//Muda Baudrate = 9600
#define CMD_SET_BAUD_19200  				0x0400A1000200 	//Muda Baudrate = 19200
#define CMD_SET_BAUD_38400  				0x0400A1000300 	//Muda Baudrate = 38400
#define CMD_SET_BAUD_57600  				0x0400A1000400 	//Muda Baudrate = 57600
#define CMD_SET_BAUD_115200 				0x0400A1000500 	//Muda Baudrate = 115200
#define CMD_SET_BAUD_230400 				0x0400A1000600 	//Muda Baudrate = 230400
#define CMD_SET_BAUD_256000 				0x0400A1000700 	//Muda Baudrate = 256000
#define CMD_SET_BAUD_460800 				0x0400A1000800 	//Muda Baudrate = 460800

#define CMD_SET_BAUD_INIT_0   				0x04
#define CMD_SET_BAUD_INIT_1					0x00
#define CMD_SET_BAUD_INIT_2					0xA1
#define CMD_SET_BAUD_INIT_3					0x00
#define CMD_SET_BAUD_INIT_5					0x00
#define CMD_SET_BAUD_9600_BYTE   			0x01	//Muda Baudrate = 9600
#define CMD_SET_BAUD_19200_BYTE  			0x02 	//Muda Baudrate = 19200
#define CMD_SET_BAUD_38400_BYTE  			0x03 	//Muda Baudrate = 38400
#define CMD_SET_BAUD_57600_BYTE  			0x04 	//Muda Baudrate = 57600
#define CMD_SET_BAUD_115200_BYTE 			0x05 	//Muda Baudrate = 115200
#define CMD_SET_BAUD_230400_BYTE 			0x06 	//Muda Baudrate = 230400
#define CMD_SET_BAUD_256000_BYTE 			0x07 	//Muda Baudrate = 256000
#define CMD_SET_BAUD_460800_BYTE 			0x08 	//Muda Baudrate = 460800


//BLUETOOTH
#define CMD_SET_ON_BLUETOOTH 				0x0400A4000100	//Ligar bluetooth
#define CMD_SET_OFF_BLUETOOTH 				0x0400A4000000 	//Desligar bluetooth

//MAC Address
#define CMD_GET_MAC_ADDRSS 					0x0400A5000100 //Exibe MAC Address
#define GET_MAC_ADDRSS_BYTE_0               0x04
#define GET_MAC_ADDRSS_BYTE_1               0x00
#define GET_MAC_ADDRSS_BYTE_2               0xA5
#define GET_MAC_ADDRSS_BYTE_3               0x00
#define GET_MAC_ADDRSS_BYTE_4               0x01
#define GET_MAC_ADDRSS_BYTE_5               0x00

//Tracking one Target
#define CMD_GET_TRACKING_MODE_SETUP 		0x02009100 //Le forma de rastreamento. 0100-> 1 alvo, 0200-> multiplos alvos  ")
#define GET_TRACKING_MODE_BYTE_0            0x02
#define GET_TRACKING_MODE_BYTE_1            0x00
#define GET_TRACKING_MODE_BYTE_2            0x91
#define GET_TRACKING_MODE_BYTE_3            0x00
#define CMD_SET_ONE_TARTGET_TRACKING 		0x02008000 //Seta Rastreamento de um alvo
#define CMD_SET_MULTIPLES_TARTGET_TRACKING 	0x02009000 //Seta Rastreamento de multiplos alvos


//Factory defaults
#define CMD_RESTORE_FACTORY_SETTINGS 		0x0200A200 //Recupera definicoes de fabrica


//Restart module (Logical reset)
#define CMD_RESTART_MODULE 					0x0200A300
#define RESTART_MODULE_BYTE_0            	0x02
#define RESTART_MODULE_BYTE_1            	0x00
#define RESTART_MODULE_BYTE_2            	0xA3
#define RESTART_MODULE_BYTE_3            	0x00
//Restart module (Logical reset) ack
#define CMD_RESTART_MODULE_ACK 				0x0400A3010000
#define RESTART_MODULE_BYTE_0_ACK        	0x04
#define RESTART_MODULE_BYTE_1_ACK        	0x00
#define RESTART_MODULE_BYTE_2_ACK        	0xA3
#define RESTART_MODULE_BYTE_3_ACK        	0x01
#define RESTART_MODULE_BYTE_4_ACK        	0x00
#define RESTART_MODULE_BYTE_5_ACK        	0x00

/*___________________________________________________________________________*/
/*_______________________________VAR GLOBAIS_________________________________*/
#if( SYSTEM_PLATFORM == STM32)
typedef struct My_LD2450
{
	//Setup infos
	uint8_t u8_ld2450_status;
	uint8_t u8_fW_version[15];
	uint8_t u8_mac_add[10];
	uint8_t u8_serial_baud_rate;
	uint8_t u8_bluetooth;
	uint8_t u8_filtro_de_area;
	uint8_t u8_qtdd_alvos;

	//Tracking infos
	int8_t 		i8_buf_alvo_1[8];
	int16_t 	i16_alvo_1_cordinate_X;
	int16_t 	i16_alvo_1_cordinate_Y;
	int16_t 	i16_alvo_1_velocity;
	uint16_t 	u16_alvo_1_distance;
	int8_t 		i8_buf_alvo_1_Cordinate_X_String[7];
	int8_t 		i8_buf_alvo_1_Cordinate_Y_String[7];
	int8_t 		i8_buf_alvo_1_Distancia_String[7];
	int8_t 		i8_buf_alvo_1_veloc_String[7];

	uint8_t 	u8_buf_alvo_2[8];
	int16_t 	i16_alvo_2_cordinate_X;
	int16_t 	i16_alvo_2_cordinate_Y;
	int16_t 	i16_alvo_2_velocity;
	uint16_t 	u16_alvo_2_distance;
	uint8_t 	u8_buf_alvo_2_Cordinate_X_String[7];
	uint8_t 	u8_buf_alvo_2_Cordinate_Y_String[7];
	uint8_t 	u8_buf_alvo_2_Distancia_String[7];
	uint8_t 	u8_buf_alvo_2_veloc_String[7];

	uint8_t 	u8_buf_alvo_3[8];
	int16_t 	i16_alvo_3_cordinate_X;
	int16_t 	i16_alvo_3_cordinate_Y;
	int16_t 	i16_alvo_3_velocity;
	uint16_t 	u16_alvo_3_distance;
	uint8_t 	u8_buf_alvo_3_Cordinate_X_String[7];
	uint8_t 	u8_buf_alvo_3_Cordinate_Y_String[7];
	uint8_t 	u8_buf_alvo_3_Distancia_String[7];
	uint8_t 	u8_buf_alvo_3_veloc_String[7];

}t_sensor_LD2450;


extern t_sensor_LD2450 My_LD2450;
#endif
/*___________________________________________________________________________*/
/*________________________________VAR LOCAIS_________________________________*/


/*___________________________________________________________________________*/
/*_____________________________FUNCOES_INTERNAS______________________________*/


/*___________________________________________________________________________*/
/*_____________________________FUNCOES_EXTERNAS______________________________*/
#if( SYSTEM_PLATFORM == STM32)
int32_t 	I32_LD2450_Get_Fw_Version_String	    	( uint8_t * u8_result );
int32_t 	I32_APP_LD2450_Get_MAC_ADD_String	        ( uint8_t * u8_buff_to_receive_ans );
int32_t 	I32_APP_LD2450_Query_target_tracking_mode	( uint8_t * u8_buff_to_receive_ans );
int16_t 	I16_APP_LD2450_Calculate_Cordinate_X    	( int8_t * 	i8_Sensor_buf );
int16_t 	I16_APP_LD2450_Calculate_Cordinate_Y  		( int8_t * i8_Sensor_buf );
uint16_t 	I16_APP_LD2450_Calculate_Target_Distance  	( int8_t * i8_Sensor_buf );
int16_t 	I16_APP_LD2450_Calculate_speed			  	( int8_t * i8_Sensor_buf );
int32_t 	I32_APP_LD2450_Get_Cordinate_X_String	    ( uint8_t * u8_string_result, 	int16_t i16_var_x );
int32_t 	I32_APP_LD2450_Get_Cordinate_Y_String	    ( uint8_t * u8_string_result, 	int16_t i16_var_y );
int32_t 	I32_APP_LD2450_Get_Target_Velocity_String	( uint8_t * u8_string_result, 	int16_t i16_vel );
int32_t 	I32_APP_LD2450_Get_Target_Distance_String	( uint8_t * u8_string_result, 	uint16_t u16_dist );
void 		V_LD2450_callback_retrig_scan_func			( void );
#endif

#ifdef __cplusplus
}
#endif

#endif //__LD2450_H__ 1
/*___________________________________________________________________________*/
/*      		>>>>> Fim do arquivo LD2450.h <<<<< 				 */
/*___________________________________________________________________________*/
