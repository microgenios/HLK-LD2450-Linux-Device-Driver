/**
 * Autor: Pabllo Lins;
 * Device Driver para o Módulo radar Hi-Link LD2450; 
 */

/*___________________________________________________________________________*/
/*___________________________INFORMACOES ADICIONAIS__________________________*/
/*
 * Este modulo foi feito com base na documentacao:
 * "Serial Communication Protocol" do fabricante Hi-Link, versao 1.03.
 *
 * DEFAULT BAUD RATE = 256000
 *
 * Command protocol frame format
 *
 * SEND COMMAND FRAME FORMAT = ( Header + In-frame data length + In-frame data + End of frame )
 * Header:                  (4 Bytes)
 * In-frame data length:    (2 Bytes)
 * In-frame data:           ( Command word (2 bytes) + Command value (N bytes) )
 * End of frame:            (4 Bytes)
 *
 * ACK COMMAND FRAME FORMAT = ( Header + In-frame data length + In-frame data + End of frame )
 * Header:                  (4 Bytes)
 * In-frame data length:    (2 Bytes)
 * In-frame data:           ( Send Command Word | 0x0100 (2 bytes) + Return value (N bytes) )
 * End of frame:            (4 Bytes)
 *
 *
 * EXAMPLE: Read firmware version command
 * Command word: 0x00A0
 * Command value: none
 * Return Value: 2 bytes ACK status (0 success, 1 failure) + 2 bytes firmware type (0x0000) + 2 bytes
 * major version number + 4 bytes minor version number
 *              Header          Data Length     Command word + Command value        End of frame
 * Send data: FD FC FB FA   +   02 00       +   A0 00                           +   04 03 02 01
*/


/*___________________________________________________________________________*/
/*_________________________HISTORICO DE REVISOES_____________________________*/



/*___________________________________________________________________________*/
/*_______________________________INCLUDES____________________________________*/
#include "LD2450.h"

#if( SYSTEM_PLATFORM == RASPBERRY)

#include <linux/module.h>
#include <linux/init.h>
#include <linux/serdev.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/kfifo.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>

/*___________________________________________________________________________*/
/*______________________Information about driver_____________________________*/
#define VERSION		1
#define SUB_VERSION	0
#define DEV_VERSION	14

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pabllo Lins");
MODULE_DESCRIPTION("Module driver for radar Hi-Link LD2450");
MODULE_VERSION("1.0.14");
//MODULE_VERSION("%d.%d.%d",VERSION, SUB_VERSION, DEV_VERSION);

#define UART_BAUDRATE LD2450_DEFAULT_BAUD_RATE

//Pin to relay signal
#define GPIO_23 (23)

//FIFO
#define FIFO_SIZE	256
static struct kfifo ld2450_fifo;
//static DEFINE_KFIFO(ld2450_fifo, char, FIFO_SIZE);


//Signal definitions
#define REG_CURRENT_TASK _IOW('a','a',int32_t*) 
static struct task_struct *task = NULL;
static int signum = 0;

//Var to be used as IRQ number storage
unsigned int GPIO_irqNumber;

//Enum with functions returns
enum
{
	e_ret_fail = -1,
	e_ret_success = 0,
	e_ret_fail_to_send_uart,
	e_ret_fail_to_receive_uart,
	e_ret_fail_to_send_uart_get_fw_version,
	e_ret_fail_to_receiv_uart_get_fw_version
};

//Enum with functions returns
enum
{
	e_mode_command = 0,
	e_mode_tracking
};

static int LD2450_Mode = e_mode_command;

/*___________________________________________________________________________*/
/*__________________Driver_internal_functions_prototype______________________*/
static int I_DRV_LD2450_Init( void );
static int I_DRV_LD2450_Liga_Modulo( void );
static int I_DRV_LD2450_Desliga_Modulo( void );
static int I_DRV_LD2450_Reset_Modulo( void );
static int I_DRV_LD2450_Module_setup( void );
static int I_DRV_LD2450_serial_command_ack_answ_compare( const char * sequence, int timeout, int i_expected_size );
static int I_DRV_LD2450_write_cmd_wait_ack_answ( const char * str_cmd, int str_cmd_size, const char * str_return, int str_return_size, int timeout );
static int I_DRV_LD2450_serial_write( const char * buffer, int size );
static int I_DRV_LD2450_Check_Fifo_Recv( void );
static int I_DRV_LD2450_Clear_Fifo( void );
static int I_DRV_LD2450_serial_read( char * message, int size );


 /*___________________________________________________________________________*/
 /*_____________________________FUNCOES_EXTERNAS______________________________*/
 static int I_APP_LD2450_Logical_Reset_Module( void );


 /*___________________________________________________________________________*/
/*_____________________________FUNCOES_INTERNAS______________________________*/
/**
  * @name   I_DRV_LD2450_Reset_Modulo
  * @brief  Função que reseta o modulo
  * @author 
  * @date   
  * @param void
  * @return int
  * @misc   
  */
static int I_DRV_LD2450_Reset_Modulo( void ) 
{
	int result = e_ret_fail;
#if ( DEBUG_DRIVER == 1 )	
	pr_info("Reseting module...\n");
#endif
	I_DRV_LD2450_Desliga_Modulo();
    msleep(1000);
	I_DRV_LD2450_Liga_Modulo();
    //msleep(1000);
	return result = e_ret_success;
}


/**
  * @name   I_DRV_LD2450_Liga_Modulo
  * @brief  Função que liga o gpio da alimentação do modulo
  * @author 
  * @date   
  * @param void
  * @return int
  * @misc   
  */
 static int I_DRV_LD2450_Liga_Modulo( void ) 
 {
	int result = e_ret_fail;
#if ( DEBUG_DRIVER == 1 )
	pr_info("Turning ON the module...\n");
#endif
	gpio_set_value(GPIO_23, 1); //enable = 1
	//msleep(2000);
#if ( DEBUG_DRIVER == 1 )
	pr_info("Module powered ON\n");
#endif
	return result = e_ret_success;
 }


 /**
  * @name   I_DRV_LD2450_Desliga_Modulo
  * @brief  Função que liga o gpio da alimentação do modulo
  * @author 
  * @date   
  * @param void
  * @return int
  * @misc   
  */
 static int I_DRV_LD2450_Desliga_Modulo( void ) 
 {
	int result = e_ret_fail;
#if ( DEBUG_DRIVER == 1 )	
	pr_info("Turning OFF the module...\n");
#endif
	gpio_set_value(GPIO_23, 0); //disable = 0
	//msleep(2000);
#if ( DEBUG_DRIVER == 1 )	
	pr_info("Module powered OFF\n");
#endif
	return result = e_ret_success;	 
 }

 /**
  * @name   I_DRV_LD2450_Clear_Fifo
  * @brief  Função que limpa a fifo
  * @author 
  * @date   
  * @param void
  * @return int
  * @misc   
  */
static int I_DRV_LD2450_Clear_Fifo( void )
{
	int result = e_ret_success;
    kfifo_reset (&ld2450_fifo);
	return result;
}



/**
  * @name   I_DRV_LD2450_Check_Fifo_Recv
  * @brief  Função que verifica se algum byte foi recebido do módulo
  * @author 
  * @date   
  * @param void
  * @return int
  * @misc   
  */
static int I_DRV_LD2450_Check_Fifo_Recv( void )
{
     return  (kfifo_is_empty(&ld2450_fifo) != 1);
}


/**
 * --------------------------------------------------------------------------------------
 * Interface Serial Device (serdev);
 */
static int  serdev_probe(struct serdev_device *serdev);
static void serdev_remove(struct serdev_device *serdev);
static int  serdev_recv(struct serdev_device *serdev, const unsigned char *buffer, size_t size); 

static struct serdev_device *serdev_g = NULL;

static struct of_device_id serdev_ids[] = {
	{
		.compatible = "brightlight,echodev",
	}, { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, serdev_ids);

static struct serdev_device_driver serdev_driver = {
	.probe = serdev_probe,
	.remove = serdev_remove,
	.driver = {
		.name = "serdev-uart",
		.of_match_table = serdev_ids,
	},
};


//Função de Callback chamada quando um caractere for recebido pela Serial da Rasp
static int serdev_recv(struct serdev_device *serdev, const unsigned char *buffer, size_t size) 
{
	if( LD2450_Mode == e_mode_command )	
	{
#if ( PRINT_SERIAL_FIFO == 1 )	
		pr_info("serdev_echo_recv: Recebido %d bytes com \"%s\"\n", size, buffer);
		pr_info("serdev: received %zu bytes\n", size);
#endif

		int copied = kfifo_in(&ld2450_fifo, (const char *) buffer, size);
		if (copied != size)
		{
#if ( PRINT_SERIAL_FIFO == 1 )	
        	pr_warn("LD2450: FIFO overflow, lost %zu bytes\n", size - copied);
#endif
		}
	}
	if( LD2450_Mode == e_mode_tracking )	
	{
		// Temporary buffer to accumulate data
		static unsigned char recv_buffer[64];
		static size_t recv_index = 0;
		
		// Iterate over the received data
		for (size_t i = 0; i < size; i++) {
			// Store byte in the temporary buffer
			recv_buffer[recv_index++] = buffer[i];

			// Check for complete message (header + tail)
			if (recv_index >= 4 &&
				recv_buffer[0] == 0xAA && recv_buffer[1] == 0xFF &&
				recv_buffer[2] == 0x03 && recv_buffer[3] == 0x00) {

				// Check for message length (30 bytes)
				if (recv_index >= 30 &&
					recv_buffer[28] == 0x55 && recv_buffer[29] == 0xCC) {
					// Valid complete message, push to FIFO
					kfifo_in(&ld2450_fifo, recv_buffer, 30);
					recv_index = 0; // Reset for next message
				}
			}

			// Reset on overflow
			if (recv_index >= sizeof(recv_buffer)) {
				recv_index = 0;
			}
		}
	}
	return size;
}

static const struct serdev_device_ops serdev_ops = {
	.receive_buf = serdev_recv,
	.write_wakeup = NULL, // optional
};

/**
 * Função chamada quando o driver for carregado no Kernel;
 */
static int serdev_probe(struct serdev_device *serdev) 
{
	int status;
#if ( DEBUG_DRIVER == 1 )	
	pr_info("Hello Serial Device...\n");
#endif

	serdev_device_set_client_ops(serdev, &serdev_ops);
	status = serdev_device_open(serdev);
	if(status) {
		printk("Error na abertura da porta serial\n");
		return -status;
	}

#if ( DEBUG_DRIVER == 1 )
	pr_info("Setting serdev baudrate...\n");
#endif
	serdev_device_set_baudrate(serdev, UART_BAUDRATE);	
	pr_info("SerDev port baudrate: %ld kbps - OK\n", UART_BAUDRATE);
	serdev_device_set_flow_control(serdev, false);

	serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	
	//Salvar o handler 'serdev' em var global para uso geral
	serdev_g = serdev;
#if ( DEBUG_DRIVER == 1 )	
	pr_info("serdev iniciado com sucesso\n");
#endif	
	return 0;
}

/**
 * Função chamada quando o driver for removido do Kernel; 
 */
static void serdev_remove(struct serdev_device *serdev) 
{
#if ( DEBUG_DRIVER == 1 )	
	pr_info("bye byte SerDev...\n");
#endif

	if (serdev)
	{
		serdev_device_close(serdev);
	}

}


/**
 * Função responsável pelo envio de um byte para o dispositivo bluetooth;
 */
static int I_DRV_LD2450_serial_write( const char * buffer, int size )
{
	if(serdev_g != NULL)
	{
		serdev_device_write_buf(serdev_g, buffer, size);
#if ( DEBUG_DRIVER == 1 )		
		pr_info("serdev_device_write_buf() success \n");
#endif
		return 0;
	}
	else
	{
		printk("serdev_device_write_buf() Fails \n");
		printk("serdev_g = %d", serdev_g);
		return -1;
	}
}

/**
 * Função responsável por enviar um signal do kernel para a aplicação
 */
static irqreturn_t int_func(int irq, void *dev)
{
    struct kernel_siginfo info;

#if ( DEBUG_DRIVER == 1 )
        pr_info("Ocorreu uma interrupção IRQ");
#endif
		
        memset(&info, 0, sizeof(struct kernel_siginfo));
        info.si_signo = SIGUSR1;
        info.si_code = SI_QUEUE;
        info.si_int = 1;
     
        if ( task != NULL )
		{
            printk(KERN_INFO "Enviando um signal para a aplicação\n");
            if(send_sig_info(SIGUSR1, &info, task) < 0)
			{
                printk(KERN_INFO "Não é possível enviar um signal\n");
            }
        }
        return IRQ_HANDLED;
}


//Configurações do Driver LD2450
static int I_APP_LD2450_Serial_Open(struct inode *inode, struct file *file);
static int I_APP_LD2450_Serial_Close(struct inode *inode, struct file *file);
static ssize_t I_APP_LD2450_Serial_Write(struct file *file, const char __user *user_buffer, size_t user_len, loff_t *ppos);
static ssize_t I_APP_LD2450_serial_read(struct file *file, char __user *user_buffer, size_t user_len, loff_t *ppos);

/**
 * Função chamada quando o driver for aberto;
 */
static int I_APP_LD2450_Serial_Open(struct inode *inode, struct file *file)
{
#if ( DEBUG_DRIVER == 1 )
	pr_info("I_APP_LD2450_Serial_Open...\n");
	pr_info("Device Numbers: %d %d\n", imajor(inode), iminor(inode));
#endif
	return 0;
}

/**
 * Função chamada quando um app escrever no driver;
 */
static ssize_t I_APP_LD2450_Serial_Write(struct file *file, const char __user *user_buffer, size_t user_len, loff_t *ppos) {
#if ( DEBUG_DRIVER == 1 )	
	pr_info("I_APP_LD2450_Serial_Write...\n");
#endif

	if(I_DRV_LD2450_serial_write((const char *) user_buffer, (int) user_len) == e_ret_fail) {
		printk("Error I_APP_LD2450_Serial_Write...\n");
	}
	return user_len;
}


static ssize_t I_APP_LD2450_serial_read(struct file *file, char __user *user_buffer, size_t user_len, loff_t *ppos)
{
	char message[32]; 
	int n_bytes, status;
	size_t len;	

#if ( DEBUG_DRIVER == 1 )
	pr_info("I_APP_LD2450_serial_read()...\n");
#endif	

	if( I_DRV_LD2450_Check_Fifo_Recv() ) 	//algum byte na fifo?
	{
		// if( LD2450_Mode == e_mode_command )	
		// {
			if( (n_bytes = I_DRV_LD2450_serial_read(message, sizeof(message))) > 0 ) 
			{

				if(user_len < n_bytes) {
					len = user_len;
				} else {
					len = n_bytes;
				}
				
				status = copy_to_user(user_buffer, message, len);
				if(status)
				{
					printk("Error durante copy_to_user\n");
					return -status;
				}


	#if ( DEBUG_DRIVER_SERIAL_READ == 1 )			
				pr_info("data: %.*s", message);
	#endif			
				return len;
			}
	// 	}
	// 	if( LD2450_Mode == e_mode_tracking )	
	// 	{
	// 		if( (n_bytes = I_DRV_LD2450_serial_read(message, sizeof(message))) >= 30 ) 
	// 		{				
	// 			status = copy_to_user(user_buffer, message, len);
	// 			if(status) {
	// 				printk("Error durante copy_to_user\n");
	// 				return -status;
	// 			}

	// #if ( DEBUG_DRIVER == 1 )			
	// 			pr_info("data: %.*s", n_bytes, message);
	// #endif			
	// 			return len;
	// 		}
	// 		else
	// 		{
	// 			if(user_len < n_bytes) {
	// 				len = user_len;
	// 			} else {
	// 				len = n_bytes;
	// 			}
	// 		}
	// 	}
	}	
	return 0;
}

static ssize_t ld2450_read(struct file *filep, char __user *buf, size_t len, loff_t *off)
{
    // // Read data from internal buffer or serdev
    // return copy_to_user(buf, internal_data, data_len) ? -EFAULT : data_len;

	const char *test_msg = "Hello from LD2450 driver\n";
	size_t msg_len = strlen(test_msg);
	if (copy_to_user(buf, test_msg, msg_len))
		return -EFAULT;
	return msg_len;
}

/**
 * Função de controle de entrada e saída do dispositivo por onde se comunica diretamente com o driver
 */
static long etx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    if (cmd == REG_CURRENT_TASK)
	{
#if ( DEBUG_DRIVER == 1 )    
		pr_info("etx_ioctl...\n");
#endif
		task = get_current();
        signum = SIGUSR1;
    }
    return 0;
}


//Função chamada quando o driver for fechado;
static int I_APP_LD2450_Serial_Close(struct inode *inode, struct file *file)
{
#if ( DEBUG_DRIVER == 1 )
	pr_info("I_APP_LD2450_Serial_Close...\n");
#endif
	return 0;
}

//kernel-defined struct 
static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = I_APP_LD2450_serial_read,
	//.read = ld2450_read,
	.write = I_APP_LD2450_Serial_Write,
	.open = I_APP_LD2450_Serial_Open,
	.unlocked_ioctl = etx_ioctl,
	.release = I_APP_LD2450_Serial_Close,
};

/**
 * Interface de configuração MISC;
 */
static struct miscdevice misc_device = {
	.name = "LD2450",
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &fops,
};


/**
 * --------------------------------------------------------------------------------------
 * Leitura do Status da Conexão Bluetooth via Parametro;
 * Identifica se há algum outro rádio bluetooth pareado;
 */
static int LD2450_callback(const char *val, const struct kernel_param *kp)
{
    if (param_set_int(val, kp) == 0) {
        printk(KERN_INFO "LD2450_Mode Cb = %d\n", LD2450_Mode);

        // If user has registered a task (via ioctl), send signal
        if (task)
		{
            struct kernel_siginfo info;
            memset(&info, 0, sizeof(struct siginfo));
            info.si_signo = SIGUSR1;
            info.si_code = SI_QUEUE;
            info.si_int = LD2450_Mode;  // send current value as payload

            if ( send_sig_info(SIGUSR1, &info, task ) < 0 )
			{
                printk(KERN_INFO "Erro ao enviar sinal para o processo\n");
            }
        }
        return 0;
    }
    return -1;
}

const struct kernel_param_ops LD2450_Op_params = 
{
        .set = &LD2450_callback,
        .get = &param_get_int, 
};
module_param_cb(LD2450_Mode, &LD2450_Op_params, &LD2450_Mode, 0660 );


/**
 * --------------------------------------------------------------------------------------
 * Aplicação LD2450;
 */
static struct task_struct *kthread_1;
static int t1 = 1;

static int thread_function(void * thread_nr) 
{
	int t_nr = *(int *) thread_nr;
	int ret;

#if ( DEBUG_DRIVER == 1 )
	printk("kthread - Thread %d starting...\n", t_nr);
#endif

	//Inicializa Modulo radar Hi-Link LD2450;
	while(I_DRV_LD2450_Init() != 0)
	{ 
		printk("Error... falha na inicialização do LD2450\n");
		
			while (!kthread_should_stop())
			{
				msleep(100);
			}

		return 1;
	}	
	
		
	// /**
	//  * Obtém o número IRQ para o pino GPIO_25
	//  */
	// GPIO_irqNumber = gpio_to_irq(GPIO_25);
	// printk("A IRQ para o pino GPIO_25 é: %i\n", GPIO_irqNumber);
	
	// /**
	//  * request_irq -> aloca recursos de interrupção e habilita linha de interrupção e o tratamento de IRQ
	//  * 
	//  * Parametros: irq, handler, irqflag, devname, dev_id
	//  * 
	//  * irq_flags:
	//  * UMH_FREEZING -> quando GPIO muda de 0 para 1
	//  * UMH_DISABLED -> quando GPIO muda de 1 para 0
	//  */
	// ret=request_irq(GPIO_irqNumber, int_func, UMH_FREEZING, "int_func", NULL);     

	// if (ret==0)
	// {
	// 	printk("O módulo de interrupção foi inicializado.\n");
	// }
	// else
	// {
	// 	printk("Ocorreu uma falha ao inicializar o módulo de interrupção. Código de Erro/Falha =%i\n", ret);
	// }
	

	printk("LD2450 - driver init successfull\n");
	LD2450_Mode = e_mode_tracking;

	// /* Working loop */
	while(!kthread_should_stop())
	{	
	 	msleep(100);
	}

#if ( DEBUG_DRIVER == 1 )	
	printk("kthread - Thread %d finished execution!\n", t_nr);
#endif
	return 0;
}


/**
 * Chamado quando o módulo for carregado no Kernel;
 */
static int __init HiLink_LD2450_driver_init(void) 
{
	int status;	
	printk("LD2450 - device driver version %d.%d.%d initing...\n", VERSION, SUB_VERSION, DEV_VERSION);
	
	if(serdev_device_driver_register(&serdev_driver))
	{
		printk("Error! Could not load driver\n");
		return -1;
	}
	msleep(10);

#if ( DEBUG_DRIVER == 1 )
	pr_info("Register misc device\n");
#endif

	status = misc_register(&misc_device);
	if(status) {
		printk("Error during Register misc device\n");
		return -status;
	}
	msleep(10);

	/** Habilita Pino Enable */
	if(gpio_request(GPIO_23, "GPIO_23"))
	{
		printk("Can not allocate GPIO_23\n");
		goto Error;
	}
	else
	{
#if ( DEBUG_DRIVER == 1 )		
		pr_info("GPIO 23 allocated\n");
#endif
	}

	/** Define a direção do Pino Enable */
	if(gpio_direction_output(GPIO_23, 0))
	{
		printk("Can not set GPIO 23 to output!\n");
		goto Error;
	}
	else
	{
#if ( DEBUG_DRIVER == 1 )		
		pr_info("GPIO 23 set as output\n");
#endif
	}
	
	/** Estado inicial do pino */
    gpio_set_value(GPIO_23, 0); //enable = 0

	if (kfifo_alloc(&ld2450_fifo, FIFO_SIZE, GFP_KERNEL)) {
		pr_err("LD2450: failed to allocate FIFO\n");
		return -ENOMEM;
	}

	/** Cria Threads responsável em enviar os comandos via Bluetooth */
    kthread_1 = kthread_create(thread_function, &t1, "kthread_1");
	if( kthread_1 != NULL )
	{
		/* Let's start the thread */
		wake_up_process(kthread_1);
#if ( DEBUG_DRIVER == 1 )		
		pr_info("kthread - Thread 1 was created and is running now!\n");
#endif
	}
		
	return 0;

Error:
	gpio_free(GPIO_23);
	return -1;
}

/**
 * Função chamada quando o driver for removido do kernel;
 */
static void __exit HiLink_LD2450_driver_exit(void) 
{
	printk("LD2450 - driver exiting...\n");
	
	serdev_device_driver_unregister(&serdev_driver);
	msleep(10);

	if (kthread_1)
	{
#if ( DEBUG_DRIVER == 1 )		
		pr_info("Stopping thread\n");		
#endif
		kthread_stop(kthread_1);
        kthread_1 = NULL;
#if ( DEBUG_DRIVER == 1 )		
		pr_info("Thread stopped\n");
#endif		
    }
	
	msleep(10);
	I_DRV_LD2450_Desliga_Modulo();
	gpio_free(GPIO_23);
	kfifo_free(&ld2450_fifo);
	msleep(10);

	misc_deregister(&misc_device);
	msleep(10);
	/**
	 * libera a IRQ
	 */
	//free_irq(GPIO_irqNumber, NULL);

	printk("LD2450 - driver closed!\n");
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
  * @name   I32_DRV_LD2450_MountProtToSendMsg
  * @brief  .
  * @author PRBSL
  * @date   27/04/2024
  * @param
  * @return uint32_t
  * @misc   mount msg in sensor protocol
  */
 int32_t   I32_DRV_LD2450_MountProtToSendMsg ( long int u32_cmd, uint8_t *p_u8_final_msg, uint16_t *p_u16_final_msg_lenght )
 {
   int32_t i32_ret               = e_ret_fail;
   uint16_t u16_index_byte_count = 0;
   int8_t i8_uart_Send_buff[80]  = {0};
 
   i8_uart_Send_buff[u16_index_byte_count++]  = PROT_START_BYTE_0;
   i8_uart_Send_buff[u16_index_byte_count++]  = PROT_START_BYTE_1;
   i8_uart_Send_buff[u16_index_byte_count++]  = PROT_START_BYTE_2;
   i8_uart_Send_buff[u16_index_byte_count++]  = PROT_START_BYTE_3;
 
   switch( u32_cmd )
   {
	 case CMD_SET_ENABLE_CONFIG:
	   //strcat(i8_uart_Send_buff, CMD_GET_FW_VERSION );
	   i8_uart_Send_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_0;
	   i8_uart_Send_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_1;
	   i8_uart_Send_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_2;
	   i8_uart_Send_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_3;
	   i8_uart_Send_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_4;
	   i8_uart_Send_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_5;
	 break;
 
	 case CMD_SET_DISABLE_CONFIG:
		 i8_uart_Send_buff[u16_index_byte_count++]  = SET_DISABLE_CONFIG_BYTE_0;
		 i8_uart_Send_buff[u16_index_byte_count++]  = SET_DISABLE_CONFIG_BYTE_1;
		 i8_uart_Send_buff[u16_index_byte_count++]  = SET_DISABLE_CONFIG_BYTE_2;
		 i8_uart_Send_buff[u16_index_byte_count++]  = SET_DISABLE_CONFIG_BYTE_3;
	 break;
 
	 case CMD_GET_FW_VERSION:
	   i8_uart_Send_buff[u16_index_byte_count++]  = GET_FW_VERSION_BYTE_0;
	   i8_uart_Send_buff[u16_index_byte_count++]  = GET_FW_VERSION_BYTE_1;
	   i8_uart_Send_buff[u16_index_byte_count++]  = GET_FW_VERSION_BYTE_2;
	   i8_uart_Send_buff[u16_index_byte_count++]  = GET_FW_VERSION_BYTE_3;
	   //0x0400A5000100 //Exibe MAC Address
	 break;
 
	 case CMD_GET_MAC_ADDRSS:
		 i8_uart_Send_buff[u16_index_byte_count++]  = GET_MAC_ADDRSS_BYTE_0;
		 i8_uart_Send_buff[u16_index_byte_count++]  = GET_MAC_ADDRSS_BYTE_1;
		 i8_uart_Send_buff[u16_index_byte_count++]  = GET_MAC_ADDRSS_BYTE_2;
		 i8_uart_Send_buff[u16_index_byte_count++]  = GET_MAC_ADDRSS_BYTE_3;
		 i8_uart_Send_buff[u16_index_byte_count++]  = GET_MAC_ADDRSS_BYTE_4;
		 i8_uart_Send_buff[u16_index_byte_count++]  = GET_MAC_ADDRSS_BYTE_5;
	 break;
 
	 case CMD_GET_TRACKING_MODE_SETUP:
		 i8_uart_Send_buff[u16_index_byte_count++]  = GET_TRACKING_MODE_BYTE_0;
		 i8_uart_Send_buff[u16_index_byte_count++]  = GET_TRACKING_MODE_BYTE_1;
		 i8_uart_Send_buff[u16_index_byte_count++]  = GET_TRACKING_MODE_BYTE_2;
		 i8_uart_Send_buff[u16_index_byte_count++]  = GET_TRACKING_MODE_BYTE_3;
		 //0x02009100 //Le forma de rastreamento. 0100-> 1 alvo, 0200-> multiplos alvos  ")
	 break;
 
	 case CMD_SET_BAUD_9600:   				//0x0400A1000100 	//Muda Baudrate = 9600
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_0;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_1;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_2;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_3;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_9600_BYTE;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_5;
	 break;
 
	 case CMD_SET_BAUD_19200:  				//0x0400A1000200 	//Muda Baudrate = 19200
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_0;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_1;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_2;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_3;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_19200_BYTE;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_5;
	 break;
 
	 case CMD_SET_BAUD_38400:  				//0x0400A1000300 	//Muda Baudrate = 38400
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_0;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_1;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_2;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_3;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_38400_BYTE;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_5;
	 break;
 
	 case CMD_SET_BAUD_57600:  				//0x0400A1000400 	//Muda Baudrate = 57600
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_0;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_1;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_2;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_3;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_57600_BYTE;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_5;
	 break;
 
	 case CMD_SET_BAUD_115200: 				//0x0400A1000500 	//Muda Baudrate = 115200
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_0;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_1;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_2;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_3;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_115200_BYTE;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_5;
	 break;
 
	 case CMD_SET_BAUD_230400: 				//0x0400A1000600 	//Muda Baudrate = 230400
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_0;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_1;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_2;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_3;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_230400_BYTE;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_5;
	 break;
 
	 case CMD_SET_BAUD_256000: 				//0x0400A1000700 	//Muda Baudrate = 256000
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_0;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_1;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_2;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_3;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_256000_BYTE;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_5;
	 break;
 
	 case CMD_SET_BAUD_460800: 				//0x0400A1000800 	//Muda Baudrate = 460800
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_0;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_1;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_2;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_3;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_460800_BYTE;
		 i8_uart_Send_buff[u16_index_byte_count++]  = CMD_SET_BAUD_INIT_5;
	 break;

	case CMD_RESTART_MODULE:	 //Restart module (Logical reset)
		i8_uart_Send_buff[u16_index_byte_count++] = RESTART_MODULE_BYTE_0;
		i8_uart_Send_buff[u16_index_byte_count++] = RESTART_MODULE_BYTE_1;
		i8_uart_Send_buff[u16_index_byte_count++] = RESTART_MODULE_BYTE_2;
		i8_uart_Send_buff[u16_index_byte_count++] = RESTART_MODULE_BYTE_3;
	break;

   }
 
   i8_uart_Send_buff[u16_index_byte_count++]  = PROT_END_BYTE_0;
   i8_uart_Send_buff[u16_index_byte_count++]  = PROT_END_BYTE_1;
   i8_uart_Send_buff[u16_index_byte_count++]  = PROT_END_BYTE_2;
   i8_uart_Send_buff[u16_index_byte_count++]  = PROT_END_BYTE_3;
	
	memcpy( p_u8_final_msg, i8_uart_Send_buff, u16_index_byte_count );
	*p_u16_final_msg_lenght = u16_index_byte_count;

#if ( DEBUG_DRIVER == 1 )
	pr_info("u16_index_byte_count: %d\n", u16_index_byte_count);
   
   	for (int i = 0; i < u16_index_byte_count; i++)
   	{
		pr_info("byte[%d] - %.2x", i, i8_uart_Send_buff[i]);
	}
#endif

	i32_ret = e_ret_success;
   	return i32_ret;
 }
 

 /**
  * @name   I32_DRV_LD2450_GetAckMsgFromSensor
  * @brief  .
  * @author PRBSL
  * @date   27/04/2024
  * @param
  * @return uint32_t
  * @misc   Ack answer sensor message
  */
 int32_t   I32_DRV_LD2450_GetAckMsgFromSensor ( long int u32_cmd, uint8_t *p_u8_ack_msg, uint16_t *p_u16_final_msg_lenght )
 {
   int32_t i32_ret               		= e_ret_fail;
   uint16_t u16_index_byte_count 		= 0;
   int8_t i8_uart_expected_buff[80]  	= {0};

   i8_uart_expected_buff[u16_index_byte_count++]  = PROT_START_BYTE_0;
   i8_uart_expected_buff[u16_index_byte_count++]  = PROT_START_BYTE_1;
   i8_uart_expected_buff[u16_index_byte_count++]  = PROT_START_BYTE_2;
   i8_uart_expected_buff[u16_index_byte_count++]  = PROT_START_BYTE_3;
 
   switch( u32_cmd )
   {
	 case CMD_SET_ENABLE_CONFIG:
		i8_uart_expected_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_0_ACK;
		i8_uart_expected_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_1_ACK;
		i8_uart_expected_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_2_ACK;
		i8_uart_expected_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_3_ACK;
		i8_uart_expected_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_4_ACK;
		i8_uart_expected_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_5_ACK;
		i8_uart_expected_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_6_ACK;
		i8_uart_expected_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_7_ACK;
		i8_uart_expected_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_8_ACK;
		i8_uart_expected_buff[u16_index_byte_count++]  = SET_ENABLE_CONFIG_BYTE_9_ACK;
	 break;
 
	 case CMD_SET_DISABLE_CONFIG:
	 break;
 
	 case CMD_GET_FW_VERSION:
		i8_uart_expected_buff[u16_index_byte_count++] = CMD_SET_ENABLE_CONFIG_ACK;
		i8_uart_expected_buff[u16_index_byte_count++] = GET_FW_VERSION_BYTE_0_ACK;
		i8_uart_expected_buff[u16_index_byte_count++] = GET_FW_VERSION_BYTE_1_ACK;
		i8_uart_expected_buff[u16_index_byte_count++] = GET_FW_VERSION_BYTE_2_ACK;
		i8_uart_expected_buff[u16_index_byte_count++] = GET_FW_VERSION_BYTE_3_ACK;
		i8_uart_expected_buff[u16_index_byte_count++] = GET_FW_VERSION_BYTE_4_ACK;
		i8_uart_expected_buff[u16_index_byte_count++] = GET_FW_VERSION_BYTE_5_ACK;
		i8_uart_expected_buff[u16_index_byte_count++] = GET_FW_VERSION_BYTE_6_ACK;		
	 break;
 
	 case CMD_GET_MAC_ADDRSS:
	 break;
 
	 case CMD_GET_TRACKING_MODE_SETUP:
	 break;
 
	 case CMD_SET_BAUD_9600:   				//0x0400A1000100 	//Muda Baudrate = 9600
	 break;
 
	 case CMD_SET_BAUD_19200:  				//0x0400A1000200 	//Muda Baudrate = 19200
	 break;
 
	 case CMD_SET_BAUD_38400:  				//0x0400A1000300 	//Muda Baudrate = 38400
	 break;
 
	 case CMD_SET_BAUD_57600:  				//0x0400A1000400 	//Muda Baudrate = 57600
	 break;
 
	 case CMD_SET_BAUD_115200: 				//0x0400A1000500 	//Muda Baudrate = 115200
	 break;
 
	 case CMD_SET_BAUD_230400: 				//0x0400A1000600 	//Muda Baudrate = 230400
	 break;
 
	 case CMD_SET_BAUD_256000: 				//0x0400A1000700 	//Muda Baudrate = 256000
	 break;
 
	 case CMD_SET_BAUD_460800: 				//0x0400A1000800 	//Muda Baudrate = 460800;
	 break;

	 case CMD_RESTART_MODULE:	 //Restart module (Logical reset)
	 	i8_uart_expected_buff[u16_index_byte_count++] = RESTART_MODULE_BYTE_0_ACK;
	 	i8_uart_expected_buff[u16_index_byte_count++] = RESTART_MODULE_BYTE_1_ACK;
	 	i8_uart_expected_buff[u16_index_byte_count++] = RESTART_MODULE_BYTE_2_ACK;
	 	i8_uart_expected_buff[u16_index_byte_count++] = RESTART_MODULE_BYTE_3_ACK;
	 	i8_uart_expected_buff[u16_index_byte_count++] = RESTART_MODULE_BYTE_4_ACK;
	 	i8_uart_expected_buff[u16_index_byte_count++] = RESTART_MODULE_BYTE_5_ACK;
 	break;
   }

   i8_uart_expected_buff[u16_index_byte_count++]  = PROT_END_BYTE_0;
   i8_uart_expected_buff[u16_index_byte_count++]  = PROT_END_BYTE_1;
   i8_uart_expected_buff[u16_index_byte_count++]  = PROT_END_BYTE_2;
   i8_uart_expected_buff[u16_index_byte_count++]  = PROT_END_BYTE_3;

	memcpy( p_u8_ack_msg, i8_uart_expected_buff, u16_index_byte_count );
	*p_u16_final_msg_lenght = u16_index_byte_count;

#if ( DEBUG_DRIVER == 1 )
	pr_info("u16_index_byte_count: %d\n", u16_index_byte_count);
   
   	for (int i = 0; i < u16_index_byte_count; i++)
   	{
		pr_info("byte[%d] - %.2x", i, i8_uart_expected_buff[i]);
	}
#endif
   i32_ret = e_ret_success;
   return i32_ret;
 }
 
 /**
   * @name   I_DRV_LD2450_Module_setup
   * @brief  .
   * @author PRBSL
   * @date   29/09/2024
   * @param  void
   * @return int
   * @misc
   */
static int I_DRV_LD2450_Module_setup ( void )
 {
	int32_t 	u32_ret                 	= e_ret_fail;
	uint8_t		u8_msg_to_sensor[40]    	= {0};
	uint8_t 	u8_msg_from_sensor[60]  	= {0};
	uint8_t 	u8_msg_ack_from_sensor[60]  = {0};	
	uint16_t 	u16_msg_to_sensor_size = 0;
	uint16_t 	u16_msg_ack_size = 0;
	int 		timeout = 1;
	int 		ret = 0;

	msleep(1000); //PRBSL-03/05/2025:Delay necessario!
#if ( DEBUG_DRIVER == 1 )	
	pr_info("Reseting fifo... \n");
#endif	
	kfifo_reset (&ld2450_fifo);


	//PRBSL-03/05/2025:Set module to command mode
	I32_DRV_LD2450_MountProtToSendMsg ( CMD_SET_ENABLE_CONFIG, u8_msg_to_sensor, &u16_msg_to_sensor_size );
	I32_DRV_LD2450_GetAckMsgFromSensor ( CMD_SET_ENABLE_CONFIG, u8_msg_ack_from_sensor, &u16_msg_ack_size );
	printk("LD2450 - Setting to command mode...");
	u32_ret = I_DRV_LD2450_write_cmd_wait_ack_answ( u8_msg_to_sensor, u16_msg_to_sensor_size, u8_msg_ack_from_sensor, u16_msg_ack_size, 1000 ); 
	if( u32_ret != e_ret_success)
	{
		printk("LD2450 - Set to command mode - FAILS");
 		goto end;
	}
	printk("LD2450 - In serial command mode - SUCCESS");


	//PRBSL-04/05/2025:Set Module baudrate 256000
	printk("LD2450 - Setting baudrate 256000 kbps...");
	// memset(&u8_msg_to_sensor, 0, sizeof(u8_msg_to_sensor));
	// memset(&u8_msg_ack_from_sensor, 0, sizeof(u8_msg_ack_from_sensor));
	// I32_DRV_LD2450_MountProtToSendMsg ( CMD_GET_FW_VERSION, u8_msg_to_sensor, &u16_msg_to_sensor_size );
	// I32_DRV_LD2450_GetAckMsgFromSensor ( CMD_GET_FW_VERSION, u8_msg_ack_from_sensor, &u16_msg_ack_size );
	// u32_ret = I_DRV_LD2450_write_cmd_wait_ack_answ( u8_msg_to_sensor, u16_msg_to_sensor_size, u8_msg_ack_from_sensor, u16_msg_ack_size, 1000 ); 
	// if( u32_ret != e_ret_success)
	// {
	// 	printk("LD2450 - Set baudrate 256000 kbps - FAILS");
		// 	goto end;
	// }
	printk("LD2450 - Set baudrate 256000 kbps - SUCCESS");
	

	//PRBSL-04/05/2025:Get firmware version
	printk("LD2450 - Getting firmware version...");
	// memset(&u8_msg_to_sensor, 0, sizeof(u8_msg_to_sensor));
	// memset(&u8_msg_ack_from_sensor, 0, sizeof(u8_msg_ack_from_sensor));
	// I32_DRV_LD2450_MountProtToSendMsg ( CMD_GET_FW_VERSION, u8_msg_to_sensor, &u16_msg_to_sensor_size );
	// I32_DRV_LD2450_GetAckMsgFromSensor ( CMD_GET_FW_VERSION, u8_msg_ack_from_sensor, &u16_msg_ack_size );
	// u32_ret = I_DRV_LD2450_write_cmd_wait_ack_answ( u8_msg_to_sensor, u16_msg_to_sensor_size, u8_msg_ack_from_sensor, u16_msg_ack_size, 1000 ); 
	// if( u32_ret != e_ret_success)
	// {
	// 	printk("LD2450 - Get firmware version - FAILS");
 	// 	goto end;
	// }
	printk("LD2450 - Get firmware version - Success");


	//PRBSL-04/05/2025:Get Module MAC ADDRESS
	printk("LD2450 - Getting MAC ADDRESS...");
	// memset(&u8_msg_to_sensor, 0, sizeof(u8_msg_to_sensor));
	// memset(&u8_msg_ack_from_sensor, 0, sizeof(u8_msg_ack_from_sensor));
	// I32_DRV_LD2450_MountProtToSendMsg ( CMD_GET_FW_VERSION, u8_msg_to_sensor, &u16_msg_to_sensor_size );
	// I32_DRV_LD2450_GetAckMsgFromSensor ( CMD_GET_FW_VERSION, u8_msg_ack_from_sensor, &u16_msg_ack_size );
	// u32_ret = I_DRV_LD2450_write_cmd_wait_ack_answ( u8_msg_to_sensor, u16_msg_to_sensor_size, u8_msg_ack_from_sensor, u16_msg_ack_size, 1000 ); 
	// if( u32_ret != e_ret_success)
	// {
	// 	printk("LD2450 - Get MAC ADDRESS - FAILS");
		// 	goto end;
	// }
	printk("LD2450 - Got MAC ADDRESS - Success");


	//PRBSL-04/05/2025:Set modules bluetooth off
	printk("LD2450 - Setting bluetooth off...");
	// memset(&u8_msg_to_sensor, 0, sizeof(u8_msg_to_sensor));
	// memset(&u8_msg_ack_from_sensor, 0, sizeof(u8_msg_ack_from_sensor));
	// I32_DRV_LD2450_MountProtToSendMsg ( CMD_GET_FW_VERSION, u8_msg_to_sensor, &u16_msg_to_sensor_size );
	// I32_DRV_LD2450_GetAckMsgFromSensor ( CMD_GET_FW_VERSION, u8_msg_ack_from_sensor, &u16_msg_ack_size );
	// u32_ret = I_DRV_LD2450_write_cmd_wait_ack_answ( u8_msg_to_sensor, u16_msg_to_sensor_size, u8_msg_ack_from_sensor, u16_msg_ack_size, 1000 ); 
	// if( u32_ret != e_ret_success)
	// {
	// 	printk("LD2450 - Set bluetooth off - FAILS");
		// 	goto end;
	// }
	printk("LD2450 - Set bluetooth off - Success");


	//PRBSL-04/05/2025:Set tracking mode 1 target
	printk("LD2450 - Setting tracking mode 1 target...");
	// memset(&u8_msg_to_sensor, 0, sizeof(u8_msg_to_sensor));
	// memset(&u8_msg_ack_from_sensor, 0, sizeof(u8_msg_ack_from_sensor));
	// I32_DRV_LD2450_MountProtToSendMsg ( CMD_GET_FW_VERSION, u8_msg_to_sensor, &u16_msg_to_sensor_size );
	// I32_DRV_LD2450_GetAckMsgFromSensor ( CMD_GET_FW_VERSION, u8_msg_ack_from_sensor, &u16_msg_ack_size );
	// u32_ret = I_DRV_LD2450_write_cmd_wait_ack_answ( u8_msg_to_sensor, u16_msg_to_sensor_size, u8_msg_ack_from_sensor, u16_msg_ack_size, 1000 ); 
	// if( u32_ret != e_ret_success)
	// {
	// 	printk("LD2450 - Set tracking mode 1 target - FAILS");
		// 	goto end;
	// }
	printk("LD2450 - Set tracking mode 1 target - Success");


	//PRBSL-04/05/2025:Exiting command mode
	printk("LD2450 - Exiting command mode...");
	memset(&u8_msg_to_sensor, 0, sizeof(u8_msg_to_sensor));
	memset(&u8_msg_ack_from_sensor, 0, sizeof(u8_msg_ack_from_sensor));
	I32_DRV_LD2450_MountProtToSendMsg ( CMD_RESTART_MODULE, u8_msg_to_sensor, &u16_msg_to_sensor_size );
	I32_DRV_LD2450_GetAckMsgFromSensor ( CMD_RESTART_MODULE, u8_msg_ack_from_sensor, &u16_msg_ack_size );
	u32_ret = I_DRV_LD2450_write_cmd_wait_ack_answ( u8_msg_to_sensor, u16_msg_to_sensor_size, u8_msg_ack_from_sensor, u16_msg_ack_size, 1000 ); 
	if( u32_ret != e_ret_success)
	{
	 	printk("LD2450 - Exit command mode - FAILS");
	 	goto end;
	}
	printk("LD2450 - Starting tracking...");
	
	
	if( u32_ret == e_ret_success ) 
	{
#if ( DEBUG_DRIVER == 1 )		  
		pr_info("LD2450 module now in serial command mode!!!");
#endif		
	}
	else
	{
		printk("Settup module - FAILS");
	}	

   end:
   return u32_ret;
 }
 
 

 //Função responsável pela configuração do modulo LD2450 pela serial;
static int I_DRV_LD2450_Init( void )
{
	int i_ret = e_ret_fail;

    i_ret = I_DRV_LD2450_Liga_Modulo();
	if ( i_ret != e_ret_success )
	{
		printk("Fail - I_DRV_LD2450_Liga_Modulo()\n");
		return i_ret;
	}

	i_ret = I_DRV_LD2450_Module_setup();
	if ( i_ret != e_ret_success )
	{
		printk("Fail - I_DRV_LD2450_Module_setup()\n");
		return i_ret;
	}

	//I_DRV_LD2450_Clear_Fifo();
	//LD2450_Mode = 1;

    return i_ret;
}

//Função responsável por ler as mensagens recebidas do radar
static int I_DRV_LD2450_serial_read( char * message, int size )
{
    int index = 0;
	int i_message = 0; 
	int rec;
	char i;
   
   /**
    * Define timeout de 1s;
    */
    int timeout = 10000; // 20*50
	//usleep(10000); // 10ms delay
    while( --timeout )
    {
        /**
         * Algum byte presente na fifo? 
         * Caso sim, realiza a leitura e armazena em 'message';
         */
        if(kfifo_is_empty(&ld2450_fifo) != 1)
        {
			rec = kfifo_get(&ld2450_fifo, &i);			
            message[index] = (char)i;

#if ( DEBUG_KERNEL_FIFO == 1 )
			pr_info("kfifo_get: 0x%.2x", (char)i);			
#endif		

			if (
				message[(index-3)] == 0xAA && message[(index-2)] == 0xFF &&
				message[(index-1)] == 0x03 && message[index] == 0x00 )
			{
#if ( DEBUG_KERNEL_FIFO == 1 )
			pr_info("Pos cabecalho");			
#endif				
				index = 3;
			}


			if ( ( message[(index-1)] == 0x55 ) && ( message[index] == 0xCC ) )
		 	{				

#if ( DEBUG_KERNEL_FIFO == 1 )
			pr_info("Fim da mensagem");			
#endif				
				i_message = 1;
				index++;
				break;

			}
            index++;
        }
		
		//msleep(50);
		//msleep(20);
    }
	
	if( i_message == 1 )
	{
#if ( DEBUG_KERNEL_FIFO == 1 )
			pr_info("\n");			
#endif
		return (index);
	}
	else
	{
    	return -1;
	}
}


static int I_DRV_LD2450_write_cmd_wait_ack_answ( const char * str_cmd, int str_cmd_size, const char * str_return, int str_return_size, int timeout )
{
    int ret = e_ret_fail;

#if ( DEBUG_DRIVER == 1 )
	pr_info("str_cmd: %d\n", str_cmd_size);

	for (int i = 0; i < str_cmd_size; i++)
	{
		pr_info("0x%.2x", str_cmd[i]);
	}

	pr_info("u16_msg_ack_size: %d\n", str_return_size);

	for (int i = 0; i < str_return_size; i++)
	{
		pr_info("0x%.2x", str_return[i]);
	}   
#endif

	//str_cmd deverá ser uma string de tamanho > 0;
    if( ( str_cmd_size == 0 ) || str_return_size == 0 )
	{
#if ( DEBUG_DRIVER == 1 )
		pr_info("FAIL: str_cmd_size or str_return_size = 0");
#endif		
		return ret; 
	} 

    if( I_DRV_LD2450_serial_write(str_cmd, str_cmd_size ) == e_ret_fail )
	{
		printk("Error I_DRV_LD2450_serial_write ... \n");
	}
    
    msleep(100);

    if( strlen(str_return) == 0 || timeout == 0 )
    {
        return ret;
    }

    ret = I_DRV_LD2450_serial_command_ack_answ_compare( str_return, timeout, str_return_size );
#if ( DEBUG_DRIVER == 1 )
	pr_info("Retorno da comparacao: %d", ret);
#endif

	return ret;
}


static int I_DRV_LD2450_serial_command_ack_answ_compare( const char * sequence, int timeout, int i_expected_size )
{
   //int i, ret;
   char i;
   int ret;
   int i_count = i_expected_size;
   int i_bytes_count = 0;
   char c_fifo_got_sequence[i_expected_size];

#if ( DEBUG_DRIVER == 1 )
	// pr_info("msg size: %d\n", i_expected_size);
	// pr_info("Expected msg:\n");
	// for (int j = 0; j < i_expected_size; j++)
	// {
	// 	pr_info("0x%.2x\n", sequence[j]);
	// }
#endif

   while( --timeout )
   {
		if ( i_expected_size == 0 )
		{
			pr_info("Return message size invalid\n");
			ret = e_ret_fail;
			goto end;
		}
		else
		{
			if( i_count == 0 )
			{				
				ret = e_ret_success;
				break;
			}
		}

       if( kfifo_is_empty( &ld2450_fifo ) != 1 )
       {
           ret = kfifo_get(&ld2450_fifo, &i);

		   i_count--;

#if ( PRINT_SERIAL_FIFO == 1 )		   
		   pr_info("kfifo_get: 0x%.2x", (char)i);
#endif
			c_fifo_got_sequence[i_bytes_count] = i;
			
			i_bytes_count++;
			if( i_bytes_count > i_expected_size )
			{
				i_bytes_count--;
				ret = e_ret_success;
				break;
			}
		}
		msleep(5); 
   }

#if ( DEBUG_DRIVER == 1 )
	// pr_info("i_bytes_count: %d\n", i_bytes_count);
	// pr_info("c_fifo_got_sequence:\n");
	// for (int j = 0; j < i_bytes_count; j++)
	// {
	// 	pr_info("0x%.2x\n", c_fifo_got_sequence[j]);
	// }
#endif   

	for (int j = 0; j < i_expected_size; j++)
	{
		if( sequence[j] != c_fifo_got_sequence[j] )
		{
			ret = e_ret_fail_to_receive_uart;
			printk("Comparasion received ACK FAILS");
			goto end;
		}
#if ( DEBUG_DRIVER == 1 )
		else
		{
			pr_info("sequence[%d]=0x%.2x  <==>  c_fifo_got_sequence[%d]=0x%.2x", j, sequence[j], j, c_fifo_got_sequence[j]);			
		}
#endif
	}


#if ( DEBUG_DRIVER == 1 )
		pr_info("Comparassion successfull\n");
#endif
	
   end:
   return ret;
}


/**
  * @name   I_APP_LD2450_Logical_Reset_Module
  * @brief  .
  * @author PRBSL
  * @date   04/05/2025
  * @param	void
  * @return int
  */
 static int I_APP_LD2450_Logical_Reset_Module( void )
 {
	int i_ret = e_ret_fail;

	//goto end;
   

	end:
	return i_ret;
 }



module_init(HiLink_LD2450_driver_init);
module_exit(HiLink_LD2450_driver_exit);


#endif //#if( SYSTEM_PLATFORM == RASPBERRY)