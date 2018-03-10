#include <linux/dma-mapping.h>  
#include <linux/module.h>  
#include <linux/sched.h>  
#include <linux/completion.h>  
#include <linux/interrupt.h>  
#include <linux/clk.h>  
#include <linux/platform_device.h>  
#include <linux/atmel_pdc.h>  
#include <asm/io.h>  
#include <asm/system.h>  
#include <asm/uaccess.h>  
#include <linux/semaphore.h>  
#include <linux/kernel.h>  
#include <linux/cdev.h>  
#include <linux/types.h>  
#include <linux/fs.h>  
#include <linux/input.h>  
#include <linux/errno.h>  
#include <linux/irq.h>  
#include <linux/debugfs.h>  
#include <linux/seq_file.h>  
#include <linux/list.h>  
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <plat/gpmc.h>
#include "sx1278.h"

  
#define DEVICE_MAJOR    ( 0 )
#define DEVICE_NAME		( "sx1278" )

#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))
#define SCK 	GPIO_TO_PIN(3, 14)
#define MOSI	GPIO_TO_PIN(3, 15)
#define MISO	GPIO_TO_PIN(3, 16)
#define CS		GPIO_TO_PIN(0, 29)
#define RESET   GPIO_TO_PIN(2, 22)

#define CMD_RESET   ( 0X9002 )
extern void gpio_init(void);

unsigned char gSendBuffer[64];

char *spi_sck, *spi_mosi, *spi_miso, *spi_cs, *spi_rst;

//--------------------

unsigned char Frequency[3] = {0x6c,0x80,0x00};
unsigned char SpreadingFactor=11;  //7-12,��Ƶ����ѡСһЩ������ʱ������һЩ��
unsigned char CodingRate = 2;        //1-4
unsigned char Bw_Frequency = 7;      //6-9
unsigned char powerValue = 15;      //�������ã����Ե�ʱ�����������Сһ��
/*
RegPaconfig 0x09
bit			name 				 discription
 7		 PaSelect			 0--> RFO pin, Output power is limited to +14dBm
                                         1--> PA_BOOST pin, Output power is limited to +20dBm

6~4		 MaxPower      Select max output power:Pmax=10.8+0.6*MaxPower [dBm]

3~0		 OutputPower   if PaSelect=0, Pout=Pmax-(15-OutputPower)
                                         if PaSelect=1, Pout=17-(15-OutputPower)
*/
unsigned char   power_data[16] = {0X80, 0X81, 0X82, 0X83, 0X84, 0X85, 0X86, 0X87, 0X88, 
                                        0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f};

unsigned char   RF_EX0_STATUS;

unsigned char   CRC_Value;

//unsigned char   SX1278_RLEN;

void SX1276WriteBuffer(unsigned char addr, unsigned char buffer)
{ 
    gSwitchEnStatus(enOpen); //NSS = 0;
    gByteWritefunc(addr | 0x80);
    gByteWritefunc(buffer);
    gSwitchEnStatus(enClose); //NSS = 1;
    //ndelay(100);//sct
}
    
unsigned char SX1276ReadBuffer(unsigned char addr)
{
    unsigned char Value;
    gSwitchEnStatus(enOpen); //NSS = 0;
    gByteWritefunc(addr & 0x7f);
    Value = gByteReadfunc();
    gSwitchEnStatus(enClose);//NSS = 1;
    //ndelay(100);//sct
    return Value; 
}
    
void SX1276LoRaSetOpMode(RFMode_SET opMode)
{
    unsigned char opModePrev;
    opModePrev = SX1276ReadBuffer(REG_LR_OPMODE);  //��0x01ģʽ�Ĵ���
    opModePrev &= 0xf8;  //�������λ
    opModePrev |= (unsigned char)opMode; //�����β�
    SX1276WriteBuffer(REG_LR_OPMODE, opModePrev); //����д��ȥ	
}

void SX1276LoRaFsk(Debugging_fsk_ook opMode)
{
    unsigned char opModePrev;
    opModePrev = SX1276ReadBuffer(REG_LR_OPMODE); //��0x01ģʽ�Ĵ���
    opModePrev &= 0x7F; //�������λ
    opModePrev |= (unsigned char)opMode;  //�����β�
    SX1276WriteBuffer(REG_LR_OPMODE, opModePrev); //����д��ȥ		
}
    
void SX1276LoRaSetRFFrequency(void)
{
    SX1276WriteBuffer(REG_LR_FRFMSB, Frequency[0]);  //д0x06�Ĵ���
    SX1276WriteBuffer(REG_LR_FRFMID, Frequency[1]);  //д0x07�Ĵ���
    SX1276WriteBuffer(REG_LR_FRFLSB, Frequency[2]);  //д0x08�Ĵ���
}

void SX1276LoRaSetRFPower(unsigned char power)
{
    //Set Pmax to +20dBm for PA_HP, Must turn off PA_LF or PA_HF, and set RegOcp
    //SX1276WriteBuffer( REG_LR_PACONFIG,  power_data[power] ); //��û����һ�䣬Ƶ���Ǽ����ⲻ���ź�,���ǿ��Խ��������շ���
    //SX1276WriteBuffer( REG_LR_OCP, 0x3f);  //add by skay,20160810, д���������Ĵ�����
    SX1276WriteBuffer(REG_LR_PADAC, 0x87);  //high power
    SX1276WriteBuffer(REG_LR_PACONFIG, power_data[power]); //��û����һ�䣬Ƶ���Ǽ����ⲻ���ź�,���ǿ��Խ��������շ���
}

    
void SX1276LoRaSetSpreadingFactor(unsigned char factor)
{
    unsigned char RECVER_DAT;
    SX1276LoRaSetNbTrigPeaks(3); //0x03-->SF7 to SF12
    RECVER_DAT = SX1276ReadBuffer(REG_LR_MODEMCONFIG2); //��0x1E�Ĵ���  
    RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG2_SF_MASK) | (factor << 4);
    SX1276WriteBuffer(REG_LR_MODEMCONFIG2, RECVER_DAT);	 
}
    
void SX1276LoRaSetNbTrigPeaks(unsigned char value)
{
    unsigned char RECVER_DAT;
    RECVER_DAT = SX1276ReadBuffer(0x31);  //Read RegDetectOptimize,
    RECVER_DAT = (RECVER_DAT & 0xF8) | value; //process;
    SX1276WriteBuffer(0x31, RECVER_DAT);  //write back;
}
    
void SX1276LoRaSetErrorCoding(unsigned char value)
{	
    unsigned char RECVER_DAT;
    RECVER_DAT = SX1276ReadBuffer(REG_LR_MODEMCONFIG1); //��0x1D�Ĵ���
    RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG1_CODINGRATE_MASK) | (value << 1);
    SX1276WriteBuffer(REG_LR_MODEMCONFIG1, RECVER_DAT);
}
    
void SX1276LoRaSetPacketCrcOn(BOOL_t enable)
{	
    unsigned char RECVER_DAT;
    RECVER_DAT = SX1276ReadBuffer(REG_LR_MODEMCONFIG2);  //��0x1E�Ĵ��� 
    RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) | (enable << 2);
    SX1276WriteBuffer(REG_LR_MODEMCONFIG2, RECVER_DAT);
}
    
void SX1276LoRaSetSignalBandwidth(unsigned char bw)
{
    unsigned char RECVER_DAT;
    RECVER_DAT = SX1276ReadBuffer(REG_LR_MODEMCONFIG1);  //��0x1D�Ĵ���
    RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG1_BW_MASK) | (bw << 4);
    SX1276WriteBuffer(REG_LR_MODEMCONFIG1, RECVER_DAT);
}
    
void SX1276LoRaSetImplicitHeaderOn(BOOL_t enable)
{
    unsigned char RECVER_DAT;
    RECVER_DAT = SX1276ReadBuffer(REG_LR_MODEMCONFIG1);  //��0x1D�Ĵ���
    RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) | (enable);
    SX1276WriteBuffer(REG_LR_MODEMCONFIG1, RECVER_DAT);
}
    
void SX1276LoRaSetSymbTimeout(unsigned int value)
{
    unsigned char RECVER_DAT[2];
    RECVER_DAT[0] = SX1276ReadBuffer(REG_LR_MODEMCONFIG2);    //��0x1E�Ĵ���
    RECVER_DAT[1] = SX1276ReadBuffer(REG_LR_SYMBTIMEOUTLSB);  //��0x1F�Ĵ���
    RECVER_DAT[0] = (RECVER_DAT[0] & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) | ((value >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK);
    RECVER_DAT[1] = value & 0xFF;
    SX1276WriteBuffer(REG_LR_MODEMCONFIG2, RECVER_DAT[0]);
    SX1276WriteBuffer(REG_LR_SYMBTIMEOUTLSB, RECVER_DAT[1]);
}
    
void SX1276LoRaSetPayloadLength(unsigned char value)
{
    SX1276WriteBuffer(REG_LR_PAYLOADLENGTH, value);  //д0x22�Ĵ���
} 
    
#if 0
void SX1276LoRaSetPreamLength(unsigned int value)
{
    unsigned char a[2];
    a[0] = (value & 0xff00) >> 8;
    a[1] = value & 0xff;
    SX1276WriteBuffer(REG_LR_PREAMBLEMSB, a[0]);
    SX1276WriteBuffer(REG_LR_PREAMBLELSB, a[1]);
}
#endif

void SX1276LoRaSetMobileNode(BOOL_t enable)
{	 
    unsigned char RECVER_DAT;
    RECVER_DAT = SX1276ReadBuffer(REG_LR_MODEMCONFIG3);  //��0x26�Ĵ���
    RECVER_DAT = (RECVER_DAT & RFLR_MODEMCONFIG3_MOBILE_NODE_MASK) | (enable << 3);
    SX1276WriteBuffer(REG_LR_MODEMCONFIG3, RECVER_DAT);
}

void SX1276LORA_INT(void)
{
    SX1276LoRaSetOpMode(Sleep_mode);  //����˯��ģʽ
    SX1276LoRaFsk(LORA_mode);	     // ������Ƶģʽ
    SX1276LoRaSetOpMode(Stdby_mode);  // ����Ϊ��ͨģʽ
    SX1276WriteBuffer(REG_LR_DIOMAPPING1, GPIO_VARE_1); //д0x40�Ĵ���,DIO����ӳ������
    SX1276WriteBuffer(REG_LR_DIOMAPPING2, GPIO_VARE_2); //д0x41�Ĵ���
    SX1276LoRaSetRFFrequency();  //Ƶ������
    SX1276LoRaSetRFPower(powerValue);  //��������
    SX1276LoRaSetSpreadingFactor(SpreadingFactor);	 // ��Ƶ��������
    SX1276LoRaSetErrorCoding(CodingRate);		 //��Ч���ݱ�
    SX1276LoRaSetPacketCrcOn(true);			 //CRC У���
    SX1276LoRaSetSignalBandwidth(Bw_Frequency);	 //������Ƶ����, 125khz
    SX1276LoRaSetImplicitHeaderOn(false);		 //ͬ��ͷ������ģʽ
    SX1276LoRaSetPayloadLength(0xff);
    SX1276LoRaSetSymbTimeout(0x3FF);
    SX1276LoRaSetMobileNode(true); 			 // �����ݵ��Ż�

    RF_RECEIVE();
}
    
void FUN_RF_SENDPACKET(unsigned char *RF_TRAN_P, unsigned char LEN)
{	
    unsigned char ASM_i;
    
    SX1276LoRaSetOpMode(Stdby_mode);
    SX1276WriteBuffer(REG_LR_HOPPERIOD, 0);	//����Ƶ������
    SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_TXD_Value);	//�򿪷����ж�
    SX1276WriteBuffer(REG_LR_PAYLOADLENGTH, LEN);	 //������ݰ�
    SX1276WriteBuffer(REG_LR_FIFOTXBASEADDR, 0); //дTx FIFO��ַ
    SX1276WriteBuffer(REG_LR_FIFOADDRPTR, 0); //SPI interface address pointer in FIFO data buffer
    gSwitchEnStatus(enOpen);   //��Ƭѡ
    gByteWritefunc(0x80);

    for (ASM_i = 0; ASM_i < LEN; ASM_i++) {
        gByteWritefunc(RF_TRAN_P[ASM_i]);
        //printk("%x\n",*RF_TRAN_P);//test
        //RF_TRAN_P++;
    }
    gSwitchEnStatus(enClose);  //��Ƭѡ
    SX1276WriteBuffer(REG_LR_DIOMAPPING1, 0x40);  //�����ж�ӳ�䵽DIO0����
    SX1276WriteBuffer(REG_LR_DIOMAPPING2, 0x00);
    SX1276LoRaSetOpMode(Transmitter_mode);     //����Ϊ����ģʽ
}
void RF_RECEIVE(void)
{
    SX1276LoRaSetOpMode(Stdby_mode);
    SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);  //0x11,�򿪽����ж�
    SX1276WriteBuffer(REG_LR_HOPPERIOD,	PACKET_MIAX_Value);//0x24�Ĵ���
    SX1276WriteBuffer(REG_LR_DIOMAPPING1, 0X00); //DIO����ӳ�����ã���Ĭ��
    SX1276WriteBuffer(REG_LR_DIOMAPPING2, 0X00);	
    SX1276LoRaSetOpMode(Receiver_mode);  //����Ϊ��������ģʽ
}

//**************�������ж����洦��Ĵ���*******************************
unsigned char recv[10];
unsigned char RF_REC_RLEN_i;
unsigned short SX1278_Interupt(unsigned char *rxdata)
{
    unsigned short SX1278_RLEN;
    char i;//test
    
    SX1278_RLEN = 0;
    RF_EX0_STATUS = SX1276ReadBuffer(REG_LR_IRQFLAGS); 
    if ((RF_EX0_STATUS & 0x40) == 0x40) { //�������
        //printk("rx1 \n");//test
        CRC_Value = SX1276ReadBuffer(REG_LR_MODEMCONFIG2);
        if ((CRC_Value & 0x04) == 0x04) { //�Ƿ��CRCУ��
            SX1276WriteBuffer(REG_LR_FIFOADDRPTR, 0x00);
            SX1278_RLEN = SX1276ReadBuffer(REG_LR_NBRXBYTES); //��ȡ���һ�������ֽ���
            gSwitchEnStatus(enOpen);
            gByteWritefunc(0x00);
            if (SX1278_RLEN > 10) { //���ղ�����10���ֽ�
                SX1278_RLEN = 10;	
            }		
            for (RF_REC_RLEN_i = 0; RF_REC_RLEN_i < SX1278_RLEN; RF_REC_RLEN_i++) {
                rxdata[RF_REC_RLEN_i] = gByteReadfunc();		
            }
            gSwitchEnStatus(enClose);
        }
        //fqcRecvData(recv,SX1278_RLEN);  //�����½��յ�������
/*
        for(i=0; i<SX1278_RLEN; i++)
    {
        printk("%x\n",rxdata[i]);//test		
    }
    * */
        SX1276LoRaSetOpMode(Stdby_mode);
        SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);  //�򿪷����ж�
        SX1276WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
        SX1276WriteBuffer(REG_LR_DIOMAPPING1, 0X00);
        SX1276WriteBuffer(REG_LR_DIOMAPPING2, 0x00);	
        SX1276LoRaSetOpMode(Receiver_mode);
    }
    else if ((RF_EX0_STATUS & 0x08) == 0x08) { //�������
        SX1276LoRaSetOpMode(Stdby_mode);
        SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);	//�򿪷����ж�
        SX1276WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
        SX1276WriteBuffer(REG_LR_DIOMAPPING1, 0X00);
        SX1276WriteBuffer(REG_LR_DIOMAPPING2, 0x00);	
        SX1276LoRaSetOpMode(Receiver_mode);    
    }
    else if ((RF_EX0_STATUS & 0x04) == 0x04) { //cad��� 
        if ((RF_EX0_STATUS & 0x01) == 0x01) {	 
        //��ʾCAD ��⵽��Ƶ�ź� ģ������˽���״̬����������
            SX1276LoRaSetOpMode(Stdby_mode);
            SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);	//�򿪷����ж�
            SX1276WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
            SX1276WriteBuffer(REG_LR_DIOMAPPING1, 0X02);
            SX1276WriteBuffer(REG_LR_DIOMAPPING2, 0x00);	
            SX1276LoRaSetOpMode(Receiver_mode);    
        } else {	
            SX1276LoRaSetOpMode(Stdby_mode);
            SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_SEELP_Value);	//�򿪷����ж�
            SX1276WriteBuffer(REG_LR_DIOMAPPING1, 0X00);
            SX1276WriteBuffer(REG_LR_DIOMAPPING2, 0X00);	
            SX1276LoRaSetOpMode(Sleep_mode);
        }
    } 
    else {
        SX1276LoRaSetOpMode(Stdby_mode);
        SX1276WriteBuffer(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);	//�򿪷����ж�
        SX1276WriteBuffer(REG_LR_HOPPERIOD, PACKET_MIAX_Value);
        SX1276WriteBuffer(REG_LR_DIOMAPPING1, 0X02);
        SX1276WriteBuffer(REG_LR_DIOMAPPING2, 0x00);	
        SX1276LoRaSetOpMode(Receiver_mode);
        
    }
    SX1276WriteBuffer(REG_LR_IRQFLAGS, 0xff);

    return SX1278_RLEN;
}

///////////////////////////////////////////////////////////////////////////////


void GPIO_Config(void)
{
    RF_REST_L;
    RF_CKL_L;
    RF_CE_H;
    RF_SDI_H;  			 //MOSI
    //SX1278_SDO = 1;  //MISO
}

void SX1276Reset(void)
{
    RF_REST_L;	
    mdelay(200);  //125.673ms
    RF_REST_H;
    mdelay(500);
}

void RF_SPI_MasterIO(unsigned char outtt)
{
    unsigned char i;
    for (i = 0; i < 8; i++) {   
        if (outtt & 0x80) {			/* check if MSB is high */
            RF_SDI_H;
        } else { 
            RF_SDI_L;						/* if not, set to low */
        }
        RF_CKL_H;						  /* toggle clock high */
        ndelay(100);
        outtt = (outtt << 1);	/* shift 1 place for next bit */
        RF_CKL_L;							/* toggle clock low */
        ndelay(100);
    }
}

unsigned char RF_SPI_READ_BYTE(void)
{	 
    unsigned char j;
    unsigned char i;
    j = 0;
    for (i = 0; i < 8; i++) {	 
        RF_CKL_H; 
        ndelay(100);
        j = (j << 1);		// shift 1 place to the left or shift in 0 //
        if(SX1278_SDO)	// check to see if bit is high //
            j = j | 0x01; 	// if high, make bit high //
                            // toggle clock high // 
        RF_CKL_L; 		    // toggle clock low //  
        ndelay(100);
    }
    return j;			    // toggle clock low //
}

void cmdSwitchEn(cmdEntype_t cmdcmd)
{
    switch (cmdcmd) {
        case enOpen: {
            RF_CE_L;
            break;
        }
        case enClose: {
            RF_CE_H;
            ndelay(100);//sct
            break;
        }
        default:
            break;
    }
}

void fqcRecvData(unsigned char *lpbuf, unsigned short len)// ���յ�RF����
{
    unsigned char i;
    unsigned char Irq_flag = 0;

    for (i = 0; i < 3; i++){
        printk("%x\n", lpbuf[i]);//test		
    }
#if 0
    for (i = 0; i < 3; i++) {
        FUN_RF_SENDPACKET(lpbuf,len); //��������
        Irq_flag=SX1276ReadBuffer(REG_LR_IRQFLAGS); 
        //while (Irq_flag&0x08 != 0x08)  //xx
        while ((Irq_flag&0x08) != 0x08) {
            mdelay(1);
            Irq_flag = SX1276ReadBuffer(REG_LR_IRQFLAGS); 
        }
    }
    //SX1278_RLEN = 0;
#endif
    //uartSendRecv();
    //Delaycc();
    //Delaycc();
    //P3 |=0xf0;

}

void main_init(void)
{   
    GPIO_Config();
    SX1276Reset();
    SX1276LORA_INT();
    //enableInterrupts();
    //SX1278_RLEN = 0;   
}

void ProcessSend(unsigned char *data, unsigned short len)
{
    unsigned char i;
    unsigned char Irq_flag = 0;

    FUN_RF_SENDPACKET(data, len); //��������
    Irq_flag = SX1276ReadBuffer(REG_LR_IRQFLAGS); 
    while ((Irq_flag&0x08) != 0x08) {
        mdelay(1);
        Irq_flag = SX1276ReadBuffer(REG_LR_IRQFLAGS); 
    }
}

unsigned short ProcessRecv(char *rxdata)
{
    unsigned short len;
    if (SX1276ReadBuffer(REG_LR_IRQFLAGS)) {      
        len = SX1278_Interupt(rxdata);
    }
}
//-----------------------------------------------------

/*
 * is_in  0-out,1-in
 */
static int spi_gpio_alloc(unsigned pin, const char *label, bool is_in)
{
    int value;

    value = gpio_request(pin, label);
    if (value == 0) {
        if (is_in) {
            value = gpio_direction_input(pin);
        } else {
            value = gpio_direction_output(pin, 0);
        }
    }
    return value;
}

unsigned char kbuf[256];
static ssize_t spi_gpio_write(struct file *file, const char __user *buf,size_t count, loff_t *ppos)
{
    int i;
//	printk("spi_gpio_write,count=%d\n",count);//test  
    copy_from_user(kbuf, buf, count); 
    ProcessSend(kbuf,count);	
    //spi_write_mes(kbuf,count);
    return count;
}

char rddata[256];
static ssize_t spi_gpio_read(struct file *file, char __user *buf,	size_t count, loff_t *ppos)
{
    
    char i;
    int ret;
    //spi_read_mes(rddata,count);
    /*
    for(i=0;i<count;i++)//test
    {
        printk("%x,",rddata[i]);
    }
    //spi_write_mes(rddata,7);//test
    */
    count = ProcessRecv(rddata);
    /*
    if(count>0)
    {
        printk("rx len=%d\n",count);
        for(i=0;i<count;i++)
        {
            printk("%x\n",rddata[i]);
        }
    }*/
    
    if (copy_to_user(buf, rddata, count)) {
        return -EFAULT;
    }
    return count;
}


static long spi_gpio_ioctl(struct file *filp, unsigned int cmd, char arg,char num)  
{  
    void __user *uarg = (void __user *) arg;
    //fpga_version_struct *fvp;
    int ret = 0;
    int tmp = 0;
    unsigned short wTem;
    char cdata;
    //printk("spi_gpio_ioctl\n");//TEST
    
    switch (cmd) {
        case CMD_RESET://        
            main_init();
            //printk("CMD_RESET\n");//TEST
            break;
        default:
            printk( "spi_gpio_ioctl:CMD ERROR\n");
            ret = -1;
            return -EINVAL;
    }
    
    if (ret < 0) {
        return ret;
    } else {	
        return 0;
    }  
}

static int spi_gpio_open(struct inode *inode, struct file *file)
{
    printk("spi_gpio_open\n");//test
    main_init();
}

void gpio_init(void)
{
    int i = 0;
    int ret = -1;
    /*
        ret = gpio_request(GPIO_TO_PIN(3, 14),led_dev_data[i].desc);
        if(ret < 0)
        {
            printk("failed to request GPIO %d{%s}, error %d\n",	led_dev_data[i].gpio,led_dev_data[i].desc, ret);
            return ret;
        }
    */
    ret = spi_gpio_alloc(SCK, spi_sck, 0);
    printk("sck ret=%d\n",ret);//test
    gpio_set_value(SCK, 1);
    
    ret = spi_gpio_alloc(MOSI, spi_mosi, 0);
    printk("mosi ret=%d\n",ret);//test
    gpio_set_value(MOSI, 1);
        
    ret = spi_gpio_alloc(CS, spi_cs, 0);
    printk("cs ret=%d\n",ret);//test
    gpio_set_value(CS, 1);
        
        
    ret = spi_gpio_alloc(MISO, spi_miso, 1);
    printk("MISO ret=%d\n",ret);//test
    
    ret = spi_gpio_alloc(RESET, spi_rst, 0);
    printk("RESET ret=%d\n",ret);//test
        
    gpio_set_value(RESET, 0);
    mdelay(200);
    gpio_set_value(RESET, 1);
    mdelay(200);

    return ;
}

static int spi_gpio_close(struct inode *inode, struct file *file)
{
    int i = 0;
    
    gpio_free(SCK);
    gpio_free(MOSI);
    gpio_free(MISO);
    gpio_free(CS);
    gpio_free(RESET);
    return 0;
}

static struct class* mclass; 
struct file_operations spi_gpio_ops =
{  
    .owner = THIS_MODULE,
    .unlocked_ioctl = spi_gpio_ioctl,
    .write = spi_gpio_write,
    .read = spi_gpio_read,
   // .open =spi_gpio_open,
   //.release	 	= 	spi_gpio_close,
};

static int __init spi_gpio_init(void)
{
    int ret;
    dev_t majoid;
    int i, cnt;//test
    unsigned short len, txlen;
    unsigned char rxda[64], txda[64];
    unsigned char opModePrev;//test
    
    ret = register_chrdev(majoid, DEVICE_NAME, &spi_gpio_ops);
    if (ret < 0) {
        printk(DEVICE_NAME " can't get major number\n");
        return ret;
    }	
    majoid = ret;
    printk(KERN_INFO "Device name:%s is registered,Device major = %d\n", DEVICE_NAME, majoid);

    mclass = class_create(THIS_MODULE, DEVICE_NAME);  
    if (IS_ERR(mclass)) {  
        printk(KERN_ALERT "fail to create class\n");  
        return -1;  
    }    
  
    device_create(mclass, NULL, MKDEV(majoid,0), NULL, "sx1278dev0"); 
    printk(KERN_ALERT "Creat sx1278 DEV NOD\n"); 
             
    gpio_init();
    
    //test-----
    main_init();
    mdelay(300);
    /*
    cnt=0;
    txda[0] = 0x5a;
    txda[1] = 0x0b;
    txda[2] = 0xfa;
    txlen=3;
    //for(i=0;i<60;i++)
    while(1)
    {
        if(cnt>100000)
        {
            ProcessSend(txda,txlen);//test
            cnt=0;
            printk("send\n");
        }
        cnt++;
    //printk("cnt=%d\n",cnt);
    //mdelay(3000);
    
    len=ProcessRecv(rxda);
    if(len>0)
    {
        printk("rx len=%d\n",len);
        for(i=0;i<len;i++)
        {
            printk("%x\n",rxda[i]);
        }
    }
    //mdelay(1000);
    //printk("send\n");
}
* */
    return 0;
}

static void spi_gpio_exit(void)  
{
    unregister_chrdev(DEVICE_MAJOR, DEVICE_NAME);
    gpio_free(SCK);
    gpio_free(MOSI);
    gpio_free(MISO);
    gpio_free(CS);
    gpio_free(RESET);
}
  
MODULE_LICENSE("GPL");  
//MODULE_LICENSE("MYGPL");  
MODULE_AUTHOR("SCT");  
  
module_init(spi_gpio_init);
module_exit(spi_gpio_exit);

///////////////////////////////////////////////////////////////////////////////
