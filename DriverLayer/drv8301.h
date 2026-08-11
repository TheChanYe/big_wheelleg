#ifndef __DRV8301_H__
#define __DRV8301_H__
#include "main.h"
#include "my_spi.h"

/*读写命令*/
#define	READ_CMD	0X01<<15
#define	WRITE_CMD	0X00<<15
/*读状态寄存器地址*/
#define READ_REGISTER1	0X00<<11
/* D10  /   D9   /   D8   	/ D7  / D6 /    D5   /   D4   	/    D3   /    D2   /  D1    /    D0    */
/*FAULT /GVDD_UV /PVDD_UV 	/OTSD /OTW /FETHA_OC /FETLA_OC	/FETHB_OC /FETLB_OC /FETHC_OC/ FETLC_OC */
/*故障  /GVDD欠压/PVDD欠压	/OTSD /过温/FETHA_OC /FETLA_OC	/FETHB_OC /FETLB_OC /FETHC_OC/ FETLC_OC */

/*读状态寄存器地址*/
#define READ_REGISTER2	0X01<<11
/*	 D7    /    D3   /    D2    /    D1   /    D0     */
/* GVDD_OV /Device ID/Device ID /Device ID/ Device ID */
/*GVDD过压*/

/*写状态寄存器地址*/
#define	WRITE_REGISTER1	0X02<<11
#define	WRITE_REGISTER2	0X03<<11

/*栅极驱动器峰值电流 GATE_CURRENT*/
#define	LIMITING_1_7A		0X00<<0//栅极驱动器的峰值电流为1.7 A 
#define LIMITING_0_7A		0X01<<0//栅极驱动器的峰值电流为0.7 A
#define LIMITING_0_25A	0X02<<0//栅极驱动器的峰值电流为0.25 A
/*栅极重置状态 GATE_RESET*/
#define GATE_RESET		0x00<<2//正常模式
#define GATE_SET	0x01<<2//重置闸门驱动器锁定故障（恢复为0）
/*PWM模式 PWM_MODE*/
#define PWM_6	0X00<<3//6个PWM输入
#define PWM_3	0X01<<3//3个PWM输入
/*过流保护模式	OCP_MODE*/
#define	LIMITING	0X00<<4	//限流
#define	SHUT_DOWN	0X01<<4	//关闭芯片
#define REPORT		0X02<<4	//仅报告
#define DISABLED	0X03<<4	//禁用OC
/*过电流阈值大小设置*/
#define	CURRENT_0	0X00<<6	//限流大小 0.060A
#define	CURRENT_1	0X01<<6	//限流大小 0.068A
#define	CURRENT_2	0X02<<6	//限流大小 0.076A
#define	CURRENT_3	0X03<<6	//限流大小 0.086A
#define	CURRENT_4	0X04<<6	//限流大小 0.097A
#define	CURRENT_5	0X05<<6	//限流大小 0.109A
#define	CURRENT_6	0X06<<6	//限流大小 0.123A
#define	CURRENT_7	0X07<<6	//限流大小 0.138A
#define	CURRENT_8	0X08<<6	//限流大小 0.155A
#define	CURRENT_9	0X09<<6	//限流大小 0.175A
#define	CURRENT_10	0X0A<<6	//限流大小 0.197A
#define	CURRENT_11	0X0B<<6	//限流大小 0.222A
#define	CURRENT_12	0X0C<<6	//限流大小 0.250A
#define	CURRENT_13	0X0D<<6	//限流大小 0.282A
#define	CURRENT_14	0X0E<<6	//限流大小 0.317A
#define	CURRENT_15	0X0F<<6	//限流大小 0.358A
#define	CURRENT_16	0X10<<6	//限流大小 0.403A
#define	CURRENT_17	0X11<<6	//限流大小 0.454A
#define	CURRENT_18	0X12<<6	//限流大小 0.511A
#define	CURRENT_19	0X13<<6	//限流大小 0.576A
#define	CURRENT_20	0X14<<6	//限流大小 0.648A
#define	CURRENT_21	0X15<<6	//限流大小 0.730A
#define	CURRENT_22	0X16<<6	//限流大小 0.822A
#define	CURRENT_23	0X17<<6	//限流大小 0.926A
#define	CURRENT_24	0X18<<6	//限流大小 1.043A
#define	CURRENT_25	0X19<<6	//限流大小 1.175A
#define	CURRENT_26	0X1A<<6	//限流大小 1.324A
#define	CURRENT_27	0X1B<<6	//限流大小 1.491A
#define	CURRENT_28	0X1C<<6	//限流大小 1.679A
#define	CURRENT_29	0X1D<<6	//限流大小 1.892A
#define	CURRENT_30	0X1E<<6	//限流大小 2.131A
#define	CURRENT_31	0X1F<<6	//限流大小 2.400A
/*过温过流报告模式*/
#define	REPORT_NOCTW	0X00//报告过温过流
#define REPORT_OT			0X01//仅报告过温
#define REPORT_OC			0X02//仅报告过流
#define REPORT_RES		0X03//仅报告过流(预留)
/*电流放大器增益*/
#define AMPLIFIER_10	0X00<<2//并联放大器的增益：10 V/V
#define AMPLTFIER_20	0X01<<2//并联放大器的增益：20 V/V
#define AMPLTFIER_40	0X02<<2//并联放大器的增益：40 V/V
#define AMPLTFIER_80	0X03<<2//并联放大器的增益：80 V/V
/*分流放大器矫正方式*/
#define	AMPLIFIER1_IN	    0X00<<4		//分流放大器1通过输入引脚连接到负载
#define	AMPLIFIER1_EXTERN	0X01<<4		//分流放大器1短路输入引脚和断开负载的外部校准
#define	AMPLIFIER2_IN	    0X00<<5		//分流放大器2通过输入引脚连接到负载
#define	AMPLIFIER2_EXTERN	0X01<<5		//分流放大器2短路输入引脚和断开负载的外部校准
/*过流恢复控制*/
#define	CYCLE_MODE	0X00<<6	//关闭到下一个PWM周期
#define	TIME_MODE	0X01<<6	//关闭64us


typedef struct __C_DRV8301 c_drv8301;

typedef struct __C_DRV8301
{
    void* this;
		int (*close_drive)(const c_drv8301* this);
		int (*open_drive)(const c_drv8301* this);
		int (*write_data)(const c_drv8301* this,u8 datasize,const u16* send_data,u16* read_data);
		int (*get_status_register)(const c_drv8301* this,u16 *register1,u16 *register2);
}c_drv8301;
c_drv8301 drv8301_create(u8 spi_channal,gpio_type* cs_gpio,uint32_t cs_pin,
																				gpio_type* en_gate_gpio,uint32_t en_gate_pin);


#endif

