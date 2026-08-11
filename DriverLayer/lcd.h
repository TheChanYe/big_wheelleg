#ifndef __LCD_H__
#define __LCD_H__
#include "main.h"
#include "my_spi.h"


#define USE_HORIZONTAL 3  //0竖屏正 1竖屏反 2横屏正 3横屏反
#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 280

#else
#define LCD_W 280
#define LCD_H 240
#endif

#define LCD_COLOR_WHITE        0xFFFF	//白色
#define LCD_COLOR_BLACK        0x0000	//黑色
#define LCD_COLOR_BLUE         0x00FF	//蓝色
#define LCD_COLOR_BRED         0XF81F
#define LCD_COLOR_GRED         0XFFE0
#define LCD_COLOR_GBLUE        0X07FF
#define LCD_COLOR_RED          0xF800	//红色
#define LCD_COLOR_MAGENTA      0xF81F	//品红色
#define LCD_COLOR_GREEN        0x07E0	//绿色
#define LCD_COLOR_CYAN         0x7FFF	//青色
#define LCD_COLOR_YELLOW       0xFFE0	//黄色
#define LCD_COLOR_BROWN        0XBC40	//棕色
#define LCD_COLOR_BRRED        0XFC07 	//棕红色
#define LCD_COLOR_GRAY         0X8430 	//灰色
#define LCD_COLOR_DARKBLUE     0X01CF 	//深蓝色
#define LCD_COLOR_LIGHTBLUE    0X7D7C 	//浅蓝色
#define LCD_COLOR_GRAYBLUE     0X5458 	//灰蓝色
#define LCD_COLOR_LIGHTGREEN   0X841F 	//浅绿色
#define LCD_COLOR_LGRAY        0XC618 	//浅灰色(PANNEL),窗体背景色
#define LCD_COLOR_LGRAYBLUE    0XA651 	//浅灰蓝色(中间层颜色)
#define LCD_COLOR_LBBLUE       0X2B12 	//浅棕蓝色(选择条目的反色)


#define LCD_BRUSH_DEFAULT_COLOR  LCD_COLOR_BROWN  /*画笔默认白色*/
#define LCD_BACK_DEFAULT_COLOR   LCD_COLOR_WHITE  /*背景默认黑色*/

typedef struct __C_LCD c_lcd;

typedef struct __C_LCD
{
    void* this;

    /************************************************* 
    * Function: set 
    * Description: 设置画笔
    * Input : <this>  tft18对象
    *         <brush> 画笔颜色
    *         <back>  背景颜色
    * Output: 无
    * Return: <E_OK>     操作成功
    *         <E_NULL>   空指针
    *         <E_ERROR>  操作失败
    * Others: 无
    * Demo  :
    *         tft18.set(&tft18,TFT18_COLOR_WHITE,TFT18_COLOR_BLACK);
    *         if(E_OK != ret)
    *         {
    *             log_error("Tft18 set failed.");
    *         }
    *************************************************/      
    int (*set)(const c_lcd* this,u16 brush,u16 back);
    
    /************************************************* 
    * Function: clear 
    * Description: 清除屏幕
    * Input : <this>  tft18对象
    * Output: 无
    * Return: <E_OK>     操作成功
    *         <E_NULL>   空指针
    *         <E_ERROR>  操作失败
    * Others: 无
    * Demo  :
    *         tft18.clear(&tft18);
    *         if(E_OK != ret)
    *         {
    *             log_error("Tft18 clear failed.");
    *         }
    *************************************************/        
    int (*clear)(const c_lcd* this,u16 color);
    
    /************************************************* 
    * Function: rectangle 
    * Description: 绘制矩形
    * Input : <this>   tft18对象
    *         <x>      矩形左上角x坐标
    *         <y>      矩形右上角y坐标
    *         <weight> 矩形宽
    *         <height> 矩形高
    * Output: 无
    * Return: <E_OK>     操作成功
    *         <E_NULL>   空指针
    *         <E_ERROR>  操作失败
    * Others: 无
    * Demo  :
    *         tft18.clear(&tft18);
    *         if(E_OK != ret)
    *         {
    *             log_error("Tft18 clear failed.");
    *         }
    *************************************************/        
    int (*rectangle)(const c_lcd* this,u16 x1,u16 y1,u16 x2,u16 y2,u16 color);
    int (*lcd_char)	(const c_lcd* this,u16 x,u16 y,char ch,u16 fc,u16 bc,u8 sizey,u8 mode);		
		int (*point)(const c_lcd* this,u16 x,u16 y,u16 color);
		int (*line)(const c_lcd* this,u16 x1,u16 y1,u16 x2,u16 y2,u16 color);
		int (*fill)(const c_lcd* this,u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);
		int (*circle)(const c_lcd* this,u16 x0,u16 y0,u16 r,u16 color);
    int (*chinese)(const c_lcd* this,u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);
    /************************************************* 
    * Function: str 
    * Description: 显示字符串
    * Input : <this>  tft18对象
    *         <x>     字符串的x坐标  
    *         <y>     字符串的y坐标  
    *         <format>格式化字符串
    *         <...>   可变参
    * Output: 无
    * Return: <E_OK>     操作成功
    *         <E_NULL>   空指针
    *         <E_PARAM>  参数错误
    *         <E_ERROR>  操作失败
    * Others: 如果字符超出屏幕能够显示的区域，过多的字符会被丢弃
    * Demo  :
    *         int a = 9999999;
    *
    *         tft18.str(&tft18,0,0 ,12,"%d",a);
    *         if(E_OK != ret)
    *         {
    *             log_error("Tft18 display failed.");
    *         }
    *************************************************/     
    int (*str)(const c_lcd* this,u16 x,u16 y,u8 size,char* format,...);

    /************************************************* 
    * Function: pic 
    * Description: 显示图片
    * Input : <this>  tft18对象
    *         <x>     图片左上角的x坐标  
    *         <y>     图片左上角的y坐标  
    *         <length>图片宽
    *         <width> 图片高
    *         <data>  图片数据 RGB565
    * Output: 无
    * Return: <E_OK>     操作成功
    *         <E_NULL>   空指针
    *         <E_PARAM>  参数错误
    *         <E_ERROR>  操作失败
    * Others: 禁止超出显示区域的显示行为
    * Demo  :
    *         u16 pic[250] = {xxx};
    *
    *         tft18.pic(&tft18,0,0 ,50,50,pic);
    *         if(E_OK != ret)
    *         {
    *             log_error("Tft18 display failed.");
    *         }
    *************************************************/   
    int (*pic)(const c_lcd* this,u16 x,u16 y,u16 length,u16 width,u16 pic[]);
}c_lcd;

/************************************************* 
* Function: tft18_create 
* Description: 创建一个tft18对象
* Input : <spi_channal>  tft18所在的spi编号
*         <type>         tft18的屏幕类型     
*                        (TFT18_DIR_V_POSITIVE)  竖屏正
*                        (TFT18_DIR_V_REVERSE )  竖屏反
*                        (TFT18_DIR_H_POSITIVE)  横屏正
*                        (TFT18_DIR_H_REVERSE )  横屏反
*         <re_gpio>      复位脚GPIO分组
*         <re_pin>       复位脚PIN
*         <dc_gpio>      数据脚GPIO分组
*         <dc_pin>       数据脚PIN
*         <cs_gpio>      片选脚GPIO分组
*         <cs_pin>       片选脚PIN
* Output: 无
* Return: 新的对象拷贝 返回值中，this指针为空 表示创建失败
* Others: 无
* Demo  :
*         c_tft18 tft18 = {0};
*         
*	      tft18 = tft18_create(MY_SPI_1,TFT18_DIR_V_REVERSE,GPIOA,GPIO_PIN_1,GPIOA,GPIO_PIN_2,GPIOA,GPIO_PIN_3);
*         if(NULL == tft18.this)
*         {
*             log_error("tft18 creat failed."); 
*         }
*************************************************/
c_lcd lcd_create(u8 spi_channal,gpio_type* re_gpio,uint32_t re_pin,
																gpio_type* dc_gpio,uint32_t dc_pin,
																gpio_type* cs_gpio,uint32_t cs_pin);


#endif

