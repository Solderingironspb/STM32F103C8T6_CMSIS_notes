#include "main.h"
#include "GMG12864_lib.h"


/*----------------------------GMG12864----------------------------------------*/
extern char tx_buffer[128]; //Буфер для отправки текста на дисплей
extern uint8_t Frame_buffer[1024]; //Буфер кадра
extern uint8_t GMG12864_width; //Ширина дисплея в пикселях
extern uint8_t GMG12864_height; //Высота дисплея в пикселях

//Для работы отрисовки графика:
extern uint8_t cnt; //счетчик накопления значений в окне графика
extern const uint8_t size_array; //размер массива. В нашем случае ширина 100 точек(График 100*50 пикселей)
extern uint8_t arr[100]; //значения на графике. Заполняются в определенный момент времени(каждый шаг сдвига графика влево)
extern bool array_is_full; //значения заполнили массив, можно сдвигать график влево
/*----------------------------GMG12864----------------------------------------*/

uint16_t Counter = 0;
uint8_t Value_for_plot = 0;

int main(void)
{
    CMSIS_Debug_init();
    CMSIS_RCC_SystemClock_72MHz();
    CMSIS_SysTick_Timer_init();
    CMSIS_SPI1_init();

    
    /*----------------------------------------------------------Настройка ножек----------------------------------------------------------*/
    //NSS - PA0 
    //RST - PA1
    //DC  - PA2
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); //Включение тактирования порта A
    
    //Настройка NSS
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE0, 0b10 << GPIO_CRL_MODE0_Pos); //Настройка GPIOA Pin 0 на выход со максимальной скоростью в 50 MHz
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF0, 0b00 << GPIO_CRL_CNF0_Pos); //Настройка GPIOA Pin 0 на выход в режиме Push-Pull
    
    //Настройка RST
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE1, 0b10 << GPIO_CRL_MODE1_Pos); //Настройка GPIOA Pin 1 на выход со максимальной скоростью в 50 MHz
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF1, 0b00 << GPIO_CRL_CNF1_Pos); //Настройка GPIOA Pin 1 на выход в режиме Push-Pull
    
    //Настройка DC
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE2, 0b10 << GPIO_CRL_MODE2_Pos); //Настройка GPIOA Pin 2 на выход со максимальной скоростью в 50 MHz
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF2, 0b00 << GPIO_CRL_CNF2_Pos); //Настройка GPIOA Pin 2 на выход в режиме Push-Pull
    
    /*----------------------------------------------------------Настройка ножек----------------------------------------------------------*/
  
    //Delay_ms(1000);
    GMG12864_Init();
    //GMG12864_logo_demonstration();
    
    /*пример фигур(да, говнокод)*/
      //Фигуры на белом
    for (uint8_t i = 0; i < 8; i++) {
        for (uint8_t i = 0; i < 10; i++) {
            GMG12864_Clean_Frame_buffer();
            GMG12864_Draw_line(0, 0, 127, 63, 1);
            GMG12864_Draw_line(0, 63, 127, 0, 1);
            GMG12864_Draw_rectangle(80, 5, 30, 20, 1);
            GMG12864_Draw_rectangle_filled(i, i, 50, 10, 1);
            GMG12864_Draw_circle(70 + i, 50, 10, 1);
            GMG12864_Draw_circle_filled(50, 25, 10 + i, 1);
            GMG12864_Draw_triangle(0, 63, 50 + i, 0, 127 - i, 20, 1);
            GMG12864_Draw_triangle_filled(40, 10 + i, 75, 20, 100, 40, 1);
            GMG12864_Update();
        }
        for (uint8_t i = 10; i > 0; i--) {
            GMG12864_Clean_Frame_buffer();
            GMG12864_Draw_line(0, 0, 127, 63, 1);
            GMG12864_Draw_line(0, 63, 127, 0, 1);
            GMG12864_Draw_rectangle(80, 5, 30, 20, 1);
            GMG12864_Draw_rectangle_filled(i, i, 50, 10, 1);
            GMG12864_Draw_circle(70 + i, 50, 10, 1);
            GMG12864_Draw_circle_filled(50, 25, 10 + i, 1);
            GMG12864_Draw_triangle(0, 63, 50 + i, 0, 127 - i, 20, 1);
            GMG12864_Draw_triangle_filled(40, 10 + i, 75, 20, 100, 40, 1);
            GMG12864_Update();
        }
        for (uint8_t i = 0; i < 20; i++) {
            GMG12864_Clean_Frame_buffer();
            GMG12864_Draw_line(0, 0, 127, 63, 1);
            GMG12864_Draw_line(0, 63, 127, 0, 1);
            GMG12864_Draw_rectangle(80, 5, 30, 20, 1);
            GMG12864_Draw_rectangle_filled(i, i, 50, 10, 1);
            GMG12864_Draw_circle(70 + i, 50, 10, 1);
            GMG12864_Draw_circle_filled(50, 25, 10 + i, 1);
            GMG12864_Draw_triangle(0, 63, 50 + i, 0, 127 - i, 20, 1);
            GMG12864_Draw_triangle_filled(40, 10 + i, 75, 20, 100, 40, 1);
            GMG12864_Update();
        }
        for (uint8_t i = 20; i > 0; i--) {
            GMG12864_Clean_Frame_buffer();
            GMG12864_Draw_line(0, 0, 127, 63, 1);
            GMG12864_Draw_line(0, 63, 127, 0, 1);
            GMG12864_Draw_rectangle(80, 5, 30, 20, 1);
            GMG12864_Draw_rectangle_filled(i, i, 50, 10, 1);
            GMG12864_Draw_circle(70 + i, 50, 10, 1);
            GMG12864_Draw_circle_filled(50, 25, 10 + i, 1);
            GMG12864_Draw_triangle(0, 63, 50 + i, 0, 127 - i, 20, 1);
            GMG12864_Draw_triangle_filled(40, 10 + i, 75, 20, 100, 40, 1);
            GMG12864_Update();
        }
    }
    //Фигуры на черном
    for (uint8_t i = 0; i < 8; i++) {
        for (uint8_t i = 0; i < 10; i++) {
            memset(Frame_buffer, 0xFF, sizeof(Frame_buffer));
            GMG12864_Draw_line(0, 0, 127, 63, 0);
            GMG12864_Draw_line(0, 63, 127, 0, 0);
            GMG12864_Draw_rectangle(80, 5, 30, 20, 0);
            GMG12864_Draw_rectangle_filled(i, i, 50, 10, 0);
            GMG12864_Draw_circle(70 + i, 50, 10, 0);
            GMG12864_Draw_circle_filled(50, 25, 10 + i, 0);
            GMG12864_Draw_triangle(0, 63, 50 + i, 0, 127 - i, 20, 0);
            GMG12864_Draw_triangle_filled(40, 10 + i, 75, 20, 100, 40, 0);
            GMG12864_Update();
        }
        for (uint8_t i = 10; i > 0; i--) {
            memset(Frame_buffer, 0xFF, sizeof(Frame_buffer));
            GMG12864_Draw_line(0, 0, 127, 63, 0);
            GMG12864_Draw_line(0, 63, 127, 0, 0);
            GMG12864_Draw_rectangle(80, 5, 30, 20, 0);
            GMG12864_Draw_rectangle_filled(i, i, 50, 10, 0);
            GMG12864_Draw_circle(70 + i, 50, 10, 0);
            GMG12864_Draw_circle_filled(50, 25, 10 + i, 0);
            GMG12864_Draw_triangle(0, 63, 50 + i, 0, 127 - i, 20, 0);
            GMG12864_Draw_triangle_filled(40, 10 + i, 75, 20, 100, 40, 0);
            GMG12864_Update();
        }
        for (uint8_t i = 0; i < 20; i++) {
            memset(Frame_buffer, 0xFF, sizeof(Frame_buffer));
            GMG12864_Draw_line(0, 0, 127, 63, 0);
            GMG12864_Draw_line(0, 63, 127, 0, 0);
            GMG12864_Draw_rectangle(80, 5, 30, 20, 0);
            GMG12864_Draw_rectangle_filled(i, i, 50, 10, 0);
            GMG12864_Draw_circle(70 + i, 50, 10, 0);
            GMG12864_Draw_circle_filled(50, 25, 10 + i, 0);
            GMG12864_Draw_triangle(0, 63, 50 + i, 0, 127 - i, 20, 0);
            GMG12864_Draw_triangle_filled(40, 10 + i, 75, 20, 100, 40, 0);
            GMG12864_Update();
        }
        for (uint8_t i = 20; i > 0; i--) {
            memset(Frame_buffer, 0xFF, sizeof(Frame_buffer));
            GMG12864_Draw_line(0, 0, 127, 63, 0);
            GMG12864_Draw_line(0, 63, 127, 0, 0);
            GMG12864_Draw_rectangle(80, 5, 30, 20, 0);
            GMG12864_Draw_rectangle_filled(i, i, 50, 10, 0);
            GMG12864_Draw_circle(70 + i, 50, 10, 0);
            GMG12864_Draw_circle_filled(50, 25, 10 + i, 0);
            GMG12864_Draw_triangle(0, 63, 50 + i, 0, 127 - i, 20, 0);
            GMG12864_Draw_triangle_filled(40, 10 + i, 75, 20, 100, 40, 0);
            GMG12864_Update();
        }
    }    
    
    /*пример фигур(да, говнокод)*/
    
    
    
    
    while (1)
    {
        
        Counter++;
        if (Counter == 5000) {
            Counter = 0;
        }
        Value_for_plot = GMG12864_Value_for_Plot(0, 5000, Counter);
        GMG12864_Fill_the_array_Plot(&cnt, arr, size_array, Value_for_plot);
        GMG12864_Generate_a_Graph(&cnt, arr, size_array, 0, 5000, 1, 0, 1);
        sprintf(tx_buffer, "%d", Counter);
        GMG12864_Decode_UTF8(0, 55, 1, 0, tx_buffer);
        GMG12864_Update();
        
  
        
    }
}
