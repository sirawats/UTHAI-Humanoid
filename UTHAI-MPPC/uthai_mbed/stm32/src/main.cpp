#include <mbed.h>

DigitalOut led1(PB_10);
DigitalOut led2(PB_4);
DigitalOut led3(PB_5);

DigitalIn button1(PB_3);
DigitalIn button2(PA_10);

int main()
{
    while (1)
    {
        if (button1.read() == 0 || button2.read() == 0)
        {
            led1.write(1);
            led2.write(0);
            led3.write(1);
        }
        else
        {
            led1.write(0);
            led2.write(1);
            led3.write(0);
        }
    }
}