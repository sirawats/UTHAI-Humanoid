#include <mbed.h>
#include <Dynamixel.h>
// #include <Mx28.h>

Dynamixel dynamixel(PA_9, PA_10, 1000000, PB_0);
Serial pc(SERIAL_TX, SERIAL_RX);
DigitalOut led_status(LED1);

int main()
{
    pc.printf("Hello World\n");
    // pc.printf("%d", dynamixel.reset(1));
    while (1)
    {
        led_status = !led_status;

        // wait(0.5);
        dynamixel.setPosition(1, 0, 0);
        wait(1);
        pc.printf("Goal is %d\n", dynamixel.getPosition(1));
        // for(int i=0;i<10;i++)
        // {
        //     pc.printf("0x%X,",dynamixel.)
        // }
        dynamixel.setPosition(1, 2048, 0);
        wait(1);
        pc.printf("Goal is %d\n", dynamixel.getPosition(1));
        // getPos(1);
        dynamixel.setPosition(1, 4095, 0);
        // setPos(1, 4095);
        wait(1);
        pc.printf("Goal is %d\n", dynamixel.getPosition(1));
        dynamixel.setPosition(1, 2048, 0);
        wait(1);
        pc.printf("Goal is %d\n", dynamixel.getPosition(1));
        // getPos(1);
        // dynamixel.GetPosition(1);111/
        // dynamixel.setPosition(1, 4095, 0);
        // wait(1.5);
        // pc.printf("Goal is %d\n", dynamixel.getPosition(1));
    }
}
