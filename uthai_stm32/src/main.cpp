#include <mbed.h>
// #include <Dynamixel.h>
// #include <Mx28.h>

// Dynamixel dynamixel(PA_9, PA_10, 1000000, PB_0);
Serial pc(SERIAL_TX, SERIAL_RX);
Serial dx(PA_9, PA_10);
DigitalOut led_status(LED1);
DigitalOut dir(PB_0);
Timer timer;
uint8_t ID = 0x01;

void moveServo(uint8_t id, uint16_t pos)
{
    uint8_t L_Position = (uint8_t)(pos & 0xFF);
    uint8_t H_Position = (uint8_t)(pos >> 8);

    dir.write(1);
    wait_ms(1);
    dx.putc(0xFF);
    dx.putc(0xFF);
    dx.putc(id);
    dx.putc(0x05);
    dx.putc(0x03);
    dx.putc(0x1E);
    dx.putc(L_Position);
    dx.putc(H_Position);
    dx.putc(~(id + 0x05 + 0x03 + 0x1E + L_Position + H_Position));
    wait_ms(1);
    dir.write(0);
}
void requestTemperature(uint8_t id)
{
    dir.write(1);
    wait_ms(1);
    dx.putc(0xFF);
    dx.putc(0xFF);
    dx.putc(id);
    dx.putc(0x04);
    dx.putc(0x02);
    dx.putc(0x24);
    dx.putc(0x02);
    dx.putc(~(id + 0x04 + 0x02 + 0x24 + 0x02));
    wait_ms(1);
    dir.write(0);
    dx.attach(&rex);
    timer.start();
}
uint8_t buf[256] = {0};
uint8_t i = 0;
void rex()
{
    if (i == 7)
    {
        timer.stop();
        dx.
    }
    buf[i] = dx.getc();
    i++;
}

void ping(uint8_t id)
{
    dir.write(1);
    wait_ms(1);
    dx.putc(0xFF);
    dx.putc(0xFF);
    dx.putc(id);
    dx.putc(0x02);
    dx.putc(0x01);
    dx.putc(~(id + 0x02 + 0x01));
    wait_ms(1);
    dir.write(0);
}

void readmodel(uint8_t id)
{
    dir.write(1);
    wait_ms(1);
    dx.putc(0xFF);
    dx.putc(0xFF);
    dx.putc(id);
    dx.putc(0x04);
    dx.putc(0x02);
    dx.putc(0x00);
    dx.putc(0x03);
    dx.putc(~(id + 0x09));
    wait_ms(1);
    dir.write(0);
}
int main()
{
    pc.printf("Hello World\n");
    dx.baud(57143);
    dir.write(0);
    wait_ms(1);

    while (1)
    {
        led_status = !led_status;
        // pc.printf("Hello World\n");

        // moveServo(1, 200); // Move to position 200
        requestTemperature(1);
        // wait_ms(1000);
        // moveServo(1, 500); // Move to position 500
        // ping(1);
        // readmodel(1);
        wait_ms(1000);
        pc.printf("\ntime %d\n", timer.read_us());
        timer.reset();
        for (int x = 0; x < 15; x++)
        {
            pc.printf("%x ", buf[x]);
        }
        i = 0;
        pc.printf("\n");
        // pc.printf("temp|%d\n", dynamixel.getTemperature(1));
        // dynamixel.getTemperature(1);
        // wait_ms(500);
        // dynamixel.setPosition(1, 0, 0);
        // wait(1);
        // pc.printf("Goal is %d\n", dynamixel.getPosition(1));
        // // for(int i=0;i<10;i++)
        // // {
        // //     pc.printf("0x%X,",dynamixel.)
        // // }
        // dynamixel.setPosition(1, 2048, 0);
        // wait(1);
        // pc.printf("Goal is %d\n", dynamixel.getPosition(1));
        // // getPos(1);
        // dynamixel.setPosition(1, 4095, 0);
        // // setPos(1, 4095);
        // wait(1);
        // pc.printf("Goal is %d\n", dynamixel.getPosition(1));
        // dynamixel.setPosition(1, 2048, 0);
        // wait(1);
        // pc.printf("Goal is %d\n", dynamixel.getPosition(1));
        // // getPos(1);
        // dynamixel.GetPosition(1);111/
        // dynamixel.setPosition(1, 4095, 0);
        // wait(1.5);
        // pc.printf("Goal is %d\n", dynamixel.getPosition(1));
    }
}
