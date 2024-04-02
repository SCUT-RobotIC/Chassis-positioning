#include <iostream>
#include <cstdio>

#include "Serial.hpp"


int main()
{
    Serial s1;
    

    std::string str = "/dev/ttyS0";   //串口号
    
    s1.OpenSerial(str, E_BaudRate::_115200, E_DataSize::_8, E_Parity::None, E_StopBit::_1);


    while (true)
    {
        unsigned char buff[] = "123456789\r\n" ;
        s1.Send(buff, sizeof(buff));
        
        unsigned char bf[100] = {0};
        int len = s1.Recv(bf, sizeof(bf));
        //std::cout << "len = " << len << std::endl;
        if(len > 0)
        {
            for(int i=0; i<len; i++)
            {
                printf("%.2X ", bf[i]);
            }
            std::cout << std::endl;
        }
        usleep(100*1000);
    }
    

    std::cout << "hello world" << std::endl;
    return 0;
}

//若uart串口的接受到的数据为“121.1755705 -120.599312 -0.125564”，如何将其解析成三个float类型的数据，分别为x_plot, y_plot, f
