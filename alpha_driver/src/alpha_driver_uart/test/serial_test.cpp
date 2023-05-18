//c++
#include <queue>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>
#include <string>
#include <unistd.h>
#include <iostream>
// ros
#include <ros/ros.h>
#include <serial/serial.h>
// ros msg type header
#include <std_msgs/String.h>

class SerialTest
{
public:
  SerialTest()
  {
    sub = nh.subscribe<std_msgs::String> ("test_received", 100, &SerialTest::callback,this);

    pub = nh.advertise<std_msgs::String>("test_publish",100);

    // init serial

    // ser = new serial::Serial("/dev/ttySC3", 115200, serial::Timeout::simpleTimeout(1000));
    ser = std::make_shared<serial::Serial>(
        "/dev/ttySC3", 115200, serial::Timeout::simpleTimeout(1000)
    ); 
    
    ser->flushInput();

    if(ser->isOpen()) {
        printf("serial is open now\n");
    }
    else {
        printf("serial is not open\n");
    }

  }

  ~SerialTest()
  {
      if(ser->isOpen()) {
          ser->close();
          printf("serial is close now\n");
      }
  }

  void callback(const std_msgs::String::ConstPtr& input);

  void receive();

  void send();

private:
  ros::NodeHandle nh;

  ros::Subscriber sub;
  ros::Publisher pub;

  // serial::Serial *ser;
  std::shared_ptr<serial::Serial> ser;

};

void SerialTest::callback(const std_msgs::String::ConstPtr& input)
{
    printf("callback\n");
}


void SerialTest::receive()
{

    std::string data;
    std::size_t sz_t = 200;
    std::string eol;
    eol.append(1,0xd);
    eol.append(1,0xa);
    
    ser->flushInput();

    while(1)
    {
        if(ser->available()){
            data.clear();
            // data = ser->readline(sz_t,eol);
            data = ser->readline(65536,eol);
            printf("%s",data.c_str());
        }

        // running 200 hz
        std::chrono::milliseconds dura(1);
        std::this_thread::sleep_for(dura);
    }


}

void SerialTest::send() {

  std::string str;
  size_t bytes_wrote;

  str = "$1,abcdefghijklmnopqrstuvwxyz\r\n";
  bytes_wrote = ser->write(str);
  std::cout<<"witre: "<<bytes_wrote <<std::endl;
  usleep(1000);

  str = "$2,abcdefghijklmnopqrstuvwxyz\r\n";
  bytes_wrote = ser->write(str);
  std::cout<<"witre: "<<bytes_wrote <<std::endl;
  usleep(1000);

  str = "$3,abcdefghijklmnopqrstuvwxyz\r\n";
  bytes_wrote = ser->write(str);
  std::cout<<"witre: "<<bytes_wrote <<std::endl;
  usleep(1000);

  str = "$4,abcdefghijklmnopqrstuvwxyz\r\n";
  bytes_wrote = ser->write(str);
  std::cout<<"witre: "<<bytes_wrote <<std::endl;
  usleep(1000);

  str = "$5,abcdefghijklmnopqrstuvwxyz\r\n";
  bytes_wrote = ser->write(str);
  std::cout<<"witre: "<<bytes_wrote <<std::endl;
  usleep(1000);

  str = "$6,abcdefghijklmnopqrstuvwxyz\r\n";
  bytes_wrote = ser->write(str);
  std::cout<<"witre: "<<bytes_wrote <<std::endl;
  usleep(1000);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Thread_Example_Node");

  SerialTest  example;

  std::thread mainThread{&SerialTest::receive, &example};
  // mainThread.join();

  ros::Rate loop_rate(1);

  while (ros::ok()) {
    example.send();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}