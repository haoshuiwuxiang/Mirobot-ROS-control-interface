#include "ros/ros.h"
#include "std_msgs/String.h"
#include <serial/serial.h>
#include <sstream>

#include "MirobotType.h"
////////////home cmd////////////////////////////////////////////////////////////////////////
#include "mirobot/SetHomeCmd.h"
serial::Serial _serial;	

int CheckOk(float time_s) //检测串口是否由OK返回
{
    ros::Duration(time_s).sleep();//等串口数据回来

    if(_serial.available())
    { 
        std_msgs::String result; 
        std::stringstream ss;
        result.data = _serial.read(_serial.available()); 
        ROS_INFO("Reading num: %d",result.data.size());
        ROS_INFO("Complete information:%s",result.data.c_str());
        if(result.data.npos == result.data.find("ok"))//没有找到ok
            return GetOkFail;
            else
            return GetPoseSuccess;
    }
    return GetOkFail;
}

int ConnectMirobot(char *portName, uint32_t baudrate)
{
    try//尝试连接机械臂的串口
	{
		_serial.setPort(portName);//"/dev/ttyUSB0"
		_serial.setBaudrate(baudrate);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		//_serial.write("Port has been open successfully\r\n");
		//ROS_INFO_STREAM("Port has been open successfully");
        return MirobotConnect_Success;
	}
	catch (serial::IOException& e)
	{
		//ROS_ERROR_STREAM("Unable to open port");
		return MirobotConnect_Error;
	}
}

int DisconnectMirobot(void)
{
    if(_serial.isOpen())
    {
    _serial.close();
    ROS_INFO("Disconnect Mirobot!");
    }
}

int SendHomeCmd(void)
{
    _serial.write("$H\n");
    return (CheckOk(0.1));
}

bool SetHomeCmdService(mirobot::SetHomeCmd::Request &req, mirobot::SetHomeCmd::Response &res)
{
    res.result = SendHomeCmd();
    return true;
}

void InitHOMEServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/MirobotServer/SetHomeCmd", SetHomeCmdService);
    serverVec.push_back(server);
}

//////////////get pose////////////////////////////////////////////////////////////////////
#include "mirobot/GetPoseCmd.h"

Pose pose;

int GetPose(Pose *pose)
{
    _serial.write("?\n");
    ros::Duration(0.1).sleep();
    std::stringstream ss;
    if(_serial.available()){ 
            //ROS_INFO("Reading from serial port:\n"); 
            std_msgs::String result; 
            result.data = _serial.read(_serial.available()); 
            //result.data[1];
            ROS_INFO("Reading num: %d",result.data.size());
            if(result.data[result.data.size()-1] == '\n')//check the last char ,if it is \n,the command is complate
                {
                    
                    std_msgs::String info;
                    ss << result;
                    info.data = ss.str();
                    ROS_INFO("Complete information:%s",info.data.c_str());
                    ss.str("");//use this method to clear ss!
                    
                    //read_pub.publish(info); 
                    int rem1,rem2;
                    rem1 = info.data.find('<') + 1; 
                                switch (info.data[rem1])
                                {
                                case 'I':
                                    pose->state = Idle;
                                    break;
                                case 'H':
                                    pose->state = Home;
                                    break;
                                case 'A':
                                    pose->state = Alarm;
                                    //ROS_INFO("Alarm!!!!");
                                    break;
                                default:
                                    pose->state = Unknow;
                                    break;
                                }
                    rem2 = info.data.find("(ABCDXYZ)",rem1) + 10; //第一个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第一个浮点数的最后一个数字位置
                               
                    std_msgs::String temp1;
                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->jointAngle[3] = atof(temp1.data.c_str());//joint A

                    rem2 = rem1 + 2;//第二个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第二个浮点数的最后一个数字位置
                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->jointAngle[4] = atof(temp1.data.c_str());//joint B

                    rem2 = rem1 + 2;//第3个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第3个浮点数的最后一个数字位置
                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->jointAngle[5] = atof(temp1.data.c_str());//joint C

                    rem2 = rem1 + 2;//第4个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第4个浮点数的最后一个数字位置
                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->jointAngle[6] = atof(temp1.data.c_str());//joint D

                    rem2 = rem1 + 2;//第5个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第5个浮点数的最后一个数字位置
                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->jointAngle[0] = atof(temp1.data.c_str());//joint X

                    rem2 = rem1 + 2;//第6个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第5个浮点数的最后一个数字位置
                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->jointAngle[1] = atof(temp1.data.c_str());//joint Y

                    rem2 = rem1 + 2;//第7个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第7个浮点数的最后一个数字位置
                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->jointAngle[2] = atof(temp1.data.c_str());//joint Z

                    //第二组数值，位姿
                    rem2 = info.data.find("(XYZ RxRyRz)",rem1) + 13; //第一个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第一个浮点数的最后一个数字位置

                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->x = atof(temp1.data.c_str());//X

                    rem2 = rem1 + 2;//第二个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第二个浮点数的最后一个数字位置
                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->y = atof(temp1.data.c_str());//Y

                    rem2 = rem1 + 2;//第3个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第3个浮点数的最后一个数字位置
                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->z = atof(temp1.data.c_str());//Z

                    rem2 = rem1 + 2;//第4个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第4个浮点数的最后一个数字位置
                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->a = atof(temp1.data.c_str());//A

                    rem2 = rem1 + 2;//第5个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第5个浮点数的最后一个数字位置
                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->b = atof(temp1.data.c_str());//B

                    rem2 = rem1 + 2;//第6个浮点数的第一个数字位置
                    rem1 = info.data.find(',',rem2) - 1;//第5个浮点数的最后一个数字位置
                    temp1.data = info.data.substr(rem2,rem1 - rem2 +1);
                    pose->c = atof(temp1.data.c_str());//C

                    return GetPoseSuccess;
                    
                        
                }
                else
                {
                    ss << result;
                    if(ss.str().size() > 500)//if ss is too long and still not have \n ,we should abandon it
                        {
                            ss.str("");
                        }
                }
            
        } 
    return GetPoseFail;
}

bool GetPoseCmdService(mirobot::GetPoseCmd::Request &req, mirobot::GetPoseCmd::Response &res)
{
    res.result = GetPose(&pose);
    if (res.result == GetPoseSuccess) {
        //ROS_INFO("test_1!!!!");
        res.state = pose.state;
        res.x = pose.x;
        res.y = pose.y;
        res.z = pose.z;
        res.a = pose.a;
        res.b = pose.b;
        res.c = pose.c;
        res.jointAngle_4 = pose.jointAngle[3];
        res.jointAngle_5 = pose.jointAngle[4];
        res.jointAngle_6 = pose.jointAngle[5];
        res.jointAngle_7 = pose.jointAngle[6]; //axis 7
        res.jointAngle_1 = pose.jointAngle[0];
        res.jointAngle_2 = pose.jointAngle[1];
        res.jointAngle_3 = pose.jointAngle[2];
    }

    return true;
}

void InitPoseServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/MirobotServer/GetPoseCmd", GetPoseCmdService);
    serverVec.push_back(server);
}

//////////////////////////////////关节运动/////////////////////////////////
#include "mirobot/SetJointCmd.h"


typedef struct tagJointCmd {
    float Joint1;
    float Joint2;
    float Joint3;
    float Joint4;
    float Joint5;
    float Joint6;
    int speed;
}JointCmd;

int SendJointRelativeCmd(JointCmd *jointcmd)//关节相对运动
{
    std::string Gcode = "";
    char angle1[10];
	char angle2[10];
	char angle3[10];
	char angle4[10];
	char angle5[10];
	char angle6[10];
    char speed[10];

	sprintf(angle1, "%.2f", jointcmd->Joint1);
	sprintf(angle2, "%.2f", jointcmd->Joint2);
	sprintf(angle3, "%.2f", jointcmd->Joint3);
	sprintf(angle4, "%.2f", jointcmd->Joint4);
    sprintf(angle5, "%.2f", jointcmd->Joint5);
	sprintf(angle6, "%.2f", jointcmd->Joint6);
    sprintf(speed,  "%d",   jointcmd->speed);

	Gcode = (std::string)"M21G91G0X" + angle1 + "Y" + angle2 + "Z" + angle3 + "A" + angle4 + "B" + angle5 + "C" + angle6 + "F" + speed + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
    return (CheckOk(0.1));
}

bool SetJointRelativeCmdService(mirobot::SetJointCmd::Request &req, mirobot::SetJointCmd::Response &res)
{
    JointCmd cmd;
    cmd.Joint1 = req.jointAngle_1;
    cmd.Joint2 = req.jointAngle_2;
    cmd.Joint3 = req.jointAngle_3;
    cmd.Joint4 = req.jointAngle_4;
    cmd.Joint5 = req.jointAngle_5;
    cmd.Joint6 = req.jointAngle_6;
    cmd.speed = req.speed;
    
    res.result = SendJointRelativeCmd(&cmd);
    if (res.result == success) {
        ROS_INFO("Relative joint motion cmd receive ok!");
    }
    return true;
}

int SendJointAbsoluteCmd(JointCmd *jointcmd)//关节绝对运动
{
    std::string Gcode = "";
    char angle1[10];
	char angle2[10];
	char angle3[10];
	char angle4[10];
	char angle5[10];
	char angle6[10];
    char speed[10];

	sprintf(angle1, "%.2f", jointcmd->Joint1);
	sprintf(angle2, "%.2f", jointcmd->Joint2);
	sprintf(angle3, "%.2f", jointcmd->Joint3);
	sprintf(angle4, "%.2f", jointcmd->Joint4);
    sprintf(angle5, "%.2f", jointcmd->Joint5);
	sprintf(angle6, "%.2f", jointcmd->Joint6);
    sprintf(speed,  "%d",   jointcmd->speed);

	Gcode = (std::string)"M21G90G0X" + angle1 + "Y" + angle2 + "Z" + angle3 + "A" + angle4 + "B" + angle5 + "C" + angle6 + "F" + speed + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
    return (CheckOk(0.1));
}

bool SetJointAbsoluteCmdService(mirobot::SetJointCmd::Request &req, mirobot::SetJointCmd::Response &res)
{
    JointCmd cmd;
    cmd.Joint1 = req.jointAngle_1;
    cmd.Joint2 = req.jointAngle_2;
    cmd.Joint3 = req.jointAngle_3;
    cmd.Joint4 = req.jointAngle_4;
    cmd.Joint5 = req.jointAngle_5;
    cmd.Joint6 = req.jointAngle_6;
    cmd.speed = req.speed;
    
    res.result = SendJointAbsoluteCmd(&cmd);
    if (res.result == success) {
        ROS_INFO("Absolute joint motion cmd receive ok!");
    }
    return true;
}

void InitJointServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/MirobotServer/SetJointRelativeCmd", SetJointRelativeCmdService);
    serverVec.push_back(server);
    server = n.advertiseService("/MirobotServer/SetJointAbsoluteCmd", SetJointAbsoluteCmdService);
    serverVec.push_back(server);
}

//////////////////////////////迪卡尔空间运动////////////////////////
#include "mirobot/SetCartCmd.h"

int SendCartRelativeCmd(CartCmd *cartcmd)//迪卡尔相对运动
{
    std::string Gcode = "";
    char x[10];
	char y[10];
	char z[10];
	char a[10];
	char b[10];
	char c[10];
    char speed[10];

	sprintf(x, "%.2f", cartcmd->x);
	sprintf(y, "%.2f", cartcmd->y);
	sprintf(z, "%.2f", cartcmd->z);
	sprintf(a, "%.2f", cartcmd->a);
    sprintf(b, "%.2f", cartcmd->b);
	sprintf(c, "%.2f", cartcmd->c);
    sprintf(speed,  "%d",   cartcmd->speed);

	Gcode = (std::string)"M20G91G0X" + x + "Y" + y + "Z" + z + "A" + a + "B" + b + "C" + c + "F" + speed + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
    return (CheckOk(0.1));
}

bool SetCartRelativeCmdService(mirobot::SetCartCmd::Request &req, mirobot::SetCartCmd::Response &res)
{
    CartCmd cmd;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.a = req.a;
    cmd.b = req.b;
    cmd.c = req.c;
    cmd.speed = req.speed;
    
    res.result = SendCartRelativeCmd(&cmd);
    if (res.result == success) {
        ROS_INFO("Relative cartesian motion cmd receive ok!");
    }
    return true;
}

int SendCartAbsoluteCmd(CartCmd *cartcmd)//迪卡尔绝对运动
{
    std::string Gcode = "";
    char x[10];
	char y[10];
	char z[10];
	char a[10];
	char b[10];
	char c[10];
    char speed[10];

	sprintf(x, "%.2f", cartcmd->x);
	sprintf(y, "%.2f", cartcmd->y);
	sprintf(z, "%.2f", cartcmd->z);
	sprintf(a, "%.2f", cartcmd->a);
    sprintf(b, "%.2f", cartcmd->b);
	sprintf(c, "%.2f", cartcmd->c);
    sprintf(speed,  "%d",   cartcmd->speed);

	Gcode = (std::string)"M20G90G0X" + x + "Y" + y + "Z" + z + "A" + a + "B" + b + "C" + c + "F" + speed + "\r\n";
	ROS_INFO("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
    return (CheckOk(0.1));
}

bool SetCartAbsoluteCmdService(mirobot::SetCartCmd::Request &req, mirobot::SetCartCmd::Response &res)
{
    CartCmd cmd;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.a = req.a;
    cmd.b = req.b;
    cmd.c = req.c;
    cmd.speed = req.speed;
    
    res.result = SendCartAbsoluteCmd(&cmd);
    if (res.result == success) {
        ROS_INFO("Absolute cartesian motion cmd receive ok!");
    }
    return true;
}

void InitCartServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/MirobotServer/SetCartRelativeCmd", SetCartRelativeCmdService);
    serverVec.push_back(server);
    server = n.advertiseService("/MirobotServer/SetCartAbsoluteCmd", SetCartAbsoluteCmdService);
    serverVec.push_back(server);
}

//////////////////////任意G代码指令服务////////////////////////////////////////////////////
#include "mirobot/SetGcodeCmd.h"

int SendGcodeCmd(std::string Gcode)//任意Gcode发送,注意：复位指令必须发送：\$h  否则$会被转意
{
    std::string _Gcode = "";
    _Gcode = (std::string)Gcode + "\r\n";
	ROS_INFO("%s", _Gcode.c_str());
	_serial.write(_Gcode.c_str());
    return (CheckOk(0.1));
}

bool SetGcodeCmdService(mirobot::SetGcodeCmd::Request &req, mirobot::SetGcodeCmd::Response &res)
{
    std::string Gcode;
    Gcode = req.gcode;
    res.result = SendGcodeCmd(Gcode);
    if (res.result == success) {
        ROS_INFO("Gcode cmd receive ok!");
    }
    return true;
}

void InitGcodeServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/MirobotServer/SetGcodeCmd", SetGcodeCmdService);
    serverVec.push_back(server);
}


// void SerialCmd_callback(const std_msgs::String::ConstPtr& msg) //send serial command callback function
// { 
//     ROS_INFO("Writing to serial port: %s",msg->data.c_str()); 
//     _serial.write(msg->data);   //发送串口数据 
// } 


int main(int argc, char *argv[])
{
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application portName");
        return -1;
    }
    // Connect Mirobot before start the service
    int result = ConnectMirobot(argv[1], 115200);//"/dev/ttyUSB0"
    switch (result) {
        case MirobotConnect_Success:
            ROS_INFO("Mirobot has been connected successfully!");
        break;
        case MirobotConnect_Error:
            ROS_ERROR("Unable to open port!");
            return -2;
        break;
        default:
        break;
    }
    ros::init(argc, argv, "MirobotServer");
    ros::NodeHandle n;

    std::vector<ros::ServiceServer> serverVec;

    InitHOMEServices(n, serverVec);//robot arm home service
    InitPoseServices(n, serverVec);//get robot pose
    InitJointServices(n, serverVec);//关节运动指令 ,分为绝对和相对
    InitCartServices(n, serverVec);//笛卡尔空间运动指令，分为相对绝对，//还未实现插补运动
    InitGcodeServices(n, serverVec);//任意G代码指令发送


    //ros::Subscriber write_sub = n.subscribe("/MirobotTopic/SerialCmd", 1, SerialCmd_callback); //topic used to send serial command.
    //ros::Publisher read_pub = n.advertise<std_msgs::String>("/MirobotTopic/RevInfo", 1000); //topic used to show mirobot serial infomation.

    ROS_INFO("Mirobot service and topic running...");
    ros::spin();
    ROS_INFO("Mirobot service and topic exiting...");

    // Disconnect Mirobot
    DisconnectMirobot();

    return 0;



}




















