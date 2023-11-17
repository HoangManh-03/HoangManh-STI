// lib ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sti_msgs/NN_cmdRequest.h>
#include <sti_msgs/NN_infoRequest.h>
#include <sti_msgs/NN_infoRespond.h>

// lib sys + socket
#include <sys/ioctl.h>
#include <linux/if.h>

#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 

// lib 
#include <vector>
#include <math.h>

#define MAXLINE  1024 
#define PI 3.141592653589793238463

typedef unsigned char byte;

using namespace std;

class read_and_respond_UDP
{
private:
    // pub object
    ros::Publisher NN_cmdPub;
    ros::Publisher NN_infoRequestPub;
    // sub object
    ros::Subscriber subNN_infoRespond;

    /* mesenger */
    sti_msgs::NN_cmdRequest NN_cmdRequest;
    sti_msgs::NN_infoRespond NN_infoRespond;
    sti_msgs::NN_infoRequest NN_infoRequest;

    /* */

    /* bien server */ 
    int sockfd;
    vector<byte> data_received;
    struct sockaddr_in servRecAddr, myAddr, servSenAddr; 
    string name_card = "wlo2";
    int PORT_receive = 8888;
    int PORT_sento = 8000;
    string HOST_receive = "172.21.84.213";

    /* bien chuong trinh */
    uint16_t process = 1;
    uint8_t NN_is_infoReceived = 0;

public:
    read_and_respond_UDP(ros::NodeHandle *nh, ros::NodeHandle *npr){
        /* get param define */
        npr->param<std::string>("name_card", name_card, "wlo2");
        npr->param<int>("PORT_receive", PORT_receive, 8888);
        npr->param<int>("PORT_sento", PORT_sento, 8000);
        /* info pub */
        NN_cmdPub = nh->advertise<sti_msgs::NN_cmdRequest>("/NN_cmdRequest", 50);
        NN_infoRequestPub = nh->advertise<sti_msgs::NN_infoRequest>("/NN_infoRequest", 50);
        /* info sub */
        subNN_infoRespond = nh->subscribe("NN_infoRespond", 1000, &read_and_respond_UDP::NN_infoCallback, this);

        /*Creating socket file descriptor */
        if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0){ 
            perror("socket creation failed"); 
            exit(EXIT_FAILURE); 
        }
        std::cout << "Socket created \n";

        /* get IP Address */
        struct ifreq ifr{};
        strcpy(ifr.ifr_name, name_card.c_str());
        ioctl(sockfd, SIOCGIFADDR, &ifr);
        char myIP[INET_ADDRSTRLEN];
        strcpy(myIP, inet_ntoa(((sockaddr_in *) &ifr.ifr_addr)->sin_addr));

        memset(&myAddr, 0, sizeof(myAddr)); 
        /* Filling server information  */ 
        myAddr.sin_family = AF_INET; 
        myAddr.sin_port = htons(PORT_receive); 
        // servaddr.sin_addr.s_addr = INADDR_ANY; 
        // myAddr.sin_addr.s_addr = inet_addr("192.168.1.100"); 
        myAddr.sin_addr.s_addr = inet_addr(static_cast<const char*>(myIP)); 

        /* Connect to server UDP */
        if (bind(sockfd, (const struct sockaddr *)&myAddr, sizeof(myAddr)) < 0) {
            perror("Connection error");
            exit(EXIT_FAILURE); 
        }
        /* lay thong tin ip va port */
        char str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(myAddr.sin_addr), str, INET_ADDRSTRLEN);
        cout << "Socket bind addr: " << str << " - port: " << ntohs(myAddr.sin_port) << endl;

    };
    ~read_and_respond_UDP(){};

    void NN_infoCallback(const sti_msgs::NN_infoRespond& data){
        NN_infoRespond = data;
        NN_is_infoReceived = 1;
    }

    void closeUPD(){
        close(sockfd);
    }

    float radToDeg(float rad){
        return (rad * (180./PI));
    }

    vector<byte> intToVectorByte(int n, int numByte) {
        vector<byte> result;
        if (numByte > 3){ result.push_back((n & 0xff000000) >> 24);}
        if (numByte > 2){ result.push_back((n & 0x00ff0000) >> 16);}
        if (numByte > 1){ result.push_back((n & 0x0000ff00) >> 8);} 
        result.push_back(n & 0x000000ff);
        return result;
    }

    int byteToInt(byte* byte) {
        int n = 0;
        n = n + (byte[0] & 0x000000ff);
        n = n + ((byte[1] & 0x000000ff) << 8);
        n = n + ((byte[2] & 0x000000ff) << 16);
        n = n + ((byte[3] & 0x000000ff) << 24);
        return n;
    }

    int bytes_list_to_int3(vector<byte> b, int pos, int len){
        if (len > 1){
            int value;
            byte bytes[len];
            for (int i = 0; i < len; i++){
                bytes[i] = b.at(pos+len-1-i);
            }

            memcpy(&value, bytes, sizeof(int));
            return value;
        }else{
            return int(b.at(pos));
        }
    }

    int bytes_list_to_int(vector<byte> b, int pos, int len){
        int value = 0;
        for (int i = 0; i < len; i++){
            value = value | (b.at(pos+len-1-i) << i*8);
        }
        return value;
    }

    char* vectorByteToChar(vector<byte> b) {
        int len = b.size();
        char c[len];
        for (int i = 0; i < len; i++) {
            c[i] = b[i] - 128;
            cout << c[i] << " ";
        }
        cout << endl;
        return c;
    }

    string convertToString(vector<byte> b, int pos, int len)
    {
        int i = pos;
        string s = "";
        for (i; i < pos + len; i++) {
            s = s + char(b[i]);
        }
        return s;
    }

    int byte_to_int(const char &data){
        return int(data);
    }

    vector<byte> coordinates_to_bytes_vs2(float _data){
        vector<byte> data_raw;
        vector<byte> byte0 = intToVectorByte(0, 1);
        data_raw.insert( data_raw.end(), byte0.begin(), byte0.end());

        int data = int(_data*1000);
        vector<byte> vectorData = intToVectorByte(data, 4);
        data_raw.insert( data_raw.end(), vectorData.begin(), vectorData.end());

        return data_raw;
    }

    vector<byte> direction_to_bytes(float data){
        vector<byte> data_raw;
        if (data > 4.) {data_raw = intToVectorByte(40000, 4);}
        else if (data >= 0. && data < 4.){
            int dir = int(radToDeg(data)*100.);
            data_raw = intToVectorByte(dir, 4);
        }else{
            int dir = int((360. + radToDeg(data))*100.);
            data_raw = intToVectorByte(dir, 4);
        }
        return data_raw;
    }

    float bytes_list_to_coor_vs1(vector<byte> b, int pos, int len){
        int rawValue = bytes_list_to_int(b, pos, len);
        float value = float(rawValue)/1000.;
        return roundf(value * 1000)/1000;
    }

    float bytes_list_to_offset(vector<byte> b, int pos, int len){
        int rawValue = bytes_list_to_int(b, pos, len);
        float value = float(rawValue)/1000.;
        return roundf(value * 1000)/1000;
    }

    float bytes_list_to_corner(vector<byte> b, int pos, int len){
        int rawValue = bytes_list_to_int(b, pos, len);
        float value = float(rawValue)/100.;
        if (value > 180.){
            value = 360. - value;
            value = (value/180.)*PI*(-1.);
        }else{
            value = (value/180.)*PI;
        }
        return roundf(value * 1000)/1000;
    }

    vector<byte> NN_infoBuild(){
        vector<byte> data_raw;

        vector<byte> posX = coordinates_to_bytes_vs2(NN_infoRespond.x);
        data_raw.insert( data_raw.end(), posX.begin(), posX.end() );

        vector<byte> posY = coordinates_to_bytes_vs2(NN_infoRespond.y);
        data_raw.insert( data_raw.end(), posY.begin(), posY.end() );

        vector<byte> dir = direction_to_bytes(NN_infoRespond.z);
        data_raw.insert( data_raw.end(), dir.begin(), dir.end() );

        vector<byte> tag = intToVectorByte(NN_infoRespond.tag, 2);
        data_raw.insert( data_raw.end(), tag.begin(), tag.end() );

        vector<byte> battery = intToVectorByte(NN_infoRespond.battery, 1);
        data_raw.insert( data_raw.end(), battery.begin(), battery.end() );

        vector<byte> status = intToVectorByte(NN_infoRespond.status, 1);
        data_raw.insert( data_raw.end(), status.begin(), status.end() );

        vector<byte> mode = intToVectorByte(NN_infoRespond.mode, 1);
        data_raw.insert( data_raw.end(), mode.begin(), mode.end() );

        vector<byte> spare1 = intToVectorByte(0, 1);
        data_raw.insert( data_raw.end(), spare1.begin(), spare1.end() );

        vector<byte> task_status = intToVectorByte(NN_infoRespond.task_status, 1);
        data_raw.insert( data_raw.end(), task_status.begin(), task_status.end() );

        return data_raw;
    }

    void NN_cmdAnalysis(){
        int len = data_received.size();
        NN_cmdRequest.id_command = bytes_list_to_int(data_received, 2, 4);
        NN_cmdRequest.process = bytes_list_to_int(data_received, 6, 1);
        NN_cmdRequest.tag = bytes_list_to_int(data_received, 7, 2);
        // NN_cmdRequest.target_id = bytes_list_to_int(data_received, 9, 2);
        NN_cmdRequest.target_x = bytes_list_to_coor_vs1(data_received, 9, 4);
        NN_cmdRequest.target_y = bytes_list_to_coor_vs1(data_received, 13, 4);
        NN_cmdRequest.target_z = bytes_list_to_corner(data_received, 17, 2);
        NN_cmdRequest.offset = bytes_list_to_offset(data_received, 19, 4);

        /* list ID, X, Y, SPEED */
        int lenList = 5;
        // vector<int> arrID[lenList];
        // vector<int> arrX[lenList];
        // vector<int> arrY[lenList];
        // vector<int> arrSpeed[lenList]; 

        NN_cmdRequest.list_id.clear();
        NN_cmdRequest.list_x.clear();
        NN_cmdRequest.list_y.clear();
        NN_cmdRequest.list_speed.clear();

        for (int i = 0; i < lenList; i++) {
            NN_cmdRequest.list_id.push_back(bytes_list_to_int(data_received, 23 + i*11, 2));
            NN_cmdRequest.list_x.push_back(bytes_list_to_coor_vs1(data_received, 25 + i*11, 4));
            NN_cmdRequest.list_y.push_back(bytes_list_to_coor_vs1(data_received, 29 + i*11, 4));
            NN_cmdRequest.list_speed.push_back(int(data_received.at(33+i*11)));
        }

        // NN_cmdRequest.list_id = arrID;
        // NN_cmdRequest.list_x = arrX;
        // NN_cmdRequest.list_y = arrY;
        // NN_cmdRequest.list_speed = arrSpeed;

        NN_cmdRequest.before_mission = bytes_list_to_int(data_received, 78, 1);
        NN_cmdRequest.after_mission = bytes_list_to_int(data_received, 79, 1);
        
        if (len > 80){
            NN_cmdRequest.command = convertToString(data_received, 80, len - 80);
        }else{
            NN_cmdRequest.command = "";
        }

    }

    void testFrameSendServer(){
        if (NN_is_infoReceived == 1){
            vector<byte> dt = NN_infoBuild();
            for (int i = 0; i < dt.size(); i++){
                cout << int(dt[i]) << " ";
            }
            cout << endl;
        }
    }

    void run(){
        if (process == 1){
            int n;
            socklen_t len = sizeof(servRecAddr);
            char buffer[MAXLINE]; 

            n = recvfrom(sockfd, (char *)buffer, MAXLINE,  
                MSG_WAITALL, (struct sockaddr *) &servRecAddr, 
                &len); 

            if (n > 0){
                buffer[n] = '\0';

                /* save info sever */
                memset(&servSenAddr, 0, sizeof(servSenAddr)); 
                servSenAddr.sin_family = servRecAddr.sin_family; 
                servSenAddr.sin_port = htons(PORT_sento); 
                servSenAddr.sin_addr.s_addr = servRecAddr.sin_addr.s_addr; 

                /* get data */
                data_received.clear();
                for (int i = 0; i < n; i++){
                    data_received.push_back(byte(buffer[i]));
                }
                process = 2;
            }
            else{
                cout << "Error -- recv() --! " << n << endl;
            }
        }
        else if (process == 2) /* kiem tra kich thuoc cua data */
        {
            if (data_received[0] != data_received.size()){
                cout << "Error lenght frame" << endl;
                process = 1;

            }else{
                process = 3;
                cout << "Data received: ";
                for (int i = 0; i < data_received.size(); i++){
                    cout << int(data_received[i]) << " ";
                }
                cout << "\n---" << endl;
            }
        }
        else if (process == 3) /* nhan dien frame*/
        {
            if (data_received[1] == 85){ /* info NN 'U' */
                process = 4;
            }else if (data_received[1] == 67){ /* info NN 'C' */
                process = 5;
            }else{
                // cout << "Recivce dif frame, data [1]: " << data_received[1] << endl;
                process = 1;
            }
        }
        else if (process == 4){ /* info NN - analysis and respond */
            if (NN_is_infoReceived == 1){
                // cout << "frame truyen process = 3 \n";
                vector<byte> mm;
                vector<byte> byte0 = intToVectorByte(27,1);
                mm.insert( mm.end(), byte0.begin(), byte0.end() );

                vector<byte> byte1 = {byte(data_received[1])};
                mm.insert( mm.end(), byte1.begin(), byte1.end() );

                vector<byte> byte2to5 = {byte(data_received[2]), byte(data_received[3]), byte(data_received[4]), byte(data_received[5])};
                mm.insert( mm.end(), byte2to5.begin(), byte2to5.end() );

                vector<byte> byte6toEnd = NN_infoBuild();
                mm.insert( mm.end(), byte6toEnd.begin(), byte6toEnd.end() );


                /* print */
                // for (int i = 0; i < mm.size(); i++){
                //     cout << int(mm[i]) << " ";
                // }
                // cout << "\n ----------------" << endl;

                /* send udp server */
                socklen_t len = sizeof(servSenAddr);
                byte msg[mm.size()];
                for (int i = 0; i < mm.size(); i++){
                    msg[i] = mm.at(i);
                }
                sendto(sockfd, msg, sizeof(msg),  
                    MSG_CONFIRM, (const struct sockaddr *) &servSenAddr, 
                    len); 
            }
            else{
                cout << "Wait respond from Sti_control \n";
            }

            /* pub info request by Server */
            int len = data_received.size();
            // cout << "id agv: " << bytes_list_to_int(data_received, 2, 4) << " name: " << convertToString(data_received, 6, len - 6) << endl;
            NN_infoRequest.id_agv = bytes_list_to_int(data_received, 2, 4);
            if (len > 6){
                NN_infoRequest.name_agv = convertToString(data_received, 6, len - 6);
            }else{
                NN_infoRequest.name_agv = "";
            }
            NN_infoRequestPub.publish(NN_infoRequest);
            process = 1;
        }
        else if (process == 5){ /* command NN - analysis and respond */
            // cout << "frame truyen process = 4 \n";
            vector<byte> mm;
            vector<byte> byte0 = intToVectorByte(6,1);
            mm.insert( mm.end(), byte0.begin(), byte0.end() );

            vector<byte> byte1to5 = {byte(data_received[1]), byte(data_received[2]), byte(data_received[3]), byte(data_received[4]), byte(data_received[5])};
            mm.insert( mm.end(), byte1to5.begin(), byte1to5.end() );

            // for (int i = 0; i < mm.size(); i++){
            //         cout << int(mm[i]) << " ";
            //     }
            // cout << " \n ----------------" << endl;

            ros::Duration(0.01).sleep();
            /* send udp server */
            socklen_t len = sizeof(servSenAddr);
            byte msg[mm.size()];
            for (int i = 0; i < mm.size(); i++){
                msg[i] = mm.at(i);
            }
            sendto(sockfd, msg, sizeof(msg),  
                MSG_CONFIRM, (const struct sockaddr *) &servSenAddr, 
                len);

            int len2 = data_received.size();
            if (len2 >= 80){
                NN_cmdAnalysis();
                NN_cmdPub.publish(NN_cmdRequest);

            }else{
                cout << "NN: Error size of Frame command: " << len2 << endl;
            }
            process = 1;
        }
        
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "stiClient_nav");
    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle("~");
    ros::Rate rate(50); // ROS Rate at 5Hz
    // private_node_handle.param<int>("hh", he, 20);

    read_and_respond_UDP class_1 = read_and_respond_UDP(&nh, &private_node_handle);

    while (ros::ok()) {
        class_1.run();
        rate.sleep();
        ros::spinOnce(); 
    }
    class_1.closeUPD(); 
    cout << "\nClose UDP socket - Exit program!" << endl;
    return 0;
}
