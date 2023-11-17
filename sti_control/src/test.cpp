// #include <stdio.h>
// #include <stdlib.h>
// #include <string>
// #include <iostream>
// #include <algorithm>
// #include <cstring>
// // lib 
// #include <vector>


// typedef unsigned char byte;

// using namespace std;


// char* byteToChar(byte* b, int length) {
//     char c[length];
//     for (int i = 0; i < length; i++) {
//     c[i] = b[i] - 128;
//     }
//     return c;
// }

// /*
// byte* charToByte(char* c, int length) {
//     byte b[length];
//     for (int i = 0; i < length; i++) {
//     b[i] = c[i] + 128;
//     }
//     return b;
// }
// */

// vector<byte> intToVectorByte(int n, int numByte) {
//     vector<byte> result;
//     if (numByte > 3){ result.push_back((n & 0xff000000) >> 24);}
//     if (numByte > 2){ result.push_back((n & 0x00ff0000) >> 16);}
//     if (numByte > 1){ result.push_back((n & 0x0000ff00) >> 8);} 
//     result.push_back(n & 0x000000ff);
//     return result;
// }

// int bytes_list_to_int3(vector<byte> b, int pos, int len){
//     int value = 0;
//     for (int i = 0; i < len; i++){
//         cout << int(b.at(pos+len-1-i)) << " ";
//         value = value | (b.at(pos+len-1-i) << i*8);
//     }
//     cout << endl;
//     return value;
// }

// int bytes_list_to_int(vector<byte> b, int pos, int len){
//     int value;
//     byte bytes[len];
//     for (int i = 0; i < len; i++){
//         bytes[i] = b.at(pos+len-1-i);
//     }
//     memcpy(&value, bytes, sizeof(int));
//     return value;
// }

// int byteToInt(byte* byte) {

//     int n = 0;
//     n = n + (byte[0] & 0x000000ff);
//     n = n + ((byte[1] & 0x000000ff) << 8);
//     n = n + ((byte[2] & 0x000000ff) << 16);
//     n = n + ((byte[3] & 0x000000ff) << 24);
//     return n;
// }

// int main(int argc, char** argv)
// {
//     // int number = 500;
//     // int number1 = 20;
//     // // byte* line = intToByte(number);
//     // vector<byte> arr1 = intToVectorByte(number, 2);
//     // vector<byte> arr2 = intToVectorByte(number1, 4);

//     // arr1.insert( arr1.end(), arr2.begin(), arr2.end() );
    

//     // for (int i = 0; i < arr1.size(); i++){
//     //     cout << int(arr1[i]) << " ";
//     // }

//     // cout << "end" << endl;
//     // int value = 0;
//     // byte bytes[3] = {1,0,0};
//     // memcpy(&value, bytes, sizeof(int));
//     // cout << value << endl;

//     vector<byte> vtb = {byte(70), byte(217), byte(0), byte(0)};
//     cout << bytes_list_to_int3(vtb, 0, 2) << endl;
//     cout << bytes_list_to_int(vtb, 0, 2) << endl;

// /*



//     TCPConnector* connector = new TCPConnector();
//     TCPStream* stream = connector->connect(argv[2], atoi(argv[1]));
//     if (stream) {
//         stream->send(byteToChar(line, 4), 4);
//         delete stream;
//     }

// */
//     return 0;
// }

/* --------------------------------------------------------------------------- */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
void check_host_name(int hostname) { //This function returns host name for local computer
   if (hostname == -1) {
      perror("gethostname");
      exit(1);
   }
}
void check_host_entry(struct hostent * hostentry) { //find host info from host name
   if (hostentry == NULL){
      perror("gethostbyname");
      exit(1);
   }
}
void IP_formatter(char *IPbuffer) { //convert IP string to dotted decimal format
   if (NULL == IPbuffer) {
      perror("inet_ntoa");
      exit(1);
   }
}


string getIPAddress(){
    string ipAddress="Unable to get IP Address";
    struct ifaddrs *interfaces = NULL;
    struct ifaddrs *temp_addr = NULL;
    int success = 0;
    // retrieve the current interfaces - returns 0 on success
    success = getifaddrs(&interfaces);
    if (success == 0) {
        // Loop through linked list of interfaces
        temp_addr = interfaces;
        while(temp_addr != NULL) {
            if(temp_addr->ifa_addr->sa_family == AF_INET) {
                // Check if interface is en0 which is the wifi connection on the iPhone
                if(strcmp(temp_addr->ifa_name, "en0")==0){
                    ipAddress=inet_ntoa(((struct sockaddr_in*)temp_addr->ifa_addr)->sin_addr);
                }
            }
            temp_addr = temp_addr->ifa_next;
        }
    }
    // Free memory
    freeifaddrs(interfaces);
    return ipAddress;
}


int main() {
   char host[256];
   char *IP;
   struct hostent *host_entry;
   int hostname;
   hostname = gethostname(host, sizeof(host)); //find the host name
   check_host_name(hostname);
   host_entry = gethostbyname(host); //find host information
   check_host_entry(host_entry);
   IP = inet_ntoa(*((struct in_addr*) host_entry->h_addr_list[0])); //Convert into IP string
   printf("Current Host Name: %s\n", host);
   printf("Host IP: %s\n", IP);
}