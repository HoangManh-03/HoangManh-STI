// Copyright (c) 2022 Papa Libasse Sow.
// https://github.com/Nandite/R2000
// Distributed under the MIT Software License (X11 license).
//
// SPDX-License-Identifier: MIT
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of
// the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "ros/ros.h"
#include "ros/console.h"

#include "std_msgs/Int8.h"
#include "sensor_msgs/LaserScan.h"
#include "r2000_driver/ScanFull.h"

#include <bits/stdc++.h>

#include "DataLink/DataLink.hpp"
#include "DataLink/DataLinkBuilder.hpp"
#include "R2000.hpp"
#include <thread>
#include "Control/Parameters.hpp"

using namespace std::chrono_literals;
// using namespace std;

/**
 * @param address The address to test.
 * @return True if the address has an ipv4 valid form, False otherwise.
 */
[[nodiscard]] bool isValidIpv4(const std::string& address)
{
    boost::system::error_code errorCode{};
    const auto ipv4Address{boost::asio::ip::address::from_string(address, errorCode)};
    return !errorCode && ipv4Address.is_v4();
}

bool interruptProgram{false};

void interrupt(int)
{
    interruptProgram = true;
}

void printUsage(std::ostream& stream)
{
    stream << "Acquire scan from the sensor. Program usage:" << std::endl << "./AcquireScan <ipv4>";
}

int main(int argc, char** argv)
{
    std::cout << "Program start!" << std::endl;

    ros::init(argc, argv, "AcquireScan");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    std::signal(SIGTERM, interrupt);
    std::signal(SIGKILL, interrupt);
    std::signal(SIGINT, interrupt);

    // if (argc < 2)
    // {
    //     printUsage(std::cout);
    //     return EXIT_SUCCESS;
    // }

    std::string deviceAddress;
    ros::param::get("~deviceAddress", deviceAddress);
    ros::Time last_acquired_point_stamp;

    // check if the address is valid
    if (!isValidIpv4(deviceAddress))
    {
        std::clog << "The provided address is not valid [" << deviceAddress << "]" << std::endl;
        printUsage(std::clog);
        return EXIT_FAILURE;
    }

    // topic pub-sub
    ros::Publisher pub_scan = nh.advertise<sensor_msgs::LaserScan>("/self_scan", 20);
    sensor_msgs::LaserScan scan_msg;

    ros::Publisher pub_scanheader = nh.advertise<r2000_driver::ScanFull>("/self_scan_header", 20);
    r2000_driver::ScanFull scanfull_msg;

    // init device
    const auto device{Device::R2000::makeShared({"R2000", deviceAddress})};
    
    // setting param for device
    auto handleParameters{Device::Parameters::ReadWriteParameters::TcpHandle{}
                              .withWatchdog()
                              .withWatchdogTimeout(60000)
                              .withPacketType(Device::Parameters::PACKET_TYPE::C)
                              };

    // get sensor capabilities
    // int min_range = 0;
    // int max_range = 0;
    // int angular_fov = 0;

    const auto sensorCapabiliesParameters{Device::Parameters::ReadOnlyParameters::Capabilities{}
                            .requestRadialMinRange()
                            .requestRadialMaxRange()
                            .requestAngularFov()};

    // auto future1{Device::Commands::GetParametersCommand.asyncExecute(2s, sensorCapabiliesParameters)}; // Asynchronous execution with 2 seconds timeout
    // if(!future1)
    // {
    //     // The command could not been submitted for execution
    // }

    // auto result{future1->get()};

    // link device with tcp
    auto future{Device::DataLinkBuilder(handleParameters).build(device, 1s)};
    
    auto [requestResult, dataLink]{future.get()};

    if (requestResult != Device::RequestResult::SUCCESS)
    {
        std::clog << "Could not establish a data link with sensor at " << device->getHostname() << " ("
                  << Device::requestResultToString(requestResult) << ")." << std::endl;
        return EXIT_FAILURE;
    }

    // Device::Commands::GetParametersCommand getParametersCommandInstance{*device}; // Tạo một instance
    // const auto started [[maybe_unused]] = getParametersCommandInstance.asyncExecute(sensorCapabiliesParameters, [&](const auto result) {
    //     // Extract the result
    //     const auto& [requestResult, parametersMap] = result;

    //     // Print the request result
    //     std::cout << "Request Result: " << Device::requestResultToString(requestResult) << std::endl;

    //     // Print the parameters map
    //     std::cout << "Capabilities:" << std::endl;
    //     for (const auto& [key, value] : parametersMap) {
    //         if (key == "radial_range_min")
    //         {
    //             min_range = std::stoi(value);
    //         }
    //         if (key == "radial_range_max")
    //         {
    //             max_range = std::stoi(value);
    //         }
    //         if (key == "angular_fov")
    //         {
    //             angular_fov = std::stoi(value)/(2*M_PI);
    //         }
    //         std::cout << "  " << key << ": " << value << std::endl;
    //     }
    // }, 2s); // Asynchronous execution with 10 seconds timeout
    
    // Create an instance of GetParametersCommand
    // Device::Commands::GetParametersCommand getParametersCommandInstance{*device};

    // // Call execute with individual builders
    // auto [result, parametersMap] = getParametersCommandInstance.execute(
    //     radialMinRangeBuilder, radialMaxRangeBuilder, angularFovBuilder
    // );

    // // Check the result
    // if (result == Device::RequestResult::SUCCESS) {
    //     std::cout << "Successfully retrieved parameters:" << std::endl;

    //     for (const auto& [key, value] : parametersMap) {
    //         std::cout << key << ": " << value << std::endl;
    //     }
    // } else {
    //     std::cerr << "Failed to retrieve parameters: "
    //             << Device::requestResultToString(result) << std::endl;
    // }

    // read data scan back 
    dataLink->addOnNewScanAvailableCallback(
        [&](const auto& newScan)
        {   
            // std::cout << "package type is [" << newScan->getHeaders()[0].packetType << "]" << std::endl;
            // std::cout << "Scan number [" << newScan->getHeaders()[0].scanNumber << "] has been received" << std::endl;
            // std::cout << "number points of scan [" << newScan->getHeaders()[0].numPointsScan << "]" << std::endl;
            // std::cout << "góc start [" << newScan->getHeaders()[0].firstAngle << "]" << std::endl;
            // std::cout << "góc tăng [" << newScan->getHeaders()[0].angularIncrement << "]" << std::endl;

            // get distances data
            // std::cout << "kích thước của mảng kc là: [" << newScan->getDistances().size() << "]" << std::endl;
            
            // get amplitudes data
            // std::cout << "kích thước của mảng biên độ là: [" << newScan->getAmplitudes().size() << "]" << std::endl;

            std::cout << "Get receive new data "<< std::endl;
            // publish r2000 header code
            scanfull_msg.magic = newScan->getHeaders()[0].magic;
            scanfull_msg.packet_type = newScan->getHeaders()[0].packetType;
            scanfull_msg.packet_size = newScan->getHeaders()[0].packetSize;
            scanfull_msg.header_size = newScan->getHeaders()[0].headerSize;
            scanfull_msg.scan_number = newScan->getHeaders()[0].scanNumber;
            scanfull_msg.packet_number = newScan->getHeaders()[0].packetNumber;
            scanfull_msg.timestamp_raw = newScan->getHeaders()[0].timestampRaw;
            scanfull_msg.timestamp_sync = newScan->getHeaders()[0].timestampSync;
            scanfull_msg.status_flags = newScan->getHeaders()[0].statusFlags;
            scanfull_msg.scan_frequency = newScan->getHeaders()[0].scanFrequency;
            scanfull_msg.num_points_scan = newScan->getHeaders()[0].numPointsScan;
            scanfull_msg.num_points_packet = newScan->getHeaders()[0].numPointsPacket;
            scanfull_msg.first_index = newScan->getHeaders()[0].firstIndex;
            scanfull_msg.first_angle = newScan->getHeaders()[0].firstAngle; 
            scanfull_msg.angular_increment = newScan->getHeaders()[0].angularIncrement;
            scanfull_msg.iq_input = newScan->getHeaders()[0].iqInput;
            scanfull_msg.iq_overload = newScan->getHeaders()[0].iqOverload;
            scanfull_msg.iq_timestamp_raw = newScan->getHeaders()[0].iqTimestampRaw;
            scanfull_msg.iq_timestamp_sync = newScan->getHeaders()[0].iqTimestampSync;

            pub_scanheader.publish(scanfull_msg);

            // publish laser scan code
            const auto scan_time = ros::Duration(1000.0 / newScan->getHeaders()[0].scanFrequency);
            scan_msg.header.seq = newScan->getHeaders()[0].scanNumber;
            scan_msg.header.stamp = ros::Time::now() - scan_time;
            scan_msg.header.frame_id.assign("scanner_link");

            scan_msg.angle_min = -M_PI;
            scan_msg.angle_max = M_PI;
            scan_msg.angle_increment = newScan->getHeaders()[0].angularIncrement / 10000.0 * (M_PI / 180.0);
            scan_msg.scan_time = static_cast<float>(scan_time.toSec());
            scan_msg.time_increment = scan_msg.scan_time / newScan->getHeaders()[0].numPointsScan;
            scan_msg.range_min = 0;
            scan_msg.range_max = 30.0;

            std::vector<float> distances(newScan->getDistances().size());
            std::transform(newScan->getDistances().begin(), newScan->getDistances().end(), distances.begin(),
                        [](float distance) { return distance / 1000.0f; });

            scan_msg.ranges = std::move(distances);

            std::vector<float> amplitudes(newScan->getAmplitudes().size());
            std::transform(newScan->getAmplitudes().begin(), newScan->getAmplitudes().end(), amplitudes.begin(),
                        [](float amplitude) { return amplitude > 5000.0f ? 5000.0f : amplitude; });

            scan_msg.intensities = std::move(amplitudes);

            // scan_msg.ranges = std::vector<float>(newScan->getDistances().begin(), newScan->getDistances().end());
            // scan_msg.intensities = std::vector<float>(newScan->getAmplitudes().begin(), newScan->getAmplitudes().end());

            pub_scan.publish(scan_msg);

        });
    while (!interruptProgram)
    {
        std::this_thread::sleep_for(1s);
        if(dataLink->isStalled())
        {
            std::clog << "Data link has stalled" << std::endl;
            break;
        }
    }
    std::cout << std::endl << "Stopping scan acquisition." << std::endl;
    return EXIT_SUCCESS;
}