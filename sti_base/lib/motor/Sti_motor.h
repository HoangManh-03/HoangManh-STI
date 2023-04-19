
/**
*   @file       : AGV_controller_motor_driver.h
*   @version    : 2.0.0
*   @brief      : dieu khien hai banh
*   @details    : dieu khien toc do, phanh, chuyen dong cua 2 banh
*/
/*==================================================================================================
*   Project              : AGV for Electronic Circuit
*   Platform             : ESP32-WROVER
*   Peripheral           :
*   Dependencies         :

*   Code Version         : 2.0.0
*   Code Revision        : 0
*   Hardware Version     : 4.0.0
*   Build Date           : 5/2/2020
*   Release Date         :
==================================================================================================*/
/*==================================================================================================
Revision History:
                             Modification     Tracking
Author                        Date D/M/Y       Number     Description of Changes
---------------------------   ----------    ------------  ------------------------------------------
Hoang Van Quang               09/7/2020          1        Create file
---------------------------   ----------    ------------  ------------------------------------------
Task:

*/
#ifndef STI_MOTOR_H
#define STI_MOTOR_H

#include "Arduino.h"
#include <PCF8574.h>
PCF8574 pcf38(0x38); // 0x20
//  200W
class STI_motor_driver{
    private:
	  // -- I/O
        // Enb
        int _FWD_pin1;
        int _REV_pin1;
	  // -- PCF8574
        int _FWD_pin2;
        int _REV_pin2;
        // cai dat toc do
        int _speed_setting_pin1;
		int _speed_setting_pin2;
        // canh bao
        int _alarm_in_pin1;
		int _alarm_in_pin2;
      // 
        // chieu dieu khien (toan cuc)
        int dir1;
		int dir2;
        // so sanh tin hieu dieu khien
        int last_analog1;
		int last_analog2;
		bool ON =  true;
		bool OFF = false;
    public:
        STI_motor_driver(int FWD_pin1, int REV_pin1, int speed_setting_pin1, int alarm_in_pin1
						,int FWD_pin2, int REV_pin2, int speed_setting_pin2,  int alarm_in_pin2) {
		// Output
            _FWD_pin1 = FWD_pin1;
            _REV_pin1 = REV_pin1;

            _FWD_pin2 = FWD_pin2;
			_REV_pin2 = REV_pin2;

            _speed_setting_pin1 = speed_setting_pin1;
			_speed_setting_pin2 = speed_setting_pin2;

        // Input    
            _alarm_in_pin1 = alarm_in_pin1;
			_alarm_in_pin2 = alarm_in_pin2;
        }
        ~STI_motor_driver() {}

        /**
        * @brief          : setup chan ket noi
        * @details        : cai dat pinmode va cac gia tri ban dau
        * @requirements   : cac gia tri chan duoc define trong file config
        */
        void driver_pin_init() {
			pcf38.begin();

			pinMode(_speed_setting_pin1, OUTPUT);
            pinMode(_speed_setting_pin2, OUTPUT);

			pinMode(_alarm_in_pin1, INPUT);
			pinMode(_alarm_in_pin2, INPUT);
			
			dacWrite(_speed_setting_pin1, 0);
			dacWrite(_speed_setting_pin2, 0);			
            //ensure that the motor is in neutral state during bootup
            pcf38.write(_FWD_pin1, OFF);
            pcf38.write(_REV_pin1, OFF); 

			pcf38.write(_FWD_pin2, OFF);
			pinMode(_REV_pin2, OUTPUT);
			digitalWrite(_REV_pin2, 0);
            
            dir1 = 0;
			dir2 = 0;
            last_analog1 = 0;
			last_analog2 = 0;
        }
		void blink(){
			// digitalWrite(_REV_pin1, ON);
			// pcf38.write(_REV_pin2, ON);
			// pcf38.write(_FWD_pin1, ON);
			// pcf38.write(_FWD_pin2, ON);
            // // pcf38.write(4, OFF);
			// pcf38.write(_REV_pin1, ON);
			// delay(500);
			// digitalWrite(_REV_pin1, OFF);
			// digitalWrite(_REV_pin2, OFF);
			// digitalWrite(_FWD_pin1, OFF);
			// digitalWrite(_FWD_pin2, OFF);
			// pcf38.write(6, OFF);
            // pcf38.write(4, ON);
			// pcf38.write(_REV_pin2, OFF);			
			delay(500);
		}
		void testPin(){
			pcf38.write(_FWD_pin1, ON);
			pcf38.write(_REV_pin1, ON);
			// dacWrite(_speed_setting_pin1, 0);

			pcf38.write(_FWD_pin2, ON);
			digitalWrite(_REV_pin2, 1);			
			// dacWrite(_speed_setting_pin2, 0);
		}		
        /**
        * @brief          : đọc chân alarm báo lỗi.
        * @details        : 0: ko lỗi | 1: động cơ trái lỗi | 2: động cơ phải lỗi | 3: cả 2 động cơ lỗi
        * @param[in]      : 
        * @param[out]     :
        */
		bool read_alarm1(){
			if (digitalRead(_alarm_in_pin1) == 0){
				return 0;
			}else{
				return 1;
			}
		}
		bool read_alarm2(){
			if (digitalRead(_alarm_in_pin2) == 0){
				return 0;
			}else{
				return 1;
			}
		}

        /**
        * @brief          : dieu khien motor
        * @details        : chay bang chieu quay va toc do
        * @param[in]      : analog (gia tri toc do motor)
		*/
        void spin(int analog1, int analog2) {
            // chon CHIEU quay motor khi chay PID 
		  // Motor2
		  	if (analog2 == 0){
				dacWrite(_speed_setting_pin2, 0);
				pcf38.write(_FWD_pin2, OFF);
				digitalWrite(_REV_pin2, OFF);
				last_analog2 = analog2; // add
            }
			else {
				if (analog2 > 0){
					if (dir2) {
						pcf38.write(_FWD_pin2, ON);
						// pcf38.write(_REV_pin2, OFF);
						digitalWrite(_REV_pin2, OFF);
					}
					else{
						pcf38.write(_FWD_pin2, OFF);
						// pcf38.write(_REV_pin2, ON);
						digitalWrite(_REV_pin2, ON);
					}					
				}
				else if (analog2 < 0){
					if (dir2) {
						pcf38.write(_FWD_pin2, OFF);
						// pcf38.write(_REV_pin2, ON);
						digitalWrite(_REV_pin2, ON);				
					}
					else{
						pcf38.write(_FWD_pin2, ON);
						// pcf38.write(_REV_pin2, OFF);
						digitalWrite(_REV_pin2, OFF);
					}					
				}

				if (last_analog2 != analog2) {
					dacWrite(_speed_setting_pin2, abs(analog2));
					last_analog2 = analog2; 
				}				
			}
		  // Motor1
		  	if (analog1 == 0){
				dacWrite(_speed_setting_pin1, 0);				  
				pcf38.write(_FWD_pin1, OFF);
				pcf38.write(_REV_pin1, OFF);
				last_analog1 = analog1;
            }
			else {
				if (analog1 > 0){
					if (dir1) {
						pcf38.write(_FWD_pin1, ON);
						pcf38.write(_REV_pin1, OFF);
					}
					else{
						pcf38.write(_FWD_pin1, OFF);
						pcf38.write(_REV_pin1, ON);
					}
				}
				else if (analog1 < 0){
					if (dir1) {
						pcf38.write(_FWD_pin1, OFF);
						pcf38.write(_REV_pin1, ON);					
					}
					else{
						pcf38.write(_FWD_pin1, ON);
						pcf38.write(_REV_pin1, OFF);
					}
				}

				if (last_analog1 != analog1) {
					dacWrite(_speed_setting_pin1, abs(analog1));
					last_analog1 = analog1;
				}

			}			
        }
        /**
        * @brief          : dao chieu dieu khien dong co
        * @details        : vi hai banh dat doi dien nhau, de cho AGV di dung thi can dieu khien nguoc chieu
        * @requirements   : dao 1 lan trong khi define ham
        */
        void Revert1(void) {
            dir1 = !dir1;
        }
        void Revert2(void) {
            dir2 = !dir2;
        }

		void stopAll(){
			pcf38.write(_FWD_pin1, ON);
			pcf38.write(_REV_pin1, ON);

			pcf38.write(_FWD_pin2, ON);
			digitalWrite(_REV_pin2, 1);

		}
};
#endif		
// 100W
    // private:
	//   // -- I/O
    //     // Enb
    //     int _enable_pin;
    //     // phanh
    //     int _brake_pin;
    //     // reset canh bao
    //     int _alarm_reset_pin;	
	//   // -- PCF8574
    //     // chieu 		
    //     int _rotation_pin1;
	// 	int _rotation_pin2;
    //     // cai dat toc do
    //     int _speed_setting_pin1;
	// 	int _speed_setting_pin2;
    //     // canh bao
    //     int _alarm_in_pin1;
	// 	int _alarm_in_pin2;
    //   // 
    //     // chieu dieu khien (toan cuc)
    //     int dir1;
	// 	int dir2;
    //     // so sanh tin hieu dieu khien
    //     int last_analog1;
	// 	int last_analog2;
	// 	bool ON = false;
	// 	bool OFF = true;
    // public:
    //     STI_motor_driver(int enable_pin, int brake_pin, int alarm_reset_pin, int rotation_pin1, int rotation_pin2 
	// 		, int speed_setting_pin1, int speed_setting_pin2, int alarm_in_pin1, int alarm_in_pin2) {
	// 	// Output
    //         _enable_pin = enable_pin;
    //         _brake_pin = brake_pin;
	// 		_alarm_reset_pin = alarm_reset_pin;

    //         _rotation_pin1 = rotation_pin1;
	// 		_rotation_pin2 = rotation_pin2;

    //         _speed_setting_pin1 = speed_setting_pin1;
	// 		_speed_setting_pin2 = speed_setting_pin2;

    //     // Input    
    //         _alarm_in_pin1 = alarm_in_pin1;
	// 		_alarm_in_pin2 = alarm_in_pin2;
    //     }
    //     ~STI_motor_driver() {}

    //     /**
    //     * @brief          : setup chan ket noi
    //     * @details        : cai dat pinmode va cac gia tri ban dau
    //     * @requirements   : cac gia tri chan duoc define trong file config
    //     */
    //     void driver_pin_init() {
	// 		pcf38.begin();

	// 		pinMode(_speed_setting_pin1, OUTPUT);
    //         pinMode(_speed_setting_pin2, OUTPUT);

	// 		pinMode(_alarm_in_pin1, INPUT);
	// 		pinMode(_alarm_in_pin2, INPUT);

	// 		reset_alarm();
			
	// 		dacWrite(_speed_setting_pin1, 0);
	// 		dacWrite(_speed_setting_pin2, 0);			
    //         //ensure that the motor is in neutral state during bootup
    //         pcf38.write(_enable_pin, OFF); // enb
    //         pcf38.write(_brake_pin, OFF);   // RUN
            
    //         dir1 = 0;
	// 		dir2 = 0;
    //         last_analog1 = 0;
	// 		last_analog2 = 0;
    //     }
    //     /**
    //     * @brief          : đọc chân alarm báo lỗi.
    //     * @details        : 0: ko lỗi | 1: động cơ trái lỗi | 2: động cơ phải lỗi | 3: cả 2 động cơ lỗi
    //     * @param[in]      : 
    //     * @param[out]     :
    //     */
	// 	bool read_alarm1(){
	// 		if (digitalRead(_alarm_in_pin1) == 0){
	// 			return 0;
	// 		}else{
	// 			return 1;
	// 		}
	// 	}
	// 	bool read_alarm2(){
	// 		if (digitalRead(_alarm_in_pin2) == 0){
	// 			return 0;
	// 		}else{
	// 			return 1;
	// 		}
	// 	}		
	// 	void control_brake(bool b){
	// 		pcf38.write(_brake_pin, b);
	// 	}
	// 	void control_enable(bool b){
	// 		pcf38.write(_enable_pin, b);
	// 	}		
	// 	void reset_alarm(){
	// 		pcf38.write(_alarm_reset_pin, ON); // Reset
	// 		delay(20);
	// 		pcf38.write(_alarm_reset_pin, OFF);
	// 		delay(20);
	// 		pcf38.write(_alarm_reset_pin, ON); // Reset
	// 	}

    //     /**
    //     * @brief          : dieu khien motor
    //     * @details        : chay bang chieu quay va toc do
    //     * @param[in]      : analog (gia tri toc do motor)
	// 	*/
    //     void spin(int analog1, int analog2) {
    //         // chon CHIEU quay motor khi chay PID 
	// 	  // Motor1
    //         if (analog1 >= 0){
    //             if (dir1) {
	// 				pcf38.write(_rotation_pin1, false);
    //             }
    //             else{
	// 				pcf38.write(_rotation_pin1, true);
    //             }
    //         }
    //         else if (analog1 < 0){
    //             if (dir1) {
	// 				pcf38.write(_rotation_pin1, true);
    //             }
    //             else{
	// 				pcf38.write(_rotation_pin1, false);
    //             }
    //         }

	// 	  // Motor2
    //         if (analog2 >= 0){
    //             if (dir2) {
	// 				pcf38.write(_rotation_pin2, false);
    //             }
    //             else{
	// 				pcf38.write(_rotation_pin2, true);
    //             }
    //         }
    //         else if (analog2 < 0){
    //             if (dir2) {
	// 				pcf38.write(_rotation_pin2, true);
    //             }
	// 			else{
	// 				pcf38.write(_rotation_pin2, false);
    //             }
    //         }

	// 	// -- Speed
	// 		if (analog1 == 0 && analog2 == 0){
	// 			pcf38.write(_enable_pin, OFF); // Not enable
	// 			dacWrite(_speed_setting_pin1, 0);
	// 			dacWrite(_speed_setting_pin2, 0);
	// 		}else{
	// 			pcf38.write(_enable_pin, ON); // Enable
	// 			if (last_analog1 != analog1) {
	// 				dacWrite(_speed_setting_pin1, abs(analog1));
	// 				last_analog1 = analog1;
	// 			}
	// 			if (last_analog2 != analog2) {
	// 				dacWrite(_speed_setting_pin2, abs(analog2));
	// 				last_analog2 = analog2;
	// 			}
    //         }
    //     }
    //     /**
    //     * @brief          : dao chieu dieu khien dong co
    //     * @details        : vi hai banh dat doi dien nhau, de cho AGV di dung thi can dieu khien nguoc chieu
    //     * @requirements   : dao 1 lan trong khi define ham
    //     */
    //     void Revert1(void) {
    //         dir1 = !dir1;
    //     }
    //     void Revert2(void) {
    //         dir2 = !dir2;
    //     }

// };
// #endif