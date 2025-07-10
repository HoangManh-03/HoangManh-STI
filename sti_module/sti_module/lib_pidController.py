import time
import rclpy
from geometry_msgs.msg import Twist

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class LineFollowerPID:
    def __init__(self, pub_vel):
        self.pub_vel = pub_vel

        self.setpoint = 7.5
        self.move_byHead = 1
        self.move_byTail = 2

        # -- Param control
        self.enable = 1
        self.direction_move = 0
        self.velocity = 0.0
        self.safety = 1

        self.save_enable = -1
        # -- Param status
        self.status = 0
        self.warn = 0
        self.error = 0

        # -- param pid
        self.Kp = 0.
        self.Ki = 0.
        self.Kd = 0.

        self.previous_error = 0.0
        self.integral = 0.0

        self.previous_error_2 = 0.0
        self.integral_2 = 0.0

        self.last_time = time.time()

        self.list_pid_vel = [(0.05, 0.019278874536821953, 0.000005, 0.0075),
                            (0.1, 0.021278874536821953, 0.00001, 0.0089), 
                            (0.2, 0.02326770784939, 0.0000103, 0.013), 
                            (0.3, 0.02570784939, 0.0000105, 0.0152), 
                            (0.4, 0.025870784939, 0.0000108, 0.0173), 
                            (0.5, 0.0259784939, 0.00001084, 0.020), 
                            (0.6, 0.0260246770784939, 0.00001088, 0.025)]

        # -- param control
        self.savetime1 = time.time()
        self.recieve_dataLine = 0

        # -- param velocity
        self.isDecelerationObstacles = 0
        self.saveVelWhenDecObs = 0.
        self.statusVel = 0
        self.curr_velocity = 0.
        self.velSt = 0.
        
        # -- 
        self.vel_in_zone3 = 0.2
        self.vel_in_zone2 = 0.1
        self.vel_min = 0.1

    def update_pid_parameters(self, linear_velocity):
        vel = linear_velocity
        if vel <= self.list_pid_vel[0][0]:
            vel = self.list_pid_vel[0][0]

        if vel >= self.list_pid_vel[-1][0]:
            vel = self.list_pid_vel[-1][0]

        for i in range(len(self.list_pid_vel) - 1):
            if self.list_pid_vel[i][0] <= vel < self.list_pid_vel[i + 1][0]:
                # Nội suy tuyến tính
                v0, Kp0, Ki0, Kd0 = self.list_pid_vel[i]
                v1, Kp1, Ki1, Kd1 = self.list_pid_vel[i + 1]
                t = (vel - v0) / (v1 - v0)
                Kp = Kp0 + t * (Kp1 - Kp0)
                Ki = Ki0 + t * (Ki1 - Ki0)
                Kd = Kd0 + t * (Kd1 - Kd0)

                return Kp, Ki, Kd
            
        return self.list_pid_vel[0][1], self.list_pid_vel[0][2], self.list_pid_vel[0][3]

    def calculate_value(self, _velocity, dir_move, data_magline):
        vel = 0.
        status = 0

        # update pid param
        self.Kp, self.Ki, self.kd = self.update_pid_parameters(_velocity)
        # print("Kp = %s, Ki = %s, Kd = %s" %(self.Kp, self.Ki, self.kd))
        # self.Ki = 0.
        value_line = data_magline
        
        if value_line == 20:
            status = 1
            vel = 0.

        elif value_line > -1 and value_line < 16:
            status = 1
            self.recieve_dataLine = 1
            self.savetime1 = time.time()

            current_time = time.time()
            dt = current_time - self.last_time
            error = self.setpoint - value_line

            self.integral += error * dt
            derivative = (error - self.previous_error) / dt

            print("error: %s, integral: %s" %(error, self.integral))

            vel = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

            self.previous_error = error
            self.last_time = current_time
        
        else:
            if self.recieve_dataLine:
                denta_t = time.time() - self.savetime1
                if denta_t < 0.5:
                    status = 2

        return status, vel
    
    def coefficientPID_t2(self, data_mangline):
        vel = 0.
        status = 0

        value_line = data_mangline
        value_line_s = -1

        self.Kp, self.Ki, self.Kd = 0.025, 0.0, 0.005

        error = -1.
        if value_line == 20:
            status = 1
            vel = 0.

        elif value_line > -1 and value_line < 16:
            status = 1
            self.recieve_dataLine = 1
            self.savetime1 = time.time()

            current_time = time.time()
            dt = current_time - self.last_time
            error = self.setpoint - value_line

            self.integral += error * dt
            # -- Gioi han
            if self.integral > 10.:
                self.integral = 10.
            elif self.integral < -10.:
                self.integral = -10.
                
            derivative = (error - self.previous_error) / dt

            print("error: %s, integral: %s, Kp: %s, Ki: %s, kd: %s" %(error, self.integral, self.Kp, self.Ki, self.Kd))

            _kp = 0.001
            vel = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

            self.previous_error = error

            # 🟠 Vòng trong: Điều chỉnh w để đạt w_desired
            kp = 1.12
            ki = 0.0
            kd = 0.001
            error_inner = vel
            self.integral_2 += error_inner * dt
            derivative_inner = (error_inner - self.previous_error_2) / dt

            w = (kp * error_inner +
                ki * self.integral_2 +
                kd * derivative_inner)

            self.previous_error_2 = error_inner
            
            self.last_time = current_time

            vel = w
        
        else:
            if self.recieve_dataLine:
                denta_t = time.time() - self.savetime1
                if denta_t < 0.5:
                    status = 2

        return status, vel, error

    def reset_pid(self):
        self.previous_error = 0.0
        self.integral = 0.0

        self.previous_error_2 = 0.0
        self.integral_2 = 0.0

        self.last_time = time.time()

        self.recieve_dataLine = 0
        self.savetime1 = time.time()

    def stop(self):
        # reset van toc
        self.curr_velocity = 0.
        self.saveTimeVel = time.time()
        self.velSt = 0.
        self.statusVel = 0
        self.isDecelerationObstacles = 0
        self.saveVelWhenDecObs = 0.

        for i in range(3):
            self.pub_vel.publish(Twist())

    def funcDecelerationByAcc(self, time_s, v_s, v_f, a):
        denlta_time_now = time.time()- time_s
        v_re = v_s + a*denlta_time_now
        if a > 0.:
            if v_re >= v_f:
                v_re = v_f
        else:
            if v_re <= v_f:
                v_re = v_f

        return v_re

    def getVeloctity(self, velCmd, _statusVel):
        if _statusVel == 0:
            return velCmd
        elif _statusVel == 1:
            return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, velCmd, 0.1)
        elif _statusVel == 2:
            return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, velCmd, -2.5)
        else:
            return 0.

    def run(self, data_mangline_front, data_mangline_behind, zone_lidar):
        warn = 0
        error = 0

        if self.enable != 2:
            if self.save_enable != self.enable or self.enable == 0:
                print("STOP!")
                self.stop()
                self.save_enable = self.enable

            self.reset_pid()

        else:
            self.save_enable = self.enable
            # -- safety
            safety = 0
            value_magline = -1
            if self.direction_move == self.move_byHead:
                value_magline = data_mangline_front.value
                safety = zone_lidar.zone_sick_ahead if self.safety else 0
            elif self.direction_move == self.move_byTail:
                value_magline = data_mangline_behind.value
                safety = zone_lidar.zone_sick_behind if self.safety else 0

            velocity = self.velocity
            velCmd = 0. 
            if safety == 1:
                velCmd = 0.
                warn = 1
                print('not safety', velCmd)
                
            elif safety == 2:
                if self.curr_velocity == 0. or self.curr_velocity >= self.vel_in_zone2:
                    velCmd = self.vel_in_zone2
                else:
                    velCmd = self.curr_velocity

                if velCmd <= self.vel_min:
                    velCmd = self.vel_min

                print('in safety zone 2!', velCmd)

            elif safety == 3:
                if self.curr_velocity == 0. or self.curr_velocity >= self.vel_in_zone3:
                    velCmd = self.vel_in_zone3
                else:
                    velCmd = self.curr_velocity

                if velCmd <= self.vel_min:
                    velCmd = self.vel_min

                print('in safety zone 3!', velCmd)

            else:
                velCmd = velocity

            if velCmd == 0.:
                self.stop()
                self.reset_pid()

            else:
                if self.curr_velocity < velCmd and self.statusVel != 1:
                    self.statusVel = 1
                    self.velSt = self.curr_velocity
                    self.saveTimeVel = time.time()

                elif self.curr_velocity > velCmd and self.statusVel != 2:
                    self.statusVel = 2
                    self.velSt = self.curr_velocity
                    self.saveTimeVel = time.time()

                elif self.curr_velocity == velCmd:
                    self.statusVel = 0

                self.curr_velocity = self.getVeloctity(velCmd, self.statusVel)

                try:
                    status, vel, error_ = self.coefficientPID_t2(value_magline)

                    if status == 0:
                        error = 1
                        self.stop()
                        self.reset_pid()
                        print("agv lech khoi line tu")

                    elif status == 1:
                        twist = Twist()
                        vel_linear = self.curr_velocity
                        # vel_linear = vel_x
                        if self.direction_move == self.move_byTail:
                            vel_linear = -1 * vel_linear

                        twist.linear.x = vel_linear
                        twist.angular.z = vel

                        self.pub_vel.publish(twist)

                        print("linear_x: %s, angular_z: %s" %(vel_linear, vel))

                except Exception as e:
                    print("ERROR! ", e)

        self.warn = warn
        self.error = error

class LineFollowerFUZZYPID:
    def __init__(self, pub_vel):
        self.pub_vel = pub_vel

        # -- Param regulations
        self.setpoint = 7.5
        self.move_byHead = 1
        self.move_byTail = 2

        # -- Param control
        self.enable = 1
        self.direction_move = 0
        self.velocity = 0.0
        self.safety = 1

        # -- Param status
        self.save_enable = -1
        self.status = 0
        self.warn = 0
        self.error = 0

        # -- param pid
        self.Kp = 0.
        self.Ki = 0.
        self.Kd = 0.

        self.previous_error = 0.0
        self.integral = 0.0

        self.previous_error_2 = 0.0
        self.integral_2 = 0.0

        # -- param Fuzzy 
        # Input
        self.error_control = ctrl.Antecedent(np.arange(-7.5, 7.5, 0.5), 'error')
        self.velocity = ctrl.Antecedent(np.arange(0, 0.7, 0.02), 'velocity')

        # Output 
        self.kp = ctrl.Consequent(np.arange(0.014, 0.028, 0.002), 'kp')
        # self.ki = ctrl.Consequent(np.arange(0, 0.000005, 0.0000002), 'ki')
        self.kd = ctrl.Consequent(np.arange(0.008, 0.015, 0.0002), 'kd')

        # Fuzzy set Error
        self.error_control['N'] = fuzz.trimf(self.error_control.universe, [-7.5, -3.5, 0.])  # Negative
        self.error_control['Z']  = fuzz.trimf(self.error_control.universe, [-0.6, 0, 0.6])       # Zero
        self.error_control['P'] = fuzz.trimf(self.error_control.universe, [0, 3.5, 7.5])      # Positive

        # Fuzzy set Velocity
        self.velocity['Low']  = fuzz.trimf(self.velocity.universe, [0, 0.15, 0.3])
        self.velocity['Medium'] = fuzz.trimf(self.velocity.universe, [0.25, 0.4, 0.55])
        self.velocity['High']  = fuzz.trimf(self.velocity.universe, [0.5, 0.6, 0.7])

        # Fuzzy set Kp
        self.kp['S']  = fuzz.trimf(self.kp.universe, [0.014, 0.018, 0.02])   # Small
        self.kp['M']  = fuzz.trimf(self.kp.universe, [0.019, 0.021, 0.023]) # Medium
        self.kp['L']  = fuzz.trimf(self.kp.universe, [0.022, 0.023, 0.025])   # Large

        # Fuzzy set Ki
        # self.ki['S']  = fuzz.trimf(self.ki.universe, [0, 0.2, 0.4])
        # self.ki['M']  = fuzz.trimf(self.ki.universe, [0.2, 0.6, 1])
        # self.ki['L']  = fuzz.trimf(self.ki.universe, [0.6, 1, 1])

        # Fuzzy set Kd
        self.kd['S']  = fuzz.trimf(self.kd.universe, [0.008, 0.0085, 0.0093])   # Small
        self.kd['M']  = fuzz.trimf(self.kd.universe, [0.009, 0.01, 0.013]) # Medium
        self.kd['L']  = fuzz.trimf(self.kd.universe, [0.013, 0.015, 0.015]) # Large

        # Fuzzy rule
        rules = [
            ctrl.Rule(self.error_control['N'] & self.velocity['Low'], (self.kp['M'], self.kd['S'])),
            ctrl.Rule(self.error_control['N'] & self.velocity['Medium'], (self.kp['M'], self.kd['M'])),
            ctrl.Rule(self.error_control['N'] & self.velocity['High'], (self.kp['L'], self.kd['L'])),

            ctrl.Rule(self.error_control['Z'] & self.velocity['Low'], (self.kp['M'], self.kd['S'])),
            ctrl.Rule(self.error_control['Z'] & self.velocity['Medium'], (self.kp['M'], self.kd['M'])),
            ctrl.Rule(self.error_control['Z'] & self.velocity['High'], (self.kp['S'], self.kd['M'])),

            ctrl.Rule(self.error_control['P'] & self.velocity['Low'], (self.kp['M'], self.kd['S'])),
            ctrl.Rule(self.error_control['P'] & self.velocity['Medium'], (self.kp['M'], self.kd['M'])),
            ctrl.Rule(self.error_control['P'] & self.velocity['High'], (self.kp['L'], self.kd['L'])),
        ]

        # Create control system
        self.pid_ctrl = ctrl.ControlSystem(rules)
        self.pid = ctrl.ControlSystemSimulation(self.pid_ctrl)

        self.last_time = time.time()

        self.list_pid_vel = [(0.05, 0.019278874536821953, 0.000005, 0.0075),
                            (0.1, 0.021278874536821953, 0.00001, 0.0089), 
                            (0.2, 0.02326770784939, 0.0000103, 0.013), 
                            (0.3, 0.02570784939, 0.0000105, 0.0152), 
                            (0.4, 0.025870784939, 0.0000108, 0.0173), 
                            (0.5, 0.0259784939, 0.00001084, 0.020), 
                            (0.6, 0.0260246770784939, 0.00001088, 0.025)]

        # -- param control
        self.savetime1 = time.time()
        self.recieve_dataLine = 0

        # -- param velocity
        self.isDecelerationObstacles = 0
        self.saveVelWhenDecObs = 0.
        self.statusVel = 0
        self.curr_velocity = 0.
        self.velSt = 0.
        
        # -- 
        self.vel_in_zone3 = 0.2
        self.vel_in_zone2 = 0.1
        self.vel_min = 0.1

    def update_pid_parameters(self, linear_velocity):
        vel = linear_velocity
        if vel <= self.list_pid_vel[0][0]:
            vel = self.list_pid_vel[0][0]

        if vel >= self.list_pid_vel[-1][0]:
            vel = self.list_pid_vel[-1][0]

        for i in range(len(self.list_pid_vel) - 1):
            if self.list_pid_vel[i][0] <= vel < self.list_pid_vel[i + 1][0]:
                # Nội suy tuyến tính
                v0, Kp0, Ki0, Kd0 = self.list_pid_vel[i]
                v1, Kp1, Ki1, Kd1 = self.list_pid_vel[i + 1]
                t = (vel - v0) / (v1 - v0)
                Kp = Kp0 + t * (Kp1 - Kp0)
                Ki = Ki0 + t * (Ki1 - Ki0)
                Kd = Kd0 + t * (Kd1 - Kd0)

                return Kp, Ki, Kd
            
        return self.list_pid_vel[0][1], self.list_pid_vel[0][2], self.list_pid_vel[0][3]

    def calculate_value(self, data_magline, velocity):
        vel = 0.
        status = 0
        value_line = data_magline
        
        error = -1.
        if value_line == 20:
            status = 1
            vel = 0.

        elif value_line > -1 and value_line < 16:
            status = 1
            self.recieve_dataLine = 1
            self.savetime1 = time.time()

            current_time = time.time()
            dt = current_time - self.last_time
            # print("dt: ", dt)
            error = self.setpoint - value_line

            # Cập nhật Fuzzy Logic
            self.pid.input['error'] = error
            self.pid.input['velocity'] = velocity
            self.pid.compute()

            # Lấy giá trị Kp, Ki, Kd từ Fuzzy
            kp_value = self.pid.output['kp']
            ki_value = 0.002
            kd_value = self.pid.output['kd']

            print("Kp = %s, Ki = %s, Kd = %s" %(kp_value, 0.0, kd_value))

            self.integral += error * dt
            # -- Gioi han
            if self.integral > 10.:
                self.integral = 10.
            elif self.integral < -10.:
                self.integral = -10.
                
            derivative = (error - self.previous_error) / dt

            print("error: %s, integral: %s" %(error, self.integral))

            vel = kp_value * error + ki_value * self.integral + kd_value * derivative

            self.previous_error = error

            kp = 1.05
            ki = 0.0
            kd = 0.001
            error_inner = vel
            self.integral_2 += error_inner * dt
            derivative_inner = (error_inner - self.previous_error_2) / dt

            w = (kp * error_inner +
                ki * self.integral_2 +
                kd * derivative_inner)

            self.previous_error_2 = error_inner
            
            self.last_time = current_time

            vel = w

            self.last_time = current_time
        
        else:
            if self.recieve_dataLine:
                denta_t = time.time() - self.savetime1
                if denta_t < 0.5:
                    status = 2

        return status, vel, error
    
    def coefficientPID_t2(self, data_mangline, velocity):
        vel = 0.
        status = 0

        value_line = data_mangline
        value_line_s = -1

        self.Kp, self.Ki, self.Kd = 0.025, 0.0, 0.005

        error = -1.
        if value_line == 20:
            status = 1
            vel = 0.

        elif value_line > -1 and value_line < 16:
            status = 1
            self.recieve_dataLine = 1
            self.savetime1 = time.time()

            current_time = time.time()
            dt = current_time - self.last_time
            error = self.setpoint - value_line

            self.integral += error * dt
            # -- Gioi han
            if self.integral > 10.:
                self.integral = 10.
            elif self.integral < -10.:
                self.integral = -10.
                
            derivative = (error - self.previous_error) / dt

            print("error: %s, integral: %s, Kp: %s, Ki: %s, kd: %s" %(error, self.integral, self.Kp, self.Ki, self.Kd))

            _kp = 0.001
            vel = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

            self.previous_error = error

            kp = 1.04
            ki = 0.0
            kd = 0.001
            error_inner = vel
            self.integral_2 += error_inner * dt
            derivative_inner = (error_inner - self.previous_error_2) / dt

            w = (kp * error_inner +
                ki * self.integral_2 +
                kd * derivative_inner)

            self.previous_error_2 = error_inner
            
            self.last_time = current_time

            vel = w
        
        else:
            if self.recieve_dataLine:
                denta_t = time.time() - self.savetime1
                if denta_t < 0.5:
                    status = 2

        return status, vel, error

    def reset_pid(self):
        self.previous_error = 0.0
        self.integral = 0.0

        self.previous_error_2 = 0.0
        self.integral_2 = 0.0

        self.last_time = time.time()

        self.recieve_dataLine = 0
        self.savetime1 = time.time()

    def stop(self):
        # reset van toc
        self.curr_velocity = 0.
        self.saveTimeVel = time.time()
        self.velSt = 0.
        self.statusVel = 0
        self.isDecelerationObstacles = 0
        self.saveVelWhenDecObs = 0.

        for i in range(3):
            self.pub_vel.publish(Twist())

    def funcDecelerationByAcc(self, time_s, v_s, v_f, a):
        denlta_time_now = time.time()- time_s
        v_re = v_s + a*denlta_time_now
        if a > 0.:
            if v_re >= v_f:
                v_re = v_f
        else:
            if v_re <= v_f:
                v_re = v_f

        return v_re

    def getVeloctity(self, velCmd, _statusVel):
        if _statusVel == 0:
            return velCmd
        elif _statusVel == 1:
            return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, velCmd, 0.1)
        elif _statusVel == 2:
            return self.funcDecelerationByAcc(self.saveTimeVel, self.velSt, velCmd, -2.5)
        else:
            return 0.

    def run(self, data_mangline_front, data_mangline_behind, zone_lidar):
        warn = 0
        error = 0

        if self.enable != 2:
            if self.save_enable != self.enable or self.enable == 0:
                print("STOP!")
                self.stop()
                self.save_enable = self.enable

            self.reset_pid()

        else:
            self.save_enable = self.enable
            # -- safety
            safety = 0
            value_magline = -1
            if self.direction_move == self.move_byHead:
                value_magline = data_mangline_front.value
                safety = zone_lidar.zone_sick_ahead if self.safety else 0
            elif self.direction_move == self.move_byTail:
                value_magline = data_mangline_behind.value
                safety = zone_lidar.zone_sick_behind if self.safety else 0

            velocity = self.velocity
            velCmd = 0. 
            if safety == 1:
                velCmd = 0.
                warn = 1
                print('not safety', velCmd)
                
            elif safety == 2:
                if self.curr_velocity == 0. or self.curr_velocity >= self.vel_in_zone2:
                    velCmd = self.vel_in_zone2
                else:
                    velCmd = self.curr_velocity

                if velCmd <= self.vel_min:
                    velCmd = self.vel_min

                print('in safety zone 2!', velCmd)

            elif safety == 3:
                if self.curr_velocity == 0. or self.curr_velocity >= self.vel_in_zone3:
                    velCmd = self.vel_in_zone3
                else:
                    velCmd = self.curr_velocity

                if velCmd <= self.vel_min:
                    velCmd = self.vel_min

                print('in safety zone 3!', velCmd)

            else:
                velCmd = velocity

            if velCmd == 0.:
                self.stop()
                self.reset_pid()

            else:
                if self.curr_velocity < velCmd and self.statusVel != 1:
                    self.statusVel = 1
                    self.velSt = self.curr_velocity
                    self.saveTimeVel = time.time()

                elif self.curr_velocity > velCmd and self.statusVel != 2:
                    self.statusVel = 2
                    self.velSt = self.curr_velocity
                    self.saveTimeVel = time.time()

                elif self.curr_velocity == velCmd:
                    self.statusVel = 0

                self.curr_velocity = self.getVeloctity(velCmd, self.statusVel)

                try:
                    status, vel, error_ = self.calculate_value(value_magline, self.curr_velocity)

                    if status == 0:
                        error = 1
                        self.stop()
                        self.reset_pid()
                        print("agv lech khoi line tu")

                    elif status == 1:
                        twist = Twist()
                        vel_linear = self.curr_velocity
                        # vel_linear = vel_x
                        if self.direction_move == self.move_byTail:
                            vel_linear = -1 * vel_linear

                        twist.linear.x = vel_linear
                        twist.angular.z = vel

                        self.pub_vel.publish(twist)

                        print("linear_x: %s, angular_z: %s" %(vel_linear, vel))

                except Exception as e:
                    print("ERROR! ", e)

        self.warn = warn
        self.error = error