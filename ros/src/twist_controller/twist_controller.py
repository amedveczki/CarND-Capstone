from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    ### DBWVideo {
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, 
            wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.yaw_controller = YawController(wheel_base, steer_ratio,
                0.1, # min speed [m/s]
                max_lat_accel,
                max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0. # min throttle value
        mx = 0.2 # max throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # 1/(2pi*tau) = cutoff freq
        ts = .02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()
    ### DBWVideo }


    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        ### DBWVideo {
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        ## rospy.logwarn("angular_vel       : {0}".format(angular_vel       ))        
        ## rospy.logwarn("linear_vel        : {0}".format(linear_vel        ))        
        ## rospy.logwarn("angular_vel       : {0}".format(angular_vel       ))        
        ## rospy.logwarn("current_vel       : {0}".format(current_vel       ))        
        ## rospy.logwarn("self.vel_lpf.get(): {0}".format(self.vel_lpf.get()))        

        steering = self.yaw_controller.get_steering
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time = self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 700 # N*M

        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Nm
        
        # TODO amedveczki - 1 bug which can be optionally fixed: wandering a little bit in the lane.
        # autoware code doesn't recompute the trajectory until certain distance waypoint/angle was passed
        # 11:00 in video - waypoint_follower.cpp -> update all the time, func which checks if waypoints are being followed -> just follow all the time

        return throttle, brake, steering

        ### DBWVideo }
        
