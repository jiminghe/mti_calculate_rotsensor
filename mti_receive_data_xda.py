#  Copyright (c) 2003-2024 Movella Technologies B.V. or subsidiaries worldwide.
#  All rights reserved.

import sys
import xsensdeviceapi as xda
from threading import Lock
import math

class XdaCallback(xda.XsCallback):
    def __init__(self, max_buffer_size = 5):
        xda.XsCallback.__init__(self)
        self.m_maxNumberOfPacketsInBuffer = max_buffer_size
        self.m_packetBuffer = list()
        self.m_lock = Lock()

    def packetAvailable(self):
        self.m_lock.acquire()
        res = len(self.m_packetBuffer) > 0
        self.m_lock.release()
        return res

    def getNextPacket(self):
        self.m_lock.acquire()
        assert(len(self.m_packetBuffer) > 0)
        oldest_packet = xda.XsDataPacket(self.m_packetBuffer.pop(0))
        self.m_lock.release()
        return oldest_packet

    def onLiveDataAvailable(self, dev, packet):
        self.m_lock.acquire()
        assert(packet != 0)
        while len(self.m_packetBuffer) >= self.m_maxNumberOfPacketsInBuffer:
            self.m_packetBuffer.pop()
        self.m_packetBuffer.append(xda.XsDataPacket(packet))
        self.m_lock.release()


class OrientationCorrector:
    def __init__(self):
        self.initialized = False
        self.q_rotSensor = None
    
    def initialize_from_first_frame(self, q_raw):
        """
        Initialize the rotation correction based on the first frame.
        This method creates a correction that will zero out roll, pitch, and yaw.
        """
        if self.initialized:
            return
        
        # Extract the raw quaternion components
        q0, q1, q2, q3 = q_raw[0], q_raw[1], q_raw[2], q_raw[3]
        
        # Convert to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler([q0, q1, q2, q3])
        print(f"Initial orientation - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
        
        # Convert angles to radians
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        
        # Create quaternion for roll (around X)
        cr = math.cos(roll_rad/2)
        sr = math.sin(roll_rad/2)
        q_roll = [cr, sr, 0, 0]
        
        # Create quaternion for pitch (around Y)
        cp = math.cos(pitch_rad/2)
        sp = math.sin(pitch_rad/2)
        q_pitch = [cp, 0, sp, 0]
        
        # Create quaternion for yaw (around Z)
        cy = math.cos(yaw_rad/2)
        sy = math.sin(yaw_rad/2)
        q_yaw = [cy, 0, 0, sy]
        
        # Combine all rotations (in ZYX order)
        q_full = self.quaternion_multiply(q_yaw, self.quaternion_multiply(q_pitch, q_roll))
        
        # Inverse (conjugate) to get the correction
        self.q_rotSensor = [q_full[0], -q_full[1], -q_full[2], -q_full[3]]
        
        self.initialized = True
        print("Orientation correction initialized with:")
        print(f"q_rotSensor: {self.q_rotSensor}")
    
    def quaternion_multiply(self, q1, q2):
        """
        Multiply two quaternions q1 and q2
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return [w, x, y, z]
    
    def correct_orientation(self, q_raw):
        """
        Apply the correction: q_corrected = q_raw * q_rotSensor
        """
        if not self.initialized:
            self.initialize_from_first_frame(q_raw)
            return q_raw  # Return raw quaternion for the first frame
        
        # Perform: q_raw * q_rotSensor
        q_corrected = self.quaternion_multiply(list(q_raw), self.q_rotSensor)
        
        return q_corrected
    
    def quaternion_to_euler(self, q):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw) in degrees
        """
        # Extract quaternion components
        w, x, y, z = q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        return roll_deg, pitch_deg, yaw_deg


if __name__ == '__main__':
    print("Creating XsControl object...")
    control = xda.XsControl_construct()
    assert(control != 0)

    xdaVersion = xda.XsVersion()
    xda.xdaVersion(xdaVersion)
    print("Using XDA version %s" % xdaVersion.toXsString())

    # Create our orientation corrector
    orientation_corrector = OrientationCorrector()

    try:
        print("Scanning for devices...")
        portInfoArray =  xda.XsScanner_scanPorts()

        # Find an MTi device
        mtPort = xda.XsPortInfo()
        for i in range(portInfoArray.size()):
            if portInfoArray[i].deviceId().isMti() or portInfoArray[i].deviceId().isMtig():
                mtPort = portInfoArray[i]
                break

        if mtPort.empty():
            raise RuntimeError("No MTi device found. Aborting.")

        did = mtPort.deviceId()
        print("Found a device with:")
        print(" Device ID: %s" % did.toXsString())
        print(" Port name: %s" % mtPort.portName())

        print("Opening port...")
        if not control.openPort(mtPort.portName(), mtPort.baudrate()):
            raise RuntimeError("Could not open port. Aborting.")

        # Get the device object
        device = control.device(did)
        assert(device != 0)

        print("Device: %s, with ID: %s opened." % (device.productCode(), device.deviceId().toXsString()))

        # Create and attach callback handler to device
        callback = XdaCallback()
        device.addCallbackHandler(callback)

        # Put the device into configuration mode before configuring the device
        print("Putting device into configuration mode...")
        if not device.gotoConfig():
            raise RuntimeError("Could not put device into configuration mode. Aborting.")
        
        # Put the device into configuration mode before configuring the device
        print("Creating a log file...")
        logFileName = "logfile.mtb"
        if device.createLogFile(logFileName) != xda.XRV_OK:
            raise RuntimeError("Failed to create a log file. Aborting.")
        else:
            print("Created a log file: %s" % logFileName)

        print("Putting device into measurement mode...")
        if not device.gotoMeasurement():
            raise RuntimeError("Could not put device into measurement mode. Aborting.")

        print("Starting recording...")
        if not device.startRecording():
            raise RuntimeError("Failed to start recording. Aborting.")

        print("Main loop. Recording data for 10 seconds.")
        print("Calibrating orientation from first frame. Please hold the device in the desired reference orientation.")

        startTime = xda.XsTimeStamp_nowMs()
        while xda.XsTimeStamp_nowMs() - startTime <= 60000:
            if callback.packetAvailable():
                # Retrieve a packet
                packet = callback.getNextPacket()

                s = ""

                if packet.containsOrientation():
                    # Get raw quaternion
                    raw_quaternion = packet.orientationQuaternion()
                    
                    # Apply orientation correction
                    corrected_quaternion = orientation_corrector.correct_orientation(raw_quaternion)
                    
                    # Get Euler angles from both raw and corrected quaternions
                    raw_euler = packet.orientationEuler()
                    raw_roll, raw_pitch, raw_yaw = raw_euler.x(), raw_euler.y(), raw_euler.z()
                    
                    # Convert corrected quaternion to Euler angles
                    corr_roll, corr_pitch, corr_yaw = orientation_corrector.quaternion_to_euler(corrected_quaternion)
                    
                    s += "Raw Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n" % (raw_roll, raw_pitch, raw_yaw)
                    s += "Cor Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n" % (corr_roll, corr_pitch, corr_yaw)

                print("%s\r" % s, end="", flush=True)

        print("\nStopping recording...")
        if not device.stopRecording():
            raise RuntimeError("Failed to stop recording. Aborting.")

        print("Closing log file...")
        if not device.closeLogFile():
            raise RuntimeError("Failed to close log file. Aborting.")

        print("Removing callback handler...")
        device.removeCallbackHandler(callback)

        print("Closing port...")
        control.closePort(mtPort.portName())

        print("Closing XsControl object...")
        control.close()

    except RuntimeError as error:
        print(error)
        sys.exit(1)
    else:
        print("Successful exit.")