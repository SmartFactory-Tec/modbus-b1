#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Int32,Int16,String
from nav_msgs.msg import Path
from geometry_msgs.msg import Vector3
from pymodbus.client import ModbusTcpClient as ModbusClient
from std_msgs.msg import Int32MultiArray as HoldingRegister
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int16
from move_base_msgs.msg import MoveBaseActionGoal,MoveBaseActionResult
import math

class Server:
  def __init__(self):
    # Subscribers
    self.battery_status = rospy.Subscriber("voltage_percentage", Int32, self.batterycallback)
    self.robot_status = rospy.Subscriber("robot_status", Int16, self.robotcallback)
    self.plan_distance = rospy.Subscriber("path_test_server", Path, self.distance_callback)
    rospy.loginfo("Subscribed to topics: voltage_percentage, robot_status, path_test_server") # Log subscription

    # Publishers
    self.newgoal =  rospy.Publisher("newcoordinates",Vector3 ,queue_size=10)
    self.mission_status = rospy.Publisher("missionstatus",Int16 ,queue_size=10)
    rospy.loginfo("Publishing to topics: newcoordinates, missionstatus") # Log publishing

    # Internal variables
    self.batterypercentage=100
    self.ro_status=0 # Initialize robot status
    self.status=2
    self.outputregister = HoldingRegister()
    self.modbusmode = {
      "Battery": 0,   #0-100
      "RobotStatus": 0,
      "Distance": 0,
    }
    self.distancegoal= 0.0
    self.distancerobot=0
    self.last_robot_status = 0 # To detect robot status changes for movement detection


  def batterycallback(self, msg):
    rospy.loginfo(f"Received battery voltage_percentage: {msg.data}") # Log battery data received
    self.batterypercentage=msg.data

  def robotcallback(self, msg):
    rospy.loginfo(f"Received robot_status: {msg.data}") # Log robot status data received
    if self.ro_status != msg.data and msg.data == 1: # Check for transition to ACTIVE (1) and status change
        rospy.loginfo("Robot status changed to ACTIVE, potential movement started.") # Log movement start
    self.ro_status=msg.data

  def distance_callback(self,data):
    rospy.loginfo("Received path_test_server data") # Log path data received
    if self.status==1: #Move
      for i in range(len(data.poses)-1):
        self.distancegoal += math.sqrt(pow((data.poses[i+1].pose.position.x - data.poses[i].pose.position.x),2) + pow((data.poses[i+1].pose.position.y - data.poses[i].pose.position.y), 2))
      self.distancerobot=int(self.distancegoal*1000)
      rospy.loginfo(f"Calculated distance from path: {self.distancegoal}, distancerobot: {self.distancerobot}") # Log calculated distance


if __name__ == '__main__':
  try:
    rospy.init_node('Server_info')
    robot = Server()
    r = rospy.Rate(10) # 10hz
    robot_moved_registered = False # Flag to register robot movement once

    while not rospy.is_shutdown():
      try:
        client =  ModbusClient("192.168.31.231",port=12345) #Server second computer
        UNIT = 0x1
        conexion = client.connect()
        rospy.loginfo("Modbus connection ready") # Log info instead of warn for successful connection
      except Exception as error:
        rospy.logwarn("Modbus connection error")
        rospy.logwarn(error)
        continue # Continue to the next loop iteration if Modbus connection fails

      try:
        rr = client.read_holding_registers(0,15,unit=UNIT)
        registers_data = rr.registers
        rospy.loginfo(f"Read holding registers: {registers_data}") # Log read registers as info
        rospy.loginfo("Info robot working")

        robot.status = registers_data[0] #MissionStatus 0-Charge, 1-Move, 2-Free, 3-Success, 4-Failure

        #Transform modbus coordinate 10->Positive; 11->Negative
        firstx=int(registers_data[1]/1000)
        firsty=int(registers_data[2]/1000)
        angle = int(registers_data[3])
        rospy.loginfo(f"Modbus raw coordinates: firstx_raw={registers_data[1]}, firsty_raw={registers_data[2]}, angle={angle}")
        rospy.loginfo(f"Initial parsed coordinates: firstx_int={firstx}, firsty_int={firsty}")


        if firstx==10:
          firstx=(registers_data[1]-firstx*1000)
        else:
          firstx=-(registers_data[1]-firstx*1000)

        if firsty==10:
          firsty=(registers_data[2]-firsty*1000)
        else:
          firsty=-(registers_data[2]-firsty*1000)

        rospy.loginfo(f"Final transformed coordinates: firstx={firstx}, firsty={firsty}")

        goalx = firstx/100.0  #Goalx
        goaly = firsty/100.0  #Goaly
        rospy.loginfo(f"Calculated goal coordinates: goalx={goalx}, goaly={goaly}")


        robot.mission_status.publish(robot.status)
        #Send info to planner trayectory to receive distance
        robot.newgoal.publish(Vector3(goalx,goaly,angle))
        rospy.loginfo(f"Published new goal: x={goalx}, y={goaly}, angle={angle}")


        robot.modbusmode["Battery"] = robot.batterypercentage
        robot.modbusmode["RobotStatus"] = robot.ro_status
        robot.modbusmode["Distance"] = robot.distancerobot
        myregisters = list(robot.modbusmode.values())
        robot.outputregister.data = [int(i) for i in myregisters]
        rospy.loginfo(f"Modbus output registers data prepared: {robot.outputregister.data}")


        rq = client.write_registers(3, robot.outputregister.data, unit=UNIT)
        rospy.loginfo(f"Written output registers to Modbus: {robot.outputregister.data}")


        # Verify robot movement registration - Register only once when status is ACTIVE
        if robot.ro_status == 1 and not robot_moved_registered:
            rospy.loginfo("Robot movement registered (Teleop or Goal based). Robot Status is ACTIVE.")
            robot_moved_registered = True # Set flag to True to register only once
        elif robot.ro_status != 1:
            robot_moved_registered = False # Reset flag when robot status is not ACTIVE


        client.close()
        rospy.loginfo("Modbus connection closed") # Log Modbus connection close
        rospy.sleep(1)

      except Exception as error:
        rospy.logwarn("Reading registers not ready")
        rospy.logwarn(error)
  except rospy.ROSInterruptException:
    rospy.loginfo("Navigation test finished.")
