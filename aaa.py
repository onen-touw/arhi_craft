import json
import logging
import sys
import time

from skyros.drone import Drone

import rospy
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback, srv
from clover.srv import SetLEDEffect
from pyzbar import pyzbar

from aruco_pose.msg import MarkerArray

ITEM_NONE = 0
DIAMOND = 1
STICK = 2
QUEST = 3


SWARM_DRONE_CNT = 3
SWARM_NET_ID = 0x3
SWARM_DRONE_ID = 2                 # TODO
SWARM_DRONE_ROLE = DIAMOND        # TODO
SWARM_DRONE_IS_LIDER = True        # TODO

TARTGET_HIGHT = 1.0

CRAFTS = {
    1: [
        {"dx": -1, "dy": 1, "t": DIAMOND, "o": False}, {"dx": 0, "dy": 1, "t": DIAMOND, "o": False}, {"dx": 1, "dy": 1, "t": DIAMOND, "o": False},  
        {"dx": -1, "dy": 0, "t": ITEM_NONE, "o": False}, {"dx": 0, "dy": 0, "t": STICK, "o": False}, {"dx": 1, "dy": 0, "t": ITEM_NONE, "o": False},  
        {"dx": -1, "dy": -1, "t": ITEM_NONE, "o": False}, {"dx": 0, "dy": -1, "t": STICK, "o": False}, {"dx": 1, "dy": -1, "t": ITEM_NONE, "o": False}  
    ], 
    2: [
        {"dx": -1, "dy": 1, "t": DIAMOND, "o": False}, {"dx": 0, "dy": 1, "t": DIAMOND, "o": False}, {"dx": 1, "dy": 1, "t": ITEM_NONE, "o": False},  
        {"dx": -1, "dy": 0, "t": DIAMOND, "o": False}, {"dx": 0, "dy": 0, "t": STICK, "o": False}, {"dx": 1, "dy": 0, "t": ITEM_NONE, "o": False},  
        {"dx": -1, "dy": -1, "t": ITEM_NONE, "o": False}, {"dx": 0, "dy": -1, "t": STICK, "o": False}, {"dx": 1, "dy": -1, "t": ITEM_NONE, "o": False} 
    ], 
    3:[
        {"dx": -1, "dy": 1, "t": ITEM_NONE, "o": False}, {"dx": 0, "dy": 1, "t": DIAMOND, "o": False}, {"dx": 1, "dy": 1, "t": DIAMOND, "o": False},  
        {"dx": -1, "dy": 0, "t": ITEM_NONE, "o": False}, {"dx": 0, "dy": 0, "t": DIAMOND, "o": False}, {"dx": 1, "dy": 0, "t": DIAMOND, "o": False},  
        {"dx": -1, "dy": -1, "t": STICK, "o": False}, {"dx": 0, "dy": -1, "t": ITEM_NONE, "o": False}, {"dx": 1, "dy": -1, "t": ITEM_NONE, "o": False} 
    ]
    }

rospy.init_node('arhid')






class arhic:
    def __init__(self):

        # rospy.Subscriber('aruco_detect/markers', MarkerArray, self.__markers_callback)
        self._img = rospy.Subscriber('main_camera/image_raw_throttled', Image, self.__image_callback, queue_size=5)
        # self._ros_led_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
        self._drone = Drone(network_id=SWARM_NET_ID, drone_id=SWARM_DRONE_ID, wifi_channel=5, tx_power=11, uart_port="/dev/ttyAMA1")

        self._drone.set_custom_message_callback(self.__msg_callback)

        self._swarm_drones = []                 #only for lider

        self._is_lider = SWARM_DRONE_IS_LIDER

        self._bridge = CvBridge()
        self._qr_data = None
        self._qr_deteceted_pos = None

        self._role = SWARM_DRONE_ROLE
        self._target_pos_in_swarm = None

        self._start_pos = None
        self._go_home = False
        print("arhic __init__")

    def __image_callback(self, data):
        img = self._bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
        barcodes = pyzbar.decode(img)
        for barcode in barcodes:
            b_data = barcode.data.decode('utf-8')
            print("Found QR: ", b_data)

            if self._qr_data is None and len(b_data) == 6:
                self._qr_data = barcode
                _qr_deteceted_pos = self._drone.get_telemetry()


            # b_type = barcode.type
            # (x, y, w, h) = barcode.rect
            # xc = x + w/2
            # yc = y + h/2
            # print('Found {} with data {} with center at x={}, y={}'.format(b_type, b_data, xc, yc))


    def task(self):
        print("start task")

        self._drone.start()

        self._start_pos = self._drone.get_telemetry()

        self.__send_start_msg()
        self._drone.wait(5)

        # Take off
        print("take off...")
        self._drone.takeoff(z=1.0)
        print("\tdone")
        self._drone.wait(5)

        telem = self._drone.get_telemetry()
        print(f"position x={telem.x} y={telem.y}")

        if self._is_lider:
            print("start lider cycle...")
            self.__lider_cycle()
            print("done lider cycle...")

        else:
            self.__slave_cycle()

        print("drone land...")
        self._drone.land(z=0.0)
        self._drone.stop()



    def __send_start_msg(self):
        # Send start json message to other drones
        start_message = {
            "t": "st",  # flight_command
            "drone_id": self._drone.drone_id,
        }
        self._drone.broadcast_custom_message(json.dumps(start_message))

 
    def __markers_callback(self, msg):
        print('Detected markers:')
        if self._item_type != 0:
            return
        
        for marker in msg.markers:
            if marker.id == 801 or marker.id == 802:
                self._item_type = 1
            elif marker.id == 803:
                self._item_type = 2
            else:
                self._item_type = 3
            print('Marker: %s' % marker)

    def __msg_callback(self, msg):
        try:
            data = json.loads(msg)
            dtype = data.get("t")

            if dtype == "fcsnd":  # flight_command
                if self._is_lider:
                    return
                
                # Check if this message is for this specific drone
                target_drone_id = data.get("d")
                if target_drone_id == self._drone.drone_id:
                    # Parse coordinates for this drone
                    slave_target = {
                        "x": data.get("x"),
                        "y": data.get("y"),
                        "z": data.get("z")
                    }
                    logging.info(f"Slave {self._drone.drone_id} got target: {slave_target}")

                    # cmd = {
                    #     "t": "fcack",                        # ack on item req
                    #     "m": self._drone.drone_id,          # this drone id
                    # }
                    # self._drone.broadcast_custom_message(json_msg = json.dumps(cmd))

            elif dtype == "st":
                #start cmd
                pass
            elif dtype == "rqi":    # request items type
                if self._is_lider:
                    return
                cmd = {
                    "t": "acki",                        # ack on item req
                    "m": self._drone.drone_id,          # this drone id
                    "r": self._item_type
                }
                self._drone.broadcast_custom_message(json_msg = json.dumps(cmd))

            elif dtype == "acki":    # ack items type
                id = data.get("m")
                
                if self._is_lider:
                    for obj in self._swarm_drones:
                        if obj["id"] == id:
                            obj["itemrole"] = data.get("r")
                            self._role_cnt += 1
                            return
                    print("wtf1")

        except json.JSONDecodeError:
            logging.warning(f"Failed to parse JSON: {msg}")

    def __wait_swarm(self):
        while True:
            # Wait for other drones to start
            print("Wait for other drones to start...")
            if self._drone.wait_for_drones(n=SWARM_DRONE_CNT-1, timeout=30.0):
                # Get network status with detailed info
                status = self._drone.get_network_status()

                # Show detailed drone info
                for drone_id, details in status["drone_details"].items():
                    self._swarm_drones.append({"id": drone_id, "itemrole": 0, "o": False, "tpos": {"x": 0, "y": 0}})
                    pos = details["position"]
                    logging.info(
                        f"  drone_{drone_id}: pos=({pos['x']:.1f},{pos['y']:.1f},{pos['z']:.1f}) "
                    )
                break
            self._drone.wait(5)

    def __check_if_lider(self):
        # Master drone logic: drone with lowest ID becomes master
        discovered_drones = self._drone.get_discovered_drones()
        swarm_drones = discovered_drones | {self._drone.drone_id}
        master_drone_id = min(swarm_drones)
        if master_drone_id == self._drone.drone_id:
            self._is_lider = True
        

    def __slave_cycle(self):

        while self._target_pos_in_swarm is None:
            self.__hold_pos(x=self._start_pos.x, y=self._start_pos.y)

        self._drone.navigate_with_avoidance(x=self._target_pos_in_swarm["x"], y=self._target_pos_in_swarm["y"], z=TARTGET_HIGHT)
        self.__hold_pos(x=self._target_pos_in_swarm["x"], y=self._target_pos_in_swarm["y"])

        while self._go_home == False:
            self.__hold_pos(x=self._target_pos_in_swarm["x"], y=self._target_pos_in_swarm["y"])

        self._drone.navigate_with_avoidance(self._start_pos.x, self._start_pos.y, z=TARTGET_HIGHT)


    def __lider_cycle(self):

        # self.__wait_swarm()

        # TODO
        self._swarm_drones.append({"id": self._drone.drone_id, "itemrole": self._role, "o": False, "tpos": {"x": 0, "y": 0}})
        # self._swarm_drones.append({"id": DRONE1_ID, "itemrole": DIAMOND, "o": False, "tpos": {"x": 0, "y": 0}})
        # self._swarm_drones.append({"id": DRONE2_ID, "itemrole": STICK, "o": False, "tpos": {"x": 0, "y": 0}})

        self._drone.wait(5)

        print("start qr-search...")
        if self.__qr_search() == False:
            print("\tfailed")

            self.__go_to_start()
            return
        
        print("\tdone")

        self._drone.wait(5)
        self.__go_to_start()
        
        # cetrilize()

        # self.__destribute_roles()

        # if self._target_pos_in_swarm is not None:
        #     self._drone.navigate_with_avoidance(x=self._target_pos_in_swarm["x"], y=self._target_pos_in_swarm["y"], z=TARTGET_HIGHT)

        # self.__hold_pos(x=self._target_pos_in_swarm["x"], y=self._target_pos_in_swarm["y"])

        # cmd = {
        #     "t": "hmsnd",                       # send go home cmd
        #     "m": self._drone.drone_id,          # master_id
        #     "d": id                            # target drone id
        # }

        # json_msg = json.dumps(cmd)
        # logging.info(f"Master sending to drone {id}: {json_msg} ({len(json_msg)} chars)")
        # self._drone.broadcast_custom_message(json_msg)



    def __go_to_start(self):
        print("go to start")
        # self._drone.navigate_with_avoidance(x=self._start_pos.x, y=self._start_pos.y, z=TARTGET_HIGHT)
        self._drone.navigate_wait(x=self._start_pos.x, y=self._start_pos.y, z=TARTGET_HIGHT, frame_id= "aruco_map")



    def __hold_pos(self, x, y, tlim = 10):
        st = time.time()

        while time.time() - st < tlim:
            self._drone.navigate_with_avoidance(x=x, y=y, z=TARTGET_HIGHT, timeout=1, frame_id= "aruco_map")
            self._drone.wait(0.1)

    def __destribute_roles(self):
        qr_cent_pos_x = 0
        qr_cent_pos_y = 0

        qrmsg = self._qr_data.data.decode('utf-8')
        craft_id = int(qrmsg[0])
        qr_pos = int(qrmsg[1])

        diamond_cnt = 0
        stick_cnt = 0

        drone_diamond_cnt = 0
        drone_stick_cnt = 0
        drone_quest_ind = -1

        craft = CRAFTS[craft_id]
        craft[qr_pos]["o"] = True

        for it in craft:
            if it["t"] == DIAMOND:
                diamond_cnt += 1
            elif it["t"] == STICK:
                stick_cnt += 1

        if craft[qr_pos]["t"] == DIAMOND:
            diamond_cnt -= 1
        elif craft[qr_pos]["t"] == STICK:
            stick_cnt -= 1

        for ind, sw in enumerate(self._swarm_drones):
            if sw["itemrole"] == DIAMOND:
                drone_diamond_cnt+=1
            elif sw["itemrole"] == STICK:
                drone_stick_cnt += 1
            elif sw["itemrole"] == QUEST:
                drone_quest_ind = ind

        print("quest drone id: ", drone_quest_ind)

        print(f"crast D={diamond_cnt} S={stick_cnt}")
        print(f"swarm D={drone_diamond_cnt} S={drone_stick_cnt}")

        if drone_diamond_cnt < diamond_cnt:
            self._swarm_drones[drone_quest_ind]["itemrole"] = DIAMOND
        elif drone_stick_cnt < stick_cnt:
            self._swarm_drones[drone_quest_ind]["itemrole"] = STICK

        centerX = qr_cent_pos_x - craft[qr_pos]["dx"]
        centerY = qr_cent_pos_y - craft[qr_pos]["dy"]

        print(f"center x={centerX} y={centerY}")

        for itmin in craft:
            if itmin["o"] == False:
                tp = itmin["t"]        # get item type
                
                for dr in self._swarm_drones:
                    if dr["o"] == False and dr["itemrole"] == tp:
                        print("www ",dr)
                        
                        dr["o"] = True
                        dr["tpos"]["x"] = itmin["dx"] + centerX
                        dr["tpos"]["y"] = itmin["dy"] + centerY
                        itmin["o"] = True
                        break

        for sw in self._swarm_drones:
            id = sw["id"]
            x = sw["tpos"]['x']
            y = sw["tpos"]['y']

            if self._drone.drone_id == id:
                self._target_pos_in_swarm = {"x": x, "y": y}
                continue

            cmd = {
                "t": "fcsnd",                       # flight_command
                "m": self._drone.drone_id,          # master_id
                "d": id,                            # target drone id
                "x": x,
                "y": y,
                "z": 2,
            }

            json_msg = json.dumps(cmd)
            logging.info(f"Master sending to drone {id}: {json_msg} ({len(json_msg)} chars)")
            self._drone.broadcast_custom_message(json_msg)

    def __qr_search(self):
        # krest
        sposes = [{"x": 0, "y": 0},{"x": 1.5, "y": 0},{"x": -1.5, "y": 0},{"x": 0, "y": 1.5},{"x": 0, "y": -1.5}, {"x": 0, "y": 0}]
        for pos in sposes:
            # self._drone.navigate_with_avoidance(x=pos["x"],y=pos["y"],z=TARTGET_HIGHT)
            r = self._drone.navigate_wait(x=pos["x"],y=pos["y"],z=TARTGET_HIGHT, frame_id= "aruco_map")
            print(pos, r)
            if self._qr_data is not None:
                return True

            self._drone.wait(0.1)

        return False


if __name__ == '__main__':
    ar = arhic()
    ar.task()
