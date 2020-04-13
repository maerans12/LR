from __future__ import print_function
import rospy
import cv2 as cv
from clever import srv
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clever.srv import SetLEDEffect
from pyzbar.pyzbar import decode as qr_read
from threading import Thread

# inits
rospy.init_node('flight')
bridge = CvBridge()

# proxys
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# pubs
color_debug = rospy.Publisher("/color_debug", Image)
qr_debug = rospy.Publisher("/qr_debug", Image)


def lenta():
    print('blink purple')
    set_effect(effect='blink', r=255, g=0, b=255)
    rospy.sleep(5)
    set_effect(r=0, g=0, b=0)
    print('blink off')


def lenta_r():
    print('blink red')
    set_effect(effect='blink', r=255, g=0, b=0)
    rospy.sleep(5)
    set_effect(r=0, g=0, b=0)
    print('blink off')


def check_temp(data):
    global cap  # var for waiting the capture
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')[80:160, 100:220]  # get frame
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # get binarized images in each color
    red = cv.inRange(hsv, (165, 70, 158), (255, 209, 255))
    yellow = cv.inRange(hsv, (10, 80, 88), (49, 220, 225))
    green = cv.inRange(hsv, (26, 28, 60), (135, 162, 225))

    # count non-zero pixels
    color = {'r': cv.countNonZero(red),
             'y': cv.countNonZero(yellow),
             'g': cv.countNonZero(green)}

    temperature[n] = max(color, key=color.get)  # get max key
    print(n, color, '     ', temperature[n])

    # draw circle in centor of colored spot (only need color)
    try:
        if temperature[n] == 'r':
            moments = cv.moments(red, 1)  # get moments for find the center
            dM01 = moments['m01']
            dM10 = moments['m10']
            dArea = moments['m00']
            x = int(dM10 / dArea)
            y = int(dM01 / dArea)
            cv.circle(frame, (x, y), 5, (0, 0, 255), -1)  # draw
        if temperature[n] == 'y':
            moments = cv.moments(yellow, 1)
            dM01 = moments['m01']
            dM10 = moments['m10']
            dArea = moments['m00']
            x = int(dM10 / dArea)
            y = int(dM01 / dArea)
            cv.circle(frame, (x, y), 5, (0, 255, 255), -1)
        if temperature[n] == 'g':
            moments = cv.moments(green, 1)
            dM01 = moments['m01']
            dM10 = moments['m10']
            dArea = moments['m00']
            x = int(dM10 / dArea)
            y = int(dM01 / dArea)
            cv.circle(frame, (x, y), 5, (0, 255, 0), -1)
    except ZeroDivisionError:
        print('zero')

    color_debug.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))  # publish to topic (for web-video-server)

    # led and print if covid
    if max(color, key=color.get) == 'y' or max(color, key=color.get) == 'r':
        t = Thread(target=lenta)
        t.daemon = True
        t.start()
        print('sbrosheno')

    # unsubscribe from topic (get only one capture)
    image_sub.unregister()
    cap = True


def qr_check(data):
    global cap
    frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    barcodes = qr_read(frame)  # read the barcode using zbar
    if barcodes:
        print(barcodes[0].data)

        # draw rect and publish to topic
        (x, y, w, h) = barcodes[0].rect
        cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        qr_debug.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))

        if barcodes[0].data == 'COVID - 19' or barcodes[0].data == 'COVID - 2019':
            t = Thread(target=lenta_r)
            t.daemon = True
            t.start()

        cap = True
        image_sub.unregister()


# coords of each point
coords = {1: [0.295, 0.295, 1],
          3: [0.295, 0.885, 1],
          5: [0.295, 1.475, 1],
          7: [0.295, 2.065, 1],
          9: [0.59, 2.655, 1],
          8: [0.885, 2.065, 1],
          6: [0.885, 1.475, 1],
          4: [0.885, 0.885, 1],
          2: [0.885, 0.295, 1]}

# dict for temperatures
temperature = {}

# copter's way
path = [1, 3, 5, 7, 9, 8, 6, 4, 2]

# take off
print()
print('take off')
navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(1.3)
telem = get_telemetry(frame_id='aruco_map')
navigate(x=telem.x, y=telem.y, z=1.5, frame_id='aruco_map')
rospy.sleep(13)

# go using our way
for n in path:
    cap = False
    print()
    print('flight to', n, coords[n])
    navigate(x=coords[n][0], y=coords[n][1], z=coords[n][2], frame_id='aruco_map')  # go to point
    rospy.sleep(4)
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, check_temp, queue_size=1)  # get capture
    while not cap:  # wait the capture
        rospy.sleep(0.5)
    rospy.sleep(3)

# home
print()
print('flight to home')
navigate(x=0, y=0., z=1.5, frame_id='aruco_map')
rospy.sleep(4)
print('land')
land()

print()
print(temperature)

print()
print('wait 2m')
rospy.sleep(120)

# take off
print()
print('take off')
navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(1.3)
telem = get_telemetry(frame_id='aruco_map')
navigate(x=telem.x, y=telem.y, z=1.5, frame_id='aruco_map')
rospy.sleep(3)

for n in path:
    if temperature[n] == 'r' or temperature[n] == 'y':  # if was temperatute high or middle
        cap = False
        print()
        print('flight to', n, coords[n])
        # flight to this point
        navigate(x=coords[n][0], y=coords[n][1], z=coords[n][2], frame_id='aruco_map')  # 1
        rospy.sleep(3)
        image_sub = rospy.Subscriber('main_camera/image_raw', Image, qr_check, queue_size=1)  # try to read qr
        rospy.sleep(4)
        if not cap:
            print('spusk')
            navigate(x=coords[n][0], y=coords[n][1], z=0.7, speed=0.5, frame_id='aruco_map')  # go down for better qr view
            while not cap:
                rospy.sleep(0.5)
        image_sub.unregister()  # unsubscribe of topic in each situation
        rospy.sleep(3)

    else:  # it it was good we won't fly there
        print()
        print(n, 'healthy at first')

# home
print()
print('flight to home')
navigate(x=0, y=0., z=1.5, frame_id='aruco_map')
rospy.sleep(5)
print('land')
land()
