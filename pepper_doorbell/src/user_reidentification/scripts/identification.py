#!/usr/bin/env python
# Title : intercom_demo.py
# Author : Yuxin Huang
# Date : 31/03/19
# Version : 1.0
import cv2
import numpy as np
import rospy, qi, argparse
import sys
import time
import naoqi
import face_recognition
from naoqi import *
from diagnostic_msgs.msg import KeyValue # message type /iot_updates uses
from std_msgs.msg import Empty

name = "unknown"

def callback(data):
	print("inside callback")

def iot_callback(data):
    if(data.key == "Hall_Intcm" and data.value == "ON"): # Button pressed
		intcm_ring()

def enter_ID(session):
    # Get the service ALTabletService.
    tabletService = session.service("ALTabletService")

    try:
        # Display a local web page located in boot-config/html folder
        # The ip of the robot from the tablet is 198.18.0.1
        tabletService.showWebview("http://198.18.0.1/apps/boot-config/preloading_dialog.html")

        time.sleep(3)

        # Javascript script for displaying a prompt
        # ALTabletBinding is a javascript binding inject in the web page displayed on the tablet
        script = """
            var name = prompt("Please enter your name"); 
	    ALTabletBinding.raiseEvent(name)
        """

        # Don't forget to disconnect the signal at the end
        signalID = 0

        # function called when the signal onJSEvent is triggered
        # by the javascript function ALTabletBinding.raiseEvent(name)
        def callback(event):
            global name
            name = event
            print ("name is "+name)
            promise.setValue(True)
            
        promise = qi.Promise()

        # attach the callback function to onJSEvent signal
        signalID = tabletService.onJSEvent.connect(callback)

        # inject and execute the javascript in the current web page displayed
        tabletService.executeJS(script)

        try:
            promise.future().hasValue(30000)
        except RuntimeError:
            raise RuntimeError('Timeout: no signal triggered')

    except Exception, e:
        print "Error was:", e
    # Hide the web view
    tabletService.hideWebview()
    # disconnect the signal
    tabletService.onJSEvent.disconnect(signalID)


def intcm_ring():
    global name
    url = 'http://192.168.1.99/mjpg/video.mjpg?login=YWRtaW46RzAwZHJvYm90NQ=+'
    video_capture = cv2.VideoCapture(url)#url
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))  
    tabletService = session.service("ALTabletService")

    #train all faces of known people 
    path = r'/home/wg7/4th/known_people'
    known_face_encodings = []
    known_face_names = []
    for parent,dirnames,filenames in os.walk(path):
        for filename in filenames:
            visiter_name_path = os.path.join(parent,filename)
            visiter_image = face_recognition.load_image_file(visiter_name_path)
            known_face_encodings.append(face_recognition.face_encodings(visiter_image)[0])
            known_face_names.append(os.path.splitext(os.path.basename(visiter_name_path))[0])


    # Initialize some variables
    face_locations = []
    face_encodings = []
    face_names = []
    process_this_frame = True
    face_appeared = ["unknown"]
    time_count = 1;
    name = "Unknown"
    while True: 
        exist = False
        # Grab a single frame of video
        ret, frame = video_capture.read()
        out.write(frame)
        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]

        # Only process every other frame of video to save time
        if process_this_frame:
        # Find all the faces and face encodings in the current frame of video
            face_locations = face_recognition.face_locations(rgb_small_frame)
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
    
            for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(known_face_encodings, face_encoding,tolerance=0.6)

                # If a match was found in known_face_encodings, just use the first one.
                if True in matches:
                    first_match_index = matches.index(True)
                    name = known_face_names[first_match_index]
                    for face_appear in face_appeared:                   
                        if name is face_appear:
                            exist = True
                    if not exist:
                        face_appeared.append(name)
			tabletService.showImageNoCache("http://192.168.1.99/jpg/image.jpg?login=YWRtaW46RzAwZHJvYm90NQ==")
                        time.sleep(3)
                        animatedProxy.say("Hello, your friend "+ name +" is at the door")	    	   
                else:
                    tabletService.showImageNoCache("http://192.168.1.99/jpg/image.jpg?login=YWRtaW46RzAwZHJvYm90NQ==")
                    time.sleep(3)
                    print("unknown")
                    animatedProxy.say("Hello, you have a new visitor who is unknown to the system, Please enter name in the tablet if you know the visitor:")
                    enter_ID(session)
                    animatedProxy.say("Hello "+name+" Please look at the camera for 5 seconds to take a picture for record")
                    time_count1 = 0
                    ret, frame = video_capture.read()
                    while ret:
                        cv2.imshow('Video', frame)		    		    
	    	        time_count1 = time_count1 + 1
                        if cv2.waitKey(1) & 0xFF == ord('q'):
		            break		    
                        if time_count1 == 65:
                            break  
		        ret, frame = video_capture.read()

    	            # Resize frame of video to 1/4 size for faster face recognition processing
    	    	    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    	            # Convert the image from BGR color (which OpenCV uses) to RGB color (which 		    	face_recognition uses)
    	    	    rgb_small_frame = small_frame[:, :, ::-1]
		    face_locations = face_recognition.face_locations(rgb_small_frame)
        	    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
                    for (top, right, bottom, left), face_encoding in zip(face_locations,face_encodings):             
                        known_face_encodings.append(face_encoding)
                        known_face_names.append(name)
                        newface_frame = frame[(top*4-100):bottom*6,(left*4-100):right*6]
                        cv2.imwrite('/home/wg7/4th/known_people/'+ name + '.jpg',newface_frame)          
                        face_appeared.append(name)
                        animatedProxy.say("Nice to meet you "+name)       
                   
                face_names.append(name)

        process_this_frame = not process_this_frame

        # Display the results
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)


            # Draw a label with a name below the face
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Display the resulting image
        cv2.imshow('Video', frame)
        time_count = time_count + 1;
        # Hit 'q' on the keyboard to quit!
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if time_count == 100:
            break

    # Release handle to the webcam
    tabletService.hideImage()
    video_capture.release()
    out.release()
    cv2.destroyAllWindows()
    time.sleep(5)

def listener():
    rospy.init_node('intcm_listener', anonymous=True) # initialise node to subscribe to topic
    rospy.Subscriber("/devices/bell", Empty, callback) # subscribe to topic where intercom ring is published
    rospy.Subscriber("/iot_updates", KeyValue, iot_callback) # subscribe to topic /iot_updates
    rospy.spin() # keeps python from exiting until node is stopped

if __name__ == '__main__':
    from naoqi import ALProxy
    print(sys.version)

    animatedProxy = ALProxy("ALAnimatedSpeech", "pepper.local", 9559) # initialise animated speech proxy

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="pepper.local",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    while True:
        listener() #does not break out of loop until manually stopped on the terminal
