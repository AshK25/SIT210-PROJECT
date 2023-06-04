

#import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import face_recognition
import imutils
import pickle
import time
import cv2
import numpy as np
from time import sleep
import RPi.GPIO as GPIO
import requests
import threading

MOTION_SENSOR_PIN = 7
RGB_LED_RED_PIN = 11
RGB_LED_GREEN_PIN = 12
RGB_LED_BLUE_PIN = 13
REC_PIN = 36
PLAYE_PIN = 38
PLAYL_PIN = 40

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTION_SENSOR_PIN, GPIO.IN)
GPIO.setup(RGB_LED_RED_PIN, GPIO.OUT)
GPIO.setup(RGB_LED_GREEN_PIN, GPIO.OUT)
GPIO.setup(RGB_LED_BLUE_PIN, GPIO.OUT)
GPIO.setup(REC_PIN, GPIO.OUT)
GPIO.setup(PLAYE_PIN, GPIO.OUT)
GPIO.setup(PLAYL_PIN, GPIO.IN)

def motion_detected():
    while True:
        motion_detected = GPIO.input(MOTION_SENSOR_PIN)

        if motion_detected:
            print("Motion Detected!")
            face_recognition_model()

def activate_alarm():
	
	global alarm_active
	alarm_active = True
	def alarm_thread():
		while alarm_active:
					GPIO.output(RGB_LED_RED_PIN, GPIO.LOW)
					GPIO.output(RGB_LED_GREEN_PIN, GPIO.LOW)
					GPIO.output(RGB_LED_BLUE_PIN, GPIO.HIGH)
					GPIO.output(PLAYE_PIN, GPIO.HIGH)
					time.sleep(0.5)
					GPIO.output(RGB_LED_RED_PIN, GPIO.LOW)
					GPIO.output(RGB_LED_GREEN_PIN, GPIO.LOW)
					GPIO.output(RGB_LED_BLUE_PIN, GPIO.LOW)
					GPIO.output(PLAYE_PIN, GPIO.LOW)
					time.sleep(0.5)
					
	alarm_thread = threading.Thread(target = alarm_thread)
	alarm_thread.start()



def notify_user():
	# IFTTT webhook key
	ifttt_webhook_key = " dQHTOBnSNZeMMFRLC5UMG8"
	
	event_name = "security_alarm"
	
	requests.post(f"https://maker.ifttt.com/trigger/{event_name}/with/key/dQHTOBnSNZeMMFRLC5UMG8")
	

def face_recognition_model():
	#Initialize 'currentname' to trigger only when a new person is identified.
	currentname = "unknown"
	#Determine faces from encodings.pickle 
	encodingsP = "encodings.pickle"

	# load the known faces and embeddings along with OpenCV's Haar
	# cascade for face detection
	print("[INFO] loading encodings + face detector...")
	data = pickle.loads(open(encodingsP, "rb").read())

	# initialize the video stream and allow the camera sensor to warm up
	# Set the ser to the followng
	vs = VideoStream(usePiCamera=True).start()
	time.sleep(2.0)

	# start the FPS counter
	fps = FPS().start()

	# loop over frames from the video file stream
	while True:
		# grab the frame from the threaded video stream and resize it
		# to 500px (to speedup processing)
		frame = vs.read()
		frame = imutils.resize(frame, width=500)
		# Detect the face boxes
		boxes = face_recognition.face_locations(frame)
		# compute the facial embeddings for each face bounding box
		encodings = face_recognition.face_encodings(frame, boxes)
		names = []

		# loop over the facial embeddings
		for encoding in encodings:
			# attempt to match each face in the input image to our known
			# encodings
			matches = face_recognition.compare_faces(data["encodings"],
				encoding)
			name = "Unknown" #if face is not recognized, then print Unknown

			# check to see if we have found a match
			if True in matches:
				# find the indexes of all matched faces then initialize a
				# dictionary to count the total number of times each face
				# was matched
				matchedIdxs = [i for (i, b) in enumerate(matches) if b]
				counts = {}

				# loop over the matched indexes and maintain a count for
				# each recognized face face
				for i in matchedIdxs:
					name = data["names"][i]
					counts[name] = counts.get(name, 0) + 1

				# determine the recognized face with the largest number
				# of votes (note: in the event of an unlikely tie Python
				# will select first entry in the dictionary)
				name = max(counts, key=counts.get)

				#If someone in your dataset is identified, print their name on the screen
				if currentname != name:
					currentname = name
					print(currentname)
					GPIO.output(RGB_LED_GREEN_PIN, GPIO.HIGH)
				
				else:
					activate_alarm()
					notify_user()
					

			# update the list of names
			names.append(name)

		# loop over the recognized faces
		for ((top, right, bottom, left), name) in zip(boxes, names):
			# draw the predicted face name on the image - color is in BGR
			cv2.rectangle(frame, (left, top), (right, bottom),
				(0, 255, 225), 2)
			y = top - 15 if top - 15 > 15 else top + 15
			cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
				.8, (0, 255, 255), 2)

		# display the image to our screen
		cv2.imshow("Facial Recognition is Running", frame)
		key = cv2.waitKey(1) & 0xFF

		# quit when 'q' key is pressed
		if key == ord("q"):
			break

		# update the FPS counter
		fps.update()

	# stop the timer and display FPS information
	fps.stop()
	print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
	print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

	# do a bit of cleanup
	cv2.destroyAllWindows()
	vs.stop()

#END OF FACIAL RECOGNITION

motion_detected()
GPIO.cleanup()
