#! /usr/bin/python3
import numpy as np
import cv2

import multiprocessing

#modules needed for LCD display
import smbus
import time

# modules needed for speech recognition
import speech_recognition as sr
from pydub import AudioSegment

#modules needed for speech output
import pyttsx3 as speak
import pygame

#modules needed for facial recognition
from PIL import Image

#modules needed for facial detection and gesture recognition
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

#define pins needed for motor commands
import gpiozero 
from gpiozero import LED


pin1 = LED(13)
pin2 = LED(18)
pin3 = LED(12)

pin_tail = LED(19)

def millis():
    return int(round(time.time() * 1000))
    
    
# places for Deepface to look up
Bethany = "/home/gouda/team_work/Bethany.jpg"
Bryn = "/home/gouda/team_work/Brynn.jpg"
Johanan = "/home/gouda/team_work/Johanan.jpg"
Sudharshan = "/home/gouda/team_work/Sudharshan.jpg"
Sylvester = "/home/gouda/team_work/Sylvester.jpg"

album = [Bethany, Bryn, Johanan, Sudharshan,Sylvester ]
names = ['Bethany', 'Bryn', 'Johanan', 'Sudharshan','Sylvester']

#phrase triggers for speech recognition to trigger
desired_phrases_facial_detection = ["come here", 'slide']
desired_phrases_gesture_detection = ["do you see this"]
desired_phrases_movement = ['speak','gouda','move backward', 'move to the right','move to the left', 'rotate clockwise', 'rotate counter clockwise','stop']

all_desired_phrases = desired_phrases_facial_detection + desired_phrases_gesture_detection + desired_phrases_movement


#variables for communication with arduino for LCD
arduino_address = 0x08
bus = smbus.SMBus(1)
expression = "Sleep" # sleep by default

vision_threshold = 15
woken_up = 0

# Function to do speech output
def speak_text(text):
    print("Speaking: ",text)
    engine = speak.init()
    engine.say(text)
    engine.runAndWait()

def play_audio(file_path, duration):
    pygame.mixer.init()
    pygame.mixer.music.load(file_path)
    pygame.mixer.music.play()
    
    # Get the starting time
    start_time = pygame.time.get_ticks()
    
    # Play for 5 seconds
    while pygame.time.get_ticks() - start_time < duration:
        pygame.time.Clock().tick(10)
    
    # Stop the playback
    pygame.mixer.music.stop()
# Function to detect faces in frame, align and follow
def follow_me_face():
    person_detected = None
    print("you are in mediapipe facial detection part of code")
    
    #initialize face detection model
    mp_face_detection = mp.solutions.face_detection
    mp_drawing = mp.solutions.drawing_utils
    face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.5)
    
    # Capture video from the default camera (0)
    cap = cv2.VideoCapture(0)
    
    frame_counter = 0
    
    while person_detected is None:
        # Read a frame from the video
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Detect faces in the frame
        results = face_detection.process(frame_rgb)
        # Check if any faces are detected
        if results.detections:
            # If a face is detected, call the facial_recognition function
            print('recognize the face')
            # Get the bounding box of the biggest face detected
            biggest_face = max(results.detections, key=lambda detection: (detection.location_data.relative_bounding_box.width * detection.location_data.relative_bounding_box.height))
                
            # Extract the bounding box parameters
            bboxC = biggest_face.location_data.relative_bounding_box
            height, width, _ = frame.shape
            xmin, ymin = int(bboxC.xmin * width), int(bboxC.ymin * height)
            xmax, ymax = int((bboxC.xmin + bboxC.width) * width), int((bboxC.ymin + bboxC.height) * height)

            # Calculate the center of the bounding box
            center_x = (xmin + xmax) // 2
            center_y = (ymin + ymax) // 2
                
            # Print the center coordinates
            bbox_area = (xmax - xmin) * (ymax - ymin)

            # align code
            if (center_x > ((width/2)+vision_threshold)) :
                rotateCW()
            elif (center_x < ((width/2)-vision_threshold)) :
                rotateCCW()
            else:
                stop()
                tail_spin_start()
                if bbox_area < 20000:
                    moveforward()
                elif bbox_area > 30000:
                    movebackward()
                else: 
                    stop()
                    person_detected = 'done'
                    break
        else:
            print("Can't find a face, trying again...")
            rotateCW()
        
    # Release the video capture object and close all windows
    cap.release()
    cv2.destroyAllWindows()
    
    return person_detected

# Function to detect faces in frame, align
def find_face():
    person_detected = None
    print("you are in mediapipe facial detection part of code")
    
    #initialize face detection model
    mp_face_detection = mp.solutions.face_detection
    mp_drawing = mp.solutions.drawing_utils
    face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.5)
    
    # Capture video from the default camera (0)
    cap = cv2.VideoCapture(0)
    
    frame_counter = 0
    
    while person_detected is None:
        # Read a frame from the video
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Detect faces in the frame
        results = face_detection.process(frame_rgb)
        # Check if any faces are detected
        if results.detections:
            # If a face is detected, call the facial_recognition function
            print('recognize the face')
            # Get the bounding box of the biggest face detected
            biggest_face = max(results.detections, key=lambda detection: (detection.location_data.relative_bounding_box.width * detection.location_data.relative_bounding_box.height))
                
            # Extract the bounding box parameters
            bboxC = biggest_face.location_data.relative_bounding_box
            height, width, _ = frame.shape
            xmin, ymin = int(bboxC.xmin * width), int(bboxC.ymin * height)
            xmax, ymax = int((bboxC.xmin + bboxC.width) * width), int((bboxC.ymin + bboxC.height) * height)

            # Calculate the center of the bounding box
            center_x = (xmin + xmax) // 2
            center_y = (ymin + ymax) // 2
                
            # Print the center coordinates
            bbox_area = (xmax - xmin) * (ymax - ymin)

            # align code
            if (center_x > ((width/2)+vision_threshold)) :
                rotateCW()
            elif (center_x < ((width/2)-vision_threshold)) :
                rotateCCW()
            else:
                stop()
                person_detected = 'done'
                break
        else:
            print("Can't find a face, trying again...")
            rotateCW()
        
    # Release the video capture object and close all windows
    cap.release()
    cv2.destroyAllWindows()
    
    return person_detected
    
centred = 0
def slide_me_face():
    global centred
    person_detected = None
    print("you are in mediapipe facial detection part of code")
    
    #initialize face detection model
    mp_face_detection = mp.solutions.face_detection
    mp_drawing = mp.solutions.drawing_utils
    face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.5)
    
    # Capture video from the default camera (0)
    cap = cv2.VideoCapture(0)
    started = 0
    frame_counter = 0
    
    while person_detected is None:
        # Read a frame from the video
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Detect faces in the frame
        results = face_detection.process(frame_rgb)
        # Check if any faces are detected
        if results.detections:
            # If a face is detected, call the facial_recognition function
            print('recognize the face')
            # Get the bounding box of the biggest face detected
            biggest_face = max(results.detections, key=lambda detection: (detection.location_data.relative_bounding_box.width * detection.location_data.relative_bounding_box.height))
                
            # Extract the bounding box parameters
            bboxC = biggest_face.location_data.relative_bounding_box
            height, width, _ = frame.shape
            xmin, ymin = int(bboxC.xmin * width), int(bboxC.ymin * height)
            xmax, ymax = int((bboxC.xmin + bboxC.width) * width), int((bboxC.ymin + bboxC.height) * height)

            # Calculate the center of the bounding box
            center_x = (xmin + xmax) // 2
            center_y = (ymin + ymax) // 2
                
            # Print the center coordinates
            bbox_area = (xmax - xmin) * (ymax - ymin)
            
            if centred == 0:
                if (center_x < ((width/2)+vision_threshold)) and (center_x > ((width/2)-vision_threshold)):
                    centred = 1
                else:
                    moveright()
            if started == 0 and centred == 1:
                current = millis()
                slide_start_time = current
                started = 1
            if centred == 1:
                # slide code
                if (millis() - slide_start_time) < 15000:
                    if (center_x > ((width/2)+vision_threshold)) :
                        moveright()
                    elif (center_x < ((width/2)-vision_threshold)) :
                        moveleft()
                    else:
                        stop()
                else:
                    person_detected = 'done'
                    centred = 0
                    stop()
                    break
        else:
            print("Can't find a face, trying again...")
            rotateCW()
        
    # Release the video capture object and close all windows
    cap.release()
    cv2.destroyAllWindows()
    
    return person_detected

# function to recognize faces
def facial_recognition(frame):
    print("you are in deepface part of code")
    for index, photo in enumerate(album):        
        result = DeepFace.verify(np.array(Image.open(photo)), np.array(frame), enforce_detection = False)
        print("person campared to is:",names[index])
        print('data is: ',result)
        if result['verified']:         
            person = names[index]
            break
            
    return person

# Function to detect gesture...need to update
def gesture_detection():
     gesture_detected = None
     print("You are in the MediaPipe hand gesture detection part of the code")
     
     # Initialize MediaPipe Hands model
     base_options = python.BaseOptions(model_asset_path='/home/gouda/team_work/gesture_recognizer.task')
     options = vision.GestureRecognizerOptions(base_options=base_options)
     recognizer = vision.GestureRecognizer.create_from_options(options)
     
     cam = cv2.VideoCapture(0)
     frame_counter = 0
     
     
     while gesture_detected is None:
         success, img = cam.read()
         
         images = []
         results = []
         mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img)
         
         recognition_result = recognizer.recognize(mp_image)
         
         if img.any():
             top_gesture = recognition_result.gestures
             if len(top_gesture) !=0:
                 if top_gesture[0][0].category_name!= 'None':
                     gesture_detected = top_gesture[0][0].category_name
     
     # Release the video capture object and close all windows
     cam.release()
     cv2.destroyAllWindows()
     return gesture_detected

# Function for displaying on LCD, connected to an Arduino
def order_arduino_for_LCD(expression):
    #list of expressions taken as input
    expressions = ["Happy", "Happy Blink","Mad","Mad Blink","Sleep"]
    value = expressions.index(expression)
    
    #to debug
    print("expression needed to convey: ",expression)
    print("expression value: ",value)
    
    bus.write_byte(arduino_address, value)
    time.sleep(0.1)  # Wait for the Arduino to process the data
    
# Function for speech recognition
def recognize_phrase(desired_phrases):
    global woken_up
    if desired_phrases is None:
        desired_phrases = all_desired_phrases
    detected = 0
    recognizer = sr.Recognizer()
    person_detected = None
    gesture_detected = None
    phrase = all_desired_phrases
    

    with sr.Microphone() as source:
        print("Listening for '{}'...".format(phrase)) 
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source, phrase_time_limit=2) 
    try:
        text = recognizer.recognize_google(audio)
        print("I heard you say: ", text)
        if text.lower() == 'wake up':
            woken_up = 1
            expression = 'Happy Blink'
            order_arduino_for_LCD(expression)
            tail_spin_start()
            time.sleep(2)
            tail_spin_stop()
        if text.lower() == 'go to sleep':
            woken_up = 0
            expression = "Sleep"
            order_arduino_for_LCD(expression)
        
        if woken_up == 1:
            for phrase1 in desired_phrases:
                if phrase1.lower().replace(" ", "") in text.lower().replace(" ", ""):
                    detected = 1
                    expression = 'Happy Blink'
                    order_arduino_for_LCD(expression)
                    # Actionables based on phrase detected
                    # Call face detection mode if you hear trigger for facial recognition, modify the condition if more than 1 trigger for this
                    if desired_phrases_facial_detection[0].lower().replace(" ", "") in text.lower().replace(" ", ""):
                        person_detected = follow_me_face()
                        if person_detected:
                            order_arduino_for_LCD('Happy')
                            speak_text("I am  " + person_detected)
                    # Activate gesture detection
                    elif desired_phrases_facial_detection[1].lower().replace(" ", "") in text.lower().replace(" ", ""):
                        person_detected = slide_me_face()
                        if person_detected:
                            order_arduino_for_LCD('Happy')
                            speak_text("I am " + person_detected)
                    elif desired_phrases_gesture_detection[0].lower().replace(" ", "") in text.lower().replace(" ", ""): 
                        gesture_detected = gesture_detection()
                        if gesture_detected:
                            print("-----------------------------")
                            speak_text("I have recognized  " + gesture_detected)
                    # Play happy barking audio
                    elif desired_phrases_movement[0].lower().replace(" ", "") in text.lower().replace(" ", ""): 
                        play_audio('labrador-barking-daniel_simon.wav',2000)
                    # activate search for gestures
                    elif desired_phrases_movement[1].lower().replace(" ", "") in text.lower().replace(" ", ""): 
                        person_detected = find_face()
                        end_loop = 0
                        while(end_loop == 0):
                            gesture_detected = gesture_detection()
                            if gesture_detected == 'Pointing_Up':
                                spin()
                            elif gesture_detected == 'Thumb_Up': #make it do the operation while barking, change bark code
                            
                                play_audio('labrador-barking-daniel_simon.wav', 2000)
                               # p = multiprocessing.Process(target=play_audio, args=('labrador-barking-daniel_simon.wav', 5000))
                               # p.start(2
                                
                                p1 = multiprocessing.Process(target=tail_spin_start)
                                p1.start()

                                expression = 'Happy'
                                order_arduino_for_LCD(expression)


                                p3 = multiprocessing.Process(target=moveforward)
                                p3.start()
                                
                                p3.join()
                                time.sleep(1)  # Increase the sleep time to 2.5 seconds

                                stop()
                                time.sleep(4)
                                p1.join()
                                tail_spin_stop()

                            elif gesture_detected == 'Thumb_Down':
                                play_audio('Growling_Dog_Sound.wav', 2000)
                                tail_spin_start()
                                expression = 'Mad'
                                order_arduino_for_LCD(expression)
                                movebackward()
                                time.sleep(0.5)
                                tail_spin_stop()
                                stop()
                            elif gesture_detected == 'Closed_Fist':
                                end_loop = 1
                            
                    # Move backward
                    elif desired_phrases_movement[2].lower().replace(" ", "") in text.lower().replace(" ", ""): 
                        movebackward()
                    # Move right
                    elif desired_phrases_movement[3].lower().replace(" ", "") in text.lower().replace(" ", ""): 
                        moveright()
                    # Move left
                    elif desired_phrases_movement[4].lower().replace(" ", "") in text.lower().replace(" ", ""): 
                        moveleft()
                    # Rotate CW
                    elif desired_phrases_movement[5].lower().replace(" ", "") in text.lower().replace(" ", ""): 
                        rotateCW()
                    # Rotate CCW
                    elif desired_phrases_movement[6].lower().replace(" ", "") in text.lower().replace(" ", ""): 
                        rotateCCW()
                    elif desired_phrases_movement[7].lower().replace(" ", "") in text.lower().replace(" ", ""): 
                        stop()    
                if detected == 1:
                    break
            if detected == 0:
                recognize_phrase(None)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio. Listening again...")
        recognize_phrase(None)
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

# functions to move

def spin():
    #make a function to perform a rotation for specific time, 360 degrees
    start = millis()
    temp = millis()
    rotateCW()
    while(temp - start <= 4*5000):
        temp = millis()
        pass
    stop()
    

def moveforward():      #1
    #function to output PWM to move forward
    pin1.on()
    pin2.off()
    pin3.off() 

def movebackward():     #3
    #function to output PWM to move forward
    pin1.off()
    pin2.on()
    pin3.off()

def moveright():        #5
    #function to output PWM to move right
    pin1.off()
    pin2.off()
    pin3.on()

def moveleft():         #4
    #function to output PWM to move left
    pin1.on()
    pin2.on()
    pin3.off()

def rotateCW():         #8
    #function to output PWM to rotate CW
    print('rotating CW')
    pin1.off()
    pin2.on()
    pin3.on()

def rotateCCW():        #6
    #function to output PWM to rotate CCW
    print('rotating CCW')
    pin1.on()
    pin2.off()
    pin3.on()
def stop():             #0
    #function to output PWM to stop bot
    pin1.off()
    pin2.off()
    pin3.off()
    pin_tail.off()

def tail_spin_start():
    print('spinning tail')
    pin_tail.on()

def tail_spin_stop():
    print('stop spinning tail')
    pin_tail.off()
    

def blink_every():
    while True:
        order_arduino_for_LCD("Happy Blink")
        time.sleep(3)

def blink_on_command():
    order_arduino_for_LCD("Happy Blink")
    order_arduino_for_LCD("Happy Blink")



order_arduino_for_LCD('Sleep')
#tail_spin()
#rotateCW()
while True:
    try:
        recognize_phrase(None)
        print("Calling speech function again")
    except Exception:
        # Handle any exceptions raised by your_function
        print("An error occurred:", e)
        # Optionally, you can break out of the loop if needed
pwm_straight.close()
pwm_sideways.close()
pwm_rotate.close()
pwm_tail.close()
