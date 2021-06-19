#!/usr/bin/env python
import subprocess, os, signal, sys, time, threading, rosbag, yaml, numpy as np, roslib, cv2, rospy, pyautogui, pynput
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from ros import rosbag
from std_msgs.msg import String
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
from pynput.mouse import Listener
from sensor_msgs.msg import CompressedImage
from rosbag import Bag
from tkinter import *

global bag_file, feature_file, input_topic, pause, buff, time_buff, buff_size, counter, current, framerate, step, start_time, compressed
pause = compressed = append = False
counter = 0
pause_time = None
buff_size = 100
input_topic = None

def main(argv):
	
	global  bag_file, feature_file, input_topic, append, start_time, pause, buff, time_buff, buff_size, counter, current, framerate, step, compressed, image_pub, new_bag

	rospy.init_node('image_feature', anonymous=True)
	#ic = image_feature()

	#new_bag = "prueba.bag"
	image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=10)

	if (len(argv) < 3):
		print("Necesita al menos 3 argumentos. (Ruta absoluta bag/Topico)")
		exit(0)		
	
	#Abrir bag y obtener frame	
	bag_file = argv[1]
	bag = rosbag.Bag(bag_file)
	info_dict = yaml.load(bag._get_yaml_info())
	topics =  info_dict['topics']
	topic = topics[1]
	messages =  topic['messages']
	duration = info_dict['duration']
	topic_type = topic['type']
	
	#Obtener el nombre del topico
	input_topic = argv[2]
	
	#Parametros
	print ("Parametros: ","\n\t- Archivo bag: ", bag_file, "\n\t- Topico: ", input_topic)
	print ("\nTopicos Rosbag encontrados: ")
	for top in topics:
		print("\t- ", top["topic"], "\n\t\t-Tipo: ", topic["type"],"\n\t\t-Fps: ", topic["frequency"])
		
	#Comprobar la compresion del topico
	if 'CompressedImage' in topic_type:
		compressed = True
		
	#Obtener fps	
	framerate = messages/duration
	step = framerate/5
	
	#Crear archivo de resultados
	feature_file = ((bag_file.split(".")[0]).split("/")[-1]) + "_RES"
	if os.path.exists(feature_file):
		if not append:
			os.remove(feature_file)
	file_obj = open(feature_file, 'a')
	
	bridge = CvBridge()
	buff = []
	time_buff = []
	
	#Bucle recorrido bag
	for topic, msg, t in bag.read_messages(topics=[input_topic]):
		current = counter
		
		#Obtener fotograma
		if not compressed:
			try:
				cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
			except CvBridgeError as e:
				print(e)
		else:
			nparr = np.fromstring(msg.data, np.uint8)
			cv_image = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)
			
			
		if counter == 0:
			start_time = t
			
		#Asociar imagen buffer	
		buff.append(msg)
		time_buff.append(t.to_sec() - start_time.to_sec())	
		
		#Mostrar frame

		cv2.imshow("Image", cv_image)
		keyPressed(cv_image, file_obj)
		
		
		#Pausar imagen
		while(pause):
			if not compressed:
				cv2.imshow("Image", bridge.imgmsg_to_cv2(buff[counter], "bgr8"))
			else:
				nparr = np.fromstring(buff[counter].data, np.uint8)
				cv2.imshow("Image", cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR))
				
			keyPressed(cv_image, file_obj)
			if counter < current and not pause:
				for msg in buff[counter::]:
					if not compressed:
						cv2.imshow("Image", bridge.imgmsg_to_cv2(msg, "bgr8"))
					else:
						nparr = np.fromstring(msg.data, np.uint8)
						cv2.imshow("Image", cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR))
					keyPressed(cv_image, file_obj)
					if pause:
						break
					if counter < current:	
						counter += 1

		if len(buff) >= buff_size:
			del buff[0:10]		
		if len(time_buff) >= buff_size:
			del time_buff[0:10]		
			counter -= 10
		counter += 1

	cv2.destroyAllWindows()
	bag.close()
	new_bag.close()
	
#Controles aplicacion
def keyPressed(cv_image, file_obj, key = None):
	global bag_file, prev_frame, pause, counter, time_buff, counter, current, framerate, step, x_mouse, y_mouse
	
	key = cv2.waitKey(int(round(1000/framerate)))
	if key == -1:
		return
	if  key & 0xFF == 27:
		cv2.destroyAllWindows()
		exit(0)	
	if  key & 0xFF == ord('a'):
		pause = True
		if counter == 0:
			return
		counter -= 1
	if  key & 0xFF == ord('d'):
		pause = True
		if current == counter:
			return
		counter += 1
	if  key & 0xFF == ord('t'):
		x_mouse, y_mouse = pyautogui.position()
		
	if  key & 0xFF == ord('c'):
		x1_mouse, y1_mouse = pyautogui.position()
		
		font = cv2.FONT_HERSHEY_COMPLEX_SMALL
		
		cv2.putText(cv_image,'Car',(x_mouse-100,y_mouse-110), font, 0.85,(0,255,0),1,cv2.LINE_AA)
		cv2.rectangle(cv_image,(x_mouse-100,y_mouse-100),(x1_mouse-100,y1_mouse-100),(0,255,0),3)
		publicar(cv_image)
		file_obj.write(str(time_buff[counter]) + "\car\n")
		
		
		cv2.imshow("I", cv_image)

	if  key & 0xFF == ord('p'):
		x1_mouse, y1_mouse = pyautogui.position()
		
		font = cv2.FONT_HERSHEY_COMPLEX_SMALL
		
		cv2.putText(cv_image,'Person',(x_mouse-100,y_mouse-110), font, 0.85,(0,255,0),1,cv2.LINE_AA)
		cv2.rectangle(cv_image,(x_mouse-100,y_mouse-100),(x1_mouse-100,y1_mouse-100),(0,255,0),3)
		publicar(cv_image)
		file_obj.write(str(time_buff[counter]) + "\person\n")
		
		
		cv2.imshow("I", cv_image)
	
	if  key & 0xFF == ord('s'):
		x1_mouse, y1_mouse = pyautogui.position()
		
		font = cv2.FONT_HERSHEY_COMPLEX_SMALL
		
		cv2.putText(cv_image,'Signal',(x_mouse-100,y_mouse-110), font, 0.85,(0,255,0),1,cv2.LINE_AA)
		cv2.rectangle(cv_image,(x_mouse-100,y_mouse-100),(x1_mouse-100,y1_mouse-100),(0,255,0),3)
		publicar(cv_image)
		file_obj.write(str(time_buff[counter]) + "\tsignal\n")
		
		
		cv2.imshow("I", cv_image)

	if  key & 0xFF == ord(' '):
		pause_time = None
		if pause is True:
			pause = False
		else:
			pause = True

def publicar(cv_image):
	msg = CompressedImage()
	msg.header.stamp = rospy.Time.now()
	msg.format = "jpeg"
	msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tobytes()
	

if __name__ =='__main__':
    main(sys.argv)
	

#python3 anotacion.py /home/roberott/Downloads/coche.bag /cv_camera/image_raw