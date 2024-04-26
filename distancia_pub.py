#!/usr/bin/python3

# para saber qual comando vc deve completar depois de #! ali em cima
# tem que colocar no terminal which python3
# copiar e adicionar logo em seguida

import rospy # biblioteca de python para ros
from sensor_msgs.msg import Image # o tipo da mensagem a ser enviada é do tipo imagem
from cv_bridge import CvBridge # pacote para converter ros para imagem open cv
import cv2 # biblioteca do open cv
 
# variables
# distance from camera to object(face) measured
x, y , h , w = 0,0 ,0 ,0
DISTANCE=0

Known_distance =31.5 # Inches
#mine is 14.3 something, measure your face width, are google it 
Known_width=5.7 #Inches
Distance_level =0
focal_lenght = 5.7*31.5
GREEN = (0,255,0) 
RED = (0,0,255)
BLACK = (0,0,0)
YELLOW =(0,255,255)
PERPEL = (255,0,255)
WHITE = (255,255,255)

fonts = cv2.FONT_HERSHEY_COMPLEX
fonts2 = cv2.FONT_HERSHEY_SCRIPT_SIMPLEX
fonts3 =cv2.FONT_HERSHEY_COMPLEX_SMALL
fonts4 =cv2.FONT_HERSHEY_TRIPLEX

def Distance_finder (Focal_Length, real_face_width, face_width_in_frame):
    # Function Discrption (Doc String)
    '''
    This Function simply Estimates the distance between object and camera using arguments(Focal_Length, Actual_object_width, Object_width_in_the_image)

    :param1 Focal_length(float): return by the Focal_Length_Finder function
    
    :param2 Real_Width(int): It is Actual width of object, in real world (like My face width is = 5.7 Inches)

    :param3 object_Width_Frame(int): width of object in the image(frame in our case, using Video feed)

    :return Distance(float) : distance Estimated  
    
    '''
    distance = (real_face_width * Focal_Length)/face_width_in_frame

    return distance

#face detection Fauction 
def face_data(image, CallOut, Distance_level):
    # Function Discrption (Doc String)
    '''
    
    This function Detect face and Draw Rectangle and display the distance over Screen
    
    :param1 Image(Mat): simply the frame 
    :param2 Call_Out(bool): If want show Distance and Rectangle on the Screen or not
    :param3 Distance_Level(int): which change the line according the Distance changes(Intractivate)

    :return1  face_width(int): it is width of face in the frame which allow us to calculate the distance and find focal length
    :return2 face(list): length of face and (face paramters)
    :return3 face_center_x: face centroid_x coordinate(x)
    :return4 face_center_y: face centroid_y coordinate(y)
 
    '''
    face_width = 0
    face_x, face_y =0,0
    face_center_x =0
    face_center_y =0
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    scaleFactor=1.3
    minNeighbors=5,
    minSize=(30, 30),
    # flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml') #classificador já pré treinado que eciste na biblioteca do open cv que identifica rostos
    faces = face_cascade.detectMultiScale(gray_image,  scaleFactor=1.301, minNeighbors=5,minSize=(10, 10)) # better detection at scaling factor 1.21/ conumse more cpu.

    for (x, y, h, w) in faces:
        # cv2.rectangle(image, (x, y), (x+w, y+h), BLACK, 1)
        face_width = w
        face_center=[]
        # Drwaing circle at the center of the face 
        face_center_x =int(w/2)+x
        face_center_y =int(h/2)+y
        if Distance_level <10:
            Distance_level=10
        
        # cv2.circle(image, (face_center_x, face_center_y),5, (255,0,255), 3 )
        if CallOut==True:
            LLV = int(h*0.12) 
             # print(LLV)
            line_thickness =2

            # cv2.rectangle(image, (x, y), (x+w, y+h), BLACK, 1)
            cv2.line(image, (x,y+LLV), (x+w, y+LLV), (GREEN),line_thickness)
            cv2.line(image, (x,y+h), (x+w, y+h), (GREEN),line_thickness)
            cv2.line(image, (x,y+LLV), (x, y+LLV+LLV), (GREEN),line_thickness)
            cv2.line(image, (x+w,y+LLV), (x+w, y+LLV+LLV), (GREEN),line_thickness)
            cv2.line(image, (x,y+h), (x, y+h-LLV), (GREEN),line_thickness)
            cv2.line(image, (x+w,y+h), (x+w, y+h-LLV), (GREEN),line_thickness)

            cv2.line(image, (x,y), (face_center_x,face_center_y ), (155,155,155),1)
            cv2.line(image, (x,y-11), (x+210, y-11), (YELLOW), 25)
            cv2.line(image, (x,y-11), (x+Distance_level, y-11), (GREEN), 25)
            
            cv2.circle(image, (face_center_x, face_center_y),2, (255,0,255), 1 )
            cv2.circle(image, (x, y),2, (255,0,255), 1 )
           
        # face_x = x
        # face_y = y

    return face_width, faces, face_center_x, face_center_y


Focal_length_found = 541.5112584611135

def publish_message():

  # o nó está "publicando" os frames dos vídeos usando uma mensagem do tipo imagem
  pub = rospy.Publisher('video_frames', Image, queue_size=10)
     
  # o primeiro argumento indica o nome do nó que está iniciando
  # o segundo argumento vê se o nó tem um único nome (pode ter números aleatórios dps)
  rospy.init_node('dist_pub_py', anonymous=True)
     
  # frequeência do loop
  rate = rospy.Rate(10) # 10hz
     
  # cria um objeto VideoCapture
  # o argumento 0 indica que estamos usando a câmera padrão 
  cap = cv2.VideoCapture(0)
  Distance_level =0
  face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml') #classificador já pré treinado que eciste na biblioteca do open cv que identifica rostos
  eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml') #classificador já pré treinado que eciste na biblioteca do open cv que identifica olhos
  
     
  # usado para converter entre imagens ROS e imagens OpenCV
  br = CvBridge()
 
  # enquanto ROS está rodando
  while not rospy.is_shutdown():
     
      # Capture frame-by-frame Captura frame por frame
      # retorna tanto V ou F (ret) quando o próprio frame do vídeo (frame)
      ret, frame = cap.read()
         
      if ret == True:

        frame_height, frame_width, _ = frame.shape
        
        # para desenhar duas linhas verticais paralelas dps
        RightBound = frame_width-140
        Left_Bound = 140

        face_width_in_frame,Faces ,FC_X, FC_Y= face_data(frame, True, Distance_level)
        # finding the distance by calling function Distance finder
        for (face_x, face_y, face_w, face_h) in Faces:
          if face_width_in_frame !=0:
            Distance = Distance_finder(Focal_length_found, Known_width,face_width_in_frame)
            Distance = round(Distance,2)
            # Drwaing Text on the screen
            Distance_level= int(Distance)
            cv2.line(frame, (50,33), (130, 33), (BLACK), 15)
            cv2.putText(frame, f"", (50,35), fonts,0.4, (YELLOW),1)
            cv2.putText(frame, f"Distancia {Distance} cm", (face_x-6,face_y-6), fonts,0.6, (BLACK),2)
    
      cv2.line(frame, (Left_Bound, 80), (Left_Bound, 480-80), (YELLOW), 2)
      cv2.line(frame, (RightBound, 80), (RightBound, 480-80), (YELLOW), 2)
      #cv2.line(frame,  (frame_width/2 -10, frame_height/2 +10),(frame_width/2 +10, frame_height/2 -10), fonts, 0.6, (BLACK),1)
      # output de debbug
      rospy.loginfo('publishing video frame')
        
        # Aqui ocorre a publicação da imagem propriamente dita
        # Assim como a conversão do frame opencv para ros (para ser publicado)
      pub.publish(br.cv2_to_imgmsg(frame))
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
