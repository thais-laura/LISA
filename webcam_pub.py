#!/usr/bin/python3

# para saber qual comando vc deve completar depois de #! ali em cima
# tem que colocar no terminal which python3
# copiar e adicionar logo em seguida

import rospy # biblioteca de python para ros
from sensor_msgs.msg import Image # o tipo da mensagem a ser enviada é do tipo imagem
from cv_bridge import CvBridge # pacote para converter ros para imagem open cv
import cv2 # biblioteca do open cv
 

def publish_message():

  # o nó está "publicando" os frames dos vídeos usando uma mensagem do tipo imagem
  pub = rospy.Publisher('video_frames', Image, queue_size=10)
     
  # o primeiro argumento indica o nome do nó que está iniciando
  # o segundo argumento vê se o nó tem um único nome (pode ter números aleatórios dps)
  rospy.init_node('video_pub_py', anonymous=True)
     
  # frequeência do loop
  rate = rospy.Rate(10) # 10hz
     
  # cria um objeto VideoCapture
  # o argumento 0 indica que estamos usando a câmera padrão 
  cap = cv2.VideoCapture(0)
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

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #convertendo a imagem para escala de cinza
        faces = face_cascade.detectMultiScale(gray, 1.3, 5) #retorna a localização das faces que aparecem na imagem

        for (x, y, w, h) in faces:
          #cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 5) #para desenhar um retangulo azul em volta do rosto encontrado

          #obterá a localizao do rosto identificado com a imagem cinza e colorida
          roi_gray = gray[y:y+w, x:x+w] 
          roi_color = frame[y:y+h, x:x+w]

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
