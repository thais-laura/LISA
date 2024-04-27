import numpy as np
import cv2

cap = cv2.VideoCapture(0) #captura o video da webcam
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml') #classificador já pré treinado que eciste na biblioteca do open cv que identifica rostos
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml') #classificador já pré treinado que eciste na biblioteca do open cv que identifica olhos

# distance from camera to object(face) measured
# centimeter
Known_distance = 76.2
 
# width of face in the real world or Object Plane
# centimeter
Known_width = 14.3
 
# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
 
# defining the fonts
fonts = cv2.FONT_HERSHEY_COMPLEX
 
# face detector object
face_detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):
 
    # finding the focal length
    focal_length = (width_in_rf_image * measured_distance) / real_width
    return focal_length
 
# distance estimation function
def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):
 
    distance = (real_face_width * Focal_Length)/face_width_in_frame
 
    # return the distance
    return distance
 
 
def face_data(image):
 
    face_width = 0  # making face width to zero
 
    # converting color image ot gray scale image
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
 
    # detecting face in the image
    faces = face_detector.detectMultiScale(gray_image, 1.3, 5)
 
    # looping through the faces detect in the image
    # getting coordinates x, y , width and height
    for (x, y, h, w) in faces:
 
        # draw the rectangle on the face
        cv2.rectangle(image, (x, y), (x+w, y+h), GREEN, 2)
 
        # getting face width in the pixels
        face_width = w
 
    # return the face width in pixel
    return face_width
 
 
# reading reference_image from directory
ref_image = cv2.imread("Ref_image.png")
 
# find the face width(pixels) in the reference_image
ref_image_face_width = face_data(ref_image)
 
# get the focal by calling "Focal_Length_Finder"
# face width in reference(pixels),
# Known_distance(centimeters),
# known_width(centimeters)
Focal_length_found = Focal_Length_Finder(
    Known_distance, Known_width, ref_image_face_width)
 
print(Focal_length_found)


while True:
    ret, frame = cap.read() #forma que nossa imagem vai ser apresentada(frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #convertendo a imagem para escala de cinza
    faces = face_cascade.detectMultiScale(gray, 1.3, 5) #retorna a localização das faces que aparecem na imagem

    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 5) #para desenhar um retangulo azul em volta do rosto encontrado

        f = FocalLength(w)
        Distance_finder(f, w)

        #obterá a localizao do rosto identificado com a imagem cinza e colorida
        roi_gray = gray[y:y+w, x:x+w] 
        roi_color = frame[y:y+h, x:x+w]

        eyes = eye_cascade.detectMultiScale(roi_gray, 1.3, 5) #detecta os olhos que aparecem na imagem
        for (ex, ey, ew, eh) in eyes:
            cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 5) #desenhar um retangulo verde em volta dos olhos

    cv2.imshow('frame', frame) #apresenta a imagem na tela

    if cv2.waitKey(1) == ord('q'): #se clicar em 'q' fecha a janela
        break

cap.release() #libera o recurso da camera
cv2.destroyAllWindows() #desativando a webcam

