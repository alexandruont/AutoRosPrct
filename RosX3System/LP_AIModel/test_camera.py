import cv2
print(cv2.__version__)
c = cv2.VideoCapture(0)
ret,frame= c.read()