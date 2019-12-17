import cv2
import numpy as np
lowerBound=np.array([33,80,40])
upperBound=np.array([102,255,255])

cam= cv2.VideoCapture(0)
kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))

while True:
    ret, img=cam.read()
    img=cv2.resize(img,(256,256))
    imgHSV= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(imgHSV,lowerBound,upperBound)
    maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
    maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

    maskFinal=maskClose
#     conts,h=cv2.findContours(maskFinal.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    
#     cv2.drawContours(img,conts,-1,(255,0,0),3)

#     cv2.imshow("maskClose",maskClose)
#     cv2.imshow("maskOpen",maskOpen)
#     cv2.imshow("mask",mask)
    #cv2.imshow("cam",img)
    s=0
 
    
    for labels in maskClose:
        for label in labels:
            if label==255:
                s=s+1
         

    if s > 600: 
        print("lolololol")
    else:
        print("xxxxxxx")
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cam.release()
cv2.destroyAllWindows()
