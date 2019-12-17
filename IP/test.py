import cv2
import numpy as np
import keras
model = keras.models.load_model("interiit_model_3.model")

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    print('running')
   # cv2.imshow('frame', frame)
    img=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

    img=cv2.resize(img,(256,256))
    img=(img.astype(np.float32)/255)
    img=np.expand_dims(img, axis=0)
    predictions=model.predict(img,verbose=0)

    y_pred_binary = (predictions > 0.5).astype(np.int)
    if(y_pred_binary==1): print("?")
    else :print("lol")
   
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
