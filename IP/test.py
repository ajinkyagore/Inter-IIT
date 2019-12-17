import cv2
import numpy as np
from keras.models import Model
from keras.models import Sequential
import numpy as np
from keras_preprocessing.image import ImageDataGenerator
from keras.optimizers import Adam, RMSprop
from keras.layers import Convolution1D, concatenate, SpatialDropout1D, GlobalMaxPool1D, GlobalAvgPool1D, Embedding, \
    Conv2D, SeparableConv1D, Add, BatchNormalization, Activation, GlobalAveragePooling2D, LeakyReLU, Flatten
from keras.layers import Dense, Input, Dropout, MaxPooling2D, Concatenate, GlobalMaxPooling2D, GlobalAveragePooling2D, \
    Lambda, Multiply, LSTM, Bidirectional, PReLU, MaxPooling1D
from keras.layers.pooling import _GlobalPooling1D
from keras.losses import binary_crossentropy
from keras.models import Model
from keras.models import Sequential
import keras
from keras.preprocessing import image
model = keras.models.load_model("interiit_model_py3.model")

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
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