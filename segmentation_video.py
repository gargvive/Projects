import numpy as np 
import cv2

cap=cv2.VideoCapture('sample2.avi') #argument 0 represent the camera number 0 of the computer

#fourcc=cv2.VideoWriter_fourcc('X','V','I','D')
fourcc = cv2.cv.CV_FOURCC(*'XVID')
#fgbg=cv2.createBackgroundSubtractorMOG()
fgbg = cv2.BackgroundSubtractorMOG()
out=cv2.VideoWriter('Segmented_Recorded_video.avi',fourcc,20.0,(640,480))


while cap.isOpened():
	ret, frame=cap.read() #capture frame by frame
	if ret==True:
		fgmask=fgbg.apply(frame)
		out.write(fgmask)
		#gray= cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		cv2.imshow('Segmentation Video',fgmask)
		if cv2.waitKey(30) & 0xff==ord('q'):
			break
	else:
		break
	

cap.release()
out.release()
cv2.destroyAllWindows()