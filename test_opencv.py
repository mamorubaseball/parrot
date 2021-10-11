import cv2

img = cv2.imread('phote/img1.JPG')
img = cv2.resize(img, (500, 200))
# この一文、なくてもよい
cv2.imshow('hosei',img)
cv2.waitKey(0) #ここで初めてウィンドウが表示される
cv2.destroyAllWindows()

