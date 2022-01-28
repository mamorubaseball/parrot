# import cv2
#
# # 入力画像の読み込み（テスト用画像ファイル）
# img = cv2.imread("IMG_8117 (1).jpg")
#
# # カスケード型識別器（自作した分類器）
# cascade = cv2.CascadeClassifier("himo2.xml")
#
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#
# # face→ballに変更（そのままでもいいですけど）
# ball = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=2, minSize=(30, 30))
#
# # 顔領域を赤色の矩形で囲む
# for (x, y, w, h) in ball:
#     cv2.rectangle(img, (x, y), (x + w, y+h), (0,0,200), 3)
#
# # 結果画像を保存
# cv2.imwrite("result_tennisball.jpg",img)
#
# #結果画像を表示
# cv2.imshow('image', img)
# cv2.waitKey(0)

for i in range(240):
    text ='./makecascade/neg/IMG_80830.jpg'


