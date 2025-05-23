import cv2


cap = cv2.VideoCapture(0)

n = 0
while True:

  _, frame = cap.read()
  cv2.imshow("1",frame)
  q = cv2.waitKey(1)
  if q==113:
    break
  if q==99:
     n+=1
     print(f"save {n}")
     cv2.imwrite(f"pp/p2_{n}.png",frame)
