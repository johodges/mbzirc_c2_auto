import cv2
import cv2.cv as cv
import numpy as np

img = cv2.imread('wrench.png',0)
sz = np.shape(img)
print sz[0]/2
print sz[1]/2

img = cv2.medianBlur(img,5)
cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

font = cv2.FONT_HERSHEY_SIMPLEX

circles = cv2.HoughCircles(img, cv2.cv.CV_HOUGH_GRADIENT,1,20,param1=50, param2=30, minRadius=0, maxRadius=100)
#print circles

# printout number of detected circles
#nc = np.size(circles)/3
#print nc
sp = np.shape(circles)
nc = sp[1]
print sp
print nc

data = np.uint16(np.around(circles)).reshape(-1,3)
print data
#print data[0,0]
print data[0,:]

dists = np.zeros(nc)
mindist = 2000
idx = 0
for i in range(nc):
    dists[i] = np.sqrt((data[i,0]-sz[1]/2)**2 + (data[i,1]-sz[0]/2)**2)
    if (dists[i] < mindist):
	mindist = dists[i]
	idx = i
print dists
print mindist
print idx

best = data[idx]
print best
#print idxest
#print mindist

#mindist = dists[idx]
#print mindist


#sorted_dists_idx = np.argsort(dists)
#print sorted_dists_idx

#sorted_point = data[sorted_dists_idx]
#best = sorted_point[0]
#print best

for i in circles[0,:]:
    cv2.circle(cimg, (i[0],i[1]), i[2], (0,255,0), 2)
    cv2.circle(cimg, (i[0],i[1]), 2, (0,0,255), 3)
    cv2.circle(cimg, (sz[1]/2,sz[0]/2), 2, (0,0,255), 3)
cv2.putText(cimg, 'Image size is: ' + str(sz[0]) + "x" + str(sz[1]),  (10,30), font, 1, (255,0,0)) 
cv2.putText(cimg, 'The closest circle is: ' + str(best[0]) + "x" + str(best[1]),  (10,60), font, 1, (255,0,0)) 
cv2.putText(cimg, 'Number of detected circles: ' + str(nc),  (10,90), font, 1, (255,0,0)) 
cv2.circle(cimg, (best[0],best[1]), best[2], (0,0,255), 2)


cv2.imshow('detected circle', cimg)
cv2.waitKey(0)
cv2.destroyAllWindows(0)


