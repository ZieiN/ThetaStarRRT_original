import numpy as np
from PIL import Image
from numpy import eye
import matplotlib.pyplot as plt
import sys
import pylab as pl
import math
width = 600;
height = 600;

def draw_line(x,y,angle,length):
  terminus_x = x + length * math.cos(angle)
  terminus_y = y + length * math.sin(angle)
  angle += 3.14159265/2.0;
  while angle>=2*3.14159265:
  	angle -= 2*3.14159265;
  terminus_x1 = x + length/4 * math.cos(angle)
  terminus_y1 = y + length/4 * math.sin(angle)
  angle += 3.14159265;
  while angle>=2*3.14159265:
  	angle -= 2*3.14159265;
  terminus_x2 = x + length/4 * math.cos(angle)
  terminus_y2 = y + length/4 * math.sin(angle)

  pl.plot([terminus_x1, terminus_x],[terminus_y1 ,terminus_y], 'r')#, markersize=0.001)
  pl.plot([terminus_x2, terminus_x],[terminus_y2,terminus_y], 'r')#, markersize=0.001)
  pl.plot([terminus_x1, terminus_x2],[terminus_y1,terminus_y2], 'r')#, markersize=0.001)
  # print [x, terminus_x],[y,terminus_y]

pl.axis('equal')
pl.axis([0, height, 0, width])
imgx = []
imgy = []
with open('../../Input/map_0.txt') as f:
	cnt = 0
	for line in f:
		if cnt == 0:
			cnt += 1
			continue
		arr = [int(i) for i in line.split()]
		for i in range(len(arr)):
			if arr[i] == 1:
				imgx.append(600-cnt-1);
				imgy.append(i);
				# pl.plot(cnt-1, i, 'k.')
				# pl.draw()
				# pl.pause(0.0001)
		cnt += 1
f.close()
pl.plot(imgy, imgx, 'k.')

a=[]
b=[]
# with open('out.txt') as f:
# 	for line in f:
# 		arr = [(float(i)) for i in line.split()]
# 		cnt += 1
# 		# if cnt % 10 == 0:
# 		a.append(arr[1]);
# 		b.append(599-arr[0]);
# 		# pl.plot(arr[0], arr[1],'r.')
# 		# if cnt%10==0:
# 		# 	draw_line(arr[1], 599-arr[0], arr[2]-3.14159265/2.0, 1);
# 		# 	pl.draw()
# 		# 	pl.pause(0.0001)
# f.close()
# pl.plot(a,b,'y-')

# with open('out1.txt') as f:
# 	cnt = 0
# 	for line in f:
# 		arr = [(float(i)) for i in line.split()]
# 		cnt += 1
# 		# if cnt % 10 == 0:
# 		a.append(arr[1]);
# 		b.append(599-arr[0]);
# 		# pl.plot(arr[0], arr[1],'r.')
# 		if cnt%300==0:
# 			pl.plot(a, b, 'b-.', linewidth=0.5);
# 			a=[]
# 			b=[]
# 		# 	draw_line(arr[1], 599-arr[0], arr[2]-3.14159265/2.0, 1);
# 		# 	pl.draw()
# 		# 	pl.pause(0.0001)
# f.close()
# for i in range(0, len(a), 2):
# 	pl.plot()
# pl.plot(a,b,'y-')

# with open('out2.txt') as f:
# 	cnt = 0
# 	for line in f:
# 		arr = [(float(i)) for i in line.split()]
# 		cnt += 1
# 		# if cnt % 10 == 0:
# 		a.append(arr[1]);
# 		b.append(599-arr[0]);
# 		# pl.plot(arr[0], arr[1],'r.')
# 		if cnt%300==0:
# 			pl.plot(a, b, 'r-.', linewidth=0.5);
# 			a=[]
# 			b=[]
# 		# 	draw_line(arr[1], 599-arr[0], arr[2]-3.14159265/2.0, 1);
# 		# 	pl.draw()
# 		# 	pl.pause(0.0001)
# f.close()
# for i in range(0, len(a), 2):
# 	pl.plot()
# pl.plot(a,b,'y-')
a1=[]
a2=[]
a3=[]
a4=[]
a5=[]
a6=[]
with open('out.txt') as f:
	for line in f:
		a = [(float(i)) for i in line.split()]
		if len(a)==2:
			a1.append(599-a[0]);
			a2.append(a[1]);
			# pl.plot(a[1], 599-a[0], 'g.')#, markersize = 1)
		else:
			a3.append(599-a[0]);
			a4.append(a[1]);
			a5.append(599-a[2]);
			a6.append(a[3]);
			# pl.plot(a[1], 599-a[0], 'b.')#,markersize=0.01)
			# pl.draw()
			# pl.pause(0.001)
			# pl.plot(a[3], 599-a[2], 'r.')#,markersize=0.01)
			pl.plot(a2,a1,'g-')
			a1.clear()
			a2.clear()
			# pl.draw()
			# pl.pause(0.001)
f.close()
# pl.plot(a2,a1,'g-')
pl.plot(a4,a3,'b.')
pl.plot(a6,a5,'r.')

pl.show()
input("hit[enter] to end.")
# pl.savefig(image_num+"-.png")
pl.close('all')
