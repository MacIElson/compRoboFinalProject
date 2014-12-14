from pickle import load
import sklearn
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.cross_validation import StratifiedKFold
from os import listdir
from os.path import isdir, join
import cv2
from scipy.stats import mode

f = open('SIFT_features.pickle','r')
descriptors = load(f)
f.close()

X = np.zeros((0,128))
y = np.zeros((0,))

categories = descriptors.keys()
for i in range(len(categories)):
	for data in descriptors[categories[i]]:
		X = np.vstack((X,np.mean(data,0)))
		y = np.hstack((y,i*np.ones((1,))))

skf = StratifiedKFold(y,5)
category_accuracies = np.zeros((len(categories),5))

fold = 0
for train, test in skf:
	Xtest = X[test,:]
	ytest = y[test]
	# try out different classifiers and parameter values if you'd like
	model = LogisticRegression(C=1)
	model.fit(X[train,:],y[train])
	print model.score(X[test,:],y[test])
	for i in range(len(categories)):
		category_accuracies[i,fold] = model.score(Xtest[ytest==i],ytest[ytest==i])
	fold += 1

# model = LogisticRegression(C=1)
# model.fit()
# model.fit(x, y)

# only worry about object categories that have at least 50 images
# images = images[0:50]
# print category + " " + str(len(images))
# cache[category] = []
descriptor_name = 'SIFT'

detector = cv2.FeatureDetector_create(descriptor_name)
extractor = cv2.DescriptorExtractor_create(descriptor_name)

for i in range(1,6):
	# file_name = join(join('./101_ObjectCategories/test',category),image)
	im = cv2.imread("test/" + "stop" + str(i) + ".jpg")
	# print im
	im_bw = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
	kp = detector.detect(im_bw)
	dc, des = extractor.compute(im_bw,kp)
	# stash the descriptors in a dictionary for later processing
	guess = model.predict(des)
	print guess
	print mode(guess)


# print out accurcies by category
# print zip(categories,np.mean(category_accuracies,axis=1))
