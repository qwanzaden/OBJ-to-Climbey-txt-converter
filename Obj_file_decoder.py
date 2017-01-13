"""
Written by Christopher Pratt
2017-1-12

Turns all .obj files, in this file's folder and all its subdirectories, into .txt files which can be loaded as custom levels in the virtual reality game climbey.
Takes no arguments and returns no arguments, just run the python code.
"""
import sys
import os
import numpy as np
import time


def get_quaternion(lst1,lst2,matchlist=None):
	"""
	Function copied from Paul J. Besl and Neil D. McKay "Method for registration of 3-D shapes", Sensor Fusion IV: Control Paradigms and Data Structures, 586 (April 30, 1992).
	Takes 2 lists of (3 perpendicular (3x1 numpy.array) a.k.a. two sets of perpendicular axes) and finds the quaternion that best rotates from the axis in lst1 to the axis in lst 2
	returns the quaternion as a list in [w,x,y,z] form
	"""
	if not matchlist:
		matchlist=range(len(lst1))
	M=np.matrix([[0,0,0],[0,0,0],[0,0,0]])

	for i,coord1 in enumerate(lst1):
		x=np.matrix(np.outer(coord1,lst2[matchlist[i]]))
		M=M+x

	N11=float(M[0][:,0]+M[1][:,1]+M[2][:,2])
	N22=float(M[0][:,0]-M[1][:,1]-M[2][:,2])
	N33=float(-M[0][:,0]+M[1][:,1]-M[2][:,2])
	N44=float(-M[0][:,0]-M[1][:,1]+M[2][:,2])
	N12=float(M[1][:,2]-M[2][:,1])
	N13=float(M[2][:,0]-M[0][:,2])
	N14=float(M[0][:,1]-M[1][:,0])
	N21=float(N12)
	N23=float(M[0][:,1]+M[1][:,0])
	N24=float(M[2][:,0]+M[0][:,2])
	N31=float(N13)
	N32=float(N23)
	N34=float(M[1][:,2]+M[2][:,1])
	N41=float(N14)
	N42=float(N24)
	N43=float(N34)

	N=np.matrix([[N11,N12,N13,N14],\
		[N21,N22,N23,N24],\
		[N31,N32,N33,N34],\
		[N41,N42,N43,N44]])


	values,vectors=np.linalg.eig(N)
	w=list(values)
	mw=max(w)
	quat= vectors[:,w.index(mw)]
	quat=np.array(quat).reshape(-1,).tolist()
	return quat

#All loaded objects will be stored as a retangle class
class rectangle:
	"""
	Loads 8 points and a .obj material name and converts them into a cube's center, size, and rotation
	"""
	
	def __init__(self, inputName, pointsList):
		"""
		Inialization takes the string from the .obj's materal/usemtl
		"""

		#Initialize variables
		self.pointList    = np.array(pointsList).astype(float)
		self.size         = np.array([])
		self.center       = np.array([])
		self.rotation     = np.array([]) #Stored as [w,x,y,z]

		#Take the raw .obj material string (like Metal.001) and remove everything after and including the first period
		self.materialName = inputName
		self.typeName     = ''
		if self.materialName.find('.') != -1:
			self.typeName = self.materialName[0:self.materialName.find('.')]
		else:
			self.typeName = inputName

		#Find the rectangles positional parameters
		self.updatePositionInfo()
	
	def updatePositionInfo(self):
		"""
		Finds the rectangle's size, center, and rotation base on its 8 points.
		"""
		
		#Rectangle center is the average of its 8 points
		self.center = np.sum(self.pointList,axis = 0)/8.0

		#Find the axis vectors of the rectangle, starting with a arbitrary "origin" point
		originCornerPosition = self.pointList[0]
		
		#Find the first point closest to the origin corner, label it as the "x" point
		vectorList        = self.pointList[1:,:] - np.repeat([originCornerPosition],7,axis = 0)
		pointDistanceList = np.linalg.norm(vectorList, axis = 1)
		originXIndex      = np.where(pointDistanceList == min(pointDistanceList))[0][0]
		originXVector     = vectorList[originXIndex]
		originXUnitVector = originXVector/np.linalg.norm(originXVector)
		originXLength     = np.linalg.norm(originXVector)
		
		#Find the other 3 vectors which are perpendicular to the x vector
		unitVectorList         = np.divide(vectorList,np.linalg.norm(vectorList, axis = 1)[:, np.newaxis])
		crossProductList       = np.cross(np.repeat([originXUnitVector],7,axis = 0),unitVectorList)
		perpendicularIndexList = np.where(np.linalg.norm(crossProductList, axis = 1)>.999999)[0]#.999999 is for roundoff errors
		if len(perpendicularIndexList) != 3:
			raise StandardError('Found ' + str(len(perpendicularIndexList)) + ' instead of 3 perpendicular vectors, probably due to roundoff error in python or the .obj file.')
		
		#The longest vector of those three vectors is not the y or z vector
		possibleVectorList    = vectorList[perpendicularIndexList]
		possibleVectorLength  = np.linalg.norm(possibleVectorList, axis = 1)
		possibleYZVectorIndex = np.where(possibleVectorLength != max(possibleVectorLength))[0]
		possibleYZVectorIndex = perpendicularIndexList[possibleYZVectorIndex]
		
		#Determine which of these vectors is the y and z vector
		if np.dot(np.cross(originXUnitVector,unitVectorList[possibleYZVectorIndex[0]]),unitVectorList[possibleYZVectorIndex[1]])>0:
			originYIndex      = possibleYZVectorIndex[0]
			originZIndex      = possibleYZVectorIndex[1]
		else:
			originYIndex      = possibleYZVectorIndex[1]
			originZIndex      = possibleYZVectorIndex[0]

		originYVector     = vectorList[originYIndex]
		originYUnitVector = unitVectorList[originYIndex]
		originYLength     = np.linalg.norm(originYVector)
		originZVector     = vectorList[originZIndex]
		originZUnitVector = unitVectorList[originZIndex]
		originZLength     = np.linalg.norm(originZVector)

		self.rotation = get_quaternion([np.array([1,0,0]),np.array([0,1,0]),np.array([0,0,1])],[originXUnitVector, originYUnitVector, originZUnitVector])

		self.size = np.array([originXLength,originYLength,originZLength])

	def printClimbyInfo(self):
		self.returnString = ''
		if self.typeName == 'Icy' or self.typeName == 'Metal' or self.typeName == 'Glass' or self.typeName == 'Grabbable' or self.typeName == 'Spikes' or self.typeName == 'Jumpy' or self.typeName == 'GravityField':
			self.returnString += '{"Type":"' + self.typeName + '",'
			self.returnString += '"Size":{"x":' + str(self.size[0]) + ',"y":' + str(self.size[1]) + ',"z":' + str(self.size[2]) + '},'
			self.returnString += '"Position":{"x":' + str(self.center[0]) + ',"y":' + str(self.center[1]) + ',"z":' + str(self.center[2]) + '},'
			self.returnString += '"Rotation":{"x":' + str(self.rotation[1]) + ',"y":' + str(self.rotation[2]) + ',"z":' + str(self.rotation[3]) + ',"w":' + str(self.rotation[0]) + '},'
			self.returnString += '"LockX":false,"LockY":false,"LockZ":false'
			self.returnString += '}'
		elif self.typeName == '[CameraRig]':
			self.returnString += '{"Type":"' + self.typeName + '",'
			self.returnString += '"Size":{"x":1.0,"y":1.0,"z":1.0},'
			self.returnString += '"Position":{"x":' + str(self.center[0]) + ',"y":' + str(self.center[1]) + ',"z":' + str(self.center[2]) + '},'
			self.returnString += '"Rotation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0},'
			self.returnString += '"LockX":false,"LockY":false,"LockZ":false'
			self.returnString += '}'
		elif self.typeName == 'Finishline':
			self.returnString += '{"Type":"Finishline","Size":{"x":0.5,"y":1.0,"z":0.5},'
			self.returnString += '"Position":{"x":' + str(self.center[0]) + ',"y":' + str(self.center[1]) + ',"z":' + str(self.center[2]) + '},'
			self.returnString += '"Rotation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0},'
			self.returnString += '"LockX":false,"LockY":false,"LockZ":false'
			self.returnString += '}'
		return self.returnString

#main code
thisFilesPath = os.path.dirname(os.path.realpath(__file__))

print('Processing all files in "' + thisFilesPath + '"')

for subdir, dirs, files in os.walk(thisFilesPath):
	for OBJFileName in files:
		
		#Skip file if it is not a .obj files
		if OBJFileName[-4:] != '.obj':
			continue

		"""
		#Option to skip files which are already up to date with their .obj file.
		if os.path.isfile(subdir + '\\' + climbeyFileName):
			if (os.path.getmtime(subdir + '\\' + climbeyFileName) >= os.path.getmtime(subdir + '\\' + OBJFileName)):
				print(climbeyFileName + ' in "' + subdir + '" is up to date')
				break
		"""

		print('processing: ' + OBJFileName)

		#open files for reading and writing
		climbeyFileName = OBJFileName[:-4] + '.txt'
		rectangleList = []
		with open(subdir + '\\' + OBJFileName, 'r') as OBJFile:
			with open(subdir + '\\' + climbeyFileName, 'w') as climbeyFile:
				#Loop through the .obj file
				OBJFileLines = list(OBJFile)
				curLine = 0
				while (curLine < len(OBJFileLines)-1):
					#Loop through the file untill the object specifier 'o ' is found
					if OBJFileLines[curLine][0:2] == 'o ':
						objectName   = OBJFileLines[curLine][2:-1]
						materialName = ''
						pointList = []
						curLine += 1

						#Continue reading the file untill another object specifier 'o ' is found
						while (curLine < len(OBJFileLines)-1):
							if OBJFileLines[curLine][0:2] == 'o ':
								#A new object has been encountered, stop processing the current one
								break
							elif OBJFileLines[curLine][0:2] == 'v ':
								#Add vertex to the list
								lineContents = str.split(OBJFileLines[curLine])
								pointList.append([float(lineContents[1]),float(lineContents[3]),float(lineContents[2])])
							elif OBJFileLines[curLine][0:6] == 'usemtl':
								#Find the .obj material/usemtl name
								materialName = OBJFileLines[curLine][7:-1]
							curLine += 1
						
						#Add the current rectangle to the list
						if len(pointList) is 8:
							rectangleList.append(rectangle(materialName,pointList))

						"""
						#debugging
						print('ObjectName: ' + objectName)
						print('materialName: ' + materialName)
						print('len(pointList): ' + str(len(pointList)))
						print('')
						"""

					else:
						curLine += 1

				climbeyFile.write('{')
				climbeyFile.write('"LevelArray":[')

				curRectangle = 0
				if len(rectangleList) > 0:
					climbeyFile.write(rectangleList[0].printClimbyInfo())
				for curRectangle in rectangleList[1:]:
					climbeyFile.write(',')
					climbeyFile.write(curRectangle.printClimbyInfo())

				climbeyFile.write(']')
				climbeyFile.write(',"MovingArray":[],"LevelSettings":{"Type":"LevelSign","Size":{"x":1.0,"y":0.949999988079071,"z":1.0},"Position":{"x":0.05000000074505806,"y":0.6000000238418579,"z":-0.6000000238418579},"Rotation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0},"LockX":false,"LockY":false,"LockZ":false,"Checkpoints":4,"Gamemode":0},"SignsArray":[]')
				climbeyFile.write('}')

print('Finished, closing in 5 seconds.')
time.sleep(5)