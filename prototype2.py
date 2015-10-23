import numpy as np
import cv2
from distance import distance

def nothing(x):
    pass

def main2():

    cv2.namedWindow('sliders')

    cv2.createTrackbar('Horizontal Threshold','sliders',5,100,nothing)
    cv2.createTrackbar('Vertical Threshold','sliders',105,200,nothing)

    cv2.createTrackbar('a','sliders',80,100,nothing)
    cv2.createTrackbar('b','sliders',80,500,nothing)

    cv2.createTrackbar('Canny Threshold 1','sliders',102,300,nothing)
    cv2.createTrackbar('Canny Threshold 2','sliders',60,300,nothing)

    cv2.createTrackbar('Rho','sliders',46,360,nothing)
    cv2.createTrackbar('Theta','sliders',160,360,nothing)

    cv2.createTrackbar('Minimum Line Length','sliders',1,300,nothing)
    cv2.createTrackbar('Maximum Line Gap','sliders',50,300,nothing)

    cv2.createTrackbar('Note Threshold','sliders',50,100,nothing)
    cv2.createTrackbar('Note Scale','sliders',100,400,nothing)

    #cap = cv2.VideoCapture(0)

    templateIn = cv2.imread('note.jpg',0)

    while(True):
        # Capture frame-by-frame
        frame = cv2.imread('C-Major.jpg')
        #ret, frame = cap.read()

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        a = 1+(2*cv2.getTrackbarPos('a','sliders'))
        b = -cv2.getTrackbarPos('b','sliders')

        bw = cv2.adaptiveThreshold(~gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, a, b)
    
        vertical = bw.copy()
        horizontal = bw.copy()

        horizThresh = cv2.getTrackbarPos('Horizontal Threshold','sliders')
        vertThresh = cv2.getTrackbarPos('Vertical Threshold','sliders')

        canny1 = cv2.getTrackbarPos('Canny Threshold 1','sliders')
        canny2 = cv2.getTrackbarPos('Canny Threshold 2','sliders')

        horizontalsize = horizontal.shape[1] / horizThresh
        horizontalStructure = cv2.getStructuringElement(cv2.MORPH_RECT, (horizontalsize,1))
        horizontal = cv2.erode(horizontal, horizontalStructure)
        horizontal = cv2.dilate(horizontal, horizontalStructure)

        verticalsize = vertical.shape[0] / vertThresh;
        verticalStructure = cv2.getStructuringElement(cv2.MORPH_RECT, (4,verticalsize));
        vertical = cv2.erode(vertical, verticalStructure)
        vertical =  cv2.dilate(vertical, verticalStructure)
        vertical = cv2.bitwise_not(vertical)

        vertical_edges = cv2.Canny(vertical,canny1,canny2,apertureSize = 3)

        noteScale = cv2.getTrackbarPos('Note Scale','sliders')/100.0
        template = cv2.resize(templateIn, (0,0), fx=noteScale, fy=noteScale) 
        w, h = template.shape[::-1]

        template_edges = cv2.Canny(template,canny1,canny2,apertureSize = 3)
        res = cv2.matchTemplate(vertical_edges,template_edges,cv2.TM_CCOEFF_NORMED)
        threshold = cv2.getTrackbarPos('Note Threshold','sliders')/100.0
        loc = np.where( res >= threshold)

        locReduced = []

        if loc is not None:
            for pt in zip(*loc[::-1]):
                x1,y1 = pt
                contains = False
                if locReduced is not None:
                    for locR in locReduced:
                        cx1,cy1 = locR
                        if distance((x1,y1),(cx1,cy1)) < 5.0:
                            contains = True
                            break
                if not contains:
                    locReduced.append((x1,y1))
                    
        if locReduced is not None:
            locReduced = sorted(locReduced,key=lambda l:l[0])
            for loc in locReduced:
                x1,y1 = loc
                cv2.circle(frame, (int(loc[0]+w/2.0),int(loc[1]+h/2.0)), 15, (0,0,255))
                


        ## Setup SimpleBlobDetector parameters.
        #params = cv2.SimpleBlobDetector_Params()
 
        ## Change thresholds
        #params.minThreshold = 1;
        #params.maxThreshold = 100;
 
        ## Filter by Area.
        #params.filterByArea = False
        #params.minArea = 300
 
        ## Filter by Circularity
        #params.filterByCircularity = True
        #params.minCircularity = 0.5
 
        ## Filter by Convexity
        #params.filterByConvexity = True
        #params.minConvexity = 0.87
 
        ## Filter by Inertia
        #params.filterByInertia = True
        #params.minInertiaRatio = 0.01
 
        ## Create a detector with the parameters
        #ver = (cv2.__version__).split('.')
        #if int(ver[0]) < 3 :
        #    detector = cv2.SimpleBlobDetector(params)
        #else : 
        #    detector = cv2.SimpleBlobDetector_create(params)

        ## Detect blobs.
        #keypoints = detector.detect(vertical)
 
        ## Draw detected blobs as red circles.
        ## cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        #frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


        rho = np.pi/(cv2.getTrackbarPos('Rho','sliders')+0.000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001)
        theta = cv2.getTrackbarPos('Theta','sliders')

        minLength = cv2.getTrackbarPos('Minimum Line Length','sliders')
        maxGap = cv2.getTrackbarPos('Maximum Line Gap','sliders')

        linesReduced = []

        lines = cv2.HoughLinesP(horizontal,1,rho,theta,minLineLength=minLength,maxLineGap=maxGap)
        if lines is not None:
            for line in lines:
                x1,y1,x2,y2 = line[0]
                contains = False
                if linesReduced is not None:
                    for lineR in linesReduced:
                        cx1,cy1,cx2,cy2 = lineR
                        if distance((x1,y1),(cx1,cy1)) < 5.0:
                            contains = True
                            break
                if not contains:
                    linesReduced.append((x1,y1,x2,y2))
                  
        barYs = []
          
        if linesReduced is not None:
            linesReduced = sorted(linesReduced,key=lambda l:l[1])
            for line in linesReduced:
                x1,y1,x2,y2 = line
                cv2.line(frame, (x1,y1),(x2,y2),(0,255,0),2)
                barYs.append(y1)

        
        avgDist = 0

        if barYs is not None:
            for i in range(len(barYs)-1):
                avgDist += barYs[i+1]-barYs[i]
            avgDist /= len(barYs)-1
            print avgDist

        

        ## Display the resulting frame
        cv2.imshow('frame',frame)
        cv2.imshow('frame2',bw)
        cv2.imshow('frame3',horizontal)
        cv2.imshow('frame4',vertical)
        cv2.imshow('frame5',vertical_edges)

        if cv2.waitKey(1) & 0xFF == ord('p'):
            while(True):
                if cv2.waitKey(1) & 0xFF == ord('o'):
                    break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    #cap.release()
    cv2.destroyAllWindows()

main2()

#Capital middle C, up lowercase, down comma, up '