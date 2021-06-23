import cv2

def contains(rect1, rect2):
    x1, y1, w1, h1 = rect1
    x2, y2, w2, h2 = rect2
    return x1 <= x2 and y1 <= y2 and x1+w1 >= x2+w2 and y1+h1 >= y2+h2 

# to detect the corners
def process_orb(img):
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Initiate ORB detector
    orb = cv2.ORB_create()
    # find the keypoints with ORB
    kp = orb.detect(imgray, None)

    # compute the descriptors with ORB
    kp, des = orb.compute(imgray, kp)
    print("keypoints", kp)
    # draw only keypoints location,not size and orientation
    img2 = cv2.drawKeypoints(img, kp, None, color=(0,255,0), flags=0)
    cv2.imshow('Contours_orb', img2)

def process_canny(img): 
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(imgray, 100, 200)
    ret, thresh = cv2.threshold(edges, 127, 255, 0)
    img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    print("found contours: ", len(contours))

    # find bounding_rects
    bounding_rects = []
    for c in contours:
        # get the bounding rect
        rect = cv2.boundingRect(c)
        bounding_rects.append(rect)

    # take ignore those rects that are completely contained by other bigger rects
    valid_rects = []
    for i, rect_i in enumerate(bounding_rects):
        is_contained = False
        for j, rect_j in enumerate(bounding_rects):
            if j == i: continue
            if contains(rect_j, rect_i):
                is_contained = True
                break

        if not is_contained: 
            valid_rects.append(rect_i)

    print("number of valid_rects: ", len(valid_rects))

    for rect in valid_rects:
        x, y, w, h = rect
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

    cv2.drawContours(img, contours, -1, (0, 0, 255), 3)
    cv2.imshow('Contours', img)


cv2.namedWindow('Contours_canny', cv2.WINDOW_NORMAL)
img = cv2.imread('images/test_image_1.jpeg')
process_orb(img)
process_canny(img)

if cv2.waitKey(0):
    cv2.destroyAllWindows()

